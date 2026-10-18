"""Bundle adjustment over the map's keyframes (ORB-SLAM's ``LocalBundleAdjustment``, lite).

The free keyframes, the points they observe, and every keyframe observing those points go
into one :class:`slam_it.foundations.ba.BAProblem`:

- each observation weighted by its level's information ``1 / s^(2 l)``;
- an observation with depth (stereo, RGB-D) also gives an inverse-depth prior ``1 / z`` with
  information ``(f b)^2 / s^(2 l)``: the disparity's variance ``s^(2 l)``, as an inverse
  depth (ORB-SLAM's stereo edge measures the right column instead);
- the keyframes that are not free held fixed, as are the ones the caller fixes;
- a Huber loss at ``sqrt(chi2_mono)`` on the weighted errors.

With no keyframe held fixed the problem has no gauge, and nothing is solved: ORB-SLAM3's
``LocalBundleAdjustment`` returns when it counts no fixed keyframe.

It solves for ``adjustment.iterations``, drops the observations failing the chi-square test
(:func:`slam_it.edge.residuals.chi2`) or behind their camera, solves again for
``adjustment.refine_iterations`` (a point left with fewer than two inlier views is left out of that
solve, back where it started: ORB-SLAM's second pass would re-solve it from its one stereo
edge, so as not to keep where its outliers pulled it), forgets the observations still failing it
(:func:`slam_it.edge.map.forget_observations`), and refreshes the surviving points
(ORB-SLAM's ``UpdateNormalAndDepth``). Rung 0's BA leaves out a free point seen from fewer
than two keyframes, so a point from one keyframe's depth stays where the depth put it until a
second keyframe sees it.

The result (:class:`Adjustment`) records how each solve went (:class:`Solve`) and how many
observations were dropped, which a run records per keyframe.

**Stopped early** (milestone 2B's B3b, local mapping in a thread of its own): given an ``abort``
check, the adjustment asks it before its first solve, and returns the map untouched if it says
stop, as ORB-SLAM3's ``LocalBundleAdjustment`` returns when its ``pbStopFlag`` is set; and again
after the first solve, where a stop skips the refining solve, the outliers of the first dropped
and its result kept, as ORB-SLAM2's ``bDoMore``. ORB-SLAM's g2o also stops between the
iterations of a solve; rung 0's solver runs each solve whole (only the Python reference takes the
check, until B3b's compiled ports).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.edge import map as maps
from slam_it.edge import residuals
from slam_it.edge.frames import Camera
from slam_it.edge.map import Map
from slam_it.edge.options import Options
from slam_it.foundations import ba, nls, se3
from slam_it.foundations.arrays import as_indices, as_mask
from slam_it.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


@dataclass(frozen=True)
class Solve:
    """How one least-squares solve of a bundle adjustment went, as a run records it.

    Attributes
    ----------
    initial_cost, final_cost : float
        The robust cost at the start and at the returned state
        (:class:`slam_it.foundations.nls.Summary`).
    iterations : int
        Steps tried, accepted or not.
    termination : slam_it.foundations.nls.Termination
        Why it stopped.
    """

    initial_cost: float
    final_cost: float
    iterations: int
    termination: nls.Termination

    @classmethod
    def of(cls, summary: nls.Summary) -> Solve:
        """Take a solve's record from its summary.

        Parameters
        ----------
        summary : slam_it.foundations.nls.Summary
            The solve's summary.

        Returns
        -------
        Solve
            Its costs, iterations and termination.
        """
        return cls(
            initial_cost=float(summary.initial_cost),
            final_cost=float(summary.final_cost),
            iterations=int(summary.iterations),
            termination=summary.termination,
        )

    def as_dict(self) -> dict[str, Any]:
        """Return the solve as JSON values, the termination by name.

        Returns
        -------
        dict
            ``initial_cost``, ``final_cost``, ``iterations`` and ``termination``.
        """
        return {
            "initial_cost": self.initial_cost,
            "final_cost": self.final_cost,
            "iterations": self.iterations,
            "termination": self.termination.value,
        }


@dataclass(frozen=True)
class Adjustment:
    """A bundle adjustment's result: the map, and how the adjustment went.

    Attributes
    ----------
    map : Map
        The map with adjusted poses and points, and the outlier observations forgotten.
    solves : tuple of Solve
        The first solve and the refining one; empty when nothing was solved: no point was
        observed, or no keyframe was held fixed.
    outliers : int
        The observations forgotten as outliers after the refining solve.
    free : int
        The keyframes adjusted: those given, but the ones held fixed among them.
    fixed : int
        The keyframes held fixed: those given that were, and every other observing their
        points.
    aborted : bool
        Whether an abort stopped it early: before its first solve (none ran) or before the
        refining one (the module docstring).
    """

    map: Map
    solves: tuple[Solve, ...]
    outliers: int
    free: int = 0
    fixed: int = 0
    aborted: bool = False


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.adjust`` submodule provides.

    An adjustment crosses back as a dict of ``map`` (as :meth:`Map.as_dict`), ``solves`` (a
    list of dicts of :class:`Solve`'s fields, the termination as its code in ``codes()``) and
    ``outliers``.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def bundle_adjust(
        self,
        m: dict[str, Any],
        camera: dict[str, Any],
        keyframes: NDArray[np.int64],
        fixed: NDArray[np.bool_],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Bundle-adjust keyframes and their points; return the new map and the solves."""
        ...


def decode_solves(raw: list[dict[str, Any]], codes: backends.Codes) -> tuple[Solve, ...]:
    """Rebuild a compiled backend's solves (a list of dicts, each termination a code).

    Parameters
    ----------
    raw : list of dict
        The solves, as the backend returned them.
    codes : slam_it.backends.Codes
        The module's ``codes()``, holding ``nls_termination``.

    Returns
    -------
    tuple of Solve
        The solves, in order.
    """
    return tuple(
        Solve(
            initial_cost=float(item["initial_cost"]),
            final_cost=float(item["final_cost"]),
            iterations=int(item["iterations"]),
            termination=nls.Termination(
                backends.decode_name(codes, "nls_termination", item["termination"])
            ),
        )
        for item in raw
    )


def _decode_adjustment(raw: dict[str, Any], codes: backends.Codes) -> Adjustment:
    """Rebuild :class:`Adjustment` from a compiled backend's dict."""
    return Adjustment(
        map=maps.decode(raw["map"], codes),
        solves=decode_solves(raw["solves"], codes),
        outliers=int(raw["outliers"]),
        free=int(raw["free"]),
        fixed=int(raw["fixed"]),
    )


def _problem(
    m: Map, camera: Camera, keyframes: IntArray, points: IntArray, fixed: Mask
) -> tuple[ba.BAProblem, IntArray, IntArray]:
    """Build the BA problem; return it, its observations' keypoint rows and its cameras' rows."""
    keyframe_of = m.keypoint_keyframe()
    free = np.zeros(m.keyframes, dtype=bool)
    free[keyframes] = True
    free &= ~fixed
    point_index = np.full(m.points, -1, dtype=np.int64)
    point_index[points] = np.arange(len(points))
    rows = np.flatnonzero(m.kp_point >= 0)
    p_rows = m.point_rows(m.kp_point[rows])
    rows = rows[point_index[p_rows] >= 0]
    p_rows = m.point_rows(m.kp_point[rows])
    observing = np.unique(keyframe_of[rows])
    camera_index = np.full(m.keyframes, -1, dtype=np.int64)
    camera_index[observing] = np.arange(len(observing))
    variance = residuals.sigma2(m, m.kp_level[rows])
    depth = m.kp_depth[rows]
    has_depth = ~np.isnan(depth)
    prior = np.where(has_depth, 1.0 / np.where(has_depth, depth, 1.0), np.nan)
    information = (camera.focal * camera.baseline) ** 2 / variance
    problem = ba.BAProblem.create(
        camera.model,
        camera.intrinsics,
        se3.inverse(m.kf_pose[observing]),
        m.pt_xyz[points],
        m.kp_xy[rows],
        camera_index[keyframe_of[rows]],
        point_index[p_rows],
        weights=1.0 / variance,
        fixed_poses=~free[observing],
        depth_prior=prior,
        depth_prior_weights=np.where(has_depth, information, 0.0),
    )
    return problem, rows, observing


def _bundle_adjust(  # noqa: PLR0913 -- the map, its camera, what to adjust, the settings, the stop
    m: Map,
    camera: Camera,
    keyframes: IntArray,
    fixed: Mask,
    options: Options,
    *,
    abort: Callable[[], bool] | None = None,
) -> Adjustment:
    """Bundle-adjust keyframes and the points they observe (reference)."""
    points = m.observed_by(keyframes)
    if len(points) == 0:
        return Adjustment(map=m, solves=(), outliers=0)
    problem, rows, observing = _problem(m, camera, keyframes, points, fixed)
    if not np.any(problem.fixed_poses):  # no gauge: ORB-SLAM3 skips it
        return Adjustment(map=m, solves=(), outliers=0)
    if abort is not None and abort():  # stopped before it started: ORB-SLAM3 returns
        return Adjustment(map=m, solves=(), outliers=0, aborted=True)
    loss = nls.Loss("huber", math.sqrt(options.adjustment.chi2_mono))
    first = ba.solve(problem, _options(loss, options.adjustment.iterations))
    solved = replace(problem, poses=first.poses, points=first.points)
    outlier = _outliers(m, camera, solved, rows, options)
    if abort is not None and abort():  # ORB-SLAM2's bDoMore: the first solve kept
        return _written_back(
            m, problem, points=points, observing=observing, result=first,
            dropped=rows[outlier], solves=(Solve.of(first.summary.solve),), aborted=True,
        )  # fmt: skip
    # A point left with fewer than two inlier views is left out of the second solve: it goes
    # back to where it was, not where its outliers pulled it.
    alone = np.bincount(problem.pt_idx[~outlier], minlength=len(points)) < 2
    second = ba.solve(
        replace(
            solved,
            points=np.where(alone[:, None], problem.points, first.points),
            weights=np.where(outlier, 0.0, problem.weights),
            depth_prior_weights=np.where(outlier, 0.0, problem.depth_prior_weights),
        ),
        _options(loss, options.adjustment.refine_iterations),
    )
    final = replace(problem, poses=second.poses, points=second.points)
    dropped = rows[_outliers(m, camera, final, rows, options)]
    return _written_back(
        m, problem, points=points, observing=observing, result=second, dropped=dropped,
        solves=(Solve.of(first.summary.solve), Solve.of(second.summary.solve)), aborted=False,
    )  # fmt: skip


def _written_back(  # noqa: PLR0913 -- the map, the problem, its result, what it dropped
    m: Map,
    problem: ba.BAProblem,
    *,
    points: IntArray,
    observing: IntArray,
    result: ba.BAResult,
    dropped: IntArray,
    solves: tuple[Solve, ...],
    aborted: bool,
) -> Adjustment:
    """Write a solve's free poses and points back to the map, forgetting the dropped observations.

    `points` are the problem's point rows in the map and `observing` its keyframe rows.
    """
    free = observing[~problem.fixed_poses]
    poses = m.kf_pose.copy()
    poses[free] = se3.inverse(result.poses[~problem.fixed_poses])
    xyz = m.pt_xyz.copy()
    xyz[points] = result.points
    ids = m.pt_id[points]
    adjusted = maps.forget_observations(replace(m, kf_pose=poses, pt_xyz=xyz), dropped)
    kept = adjusted.point_rows(ids)
    return Adjustment(
        map=maps.refresh_points(adjusted, kept[kept >= 0]),
        solves=solves,
        outliers=len(dropped),
        free=int(np.count_nonzero(~problem.fixed_poses)),
        fixed=int(np.count_nonzero(problem.fixed_poses)),
        aborted=aborted,
    )


def bundle_adjust(  # noqa: PLR0913 -- the map, its camera, what to adjust, the settings
    m: Map,
    camera: Camera,
    keyframes: ArrayLike,
    fixed: ArrayLike,
    options: Options,
    *,
    abort: Callable[[], bool] | None = None,
    backend: str = "python",
) -> Adjustment:
    """Bundle-adjust keyframes and the points they observe (``LocalBundleAdjustment``).

    Solves for ``adjustment.iterations``, drops the observations failing the chi-square test (or
    behind their camera), solves again for ``adjustment.refine_iterations`` (a point left with fewer
    than two inlier views restarting from its position before the adjustment), then forgets
    the observations still failing it and refreshes the adjusted points' descriptors, normals and
    distances. Only the free keyframes' poses are written back, so a fixed keyframe keeps its
    pose exactly.

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    keyframes : array_like of int
        The rows of the keyframes to adjust; every other keyframe observing their points is
        held fixed.
    fixed : array_like of bool, shape (K,)
        Keyframes held fixed even among `keyframes` (the map's first, for the gauge).
    options : Options
        The iterations and thresholds.
    abort : callable, optional
        Asked before each solve whether to stop (the module docstring's "Stopped early");
        default: never.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Adjustment
        The map with adjusted poses and points and the outlier observations forgotten; the
        two solves (none, the map as it was, if no point was observed or no keyframe was held
        fixed, and one if it was stopped before the second) and the outliers dropped.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a keyframe row is out of range, `fixed` is not one flag per keyframe, or `abort` is
        given to a compiled backend.
    """
    if abort is not None and backend != "python":
        raise InvalidInputError("abort: only the Python reference stops early so far")
    given = np.asarray(keyframes)
    chosen = as_indices(given, given.size, m.keyframes, "keyframes")
    held = as_mask(fixed, (m.keyframes,), "fixed")
    return backends.call(
        backend,
        __name__,
        "bundle_adjust",
        reference=lambda: _bundle_adjust(m, camera, chosen, held, options, abort=abort),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "keyframes": chosen,
            "fixed": held,
            "options": options.as_dict(),
        },
        decode=_decode_adjustment,
    )


def _options(loss: nls.Loss, iterations: int) -> ba.BAOptions:
    """Return the BA settings: Levenberg-Marquardt with a robust loss, capped iterations."""
    return ba.BAOptions(solver=nls.Options(loss=loss, max_iterations=iterations))


def _outliers(
    m: Map, camera: Camera, problem: ba.BAProblem, rows: IntArray, options: Options
) -> Mask:
    """Which observations fail the chi-square test, or are behind their camera."""
    t_cam_world = se3.inverse(problem.poses)[problem.cam_idx]
    local = np.einsum("mij,mj->mi", t_cam_world[:, :3, :3], problem.points[problem.pt_idx])
    local += t_cam_world[:, :3, 3]
    right = m.kp_right[rows]
    error = residuals.chi2(
        camera, local, m.kp_xy[rows], right, residuals.sigma2(m, m.kp_level[rows])
    )
    return np.asarray(~(error < residuals.thresholds(right, options)))
