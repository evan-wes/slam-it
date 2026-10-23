"""Factor graphs on GTSAM: a pose graph, a similarity refined against matches, and BA.

Rung 0's back end for loop closure (milestone 2B's B3; the online-SLAM design, section 10),
on GTSAM (Dellaert and Kaess, "Factor Graphs for Robot Perception", 2017; Dellaert, "Factor
graphs and GTSAM: a hands-on introduction", 2012). Three solves, as ORB-SLAM makes them with
g2o (``Optimizer.cc``):

- :func:`optimize_pose_graph`, a pose graph over SE(3) or Sim(3): the essential graph
  (``OptimizeEssentialGraph``);
- :func:`refine_similarity`, the similarity (or rigid motion) that carries one keyframe's
  matched points into another's images, both ways (``OptimizeSim3``);
- :func:`bundle_adjust`, every pose and point against every observation, with no robust loss
  (``GlobalBundleAdjustemnt``, as loop closing calls it) or a Huber loss (the welding BA of a
  merge, ``LocalBundleAdjustment``'s).

Python builds them on GTSAM's official bindings, C++ on the library they wrap, and Rust through
a bridge (``rust/slam-it-foundations``). The two factors GTSAM lacks, a point's reprojection
through rung 0's camera models (radtan's ``k3`` and a stereo right column among them) and a
match's reprojection through a similarity, are each language's own: here GTSAM's
``CustomFactor`` calls back into Python for the residual and its Jacobians. Bundle adjustment
through a pinhole camera takes GTSAM's own factors instead (``GenericProjectionFactorCal3_S2``,
and ``GenericStereoFactor3D`` with a right column), which model it exactly: GTSAM's Python
wrapper copies the whole of a problem's values on every call into a ``CustomFactor``, so a map's
bundle adjustment through them took minutes where GTSAM's own take under a second (decided with
evan-wes on 2026-10-17). Every language's projection factor gives a point behind its camera
GTSAM's own factors' constant error, twice the focal length in each row, with no gradient: with
none, global BA's lack of a robust loss let a monocular map's solve move points behind their
cameras until nothing was left to cost, and the map collapsed (the loop room, 2026-10-17).

**Conventions.**

- A pose crosses as a camera-to-world ``T_world_cam``, 4x4, as rung 0's BA takes it.
- A similarity crosses as the 4x4 ``[[s R, t], [0, 1]]``, mapping ``p`` to ``s R p + t``, as
  rung 0's alignment returns it and ORB-SLAM's ``g2o::Sim3`` acts. GTSAM's ``Similarity3``
  maps ``p`` to ``s (R p + t')``, its matrix ``[[R, t'], [0, 1/s]]``, so ``t' = t / s``.
- GTSAM perturbs on the right, ``X ⊕ ξ = X Exp(ξ)``, rotation first: ``(ω, v)`` for a pose,
  ``(ω, v, λ)`` for a similarity, ``λ`` the logarithm of the scale. Jacobians are in that
  order. (Rung 0's own SE(3) orders its tangent ``(ρ, θ)``; nothing here crosses it.)
- An error is GTSAM's: ``½ Σ ρ(|r|²_Σ)``, a robust loss on the whitened norm.

**Solves.** Levenberg-Marquardt, GTSAM's, driven here one ``iterate()`` at a time so that
every language stops by the same rule (:class:`Termination`), and a ``stop`` check between
iterations can end it, as g2o's ``setForceStopFlag`` ends ORB-SLAM's. Each ``iterate()`` linearises,
then raises its damping ``λ`` until a step lowers the error, or gives up at GTSAM's upper
bound (``1e5``); a step that lowers the error divides ``λ`` by 10 (GTSAM's defaults). An
error GTSAM raises while solving is :class:`~slam_it.foundations.errors.NotConvergedError`,
as the compiled backends report it.

**Held cameras and poses are constants,** never variables, as g2o's fixed vertices hold
ORB-SLAM's (``setFixed``): a held camera's observation is a factor on its point alone (through a
pinhole camera, GTSAM's ``TriangulationFactorCal3_S2``, or with a right column a custom factor
through its ``StereoCamera``, each projecting as the camera's own factor does); a pose graph's
edge with one end held is a factor on its other end (the edge's ``BetweenFactor`` evaluated with
that end at its value); and the edges with both ends held, whose error is a constant, are one
factor on a free pose with a zero Jacobian, so that every error GTSAM reads, its search over
dampings' included, is the whole graph's, as when the held poses were variables. Each held one
comes back exactly as given. Until 0.3.17.2 they were GTSAM's
``NonlinearEquality``, a hard constraint whose ``Constrained`` noise model makes GTSAM's
``EliminatePreferCholesky`` eliminate every clique holding one by dense QR: zju-B6's merge, 52
cameras held in its welding BA, took 658 s so and its run peaked at 2,509 MiB, against 1.4 s
and 300 MiB with them constants (C++, its first 1,070 frames, one core, 2026-10-23).

**A Sim(3) pose graph's edges are linearised to first order.** GTSAM 4.3's ``Similarity3``
has no Jacobian of its chart's ``Local``, so ``BetweenFactor<Similarity3>`` uses the plain
between-Jacobians, exact only at a zero error (``Pose3``'s are exact); g2o's ``EdgeSim3``, as
ORB-SLAM runs it, differentiates numerically. On a 30-node Sim(3) ring the two solutions
agreed within 24 µm with 2.4x scale and 20° rotation drift (B3's review panel, L4).
"""

from __future__ import annotations

import enum
import math
from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Any, NamedTuple, Protocol

import gtsam
import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations import camera
from slam_it.foundations.arrays import as_batch, as_real
from slam_it.foundations.errors import InvalidInputError, NotConvergedError
from slam_it.foundations.so3 import check_rotation

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]

POSE_DIM = 6
"""A pose's tangent dimension, ``(ω, v)``."""

SIMILARITY_DIM = 7
"""A similarity's tangent dimension, ``(ω, v, λ)``."""

STEREO_DIMENSION = 3
"""A stereo observation's length: ``(u, v, u_r)``, the right column after the pixel."""


class Termination(enum.StrEnum):
    """Why a solve stopped."""

    FUNCTION = "function"
    """Converged: an iteration lowered the error by no more than the tolerances, or left it
    at zero."""
    MAX_ITERATIONS = "max_iterations"
    """Not converged: ``max_iterations`` iterations ran."""
    DAMPING_LIMIT = "damping_limit"
    """Not converged: no step lowered the error before the damping reached its upper bound
    (also how a solve at its minimum ends when both tolerances are 0)."""
    STOPPED = "stopped"
    """Not converged: stopped from outside, between iterations (a ``stop`` check)."""


CONVERGED = frozenset({Termination.FUNCTION})
"""The terminations that count as converged."""


@dataclass(frozen=True)
class Options:
    """How a solve runs.

    Attributes
    ----------
    max_iterations : int
        Iterations at most (ORB-SLAM's ``optimize(n)``): each linearises once and tries
        dampings until a step lowers the error. At least 1.
    initial_damping : float
        Levenberg-Marquardt's first ``λ``, added to the normal equations' diagonal (GTSAM's
        ``lambdaInitial``; ORB-SLAM's essential graph sets g2o's to 1e-16). Positive.
    relative_tolerance, absolute_tolerance : float
        An iteration that lowers the error by at most ``relative_tolerance`` times it, or by at
        most ``absolute_tolerance``, ends the solve as converged. Non-negative. The default
        relative tolerance, 1e-10, ends a solve at its minimum as converged, where g2o, which
        ORB-SLAM's solves run on, keeps iterating to its count without changing anything that
        matters; at 0, no step can lower the error there, and the solve ends at the damping's
        limit, not converged.
    """

    max_iterations: int = 10
    initial_damping: float = 1e-5
    relative_tolerance: float = 1e-10
    absolute_tolerance: float = 0.0

    def __post_init__(self) -> None:
        """Check the options.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If an option is out of its range.
        """
        if isinstance(self.max_iterations, bool) or not isinstance(self.max_iterations, int):
            raise InvalidInputError("max_iterations: expected an int")
        if self.max_iterations < 1:
            raise InvalidInputError(f"max_iterations: expected >= 1, got {self.max_iterations}")
        if not (math.isfinite(self.initial_damping) and self.initial_damping > 0.0):
            raise InvalidInputError(f"initial_damping: expected > 0, got {self.initial_damping}")
        for name in ("relative_tolerance", "absolute_tolerance"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise InvalidInputError(f"{name}: expected >= 0, got {value}")

    def as_dict(self) -> dict[str, Any]:
        """Return the options as the compiled backends take them: every field, by name."""
        return asdict(self)


@dataclass(frozen=True)
class Summary:
    """How a solve went.

    Satisfies :class:`slam_it.foundations.solve.Summary`.

    Attributes
    ----------
    initial_error, final_error : float
        GTSAM's error, ``½ Σ ρ(|r|²_Σ)``, at the start and at the returned values.
    iterations : int
        The iterations run (each one linearisation and its search over dampings).
    termination : Termination
        Why the solve stopped.
    converged : bool
        Whether ``termination`` is one of :data:`CONVERGED`.
    """

    initial_error: float
    final_error: float
    iterations: int
    termination: Termination
    converged: bool


@dataclass(frozen=True)
class PoseGraph:
    """A pose graph's solution.

    Attributes
    ----------
    poses : ndarray of float64, shape (N, 4, 4)
        Each pose, camera-to-world: rigid, or a similarity ``[[s R, t], [0, 1]]``.
    summary : Summary
        How the solve went.
    """

    poses: Array
    summary: Summary


@dataclass(frozen=True)
class SimilarityRefinement:
    """A refined similarity (or rigid motion), and how far each match is from it.

    Attributes
    ----------
    similarity : ndarray of float64, shape (4, 4)
        ``S_12``, ``[[s R, t], [0, 1]]``, carrying camera 2's points into camera 1's frame
        (``s = 1`` when the scale was held).
    chi2 : ndarray of float64, shape (M, 2)
        Each match's whitened squared error at the result, before the robust loss: in image 1
        (its point from camera 2), then in image 2 (its point from camera 1); infinite where
        the carried point is behind the camera, NaN where image 2 does not observe the match.
    summary : Summary
        How the solve went.
    """

    similarity: Array
    chi2: Array
    summary: Summary


@dataclass(frozen=True)
class Adjustment:
    """A bundle adjustment's solution.

    Attributes
    ----------
    poses : ndarray of float64, shape (C, 4, 4)
        Each camera's ``T_world_cam``.
    points : ndarray of float64, shape (P, 3)
        Each point, in the world.
    summary : Summary
        How the solve went.
    """

    poses: Array
    points: Array
    summary: Summary


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.graphs`` submodule provides."""

    def codes(self) -> dict[str, list[str]]:
        """Return the module's enum names in code order (``termination``)."""
        ...

    def optimize_pose_graph(  # noqa: PLR0913, PLR0917 -- the graph, its kind and the options
        self,
        poses: Array,
        edges: IntArray,
        relatives: Array,
        fixed: IntArray,
        similarity: bool,
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Solve a pose graph; return ``poses`` and the summary's fields."""
        ...

    def refine_similarity(  # noqa: PLR0913, PLR0917 -- the matches, their camera and the solve
        self,
        similarity: Array,
        points1: Array,
        points2: Array,
        pixels1: Array,
        pixels2: Array,
        information: Array,
        model: int,
        intrinsics: Array,
        fixed_scale: bool,
        huber: float | None,
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Refine a similarity; return ``similarity``, ``chi2`` and the summary's fields."""
        ...

    def bundle_adjust(  # noqa: PLR0913, PLR0917 -- the problem, its camera and the options
        self,
        poses: Array,
        points: Array,
        cameras: IntArray,
        point_ids: IntArray,
        pixels: Array,
        right: Array,
        information: Array,
        model: int,
        intrinsics: Array,
        bf: float,
        fixed: IntArray,
        huber: Array | None,
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Adjust a bundle; return ``poses``, ``points`` and the summary's fields."""
        ...


# --- conversions --------------------------------------------------------------------------


def _similarity_parts(matrix: Array) -> tuple[Array, Array, float]:
    """Split ``[[s R, t], [0, 1]]`` into ``R``, ``t`` and ``s``."""
    scale = float(np.cbrt(np.linalg.det(matrix[:3, :3])))
    return matrix[:3, :3] / scale, matrix[:3, 3].copy(), scale


def _to_similarity3(matrix: Array) -> gtsam.Similarity3:
    """Return GTSAM's ``Similarity3`` for ``[[s R, t], [0, 1]]`` (the module docstring)."""
    rotation, translation, scale = _similarity_parts(matrix)
    return gtsam.Similarity3(gtsam.Rot3(rotation), translation / scale, scale)


def _from_similarity3(value: gtsam.Similarity3) -> Array:
    """Return ``[[s R, t], [0, 1]]`` for GTSAM's ``Similarity3`` (the module docstring)."""
    matrix = np.asarray(value.matrix(), dtype=np.float64)
    scale = 1.0 / matrix[3, 3]
    out = np.eye(4)
    out[:3, :3] = scale * matrix[:3, :3]
    out[:3, 3] = scale * matrix[:3, 3]
    return out


def _hat(vector: Array) -> Array:
    """Return the cross-product matrix ``[v]×``."""
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


# --- the factors' residuals ---------------------------------------------------------------


class ProjectionResidual(NamedTuple):
    """A point's reprojection error in one camera, and its Jacobians.

    Attributes
    ----------
    residual : ndarray of float64, shape (D,)
        The predicted minus the measured, ``D`` = 2, or 3 with a right column.
    d_pose : ndarray of float64, shape (D, 6)
        Its Jacobian on the pose's tangent ``(ω, v)``, perturbed on the right.
    d_point : ndarray of float64, shape (D, 3)
        Its Jacobian on the point.
    in_front : bool
        Whether the point is in front of the camera (``Z > camera.MIN_DEPTH``). One that is not
        cannot be projected: its residual is twice the focal length in every row, its
        Jacobians zero, GTSAM's own projection factors' convention (a constant error with no
        gradient, so a solve gains nothing by moving a point behind its camera).
    """

    residual: Array
    d_pose: Array
    d_point: Array
    in_front: bool


class SimilarityResiduals(NamedTuple):
    """A match's two reprojection errors through a similarity, and their Jacobians.

    Attributes
    ----------
    residual1, d_similarity1 : ndarray of float64, shapes (2,) and (2, 6 or 7)
        ``π(S_12 X_2) - u_1``, and its Jacobian on the motion's tangent, on the right.
    residual2, d_similarity2 : ndarray of float64, shapes (2,) and (2, 6 or 7)
        ``π(S_12⁻¹ X_1) - u_2``, and its Jacobian.
    in_front1, in_front2 : bool
        Whether each carried point is in front of its camera; one that is not contributes
        nothing (zero residual and Jacobian), and its chi-square is infinite. Unlike bundle
        adjustment's constant (:class:`ProjectionResidual`), a step here that carries an inlier
        behind its camera lowers its cost to 0; the effect limits itself, as that match's
        infinite chi-square drops it from the inliers ORB-SLAM's refinement counts, and a
        constant would change which matches survive its two rounds (B3's review panel, L3).
    """

    residual1: Array
    d_similarity1: Array
    residual2: Array
    d_similarity2: Array
    in_front1: bool
    in_front2: bool


def _projected(model: str, intrinsics: Array, point: Array) -> tuple[Array, Array] | None:
    """Return a camera-frame point's pixel and its Jacobian on the point, or None if behind."""
    if not point[2] > camera.MIN_DEPTH:
        return None
    d_pixel, _ = camera.project_jacobians(model, intrinsics, point)
    return camera.project(model, intrinsics, point), d_pixel


def projection_residual(  # noqa: PLR0913, PLR0917 -- the observation and its camera
    pose: Array,
    point: Array,
    measured: Array,
    model: str,
    intrinsics: Array,
    bf: float,
) -> ProjectionResidual:
    """Return a point's reprojection error in one camera, and its Jacobians.

    Parameters
    ----------
    pose : ndarray of float64, shape (4, 4)
        The camera's ``T_world_cam``.
    point : ndarray of float64, shape (3,)
        The point, in the world.
    measured : ndarray of float64, shape (2,) or (3,)
        The observed ``(u, v)``, and the right column ``u_r`` for a stereo observation.
    model, intrinsics : str, ndarray of float64
        The camera model and its intrinsics (:mod:`slam_it.foundations.camera`).
    bf : float
        The baseline times the focal length, for a right column ``u - bf / z``.

    Returns
    -------
    ProjectionResidual
        The residual, its Jacobians, and whether the point is in front of the camera.
    """
    rotation, translation = pose[:3, :3], pose[:3, 3]
    in_camera = rotation.T @ (point - translation)
    size = measured.shape[0]
    projected = _projected(model, intrinsics, in_camera)
    if projected is None:
        fx = float(intrinsics[camera.FOCAL_INDICES[model][0]])
        behind = np.full(size, 2.0 * fx)
        return ProjectionResidual(behind, np.zeros((size, 6)), np.zeros((size, 3)), False)
    pixel, d_pixel = projected
    if size == 3:
        z = in_camera[2]
        right_row = d_pixel[0] + np.array([0.0, 0.0, bf / (z * z)])
        predicted = np.array([pixel[0], pixel[1], pixel[0] - bf / z])
        d_predicted = np.vstack([d_pixel, right_row])
    else:
        predicted = pixel
        d_predicted = d_pixel
    # (T Exp(ξ))⁻¹ X = Exp(-ξ) T⁻¹ X ≈ X_c - ω × X_c - v.
    d_camera_d_pose = np.hstack([_hat(in_camera), -np.eye(3)])
    return ProjectionResidual(
        predicted - measured, d_predicted @ d_camera_d_pose, d_predicted @ rotation.T, True
    )


def similarity_residuals(  # noqa: PLR0913, PLR0917 -- the match, its camera, the motion's kind
    similarity: Array,
    point1: Array,
    point2: Array,
    pixel1: Array,
    pixel2: Array,
    model: str,
    intrinsics: Array,
    fixed_scale: bool,
) -> SimilarityResiduals:
    """Return a match's two reprojection errors through a similarity, and their Jacobians.

    Parameters
    ----------
    similarity : ndarray of float64, shape (4, 4)
        ``S_12``, ``[[s R, t], [0, 1]]`` (``s = 1`` when the scale is held).
    point1, point2 : ndarray of float64, shape (3,)
        The match's point in camera 1's frame and in camera 2's.
    pixel1, pixel2 : ndarray of float64, shape (2,)
        Where images 1 and 2 observe it.
    model, intrinsics : str, ndarray of float64
        The camera model and its intrinsics, the same for both images.
    fixed_scale : bool
        Whether the motion is rigid: its tangent is then ``(ω, v)``, not ``(ω, v, λ)``.

    Returns
    -------
    SimilarityResiduals
        Both residuals, their Jacobians, and whether each carried point is in front.
    """
    rotation, translation, scale = _similarity_parts(similarity)
    dimension = POSE_DIM if fixed_scale else SIMILARITY_DIM
    forward = scale * rotation @ point2 + translation
    backward = rotation.T @ (point1 - translation) / scale
    # S Exp(ξ) p ≈ S (p + ω × p + v + λ p), and (S Exp(ξ))⁻¹ q = Exp(-ξ) S⁻¹ q.
    d_forward = scale * rotation @ np.hstack([-_hat(point2), np.eye(3), point2[:, None]])
    d_backward = np.hstack([_hat(backward), -np.eye(3), -backward[:, None]])
    out: list[Array] = []
    flags: list[bool] = []
    for carried, d_carried, pixel in ((forward, d_forward, pixel1), (backward, d_backward, pixel2)):
        projected = _projected(model, intrinsics, carried)
        if projected is None:
            out += [np.zeros(2), np.zeros((2, dimension))]
            flags.append(False)
        else:
            out += [projected[0] - pixel, projected[1] @ d_carried[:, :dimension]]
            flags.append(True)
    return SimilarityResiduals(out[0], out[1], out[2], out[3], flags[0], flags[1])


# --- the solve ----------------------------------------------------------------------------


def _solve(
    graph: gtsam.NonlinearFactorGraph,
    values: gtsam.Values,
    options: Options,
    stop: Callable[[], bool] | None = None,
) -> tuple[gtsam.Values, Summary]:
    """Run Levenberg-Marquardt by :class:`Termination`'s rule; return the values and summary.

    `stop`, if given, is asked before each iteration whether to end the solve there.
    """
    params = gtsam.LevenbergMarquardtParams()
    params.setlambdaInitial(options.initial_damping)
    params.setRelativeErrorTol(options.relative_tolerance)
    params.setAbsoluteErrorTol(options.absolute_tolerance)
    # Whatever GTSAM raises (its wrapper's RuntimeError, an index or a key it cannot find, an
    # error inside a custom factor's callback) ends the solve as NotConvergedError, as the C++
    # and Rust cores catch every exception GTSAM throws: so a broad catch, here alone.
    try:
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
        initial = current = float(optimizer.error())
    except Exception as failure:
        raise NotConvergedError(f"GTSAM failed: {failure}") from failure
    termination = Termination.MAX_ITERATIONS
    iterations = 0
    while current > 0.0 and iterations < options.max_iterations:
        if stop is not None and stop():
            termination = Termination.STOPPED
            break
        try:
            optimizer.iterate()
        except Exception as failure:
            raise NotConvergedError(f"GTSAM failed: {failure}") from failure
        iterations += 1
        error = float(optimizer.error())
        if error >= current and optimizer.lambda_() >= params.getlambdaUpperBound():
            termination = Termination.DAMPING_LIMIT
            break
        decrease = current - error
        current = error
        if decrease <= options.absolute_tolerance or decrease <= options.relative_tolerance * (
            current + decrease
        ):
            termination = Termination.FUNCTION
            break
    if current == 0.0:
        termination = Termination.FUNCTION
    summary = Summary(
        initial_error=initial,
        final_error=current,
        iterations=iterations,
        termination=termination,
        converged=termination in CONVERGED,
    )
    return optimizer.values(), summary


def summary_from_backend(raw: dict[str, Any], codes: dict[str, list[str]]) -> Summary:
    """Build a solve's summary from the dict a compiled backend returned.

    Parameters
    ----------
    raw : dict
        ``initial_error``, ``final_error``, ``iterations`` and ``termination`` (an index into
        ``codes["graphs_termination"]``).
    codes : dict of str to list of str
        The backend's enum names in code order (its ``codes()``).

    Returns
    -------
    Summary
        The summary.
    """
    termination = Termination(backends.decode_name(codes, "graphs_termination", raw["termination"]))
    return Summary(
        initial_error=float(raw["initial_error"]),
        final_error=float(raw["final_error"]),
        iterations=int(raw["iterations"]),
        termination=termination,
        converged=termination in CONVERGED,
    )


# --- input checks -------------------------------------------------------------------------


def _poses(poses: ArrayLike, similarity: bool, name: str) -> Array:
    """Check (N, 4, 4) poses: rigid, or similarities with a positive scale."""
    out, lead = as_batch(poses, (4, 4), name)
    if len(lead) != 1 or lead[0] == 0:
        raise InvalidInputError(f"{name}: expected (N, 4, 4) with N >= 1, got {out.shape}")
    for k, matrix in enumerate(out):
        if not np.array_equal(matrix[3], [0.0, 0.0, 0.0, 1.0]):
            raise InvalidInputError(f"{name}[{k}]: expected a last row of [0, 0, 0, 1]")
        block = matrix[:3, :3]
        determinant = float(np.linalg.det(block))
        if not determinant > 0.0:
            raise InvalidInputError(f"{name}[{k}]: expected a positive determinant")
        scale = float(np.cbrt(determinant)) if similarity else 1.0
        try:
            check_rotation((block / scale)[None])
        except InvalidInputError as error:
            raise InvalidInputError(f"{name}[{k}]: {error}") from None
    return out


def _indices(values: ArrayLike, bound: int, name: str, *, columns: int | None = None) -> IntArray:
    """Check integer indices in ``[0, bound)``, one-dimensional or with `columns` columns."""
    array = np.asarray(values)
    if array.size and not np.issubdtype(array.dtype, np.integer):
        raise InvalidInputError(f"{name}: expected integers")
    out = array.astype(np.int64).reshape(-1) if columns is None else array.astype(np.int64)
    if columns is not None and (out.ndim != 2 or out.shape[1] != columns):
        raise InvalidInputError(f"{name}: expected shape (E, {columns}), got {out.shape}")
    if out.size and (out.min() < 0 or out.max() >= bound):
        raise InvalidInputError(f"{name}: expected indices in [0, {bound})")
    return out


def _information(values: ArrayLike, count: int, name: str) -> Array:
    """Check `count` positive, finite informations."""
    out = as_real(values, name).reshape(-1)
    if out.shape != (count,) or not np.all(out > 0.0):
        raise InvalidInputError(f"{name}: expected {count} positive values")
    return out


# --- the pose graph -----------------------------------------------------------------------


def _held_end(
    edge: gtsam.NonlinearFactor,
    held: int,
    value: gtsam.Pose3 | gtsam.Similarity3,
    free: int,
    similarity: bool,
) -> gtsam.CustomFactor:
    """Return a pose graph's edge with one end held at `value`: a factor on its `free` end alone.

    The edge's own ``BetweenFactor`` is linearised with the held end at its value, so the
    residual and the free end's Jacobian are GTSAM's own, whitened by the edge's noise; they are
    wrapped in unit noise, which whitens them no further. GTSAM's Python wrapper has no unary
    form of the between factor.
    """
    dimension = SIMILARITY_DIM if similarity else POSE_DIM
    columns = slice(0, dimension) if edge.keys()[0] == free else slice(dimension, 2 * dimension)

    def evaluate(values: gtsam.Values) -> tuple[Array, list[Array]]:
        ends = gtsam.Values()
        ends.insert(held, value)
        ends.insert(free, values.atSimilarity3(free) if similarity else values.atPose3(free))
        jacobian, rhs = edge.linearize(ends).jacobian()
        return -rhs, [jacobian[:, columns]]

    return _custom(gtsam.noiseModel.Unit.Create(dimension), [free], evaluate)


def _held_edges(residuals: Array, free: int, similarity: bool) -> gtsam.CustomFactor:
    """Return a pose graph's edges with both ends held as one factor on a `free` pose.

    Their stacked `residuals`, a constant, with unit noise and a zero Jacobian: the factor moves
    nothing, but every error GTSAM reads, its search over dampings' included, holds theirs.
    """
    jacobian = np.zeros((residuals.shape[0], SIMILARITY_DIM if similarity else POSE_DIM))

    def evaluate(_: gtsam.Values) -> tuple[Array, list[Array]]:
        return residuals, [jacobian]

    return _custom(gtsam.noiseModel.Unit.Create(residuals.shape[0]), [free], evaluate)


def _pose_graph(  # noqa: PLR0913, PLR0917 -- the graph, its kind and the options
    poses: Array,
    edges: IntArray,
    relatives: Array,
    fixed: IntArray,
    similarity: bool,
    options: Options,
) -> PoseGraph:
    """Solve a pose graph (reference); the held poses are constants (the module docstring)."""
    held = {int(k) for k in fixed}
    value_of: Callable[[Array], gtsam.Pose3 | gtsam.Similarity3] = (
        _to_similarity3 if similarity else gtsam.Pose3
    )
    between = gtsam.BetweenFactorSimilarity3 if similarity else gtsam.BetweenFactorPose3
    unit = gtsam.noiseModel.Unit.Create(SIMILARITY_DIM if similarity else POSE_DIM)
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    for k, pose in enumerate(poses):
        if k not in held:
            values.insert(k, value_of(pose))
    parts: list[Array] = []
    for (i, j), relative in zip(edges.tolist(), relatives, strict=True):
        edge = between(i, j, value_of(relative), unit)
        if i in held and j in held:
            ends = gtsam.Values()
            ends.insert(i, value_of(poses[i]))
            ends.insert(j, value_of(poses[j]))
            parts.append(np.asarray(edge.unwhitenedError(ends), dtype=np.float64))
        elif i in held or j in held:
            end, other = (i, j) if i in held else (j, i)
            graph.add(_held_end(edge, end, value_of(poses[end]), other, similarity))
        else:
            graph.add(edge)
    residuals = np.concatenate(parts) if parts else np.zeros(0)
    free = next((k for k in range(poses.shape[0]) if k not in held), None)
    if free is None:  # every pose held: nothing is solved, the error the held edges'
        error = 0.5 * float(residuals @ residuals)
        nothing = Summary(
            initial_error=error,
            final_error=error,
            iterations=0,
            termination=Termination.FUNCTION,
            converged=True,
        )
        return PoseGraph(poses=poses.copy(), summary=nothing)
    if parts:
        graph.add(_held_edges(residuals, free, similarity))
    solved, summary = _solve(graph, values, options)
    out = poses.copy()
    for k in range(poses.shape[0]):
        if k not in held:
            out[k] = (
                _from_similarity3(solved.atSimilarity3(k))
                if similarity
                else np.asarray(solved.atPose3(k).matrix(), dtype=np.float64)
            )
    return PoseGraph(poses=out, summary=summary)


def optimize_pose_graph(  # noqa: PLR0913 -- the graph, its kind, options, backend
    poses: ArrayLike,
    edges: ArrayLike,
    relatives: ArrayLike,
    fixed: ArrayLike,
    *,
    similarity: bool,
    options: Options | None = None,
    backend: str = "python",
) -> PoseGraph:
    """Optimise a pose graph: each edge's measured relative pose, each with unit information.

    ORB-SLAM's ``OptimizeEssentialGraph`` on GTSAM: each edge ``(i, j)`` measures the pose of
    ``j`` in ``i``'s frame, ``T_i_j = T_world_i⁻¹ T_world_j`` (GTSAM's ``BetweenFactor``),
    with unit information, as ORB-SLAM's edges have; the poses in `fixed` are held as
    constants, and at least one must be, for the graph's gauge. An edge with one end held is a
    factor on its other end; the edges with both ends held, whose error is a constant, are one
    factor on a free pose with a zero Jacobian, so that every error GTSAM reads, and the
    summary's, holds theirs. With every pose held, nothing is solved: the poses come back as
    given, after no iteration.

    Parameters
    ----------
    poses : array_like, shape (N, 4, 4)
        Each pose's starting value, camera-to-world: rigid, or with `similarity` a similarity
        ``[[s R, t], [0, 1]]``, ``s > 0``.
    edges : array_like of int, shape (E, 2)
        Each edge's two poses ``(i, j)``.
    relatives : array_like, shape (E, 4, 4)
        Each edge's measured ``T_i_j``, of the poses' kind.
    fixed : array_like of int, shape (F,)
        The poses held fixed, F >= 1; each comes back exactly as given.
    similarity : bool
        Optimise Sim(3), as a monocular map's drift needs, rather than SE(3).
    options : Options, optional
        How the solve runs (default: :class:`Options`'s).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    PoseGraph
        The poses and how the solve went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a pose or relative is not of the kind (a rotation, or a rotation and a positive
        scale, over a last row of ``[0, 0, 0, 1]``), the shapes disagree, an index is out of
        range, an edge joins a pose to itself, or no pose is fixed.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while solving.
    """
    chosen = options or Options()
    checked = _poses(poses, similarity, "poses")
    count = checked.shape[0]
    pairs = _indices(edges, count, "edges", columns=2)
    measured = _poses(relatives, similarity, "relatives") if pairs.size else np.zeros((0, 4, 4))
    if measured.shape[0] != pairs.shape[0]:
        raise InvalidInputError("relatives: expected one per edge")
    if pairs.size and np.any(pairs[:, 0] == pairs[:, 1]):
        raise InvalidInputError("edges: an edge joins a pose to itself")
    held = np.unique(_indices(fixed, count, "fixed"))
    if held.size == 0:
        raise InvalidInputError("fixed: expected at least one pose held fixed (the gauge)")
    return backends.call(
        backend,
        __name__,
        "optimize_pose_graph",
        reference=lambda: _pose_graph(checked, pairs, measured, held, similarity, chosen),
        arguments={
            "poses": checked,
            "edges": pairs,
            "relatives": measured,
            "fixed": held,
            "similarity": similarity,
            "options": chosen.as_dict(),
        },
        decode=lambda raw, codes: PoseGraph(
            poses=backends.checked_array(raw["poses"], np.float64, 3, "poses"),
            summary=summary_from_backend(raw, codes),
        ),
    )


# --- the similarity's refinement ----------------------------------------------------------


def _custom(
    noise: gtsam.noiseModel.Base,
    keys: list[int],
    evaluate: Callable[[gtsam.Values], tuple[Array, list[Array]]],
) -> gtsam.CustomFactor:
    """Wrap a residual and its Jacobians, on GTSAM's values, as a ``CustomFactor``."""

    def error(_: gtsam.CustomFactor, values: gtsam.Values, jacobians: Any) -> Array:  # noqa: ANN401 -- GTSAM's JacobianVector
        residual, blocks = evaluate(values)
        if jacobians is not None:
            for k, block in enumerate(blocks):
                jacobians[k] = block
        return residual

    return gtsam.CustomFactor(noise, keys, error)


def _motion_matrix(values: gtsam.Values, fixed_scale: bool) -> Array:
    """Return the refined motion's ``[[s R, t], [0, 1]]`` from GTSAM's values (key 0)."""
    if fixed_scale:
        return np.asarray(values.atPose3(0).matrix(), dtype=np.float64)
    return _from_similarity3(values.atSimilarity3(0))


def _refine(  # noqa: PLR0913, PLR0917 -- the matches, their camera and the solve
    similarity: Array,
    points1: Array,
    points2: Array,
    pixels1: Array,
    pixels2: Array,
    information: Array,
    model: str,
    intrinsics: Array,
    fixed_scale: bool,
    huber: float | None,
    options: Options,
) -> SimilarityRefinement:
    """Refine a similarity against its matches (reference)."""
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    if fixed_scale:
        values.insert(0, gtsam.Pose3(similarity))
    else:
        values.insert(0, _to_similarity3(similarity))

    def residuals(k: int, motion: Array) -> SimilarityResiduals:
        return similarity_residuals(
            motion, points1[k], points2[k], pixels1[k], pixels2[k], model, intrinsics, fixed_scale
        )

    for k in range(points1.shape[0]):
        for side in (0, 1) if np.isfinite(pixels2[k, 0]) else (0,):
            noise = gtsam.noiseModel.Isotropic.Precision(2, float(information[k, side]))
            if huber is not None:
                noise = gtsam.noiseModel.Robust.Create(
                    gtsam.noiseModel.mEstimator.Huber.Create(huber), noise
                )

            def evaluate(
                values: gtsam.Values, k: int = k, side: int = side
            ) -> tuple[Array, list[Array]]:
                both = residuals(k, _motion_matrix(values, fixed_scale))
                if side == 0:
                    return both.residual1, [both.d_similarity1]
                return both.residual2, [both.d_similarity2]

            graph.add(_custom(noise, [0], evaluate))
    solved, summary = _solve(graph, values, options)
    motion = _motion_matrix(solved, fixed_scale)
    chi2 = np.zeros((points1.shape[0], 2))
    for k in range(points1.shape[0]):
        both = residuals(k, motion)
        second = both.residual2 @ both.residual2 if both.in_front2 else np.inf
        chi2[k] = information[k] * np.array(
            [
                both.residual1 @ both.residual1 if both.in_front1 else np.inf,
                second if np.isfinite(pixels2[k, 0]) else np.nan,
            ]
        )
    return SimilarityRefinement(similarity=motion, chi2=chi2, summary=summary)


def refine_similarity(  # noqa: PLR0913, PLR0917 -- the matches, their camera and the solve
    similarity: ArrayLike,
    points1: ArrayLike,
    points2: ArrayLike,
    pixels1: ArrayLike,
    pixels2: ArrayLike,
    information: ArrayLike,
    *,
    model: str,
    intrinsics: ArrayLike,
    fixed_scale: bool,
    huber: float | None,
    options: Options | None = None,
    backend: str = "python",
) -> SimilarityRefinement:
    """Refine the similarity carrying one keyframe's matched points into another's images.

    ORB-SLAM's ``OptimizeSim3`` on GTSAM, one round: the motion ``S_12`` is the only variable,
    the points held where they are; each match gives two errors, its camera-2 point carried
    into image 1 and its camera-1 point carried into image 2, each with its keypoint's
    information and a Huber loss at `huber` (ORB-SLAM: ``√10``). The caller drops the matches
    whose ``chi2`` exceeds its bound and runs another round with no robust loss, as ORB-SLAM
    does.

    Parameters
    ----------
    similarity : array_like, shape (4, 4)
        The starting ``S_12``, ``[[s R, t], [0, 1]]``, ``s > 0`` (``s = 1`` when `fixed_scale`).
    points1, points2 : array_like, shape (M, 3)
        Each match's point in camera 1's frame and in camera 2's, in front of each.
    pixels1, pixels2 : array_like, shape (M, 2)
        Where images 1 and 2 observe them. A row of ``pixels2`` may be NaN: image 2 does not
        observe that match (its point was found in a neighbour of keyframe 2), and only its
        image-1 error counts. ORB-SLAM3 gives such a match an image-2 error against its
        normalised coordinates, not pixels, which counts it as an outlier.
    information : array_like, shape (M, 2)
        Each observation's information, ``1/σ²`` in px⁻², in image 1 then image 2.
    model : {"pinhole", "radtan", "radial"}
        The camera model, the same for both images.
    intrinsics : array_like, shape (K,)
        Its intrinsics.
    fixed_scale : bool
        Hold the scale at 1, refining a rigid motion (stereo and RGB-D: ORB-SLAM's
        ``mbFixScale``).
    huber : float or None
        The Huber loss's threshold on the whitened error's norm, > 0; None for no robust loss.
    options : Options, optional
        How the solve runs (default: :class:`Options`'s).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    SimilarityRefinement
        The motion, each match's two chi-squares at it, and how the solve went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the motion is not a similarity (a rigid motion with `fixed_scale`), the shapes
        disagree, there is no match, a row of `pixels2` is neither finite nor wholly NaN, an
        information or `huber` is not positive, or the camera is invalid.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while solving.
    """
    chosen = options or Options()
    (motion,) = _poses(np.asarray(similarity)[None], not fixed_scale, "similarity")
    one, lead = as_batch(points1, (3,), "points1")
    two, lead2 = as_batch(points2, (3,), "points2")
    seen1, lead3 = as_batch(pixels1, (2,), "pixels1")
    raw2 = np.asarray(pixels2, dtype=np.float64)
    shaped = raw2.ndim == 2 and raw2.shape[1] == 2
    unseen = (
        np.asarray(np.isnan(raw2).all(axis=1), dtype=np.bool_)
        if shaped
        else np.zeros(0, dtype=np.bool_)
    )
    if shaped and not np.isfinite(raw2[~unseen]).all():
        raise InvalidInputError(
            "pixels2: expected each row finite, or wholly NaN where image 2 does not observe it"
        )
    seen2, lead4 = as_batch(
        np.where(unseen[:, None], 0.0, raw2) if shaped else raw2, (2,), "pixels2"
    )
    if shaped:
        seen2[unseen] = np.nan
    if len(lead) != 1 or lead[0] == 0 or not lead == lead2 == lead3 == lead4:
        raise InvalidInputError("matches: expected equal (M, 3) and (M, 2) arrays, M >= 1")
    weights = _information(information, 2 * lead[0], "information").reshape(lead[0], 2)
    if huber is not None and not (math.isfinite(huber) and huber > 0.0):
        raise InvalidInputError(f"huber: expected > 0 or None, got {huber}")
    code, params = camera.as_model(model, intrinsics)

    def decode(raw: dict[str, Any], codes: dict[str, list[str]]) -> SimilarityRefinement:
        return SimilarityRefinement(
            similarity=backends.checked_array(raw["similarity"], np.float64, 2, "similarity"),
            chi2=backends.checked_array(raw["chi2"], np.float64, 2, "chi2"),
            summary=summary_from_backend(raw, codes),
        )

    return backends.call(
        backend,
        __name__,
        "refine_similarity",
        reference=lambda: _refine(
            motion, one, two, seen1, seen2, weights, model, params, fixed_scale, huber, chosen
        ),
        arguments={
            "similarity": motion,
            "points1": one,
            "points2": two,
            "pixels1": seen1,
            "pixels2": seen2,
            "information": weights,
            "model": code,
            "intrinsics": params,
            "fixed_scale": fixed_scale,
            "huber": huber,
            "options": chosen.as_dict(),
        },
        decode=decode,
    )


# --- bundle adjustment --------------------------------------------------------------------


def _pinhole_factors(
    intrinsics: Array, bf: float
) -> Callable[[Array, gtsam.noiseModel.Base, int | Array, int], gtsam.NonlinearFactor]:
    """Return a builder of GTSAM's own projection factors for a pinhole camera.

    A measurement ``(u, v)`` makes a ``GenericProjectionFactorCal3_S2``; ``(u, v, u_r)`` a
    ``GenericStereoFactor3D``, its baseline ``bf / fx``. A held camera, given by its pose rather
    than its key, makes a factor on the point alone that projects as those do:
    ``TriangulationFactorCal3_S2``, and with a right column a custom factor through GTSAM's
    ``StereoCamera`` (its wrapper has no stereo triangulation factor), costing a point behind
    the camera what ``GenericStereoFactor3D`` costs it.
    """
    fx, fy, cx, cy = (float(x) for x in intrinsics)
    mono = gtsam.Cal3_S2(fx, fy, 0.0, cx, cy)
    stereo = gtsam.Cal3_S2Stereo(fx, fy, 0.0, cx, cy, bf / fx) if bf > 0.0 else None

    def held_stereo(
        observed: gtsam.StereoPoint2, noise: gtsam.noiseModel.Base, pose: Array, point: int
    ) -> gtsam.NonlinearFactor:
        rig = gtsam.StereoCamera(gtsam.Pose3(pose), stereo)

        def evaluate(values: gtsam.Values) -> tuple[Array, list[Array]]:
            at = values.atPoint3(point)
            if rig.pose().transformTo(at)[2] <= 0.0:  # GTSAM's StereoCheiralityException
                return np.full(STEREO_DIMENSION, 2.0 * fx), [np.zeros((STEREO_DIMENSION, 3))]
            d_point = np.zeros((STEREO_DIMENSION, 3))
            predicted = rig.project2(at, None, d_point)
            return predicted.vector() - observed.vector(), [d_point]

        return _custom(noise, [point], evaluate)

    def build(
        measured: Array, noise: gtsam.noiseModel.Base, pose: int | Array, point: int
    ) -> gtsam.NonlinearFactor:
        if measured.shape[0] == STEREO_DIMENSION:
            u, v, u_right = (float(x) for x in measured)
            observed = gtsam.StereoPoint2(u, u_right, v)
            if isinstance(pose, int):
                return gtsam.GenericStereoFactor3D(observed, noise, pose, point, stereo)
            return held_stereo(observed, noise, pose, point)
        if isinstance(pose, int):
            return gtsam.GenericProjectionFactorCal3_S2(measured, noise, pose, point, mono)
        lens = gtsam.PinholeCameraCal3_S2(gtsam.Pose3(pose), mono)
        return gtsam.TriangulationFactorCal3_S2(lens, measured, noise, point)

    return build


def _bundle(  # noqa: PLR0913, PLR0917 -- the problem, its camera and the solve
    poses: Array,
    points: Array,
    cameras: IntArray,
    point_ids: IntArray,
    pixels: Array,
    right: Array,
    information: Array,
    model: str,
    intrinsics: Array,
    bf: float,
    fixed: IntArray,
    huber: Array | None,
    options: Options,
    stop: Callable[[], bool] | None = None,
) -> Adjustment:
    """Adjust a bundle (reference); the held cameras are constants (the module docstring)."""
    held = {int(c) for c in fixed}
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    first_point = poses.shape[0]
    for c, pose in enumerate(poses):
        if c not in held:
            values.insert(c, gtsam.Pose3(pose))
    for p, point in enumerate(points):
        values.insert(first_point + p, np.asarray(point, dtype=np.float64))
    builtin = _pinhole_factors(intrinsics, bf) if model == "pinhole" else None
    for m in range(cameras.shape[0]):
        stereo = not math.isnan(right[m])
        measured = np.append(pixels[m], right[m]) if stereo else pixels[m]
        c, p = int(cameras[m]), int(point_ids[m])
        noise = gtsam.noiseModel.Isotropic.Precision(measured.shape[0], float(information[m]))
        if huber is not None:
            loss = gtsam.noiseModel.mEstimator.Huber.Create(float(huber[m]))
            noise = gtsam.noiseModel.Robust.Create(loss, noise)
        if builtin is not None:
            graph.add(builtin(measured, noise, poses[c] if c in held else c, first_point + p))
            continue

        def evaluate(
            values: gtsam.Values, c: int = c, p: int = p, measured: Array = measured
        ) -> tuple[Array, list[Array]]:
            projected = projection_residual(
                poses[c] if c in held else np.asarray(values.atPose3(c).matrix(), np.float64),
                np.asarray(values.atPoint3(first_point + p), dtype=np.float64),
                measured,
                model,
                intrinsics,
                bf,
            )
            if c in held:
                return projected.residual, [projected.d_point]
            return projected.residual, [projected.d_pose, projected.d_point]

        keys = [first_point + p] if c in held else [c, first_point + p]
        graph.add(_custom(noise, keys, evaluate))
    solved, summary = _solve(graph, values, options, stop)
    return Adjustment(
        poses=np.stack(
            [
                poses[c] if c in held else np.asarray(solved.atPose3(c).matrix(), np.float64)
                for c in range(len(poses))
            ]
        ),
        points=np.stack(
            [
                np.asarray(solved.atPoint3(first_point + p), dtype=np.float64)
                for p in range(len(points))
            ]
        ).reshape(-1, 3),
        summary=summary,
    )


def bundle_adjust(  # noqa: PLR0913, PLR0917 -- the problem, its camera, options, backend
    poses: ArrayLike,
    points: ArrayLike,
    cameras: ArrayLike,
    point_ids: ArrayLike,
    pixels: ArrayLike,
    right: ArrayLike,
    information: ArrayLike,
    *,
    model: str,
    intrinsics: ArrayLike,
    bf: float,
    fixed: ArrayLike,
    huber: ArrayLike | None = None,
    options: Options | None = None,
    stop: Callable[[], bool] | None = None,
    backend: str = "python",
) -> Adjustment:
    """Adjust every pose and point against every observation, with no robust loss or Huber's.

    ORB-SLAM's ``GlobalBundleAdjustemnt`` as loop closing calls it (``bRobust`` false), on
    GTSAM: each observation's reprojection through the camera model, with a right column
    ``u - bf / z`` where it has one (ORB-SLAM's stereo edge), weighted by its information; the
    cameras in `fixed` held as constants, each of their observations a factor on its point
    alone. Every point must be seen; one seen once is free along its ray and constrained across
    it, and the damping holds it along the ray. With `huber`, each observation's whitened error
    passes through a Huber loss at its threshold (GTSAM's ``mEstimator::Huber``), as ORB-SLAM's
    local and welding BAs weigh theirs (``RobustKernelHuber``).

    Parameters
    ----------
    poses : array_like, shape (C, 4, 4)
        Each camera's starting ``T_world_cam``.
    points : array_like, shape (P, 3)
        Each point's starting position in the world.
    cameras, point_ids : array_like of int, shape (M,)
        Each observation's camera and point.
    pixels : array_like, shape (M, 2)
        Each observation's ``(u, v)``.
    right : array_like, shape (M,)
        Each observation's right column, NaN for none.
    information : array_like, shape (M,)
        Each observation's information, ``1/σ²`` in px⁻².
    model : {"pinhole", "radtan", "radial"}
        The camera model.
    intrinsics : array_like, shape (K,)
        Its intrinsics.
    bf : float
        The baseline times the focal length, > 0 if any observation has a right column.
    fixed : array_like of int, shape (F,)
        The cameras held fixed; each comes back exactly as given.
    huber : array_like, shape (M,), optional
        Each observation's Huber threshold on its whitened error's norm, positive (default:
        no robust loss).
    options : Options, optional
        How the solve runs (default: :class:`Options`'s).
    stop : callable, optional
        Asked before each iteration whether to stop (:attr:`Termination.STOPPED`); the Python
        reference's alone (a compiled core takes its stop flag in its own call).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Adjustment
        The poses, the points and how the solve went.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If a pose is not rigid, the shapes disagree, an index is out of range, a point is not
        observed, an information or a Huber threshold is not positive, a right column is given
        with ``bf <= 0``, the camera is invalid, or a ``stop`` is given to a compiled backend.
    slam_it.foundations.errors.NotConvergedError
        If GTSAM fails while solving.
    """
    if stop is not None and backend != "python":
        raise InvalidInputError("stop: a compiled backend's core takes its own stop flag")
    chosen = options or Options()
    checked = _poses(poses, False, "poses")
    world, lead = as_batch(points, (3,), "points")
    if len(lead) != 1 or lead[0] == 0:
        raise InvalidInputError(f"points: expected (P, 3) with P >= 1, got {world.shape}")
    which = _indices(cameras, checked.shape[0], "cameras")
    whose = _indices(point_ids, world.shape[0], "point_ids")
    seen, lead_px = as_batch(pixels, (2,), "pixels")
    columns = np.asarray(right, dtype=np.float64).reshape(-1)
    if not which.shape[0] == whose.shape[0] == seen.shape[0] == columns.shape[0] or lead_px == ():
        raise InvalidInputError("observations: expected cameras, point_ids, pixels, right alike")
    if np.any(np.isinf(columns)):
        raise InvalidInputError("right: expected finite values or NaN")
    weights = _information(information, which.shape[0], "information")
    if np.unique(whose).shape[0] != world.shape[0]:
        raise InvalidInputError("point_ids: every point must be observed")
    if np.any(~np.isnan(columns)) and not (math.isfinite(bf) and bf > 0.0):
        raise InvalidInputError(f"bf: expected > 0 with a right column, got {bf}")
    held = np.unique(_indices(fixed, checked.shape[0], "fixed"))
    code, params = camera.as_model(model, intrinsics)
    robust = None if huber is None else _information(huber, which.shape[0], "huber")

    def decode(raw: dict[str, Any], codes: dict[str, list[str]]) -> Adjustment:
        return Adjustment(
            poses=backends.checked_array(raw["poses"], np.float64, 3, "poses"),
            points=backends.checked_array(raw["points"], np.float64, 2, "points"),
            summary=summary_from_backend(raw, codes),
        )

    return backends.call(
        backend,
        __name__,
        "bundle_adjust",
        reference=lambda: _bundle(
            checked,
            world,
            which,
            whose,
            seen,
            columns,
            weights,
            model,
            params,
            bf,
            held,
            robust,
            chosen,
            stop,
        ),
        arguments={
            "poses": checked,
            "points": world,
            "cameras": which,
            "point_ids": whose,
            "pixels": seen,
            "right": columns,
            "information": weights,
            "model": code,
            "intrinsics": params,
            "bf": bf,
            "fixed": held,
            "huber": robust,
            "options": chosen.as_dict(),
        },
        decode=decode,
    )
