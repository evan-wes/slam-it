"""Tracking: place each frame against the map (ORB-SLAM's ``Tracking``, lite).

- **Tracking the last frame** (:func:`track_last_frame`, ``TrackWithMotionModel``): the pose
  predicted by a constant velocity (:func:`predict`); the last frame's map points projected
  with it and matched in a window (:func:`slam_it.foundations.matching.search_by_projection`),
  twice as wide if too few match; the pose refined (:func:`optimize_pose`).
- **Tracking the local map** (:func:`track_local_map`, ``TrackLocalMap``): the points the
  sliding window's keyframes observe, those in view (:func:`in_view`, ``isInFrustum``)
  projected and matched at their predicted level; the pose refined again. Its inliers decide
  whether the frame is tracked.
- **The fallback** (:func:`relocalize_locally`): when the prediction fails, the frame's
  descriptors are matched to the local map's and PnP RANSAC
  (:func:`slam_it.foundations.pnp.pnp_ransac`) places it. It recovers only near the local map;
  past it, the pipeline relocalises the frame against the whole map through its bag of words
  (:mod:`slam_it.edge.relocalization`), and a track lost closes the map, the next frame starting
  a new one (:class:`slam_it.edge.pipeline.Pipeline`).

Each stage reports how it went beside its result (:class:`MotionSummary`,
:class:`PnpSummary`, :meth:`LocalTracking.summary`), which a run records per frame, and the map
point it matched each keypoint to before refining the pose (``matched``), from which a run
records each keypoint's fate (:class:`slam_it.edge.pipeline.Fate`).

**Pose refinement** (:func:`optimize_pose`, ORB-SLAM's ``PoseOptimization``) runs rounds of
:func:`slam_it.foundations.pnp.refine_pose` (Cauchy, on the round's inliers, each match
weighted by its level's information ``1/s^(2l)``, ORB-SLAM's ``invSigma2``) and classifies
every match afresh after each, by the chi-square test (:func:`slam_it.edge.residuals.chi2`).
The solve itself uses the left image only (ORB-SLAM's stereo edges add the right column).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.edge import frames, residuals
from slam_it.edge import map as maps
from slam_it.edge.frames import Camera, Frame, Tracked
from slam_it.edge.map import Map
from slam_it.edge.options import Options, check_mode
from slam_it.foundations import matching, p3p, pnp, ransac, so3
from slam_it.foundations.arrays import as_flag
from slam_it.foundations.errors import DegenerateError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]
Mask = NDArray[np.bool_]


@dataclass(frozen=True)
class InView:
    """Map points in a camera's view (``isInFrustum``), with where and how to search them.

    Attributes
    ----------
    rows : ndarray of int64, shape (V,)
        The points' rows.
    xy : ndarray of float64, shape (V, 2)
        Their projections.
    right : ndarray of float64, shape (V,)
        Their predicted right columns (NaN without a baseline).
    level : ndarray of int64, shape (V,)
        Their predicted levels.
    view_cos : ndarray of float64, shape (V,)
        The cosine between each viewing ray and the point's mean viewing direction.
    """

    rows: IntArray
    xy: Array
    right: Array
    level: IntArray
    view_cos: Array


@dataclass(frozen=True)
class LocalMapSummary:
    """How tracking the local map went, as a run records it.

    Attributes
    ----------
    in_view : int
        The points in view: those the frame had matched, and the local map's others in view.
    matches : int
        The keypoints matched before the pose was refined.
    inliers : int
        The matches the refinement kept.
    """

    in_view: int
    matches: int
    inliers: int

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``in_view``, ``matches`` and ``inliers``.
        """
        return {"in_view": self.in_view, "matches": self.matches, "inliers": self.inliers}


@dataclass(frozen=True)
class LocalTracking:
    """A frame tracked against the local map, and the counts it adds to.

    Attributes
    ----------
    tracked : slam_it.edge.frames.Tracked
        The refined pose and the inlier matches.
    visible : ndarray of int64
        The ids of the points that were in view (their visible counts grow).
    found : ndarray of int64
        The ids of the inliers (their found counts grow).
    matched : ndarray of int64, shape (N,)
        The id of the map point each keypoint matched before the pose was refined, -1 where
        none.
    """

    tracked: Tracked
    visible: IntArray
    found: IntArray
    matched: IntArray

    def summary(self) -> LocalMapSummary:
        """Return how the stage went.

        Returns
        -------
        LocalMapSummary
            The points in view, the matches and the inliers.
        """
        return LocalMapSummary(
            in_view=len(self.visible),
            matches=int(np.count_nonzero(self.matched >= 0)),
            inliers=self.tracked.inliers,
        )


@dataclass(frozen=True)
class MotionSummary:
    """How tracking the last frame went (``TrackWithMotionModel``), as a run records it.

    Attributes
    ----------
    matches : int
        The keypoints matched in the last window searched.
    widened : bool
        Whether the window was doubled because too few matched in the first.
    inliers : int or None
        The matches the refinement kept; None if too few matched to refine.
    """

    matches: int
    widened: bool
    inliers: int | None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``matches``, ``widened`` and ``inliers``.
        """
        return {"matches": self.matches, "widened": self.widened, "inliers": self.inliers}


@dataclass(frozen=True)
class Motion:
    """Tracking the last frame: the frame, if it was tracked, and how it went.

    Attributes
    ----------
    tracked : slam_it.edge.frames.Tracked or None
        The tracked frame; None if too few points matched or survived refinement.
    summary : MotionSummary
        How it went.
    matched : ndarray of int64, shape (N,)
        The id of the map point each keypoint matched before the pose was refined, -1 where
        none.
    """

    tracked: Tracked | None
    summary: MotionSummary
    matched: IntArray


@dataclass(frozen=True)
class PnpSummary:
    """How the PnP fallback went, as a run records it.

    Each count is None when the fallback stopped before reaching it.

    Attributes
    ----------
    candidates : int
        The local map's points.
    matches : int or None
        The keypoints matched to them by descriptor, with a bearing in front.
    iterations : int or None
        RANSAC's samples, if it found a pose.
    termination : slam_it.foundations.ransac.Termination or None
        Why RANSAC stopped, if it found a pose.
    degenerate : bool
        Whether RANSAC found no pose.
    inliers : int or None
        RANSAC's inliers.
    refined : int or None
        The inliers the refinement kept.
    """

    candidates: int
    matches: int | None = None
    iterations: int | None = None
    termination: ransac.Termination | None = None
    degenerate: bool = False
    inliers: int | None = None
    refined: int | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the summary as JSON values.

        Returns
        -------
        dict
            ``candidates``, ``matches``, ``iterations``, ``termination`` (by name;
            ``"degenerate"`` when RANSAC found no pose), ``inliers`` and ``refined``.
        """
        termination = None if self.termination is None else self.termination.value
        return {
            "candidates": self.candidates,
            "matches": self.matches,
            "iterations": self.iterations,
            "termination": "degenerate" if self.degenerate else termination,
            "inliers": self.inliers,
            "refined": self.refined,
        }


@dataclass(frozen=True)
class PnpRun:
    """A PnP RANSAC run on matched points, as the fallback and relocalisation make it.

    Attributes
    ----------
    pose : ndarray of float64, shape (4, 4), or None
        Its pose, ``T_cam_world``; None if no sample gave one.
    inliers : ndarray of bool, shape (n,), or None
        Which matches are its inliers.
    iterations : int or None
        RANSAC's samples, if it found a pose.
    termination : slam_it.foundations.ransac.Termination or None
        Why it stopped, if it found a pose.
    """

    pose: NDArray[np.float64] | None
    inliers: NDArray[np.bool_] | None = None
    iterations: int | None = None
    termination: ransac.Termination | None = None

    @property
    def count(self) -> int:
        """Its inliers, 0 if it found no pose."""
        return 0 if self.inliers is None else int(np.count_nonzero(self.inliers))

    def recorded(self) -> dict[str, Any]:
        """Return what a stage's summary records of it.

        Returns
        -------
        dict
            ``degenerate`` if it found no pose; else its ``iterations``, ``termination`` and
            ``inliers``.
        """
        if self.pose is None:
            return {"degenerate": True}
        return {
            "iterations": self.iterations,
            "termination": self.termination,
            "inliers": self.count,
        }


def run_pnp(
    xyz: NDArray[np.float64], bearings: NDArray[np.float64], settings: ransac.Options
) -> PnpRun:
    """Run PnP RANSAC on matched points, a run in which no sample gave a pose reported, not raised.

    Parameters
    ----------
    xyz : ndarray of float64, shape (n, 3)
        The points, in the world frame.
    bearings : ndarray of float64, shape (n, 3)
        Their keypoints' bearings.
    settings : slam_it.foundations.ransac.Options
        RANSAC's settings.

    Returns
    -------
    PnpRun
        The run.
    """
    try:
        solved = pnp.pnp_ransac(xyz, bearings, settings)
    except DegenerateError:
        return PnpRun(None)
    run = solved.summary.run
    return PnpRun(solved.state.model, solved.state.inliers, run.iterations, run.termination)


@dataclass(frozen=True)
class Relocalization:
    """The PnP fallback: the frame, if it was placed, and how it went.

    Attributes
    ----------
    tracked : slam_it.edge.frames.Tracked or None
        The frame placed and refined; None if it was not.
    summary : PnpSummary
        How it went.
    matched : ndarray of int64, shape (N,)
        The id of the map point each keypoint matched by descriptor (with a bearing in front),
        -1 where none: all -1 if the fallback stopped before matching.
    """

    tracked: Tracked | None
    summary: PnpSummary
    matched: IntArray


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.tracking`` submodule provides.

    A tracked frame crosses as a dict of ``pose`` and ``points``; :class:`InView` and
    :class:`LocalTracking` as dicts of their fields, the latter's tracked frame flattened into
    ``pose`` and ``points``; :class:`Motion` and :class:`Relocalization` as dicts of
    ``tracked`` (or None), ``matched`` and their summary's fields, RANSAC's termination as its
    code in ``codes()`` (or None).
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def optimize_pose(
        self,
        m: dict[str, Any],
        camera: dict[str, Any],
        frame: dict[str, Any],
        tracked: dict[str, Any],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Refine a frame's pose against its matched points."""
        ...

    def in_view(
        self,
        m: dict[str, Any],
        camera: dict[str, Any],
        pose: NDArray[np.float64],
        rows: NDArray[np.int64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Select the points a camera sees."""
        ...

    def track_last_frame(  # noqa: PLR0913 -- the stage's state, as the reference takes it
        self,
        *,
        m: dict[str, Any],
        camera: dict[str, Any],
        frame: dict[str, Any],
        last: dict[str, Any],
        last_tracked: dict[str, Any],
        predicted: NDArray[np.float64],
        mode: str,
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Track the last frame's points into this one."""
        ...

    def track_local_map(  # noqa: PLR0913 -- the stage's state, as the reference takes it
        self,
        *,
        m: dict[str, Any],
        camera: dict[str, Any],
        frame: dict[str, Any],
        tracked: dict[str, Any],
        mode: str,
        options: dict[str, Any],
        recovering: bool,
    ) -> dict[str, Any]:
        """Match the local map's points in view and refine the pose."""
        ...

    def relocalize_locally(
        self,
        m: dict[str, Any],
        camera: dict[str, Any],
        frame: dict[str, Any],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Place a frame against the local map by descriptors and PnP RANSAC."""
        ...


def _decode_found(raw: dict[str, Any], codes: backends.Codes) -> Tracked:
    """Rebuild a tracked frame that is never None."""
    tracked = frames.decode_tracked(raw, codes)
    assert tracked is not None
    return tracked


def _decode_in_view(raw: dict[str, Any], codes: backends.Codes) -> InView:
    """Rebuild :class:`InView` from a compiled backend's dict."""
    del codes
    return InView(
        rows=backends.checked_array(raw["rows"], np.int64, 1, "in_view rows"),
        xy=backends.checked_array(raw["xy"], np.float64, 2, "in_view xy"),
        right=backends.checked_array(raw["right"], np.float64, 1, "in_view right"),
        level=backends.checked_array(raw["level"], np.int64, 1, "in_view level"),
        view_cos=backends.checked_array(raw["view_cos"], np.float64, 1, "in_view view_cos"),
    )


def _decode_local(raw: dict[str, Any], codes: backends.Codes) -> LocalTracking:
    """Rebuild :class:`LocalTracking` from a compiled backend's dict."""
    return LocalTracking(
        tracked=_decode_found(raw, codes),
        visible=backends.checked_array(raw["visible"], np.int64, 1, "visible ids"),
        found=backends.checked_array(raw["found"], np.int64, 1, "found ids"),
        matched=backends.checked_array(raw["matched"], np.int64, 1, "matched ids"),
    )


def _optional_int(raw: Any) -> int | None:  # noqa: ANN401 -- a compiled backend's value
    """Return a compiled backend's count, or None."""
    return None if raw is None else int(raw)


def _decode_motion(raw: dict[str, Any], codes: backends.Codes) -> Motion:
    """Rebuild :class:`Motion` from a compiled backend's dict."""
    return Motion(
        tracked=frames.decode_tracked(raw["tracked"], codes),
        summary=MotionSummary(
            matches=int(raw["matches"]),
            widened=bool(raw["widened"]),
            inliers=_optional_int(raw["inliers"]),
        ),
        matched=backends.checked_array(raw["matched"], np.int64, 1, "matched ids"),
    )


def _decode_relocalization(raw: dict[str, Any], codes: backends.Codes) -> Relocalization:
    """Rebuild :class:`Relocalization` from a compiled backend's dict."""
    code = raw["termination"]
    termination = (
        None
        if code is None
        else ransac.Termination(backends.decode_name(codes, "ransac_termination", code))
    )
    return Relocalization(
        tracked=frames.decode_tracked(raw["tracked"], codes),
        summary=PnpSummary(
            candidates=int(raw["candidates"]),
            matches=_optional_int(raw["matches"]),
            iterations=_optional_int(raw["iterations"]),
            termination=termination,
            degenerate=bool(raw["degenerate"]),
            inliers=_optional_int(raw["inliers"]),
            refined=_optional_int(raw["refined"]),
        ),
        matched=backends.checked_array(raw["matched"], np.int64, 1, "matched ids"),
    )


def predict(last_pose: Array, velocity: Array | None) -> Array:
    """Predict a frame's pose by the constant-velocity model.

    The product's rotation is projected back onto SO(3): refinement keeps whatever drift its
    starting pose has, and the motion model, inverting by transposing, would otherwise feed
    that drift back to every later frame, compounding it.

    Parameters
    ----------
    last_pose : ndarray of float64, shape (4, 4)
        The last frame's ``T_cam_world``.
    velocity : ndarray of float64, shape (4, 4) or None
        The last motion, ``T_last_previous``; None before there is one (the pose is kept).

    Returns
    -------
    ndarray of float64, shape (4, 4)
        ``velocity @ last_pose``, its rotation the nearest one.
    """
    if velocity is None:
        return last_pose.copy()
    out = velocity @ last_pose
    out[:3, :3] = so3.project_to_so3(out[:3, :3])
    return np.asarray(out, dtype=np.float64)


def _optimize_pose(
    m: Map, camera: Camera, frame: Frame, tracked: Tracked, options: Options
) -> Tracked:
    """Refine a frame's pose against its matched points (reference)."""
    points = tracked.points
    rows = m.point_rows(points)
    matched = (rows >= 0) & (frame.bearings[:, 2] > 0.0)  # NaN bearings fail too
    xyz = np.zeros((frame.size, 3))
    xyz[matched] = m.pt_xyz[rows[matched]]
    keypoints = frame.features.keypoints
    variance = residuals.sigma2(m, keypoints.level[matched])
    limit = residuals.thresholds(frame.right[matched], options)
    # The Cauchy scale: the chi-square bound at level 0, in pixels, in the normalised plane;
    # the level weights whiten each residual to level-0 units, so one scale serves all.
    scale = math.sqrt(options.chi2_mono) / camera.focal
    use = matched.copy()
    current = np.asarray(tracked.pose, dtype=np.float64)
    for _ in range(options.pose_rounds):
        use[np.flatnonzero(use)[maps.to_camera(current, xyz[use])[:, 2] <= 0.0]] = False
        if np.count_nonzero(use) < p3p.SAMPLE_SIZE:  # refine_pose's minimum
            break
        weights = 1.0 / variance[use[matched]]  # each level's information (invSigma2)
        current = pnp.refine_pose(
            current, xyz[use], frame.bearings[use], scale=scale, weights=weights
        ).state
        error = residuals.chi2(
            camera,
            maps.to_camera(current, xyz[matched]),
            keypoints.xy[matched],
            frame.right[matched],
            variance,
        )
        use = matched.copy()
        use[matched] = error < limit
    return Tracked(pose=current, points=np.where(use, points, -1).astype(np.int64))


def optimize_pose(  # noqa: PLR0913 -- the map, its camera, the frame tracked, the settings
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    options: Options,
    *,
    backend: str = "python",
) -> Tracked:
    """Refine a frame's pose against its matched points, dropping outliers (``PoseOptimization``).

    ``pose_rounds`` rounds, each refining the pose on the matches the last one kept (those in
    front of the camera), each weighted by its level's information ``1/s^(2l)``, and
    classifying every match afresh by the chi-square test.

    Parameters
    ----------
    m : Map
        The map (the points' positions).
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The frame.
    tracked : slam_it.edge.frames.Tracked
        The starting ``T_cam_world``, and the point id each keypoint matched (or -1).
    options : Options
        The rounds and chi-square thresholds.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    slam_it.edge.frames.Tracked
        The refined pose and the inlier matches (outliers set to -1). With fewer than three
        usable matches the pose is returned as it was.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the pose is not a finite 4x4 matrix, or the matches are not one per keypoint.
    """
    start = frames.checked_tracked(tracked, frame, "tracked")
    return backends.call(
        backend,
        __name__,
        "optimize_pose",
        reference=lambda: _optimize_pose(m, camera, frame, start, options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "frame": frame.as_dict(),
            "tracked": start.as_dict(),
            "options": options.as_dict(),
        },
        decode=_decode_found,
    )


def _in_view(m: Map, camera: Camera, pose: Array, rows: IntArray, options: Options) -> InView:
    """Select the points a camera sees (reference)."""
    xyz = m.pt_xyz[rows]
    local = maps.to_camera(pose, xyz)
    pixels, keep = camera.project_in_view(local)
    offset = xyz - maps.camera_centre(pose)
    distance = np.linalg.norm(offset, axis=1)
    keep &= maps.in_distance_range(m, rows, distance, options)
    view_cos = maps.viewing_cos(m, rows, offset, np.where(keep, distance, 1.0))
    keep &= view_cos >= options.view_cos_limit
    chosen = rows[keep]
    right = np.full(len(chosen), np.nan)
    if camera.baseline > 0.0:
        right = pixels[keep, 0] - camera.focal * camera.baseline / local[keep, 2]
    return InView(
        rows=chosen,
        xy=pixels[keep],
        right=right,
        level=maps.predict_level(m, chosen, distance[keep]),
        view_cos=view_cos[keep],
    )


def in_view(  # noqa: PLR0913 -- the map, its camera, a pose, the candidates, the settings
    m: Map,
    camera: Camera,
    pose: ArrayLike,
    rows: ArrayLike,
    options: Options,
    *,
    backend: str = "python",
) -> InView:
    """Select the points a camera sees, at a distance and angle they are matchable from.

    A point is in view when it is in front, projects inside the image, lies in its distance
    range (:func:`slam_it.edge.map.in_distance_range`), and is seen within ``view_cos_limit`` of
    its mean viewing direction (ORB-SLAM's ``isInFrustum``).

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    pose : array_like, shape (4, 4)
        Its ``T_cam_world``.
    rows : array_like of int
        The candidate points' rows.
    options : Options
        The viewing-angle limit and the distance margins.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    InView
        The points in view, in the order of `rows`.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the pose is not a finite 4x4 matrix, or a row is out of range.
    """
    matrix = frames.as_pose(pose, "pose")
    given = np.asarray(rows)
    chosen = maps.as_point_rows(m, given)
    return backends.call(
        backend,
        __name__,
        "in_view",
        reference=lambda: _in_view(m, camera, matrix, chosen, options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "pose": matrix,
            "rows": chosen,
            "options": options.as_dict(),
        },
        decode=_decode_in_view,
    )


def _search(
    frame: Frame,
    projections: matching.Projections,
    ids: IntArray,
    options: matching.SearchOptions,
    *,
    taken: Mask,
) -> IntArray:
    """Match projected points (their ids) to the frame; return the id each keypoint matched."""
    found = frames.search(frame, projections, options, taken=taken)
    points = np.full(frame.size, -1, dtype=np.int64)
    points[found.pairs[:, 1]] = ids[found.pairs[:, 0]]
    return points


def _track_last_frame(  # noqa: PLR0913 -- the stage's state
    m: Map,
    camera: Camera,
    frame: Frame,
    last: Frame,
    last_tracked: Tracked,
    *,
    predicted: Array,
    mode: str,
    options: Options,
) -> Motion:
    """Track the last frame's points into this one (reference)."""
    rows = m.point_rows(last_tracked.points)
    have = np.flatnonzero(rows >= 0)
    local = maps.to_camera(predicted, m.pt_xyz[rows[have]])
    pixels, ok = camera.project_in_view(local)
    have, local, pixels = have[ok], local[ok], pixels[ok]
    last_level = last.features.keypoints.level[have]
    low, high = last_level - 1, last_level + 1
    right = np.full(len(have), np.nan)
    window = options.stereo_motion_window if mode == "stereo" else options.motion_window
    if mode != "monocular":
        right = pixels[:, 0] - camera.focal * camera.baseline / local[:, 2]
        # The current camera's centre in the last camera's frame (ORB-SLAM's tlc): moving
        # forward, a point appears larger, so it is found on no finer level than before;
        # moving backward, on no coarser one.
        ahead = last_tracked.pose[2, :3] @ maps.camera_centre(predicted)
        ahead += last_tracked.pose[2, 3]
        if ahead > camera.baseline:
            low, high = last_level, np.full(len(have), m.levels - 1)
        elif -ahead > camera.baseline:
            low, high = np.zeros(len(have), dtype=np.int64), last_level
    searched = matching.SearchOptions(
        max_distance=options.th_high, ratio=1.0, check_orientation=True
    )
    points = np.full(frame.size, -1, dtype=np.int64)
    widened = False
    for factor in (1.0, 2.0):
        widened = factor > 1.0
        projections = matching.Projections(
            xy=pixels,
            radius=np.asarray(factor * window * m.scale_factor**last_level, dtype=np.float64),
            min_level=low,
            max_level=high,
            descriptors=m.pt_descriptor[rows[have]],
            angle=last.features.keypoints.angle[have],
            right=right,
        )
        points = _search(
            frame, projections, m.pt_id[rows[have]], searched, taken=np.zeros(frame.size, bool)
        )
        if np.count_nonzero(points >= 0) >= options.min_motion_matches:
            break
    matches = int(np.count_nonzero(points >= 0))
    if matches < options.min_motion_matches:
        return Motion(tracked=None, summary=MotionSummary(matches, widened, None), matched=points)
    tracked = _optimize_pose(m, camera, frame, Tracked(pose=predicted, points=points), options)
    summary = MotionSummary(matches, widened, tracked.inliers)
    kept = tracked if tracked.inliers >= options.min_motion_inliers else None
    return Motion(kept, summary, matched=points)


def track_last_frame(  # noqa: PLR0913 -- the stage's state, and the backend
    m: Map,
    camera: Camera,
    frame: Frame,
    last: Frame,
    last_tracked: Tracked,
    *,
    predicted: ArrayLike,
    mode: str,
    options: Options,
    backend: str = "python",
) -> Motion:
    """Track the last frame's points into this one (``TrackWithMotionModel``).

    The last frame's inlier points are projected with the predicted pose and searched in a
    window of ``motion_window`` (``stereo_motion_window`` in stereo) pixels at the last
    keypoint's level, on its level and the next ones (with depth: on its level and the
    coarser ones when moving forward, the finer ones when moving back); twice as wide if fewer
    than ``min_motion_matches`` match. The pose is then refined (:func:`optimize_pose`).

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The frame to track.
    last : slam_it.edge.frames.Frame
        The last frame.
    last_tracked : slam_it.edge.frames.Tracked
        Its pose and matches.
    predicted : array_like, shape (4, 4)
        The predicted pose.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The windows and minimums.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Motion
        The tracked frame, none if fewer than ``min_motion_matches`` points matched or fewer
        than ``min_motion_inliers`` survived refinement; the matches, whether the window was
        widened, and the inliers; and each keypoint's match before refinement.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, a pose is not a finite 4x4 matrix, or the last frame's matches
        are not one per keypoint.
    """
    check_mode(mode)
    matrix = frames.as_pose(predicted, "predicted")
    previous = frames.checked_tracked(last_tracked, last, "last_tracked")
    return backends.call(
        backend,
        __name__,
        "track_last_frame",
        reference=lambda: _track_last_frame(
            m, camera, frame, last, previous, predicted=matrix, mode=mode, options=options
        ),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "frame": frame.as_dict(),
            "last": last.as_dict(),
            "last_tracked": previous.as_dict(),
            "predicted": matrix,
            "mode": mode,
            "options": options.as_dict(),
        },
        decode=_decode_motion,
    )


def _track_local_map(  # noqa: PLR0913 -- the stage's state
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    recovering: bool,
) -> LocalTracking:
    """Match the local map's points in view and refine the pose (reference)."""
    local = m.observed_by(m.window(options.window))
    already = m.point_rows(tracked.points)
    already = already[already >= 0]
    view = _in_view(m, camera, tracked.pose, np.setdiff1d(local, already), options)
    if recovering:
        th = options.recovery_local_window
    else:
        th = options.rgbd_local_window if mode == "rgbd" else options.local_window
    narrow = np.where(
        view.view_cos > options.aligned_view_cos,
        options.aligned_view_radius,
        options.oblique_view_radius,
    )
    projections = matching.Projections(
        xy=view.xy,
        radius=th * narrow * m.scale_factor**view.level,
        min_level=view.level - 1,
        max_level=view.level,
        descriptors=m.pt_descriptor[view.rows],
        angle=np.zeros(len(view.rows), dtype=np.int64),
        right=view.right,
    )
    searched = matching.SearchOptions(
        max_distance=options.th_high, ratio=options.local_map_ratio, check_orientation=False
    )
    found = _search(frame, projections, m.pt_id[view.rows], searched, taken=tracked.points >= 0)
    points = np.where(tracked.points >= 0, tracked.points, found)
    refined = _optimize_pose(m, camera, frame, Tracked(pose=tracked.pose, points=points), options)
    return LocalTracking(
        tracked=refined,
        visible=np.concatenate([m.pt_id[already], m.pt_id[view.rows]]),
        found=refined.points[refined.points >= 0],
        matched=points,
    )


def track_local_map(  # noqa: PLR0913 -- the stage's state, the recovery flag, and the backend
    m: Map,
    camera: Camera,
    frame: Frame,
    tracked: Tracked,
    *,
    mode: str,
    options: Options,
    recovering: bool = False,
    backend: str = "python",
) -> LocalTracking:
    """Match the local map's points in view and refine the pose (``TrackLocalMap``).

    The local map is the points the window's keyframes observe. Those the frame has not
    matched yet and that are in view (:func:`in_view`) are searched at their predicted level
    and the one below, in a window of ``local_window`` (``rgbd_local_window`` in RGB-D,
    ``recovery_local_window`` just after a recovery) times ``aligned_view_radius`` or
    ``oblique_view_radius`` pixels at that level (``RadiusByViewingCos``), with the
    ``local_map_ratio`` test.

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The frame.
    tracked : slam_it.edge.frames.Tracked
        Its pose and matches so far.
    mode : {"monocular", "stereo", "rgbd"}
        The pipeline's mode.
    options : Options
        The window, thresholds and minimums.
    recovering : bool, optional
        Whether the frame is a recovery's or the next (``mnLastRelocFrameId + 2``): the
        window is then ``recovery_local_window``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    LocalTracking
        The refined frame, the ids of the points in view and of the inliers, and each
        keypoint's match before refinement.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the mode is unknown, the pose is not a finite 4x4 matrix, the matches are not one
        per keypoint, or `recovering` is not a bool.
    """
    check_mode(mode)
    flag = as_flag(recovering, "recovering")
    so_far = frames.checked_tracked(tracked, frame, "tracked")
    return backends.call(
        backend,
        __name__,
        "track_local_map",
        reference=lambda: _track_local_map(
            m, camera, frame, so_far, mode=mode, options=options, recovering=flag
        ),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "frame": frame.as_dict(),
            "tracked": so_far.as_dict(),
            "mode": mode,
            "options": options.as_dict(),
            "recovering": flag,
        },
        decode=_decode_local,
    )


def _relocalize_locally(m: Map, camera: Camera, frame: Frame, options: Options) -> Relocalization:
    """Place a frame against the local map by descriptors and PnP RANSAC (reference)."""
    local = m.observed_by(m.window(options.window))
    candidates = len(local)
    points = np.full(frame.size, -1, dtype=np.int64)
    if candidates < options.pnp_min_inliers:
        return Relocalization(tracked=None, summary=PnpSummary(candidates), matched=points)
    found = matching.match(
        frame.features.descriptors,
        m.pt_descriptor[local],
        frame.features.keypoints.angle,
        np.zeros(len(local), dtype=np.int64),
        matching.Options(
            max_distance=options.th_high, ratio=options.relocalize_ratio, check_orientation=False
        ),
    )
    pairs = found.pairs[frame.bearings[found.pairs[:, 0], 2] > 0.0]
    points[pairs[:, 0]] = m.pt_id[local[pairs[:, 1]]]
    matched = PnpSummary(candidates, matches=len(pairs))
    if len(pairs) < options.pnp_min_inliers:
        return Relocalization(tracked=None, summary=matched, matched=points)
    settings = ransac.Options(threshold=options.pnp_threshold / camera.focal, seed=options.seed)
    run = run_pnp(m.pt_xyz[local[pairs[:, 1]]], frame.bearings[pairs[:, 0]], settings)
    sampled = replace(matched, **run.recorded())
    if run.pose is None or run.inliers is None or run.count < options.pnp_min_inliers:
        return Relocalization(tracked=None, summary=sampled, matched=points)
    sample = np.full(frame.size, -1, dtype=np.int64)
    sample[pairs[run.inliers, 0]] = m.pt_id[local[pairs[run.inliers, 1]]]
    tracked = _optimize_pose(m, camera, frame, Tracked(pose=run.pose, points=sample), options)
    summary = replace(sampled, refined=tracked.inliers)
    kept = tracked if tracked.inliers >= options.pnp_min_inliers else None
    return Relocalization(kept, summary, matched=points)


def relocalize_locally(
    m: Map, camera: Camera, frame: Frame, options: Options, *, backend: str = "python"
) -> Relocalization:
    """Place a frame against the local map by descriptors and PnP RANSAC (the fallback).

    The frame's descriptors are matched to the local map's points' (``th_high``, the
    ``relocalize_ratio`` test), PnP RANSAC places it on the matches (``pnp_threshold``
    pixels, ``seed``), and the pose is refined (:func:`optimize_pose`). It recovers only near
    the local map: ORB-SLAM has no such step, relocalising against the keyframes its bag of
    words finds, as :func:`slam_it.edge.relocalization.relocalize` does after it.

    Parameters
    ----------
    m : Map
        The map.
    camera : slam_it.edge.frames.Camera
        The camera.
    frame : slam_it.edge.frames.Frame
        The frame.
    options : Options
        The RANSAC threshold, minimum inliers and seed.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    Relocalization
        The frame placed and refined, none if fewer than ``pnp_min_inliers`` points match,
        RANSAC finds no pose, or fewer inliers survive; how far it got
        (:class:`PnpSummary`); and each keypoint's match by descriptor.
    """
    return backends.call(
        backend,
        __name__,
        "relocalize_locally",
        reference=lambda: _relocalize_locally(m, camera, frame, options),
        arguments={
            "m": m.as_dict(),
            "camera": camera.as_dict(),
            "frame": frame.as_dict(),
            "options": options.as_dict(),
        },
        decode=_decode_relocalization,
    )
