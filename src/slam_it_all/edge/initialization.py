"""Initialisation: starting the map, from depth or from two views.

ORB-SLAM's ``StereoInitialization`` and ``MonocularInitialization``:

- **Stereo and RGB-D** (:func:`initialize_from_depth`): the first frame with more than
  ``min_init_features_depth`` keypoints, ``min_init_depth_points`` of them with depth, is the
  first keyframe, at the origin, with a point at every keypoint with depth, so the map is
  metric from the start.
- **Monocular** (:func:`initialize_monocular`): a reference frame and a later one are matched
  within a window (:func:`match_for_initialization`, ``SearchForInitialization``) and
  initialised by :func:`slam_it_all.foundations.twoview.initialize`; the two become keyframes, a
  bundle adjustment refines them, and the map is scaled so the first keyframe's median
  depth is 1 (a monocular map's scale is its own). The pipeline extracts these frames with
  more features (:meth:`slam_it_all.edge.options.Options.extraction`), and keeps its reference
  until too few keypoints match it (:class:`slam_it_all.edge.pipeline.Pipeline`).
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it_all import backends
from slam_it_all.edge import adjust, frames
from slam_it_all.edge import map as maps
from slam_it_all.edge.frames import Camera, Frame, Tracked
from slam_it_all.edge.map import Map
from slam_it_all.edge.options import Options
from slam_it_all.foundations import matching, triangulation, twoview
from slam_it_all.foundations.arrays import as_indices, as_real
from slam_it_all.foundations.errors import DegenerateError, InvalidInputError

Array = NDArray[np.float64]
IntArray = NDArray[np.int64]


class Implementation(Protocol):
    """The functions a compiled backend's ``edge.initialization`` submodule provides.

    A started map crosses back as a dict of ``map`` (as :meth:`Map.as_dict`), ``pose`` and
    ``points`` (the tracked frame), or None.
    """

    def initialize_from_depth(
        self, frame: dict[str, Any], options: dict[str, Any]
    ) -> dict[str, Any] | None:
        """Start the map from a stereo or RGB-D frame."""
        ...

    def match_for_initialization(
        self,
        reference: dict[str, Any],
        frame: dict[str, Any],
        previous: NDArray[np.float64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Match a monocular reference frame to a later one, where each was last matched."""
        ...

    def initialize_monocular(
        self,
        camera: dict[str, Any],
        reference: dict[str, Any],
        frame: dict[str, Any],
        pairs: NDArray[np.int64],
        options: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Start the map from two monocular frames and their matches."""
        ...


def _check_pyramid(frame: Frame, options: Options, name: str) -> None:
    """Refuse a frame whose pyramid is not the settings'."""
    if frame.features.levels != options.levels or (
        frame.features.scale_factor != options.scale_factor
    ):
        raise InvalidInputError(f"{name}: its pyramid must be the settings'")


def _decode_started(
    raw: dict[str, Any] | None, codes: backends.Codes
) -> tuple[Map, Tracked] | None:
    """Rebuild a started map and its tracked frame from a compiled backend's dict."""
    if raw is None:
        return None
    pose = backends.checked_array(raw["pose"], np.float64, 2, "tracked pose")
    points = backends.checked_array(raw["points"], np.int64, 1, "tracked points")
    return maps.decode(raw["map"], codes), Tracked(pose=pose, points=points)


def _initialize_from_depth(frame: Frame, options: Options) -> tuple[Map, Tracked] | None:
    """Start the map from a stereo or RGB-D frame (reference)."""
    if frame.size <= options.min_init_features_depth:
        return None
    has_depth = np.flatnonzero(~np.isnan(frame.depth) & ~np.isnan(frame.bearings[:, 0]))
    if len(has_depth) < options.min_init_depth_points:
        return None
    pose = np.eye(4)
    m = maps.insert_keyframe(
        maps.empty(options.levels, options.scale_factor),
        frame,
        pose,
        np.full(frame.size, -1, dtype=np.int64),
    )
    bearings = frame.bearings[has_depth]
    xyz = bearings * (frame.depth[has_depth] / bearings[:, 2])[:, None]
    m = maps.add_points(m, xyz, has_depth[:, None].astype(np.int64))
    points = np.full(frame.size, -1, dtype=np.int64)
    points[has_depth] = m.pt_id
    return m, Tracked(pose=pose, points=points)


def initialize_from_depth(
    frame: Frame, options: Options, *, backend: str = "python"
) -> tuple[Map, Tracked] | None:
    """Start the map from a stereo or RGB-D frame (``StereoInitialization``).

    Parameters
    ----------
    frame : slam_it_all.edge.frames.Frame
        The frame.
    options : Options
        The minimum keypoints, and the pyramid.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    tuple of (Map, Tracked) or None
        The map, its one keyframe at the origin with a point at every keypoint with depth
        (and a bearing), and the frame tracked; None unless the frame has more than
        ``min_init_features_depth`` keypoints and ``min_init_depth_points`` with depth.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If the frame's pyramid is not the settings'.
    """
    _check_pyramid(frame, options, "frame")
    return backends.call(
        backend,
        __name__,
        "initialize_from_depth",
        reference=lambda: _initialize_from_depth(frame, options),
        arguments={"frame": frame.as_dict(), "options": options.as_dict()},
        decode=_decode_started,
    )


def _match_for_initialization(
    reference: Frame, frame: Frame, previous: Array, options: Options
) -> matching.Matches:
    """Match a monocular reference frame to a later one (reference)."""
    keypoints = reference.features.keypoints
    rows = np.flatnonzero(keypoints.level == 0)
    n = len(rows)
    projections = matching.Projections(
        xy=previous[rows],
        radius=np.full(n, options.init_window),
        min_level=np.zeros(n, dtype=np.int64),
        max_level=np.zeros(n, dtype=np.int64),
        descriptors=reference.features.descriptors[rows],
        angle=keypoints.angle[rows],
        right=np.full(n, np.nan),
    )
    searched = matching.SearchOptions(
        max_distance=options.th_low, ratio=options.init_ratio, check_orientation=True
    )
    found = frames.search(frame, projections, searched)
    return matching.Matches(
        pairs=np.column_stack([rows[found.pairs[:, 0]], found.pairs[:, 1]]).astype(np.int64),
        distances=found.distances,
    )


def match_for_initialization(
    reference: Frame,
    frame: Frame,
    previous: ArrayLike,
    options: Options,
    *,
    backend: str = "python",
) -> matching.Matches:
    """Match a monocular reference frame to a later one (``SearchForInitialization``).

    Each level-0 keypoint of the reference is searched where it was last matched
    (``previous``, ORB-SLAM's ``vbPrevMatched``: the caller moves a keypoint's position to its
    match's after each search, so the window follows the motion), within ``init_window``,
    among the later frame's level-0 keypoints, with ORB-SLAM's stricter distance, its 0.9
    ratio test and the rotation check.

    Parameters
    ----------
    reference : slam_it_all.edge.frames.Frame
        The reference frame.
    frame : slam_it_all.edge.frames.Frame
        The later frame.
    previous : array_like, shape (N, 2)
        Where each of the reference's N keypoints was last matched, in pixels (its own
        position until it is matched).
    options : Options
        The window and ratio.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    slam_it_all.foundations.matching.Matches
        ``(reference keypoint, frame keypoint)`` pairs, by reference keypoint.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If a frame's pyramid is not the settings', or `previous` is not one finite pixel per
        reference keypoint.
    """
    _check_pyramid(reference, options, "reference")
    _check_pyramid(frame, options, "frame")
    positions = as_real(previous, "previous")
    if positions.shape != (reference.size, 2) or not np.all(np.isfinite(positions)):
        raise InvalidInputError(
            f"previous: expected ({reference.size}, 2) finite pixels, got {positions.shape}"
        )
    return backends.call(
        backend,
        __name__,
        "match_for_initialization",
        reference=lambda: _match_for_initialization(reference, frame, positions, options),
        arguments={
            "reference": reference.as_dict(),
            "frame": frame.as_dict(),
            "previous": positions,
            "options": options.as_dict(),
        },
        decode=matching.decode_matches,
    )


def _kept_points(
    result: twoview.Initialization, bearings1: Array, bearings2: Array, options: Options
) -> tuple[IntArray, Array]:
    """Return the matches a monocular start keeps, and their points (``CheckRT``).

    The chosen motion's inliers in front of both views whose rays' cosine is below
    ``init_parallax_cos``: two-view's own triangulated points need its larger parallax, which
    only decides whether the views initialise.
    """
    inliers = np.flatnonzero(result.inliers)
    found = triangulation.two_view(result.relative_pose, bearings1[inliers], bearings2[inliers])
    keep = found.in_front & (np.cos(found.parallax) < options.init_parallax_cos)
    return inliers[keep], found.points[keep]


def _initialize_monocular(
    camera: Camera, reference: Frame, frame: Frame, matched: IntArray, options: Options
) -> tuple[Map, Tracked] | None:
    """Start the map from two monocular frames (reference)."""
    valid = ~np.isnan(reference.bearings[matched[:, 0], 0]) & ~np.isnan(
        frame.bearings[matched[:, 1], 0]
    )
    pairs = matched[valid]
    if len(pairs) < options.init_min_matches:
        return None
    try:
        result = twoview.initialize(
            reference.bearings[pairs[:, 0]],
            frame.bearings[pairs[:, 1]],
            twoview.Options(focal=camera.focal, seed=options.seed),
        )
    except DegenerateError:
        return None
    if result.status is not twoview.Status.INITIALIZED:
        return None
    good, xyz = _kept_points(
        result, reference.bearings[pairs[:, 0]], frame.bearings[pairs[:, 1]], options
    )
    m = maps.insert_keyframe(
        maps.empty(options.levels, options.scale_factor),
        reference,
        np.eye(4),
        np.full(reference.size, -1, dtype=np.int64),
    )
    m = maps.insert_keyframe(
        m, frame, result.relative_pose, np.full(frame.size, -1, dtype=np.int64)
    )
    second = int(m.kf_start[1])
    observers = np.column_stack([pairs[good, 0], second + pairs[good, 1]]).astype(np.int64)
    m = maps.add_points(m, xyz, observers)
    m = adjust.bundle_adjust(
        m,
        camera,
        np.arange(2, dtype=np.int64),
        np.array([True, False]),
        replace(options, ba_iterations=options.init_ba_iterations),
    )
    if m.points < options.init_min_matches:
        return None  # ORB-SLAM resets with fewer than 100 tracked points
    first = m.kf_pose[0]
    median = float(np.median(m.pt_xyz @ first[2, :3] + first[2, 3]))
    if not median > 0.0:
        return None
    m = _scaled(m, 1.0 / median)
    return m, Tracked(pose=m.kf_pose[1].copy(), points=m.kp_point[m.keypoint_rows(1)].copy())


def initialize_monocular(  # noqa: PLR0913 -- the two frames, their matches, the settings
    camera: Camera,
    reference: Frame,
    frame: Frame,
    matches: matching.Matches,
    options: Options,
    *,
    backend: str = "python",
) -> tuple[Map, Tracked] | None:
    """Start the map from two monocular frames and their matches (``MonocularInitialization``).

    The matches (:func:`match_for_initialization`) are initialised by
    :func:`slam_it_all.foundations.twoview.initialize`. The chosen motion's inliers in front of
    both views with rays whose cosine is below ``init_parallax_cos`` become points
    (``CheckRT``); the frames become keyframes, a bundle adjustment of
    ``init_ba_iterations`` refines them with the first held fixed, and the map is scaled
    so the first keyframe's median depth is 1.

    Parameters
    ----------
    camera : slam_it_all.edge.frames.Camera
        The camera.
    reference : slam_it_all.edge.frames.Frame
        The reference frame (the first keyframe, at the origin).
    frame : slam_it_all.edge.frames.Frame
        The later frame.
    matches : slam_it_all.foundations.matching.Matches
        ``(reference keypoint, frame keypoint)`` pairs.
    options : Options
        The window, the minimum matches, the seed, and the pyramid.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    tuple of (Map, Tracked) or None
        The map of two keyframes and their triangulated points, and the later frame tracked;
        None if fewer than ``init_min_matches`` keypoints match, the two views do not
        initialise, or fewer than ``init_min_matches`` points survive the adjustment.

    Raises
    ------
    slam_it_all.foundations.errors.InvalidInputError
        If a frame's pyramid is not the settings', or a pair is not a keypoint of each frame.
    """
    _check_pyramid(reference, options, "reference")
    _check_pyramid(frame, options, "frame")
    pairs = np.asarray(matches.pairs)
    if pairs.ndim != 2 or pairs.shape[1] != 2:
        raise InvalidInputError(f"matches.pairs: expected shape (M, 2), got {pairs.shape}")
    pairs = np.column_stack(
        [
            as_indices(pairs[:, 0], len(pairs), reference.size, "matches.pairs"),
            as_indices(pairs[:, 1], len(pairs), frame.size, "matches.pairs"),
        ]
    ).astype(np.int64)
    return backends.call(
        backend,
        __name__,
        "initialize_monocular",
        reference=lambda: _initialize_monocular(camera, reference, frame, pairs, options),
        arguments={
            "camera": camera.as_dict(),
            "reference": reference.as_dict(),
            "frame": frame.as_dict(),
            "pairs": pairs,
            "options": options.as_dict(),
        },
        decode=_decode_started,
    )


def _scaled(m: Map, factor: float) -> Map:
    """Scale the map's translations and points (a monocular map's gauge)."""
    poses = m.kf_pose.copy()
    poses[:, :3, 3] *= factor
    scaled = replace(m, kf_pose=poses, pt_xyz=m.pt_xyz * factor)
    return maps.refresh_points(scaled, np.arange(scaled.points, dtype=np.int64))
