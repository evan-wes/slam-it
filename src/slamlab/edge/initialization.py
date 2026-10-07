"""Initialisation: starting the map, from depth or from two views.

ORB-SLAM's ``StereoInitialization`` and ``MonocularInitialization``:

- **Stereo and RGB-D** (:func:`initialize_from_depth`): the first frame with enough keypoints
  with depth is the first keyframe, at the origin, a point created at every one of them, so
  the map is metric from the start.
- **Monocular** (:func:`initialize_monocular`): a reference frame and a later one are matched
  within a window (:func:`match_for_initialization`, ``SearchForInitialization``) and
  initialised by :func:`slamlab.foundations.twoview.initialize`; the two become keyframes, a
  bundle adjustment refines them, and the map is scaled so the first keyframe's median
  depth is 1 (a monocular map's scale is its own). The pipeline extracts these frames with
  more features (:meth:`slamlab.edge.options.Options.extraction`), and keeps its reference
  until too few keypoints match it (:class:`slamlab.edge.pipeline.Pipeline`).
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.edge import adjust, frames
from slamlab.edge import map as maps
from slamlab.edge.frames import Camera, Frame, Tracked
from slamlab.edge.map import Map, frame_dict
from slamlab.edge.options import TH_LOW, Options
from slamlab.foundations import matching, twoview
from slamlab.foundations.errors import DegenerateError, InvalidInputError

Array = NDArray[np.float64]

INIT_BA_ITERATIONS = 20
"""The monocular map's first bundle adjustment's iterations (ORB-SLAM's
``GlobalBundleAdjustemnt(map, 20)``)."""


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
        self, reference: dict[str, Any], frame: dict[str, Any], options: dict[str, Any]
    ) -> dict[str, Any]:
        """Match a monocular reference frame to a later one."""
        ...

    def initialize_monocular(
        self,
        camera: dict[str, Any],
        reference: dict[str, Any],
        frame: dict[str, Any],
        options: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Start the map from two monocular frames."""
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
    if frame.size < options.min_init_features_depth:
        return None
    has_depth = np.flatnonzero(~np.isnan(frame.depth) & ~np.isnan(frame.bearings[:, 0]))
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
    frame : slamlab.edge.frames.Frame
        The frame.
    options : Options
        The minimum keypoints, and the pyramid.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    tuple of (Map, Tracked) or None
        The map, its one keyframe at the origin with a point at every keypoint with depth
        (and a bearing), and the frame tracked; None if the frame has fewer than
        ``min_init_features_depth`` keypoints.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the frame's pyramid is not the settings'.
    """
    _check_pyramid(frame, options, "frame")
    return backends.call(
        backend,
        __name__,
        "initialize_from_depth",
        reference=lambda: _initialize_from_depth(frame, options),
        arguments={"frame": frame_dict(frame), "options": options.as_dict()},
        decode=_decode_started,
    )


def _match_for_initialization(reference: Frame, frame: Frame, options: Options) -> matching.Matches:
    """Match a monocular reference frame to a later one (reference)."""
    keypoints = reference.features.keypoints
    rows = np.flatnonzero(keypoints.level == 0)
    n = len(rows)
    projections = matching.Projections(
        xy=keypoints.xy[rows],
        radius=np.full(n, options.init_window),
        min_level=np.zeros(n, dtype=np.int64),
        max_level=np.zeros(n, dtype=np.int64),
        descriptors=reference.features.descriptors[rows],
        angle=keypoints.angle[rows],
        right=np.full(n, np.nan),
    )
    searched = matching.SearchOptions(
        max_distance=TH_LOW, ratio=options.init_ratio, check_orientation=True
    )
    found = frames.search(frame, projections, searched)
    return matching.Matches(
        pairs=np.column_stack([rows[found.pairs[:, 0]], found.pairs[:, 1]]).astype(np.int64),
        distances=found.distances,
    )


def match_for_initialization(
    reference: Frame, frame: Frame, options: Options, *, backend: str = "python"
) -> matching.Matches:
    """Match a monocular reference frame to a later one (``SearchForInitialization``).

    Each level-0 keypoint of the reference is searched at its own position, within
    ``init_window``, among the later frame's level-0 keypoints, with ORB-SLAM's stricter
    distance, its 0.9 ratio test and the rotation check.

    Parameters
    ----------
    reference : slamlab.edge.frames.Frame
        The reference frame.
    frame : slamlab.edge.frames.Frame
        The later frame.
    options : Options
        The window and ratio.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    slamlab.foundations.matching.Matches
        ``(reference keypoint, frame keypoint)`` pairs, by reference keypoint.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If a frame's pyramid is not the settings'.
    """
    _check_pyramid(reference, options, "reference")
    _check_pyramid(frame, options, "frame")
    return backends.call(
        backend,
        __name__,
        "match_for_initialization",
        reference=lambda: _match_for_initialization(reference, frame, options),
        arguments={
            "reference": frame_dict(reference),
            "frame": frame_dict(frame),
            "options": options.as_dict(),
        },
        decode=matching.decode_matches,
    )


def _initialize_monocular(
    camera: Camera, reference: Frame, frame: Frame, options: Options
) -> tuple[Map, Tracked] | None:
    """Start the map from two monocular frames (reference)."""
    found = _match_for_initialization(reference, frame, options)
    valid = ~np.isnan(reference.bearings[found.pairs[:, 0], 0]) & ~np.isnan(
        frame.bearings[found.pairs[:, 1], 0]
    )
    pairs = found.pairs[valid]
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
    good = np.flatnonzero(result.triangulated)
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
    m = maps.add_points(m, result.points[good], observers)
    m = adjust.bundle_adjust(
        m,
        camera,
        np.arange(2, dtype=np.int64),
        np.array([True, False]),
        replace(options, ba_iterations=INIT_BA_ITERATIONS),
    )
    if m.points < options.init_min_matches:
        return None  # ORB-SLAM resets with fewer than 100 tracked points
    first = m.kf_pose[0]
    median = float(np.median(m.pt_xyz @ first[2, :3] + first[2, 3]))
    if not median > 0.0:
        return None
    m = _scaled(m, 1.0 / median)
    return m, Tracked(pose=m.kf_pose[1].copy(), points=m.kp_point[m.keypoint_rows(1)].copy())


def initialize_monocular(
    camera: Camera, reference: Frame, frame: Frame, options: Options, *, backend: str = "python"
) -> tuple[Map, Tracked] | None:
    """Start the map from two monocular frames (``MonocularInitialization``).

    The two frames are matched (:func:`match_for_initialization`) and initialised by
    :func:`slamlab.foundations.twoview.initialize`; they become keyframes, a bundle adjustment
    of :data:`INIT_BA_ITERATIONS` refines them with the first held fixed, and the map is
    scaled so the first keyframe's median depth is 1.

    Parameters
    ----------
    camera : slamlab.edge.frames.Camera
        The camera.
    reference : slamlab.edge.frames.Frame
        The reference frame (the first keyframe, at the origin).
    frame : slamlab.edge.frames.Frame
        The later frame.
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
    slamlab.foundations.errors.InvalidInputError
        If a frame's pyramid is not the settings'.
    """
    _check_pyramid(reference, options, "reference")
    _check_pyramid(frame, options, "frame")
    return backends.call(
        backend,
        __name__,
        "initialize_monocular",
        reference=lambda: _initialize_monocular(camera, reference, frame, options),
        arguments={
            "camera": camera.as_dict(),
            "reference": frame_dict(reference),
            "frame": frame_dict(frame),
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
