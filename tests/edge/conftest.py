"""The edge rung's test states: a synthetic room, its truth map, and the pipeline's runs.

Edge parity states (maps of hundreds of points, keyframes of hundreds of keypoints) would run
to megabytes as stored fixtures, so they are generated here, deterministically: the room is
rendered (:mod:`tests.edge.room`), its frames built in RGB-D, and a map assembled from the
true poses with the map's own operations (:func:`truth_map`), independent of the pipeline.
Each stage's parity test runs every backend on these states and compares with the Python
reference under :data:`tests.edge.support.TOLERANCE`. The pipeline's own tests run the whole
room in each mode (:func:`run_room`).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from slamlab.edge import frames, pipeline
from slamlab.edge import map as maps
from slamlab.edge.map import Map
from slamlab.foundations import matching
from tests.conftest import require_backend
from tests.edge import room
from tests.edge.support import CAMERA, FRAMES, OPTIONS

KEYFRAMES = (0, 12, 24, 36)
"""The frames the truth map makes keyframes."""

MONOCULAR_PAIR = (0, 10)
"""The frames the monocular initialisation's tests start the map from."""

MATCH_PIXELS = 2.0
"""A point observed by a later keyframe projects within this of the keypoint."""

MATCH_BITS = 50
"""...and its descriptor is within this Hamming distance."""


@dataclass(frozen=True)
class Truth:
    """The room's frames in RGB-D, their true poses, and a map built from them."""

    camera: frames.Camera
    frames: list[frames.Frame]
    poses: list[np.ndarray]
    map: Map


def _observe(m: Map, frame: frames.Frame, pose: np.ndarray) -> np.ndarray:
    """Return the point each keypoint sees: the nearest projection within the limits."""
    points = np.full(frame.size, -1, dtype=np.int64)
    local = m.pt_xyz @ pose[:3, :3].T + pose[:3, 3]
    front = np.flatnonzero(local[:, 2] > 0.1)
    if len(front) == 0:
        return points
    pixels = CAMERA.project(local[front])
    xy = frame.features.keypoints.xy
    gap = np.linalg.norm(pixels[:, None, :] - xy[None, :, :], axis=2)
    bits = matching.distances(m.pt_descriptor[front], frame.features.descriptors)
    gap = np.where(bits <= MATCH_BITS, gap, np.inf)
    taken = np.zeros(frame.size, dtype=bool)
    for i in np.argsort(gap.min(axis=1), kind="stable"):
        k = int(np.argmin(np.where(taken, np.inf, gap[i])))
        if gap[i, k] < MATCH_PIXELS and not taken[k]:
            points[k] = m.pt_id[front[i]]
            taken[k] = True
    return points


def _points_from_depth(m: Map, pose: np.ndarray) -> Map:
    """Add a point at every second keypoint of the newest keyframe that has depth and sees none.

    The others stay free, for the operations that need free keypoints (creating points,
    triangulating).
    """
    rows = np.arange(m.kf_start[-2], m.kf_start[-1])
    free = rows[(m.kp_point[rows] < 0) & ~np.isnan(m.kp_depth[rows])][::2]
    bearings = m.kp_bearing[free]
    local = bearings * (m.kp_depth[free] / bearings[:, 2])[:, None]
    world = (local - pose[:3, 3]) @ pose[:3, :3]
    return maps.add_points(m, world, free[:, None])


def truth_map(built: list[frames.Frame], poses: list[np.ndarray]) -> Map:
    """Build a map of the room from its true poses, with the map's own operations.

    Parameters
    ----------
    built : list of slamlab.edge.frames.Frame
        The frames, in RGB-D.
    poses : list of ndarray, shape (4, 4)
        Their true ``T_cam_world``.

    Returns
    -------
    Map
        Keyframes at :data:`KEYFRAMES`, each observing the earlier points it sees and adding
        points from its own depth at every second free keypoint.
    """
    m = maps.empty(OPTIONS.levels, OPTIONS.scale_factor)
    for i in KEYFRAMES:
        observed = _observe(m, built[i], poses[i])
        m = maps.insert_keyframe(m, built[i], poses[i], observed)
        m = _points_from_depth(m, poses[i])
    return m


@pytest.fixture(scope="session")
def rendered() -> list[room.Frame]:
    """Render the room (left views only)."""
    return room.sequence(FRAMES)


@pytest.fixture(scope="session")
def rendered_stereo(rendered: list[room.Frame]) -> list[room.Frame]:
    """Render the room with its right views too."""
    return room.with_right_views(rendered)


@pytest.fixture(scope="session")
def truth(rendered: list[room.Frame]) -> Truth:
    """Build the room in RGB-D, and its map from the true poses."""
    extraction = OPTIONS.extraction()
    built = [frames.rgbd(f.gray, f.depth, f.timestamp, CAMERA, extraction) for f in rendered]
    poses = [np.linalg.inv(f.T_world_cam) for f in rendered]
    return Truth(CAMERA, built, poses, truth_map(built, poses))


@pytest.fixture(scope="session")
def monocular_pair(rendered: list[room.Frame]) -> tuple[frames.Frame, frames.Frame]:
    """Build :data:`MONOCULAR_PAIR` as monocular frames, with initialisation's features."""
    extraction = OPTIONS.extraction(initializing=True)
    first, second = (rendered[i] for i in MONOCULAR_PAIR)
    return (
        frames.monocular(first.gray, first.timestamp, CAMERA, extraction),
        frames.monocular(second.gray, second.timestamp, CAMERA, extraction),
    )


@dataclass(frozen=True)
class Run:
    """The room run through the Python pipeline in one mode."""

    camera: frames.Camera
    results: list[pipeline.FrameResult]
    truth: list[np.ndarray]
    pipeline: pipeline.Pipeline


def run_room(mode: str, rendered: list[room.Frame]) -> Run:
    """Run the rendered room through the Python pipeline, its frames built by the C++ core.

    The C++ core extracts the features to keep the session fast (rung 0 holds it to Python's
    ORB); a checkout without it skips, or fails where it is required.

    Parameters
    ----------
    mode : {"monocular", "stereo", "rgbd"}
        The mode; stereo needs the right views rendered.
    rendered : list of tests.edge.room.Frame
        The room.

    Returns
    -------
    Run
        Each frame's result and true ``T_world_cam``, and the pipeline after the last.
    """
    require_backend("cpp")
    baseline = 0.0 if mode == "monocular" else room.BASELINE
    camera = frames.Camera(
        "pinhole", room.SMALL.intrinsics, room.SMALL.width, room.SMALL.height, baseline
    )
    run = pipeline.Pipeline(mode, camera, OPTIONS)
    results = []
    for f in rendered:
        if mode == "monocular":
            frame = frames.monocular(f.gray, f.timestamp, camera, run.extraction(), backend="cpp")
        elif mode == "rgbd":
            frame = frames.rgbd(
                f.gray, f.depth, f.timestamp, camera, run.extraction(), backend="cpp"
            )
        else:
            assert f.right is not None
            frame = frames.stereo_pair(
                f.gray, f.right, f.timestamp, camera, run.extraction(), backend="cpp"
            )
        results.append(run.track(frame))
    return Run(camera, results, [f.T_world_cam for f in rendered], run)


@pytest.fixture(scope="session")
def runs(rendered: list[room.Frame], rendered_stereo: list[room.Frame]) -> dict[str, Run]:
    """Run the room in every mode."""
    return {
        "monocular": run_room("monocular", rendered),
        "stereo": run_room("stereo", rendered_stereo),
        "rgbd": run_room("rgbd", rendered),
    }
