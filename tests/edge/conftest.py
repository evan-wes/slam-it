"""The edge rung's test states: a synthetic room, its truth map, and the pipeline's runs.

Edge parity states (maps of hundreds of points, keyframes of hundreds of keypoints) would run
to megabytes as stored fixtures, so they are generated here, deterministically: the room is
rendered (:mod:`tests.edge.room`), its frames built in RGB-D, and a map assembled from the
true poses with the map's own operations (:func:`truth_map`), independent of the pipeline.
Each stage's parity test runs every backend on these states and compares with the Python
reference under :data:`tests.edge.support.TOLERANCE`. The pipeline's own tests run the whole
room in each mode (:func:`run_room`).

Loop closing's states come from the loop room (:func:`tests.edge.room.loop_path`), whose camera
comes back round to where it began: a map of every :data:`LOOP_EVERY`-th frame built from the
true poses as :func:`truth_map` builds one, but each keyframe observing only the points of the
:data:`LOOP_WINDOW` keyframes before it, as local mapping searches only a keyframe's neighbours,
so the end of the loop makes its own points again where the start's are; each observation
then moved to where its point projects (:func:`exact_observations`), so the true poses are the
best fit to them; then drifted (:func:`loop_drift`), each keyframe and the points it made
carried by a drift that grows round the loop, so the loop's end is out of place and a loop
closure has something to correct.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
import pytest

from slam_it.edge import covisibility, frames, pipeline
from slam_it.edge import map as maps
from slam_it.edge.database import Database
from slam_it.edge.map import Map
from slam_it.eval import vocabularies
from slam_it.foundations import matching, so3, vocabulary
from tests.conftest import require_backend
from tests.edge import room
from tests.edge.support import CAMERA, FRAMES, OPTIONS, words_for

KEYFRAMES = (0, 12, 24, 36)
"""The frames the truth map makes keyframes."""

MONOCULAR_PAIR = (0, 10)
"""The frames the monocular initialisation's tests start the map from."""

MATCH_PIXELS = 2.0
"""A point observed by a later keyframe projects within this of the keypoint."""

MATCH_BITS = 50
"""...and its descriptor is within this Hamming distance."""

LOOP_EVERY = 10
"""The loop map makes every this many frames of the loop room a keyframe."""

LOOP_WINDOW = 4
"""A loop-map keyframe observes the points of only this many keyframes before it."""

LOOP_YAW = math.radians(4.0)
"""The drift's yaw by the loop's end."""

LOOP_SHIFT = (0.06, 0.02, -0.04)
"""The drift's shift by the loop's end, in metres."""

LOOP_SCALE = 1.08
"""The drift's scale by the loop's end, when it drifts in scale (monocular)."""


@dataclass(frozen=True)
class Truth:
    """The room's frames in RGB-D, their true poses, and a map built from them."""

    camera: frames.Camera
    frames: list[frames.Frame]
    poses: list[np.ndarray]
    map: Map


def _observe(
    m: Map, frame: frames.Frame, pose: np.ndarray, among: np.ndarray | None = None
) -> np.ndarray:
    """Return the point each keypoint sees: the nearest projection within the limits.

    Only the points `among` marks are looked for (default: all).
    """
    points = np.full(frame.size, -1, dtype=np.int64)
    local = m.pt_xyz @ pose[:3, :3].T + pose[:3, 3]
    eligible = np.ones(m.points, dtype=bool) if among is None else among
    front = np.flatnonzero((local[:, 2] > 0.1) & eligible)
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
    built : list of slam_it.edge.frames.Frame
        The frames, in RGB-D.
    poses : list of ndarray, shape (4, 4)
        Their true ``T_cam_world``.

    Returns
    -------
    Map
        Keyframes at :data:`KEYFRAMES`, each observing the earlier points it sees and adding
        points from its own depth at every second free keypoint.
    """
    m = maps.empty(OPTIONS.features.levels, OPTIONS.features.scale_factor)
    for i in KEYFRAMES:
        observed = _observe(m, built[i], poses[i])
        m = maps.insert_keyframe(m, built[i], poses[i], observed)
        m = _points_from_depth(m, poses[i])
    return m


def loop_drift(fraction: float, *, scaled: bool) -> np.ndarray:
    """Return the drift of a keyframe a fraction of the way round the loop.

    A similarity on the world, ``[[s R, t], [0, 1]]``: :data:`LOOP_YAW`, :data:`LOOP_SHIFT`
    and, when `scaled`, :data:`LOOP_SCALE`, each grown in proportion from none at the start.

    Parameters
    ----------
    fraction : float
        How far round, from 0 to 1.
    scaled : bool
        Drift in scale too.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        The drift.
    """
    scale = 1.0 + fraction * (LOOP_SCALE - 1.0) if scaled else 1.0
    out = np.eye(4)
    out[:3, :3] = scale * so3.exp([0.0, fraction * LOOP_YAW, 0.0])
    out[:3, 3] = fraction * np.asarray(LOOP_SHIFT)
    return out


def drift_map(m: Map, drift: list[np.ndarray]) -> Map:
    """Carry each keyframe, and the points it made, by its drift.

    A world point ``p`` made by keyframe ``j`` goes to ``D_j p``; keyframe ``j``'s pose goes to
    the rigid one under which ``D_j p`` lies where ``s_j`` times ``p`` lay before, so each
    keyframe still sees its own points where its keypoints are, and a point another keyframe
    made by the difference of their drifts.

    Parameters
    ----------
    m : Map
        The map.
    drift : list of ndarray, shape (4, 4)
        Each keyframe's drift, by row.

    Returns
    -------
    Map
        The map drifted, every point refreshed.
    """
    poses, xyz = m.kf_pose.copy(), m.pt_xyz.copy()
    for k, d in enumerate(drift):
        scale = float(np.linalg.norm(d[:3, 0]))
        turn = d[:3, :3] / scale
        rotation = m.kf_pose[k, :3, :3] @ turn.T
        poses[k, :3, :3] = rotation
        poses[k, :3, 3] = scale * m.kf_pose[k, :3, 3] - rotation @ d[:3, 3]
    made_by = m.keyframe_rows(m.pt_first_kf)
    for p in range(m.points):
        d = drift[int(made_by[p])]
        xyz[p] = d[:3, :3] @ m.pt_xyz[p] + d[:3, 3]
    return maps.refresh_points(replace(m, kf_pose=poses, pt_xyz=xyz), np.arange(m.points))


def exact_observations(m: Map) -> Map:
    """Return the map with each observation where its point projects, its right column too.

    The keypoints' bearings follow; a keyframe's free keypoints stay where they were detected.

    Parameters
    ----------
    m : Map
        The map.

    Returns
    -------
    Map
        The map, its observed keypoints moved.
    """
    rows = np.flatnonzero(m.kp_point >= 0)
    keyframe = m.keypoint_keyframe()[rows]
    world = m.pt_xyz[m.point_rows(m.kp_point[rows])]
    local = np.einsum("nij,nj->ni", m.kf_pose[keyframe, :3, :3], world)
    local += m.kf_pose[keyframe, :3, 3]
    xy, right, bearing = m.kp_xy.copy(), m.kp_right.copy(), m.kp_bearing.copy()
    xy[rows] = CAMERA.project(local)
    bearing[rows] = CAMERA.bearings(xy[rows])
    measured = rows[~np.isnan(m.kp_right[rows])]
    right[measured] = (
        xy[measured, 0] - CAMERA.focal * CAMERA.baseline / local[np.isin(rows, measured), 2]
    )
    return replace(m, kp_xy=xy, kp_right=right, kp_bearing=bearing)


def loop_map(built: list[frames.Frame], poses: list[np.ndarray]) -> Map:
    """Build a map of the loop room from its true poses, each keyframe seeing only its window.

    Each frame becomes a keyframe observing the points the :data:`LOOP_WINDOW` keyframes before
    it made, and adding points from its own depth at every second free keypoint, as
    :func:`truth_map`; each is given its parent (:func:`slam_it.edge.covisibility.adopt`).

    Parameters
    ----------
    built : list of slam_it.edge.frames.Frame
        The keyframes' frames, in RGB-D.
    poses : list of ndarray, shape (4, 4)
        Their true ``T_cam_world``.

    Returns
    -------
    Map
        The map, undrifted, its observations exact (:func:`exact_observations`).
    """
    m = maps.empty(OPTIONS.features.levels, OPTIONS.features.scale_factor)
    for i, (frame, pose) in enumerate(zip(built, poses, strict=True)):
        observed = _observe(m, frame, pose, among=m.pt_first_kf >= i - LOOP_WINDOW)
        m = maps.insert_keyframe(m, frame, pose, observed)
        m = _points_from_depth(m, pose)
        m = covisibility.adopt(m, m.keyframes - 1)
    return exact_observations(m)


@dataclass(frozen=True)
class Loop:
    """The loop room's keyframes, their true poses, and their map undrifted and drifted.

    ``scaled`` is the monocular variant: no depth and no right column, the drift a similarity.
    """

    frames: list[frames.Frame]
    poses: list[np.ndarray]
    truth: Map
    drift: list[np.ndarray]
    map: Map
    scaled: bool


def _without_depth(m: Map) -> Map:
    """Return the map as a monocular one holds it: no depth and no right column."""
    return replace(
        m, kp_depth=np.full_like(m.kp_depth, np.nan), kp_right=np.full_like(m.kp_right, np.nan)
    )


@pytest.fixture(scope="session")
def loop_keyframes() -> tuple[list[frames.Frame], list[np.ndarray], Map]:
    """Render the loop room's keyframes, build them in RGB-D, and map them from the truth."""
    path = room.loop_path(room.LOOP_FRAMES)[::LOOP_EVERY]
    scene = room.Room()
    extraction = OPTIONS.extraction()
    built, poses = [], []
    for i, pose in enumerate(path):
        gray, depth, _ = scene.render(room.SMALL, pose)
        built.append(frames.rgbd(gray, depth, i * LOOP_EVERY / 30.0, CAMERA, extraction))
        poses.append(np.linalg.inv(pose))
    return built, poses, loop_map(built, poses)


def _loop(keyframes: tuple[list[frames.Frame], list[np.ndarray], Map], *, scaled: bool) -> Loop:
    """Drift the loop map, rigidly or in scale too."""
    built, poses, truth = keyframes
    drift = [loop_drift(k / (truth.keyframes - 1), scaled=scaled) for k in range(truth.keyframes)]
    drifted = drift_map(truth, drift)
    if scaled:
        truth, drifted = _without_depth(truth), _without_depth(drifted)
    return Loop(built, poses, truth, drift, drifted, scaled)


@pytest.fixture(scope="session")
def loop_rigid(loop_keyframes: tuple[list[frames.Frame], list[np.ndarray], Map]) -> Loop:
    """Return the loop map in RGB-D, drifted rigidly."""
    return _loop(loop_keyframes, scaled=False)


@pytest.fixture(scope="session")
def loop_scaled(loop_keyframes: tuple[list[frames.Frame], list[np.ndarray], Map]) -> Loop:
    """Return the loop map as monocular, drifted in scale too."""
    return _loop(loop_keyframes, scaled=True)


@pytest.fixture(scope="session")
def loop_database(
    loop_keyframes: tuple[list[frames.Frame], list[np.ndarray], Map],
    words: vocabulary.Vocabulary,
) -> Database:
    """Put the loop map's keyframes in a database (tests must not change it)."""
    m = loop_keyframes[2]
    db = Database(words)
    for k in range(m.keyframes):
        db.add(int(m.kf_id[k]), m.kp_descriptor[m.keypoint_rows(k)])
    return db


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
def words() -> vocabulary.Vocabulary:
    """Return the committed vocabulary."""
    return vocabularies.default()


@pytest.fixture(scope="session")
def room_database(truth: Truth, words: vocabulary.Vocabulary) -> Database:
    """Put the truth map's keyframes in a database (tests must not change it)."""
    db = Database(words)
    for k in range(truth.map.keyframes):
        db.add(int(truth.map.kf_id[k]), truth.map.kp_descriptor[truth.map.keypoint_rows(k)])
    return db


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
    run = pipeline.Pipeline(mode, camera, OPTIONS, words_for(OPTIONS))
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
