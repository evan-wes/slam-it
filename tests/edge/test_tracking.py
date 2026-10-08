"""Tracking's tests: every backend against the reference, and its rules.

On the room's truth map, and on an exact scene whose keypoints sit on chosen levels.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace

import numpy as np
import pytest

from slamlab import backends
from slamlab.edge import map as maps
from slamlab.edge import tracking
from slamlab.edge.frames import Camera, Frame, Tracked
from slamlab.edge.map import Map
from slamlab.foundations import orb
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import require_function
from tests.edge.conftest import Truth
from tests.edge.support import CAMERA, OPTIONS, SOLVER_TOLERANCE, assert_close

LAST, CURRENT = 36, 37
"""The room's frames tracked: the last keyframe, and the frame after it."""

NUDGE = np.array([0.01, 0.0, -0.01])
"""The prediction's error: about a centimetre and a half."""


def _close(a: Tracked | None, b: Tracked | None) -> None:
    """Check two tracked frames: both None, or the same matches and poses (solver tolerance)."""
    assert (a is None) == (b is None)
    if a is None or b is None:
        return
    assert_close(a.pose, b.pose, SOLVER_TOLERANCE)
    np.testing.assert_array_equal(a.points, b.points)


@dataclass(frozen=True)
class Room:
    """The room's map, the frames tracked, and what is known of them."""

    map: Map
    last: Frame
    frame: Frame
    last_tracked: Tracked
    truth: np.ndarray
    predicted: np.ndarray


@pytest.fixture(scope="module")
def room_state(truth: Truth) -> Room:
    """Track frame :data:`CURRENT` from keyframe :data:`LAST` of the truth map."""
    m = truth.map
    last_tracked = Tracked(pose=truth.poses[LAST], points=m.kp_point[m.keypoint_rows(3)].copy())
    predicted = truth.poses[CURRENT].copy()
    predicted[:3, 3] += NUDGE
    return Room(
        m, truth.frames[LAST], truth.frames[CURRENT], last_tracked, truth.poses[CURRENT], predicted
    )


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("mode", ["monocular", "stereo", "rgbd"])
def test_tracking_matches_the_reference(backend: str, mode: str, room_state: Room) -> None:
    """Every backend tracks the last frame, then the local map, as the reference does."""
    for function in ("track_last_frame", "track_local_map"):
        require_function(backend, tracking.__name__, function)
    r = room_state
    args = (r.map, CAMERA, r.frame, r.last, r.last_tracked)
    expected = tracking.track_last_frame(*args, predicted=r.predicted, mode=mode, options=OPTIONS)
    assert expected is not None
    got_last = tracking.track_last_frame(
        *args, predicted=r.predicted, mode=mode, options=OPTIONS, backend=backend
    )
    _close(got_last, expected)
    for recovering in (False, True):
        local = tracking.track_local_map(
            r.map, CAMERA, r.frame, expected, mode=mode, options=OPTIONS, recovering=recovering
        )
        got = tracking.track_local_map(
            r.map, CAMERA, r.frame, expected, mode=mode, options=OPTIONS, recovering=recovering,
            backend=backend,
        )  # fmt: skip
        _close(got.tracked, local.tracked)
        np.testing.assert_array_equal(got.visible, local.visible)
        np.testing.assert_array_equal(got.found, local.found)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_stages_match_the_reference(backend: str, room_state: Room) -> None:
    """Every backend refines, selects the points in view, and relocalises as the reference."""
    for function in ("optimize_pose", "in_view", "relocalize_locally"):
        require_function(backend, tracking.__name__, function)
    r = room_state
    rows = np.arange(r.map.points)
    expected = tracking.in_view(r.map, CAMERA, r.truth, rows, OPTIONS)
    got = tracking.in_view(r.map, CAMERA, r.truth, rows, OPTIONS, backend=backend)
    for name in ("rows", "level"):
        np.testing.assert_array_equal(getattr(got, name), getattr(expected, name))
    for name in ("xy", "right", "view_cos"):
        assert_close(getattr(got, name), getattr(expected, name))
    start = Tracked(pose=r.predicted, points=r.last_tracked.points)  # the last frame's matches
    _close(
        tracking.optimize_pose(r.map, CAMERA, r.last, start, OPTIONS, backend=backend),
        tracking.optimize_pose(r.map, CAMERA, r.last, start, OPTIONS),
    )
    _close(
        tracking.relocalize_locally(r.map, CAMERA, r.frame, OPTIONS, backend=backend),
        tracking.relocalize_locally(r.map, CAMERA, r.frame, OPTIONS),
    )
    strict = replace(OPTIONS, pnp_min_inliers=r.map.points + 1)
    assert tracking.relocalize_locally(r.map, CAMERA, r.frame, strict, backend=backend) is None


def test_tracking_finds_the_pose(room_state: Room) -> None:
    """From a prediction off by a centimetre and a half, the local map pins the pose."""
    r = room_state
    last = tracking.track_last_frame(
        r.map, CAMERA, r.frame, r.last, r.last_tracked,
        predicted=r.predicted, mode="rgbd", options=OPTIONS,
    )  # fmt: skip
    assert last is not None
    local = tracking.track_local_map(r.map, CAMERA, r.frame, last, mode="rgbd", options=OPTIONS)
    assert local.tracked.inliers > last.inliers >= OPTIONS.min_tracked
    assert np.abs(local.tracked.pose[:3, 3] - r.truth[:3, 3]).max() < 0.01
    np.testing.assert_array_equal(local.found, local.tracked.points[local.tracked.points >= 0])
    assert set(local.visible) >= set(last.points[last.points >= 0])


def test_relocalizing_places_the_frame(room_state: Room) -> None:
    """Without a prediction, descriptors and PnP place the frame near the local map."""
    r = room_state
    placed = tracking.relocalize_locally(r.map, CAMERA, r.frame, OPTIONS)
    assert placed is not None
    assert placed.inliers >= OPTIONS.pnp_min_inliers
    assert np.abs(placed.pose[:3, 3] - r.truth[:3, 3]).max() < 0.01


def test_a_point_behind_or_turned_away_is_not_in_view(room_state: Room) -> None:
    """Turned around, the camera sees none of the map; with tight distances, fewer points."""
    r = room_state
    rows = np.arange(r.map.points)
    turned = r.truth.copy()
    turned[:3, :3] = np.diag([-1.0, 1.0, -1.0]) @ turned[:3, :3]
    assert len(tracking.in_view(r.map, CAMERA, turned, rows, OPTIONS).rows) == 0
    near = replace(r.map, pt_max_distance=r.map.pt_max_distance * 0.5)
    seen = tracking.in_view(r.map, CAMERA, r.truth, rows, OPTIONS)
    assert 0 < len(tracking.in_view(near, CAMERA, r.truth, rows, OPTIONS).rows) < len(seen.rows)


def test_refinement_drops_a_wrong_match(room_state: Room) -> None:
    """A keypoint matched to a far-away point fails the chi-square test and is unmatched."""
    r = room_state
    points = r.last_tracked.points.copy()
    matched = np.flatnonzero(points >= 0)
    points[matched[0]], points[matched[-1]] = points[matched[-1]], -1  # each point once
    start = Tracked(pose=r.last_tracked.pose, points=points)
    refined = tracking.optimize_pose(r.map, CAMERA, r.last, start, OPTIONS)
    assert refined.points[matched[0]] == -1
    assert np.abs(refined.pose[:3, 3] - r.last_tracked.pose[:3, 3]).max() < 0.01


def test_predict_keeps_the_pose_without_a_velocity() -> None:
    """No velocity: the last pose; with one, the motion applied again."""
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    np.testing.assert_array_equal(tracking.predict(pose, None), pose)
    velocity = np.eye(4)
    velocity[0, 3] = 0.5
    np.testing.assert_array_equal(tracking.predict(pose, velocity)[:3, 3], [1.5, 2.0, 3.0])


# An exact scene: 60 points, keypoints at their projections on a chosen level.

SCENE_CAMERA = Camera("pinhole", np.array([250.0, 250.0, 160.0, 120.0]), 320, 240, 0.1)
"""The exact scene's camera: 250 px focal, a 0.1 m baseline."""


def _scene_points() -> np.ndarray:
    """Return a 10 x 6 grid of points three metres out, its depths rippled."""
    p = np.arange(60)
    depth = 3.0 + 0.4 * np.sin(1.7 * p)
    return np.column_stack([-1.0 + (p % 10) * 2.0 / 9.0, -0.6 + (p // 10) * 0.24, depth])


def _scene_frame(pose: np.ndarray, level: int) -> Frame:
    """Return a frame seeing every scene point, its keypoints on one level, without depth."""
    local = _scene_points() @ pose[:3, :3].T + pose[:3, 3]
    xy = SCENE_CAMERA.project(local)
    n = len(xy)
    keypoints = orb.Keypoints(
        xy=xy,
        level=np.full(n, level, dtype=np.int64),
        level_xy=np.rint(xy / 1.2**level).astype(np.int64),
        response=np.zeros(n, dtype=np.int64),
        angle=np.zeros(n, dtype=np.int64),
    )
    p, b = np.meshgrid(np.arange(n), np.arange(orb.DESCRIPTOR_BYTES), indexing="ij")
    descriptors = ((p * 37 + b * 11) & 0xFF).astype(np.uint8)
    features = orb.Features(keypoints, descriptors, scale_factor=1.2, levels=8)
    return Frame(0.0, features, SCENE_CAMERA.bearings(xy), np.full(n, np.nan), np.full(n, np.nan))


def _moved(dz: float) -> np.ndarray:
    """Return the pose of a camera moved ``dz`` metres along its axis."""
    pose = np.eye(4)
    pose[2, 3] = -dz
    return pose


@pytest.fixture(scope="module")
def scene() -> tuple[Map, Frame, Tracked]:
    """Build the scene's map of one keyframe, its keypoints on level 2, each seeing its point."""
    last = _scene_frame(np.eye(4), level=2)
    m = maps.insert_keyframe(maps.empty(8, 1.2), last, np.eye(4), np.full(last.size, -1))
    m = maps.add_points(m, _scene_points(), np.arange(last.size)[:, None])
    return m, last, Tracked(pose=np.eye(4), points=m.pt_id.copy())


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("mode", "dz", "found"),
    [("rgbd", 0.3, False), ("rgbd", -0.3, True), ("monocular", 0.3, True)],
)
def test_moving_forward_searches_no_finer_level(
    backend: str, mode: str, dz: float, found: bool, scene: tuple[Map, Frame, Tracked]
) -> None:
    """With depth, moving forward a point seen on level 2 is not sought on level 1.

    Moving back (by more than the baseline), or without depth, it is (``TrackWithMotionModel``).
    """
    chosen = require_function(backend, tracking.__name__, "track_last_frame")
    m, last, last_tracked = scene
    frame = _scene_frame(_moved(dz), level=1)
    tracked = tracking.track_last_frame(
        m, SCENE_CAMERA, frame, last, last_tracked,
        predicted=_moved(dz), mode=mode, options=OPTIONS, backend=chosen,
    )  # fmt: skip
    assert (tracked is not None) == found
    if tracked is not None:
        assert tracked.inliers == frame.size
        np.testing.assert_allclose(tracked.pose, _moved(dz), atol=1e-9)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rejects_malformed_arguments(backend: str, room_state: Room) -> None:
    """A bad mode, pose, matches or recovery flag raises before any backend runs."""
    r = room_state
    short = Tracked(pose=r.last_tracked.pose, points=r.last_tracked.points[:-1])
    flat = Tracked(pose=np.eye(3), points=r.last_tracked.points)
    calls: list[Callable[[], object]] = [
        lambda: tracking.track_last_frame(
            r.map, CAMERA, r.frame, r.last, r.last_tracked,
            predicted=r.predicted, mode="lidar", options=OPTIONS, backend=backend,
        ),
        lambda: tracking.track_last_frame(
            r.map, CAMERA, r.frame, r.last, short,
            predicted=r.predicted, mode="rgbd", options=OPTIONS, backend=backend,
        ),
        lambda: tracking.track_local_map(
            r.map, CAMERA, r.last, r.last_tracked,
            mode="rgbd", options=OPTIONS, recovering="yes",  # type: ignore[arg-type]
            backend=backend,
        ),
        lambda: tracking.optimize_pose(r.map, CAMERA, r.last, flat, OPTIONS, backend=backend),
        lambda: tracking.in_view(r.map, CAMERA, r.truth, [r.map.points], OPTIONS, backend=backend),
    ]  # fmt: skip
    for call in calls:
        with pytest.raises(InvalidInputError):
            call()
