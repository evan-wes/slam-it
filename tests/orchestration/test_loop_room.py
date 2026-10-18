"""The loop room's bar: every language closes its loop, at the right place, and maps well.

The online-SLAM design's section 10 holds each language to a bar of results on the same data,
not to the others' decisions. On the loop room (:func:`tests.edge.room.loop_path`, written by
:func:`tests.orchestration.datasets.loop_frames`), each backend's run must: track nearly every
frame, in one map; close a loop, and none at a wrong place (by the truth, its two keyframes
more than :data:`WRONG_DISTANCE` apart or looking more than :data:`WRONG_ANGLE` apart, so they
could not have seen the same place); keep its final trajectory within :data:`LOOP_ATE` of the
truth, and closer than as tracked; and leave its points near the room's walls
(:data:`SURFACE`). The bounds are about half again B3's first runs' (2026-10-17, the same in
every backend in RGB-D and stereo): final ATE RMSE 4.67 cm (RGB-D), 3.64 cm (stereo) and
1.14 to 1.34 cm (monocular); the points' median distance to the walls 7.3, 7.1 and 3.9 to 4.3
cm; 276 of the 280 frames tracked in monocular, every one otherwise.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from slam_it import backends
from slam_it.eval import runs
from tests.edge import room
from tests.orchestration.conftest import RoomRun
from tests.orchestration.datasets import LOOP_CASES, Room

LOOP_ATE = {"rgbd": 0.07, "stereo": 0.055, "monocular": 0.02}
"""The final trajectory's ATE (RMSE, metres) each mode stays under."""

SURFACE = {"rgbd": 0.11, "stereo": 0.11, "monocular": 0.065}
"""The median distance, in metres, of the final points to the room's walls and boxes, the map
placed on the truth by its ATE alignment."""

TRACKED = 0.95
"""The share of the frames tracked at least: the design's bar."""

WRONG_DISTANCE = 3.0
"""Two keyframes farther apart than this, in metres, by the truth, are at different places."""

WRONG_ANGLE = math.radians(60.0)
"""Two keyframes whose viewing directions are further apart than this, by the truth, are at
different places; a turn about the viewing direction leaves the place seen."""


def _truth_at(folders: Room, times: np.ndarray) -> np.ndarray:
    """Return the true ``T_world_cam`` at each time: the rendered frame's, the nearest."""
    rows = np.abs(folders.timestamps[None, :] - np.asarray(times)[:, None]).argmin(axis=1)
    return np.asarray(folders.poses[rows], dtype=np.float64)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_every_backend_closes_the_loop_room(
    dataset: str, mode: str, backend: str, loop_folders: Room, loop_runs: RoomRun
) -> None:
    """Nearly every frame tracked in one map, a loop closed, the final trajectory near the truth.

    The final trajectory is closer to the truth than the poses as tracked: the loop corrected
    the drift.
    """
    run = loop_runs(dataset, mode, backend)
    assert len(run.timestamps) >= TRACKED * len(run.diagnostics)
    assert len(run.maps()) == 1
    assert len(run.keyframe_loops) >= 1
    final = runs.evaluate_absolute(run, loop_folders.timestamps, loop_folders.poses)
    tracked = runs.evaluate_absolute(run, loop_folders.timestamps, loop_folders.poses, tracked=True)
    assert final.ate_rmse < LOOP_ATE[mode]
    assert final.ate_rmse < tracked.ate_rmse


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_no_loop_closes_at_a_wrong_place(
    dataset: str, mode: str, backend: str, loop_folders: Room, loop_runs: RoomRun
) -> None:
    """By the truth, every loop's two keyframes were near each other and looked alike."""
    run = loop_runs(dataset, mode, backend)
    truth = _truth_at(loop_folders, run.keyframe_timestamps)
    for older, newer in run.keyframe_loops.tolist():
        a, b = truth[older], truth[newer]
        assert np.linalg.norm(a[:3, 3] - b[:3, 3]) < WRONG_DISTANCE
        assert math.acos(np.clip(a[:3, 2] @ b[:3, 2], -1.0, 1.0)) < WRONG_ANGLE


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_the_points_lie_on_the_walls(
    dataset: str, mode: str, backend: str, loop_folders: Room, loop_runs: RoomRun
) -> None:
    """Placed on the truth by the trajectory's alignment, the points lie near the room's walls."""
    run = loop_runs(dataset, mode, backend)
    (placed,) = runs.evaluate_absolute(run, loop_folders.timestamps, loop_folders.poses).maps
    alignment = placed.absolute.alignment
    points = run.final_points() @ alignment[:3, :3].T + alignment[:3, 3]
    assert float(np.median(room.surface_distance(points))) < SURFACE[mode]
