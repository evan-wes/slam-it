"""The loop room's bar: every language closes its loop, at the right place, and maps well.

The online-SLAM design's section 10 holds each language to a bar of results on the same data,
not to the others' decisions. On the loop room (:func:`tests.edge.room.loop_path`, written by
:func:`tests.orchestration.datasets.loop_frames`), each backend's run must: track nearly every
frame, in one map; close a loop, and none at a wrong place (judged as the test-set measure
judges real data, by :func:`slam_it.eval.loops.judge_loops`: by the truth, its two keyframes
more than :data:`slam_it.eval.loops.WRONG_DISTANCE` apart or looking more than
:data:`slam_it.eval.loops.WRONG_ANGLE` apart, so they could not have seen the same place);
keep its final trajectory within :data:`LOOP_ATE` of the truth, and closer than as tracked; and
leave its points near the room's walls (:data:`SURFACE`). The bounds are about half again B3's
first runs' (2026-10-17, the same in every backend in RGB-D and stereo): final ATE RMSE 4.67 cm
(RGB-D), 3.64 cm (stereo) and 1.14 to 1.34 cm (monocular); the points' median distance to the
walls 7.3, 7.1 and 3.9 to 4.3 cm; 276 of the 280 frames tracked in monocular, every one
otherwise.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from slam_it import backends
from slam_it.eval import loops, runs
from tests.edge import room
from tests.edge.support import OPTIONS
from tests.orchestration.conftest import RoomRun
from tests.orchestration.datasets import LOOP_CASES, Room

LOOP_ATE = {"rgbd": 0.07, "stereo": 0.055, "monocular": 0.02}
"""The final trajectory's ATE (RMSE, metres) each mode stays under."""

SURFACE = {"rgbd": 0.11, "stereo": 0.11, "monocular": 0.065}
"""The median distance, in metres, of the final points to the room's walls and boxes, the map
placed on the truth by its ATE alignment."""

TRACKED = 0.95
"""The share of the frames tracked at least: the design's bar."""


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
    """By the truth, every loop's two keyframes were near each other and looked alike.

    Judged by the test-set measure's judge (B3's review panel, A7), which finds every keyframe
    in the truth: the room's frames are the truth's.
    """
    run = loop_runs(dataset, mode, backend)
    truth = (loop_folders.timestamps, loop_folders.poses)
    judged = loops.judge_loops(run, truth, runs.evaluate_absolute(run, *truth))
    assert [loop["wrong_place"] for loop in judged] == [False] * len(run.keyframe_loops)


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


def test_a_loop_closes_with_relocalisation_off(loop_folders: Room, tmp_path: Path) -> None:
    """Loop closing alone keeps the keyframe database, and closes the room's loop.

    Before B3's review panel (H7) the database went with relocalisation, and loop closing
    without it closed nothing (run 2, H23). Python, monocular: the quickest case to run.
    """
    options = OPTIONS.with_settings({"relocalization.enabled": False})
    run = RoomRun(loop_folders, options, tmp_path)("tum", "monocular", "python")
    assert run.vocabulary is not None
    assert len(run.keyframe_loops) >= 1
