"""The merge room's bar: every language merges the map a lost track left with the next one.

The merge room (:func:`tests.orchestration.datasets.merge_frames`) is the loop room blinded
from :data:`~tests.orchestration.datasets.MERGE_AT` for longer than the loss window, so its
first map is lost and kept, and a new map starts; the camera then comes back over the first
map's start. At :data:`~tests.orchestration.datasets.MERGE_OPTIONS`, each backend's run must
(the online-SLAM design's section 14, item 8): end in one map, merged, and at no wrong place
(:func:`tests.orchestration.datasets.wrong_places`); track the frames it can see; keep its
final trajectory within :data:`MERGE_ATE` of the truth; and leave its points near the room's
walls (:data:`SURFACE`). The final trajectory is scored over the one merged map, aligned once,
so its ATE compares with neither map's ATE as tracked, each aligned on its own. The bounds are
about half again the first runs' (2026-10-17, the same in Python and C++): final ATE RMSE 5.46
cm (RGB-D), 5.17 cm (stereo) and 1.85 cm (monocular); the points' median distance to the walls
6.8, 6.2 and 6.1 cm; every frame the camera can see tracked in RGB-D and stereo, 176 of the 184
in monocular.
"""

from __future__ import annotations

import numpy as np
import pytest

from slam_it.edge.options import Options
from slam_it.eval import runs
from tests.edge import room
from tests.orchestration.conftest import RoomRun
from tests.orchestration.datasets import (
    LOOP_CASES,
    MERGE_AT,
    MERGE_BLANKS,
    Room,
    merge_frames,
    wrong_places,
)

MERGE_ATE = {"rgbd": 0.08, "stereo": 0.08, "monocular": 0.028}
"""The final trajectory's ATE (RMSE, metres) over the merged map, aligned once, each mode stays
under."""

SURFACE = {"rgbd": 0.10, "stereo": 0.095, "monocular": 0.09}
"""The median distance, in metres, of the final points to the room's walls and boxes, the map
placed on the truth by its ATE alignment."""

TRACKED = 0.95
"""The share of the frames the camera can see (all but the black ones) tracked at least: the
design's bar."""

MERGING = ("python", "cpp")
"""The backends that merge maps: Rust's port of milestone 2B's B4 follows."""


def _frames(count: int) -> list[room.Frame]:
    """Return tiny frames at the room's 30 Hz, each lit."""
    lit = np.full((2, 3), 9, dtype=np.uint8)
    depth = np.ones((2, 3))
    return [room.Frame(k / 30.0, lit, depth, lit, np.eye(4)) for k in range(count)]


def test_the_merge_room_is_blind_for_longer_than_the_loss_window() -> None:
    """Its black frames outlast ``loss.relocalize_seconds``, and end before the loop comes back."""
    given = _frames(room.LOOP_FRAMES)
    made = merge_frames(given)
    assert len(made) == len(given)
    black = [k for k, f in enumerate(made) if not np.any(f.gray)]
    assert black == list(range(MERGE_AT, MERGE_AT + MERGE_BLANKS))
    for k in black:
        assert not np.any(made[k].depth)
        assert made[k].right is not None
        assert not np.any(made[k].right)
    assert [f.timestamp for f in made] == [f.timestamp for f in given]
    seconds = made[black[-1] + 1].timestamp - made[black[0]].timestamp
    assert seconds > Options().loss.relocalize_seconds
    back = room.LOOP_FRAMES * 2.0 * np.pi / room.LOOP_TURN  # the frame at 360 degrees
    assert black[-1] < back


@pytest.mark.parametrize("backend", MERGING)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_every_backend_merges_the_merge_room(
    dataset: str, mode: str, backend: str, merge_folders: Room, merge_runs: RoomRun
) -> None:
    """The run ends in one map, merged at the right place, its final trajectory near the truth.

    Its first map was lost and kept, and the second started after the black frames.
    """
    run = merge_runs(dataset, mode, backend)
    assert [d["state"] for d in run.diagnostics].count("lost") == 1
    assert sorted(set(run.tracked_maps.tolist())) == [0, 1]
    assert run.maps().tolist() == [0]
    assert len(run.keyframe_merges) >= 1
    assert not wrong_places(merge_folders, run.keyframe_timestamps, run.keyframe_merges)
    assert len(run.timestamps) >= TRACKED * (len(run.diagnostics) - MERGE_BLANKS)
    final = runs.evaluate_absolute(run, merge_folders.timestamps, merge_folders.poses)
    (merged,) = final.maps
    assert merged.absolute.rmse < MERGE_ATE[mode]


@pytest.mark.parametrize("backend", MERGING)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_the_merged_map_s_points_lie_on_the_walls(
    dataset: str, mode: str, backend: str, merge_folders: Room, merge_runs: RoomRun
) -> None:
    """Placed on the truth by the trajectory's alignment, its points lie near the room's walls."""
    run = merge_runs(dataset, mode, backend)
    (placed,) = runs.evaluate_absolute(run, merge_folders.timestamps, merge_folders.poses).maps
    alignment = placed.absolute.alignment
    points = run.final_points() @ alignment[:3, :3].T + alignment[:3, 3]
    assert float(np.median(room.surface_distance(points))) < SURFACE[mode]
