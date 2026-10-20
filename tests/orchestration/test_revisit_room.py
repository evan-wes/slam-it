"""The revisit room's bar: every language takes a kept map up again when relocalisation finds it.

The revisit room (:func:`tests.orchestration.datasets.revisit_frames`) is the merge room with
loop closing off, so its two maps never merge, and blinded a second time, from
:data:`~tests.orchestration.datasets.REVISIT_AT`, for less than the loss window: the camera comes
back over the first map's start, which the second map has not seen. At
:data:`~tests.orchestration.datasets.REVISIT_OPTIONS`, each backend's run must (the online-SLAM
design's section 14, item 2): be lost once, on the first black stretch; on the first frame after
the second, relocalise in the first map, kept, leave the second, kept too (``left`` recorded),
and go on in the first, tracking every frame to the end there, the maps it tracks in going 0,
1, 0; and keep both maps to the end. Inline in every language, threaded in each native scheme
(:data:`tests.orchestration.test_merge_room_threads.THREADED`), and under ThreadSanitizer in both
of C++'s (``test_merge_room_tsan.py``).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge import pipeline
from slam_it.eval import runs
from slam_it.orchestration import vo
from tests.edge import room
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import (
    LOOP_CASES,
    MERGE_BLANKS,
    REVISIT_AT,
    REVISIT_BLANKS,
    REVISIT_OPTIONS,
    Room,
    revisit_frames,
)
from tests.orchestration.test_merge_room_threads import THREADED


def assert_revisited(run: runs.Run) -> None:
    """Assert a run of the revisit room took its first map up again, as the module docstring says.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.
    """
    states = [d["state"] for d in run.diagnostics]
    assert states.count("lost") == 1
    left = [k for k, d in enumerate(run.diagnostics) if d["left"] is not None]
    assert left == [REVISIT_AT + REVISIT_BLANKS], "the first frame after the second blind stretch"
    taken = run.diagnostics[left[0]]
    assert (taken["map"], taken["placed_by"]) == (0, pipeline.Placement.RELOCALIZATION.value)
    assert taken["left"]["reset"] is False
    tracked = run.tracked_maps.tolist()
    assert [m for k, m in enumerate(tracked) if k == 0 or m != tracked[k - 1]] == [0, 1, 0]
    assert run.maps().tolist() == [0, 1]
    assert len(run.keyframe_merges) == 0
    since = run.timestamps >= taken["timestamp"]
    assert np.count_nonzero(since) == len(run.diagnostics) - left[0], "every frame from it on"
    assert set(run.tracked_maps[since].tolist()) == {0}


def test_the_revisit_room_is_blind_twice_within_the_loss_window_the_second_time() -> None:
    """Its second black frames, after the camera's turn passes 345 degrees, last 1 s."""
    lit = np.full((2, 3), 9, dtype=np.uint8)
    given = [room.Frame(k / 30.0, lit, np.ones((2, 3)), lit, np.eye(4)) for k in range(280)]
    made = revisit_frames(given)
    black = [k for k, f in enumerate(made) if not np.any(f.gray)]
    second = list(range(REVISIT_AT, REVISIT_AT + REVISIT_BLANKS))
    assert black[-REVISIT_BLANKS:] == second
    assert len(black) == MERGE_BLANKS + REVISIT_BLANKS
    seconds = made[second[-1] + 1].timestamp - made[second[0]].timestamp
    assert seconds < REVISIT_OPTIONS.loss.relocalize_seconds
    assert not REVISIT_OPTIONS.loops.enabled


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_every_backend_takes_the_first_map_up_again(
    dataset: str, mode: str, backend: str, revisit_runs: RoomRun
) -> None:
    """Relocalised in the first map, the run leaves the second and goes on in the first."""
    assert_revisited(revisit_runs(dataset, mode, backend))


@pytest.mark.parametrize(("backend", "locks"), THREADED)
@pytest.mark.parametrize(("dataset", "mode"), LOOP_CASES)
def test_a_native_threaded_run_takes_the_first_map_up_again(  # noqa: PLR0917 -- the case, the fixtures
    dataset: str, mode: str, backend: str, locks: bool, revisit_folders: Room, tmp_path: Path
) -> None:
    """Threaded, the threads settled as the map is left, the run goes on in the first map."""
    folder = revisit_folders.tum if dataset == "tum" else revisit_folders.euroc
    run = vo.run(
        dataset, folder, mode, tmp_path / "run", backend=require_program(backend),
        options=REVISIT_OPTIONS, threaded=True, locks=locks,
    )  # fmt: skip
    assert (run.threaded, run.locks) == (True, locks)
    assert_revisited(run)
