"""The merge room threaded under ThreadSanitizer, in both of C++'s schemes (option B's merge).

``scripts/check.sh``'s ThreadSanitizer step runs this file with the loop room's
(``test_loop_room_tsan.py``), through the instrumented program as that file runs it
(:func:`tests.orchestration.test_loop_room_tsan.run_sanitized`): one threaded run of the merge room
in RGB-D, in our design and in ORB-SLAM's locking, so that the paths a merge adds run under the
check: loop closing searching the kept maps beside its loop search and leaving the merge waiting,
tracking draining mapping and loop closing between two frames, and the merge made with mapping
stopped while a global BA may be superseded and started again. The sanitizer halts the program
at the first race it reports, which fails the run.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.orchestration.datasets import MERGE_OPTIONS, Room
from tests.orchestration.test_loop_room_tsan import run_sanitized

pytestmark = pytest.mark.tsan


@pytest.mark.parametrize("locks", [False, True], ids=["view", "locks"])
def test_the_merge_room_merges_under_thread_sanitizer(
    locks: bool, merge_folders: Room, tmp_path: Path
) -> None:
    """Threaded, RGB-D, the two maps merge into one, with no race reported."""
    run = run_sanitized(merge_folders.tum, MERGE_OPTIONS, tmp_path / "run", locks=locks)
    assert run.maps().tolist() == [0], "the two maps merged into one"
    assert len(run.keyframe_merges) >= 1
