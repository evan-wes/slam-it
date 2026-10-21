"""The merge and revisit rooms threaded under ThreadSanitizer, in both of C++'s schemes.

``scripts/check.sh``'s ThreadSanitizer step runs this file with the loop room's
(``test_loop_room_tsan.py``), through the instrumented program as that file runs it
(:func:`tests.orchestration.test_loop_room_tsan.run_sanitized`): one threaded run of each room
in RGB-D, in our design and in ORB-SLAM's locking. The merge room's (option B's merge) runs the
paths a merge adds under the check: loop closing searching the kept maps beside its loop search
and leaving the merge waiting, tracking draining mapping and loop closing between two frames,
and the merge made with mapping stopped while a global BA may be superseded and started again.
The revisit room's runs the paths taking a kept map up adds: relocalisation in every map,
tracking settling the threads, the map left kept and the kept map taken up with its database
(C++'s baseline lets its lock go while the threads settle). The sanitizer halts the program at
the first race it reports, which fails the run.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.orchestration.datasets import MERGE_OPTIONS, REVISIT_OPTIONS, Room
from tests.orchestration.test_loop_room_tsan import run_sanitized
from tests.orchestration.test_revisit_room import assert_revisited

pytestmark = pytest.mark.tsan


@pytest.mark.parametrize("locks", [False, True], ids=["view", "locks"])
def test_the_merge_room_merges_under_thread_sanitizer(
    locks: bool, merge_folders: Room, tmp_path: Path
) -> None:
    """Threaded, RGB-D, the two maps merge into one, with no race reported."""
    run = run_sanitized(merge_folders.tum, MERGE_OPTIONS, tmp_path / "run", locks=locks)
    assert run.maps().tolist() == [0], "the two maps merged into one"
    assert len(run.keyframe_merges) >= 1


@pytest.mark.parametrize("locks", [False, True], ids=["view", "locks"])
def test_the_revisit_room_takes_its_first_map_up_under_thread_sanitizer(
    locks: bool, revisit_folders: Room, tmp_path: Path
) -> None:
    """Threaded, RGB-D, the first map is taken up again as the bar asks, with no race reported."""
    run = run_sanitized(revisit_folders.tum, REVISIT_OPTIONS, tmp_path / "run", locks=locks)
    assert_revisited(run, revisit_folders, "rgbd")
