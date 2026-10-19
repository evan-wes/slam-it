"""The loop room threaded under ThreadSanitizer, in both of C++'s schemes (B3b's review panel, A3).

``scripts/check.sh``'s ThreadSanitizer step builds the C++ tests and programs once more with the
sanitizer (``build/tsan``) and runs this file alone (``-m tsan``), the instrumented program named
by :data:`TSAN_PROGRAM`: one threaded run of the loop room with loops on, in our design and in
ORB-SLAM's locking, so that the paths where two writers meet run under the check: loop closing's
stop inside its hold and its correction, a global BA in a thread of its own and its result carried
onto the map, and tracking following each correction. The native tests under the sanitizer never
close a loop: their scene has none. The sanitizer halts the program at the first race it reports
(:data:`TSAN_OPTIONS`), which fails the run.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from slam_it.eval import runs, vocabularies
from slam_it.orchestration import vo
from tests.edge.support import OPTIONS
from tests.orchestration.datasets import Room

pytestmark = pytest.mark.tsan

TSAN_PROGRAM = "SLAM_IT_TSAN_VO"
"""The environment variable naming the ThreadSanitizer build's ``slam_it_vo``."""

TSAN_OPTIONS = "halt_on_error=1:second_deadlock_stack=1"
"""The sanitizer's settings, as ``scripts/check.sh`` runs the C++ tests: the first race reported
ends the program, and a deadlock's report shows both stacks."""


@pytest.mark.parametrize("locks", [False, True], ids=["view", "locks"])
def test_the_loop_room_closes_its_loop_under_thread_sanitizer(
    locks: bool, loop_folders: Room, tmp_path: Path
) -> None:
    """Threaded, RGB-D, a loop closes and a global BA runs in its thread, with no race reported."""
    program = os.environ.get(TSAN_PROGRAM)
    if not program:
        pytest.fail(f"{TSAN_PROGRAM}: set it to a ThreadSanitizer build's slam_it_vo")
    request = vo.Request(
        "tum", loop_folders.tum, "rgbd", tmp_path / "run", options=OPTIONS,
        vocabulary=vocabularies.DEFAULT, threaded=True, locks=locks,
    )  # fmt: skip
    vo.check(request)
    done = subprocess.run(
        [program, *request.arguments()],
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "TSAN_OPTIONS": TSAN_OPTIONS},
    )
    assert done.returncode == 0, done.stderr[-4000:]
    assert "ThreadSanitizer" not in done.stderr
    run = runs.read_run(request.out)
    assert len(run.keyframe_loops) >= 1, "a loop closed and was corrected"
    assert any(frame.get("global_ba") for frame in run.diagnostics), "a global BA ran"
