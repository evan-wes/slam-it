"""Runs of several sequences, each a session (the online-SLAM design's section 14, item 5).

The room run twice in turn through every backend, the command line carrying the sequences, the
sessions' time offsets, and what a run of several sequences refuses.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from slam_it import backends
from slam_it.edge.options import Options
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo
from tests.conftest import FIXTURES
from tests.edge.support import FRAMES, OPTIONS
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import Room

SEQUENCES = FIXTURES / "eval" / "sequences"
"""The readers' fixture folders: their files listed, their images absent."""

AFTER = 8
"""The second session's frames run: enough to start its map and track it."""

DECISIONS = ("state", "map", "tracked", "keyframe", "placed_by")
"""What a frame's diagnostics line says the pipeline decided."""


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_room_twice_is_two_sessions(
    backend: str, room_folders: Room, room_runs: RoomRun, tmp_path: Path
) -> None:
    """The room run twice in turn: the second session's first frame starts map 1.

    The first session runs as the room alone does; the second's clock is moved on past the
    first's, and its first frame records the boundary and the first map closed as it was then.
    """
    run = vo.run(
        "tum", [room_folders.tum, room_folders.tum], "rgbd", tmp_path / "run",
        count=FRAMES + AFTER, backend=require_program(backend), options=OPTIONS,
    )  # fmt: skip
    alone = room_runs("tum", "rgbd", backend)
    assert run.sequence == "room_tum+room_tum"
    assert [k for k, line in enumerate(run.diagnostics) if "session" in line] == [FRAMES]
    times = vo.open_source("tum", room_folders.tum, "rgbd").times
    offset = times[-1] + np.median(np.diff(times)) - times[0]
    session = run.diagnostics[FRAMES]["session"]
    assert (session["index"], session["sequence"]) == (1, "room_tum")
    assert session["offset"] == pytest.approx(offset)
    alive = alone.keyframe_lives[:, 1] == -1
    kept = int(np.sum(alive & (alone.keyframe_maps == 0)))
    points = int(np.sum((alone.point_lives[:, 1] == -1) & (alone.point_maps == 0)))
    reset = kept < OPTIONS.loss.session_keyframes
    assert session["closed"] == {"keyframes": kept, "points": points, "reset": reset}
    stamps = [line["timestamp"] for line in run.diagnostics]
    np.testing.assert_allclose(stamps, [*times, *(times[:AFTER] + offset)], rtol=0, atol=1e-9)
    for ours, theirs in zip(run.diagnostics[:FRAMES], alone.diagnostics, strict=True):
        assert [ours[key] for key in DECISIONS] == [theirs[key] for key in DECISIONS]
    first = run.diagnostics[FRAMES]
    assert (first["state"], first["map"], first["placed_by"]) == ("tracking", 1, "initialization")
    assert all(line["map"] == 1 for line in run.diagnostics[FRAMES:])


def test_the_sequences_cross_as_a_command_line() -> None:
    """Each sequence is a ``--sequence``, in turn, and reads back as it was."""
    request = vo.Request("tum", (Path("a"), Path("b")), "rgbd", Path("out"))
    arguments = request.arguments()
    assert arguments[:6] == ["--dataset", "tum", "--sequence", "a", "--sequence", "b"]
    assert vo.parse(arguments) == request


@pytest.mark.parametrize(
    ("times", "offsets"),
    [
        ([[0.0, 0.5, 1.0], [2.0, 2.5]], [0.0, 0.0]),  # after the ones before: its own clock
        ([[0.0, 0.5, 1.0], [0.0, 0.5]], [0.0, 1.5]),  # again from 0: a period after the last
        ([[0.0, 0.1, 0.2, 0.6], [], [1.0], [0.0]], [0.0, 0.0, 0.0, 2.0]),  # median; 1 s for one
        ([[], [5.0]], [0.0, 0.0]),  # nothing before
    ],
)
def test_each_session_keeps_its_clock_or_is_moved_on(
    times: list[list[float]], offsets: list[float]
) -> None:
    """A session's times are moved on only when they do not come after those before it."""
    assert vo.session_offsets([np.array(t) for t in times]) == offsets


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_the_sequences_share_one_camera(backend: str, tmp_path: Path) -> None:
    """A run of two sequences is named for both; sequences of different cameras are refused."""
    options = Options().with_settings({"relocalization.enabled": False})
    program = require_program(backend)
    both = vo.run(
        "tum", [SEQUENCES / "tum_camera", SEQUENCES / "tum_no_depth"], "monocular",
        tmp_path / "both", count=0, backend=program, options=options,
    )  # fmt: skip
    assert both.sequence == "tum_camera+tum_no_depth"
    with pytest.raises(InvalidInputError, match="--sequence: the sequences' cameras differ"):
        vo.run(
            "tum", [SEQUENCES / "tum_camera", SEQUENCES / "rgbd_dataset_freiburg1_fixture"],
            "monocular", tmp_path / "differ", count=0, backend=program, options=options,
        )  # fmt: skip


def test_a_run_needs_a_sequence(tmp_path: Path) -> None:
    """A run of no sequence is refused before anything runs."""
    with pytest.raises(InvalidInputError, match="--sequence: missing"):
        vo.run("tum", [], "rgbd", tmp_path / "run", options=OPTIONS)
