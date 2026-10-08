"""Whole sequences through every backend.

The room from its TUM and EuRoC folders, the native programs' command line, and their failures.
"""

from __future__ import annotations

import subprocess
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from slamlab import backends
from slamlab.edge import frames, tracking
from slamlab.edge.map import Map
from slamlab.edge.options import Options
from slamlab.eval import runs
from slamlab.foundations.errors import DegenerateError, InvalidInputError, NotConvergedError
from slamlab.orchestration import vo
from tests.conftest import FIXTURES, assert_matches_fixture, load_fixture
from tests.edge.support import (
    ATE_BOUND,
    FRAMES,
    MONOCULAR_START,
    OPTIONS,
    SOLVER_TOLERANCE,
    assert_close,
)
from tests.fixtures.edge.generate_room_runs import record
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import BLANK_AT, CASES, GAP_OPTIONS, Room, room_inputs

SEQUENCES = FIXTURES / "eval" / "sequences"
"""The readers' fixture folders: their files listed, their images absent."""


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_backend_tracks_the_room(
    dataset: str, mode: str, backend: str, room_folders: Room, room_runs: RoomRun
) -> None:
    """Every frame after the start is tracked, close to the truth, and the run says what ran."""
    run = room_runs(dataset, mode, backend)
    assert (run.dataset, run.mode, run.backend) == (dataset, mode, backend)
    assert run.sequence == f"room_{dataset}"
    assert run.settings == OPTIONS.as_dict()
    assert len(run.diagnostics) == FRAMES
    assert all(d["state"] == "tracking" for d in run.diagnostics[MONOCULAR_START:])
    started = FRAMES - MONOCULAR_START if mode == "monocular" else FRAMES
    assert len(run.timestamps) >= started
    assert len(run.keyframe_timestamps) >= 2
    assert len(run.points) > 0
    error = runs.evaluate(run, room_folders.timestamps, room_folders.poses)
    assert error.matched == len(run.timestamps)
    assert error.absolute.rmse < ATE_BOUND[mode]


STEPS = ("read_seconds", "build_seconds", "track_seconds", "map_seconds")
"""The steps each frame's diagnostics time (see :mod:`slamlab.orchestration.vo`)."""


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_frame_times_its_steps(
    dataset: str, mode: str, backend: str, room_runs: RoomRun
) -> None:
    """Every frame times reading, building and tracking; only keyframes after the start map.

    The frame that starts the map is initialisation, which tracking's time holds, as ORB-SLAM's
    tracking thread initialises.
    """
    run = room_runs(dataset, mode, backend)
    for d in run.diagnostics:
        assert all(d[step] > 0.0 for step in STEPS[:3])
    start = next(i for i, d in enumerate(run.diagnostics) if d["keyframe"])
    mapped = [d["map_seconds"] > 0.0 for d in run.diagnostics]
    assert mapped == [d["keyframe"] and i > start for i, d in enumerate(run.diagnostics)]
    assert all(d["map_seconds"] == 0.0 for d in run.diagnostics if not d["keyframe"])


def _assert_same_run(native: runs.Run, reference: runs.Run) -> None:
    """Check that a native run made the reference's decisions and placed what it placed.

    The same frames tracked with the same matches, the same keyframes, and poses and points
    within the solver tolerance.
    """
    np.testing.assert_array_equal(native.timestamps, reference.timestamps)
    np.testing.assert_array_equal(native.keyframe_timestamps, reference.keyframe_timestamps)
    assert native.points.shape == reference.points.shape
    assert_close(native.poses, reference.poses, SOLVER_TOLERANCE)
    assert_close(native.keyframe_poses, reference.keyframe_poses, SOLVER_TOLERANCE)
    assert_close(native.points, reference.points, SOLVER_TOLERANCE)
    states = [(d["state"], d["tracked"], d["keyframe"]) for d in native.diagnostics]
    assert states == [(d["state"], d["tracked"], d["keyframe"]) for d in reference.diagnostics]


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree(dataset: str, mode: str, backend: str, room_runs: RoomRun) -> None:
    """A native program makes Python's decisions and places every pose and point as Python does."""
    _assert_same_run(room_runs(dataset, mode, backend), room_runs(dataset, mode, "python"))


@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_gap_room_fails_once_and_recovers_twice(
    dataset: str, mode: str, gap_folders: Room, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """The blank frame fails; across the gap PnP recovers twice; every other frame is tracked.

    The blank frame fails, the fallback too; across the gap the motion model fails and PnP
    recovers, twice (the recovery's velocity spans the gap); every other frame from the start
    is tracked (review panel findings A3, A14).
    """
    recovered: list[bool] = []
    relocalize = tracking.relocalize_locally

    def counting(
        m: Map, camera: frames.Camera, frame: frames.Frame, options: Options
    ) -> frames.Tracked | None:
        found = relocalize(m, camera, frame, options)
        recovered.append(found is not None)
        return found

    monkeypatch.setattr(tracking, "relocalize_locally", counting)
    folder = gap_folders.tum if dataset == "tum" else gap_folders.euroc
    run = vo.run(dataset, folder, mode, tmp_path, options=GAP_OPTIONS)
    assert recovered == [False, True, True]
    started = next(i for i, d in enumerate(run.diagnostics) if d["tracked"] > 0)
    untracked = [i for i, d in enumerate(run.diagnostics) if i >= started and d["tracked"] == 0]
    assert untracked == [BLANK_AT]
    assert len(run.timestamps) == len(run.diagnostics) - started - 1
    assert all(d["state"] == "tracking" for d in run.diagnostics[started:])


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree_across_the_gap(
    dataset: str, mode: str, backend: str, gap_runs: RoomRun
) -> None:
    """Through a recovery, a native program makes Python's decisions as on the whole room."""
    _assert_same_run(gap_runs(dataset, mode, backend), gap_runs(dataset, mode, "python"))


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree_off_the_defaults(
    dataset: str, mode: str, backend: str, moved_runs: RoomRun
) -> None:
    """With the newer settings off their defaults, a native program makes Python's decisions.

    Review panel finding H21: at the defaults, a native that hard-coded a setting would agree.
    """
    _assert_same_run(moved_runs(dataset, mode, backend), moved_runs(dataset, mode, "python"))


@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_moved_settings_change_the_decisions(
    dataset: str, mode: str, moved_runs: RoomRun, gap_runs: RoomRun
) -> None:
    """Off their defaults, the settings change what Python decides on the gap room.

    So the agreement off the defaults is evidence that the natives read them.
    """
    moved, default = moved_runs(dataset, mode, "python"), gap_runs(dataset, mode, "python")
    assert record(moved) != record(default)


GOLDEN = load_fixture("edge/room_runs.json")
"""What the pipeline decided on the room and the gap room (``generate_room_runs.py``)."""


def test_the_golden_record_is_of_these_rooms() -> None:
    """The record is of the rooms and settings the tests run, and holds exactly their cases.

    Checked before any decision, so a change to the rooms, rather than to the pipeline, is
    named as one (review panel finding A20).
    """
    assert GOLDEN["inputs"] == room_inputs()
    names = {f"{name}/{dataset}/{mode}" for name in ("room", "gap") for dataset, mode in CASES}
    assert set(GOLDEN["cases"]) == names


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("name", ["room", "gap"])
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_every_backend_matches_the_golden_record(  # noqa: PLR0917 -- the case, both rooms
    dataset: str, mode: str, name: str, backend: str, room_runs: RoomRun, gap_runs: RoomRun
) -> None:
    """Every backend's run is the recorded one.

    A decision changed in all three languages fails here until the record is regenerated
    (review panel finding A1).
    """
    run = (room_runs if name == "room" else gap_runs)(dataset, mode, backend)
    expected = GOLDEN["cases"][f"{name}/{dataset}/{mode}"]
    assert record(run) | {"poses": None} == expected | {"poses": None}
    assert_matches_fixture(run.poses.reshape(-1, 16), expected["poses"], GOLDEN["tolerance"])


def test_a_request_crosses_as_a_command_line() -> None:
    """Only settings off their defaults are passed, integers as integers, reals round-tripping."""
    options = Options(window=7, close_depth=0.1 + 0.2)
    request = vo.Request("tum", Path("seq"), "rgbd", Path("out"), first=2, count=5, options=options)
    assert request.arguments() == [
        *("--dataset", "tum", "--sequence", "seq", "--mode", "rgbd", "--out", "out"),
        *("--first", "2", "--count", "5"),
        *("--option", "close_depth=0.30000000000000004", "--option", "window=7"),
    ]


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_run_records_its_settings(backend: str, tmp_path: Path) -> None:
    """A run of no frames records the settings asked for, every one, exactly."""
    options = Options(n_features=700, close_depth=0.1 + 0.2, chi2_mono=5.0, seed=3)
    run = vo.run(
        "tum",
        SEQUENCES / "tum_camera",
        "rgbd",
        tmp_path,
        backend=require_program(backend),
        options=options,
        count=0,
    )
    assert run.settings == options.as_dict()
    assert len(run.timestamps) == len(run.diagnostics) == 0


@pytest.mark.parametrize(
    ("dataset", "mode", "message"),
    [
        ("kitti", "rgbd", "dataset"),
        ("tum", "lidar", "mode"),
        ("tum", "stereo", "no stereo pairs"),
        ("euroc", "rgbd", "no depth maps"),
    ],
)
def test_refuses_what_a_dataset_does_not_record(dataset: str, mode: str, message: str) -> None:
    """An unknown dataset or mode, or a mode the dataset has no sensor for, is refused."""
    with pytest.raises(InvalidInputError, match=message):
        vo.run(dataset, SEQUENCES / "tum_camera", mode, "unused")


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rgbd_needs_the_depth_sensors_bf(backend: str, tmp_path: Path) -> None:
    """An RGB-D run on a folder whose camera gives no bf is refused, by every backend."""
    with pytest.raises(InvalidInputError, match="bf"):
        vo.run(
            "tum",
            SEQUENCES / "tum_no_depth",
            "rgbd",
            tmp_path,
            backend=require_program(backend),
            count=1,
        )


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_a_stereo_run_matches_with_its_settings(
    backend: str, room_folders: Room, tmp_path: Path
) -> None:
    """The stereo matcher's settings reach every backend's frames (review panel finding A15).

    Below a Hamming distance of 1 hardly a pair matches, so no frame has the depths to start
    the map; the run records the setting.
    """
    strict = replace(OPTIONS, stereo_max_distance=1)
    run = vo.run(
        "euroc",
        room_folders.euroc,
        "stereo",
        tmp_path,
        backend=require_program(backend),
        options=strict,
        count=3,
    )
    assert len(run.timestamps) == 0
    assert run.settings["stereo_max_distance"] == 1


def test_refuses_negative_frames() -> None:
    """A negative first frame or count is refused."""
    with pytest.raises(InvalidInputError, match="at least 0"):
        vo.run("tum", SEQUENCES / "tum_camera", "rgbd", "unused", count=-1)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_a_failed_program_raises_its_error(backend: str, tmp_path: Path) -> None:
    """A native run that fails raises the error of its kind, with the program's message."""
    with pytest.raises(InvalidInputError, match="no such file"):
        vo.run(
            "euroc",
            SEQUENCES / "euroc",
            "stereo",
            tmp_path,
            backend=require_program(backend),
            count=1,
        )


@pytest.mark.parametrize(
    ("code", "error"),
    [(vo.EXIT_FAILED, InvalidInputError), (vo.EXIT_FAILED + 1, DegenerateError),
     (vo.EXIT_FAILED + 2, NotConvergedError)],
)  # fmt: skip
def test_a_programs_exit_code_names_its_error(
    code: int, error: type[Exception], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A failed program's exit code raises the error of its kind, with the program's message."""
    program = tmp_path / "slamlab_vo"
    program.write_text(f"#!/bin/sh\necho 'slamlab_vo: it failed' >&2\nexit {code}\n")
    program.chmod(0o755)
    monkeypatch.setattr(backends, "program", lambda backend, name: program)
    with pytest.raises(error, match="it failed"):
        vo.run("tum", SEQUENCES / "tum_camera", "rgbd", tmp_path / "run", backend="cpp", count=0)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_a_program_refuses_a_bad_command_line(backend: str) -> None:
    """A command line the program cannot read exits with the usage, and code EXIT_USAGE."""
    program = backends.program(require_program(backend), vo.PROGRAM)
    for arguments in (["--dataset", "tum"], ["--option", "windows=7"], ["--mode"]):
        done = subprocess.run(
            [str(program), *arguments], capture_output=True, text=True, check=False
        )
        assert done.returncode == vo.EXIT_USAGE
        assert "usage: slamlab_vo" in done.stderr
