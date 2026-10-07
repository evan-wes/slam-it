"""Whole sequences through every backend.

The room from its TUM and EuRoC folders, the native programs' command line, and their failures.
"""

from __future__ import annotations

import subprocess
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
from tests.edge.conftest import ATE_BOUND, FRAMES, MONOCULAR_START, OPTIONS
from tests.edge.support import SOLVER_TOLERANCE, assert_close
from tests.orchestration.conftest import RoomRun, require_program
from tests.orchestration.datasets import CASES, GAP_OPTIONS, Room

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


def test_the_gap_forces_a_recovery(
    gap_folders: Room, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Across the gap the motion model fails, PnP recovers, and every frame is tracked."""
    recovered: list[bool] = []
    relocalize = tracking.relocalize_locally

    def counting(
        m: Map, camera: frames.Camera, frame: frames.Frame, options: Options
    ) -> frames.Tracked | None:
        found = relocalize(m, camera, frame, options)
        recovered.append(found is not None)
        return found

    monkeypatch.setattr(tracking, "relocalize_locally", counting)
    run = vo.run("tum", gap_folders.tum, "rgbd", tmp_path, options=GAP_OPTIONS)
    assert recovered
    assert all(recovered)
    assert all(d["state"] == "tracking" for d in run.diagnostics)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize(("dataset", "mode"), CASES)
def test_the_backends_agree_across_the_gap(
    dataset: str, mode: str, backend: str, gap_runs: RoomRun
) -> None:
    """Through a recovery, a native program makes Python's decisions as on the whole room."""
    _assert_same_run(gap_runs(dataset, mode, backend), gap_runs(dataset, mode, "python"))


GOLDEN = load_fixture("edge/room_runs.json")
"""What the pipeline decided on the room and the gap room (``generate_room_runs.py``)."""


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
    decisions = [[d["state"], d["tracked"], d["keyframe"]] for d in run.diagnostics]
    assert decisions == expected["diagnostics"]
    np.testing.assert_array_equal(run.timestamps, expected["timestamps"])
    np.testing.assert_array_equal(run.keyframe_timestamps, expected["keyframe_timestamps"])
    assert len(run.points) == expected["points"]
    assert_matches_fixture(run.poses, expected["poses"], GOLDEN["tolerance"])


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
