"""Tests for scripts/measure_testset.py: the room measured as a test set's sequence; its config."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from slam_it.eval import lenses, runs, tum
from slam_it.foundations.errors import InvalidInputError
from tests.edge import room
from tests.edge.support import ATE_BOUND, FRAMES
from tests.orchestration.conftest import require_program
from tests.orchestration.datasets import write_tum

REPO = Path(__file__).resolve().parents[1]


def _script() -> ModuleType:
    """Import scripts/measure_testset.py as a module."""
    spec = importlib.util.spec_from_file_location(
        "measure_testset", REPO / "scripts/measure_testset.py"
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def sequence(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Write the room as a TUM-format folder with depth and its truth."""
    folder = tmp_path_factory.mktemp("room") / "room"
    write_tum(folder, room.sequence(FRAMES))
    return folder


@pytest.fixture(scope="module")
def cpp() -> None:
    """Skip without the C++ program; its first import, which may rebuild it, is not captured."""
    require_program("cpp")


def _config(path: Path, sequence: Path, name: str = "room") -> Path:
    """Write a config of one sequence, the room in RGB-D."""
    path.write_text(
        f'[[sequence]]\nname = "{name}"\nfolder = "{sequence}"\nmode = "rgbd"\n'
        'reference = "the room\'s own poses"\n'
    )
    return path


@pytest.mark.usefixtures("cpp")
def test_a_sequence_is_run_measured_and_tabled(
    sequence: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Tracked whole from its first frame, scored against its truth, its latency judged."""
    config = _config(tmp_path / "testset.toml", sequence)
    assert _script().main([str(config), str(tmp_path / "out")]) == 0
    lines = (tmp_path / "out" / "results.jsonl").read_text().splitlines()
    assert len(lines) == 1
    result = json.loads(lines[0])
    assert (result["name"], result["mode"], result["camera"], result["size"]) == (
        "room", "rgbd", "exact", [320, 240],
    )  # fmt: skip
    assert (result["features"], result["frames"], result["tracked"]) == (1000, FRAMES, FRAMES)
    assert (result["share"], result["tracked_from"], result["losses"]) == (1.0, 0.0, 0)
    assert [(s["map"], s["tracked"]) for s in result["segments"]] == [(0, FRAMES)]
    assert result["resets"] == 0
    assert [m["map"] for m in result["map_ate"]] == [0]
    assert result["ate"] < ATE_BOUND["rgbd"]
    assert (result["matched"], result["reference"]) == (FRAMES, "the room's own poses")
    assert result["p95"] > 0.0
    assert result["deadline"] == pytest.approx(1 / 30, abs=1e-6)  # rgb.txt's microseconds
    assert result["late"] >= 0
    assert result["unjudged"] is None
    table = capsys.readouterr().out.splitlines()
    assert table[0].startswith("| sequence | mode |")
    assert table[2].startswith(f"| room | rgbd | 320x240 | 1000 | {FRAMES}/{FRAMES} (100%) |")
    assert "| 1 (0 reset) | 0 |" in table[2]


@pytest.mark.usefixtures("cpp")
def test_a_nominal_camera_is_guessed_on_links_to_the_frames(sequence: Path, tmp_path: Path) -> None:
    """The guess is a typical phone's view at the frames' size; frames and depth are links."""
    config = _config(tmp_path / "testset.toml", sequence)
    assert _script().main([str(config), str(tmp_path / "out"), "--camera", "nominal"]) == 0
    folder = tmp_path / "out" / "room" / "sequence"
    assert (folder / "rgb").is_symlink()
    assert (folder / "depth").resolve() == (sequence / "depth").resolve()
    camera, own = tum.read_camera(folder), tum.read_camera(sequence)
    guess = lenses.nominal(320, 240)
    np.testing.assert_allclose(camera.intrinsics, [guess.fx, guess.fy, guess.cx, guess.cy])
    assert camera.depth_scale == own.depth_scale
    assert camera.bf is not None
    assert own.bf is not None
    assert camera.bf / guess.fx == pytest.approx(own.bf / own.intrinsics[0])  # the baseline kept
    result = json.loads((tmp_path / "out" / "results.jsonl").read_text())
    assert result["camera"] == "nominal"


def test_a_sequence_that_fails_is_recorded_and_the_rest_measured(tmp_path: Path) -> None:
    """A folder that is not there fails its sequence alone."""
    config = _config(tmp_path / "testset.toml", tmp_path / "missing", name="gone")
    assert _script().main([str(config), str(tmp_path / "out")]) == 0
    result = json.loads((tmp_path / "out" / "results.jsonl").read_text())
    assert result["name"] == "gone"
    assert "camera.txt" in result["failed"]


KEYS = 'name = "a"\nfolder = "/a"\nmode = "rgbd"\nreference = "r"\n'
"""A sequence's keys, all given."""


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("[[sequence]\n", "not TOML"),
        ('title = "nothing"\n', r"no \[\[sequence\]\]"),
        ("[[sequence]]\n" + KEYS.replace('mode = "rgbd"\n', ""), "expected the keys"),
        ("[[sequence]]\n" + KEYS + "extra = 1\n", "expected the keys"),
        ("[[sequence]]\n" + KEYS.replace('"/a"', "1"), "expected strings"),
        ("[[sequence]]\n" + KEYS.replace("rgbd", "lidar"), "unknown mode 'lidar'"),
        ("[[sequence]]\n" + KEYS + "[[sequence]]\n" + KEYS, "share a name"),
    ],
)
def test_a_malformed_config_is_refused(text: str, message: str, tmp_path: Path) -> None:
    """Each refusal of a config.

    Not TOML, no sequence, a key missing or extra, a number for a path, a mode unknown, two
    sequences of one name.
    """
    path = tmp_path / "testset.toml"
    path.write_text(text)
    with pytest.raises(InvalidInputError, match=message):
        _script().read_config(path)


def test_the_template_is_a_config_the_script_reads() -> None:
    """The repository's template lists the test set's kinds of sequence, every key given."""
    entries = _script().read_config(REPO / "scripts" / "testset.template.toml")
    assert [e.mode for e in entries] == ["monocular", "monocular", "monocular", "rgbd"]


@pytest.mark.parametrize(
    ("extra", "message"), [(["--only", "other"], "not in the config"), ([], "new or empty")]
)
def test_main_refuses_an_unknown_name_or_a_used_folder(
    extra: list[str], message: str, sequence: Path, tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:  # fmt: skip
    """``--only`` names a sequence the config lists; the results go to a new or empty folder."""
    config = _config(tmp_path / "testset.toml", sequence)
    out = tmp_path / "out"
    if not extra:
        out.mkdir()
        (out / "kept.txt").write_text("not ours")
    with pytest.raises(SystemExit):
        _script().main([str(config), str(out), *extra])
    assert message in capsys.readouterr().err


@pytest.mark.usefixtures("cpp")
def test_a_reference_too_sparse_for_a_relative_error_still_scores_the_ate(
    sequence: Path, tmp_path: Path
) -> None:
    """A reference at a third of the frame rate, on the frames' times: every third frame scored."""
    folder = tmp_path / "sparse"
    folder.mkdir()
    for name in ("rgb", "rgb.txt", "depth", "depth.txt", "camera.txt"):
        (folder / name).symlink_to(sequence / name)
    times, poses = runs.read_trajectory(sequence / "groundtruth.txt")
    runs.write_trajectory(folder / "groundtruth.txt", times[::3], poses[::3])
    config = _config(tmp_path / "testset.toml", folder)
    assert _script().main([str(config), str(tmp_path / "out")]) == 0
    result = json.loads((tmp_path / "out" / "results.jsonl").read_text())
    assert (result["matched"], result["unscored"]) == (len(times[::3]), None)
    assert result["ate"] < ATE_BOUND["rgbd"]


@pytest.mark.usefixtures("cpp")
def test_a_run_too_short_to_judge_says_why(tmp_path: Path) -> None:
    """One frame has no period to judge its latency against: said, not hidden."""
    write_tum(tmp_path / "one", room.sequence(1))
    config = _config(tmp_path / "testset.toml", tmp_path / "one")
    assert _script().main([str(config), str(tmp_path / "out")]) == 0
    result = json.loads((tmp_path / "out" / "results.jsonl").read_text())
    assert result["p95"] is None
    assert "two frames" in result["unjudged"]


def test_the_measure_never_needs_the_video_import() -> None:
    """Measuring TUM-format folders needs neither PyAV nor the demo's worker."""
    script = str(REPO / "scripts/measure_testset.py")
    probe = (
        "import importlib.util, sys\n"
        f"spec = importlib.util.spec_from_file_location('m', {script!r})\n"
        "spec.loader.exec_module(importlib.util.module_from_spec(spec))\n"
        "print(sorted(m for m in ('av', 'slam_it.eval.video', 'slam_it.orchestration.worker')"
        " if m in sys.modules))\n"
    )
    found = subprocess.run(
        [sys.executable, "-c", probe], capture_output=True, text=True, check=True
    )
    assert found.stdout.strip() == "[]"
