"""Tests for scripts/measure_testset.py: the room measured as a test set's sequence; its config."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from slam_it.edge.options import Options
from slam_it.eval import lenses, runs, tum
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo
from tests.edge import room
from tests.edge.support import ATE_BOUND, FRAMES
from tests.eval.synthetic_runs import line_run
from tests.orchestration.conftest import require_program
from tests.orchestration.datasets import (
    FOUND_AT,
    FOUND_BLANKS,
    FOUND_OPTIONS,
    found_frames,
    write_euroc,
    write_tum,
)

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
        f'[[sequence]]\nname = "{name}"\nfolders = ["{sequence}"]\nmode = "rgbd"\n'
        'reference = "the room\'s own poses"\n'
    )
    return path


def test_a_stretch_ends_relocalised_recovered_lost_or_with_the_sequence() -> None:
    """Untracked frames are those tracking failed, a frame placed that then failed among them."""
    states = [
        (0, "tracking", "initialization", 200),
        (0, "tracking", "motion", 0),  # placed, then failed the local map
        (0, "tracking", None, 0),
        (0, "tracking", "relocalization", 80),
        (0, "tracking", None, 0),
        (0, "tracking", "motion", 150),
        (0, "tracking", None, 0),
        (0, "lost", None, 0),
        (None, "not_initialized", None, 0),
        (1, "tracking", "initialization", 200),
        (1, "tracking", None, 0),
    ]
    diagnostics = [
        {"timestamp": float(k), "map": m, "state": state, "placed_by": placed, "tracked": n}
        for k, (m, state, placed, n) in enumerate(states)
    ]
    assert _script().stretches(diagnostics) == [
        ("relocalised", 2, 2.0), ("recovered", 1, 1.0), ("lost", 2, 1.0), ("end", 1, 0.0),
    ]  # fmt: skip


def test_a_frame_is_relocalised_tried_placed_or_neither() -> None:
    """What a frame needed, read from its diagnostic."""
    summary = {"words": 10, "placed": False}
    kind = _script().kind
    assert [
        kind({"placed_by": "relocalization", "relocalization": {**summary, "placed": True}}),
        kind({"placed_by": None, "relocalization": summary}),
        kind({"placed_by": "motion", "relocalization": None}),
        kind({"placed_by": None, "relocalization": None}),
    ] == ["relocalised", "tried", "placed", "other"]


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
    assert result["options"] == {}
    assert (result["share"], result["tracked_from"], result["losses"]) == (1.0, 0.0, 0)
    assert [(s["map"], s["tracked"]) for s in result["segments"]] == [(0, FRAMES)]
    assert (result["resets"], result["relocalizations"]) == (0, 0)
    assert [m["map"] for m in result["map_ate"]] == [0]
    for key in ("ate", "ate_tracked", "ate_keyframes"):  # final, as tracked, the keyframes'
        assert result[key] < ATE_BOUND["rgbd"]
    assert result["loops"] == []  # the room closes none
    assert (result["matched"], result["reference"]) == (FRAMES, "the room's own poses")
    assert result["p95"] > 0.0
    assert result["deadline"] == pytest.approx(1 / 30, abs=1e-6)  # rgb.txt's microseconds
    assert result["late"] >= 0
    assert result["unjudged"] is None
    # Section 13's measures: in order, nothing waits or is refused, and the backlog is judged.
    assert (result["threaded"], result["locks"], result["budget"]) == (False, False, None)
    assert result["on_time"] == 1.0 - result["late"] / FRAMES
    assert len(result["late_frames"]) == result["late"]
    assert result["median"] <= result["p99"] <= result["worst"]
    assert (result["waited"], result["refused"], result["queue"], result["aborted"]) == (
        0.0, 0, 0, 0,
    )  # fmt: skip
    assert result["asked"] >= 1
    assert result["backlog"] >= 0.0
    # The run's details: nothing untracked, every frame placed (the first by the map's start),
    # and no relocalised frame to hold against the truth.
    assert (result["lost_frames"], result["stretches"]) == (Options().loss.lost_frames, [])
    assert {k: len(v) for k, v in result["latency"].items()} == {
        "relocalised": 0, "tried": 0, "placed": FRAMES, "other": 0,
    }  # fmt: skip
    assert result["relocalised_frames"] == []
    table = capsys.readouterr().out.splitlines()
    assert table[0].startswith("| sequence | mode |")
    assert table[2].startswith(f"| room | rgbd | 320x240 | 1000 | {FRAMES}/{FRAMES} (100%) |")
    assert "| 1 (0 reset) | 0 | 0 |" in table[2]
    assert table[0].count("|") == table[1].count("|") == table[2].count("|")


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


KEYS = 'name = "a"\nfolders = ["/a"]\nmode = "rgbd"\nreference = "r"\n'
"""A sequence's keys, all given."""


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("[[sequence]\n", "not TOML"),
        ('title = "nothing"\n', r"no \[\[sequence\]\]"),
        ("[[sequence]]\n" + KEYS.replace('mode = "rgbd"\n', ""), "expected the keys"),
        ("[[sequence]]\n" + KEYS + "extra = 1\n", "expected the keys"),
        ("[[sequence]]\n" + KEYS.replace('"r"', "1"), "expected strings"),
        ("[[sequence]]\n" + KEYS.replace("rgbd", "lidar"), "unknown mode 'lidar'"),
        ("[[sequence]]\n" + KEYS + 'dataset = "kitti"\n', "unknown dataset 'kitti'"),
        ("[[sequence]]\n" + KEYS + "[[sequence]]\n" + KEYS, "share a name"),
        ("[[sequence]]\n" + KEYS + 'folder = "/b"\n', "expected the keys"),
        ("[[sequence]]\n" + KEYS.replace('folders = ["/a"]\n', ""), "expected the keys"),
        ("[[sequence]]\n" + KEYS.replace('["/a"]', "[]"), "a list of them"),
        ("[[sequence]]\n" + KEYS.replace('["/a"]', '["/a", 1]'), "list of them"),
        ("[[sequence]]\n" + KEYS.replace('["/a"]', '"/a"'), "a list of them"),
    ],
)
def test_a_malformed_config_is_refused(text: str, message: str, tmp_path: Path) -> None:
    """Each refusal of a config.

    Not TOML, no sequence, a key missing or extra (``folder`` among them: ``folders`` alone
    says where a sequence is), a number for a string, a mode unknown, two sequences of one name;
    no folders, or folders not a list of paths.
    """
    path = tmp_path / "testset.toml"
    path.write_text(text)
    with pytest.raises(InvalidInputError, match=message):
        _script().read_config(path)


@pytest.mark.usefixtures("cpp")
def test_sequences_in_turn_are_measured_as_one_run(sequence: Path, tmp_path: Path) -> None:
    """The room twice, as two sessions: each its frames and map, scored on the truths in turn.

    Every map kept (``loss.session_keyframes`` 0), each session's frames end in its own map; the
    second session's truth, moved on with its frames, matches every frame of it.
    """
    config = tmp_path / "testset.toml"
    config.write_text(
        f'[[sequence]]\nname = "twice"\nfolders = ["{sequence}", "{sequence}"]\n'
        'mode = "rgbd"\nreference = "the room\'s own poses"\n'
    )
    out = tmp_path / "out"
    assert _script().main([str(config), str(out), "--option", "loss.session_keyframes=0"]) == 0
    result = json.loads((out / "results.jsonl").read_text())
    assert (result["frames"], result["tracked"], result["matched"]) == (2 * FRAMES,) * 3
    assert result["maps"] == [0, 1]
    first, second = result["sessions"]
    times = vo.open_source("tum", sequence, "rgbd").times
    assert second["offset"] == pytest.approx(times[-1] + np.median(np.diff(times)) - times[0])
    assert (first["offset"], first["closed"]) == (0.0, None)
    for k, session in enumerate((first, second)):
        assert (session["sequence"], session["frames"], session["tracked"]) == (
            "room", FRAMES, FRAMES,
        )  # fmt: skip
        assert session["maps"] == [k]
    assert second["closed"]["reset"] is False
    assert second["closed"]["keyframes"] > 0
    assert result["ate"] < ATE_BOUND["rgbd"]
    assert (result["loops"], result["merges"]) == ([], [])


def test_a_sequence_s_sessions_offsets_are_its_run_s(sequence: Path) -> None:
    """One folder keeps its own clock; the second of two is moved on past the first's end."""
    script = _script()
    once, twice = (script.Entry("room", (sequence,) * k, "rgbd", "r") for k in (1, 2))
    times = vo.open_source("tum", sequence, "rgbd").times
    assert script.offsets_of(once) == [0.0]
    assert script.offsets_of(twice) == vo.session_offsets([times, times])
    assert script.offsets_of(twice)[1] > times[-1] - times[0]


def test_the_truths_in_turn_ascend(sequence: Path) -> None:
    """Each session's truth is moved on by its offset; times not after those before are dropped."""
    times, poses = runs.ground_truth("tum", sequence) or pytest.fail("the room has its truth")
    moved = _script().truth_of("tum", [sequence, sequence], [0.0, 10.0])
    assert moved is not None
    np.testing.assert_array_equal(moved[0], np.concatenate([times, times + 10.0]))
    np.testing.assert_array_equal(moved[1], np.concatenate([poses, poses]))
    same = _script().truth_of("tum", [sequence, sequence], [0.0, 0.0])
    assert same is not None
    np.testing.assert_array_equal(same[0], times)  # the second's times, none after, all dropped
    untrue = REPO / "tests" / "fixtures" / "eval" / "sequences" / "tum_camera"  # no truth file
    assert _script().truth_of("tum", [untrue], [0.0]) is None


def test_a_euroc_sequence_is_measured_from_its_own_folder(tmp_path: Path) -> None:
    """A sequence naming its dataset ``euroc`` runs as EuRoC, in stereo.

    The room's EuRoC folder has no ground truth, which the result says; the room closes no loop.
    """
    folder = tmp_path / "room_euroc"
    write_euroc(folder, room.sequence(FRAMES, stereo=True))
    config = tmp_path / "testset.toml"
    config.write_text(
        f'[[sequence]]\nname = "room"\nfolders = ["{folder}"]\nmode = "stereo"\n'
        'reference = "the room\'s own poses"\ndataset = "euroc"\n'
    )
    assert _script().main([str(config), str(tmp_path / "out")]) == 0
    result = json.loads((tmp_path / "out" / "results.jsonl").read_text())
    assert (result["dataset"], result["mode"], result["tracked"]) == ("euroc", "stereo", FRAMES)
    assert (result["ate"], result["unscored"]) == (None, "no ground truth")
    assert result["loops"] == []


def test_the_keyframes_ate_is_their_own() -> None:
    """The final keyframes' ATE is scored on them alone: a keyframe moved 1 m shows."""
    run, truth = line_run(30)
    assert _script().keyframes_ate(run, truth) == pytest.approx(0.0, abs=1e-9)  # the truth's
    moved = run.keyframe_poses.copy()
    moved[1, 2, 3] += 1.0
    error = _script().keyframes_ate(replace(run, keyframe_poses=moved), truth)
    assert error is not None
    assert error > 0.1


def test_the_template_is_a_config_the_script_reads() -> None:
    """The repository's template lists the test set's kinds of sequence, every key given."""
    entries = _script().read_config(REPO / "scripts" / "testset.template.toml")
    assert [e.mode for e in entries] == ["monocular", "monocular", "monocular", "rgbd"]


@pytest.mark.parametrize(
    ("extra", "message"),
    [
        (["--only", "other"], "not in the config"),
        (["--option", "nothing=1"], "no setting 'nothing'"),
        ([], "new or empty"),
    ],
)
def test_main_refuses_an_unknown_name_or_a_used_folder(
    extra: list[str], message: str, sequence: Path, tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:  # fmt: skip
    """``--only`` names a listed sequence, ``--option`` a setting; the folder is new or empty."""
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


def test_options_are_read_as_json_and_checked_as_a_run_checks_them() -> None:
    """A value is JSON where it parses; each must be one its setting takes."""
    read = _script().read_options
    assert read(["relocalization.enabled=false", "loss.relocalize_seconds=0.5", "seed=2"]) == {
        "relocalization.enabled": False, "loss.relocalize_seconds": 0.5, "seed": 2,
    }  # fmt: skip
    assert read([]) == {}


@pytest.mark.parametrize(
    ("given", "message"),
    [
        (["relocalization.enabled"], "expected KEY=VALUE"),
        (["nothing=1"], "no setting 'nothing'"),
        (["seed=1", "seed=2"], "seed given twice"),
        (["relocalization.enabled=maybe"], "relocalization.enabled"),
        (["seed=-1"], "seed"),
    ],
)
def test_a_bad_option_is_refused(given: list[str], message: str) -> None:
    """Not KEY=VALUE, no such setting, one given twice, a value its setting does not take."""
    with pytest.raises(InvalidInputError, match=message):
        _script().read_options(given)


@pytest.mark.usefixtures("cpp")
def test_an_option_replaces_a_setting_in_every_run(sequence: Path, tmp_path: Path) -> None:
    """Each result names the settings it replaced, and its run's settings hold them."""
    config = _config(tmp_path / "testset.toml", sequence)
    out = tmp_path / "out"
    replaced = ["--option", "relocalization.enabled=false", "--option", "seed=3"]
    assert _script().main([str(config), str(out), *replaced]) == 0
    result = json.loads((out / "results.jsonl").read_text())
    assert result["options"] == {"relocalization.enabled": False, "seed": 3}
    settings = runs.read_run(out / "room" / "run").settings
    assert (settings["relocalization"]["enabled"], settings["seed"]) == (False, 3)


@pytest.mark.usefixtures("cpp")
def test_the_found_room_relocalises_unless_an_option_turns_it_off(tmp_path: Path) -> None:
    """The golden record's found room, measured with its settings: found again, or not."""
    write_tum(tmp_path / "found", found_frames(room.sequence(FRAMES)))
    config = _config(tmp_path / "testset.toml", tmp_path / "found", name="found")
    default = Options().settings()
    found = [
        f"--option={name}={json.dumps(value)}"
        for name, value in FOUND_OPTIONS.settings().items()
        if value != default[name]
    ]
    measured = {}
    for name, extra in (("on", []), ("off", ["--option=relocalization.enabled=false"])):
        out = tmp_path / name
        assert _script().main([str(config), str(out), "--features", "default", *found, *extra]) == 0
        measured[name] = json.loads((out / "results.jsonl").read_text())
    # Relocalised on the first frame after the blank ones, as the golden record has it.
    assert measured["on"]["relocalizations"] == 1
    placed = [
        k
        for k, entry in enumerate(runs.read_run(tmp_path / "on" / "found" / "run").diagnostics)
        if entry.get("placed_by") == runs.RELOCALIZED
    ]
    assert placed == [FOUND_AT + FOUND_BLANKS]
    assert measured["off"]["relocalizations"] == 0
