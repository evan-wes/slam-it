"""Tests for scripts/compare_testsets.py: the found room measured with relocalisation off and on."""

from __future__ import annotations

import importlib
import json
import shutil
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from slam_it.edge.options import Options
from slam_it.foundations.errors import InvalidInputError
from tests.edge import room
from tests.edge.support import ATE_BOUND, FRAMES
from tests.orchestration.conftest import require_program
from tests.orchestration.datasets import FOUND_BLANKS, FOUND_OPTIONS, found_frames, write_tum

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture
def script(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import scripts/compare_testsets.py by name, with its neighbour the measure."""
    monkeypatch.syspath_prepend(str(REPO / "scripts"))
    return importlib.import_module("compare_testsets")


@pytest.fixture(scope="module")
def batches(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path, Path]:
    """Measure the found room with its own settings, relocalisation off then on.

    Returns the two measures' folders, each of one sequence, ``room-found``, in RGB-D, and their
    config.
    """
    require_program("cpp")
    base = tmp_path_factory.mktemp("compare")
    write_tum(base / "room", found_frames(room.sequence(FRAMES)))
    config = base / "testset.toml"
    config.write_text(
        f'[[sequence]]\nname = "room-found"\nfolder = "{base / "room"}"\nmode = "rgbd"\n'
        'reference = "its own poses"\n'
    )
    default = Options().settings()
    replaced = {
        name: value for name, value in FOUND_OPTIONS.settings().items() if value != default[name]
    }
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(REPO / "scripts"))
        measure = importlib.import_module("measure_testset")
    (entry,) = measure.read_config(config)
    for name, relocalize in (("off", False), ("on", True)):
        out = base / name
        out.mkdir()
        result = measure.measure(
            entry, out / entry.name, backend="cpp", by_size=False, nominal=False,
            replaced={**replaced, "relocalization.enabled": relocalize},
        )  # fmt: skip
        (out / "results.jsonl").write_text(json.dumps(result) + "\n")
    return base / "off", base / "on", config


def test_a_batch_names_its_failed_sequences_and_groups_by_dataset_and_mode(
    script: ModuleType, tmp_path: Path
) -> None:
    """A failed sequence is named apart; a group is the name up to its first ``-``, and the mode."""
    details = dict.fromkeys(script.measure_testset.DETAILS)
    rows = [
        {"name": "zju-B0", "mode": "monocular", **details},
        {"name": "tum-x-rgbd", "failed": "why"},
    ]
    (tmp_path / "results.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    batch = script.read_batch(tmp_path)
    assert (list(batch.results), batch.failed) == (["zju-B0"], ["tum-x-rgbd"])
    assert script.group(batch.results["zju-B0"]) == "zju monocular"


def test_a_result_without_its_runs_details_is_refused(script: ModuleType, tmp_path: Path) -> None:
    """A measure made before results carried their run's details is compared once given them."""
    rows = [{"name": "zju-B0", "mode": "monocular"}, {"name": "zju-B1", "mode": "monocular"}]
    (tmp_path / "results.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(InvalidInputError, match="no run details for zju-B0, zju-B1"):
        script.read_batch(tmp_path)


def test_the_found_room_compared_off_and_on(
    script: ModuleType, batches: tuple[Path, Path, Path], capsys: pytest.CaptureFixture[str]
) -> None:
    """Off, its blank frames lose the track; on, they end found again, in the right place."""
    off, on, _ = batches
    base, new = script.read_batch(off), script.read_batch(on)
    names = ["room-found"]
    # Off: the frames after the blank ones fail too, the local fallback out of reach, until the
    # map is lost after lost_frames failures. On: the blank frames end relocalised.
    lost_frames = Options().loss.lost_frames
    assert (
        script.endings(base, names)[2]
        == f"| room rgbd | 0 / 0 / 1 / 0 | 0 / 0 / {lost_frames} / 0 | 0 |"
    )
    assert (
        script.endings(new, names)[2]
        == f"| room rgbd | 1 / 0 / 0 / 0 | {FOUND_BLANKS} / 0 / 0 / 0 | 0 |"
    )
    (placed,) = script.against_truth(new, names)[2:]
    cells = [cell.strip() for cell in placed.strip("|").split("|")]
    assert (cells[0], cells[2]) == ("room-found", "0")
    assert float(cells[3]) < 100 * ATE_BOUND["rgbd"]
    assert script.main(["compare", str(off), str(on)]) == 0
    out = capsys.readouterr().out
    assert out.startswith("1 runs compared")
    for title in (
        "By group",
        "The runs that moved",
        "How failures ended, new",
        "Latency by frame, new",
        "Relocalised frames against the ground truth, new",
    ):
        assert f"## {title}" in out
    assert "| room rgbd | relocalised | 1 |" in out


def _undetailed(script: ModuleType, on: Path, copy: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Copy the measure in `on` to `copy`, its one result without its run's details.

    Returns the result as measured, and as written to the copy.
    """
    shutil.copytree(on, copy)
    (measured,) = [json.loads(line) for line in (on / "results.jsonl").read_text().splitlines()]
    bare = {k: v for k, v in measured.items() if k not in script.measure_testset.DETAILS}
    (copy / "results.jsonl").write_text(json.dumps(bare) + "\n")
    return measured, bare


def test_a_measure_made_without_its_runs_details_is_given_them(
    script: ModuleType, batches: tuple[Path, Path, Path], tmp_path: Path
) -> None:
    """``summarize`` reads each run and its truth for the details the measure itself records."""
    _, on, config = batches
    copy = tmp_path / "on"
    measured, bare = _undetailed(script, on, copy)
    assert script.main(["summarize", str(copy), str(config)]) == 0
    assert json.loads((copy / "results.jsonl").read_text()) == measured
    assert json.loads((copy / script.UNDETAILED).read_text()) == bare
    assert script.summarize(copy, config) == 0  # nothing left to give


def test_summarize_reads_each_sequences_truth_as_its_dataset(
    script: ModuleType,
    batches: tuple[Path, Path, Path],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A sequence's truth is read as its config names its dataset (B3's review panel, H10)."""
    _, on, config = batches
    copy = tmp_path / "on"
    _undetailed(script, on, copy)
    euroc = tmp_path / "euroc.toml"
    euroc.write_text(config.read_text() + 'dataset = "euroc"\n')
    read: list[tuple[str, Path]] = []
    monkeypatch.setattr(script.runs, "ground_truth", lambda *given: read.append(given))
    assert script.summarize(copy, euroc) == 1
    ((folder,),) = [e.folders for e in script.measure_testset.read_config(config)]
    assert read == [("euroc", folder)]
