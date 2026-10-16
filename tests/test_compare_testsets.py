"""Tests for scripts/compare_testsets.py: the found room measured with relocalisation off and on."""

from __future__ import annotations

import dataclasses
import importlib
import json
from pathlib import Path
from types import ModuleType

import pytest

from slam_it.edge.options import Options
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
    default = Options()
    replaced = {
        item.name: getattr(FOUND_OPTIONS, item.name)
        for item in dataclasses.fields(Options)
        if getattr(FOUND_OPTIONS, item.name) != getattr(default, item.name)
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
            replaced={**replaced, "relocalize": relocalize},
        )  # fmt: skip
        (out / "results.jsonl").write_text(json.dumps(result) + "\n")
    return base / "off", base / "on", config


def test_a_stretch_ends_relocalised_recovered_lost_or_with_the_sequence(script: ModuleType) -> None:
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
    assert script.stretches(diagnostics) == [
        ("relocalised", 2, 2.0), ("recovered", 1, 1.0), ("lost", 2, 1.0), ("end", 1, 0.0),
    ]  # fmt: skip


def test_a_frame_is_relocalised_tried_placed_or_neither(script: ModuleType) -> None:
    """What a frame needed, read from its diagnostic."""
    summary = {"words": 10, "placed": False}
    assert [
        script.kind({"placed_by": "relocalization", "relocalization": {**summary, "placed": True}}),
        script.kind({"placed_by": None, "relocalization": summary}),
        script.kind({"placed_by": "motion", "relocalization": None}),
        script.kind({"placed_by": None, "relocalization": None}),
    ] == ["relocalised", "tried", "placed", "other"]


def test_a_batch_names_its_failed_sequences_and_groups_by_dataset_and_mode(
    script: ModuleType, tmp_path: Path
) -> None:
    """A failed sequence is named apart; a group is the name up to its first ``-``, and the mode."""
    rows = [{"name": "zju-B0", "mode": "monocular"}, {"name": "tum-x-rgbd", "failed": "why"}]
    (tmp_path / "results.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    batch = script.read_batch(tmp_path)
    assert (list(batch.results), batch.failed) == (["zju-B0"], ["tum-x-rgbd"])
    assert script.group(batch.results["zju-B0"]) == "zju monocular"


def test_the_found_room_compared_off_and_on(
    script: ModuleType, batches: tuple[Path, Path, Path], capsys: pytest.CaptureFixture[str]
) -> None:
    """Off, its blank frames lose the track; on, they end found again, in the right place."""
    off, on, config = batches
    base, new = script.read_batch(off), script.read_batch(on)
    names = ["room-found"]
    # Off: the frames after the blank ones fail too, the local fallback out of reach, until the
    # map is lost after lost_frames failures. On: the blank frames end relocalised.
    lost_frames = Options().lost_frames
    assert (
        script.endings(base, names)[2]
        == f"| room rgbd | 0 / 0 / 1 / 0 | 0 / 0 / {lost_frames} / 0 | 0 |"
    )
    assert (
        script.endings(new, names)[2]
        == f"| room rgbd | 1 / 0 / 0 / 0 | {FOUND_BLANKS} / 0 / 0 / 0 | 0 |"
    )
    (placed,) = script.against_truth(new, names, {"room-found": config.parent / "room"})[2:]
    cells = [cell.strip() for cell in placed.strip("|").split("|")]
    assert (cells[0], cells[2]) == ("room-found", "0")
    assert float(cells[3]) < 100 * ATE_BOUND["rgbd"]
    assert script.main([str(off), str(on), "--config", str(config)]) == 0
    out = capsys.readouterr().out
    assert out.startswith("1 runs compared")
    for title in (
        "By group",
        "The runs that moved",
        "How failures ended, new",
        "Latency by frame, new",
    ):
        assert f"## {title}" in out
    assert "| room rgbd | relocalised | 1 |" in out
