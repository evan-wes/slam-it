"""Tests for scripts/measure_sharded.py: the room measured at two seeds side by side; its plan."""

from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
from types import ModuleType

import pytest

from slam_it.eval import runs
from slam_it.foundations.errors import InvalidInputError
from tests.edge import room
from tests.edge.support import FRAMES
from tests.orchestration.conftest import require_program
from tests.orchestration.datasets import write_tum

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture
def script(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import scripts/measure_sharded.py by name, as its workers find it, with its neighbour."""
    monkeypatch.syspath_prepend(str(REPO / "scripts"))
    return importlib.import_module("measure_sharded")


@pytest.fixture(scope="module")
def config(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Write the room as a TUM-format folder, and a config listing it in RGB-D."""
    base = tmp_path_factory.mktemp("sharded")
    write_tum(base / "room", room.sequence(FRAMES))
    path = base / "testset.toml"
    path.write_text(
        f'[[sequence]]\nname = "room"\nfolder = "{base / "room"}"\nmode = "rgbd"\n'
        'reference = "its own poses"\n'
    )
    return path


def test_workers_take_the_highest_numbered_cpus(script: ModuleType) -> None:
    """All the CPUs offered by default; else as many as asked, the highest-numbered."""
    offered = {0, 1, 2, 3, 8, 9}
    assert script.cpus_for(None, offered) == [0, 1, 2, 3, 8, 9]
    assert script.cpus_for(2, offered) == [8, 9]
    for jobs in (0, 7):
        with pytest.raises(InvalidInputError, match="--jobs: expected 1 to 6"):
            script.cpus_for(jobs, offered)


def test_a_sequences_frames_are_its_listed_images(script: ModuleType, tmp_path: Path) -> None:
    """Comments and blank lines are not frames; a folder without the list has none."""
    (tmp_path / "rgb.txt").write_text("# timestamp filename\n0.0 rgb/0.png\n\n0.1 rgb/1.png\n")
    assert script.frames_of(tmp_path) == 2
    assert script.frames_of(tmp_path / "missing") == 0


def test_the_longest_sequences_run_first(script: ModuleType) -> None:
    """Every seed's run of every sequence, longest first, then by seed, then in config order."""
    entries = [
        script.measure_testset.Entry(name, Path(name), "rgbd", "r") for name in ("a", "b", "c")
    ]
    jobs = script.schedule(entries, [0, 1], {"a": 10, "b": 30, "c": 10})
    assert [(seed, e.name) for seed, e in jobs] == [
        (0, "b"), (1, "b"), (0, "a"), (0, "c"), (1, "a"), (1, "c"),
    ]  # fmt: skip


@pytest.mark.parametrize(
    ("extra", "message"),
    [
        (["--option", "seed=1"], "the seeds are --seeds"),
        (["--seeds", "0", "0"], "a seed given twice"),
        (["--seeds", "-1"], "seed"),
        (["--jobs", "0"], "--jobs: expected 1 to"),
    ],
)
def test_main_refuses_a_seed_twice_or_out_of_range_and_jobs_beyond_the_cpus(
    extra: list[str], message: str, script: ModuleType, config: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:  # fmt: skip
    """A seed is given through ``--seeds`` once each; the workers are at most the CPUs."""
    out = config.with_name("refused")
    with pytest.raises(SystemExit):
        script.main([str(config), str(out), *extra])
    assert message in capsys.readouterr().err
    assert not out.exists()  # refused before any folder is made


def test_each_seed_is_measured_into_its_own_folder(
    script: ModuleType, config: Path, tmp_path: Path
) -> None:
    """Two seeds side by side: each seed's results, its runs at that seed, the options passed on."""
    require_program("cpp")
    out = tmp_path / "out"
    jobs = str(min(2, len(os.sched_getaffinity(0))))
    arguments = [str(config), str(out), "--seeds", "0", "1", "--jobs", jobs]
    assert script.main([*arguments, "--option", "relocalize=false"]) == 0
    for seed in (0, 1):
        lines = (out / f"seed-{seed}" / "results.jsonl").read_text().splitlines()
        assert len(lines) == 1
        result = json.loads(lines[0])
        assert result["options"] == {"relocalize": False, "seed": seed}
        assert (result["name"], result["tracked"]) == ("room", FRAMES)
        settings = runs.read_run(out / f"seed-{seed}" / "room" / "run").settings
        assert (settings["seed"], settings["relocalize"]) == (seed, False)
