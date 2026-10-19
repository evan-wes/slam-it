"""Tests for scripts/run_vo.py: its --option reader is the programs' (vo.parse_setting)."""

from __future__ import annotations

import importlib.util
from dataclasses import replace
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from slam_it.edge.options import Options
from slam_it.eval import runs
from tests.conftest import FIXTURES

REPO = Path(__file__).resolve().parents[1]


def _script() -> ModuleType:
    """Import scripts/run_vo.py as a module."""
    spec = importlib.util.spec_from_file_location("run_vo", REPO / "scripts/run_vo.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_options_sets_the_pairs_fields() -> None:
    """KEY=VALUE pairs set those settings, each of its type; the rest keep their defaults."""
    options = _script().parse_options(
        ["covisibility.local_neighbours=7", "stereo.close_depth=35.5"]
    )
    assert (options.covisibility.local_neighbours, options.stereo.close_depth) == (7, 35.5)
    assert options.features.n_features == Options().features.n_features


@pytest.mark.parametrize(
    "pair", ["covisibility.local_neighbours=1_000", "stereo.close_depth=inf", "nothing=1", "window"]
)
def test_parse_options_refuses_as_the_programs_do(pair: str) -> None:
    """A pair the programs refuse ends the script with the reader's message."""
    with pytest.raises(SystemExit, match="--option: "):
        _script().parse_options([pair])


def test_score_reports_a_run_on_a_line_unscored(capsys: pytest.CaptureFixture[str]) -> None:
    """Centres on a line leave the alignment's rotation free: the run is reported, not scored."""
    run = runs.read_run(FIXTURES / "eval" / "run")
    poses = np.repeat(np.eye(4)[None], len(run.poses), axis=0)
    poses[:, 0, 3] = np.arange(len(poses))
    on_a_line = replace(run, poses=poses)
    assert _script().score(on_a_line, (run.timestamps, poses)) is None
    assert "not scored: " in capsys.readouterr().out
