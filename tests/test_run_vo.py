"""Tests for scripts/run_vo.py: its --option reader is the programs' (vo.parse_setting)."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest

from slam_it_all.edge.options import Options

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
    options = _script().parse_options(["window=7", "close_depth=35.5"])
    assert (options.window, options.close_depth) == (7, 35.5)
    assert options.n_features == Options().n_features


@pytest.mark.parametrize("pair", ["window=1_000", "close_depth=inf", "nothing=1", "window"])
def test_parse_options_refuses_as_the_programs_do(pair: str) -> None:
    """A pair the programs refuse ends the script with the reader's message."""
    with pytest.raises(SystemExit, match="--option: "):
        _script().parse_options([pair])
