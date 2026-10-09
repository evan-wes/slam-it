"""Generate ``bal.json``: a tiny BAL file as the Python reader converts it.

The native BAL readers (the Rust example's, ``rust/slam-it-all-foundations/examples/bal/reader.rs``,
and the C++ benchmark's, ``cpp/bench/foundations/bal_reader.hpp``) copy
``slam_it_all.eval.readers.read_bal``'s conversion; their tests read this fixture, so the three
cannot drift apart. Run from the repository root after changing ``bal_tiny.txt``::

    python -m tests.fixtures.eval.generate_bal
"""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path
from typing import Any

import numpy as np

from slam_it_all.eval import readers
from slam_it_all.foundations.ba import BAProblem

HERE = Path(__file__).parent

ARRAYS = tuple(f.name for f in fields(BAProblem) if f.name != "camera_model")
"""Every array field of ``BAProblem``: the BAL conversion and every default it leaves."""


def _plain(values: np.ndarray) -> list[Any]:
    """Return an array as nested lists, NaN as ``None`` (JSON has no NaN)."""
    plain = values.astype(object)
    if values.dtype.kind == "f":
        plain[np.isnan(values)] = None
    result: list[Any] = plain.tolist()
    return result


def main() -> None:
    """Read ``bal_tiny.txt`` with the Python reader and write what it gives."""
    problem = readers.read_bal(HERE / "bal_tiny.txt")
    fixture = {
        "description": "bal_tiny.txt as slam_it_all.eval.readers.read_bal converts it "
        "(generate_bal.py): the model, and every array field (NaN as null, booleans as "
        "true/false). The native BAL readers must give the same problem.",
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 1e-13,
            "reason": "an SO(3) exponential and a rigid inverse per camera; translations ~5",
        },
        "input": "bal_tiny.txt",
        "expected": {
            "model": problem.camera_model,
            "arrays": {name: _plain(getattr(problem, name)) for name in ARRAYS},
        },
    }
    (HERE / "bal.json").write_text(json.dumps(fixture, indent=2) + "\n")


if __name__ == "__main__":
    main()
