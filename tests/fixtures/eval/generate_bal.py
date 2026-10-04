"""Generate ``bal.json``: a tiny BAL file as the Python reader converts it.

The native BAL readers (the Rust example's, ``rust/slamlab-foundations/examples/bal/reader.rs``,
and the C++ benchmark's, ``cpp/bench/foundations/bal_reader.hpp``) copy
``slamlab.eval.readers.read_bal``'s conversion; their tests read this fixture, so the three
cannot drift apart. Run from the repository root after changing ``bal_tiny.txt``::

    python -m tests.fixtures.eval.generate_bal
"""

from __future__ import annotations

import json
from pathlib import Path

from slamlab.eval import readers

HERE = Path(__file__).parent


def main() -> None:
    """Read ``bal_tiny.txt`` with the Python reader and write what it gives."""
    problem = readers.read_bal(HERE / "bal_tiny.txt")
    fixture = {
        "description": "bal_tiny.txt as slamlab.eval.readers.read_bal converts it "
        "(generate_bal.py); the native BAL readers must give the same problem.",
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 1e-13,
            "reason": "an SO(3) exponential and a rigid inverse per camera; translations ~5",
        },
        "input": "bal_tiny.txt",
        "expected": {
            "model": problem.camera_model,
            "poses": problem.poses.tolist(),
            "intrinsics": problem.intrinsics.tolist(),
            "fixed_intrinsics": problem.fixed_intrinsics.tolist(),
            "points": problem.points.tolist(),
            "observations": problem.observations.tolist(),
            "cam_idx": problem.cam_idx.tolist(),
            "pt_idx": problem.pt_idx.tolist(),
        },
    }
    (HERE / "bal.json").write_text(json.dumps(fixture, indent=2) + "\n")


if __name__ == "__main__":
    main()
