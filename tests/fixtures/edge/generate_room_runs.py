"""Generate ``room_runs.json``: the golden record of the room's runs.

Run from the repository root after a change meant to alter what the pipeline decides::

    python -m tests.fixtures.edge.generate_room_runs

The edge rung's stage parity compares every backend live with the Python reference, on states
built from the synthetic room (stored, they would run to megabytes), at the tolerances of
``tests/edge/support.py``. A change made in all three languages at once passes those
comparisons. This record of what the Python pipeline decided on the room, whole and with its
gap (:mod:`tests.orchestration.datasets`), catches it: every backend's runs are held to it
(``tests/orchestration/test_vo.py``), so regenerating it is the deliberate act that accepts new
decisions, and its diff shows them (review panel finding A1).

Per room, dataset and mode: each frame's state, tracked inliers and keyframe flag; the tracked
frames' times and ``T_world_cam``; the keyframes' times; the number of map points.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

from slamlab.eval import runs
from slamlab.orchestration import vo
from tests.edge import room
from tests.edge.conftest import FRAMES, OPTIONS
from tests.orchestration.datasets import CASES, GAP, GAP_OPTIONS, write_room

HERE = Path(__file__).resolve().parent


def _record(run: runs.Run) -> dict[str, Any]:
    """Return what a run decided, as the record holds it."""
    return {
        "diagnostics": [[d["state"], d["tracked"], d["keyframe"]] for d in run.diagnostics],
        "timestamps": run.timestamps.tolist(),
        "poses": run.poses.tolist(),
        "keyframe_timestamps": run.keyframe_timestamps.tolist(),
        "points": len(run.points),
    }


def main() -> None:
    """Run the room and the gap room in every case with the Python reference; write the record."""
    rendered = room.sequence(FRAMES, stereo=True)
    rooms = (
        ("room", rendered, OPTIONS),
        ("gap", rendered[: GAP[0]] + rendered[GAP[1] :], GAP_OPTIONS),
    )
    cases: dict[str, dict[str, Any]] = {}
    with tempfile.TemporaryDirectory() as scratch:
        base = Path(scratch)
        for name, frames, options in rooms:
            folders = write_room(base / name, frames)
            for dataset, mode in CASES:
                folder = folders.tum if dataset == "tum" else folders.euroc
                out = base / f"{name}-{dataset}-{mode}"
                cases[f"{name}/{dataset}/{mode}"] = _record(
                    vo.run(dataset, folder, mode, out, options=options)
                )
    doc = {
        "description": "The room's runs: a golden record (generate_room_runs.py).",
        "oracle": "the Python reference pipeline, as it decided when the record was made",
        "tolerance": {
            "comparison": "close",
            "rtol": 1e-9,
            "atol": 1e-9,
            "reason": "decisions exactly; poses as the backends agree through bundle "
            "adjustment (tests/edge/support.py's SOLVER_TOLERANCE)",
        },
        "cases": cases,
    }
    (HERE / "room_runs.json").write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote room_runs.json ({len(cases)} cases)")


if __name__ == "__main__":
    main()
