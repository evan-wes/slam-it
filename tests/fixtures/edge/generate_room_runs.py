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
frames' times and ``T_world_cam``; the keyframes' times; the number of map points. The record
also holds its inputs (:func:`tests.orchestration.datasets.room_inputs`), which the tests check
first. The two monocular cases decide alike, the room being the same in both datasets' folders;
both are kept, since each holds a different reader.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any

from slamlab.eval import runs
from slamlab.orchestration import vo
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS, SOLVER_TOLERANCE
from tests.fixtures.foundations._layout import dumps
from tests.orchestration.datasets import CASES, GAP_OPTIONS, gap_frames, room_inputs, write_room

HERE = Path(__file__).resolve().parent


def record(run: runs.Run) -> dict[str, Any]:
    """Return what a run decided, as the record holds it.

    Parameters
    ----------
    run : slamlab.eval.runs.Run
        The run.

    Returns
    -------
    dict
        Each frame's state (one string), tracked inliers and keyframe flag (0 or 1); the
        tracked frames' times and ``T_world_cam``, a row of 16 each; the keyframes' times; the
        number of map points. Columns of numbers, so the record lays out one line per column
        and a changed decision shows as a changed line.
    """
    return {
        "states": " ".join(d["state"] for d in run.diagnostics),
        "tracked": [d["tracked"] for d in run.diagnostics],
        "keyframes": [int(d["keyframe"]) for d in run.diagnostics],
        "timestamps": run.timestamps.tolist(),
        "poses": run.poses.reshape(-1, 16).tolist(),
        "keyframe_timestamps": run.keyframe_timestamps.tolist(),
        "points": len(run.points),
    }


def main() -> None:
    """Run the room and the gap room in every case with the Python reference; write the record."""
    rendered = room.sequence(FRAMES, stereo=True)
    rooms = (
        ("room", rendered, OPTIONS),
        ("gap", gap_frames(rendered), GAP_OPTIONS),
    )
    cases: dict[str, dict[str, Any]] = {}
    with tempfile.TemporaryDirectory() as scratch:
        base = Path(scratch)
        for name, frames, options in rooms:
            folders = write_room(base / name, frames)
            for dataset, mode in CASES:
                folder = folders.tum if dataset == "tum" else folders.euroc
                out = base / f"{name}-{dataset}-{mode}"
                cases[f"{name}/{dataset}/{mode}"] = record(
                    vo.run(dataset, folder, mode, out, options=options)
                )
    doc = {
        "description": "The room's runs: a golden record (generate_room_runs.py).",
        "oracle": "the Python reference pipeline, as it decided when the record was made",
        "tolerance": {
            "comparison": "close",
            **SOLVER_TOLERANCE,
            "reason": "decisions exactly; poses as the backends agree through bundle "
            "adjustment (tests/edge/support.py's SOLVER_TOLERANCE)",
        },
        "inputs": room_inputs(),
        "cases": cases,
    }
    (HERE / "room_runs.json").write_text(dumps(doc))
    print(f"wrote room_runs.json ({len(cases)} cases)")


if __name__ == "__main__":
    main()
