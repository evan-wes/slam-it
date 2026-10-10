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
frames' times and ``T_world_cam``; the keyframes' times; the number of map points; and each
frame's stage summaries (:class:`slam_it_all.edge.pipeline.FrameSummary`), one line per frame.
The record also holds its inputs (:func:`tests.orchestration.datasets.room_inputs`), which the
tests check first. The two monocular cases decide alike, the room being the same in both
datasets' folders; both are kept, since each holds a different reader.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

from slam_it_all.edge.pipeline import FrameSummary
from slam_it_all.eval import runs
from slam_it_all.orchestration import vo
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS, SOLVER_TOLERANCE
from tests.fixtures.foundations._layout import dumps
from tests.orchestration.datasets import CASES, GAP_OPTIONS, gap_frames, room_inputs, write_room

HERE = Path(__file__).resolve().parent

SUMMARY_KEYS = tuple(FrameSummary().as_dict())
"""The keys each frame's diagnostics line gives its stage summaries."""


def summaries(run: runs.Run) -> list[dict[str, Any]]:
    """Return each frame's stage summaries, from its diagnostics.

    Parameters
    ----------
    run : slam_it_all.eval.runs.Run
        The run.

    Returns
    -------
    list of dict
        One per frame: :data:`SUMMARY_KEYS` and their values.
    """
    return [{key: d[key] for key in SUMMARY_KEYS} for d in run.diagnostics]


def record(run: runs.Run) -> dict[str, Any]:
    """Return what a run decided, as the record holds it.

    Parameters
    ----------
    run : slam_it_all.eval.runs.Run
        The run.

    Returns
    -------
    dict
        Each frame's state (one string), tracked inliers and keyframe flag (0 or 1); the
        tracked frames' times and ``T_world_cam``, a row of 16 each; the keyframes' times; the
        number of map points; each frame's stage summaries. Columns of numbers, and one
        summary per frame, so the record lays out one line per column or frame and a changed
        decision shows as a changed line.
    """
    return {
        "states": " ".join(d["state"] for d in run.diagnostics),
        "tracked": [d["tracked"] for d in run.diagnostics],
        "keyframes": [int(d["keyframe"]) for d in run.diagnostics],
        "timestamps": run.timestamps.tolist(),
        "poses": run.poses.reshape(-1, 16).tolist(),
        "keyframe_timestamps": run.keyframe_timestamps.tolist(),
        "points": len(run.points),
        "summaries": summaries(run),
    }


def _laid_out(doc: dict[str, Any]) -> str:
    """Serialise the record: the fixtures' layout, each frame's summaries on one line."""
    held: dict[str, list[dict[str, Any]]] = {}
    for name, case in doc["cases"].items():
        held[name] = case["summaries"]
        case["summaries"] = f"@summaries:{name}@"
    text = dumps(doc)
    for name, frames in held.items():
        lines = ",\n    ".join(json.dumps(frame) for frame in frames)
        text = text.replace(json.dumps(f"@summaries:{name}@"), "[\n    " + lines + "\n   ]")
        doc["cases"][name]["summaries"] = frames
    return text


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
    (HERE / "room_runs.json").write_text(_laid_out(doc))
    print(f"wrote room_runs.json ({len(cases)} cases)")


if __name__ == "__main__":
    main()
