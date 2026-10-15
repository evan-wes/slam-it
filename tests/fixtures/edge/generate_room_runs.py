"""Generate ``room_runs.json``: the golden record of the room's runs.

Run from the repository root after a change meant to alter what the pipeline decides::

    python -m tests.fixtures.edge.generate_room_runs

The edge rung's stage parity compares every backend live with the Python reference, on states
built from the synthetic room (stored, they would run to megabytes), at the tolerances of
``tests/edge/support.py``. A change made in all three languages at once passes those
comparisons. This record of what the Python pipeline decided on the room, whole, with its gap,
lost twice, its lost maps reset or kept, and lost and found again by relocalisation
(:mod:`tests.orchestration.datasets`), catches it:
every backend's runs are held to it (``tests/orchestration/test_vo.py``), so regenerating it is
the deliberate act that accepts new decisions, and its diff shows them (review panel finding
A1).

Per room, dataset and mode: each frame's state, map, tracked inliers and keyframe flag; the
tracked frames' times, ``T_world_cam`` and maps; the final map's keyframes' times and its number
of points; every point's and keyframe's life (the run format's ``*_lives.npy``), as three
columns each, and map;
each frame's count of keypoints of each fate (:data:`slam_it.eval.runs.FATES`), a column per
fate; and each frame's stage summaries (:class:`slam_it.edge.pipeline.FrameSummary`), one
line per frame.
The record also holds its inputs (:func:`tests.orchestration.datasets.room_inputs`), which the
tests check first. The two monocular cases decide alike, the room being the same in both
datasets' folders; both are kept, since each holds a different reader.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from slam_it.edge.pipeline import FrameSummary
from slam_it.eval import runs
from slam_it.orchestration import vo
from tests.edge import room
from tests.edge.support import FRAMES, OPTIONS, SOLVER_TOLERANCE
from tests.fixtures.foundations._layout import dumps
from tests.orchestration.datasets import (
    CASES,
    FOUND_OPTIONS,
    GAP_OPTIONS,
    KEPT_OPTIONS,
    found_frames,
    gap_frames,
    lost_frames,
    room_inputs,
    write_room,
)

HERE = Path(__file__).resolve().parent

SUMMARY_KEYS = tuple(FrameSummary().as_dict())
"""The keys each frame's diagnostics line gives its stage summaries."""


def summaries(run: runs.Run) -> list[dict[str, Any]]:
    """Return each frame's stage summaries, from its diagnostics.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.

    Returns
    -------
    list of dict
        One per frame: :data:`SUMMARY_KEYS` and their values.
    """
    return [{key: d[key] for key in SUMMARY_KEYS} for d in run.diagnostics]


def fate_counts(run: runs.Run, features: runs.Features) -> list[list[int]]:
    """Return each frame's count of keypoints of each fate.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.
    features : slam_it.eval.runs.Features
        Its feature records.

    Returns
    -------
    list of list of int
        One column per fate of :data:`slam_it.eval.runs.FATES`, one count per frame.
    """
    counts = np.zeros((len(runs.FATES), len(run.diagnostics)), dtype=np.int64)
    frame, fate = features.records[:, 0], features.records[:, 2]
    np.add.at(counts, (fate, frame), 1)
    return counts.tolist()


def record(run: runs.Run, features: runs.Features) -> dict[str, Any]:
    """Return what a run decided, as the record holds it.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run.
    features : slam_it.eval.runs.Features
        Its feature records.

    Returns
    -------
    dict
        Each frame's state (one string), map (null before one), tracked inliers and keyframe
        flag (0 or 1); the tracked frames' times, ``T_world_cam`` (a row of 16 each) and maps;
        the final map's keyframes' times and number of points; the lives of every point and
        keyframe, as columns (made, removed, why), and their maps; each frame's count of
        keypoints of each fate, a column per fate; each frame's stage summaries. Columns of
        numbers, and one summary per frame, so the record lays out one line per column or
        frame and a changed decision shows as a changed line.
    """
    return {
        "states": " ".join(d["state"] for d in run.diagnostics),
        "maps": [d["map"] for d in run.diagnostics],
        "tracked": [d["tracked"] for d in run.diagnostics],
        "keyframes": [int(d["keyframe"]) for d in run.diagnostics],
        "timestamps": run.timestamps.tolist(),
        "poses": run.poses.reshape(-1, 16).tolist(),
        "pose_maps": run.pose_maps.tolist(),
        "keyframe_timestamps": run.final_keyframes()[0].tolist(),
        "points": len(run.final_points()),
        "point_lives": run.point_lives.T.tolist(),
        "keyframe_lives": run.keyframe_lives.T.tolist(),
        "point_maps": run.point_maps.tolist(),
        "keyframe_maps": run.keyframe_maps.tolist(),
        "fates": fate_counts(run, features),
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
    """Run each room in every case with the Python reference; write the record."""
    rendered = room.sequence(FRAMES, stereo=True)
    lost = lost_frames(rendered)
    rooms = (
        ("room", rendered, OPTIONS),
        ("gap", gap_frames(rendered), GAP_OPTIONS),
        ("lost", lost, OPTIONS),
        ("kept", lost, KEPT_OPTIONS),
        ("found", found_frames(rendered), FOUND_OPTIONS),
    )
    cases: dict[str, dict[str, Any]] = {}
    with tempfile.TemporaryDirectory() as scratch:
        base = Path(scratch)
        for name, frames, options in rooms:
            folders = write_room(base / name, frames)
            for dataset, mode in CASES:
                folder = folders.tum if dataset == "tum" else folders.euroc
                out = base / f"{name}-{dataset}-{mode}"
                run = vo.run(dataset, folder, mode, out, options=options, record_features=True)
                cases[f"{name}/{dataset}/{mode}"] = record(run, runs.read_features(out, run))
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
