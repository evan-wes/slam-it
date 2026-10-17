"""Generate the golden run folder ``run/`` and the values it was written from, ``run_values.json``.

Run from the repository root after changing the run (the fixtures are committed)::

    python -m tests.fixtures.eval.generate_run

Python's :func:`slam_it.eval.runs.write_run` writes the folder from the values in
``run_values.json`` (``backend`` aside: each writer names its own). The C++ and Rust run
writers write the same values and are held to the folder: the ``.npy`` files byte for byte,
``run.json``, ``trajectory.txt`` and ``diagnostics.jsonl`` value for value (their text may
differ). The diagnostics nest as the edge pipeline's stage summaries do: objects, lists and
nulls inside each line. The run records its features, every fate among them and a frame with
no keypoints: :class:`slam_it.eval.runs.FeatureWriter` streams them frame by frame from
``features``, and the native feature writers are held to its two files byte for byte. It
relocalised, with the vocabulary reader's fixture (``vocabulary/``): its ``vocabulary`` is that
folder's record, which the native writers write from the header their readers read.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from slam_it.eval import runs, vocabularies
from slam_it.foundations import so3

HERE = Path(__file__).parent
FOLDER = HERE / "run"


def _poses(rng: np.random.Generator, count: int) -> np.ndarray:
    """Return random rigid transforms."""
    out = np.tile(np.eye(4), (count, 1, 1))
    out[:, :3, :3] = so3.exp(rng.normal(0.0, 1.0, (count, 3)))
    out[:, :3, 3] = rng.normal(0.0, 2.0, (count, 3))
    return out


def _summary(k: int) -> dict[str, Any]:
    """Return frame ``k``'s stage summaries, nested as the edge pipeline writes them.

    Frame 0 starts map 0; frame 2 loses it, kept; frame 3 starts map 1, and frame 4 is its
    second keyframe, on which a loop closes back to the first.
    """
    solve = {
        "initial_cost": 12.5 / (k + 1),
        "final_cost": 0.75 / (k + 1),
        "iterations": 5 + k,
        "termination": "function",
    }
    starts = k in (0, 3)
    return {
        "placed_by": "initialization" if starts else (None if k == 2 else "motion"),
        "init": {"matches": None, "outcome": "initialized", "ba": []} if starts else None,
        "motion": None if starts else {"matches": 200 + k, "widened": k == 2, "inliers": 180 + k},
        "pnp": {
            "candidates": 500,
            "matches": 120,
            "iterations": None,
            "termination": "degenerate",
            "inliers": None,
            "refined": None,
        }
        if k == 2
        else None,
        "local_map": {"in_view": 400 + k, "matches": 300 + k, "inliers": 290 + k}
        if k in (1, 4, 5)
        else None,
        "mapping": {
            "from_depth": 40,
            "culled_points": 3,
            "triangulated": 12,
            "fused": 2,
            "ba": [solve, solve],
            "outliers": 7,
            "weak_points": 1,
            "culled_keyframes": 0,
        }
        if k == 4
        else None,
        "loop": _loop() if k == 4 else None,
        "lost": {"keyframes": 1, "points": 2, "reset": False} if k == 2 else None,
    }


def _loop() -> dict[str, Any]:
    """Return a keyframe's loop summary, nested as the edge pipeline writes it: closed at once."""
    candidate = {
        "keyframe": 0,
        "score": 0.125,
        "near": False,
        "matches": 64,
        "best": 0,
        "inliers": 40,
        "projected": 90,
        "refined": 35,
        "reprojected": 120,
        "confirmations": 3,
    }
    solve = {
        "initial_error": 0.5,
        "final_error": 0.0625,
        "iterations": 4,
        "termination": "function",
    }
    return {
        "detection": {
            "outcome": "closed",
            "confirmation": None,
            "sharing": 1,
            "scored": 1,
            "candidates": [candidate],
            "confirmations": 3,
        },
        "correction": {
            "current": 1,
            "matched": 0,
            "similarity": [[1.0, 0.0, 0.0, 0.5], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.25],
                           [0.0, 0.0, 0.0, 1.0]],
            "corrected": 2,
            "moved": 3,
            "replaced": 1,
            "fused": 1,
            "links": 1,
            "edges": 2,
            "essential": solve,
            "global_ba": {**solve, "iterations": 10, "termination": "max_iterations"},
        },
    }  # fmt: skip


def _features(rng: np.random.Generator) -> list[dict[str, list[Any]]]:
    """Return each frame's keypoints: pixels, levels, fates and points, as the frames go.

    Frame 0 starts map 0 with points 0 and 1; frame 1 tracks one, rejects the other; the lost
    frame 2 matched point 0 and rejected it; frame 3 starts map 1 with points 2 and 3; frame 4,
    a keyframe, tracks point 2, rejects 3 and makes 4; frame 5 has no keypoints. Pixels have
    bits float32 rounds away.
    """
    unmatched, rejected, tracked, made = range(4)
    frames = [
        [(made, 0), (made, 1), (unmatched, -1)],
        [(tracked, 0), (rejected, 1)],
        [(rejected, 0), (unmatched, -1)],
        [(made, 2), (made, 3), (unmatched, -1)],
        [(tracked, 2), (made, 4), (rejected, 3), (unmatched, -1)],
        [],
    ]
    return [
        {
            "xy": rng.uniform(0.0, 640.0, (len(keypoints), 2)).tolist(),
            "level": [k % 8 for k in range(len(keypoints))],
            "fate": [fate for fate, _ in keypoints],
            "point": [point for _, point in keypoints],
        }
        for keypoints in frames
    ]


def values() -> dict[str, Any]:
    """Return the run's values, as JSON: two maps, the first lost on frame 2 and kept."""
    rng = np.random.default_rng(8)
    times = 1305031102.175304 + 0.0333 * np.arange(6)
    tracked = [0, 1, 3, 4, 5]
    poses = _poses(rng, len(tracked))
    keyframes = [0, 3, 4]
    out = {
        "dataset": "tum",
        "sequence": "rgbd_dataset_freiburg1_fixture",
        "mode": "rgbd",
        "timestamps": times[tracked].tolist(),
        "poses": poses.tolist(),
        "pose_maps": [0, 0, 1, 1, 1],
        "keyframe_timestamps": times[keyframes].tolist(),
        "keyframe_poses": poses[[0, 2, 3]].tolist(),
        "keyframe_maps": [0, 1, 1],
        "points": rng.normal(0.0, 3.0, (5, 3)).tolist(),
        "point_maps": [0, 0, 1, 1, 1],
        "point_lives": [[0, -1, -1], [0, -1, -1], [3, -1, -1], [3, 4, 1], [4, -1, -1]],
        "keyframe_lives": [[0, -1, -1], [3, -1, -1], [4, -1, -1]],
        "keyframe_parents": [-1, -1, 1],
        "keyframe_links": [[1, 2, 120]],
        "keyframe_loops": [[1, 2]],
        "settings": {
            "features": {"n_features": 1000, "scale_factor": 1.2},
            "covisibility": {"local_neighbours": 10},
            "adjustment": {"chi2_mono": 5.991},
        },
        "vocabulary": vocabularies.record(HERE / "vocabulary"),
        "peak_memory": 48_381_952,
        "cpus": [4, 5, 6, 7],
        "memory_cap": 8_589_934_592,
        "record_features": True,
        "features": _features(rng),
        "diagnostics": [
            {
                "timestamp": float(t),
                "state": "lost" if k == 2 else "tracking",
                "map": 0 if k < 3 else 1,
                "tracked": 0 if k == 2 else 300 + k,
                "keyframe": k in keyframes,
                "read_seconds": 0.0025 * (k + 1),
                "build_seconds": 0.0125 * (k + 1),
                "track_seconds": 0.005 * (k + 1),
                "map_seconds": 0.0625 if k == 4 else 0.0,
                "loop_seconds": 0.1328125 if k == 4 else 0.0,
                **_summary(k),
            }
            for k, t in enumerate(times)
        ],
    }
    as_tracked = poses.copy()  # drawn last, so the other values stay as they were
    as_tracked[:, :3, 3] += rng.normal(0.0, 0.05, (len(tracked), 3))
    out["tracked_poses"] = as_tracked.tolist()
    return out


def main() -> None:
    """Write the values and the golden folder."""
    run = values()
    (HERE / "run_values.json").write_text(json.dumps(run, indent=1) + "\n")
    runs.write_run(
        FOLDER,
        runs.Run(
            dataset=run["dataset"],
            sequence=run["sequence"],
            mode=run["mode"],
            backend="python",
            timestamps=np.array(run["timestamps"]),
            poses=np.array(run["poses"]),
            tracked_poses=np.array(run["tracked_poses"]),
            keyframe_timestamps=np.array(run["keyframe_timestamps"]),
            keyframe_poses=np.array(run["keyframe_poses"]),
            points=np.array(run["points"]),
            settings=run["settings"],
            vocabulary=run["vocabulary"],
            diagnostics=run["diagnostics"],
            peak_memory=run["peak_memory"],
            cpus=run["cpus"],
            memory_cap=run["memory_cap"],
            point_lives=np.array(run["point_lives"], dtype=np.int64),
            keyframe_lives=np.array(run["keyframe_lives"], dtype=np.int64),
            keyframe_parents=np.array(run["keyframe_parents"], dtype=np.int64),
            keyframe_links=np.array(run["keyframe_links"], dtype=np.int64),
            keyframe_loops=np.array(run["keyframe_loops"], dtype=np.int64),
            record_features=run["record_features"],
            pose_maps=np.array(run["pose_maps"], dtype=np.int64),
            keyframe_maps=np.array(run["keyframe_maps"], dtype=np.int64),
            point_maps=np.array(run["point_maps"], dtype=np.int64),
        ),
    )
    with runs.FeatureWriter(FOLDER) as features:
        for frame in run["features"]:
            features.write(
                np.array(frame["xy"]).reshape(-1, 2),
                np.array(frame["level"], dtype=np.int64),
                np.array(frame["fate"], dtype=np.int64),
                np.array(frame["point"], dtype=np.int64),
            )


if __name__ == "__main__":
    main()
