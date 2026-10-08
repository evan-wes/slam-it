"""Generate the golden run folder ``run/`` and the values it was written from, ``run_values.json``.

Run from the repository root after changing the run (the fixtures are committed)::

    python -m tests.fixtures.eval.generate_run

Python's :func:`slamlab.eval.runs.write_run` writes the folder from the values in
``run_values.json`` (``backend`` aside: each writer names its own). The C++ and Rust run
writers write the same values and are held to the folder: the ``.npy`` files byte for byte,
``run.json``, ``trajectory.txt`` and ``diagnostics.jsonl`` value for value (their text may
differ).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from slamlab.eval import runs
from slamlab.foundations import so3

HERE = Path(__file__).parent
FOLDER = HERE / "run"


def _poses(rng: np.random.Generator, count: int) -> np.ndarray:
    """Return random rigid transforms."""
    out = np.tile(np.eye(4), (count, 1, 1))
    out[:, :3, :3] = so3.exp(rng.normal(0.0, 1.0, (count, 3)))
    out[:, :3, 3] = rng.normal(0.0, 2.0, (count, 3))
    return out


def values() -> dict[str, Any]:
    """Return the run's values, as JSON."""
    rng = np.random.default_rng(8)
    times = 1305031102.175304 + 0.0333 * np.arange(5)
    poses = _poses(rng, 5)
    return {
        "dataset": "tum",
        "sequence": "rgbd_dataset_freiburg1_fixture",
        "mode": "rgbd",
        "timestamps": times.tolist(),
        "poses": poses.tolist(),
        "keyframe_timestamps": times[[0, 3]].tolist(),
        "keyframe_poses": poses[[0, 3]].tolist(),
        "points": rng.normal(0.0, 3.0, (4, 3)).tolist(),
        "settings": {"n_features": 1000, "scale_factor": 1.2, "window": 10, "chi2_mono": 5.991},
        "diagnostics": [
            {
                "timestamp": float(t),
                "state": "tracking",
                "tracked": 300 + k,
                "keyframe": k in (0, 3),
                "read_seconds": 0.0025 * (k + 1),
                "build_seconds": 0.0125 * (k + 1),
                "track_seconds": 0.005 * (k + 1),
                "map_seconds": 0.0625 if k == 3 else 0.0,
            }
            for k, t in enumerate(times)
        ],
    }


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
            keyframe_timestamps=np.array(run["keyframe_timestamps"]),
            keyframe_poses=np.array(run["keyframe_poses"]),
            points=np.array(run["points"]),
            settings=run["settings"],
            diagnostics=run["diagnostics"],
        ),
    )


if __name__ == "__main__":
    main()
