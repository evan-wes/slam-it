"""Synthetic runs for the evaluation's tests, built in memory with their ground truth.

Shared by the loops' judge's tests (``tests/eval/test_loops.py``) and the test-set measure's
(``tests/test_measure_testset.py``).
"""

from __future__ import annotations

import numpy as np

from slam_it.eval import runs


def line_run(count: int) -> tuple[runs.Run, runs.Truth]:
    """Return a run along a 6 m line, keyframes every tenth frame at the truth's poses.

    The frames' poses are a centimetre off the truth's, so the map has a median error.
    """
    times = np.arange(count, dtype=np.float64) / 10.0
    poses = np.tile(np.eye(4), (count, 1, 1))
    poses[:, 0, 3] = np.linspace(0.0, 6.0, count)
    poses[:, 1, 3] = 0.1 * np.sin(times)  # off a line, so the ATE aligns
    tracked = poses.copy()
    tracked[:, :3, 3] += np.random.default_rng(4).normal(0.0, 0.01, (count, 3))
    rows = [0, 10, 20, count - 1]
    run = runs.Run(
        "synthetic", "line", "rgbd", "python", times, tracked, tracked,
        keyframe_timestamps=times[rows], keyframe_poses=poses[rows].copy(),
        keyframe_lives=np.array([[k, -1, -1] for k in rows], dtype=np.int64),
        pose_maps=np.zeros(count, dtype=np.int64),
        keyframe_maps=np.zeros(len(rows), dtype=np.int64),
    )  # fmt: skip
    return run, (times, poses)
