"""The run format: exact round trips, TUM's trajectory format, scoring, and failures."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab.eval import runs
from slamlab.foundations import so3
from slamlab.foundations.errors import InvalidInputError


def _poses(n: int, seed: int = 0) -> NDArray[np.float64]:
    """Draw ``n`` random rigid transforms."""
    rng = np.random.default_rng(seed)
    out = np.tile(np.eye(4), (n, 1, 1))
    out[:, :3, :3] = so3.exp(rng.uniform(-2, 2, (n, 3)))
    out[:, :3, 3] = rng.uniform(-5, 5, (n, 3))
    return out


def _run(n: int = 12) -> runs.Run:
    """Build a small run with keyframes, points, settings and diagnostics."""
    times = (1403715274.312143 + 0.05 * np.arange(n)).astype(np.float64)
    return runs.Run(
        dataset="euroc",
        sequence="V1_01_easy",
        mode="stereo",
        backend="cpp",
        timestamps=times,
        poses=_poses(n),
        keyframe_timestamps=times[::4],
        keyframe_poses=_poses(n)[::4],
        points=np.random.default_rng(1).normal(size=(30, 3)),
        settings={"orb": {"n_features": 1000}, "window": 7},
        diagnostics=[{"timestamp": float(t), "tracked": 120 + k} for k, t in enumerate(times)],
    )


def test_round_trip(tmp_path: Path) -> None:
    """Every field survives writing and reading; times and positions exactly."""
    run = _run()
    runs.write_run(tmp_path / "run", run)
    back = runs.read_run(tmp_path / "run")
    assert (back.dataset, back.sequence, back.mode, back.backend) == (
        "euroc",
        "V1_01_easy",
        "stereo",
        "cpp",
    )
    np.testing.assert_array_equal(back.timestamps, run.timestamps)
    np.testing.assert_array_equal(back.poses[:, :3, 3], run.poses[:, :3, 3])
    np.testing.assert_allclose(back.poses, run.poses, rtol=0, atol=1e-15)
    np.testing.assert_array_equal(back.keyframe_poses, run.keyframe_poses)
    np.testing.assert_array_equal(back.points, run.points)
    assert back.settings == run.settings
    assert back.diagnostics == run.diagnostics


def test_reads_a_tum_trajectory(tmp_path: Path) -> None:
    """A trajectory as TUM's tools write it (comments, four decimals) reads as poses."""
    path = tmp_path / "groundtruth.txt"
    path.write_text(
        "# ground truth trajectory\n# timestamp tx ty tz qx qy qz qw\n"
        "1305031098.6659 1.3563 0.6305 1.6380 0.6132 0.5962 -0.3311 -0.3986\n"
        "1305031098.6758 1.3543 0.6326 1.6371 0.6129 0.5966 -0.3316 -0.3980\n"
    )
    times, poses = runs.read_trajectory(path)
    np.testing.assert_array_equal(times, [1305031098.6659, 1305031098.6758])
    np.testing.assert_array_equal(poses[0, :3, 3], [1.3563, 0.6305, 1.6380])
    np.testing.assert_allclose(
        poses[:, :3, :3] @ poses[:, :3, :3].transpose(0, 2, 1),
        np.tile(np.eye(3), (2, 1, 1)),
        atol=1e-12,
    )


def test_evaluate_scores_a_noisy_run() -> None:
    """A run with centimetre noise scores a centimetre-level ATE on its matched frames."""
    truth_times = np.arange(0.0, 10.0, 0.005)
    angles = np.column_stack([0.1 * truth_times, 0.3 * truth_times, np.zeros_like(truth_times)])
    truth = np.tile(np.eye(4), (len(truth_times), 1, 1))
    truth[:, :3, :3] = so3.exp(angles)
    truth[:, :3, 3] = np.column_stack([np.cos(truth_times), np.sin(truth_times), 0.1 * truth_times])
    frames = np.arange(0, len(truth_times), 10)
    estimate = truth[frames].copy()
    estimate[:, :3, 3] += 0.01 * np.random.default_rng(2).normal(size=(len(frames), 3))
    run = runs.Run("synthetic", "circle", "rgbd", "python", truth_times[frames] + 0.001, estimate)
    scored = runs.evaluate(run, truth_times, truth)
    assert scored.matched == len(frames)
    assert 0.01 < scored.absolute.rmse < 0.025
    assert scored.absolute.scale == 1.0


@pytest.mark.parametrize(
    ("edit", "message"),
    [
        (lambda info: info.update(version=2), "version 1"),
        (lambda info: info.update(format="other"), "format"),
        (lambda info: info.update(mode="lidar"), "mode"),
        (lambda info: info.update(backend="java"), "backend"),
    ],
)
def test_read_run_rejects_unknown_info(tmp_path: Path, edit: object, message: str) -> None:
    """A run.json of another format, version, mode or backend is refused."""
    runs.write_run(tmp_path, _run())
    info = json.loads((tmp_path / "run.json").read_text())
    assert callable(edit)
    edit(info)
    (tmp_path / "run.json").write_text(json.dumps(info))
    with pytest.raises(InvalidInputError, match=message):
        runs.read_run(tmp_path)


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("1.0 0 0 0 0 0 0 1\n0.5 0 0 0 0 0 0 1\n", "ascend"),
        ("1.0 0 0 0 0 0 0\n", "expected 8 finite numbers"),
        ("1.0 0 0 x 0 0 0 1\n", "expected numbers"),
        ("1.0 0 0 0 0 0 0 0\n", "quaternion is zero"),
    ],
)
def test_read_trajectory_rejects(tmp_path: Path, text: str, message: str) -> None:
    """Unordered times, short lines, non-numbers and zero quaternions are refused."""
    (tmp_path / "t.txt").write_text(text)
    with pytest.raises(InvalidInputError, match=message):
        runs.read_trajectory(tmp_path / "t.txt")
