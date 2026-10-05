"""Trajectory metrics: ATE and RPE on trajectories with known errors, and association."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab.eval import metrics
from slamlab.foundations import se3, so3
from slamlab.foundations.errors import DegenerateError, InvalidInputError


def _trajectory(n: int = 40, seed: int = 0) -> NDArray[np.float64]:
    """Draw a smooth trajectory of ``T_world_cam`` poses: a curve with turning cameras."""
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, 4.0, n)
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, :3, :3] = so3.exp(np.column_stack([0.2 * np.sin(t), 0.5 * t, 0.1 * t]))
    poses[:, :3, 3] = np.column_stack([np.cos(t), np.sin(1.3 * t), 0.3 * t]) + 0.01 * rng.normal(
        size=(n, 3)
    )
    return poses


def _similarity(scale: float) -> NDArray[np.float64]:
    """Return a similarity ``[[s R, t], [0, 1]]`` with a fixed rotation and translation."""
    out = np.eye(4)
    out[:3, :3] = scale * so3.exp(np.array([0.3, -0.7, 1.1]))
    out[:3, 3] = [5.0, -2.0, 0.5]
    return out


def _moved(poses: NDArray[np.float64], g: NDArray[np.float64]) -> NDArray[np.float64]:
    """Express a trajectory in a frame related by the similarity `g` (centres and turns)."""
    scale = np.cbrt(np.linalg.det(g[:3, :3]))
    out = poses.copy()
    out[:, :3, :3] = (g[:3, :3] / scale) @ poses[:, :3, :3]
    out[:, :3, 3] = poses[:, :3, 3] @ g[:3, :3].T + g[:3, 3]
    return out


def test_ate_is_zero_for_the_truth() -> None:
    """The truth against itself has no error and the identity alignment."""
    truth = _trajectory()
    ate = metrics.absolute_error(truth, truth)
    assert ate.rmse < 1e-12
    np.testing.assert_allclose(ate.alignment, np.eye(4), atol=1e-12)


@pytest.mark.parametrize("scale", [1.0, 0.37])
def test_ate_removes_a_similarity(scale: float) -> None:
    """A trajectory in another frame (and, for Sim(3), scale) aligns back exactly."""
    truth = _trajectory()
    estimate = _moved(truth, _similarity(scale))
    ate = metrics.absolute_error(estimate, truth, with_scale=scale != 1.0)
    assert ate.rmse < 1e-9
    np.testing.assert_allclose(ate.scale, 1.0 / scale, rtol=1e-12)


def test_rigid_ate_does_not_hide_a_scale_error() -> None:
    """SE(3) alignment leaves a monocular scale error visible; Sim(3) removes it."""
    truth = _trajectory()
    estimate = _moved(truth, _similarity(0.5))
    assert metrics.absolute_error(estimate, truth).rmse > 0.1
    assert metrics.absolute_error(estimate, truth, with_scale=True).rmse < 1e-9


def test_ate_statistics_of_known_errors() -> None:
    """Centre offsets of known lengths give their RMSE, mean, median and maximum."""
    truth = _trajectory(n=30)
    offsets = np.random.default_rng(1).normal(size=(30, 3))
    offsets -= offsets.mean(axis=0)  # nothing for the alignment to absorb in translation
    estimate = truth.copy()
    estimate[:, :3, 3] += 1e-3 * offsets
    ate = metrics.absolute_error(estimate, truth)
    assert ate.max == pytest.approx(np.max(ate.errors))
    assert ate.rmse == pytest.approx(np.sqrt(np.mean(ate.errors**2)))
    assert ate.rmse == pytest.approx(1e-3 * np.sqrt(np.mean(np.sum(offsets**2, axis=1))), rel=0.05)


def test_rpe_measures_drift_per_step() -> None:
    """A 2 cm extra step forward per frame reads as 2 cm of translational drift each."""
    truth = _trajectory()
    estimate = truth.copy()
    step = np.eye(4)
    step[:3, 3] = [0.0, 0.0, 0.02]
    for k in range(1, len(truth)):
        motion = se3.compose(se3.inverse(truth[k - 1]), truth[k])
        estimate[k] = estimate[k - 1] @ motion @ step
    rpe = metrics.relative_error(estimate, truth)
    np.testing.assert_allclose(rpe.translation, 0.02, rtol=1e-9)
    np.testing.assert_allclose(rpe.rotation, 0.0, atol=1e-9)
    assert metrics.relative_error(truth, truth, delta=5).translation_rmse < 1e-12


def test_rpe_scale_undoes_a_monocular_scale() -> None:
    """A trajectory at half scale has no drift once scaled back."""
    truth = _trajectory()
    estimate = _moved(truth, _similarity(0.5))
    assert metrics.relative_error(estimate, truth).translation_rmse > 0.01
    assert metrics.relative_error(estimate, truth, scale=2.0).translation_rmse < 1e-9


def test_associate_pairs_nearest_times_once() -> None:
    """Each time takes its nearest reference within the tolerance, each reference once."""
    times = np.array([0.0, 0.1, 0.1035, 0.3, 0.9])
    reference = np.array([0.001, 0.102, 0.31, 0.5])
    ours, theirs = metrics.associate(times, reference, 0.02)
    np.testing.assert_array_equal(ours, [0, 2, 3])
    np.testing.assert_array_equal(theirs, [0, 1, 2])


@pytest.mark.parametrize(
    ("call", "error", "message"),
    [
        (lambda t: metrics.absolute_error(t, t[:-1]), InvalidInputError, "equal"),
        (lambda t: metrics.absolute_error(t[:2], t[:2]), DegenerateError, "fewer than three"),
        (lambda t: metrics.relative_error(t, t, delta=0), InvalidInputError, "delta"),
        (lambda t: metrics.relative_error(t, t, delta=len(t)), InvalidInputError, "delta"),
        (lambda t: metrics.relative_error(t, t, scale=0.0), InvalidInputError, "scale"),
        (lambda t: metrics.associate([0.0], [0.0], -1.0), InvalidInputError, "max_difference"),
    ],
)
def test_rejects(call: object, error: type, message: str) -> None:
    """Mismatched, too short or out-of-range inputs are refused."""
    assert callable(call)
    with pytest.raises(error, match=message):
        call(_trajectory(n=10))
