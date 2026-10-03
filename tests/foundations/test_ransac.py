"""RANSAC core tests, through a line-fitting estimator: recovery, stopping, LO, parity data."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab.foundations import ransac, solve
from slamlab.foundations.errors import DegenerateError, InvalidInputError

Line = NDArray[np.float64]
"""A line ``a x + b y + c = 0`` as ``(a, b, c)`` with ``a² + b² = 1``."""


class LineEstimator:
    """Fit 2D lines: two points minimally, total least squares otherwise."""

    def __init__(self, points: NDArray[np.float64]) -> None:
        self.points = points

    @property
    def sample_size(self) -> int:
        """Two points determine a line."""
        return 2

    @property
    def size(self) -> int:
        """The number of points."""
        return int(self.points.shape[0])

    def fit_minimal(self, sample: NDArray[np.int64]) -> list[Line]:
        """Fit the line through two points; none if they coincide."""
        p, q = self.points[sample]
        direction = q - p
        length = float(np.hypot(*direction))
        if length == 0.0:
            return []
        a, b = -direction[1] / length, direction[0] / length
        return [np.array([a, b, -(a * p[0] + b * p[1])])]

    def fit_nonminimal(self, subset: NDArray[np.int64]) -> list[Line]:
        """Fit by total least squares: the normal is the centred points' least direction."""
        chosen = self.points[subset]
        mean = chosen.mean(axis=0)
        normal = np.linalg.svd(chosen - mean)[2][-1]
        return [np.array([normal[0], normal[1], -float(normal @ mean)])]

    def residuals(self, model: Line) -> NDArray[np.float64]:
        """Return squared point-to-line distances."""
        return np.asarray((self.points @ model[:2] + model[2]) ** 2, dtype=np.float64)


def parity_points() -> NDArray[np.float64]:
    """Return the points cpp/tests/foundations/test_ransac.cpp builds identically.

    On ``y = 2x + 1`` with a deterministic wobble of 0.01 sin(i), except every third point,
    which is an outlier at ``-((37 i) mod 100) / 10 - 1``, below the line. Integer arithmetic
    and libm's ``sin`` give the same doubles in both languages.
    """
    points = []
    for i in range(100):
        x = i / 10.0
        y = -((37 * i) % 100) / 10.0 - 1.0 if i % 3 == 0 else 2.0 * x + 1.0 + 0.01 * math.sin(i)
        points.append((x, y))
    return np.array(points)


OPTIONS = ransac.Options(threshold=0.05, seed=42)
PARITY_ITERATIONS = 13
"""Samples the parity run draws, in both backends."""
PARITY_LOCAL = 2
"""Local optimisations the parity run makes, in both backends."""
PARITY_MODEL = [0.8944198368027669, -0.4472283035919258, 0.4474675589870847]
"""The parity run's line, which the C++ core reproduces to rounding."""


def test_recovers_the_line_and_its_inliers() -> None:
    """The model is y = 2x + 1 and the inliers are exactly the non-outliers."""
    result = ransac.estimate(LineEstimator(parity_points()), OPTIONS)
    a, b, c = result.state.model
    assert -a / b == pytest.approx(2.0, abs=1e-3)
    assert -c / b == pytest.approx(1.0, abs=1e-2)
    expected = np.array([i % 3 != 0 for i in range(100)])
    np.testing.assert_array_equal(result.state.inliers, expected)
    assert result.summary.inlier_count == 66
    assert result.summary.converged
    assert result.summary.termination == ransac.Termination.CONFIDENCE


def test_parity_values_for_the_cpp_core() -> None:
    """The run cpp/tests/foundations/test_ransac.cpp repeats: same samples, same stop."""
    summary = ransac.estimate(LineEstimator(parity_points()), OPTIONS).summary
    assert (summary.iterations, summary.local_optimizations, summary.degenerate_samples) == (
        PARITY_ITERATIONS,
        PARITY_LOCAL,
        0,
    )
    model = ransac.estimate(LineEstimator(parity_points()), OPTIONS).state.model
    np.testing.assert_allclose(model, PARITY_MODEL, rtol=0, atol=1e-15)


def test_stops_at_the_adaptive_bound() -> None:
    """Sampling stops on the first iteration at or past the confidence bound."""
    summary = ransac.estimate(LineEstimator(parity_points()), OPTIONS).summary
    bound = ransac.required_iterations(66 / 100, 2, OPTIONS.confidence)
    assert summary.iterations == math.ceil(bound)


def test_local_optimization_lowers_the_cost() -> None:
    """With LO the final MSAC cost is lower than without it, on the same samples."""
    points = parity_points()
    plain = ransac.estimate(LineEstimator(points), replace(OPTIONS, local_optimization=False))
    refined = ransac.estimate(LineEstimator(points), OPTIONS)
    assert refined.summary.score < plain.summary.score
    assert plain.summary.local_optimizations == 0


def test_iteration_cap_is_a_result_not_an_error() -> None:
    """Capped below the bound, the run returns its best model, unconverged."""
    capped = ransac.estimate(LineEstimator(parity_points()), replace(OPTIONS, max_iterations=1))
    assert capped.summary.iterations == 1
    assert capped.summary.termination == ransac.Termination.MAX_ITERATIONS
    assert not capped.summary.converged


def test_summary_meets_the_shared_contract() -> None:
    """The RANSAC summary is a :class:`solve.Summary`; mypy checks the assignment."""
    shared: solve.Summary = ransac.estimate(LineEstimator(parity_points()), OPTIONS).summary
    assert shared.termination in ransac.CONVERGED


def test_degenerate_samples_are_counted_and_all_degenerate_raises() -> None:
    """Coincident points give no model: counted while others work, raised when none do."""
    points = parity_points()
    points[:50] = points[0]
    result = ransac.estimate(LineEstimator(points), replace(OPTIONS, max_iterations=200))
    assert result.summary.degenerate_samples > 0
    with pytest.raises(DegenerateError, match="none of"):
        ransac.estimate(LineEstimator(np.zeros((10, 2))), OPTIONS)


def test_required_iterations() -> None:
    """The textbook bound, and its edges."""
    assert ransac.required_iterations(0.5, 4, 0.99) == pytest.approx(
        math.log(0.01) / math.log(1 - 0.5**4)
    )
    assert ransac.required_iterations(1.0, 4, 0.99) == 0.0
    assert ransac.required_iterations(0.0, 4, 0.99) == math.inf


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"threshold": 0.0}, "threshold"),
        ({"threshold": math.nan}, "threshold"),
        ({"threshold": 1.0, "confidence": 1.0}, "confidence"),
        ({"threshold": 1.0, "max_iterations": 0}, "max_iterations"),
        ({"threshold": 1.0, "lo_steps": 0}, "lo_steps"),
        ({"threshold": 1.0, "lo_threshold_multiplier": 0.5}, "lo_threshold_multiplier"),
    ],
)
def test_rejects_invalid_options(kwargs: dict[str, float], message: str) -> None:
    """Out-of-range settings are rejected when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        ransac.Options(**kwargs)  # type: ignore[arg-type]


def test_rejects_too_few_data() -> None:
    """Fewer data than a minimal sample cannot be sampled."""
    with pytest.raises(InvalidInputError, match="fewer than a minimal sample"):
        ransac.estimate(LineEstimator(np.zeros((1, 2))), OPTIONS)
