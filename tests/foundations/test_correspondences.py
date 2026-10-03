"""Tests of the unchecked two-view building blocks in ``foundations.correspondences``."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab.foundations import correspondences as co
from slamlab.foundations.errors import DegenerateError


def test_homogeneous_and_normalised_plane() -> None:
    """Points gain a 1; bearings map to ``f / f_z``."""
    np.testing.assert_array_equal(co.homogeneous(np.array([[2.0, 3.0]])), [[2.0, 3.0, 1.0]])
    np.testing.assert_allclose(
        co.normalised_plane(np.array([[1.0, 2.0, 4.0]])), [[0.25, 0.5, 1.0]], rtol=0, atol=0
    )


def test_hartley_normalisation_centres_and_scales() -> None:
    """The similarity puts the centroid at the origin and the mean distance at √2."""
    points = np.random.default_rng(0).uniform(100.0, 600.0, size=(50, 2))
    t = co.hartley_normalisation(points)
    assert t is not None
    mapped = co.homogeneous(points) @ t.T
    np.testing.assert_allclose(mapped[:, :2].mean(axis=0), [0.0, 0.0], rtol=0, atol=1e-12)
    assert np.mean(np.linalg.norm(mapped[:, :2], axis=1)) == pytest.approx(np.sqrt(2.0))
    assert co.hartley_normalisation(np.ones((5, 2))) is None


def test_null_vector_needs_a_unique_solution() -> None:
    """A rank-8 system gives its null vector; a rank-7 one gives None."""
    rng = np.random.default_rng(1)
    truth = rng.normal(size=9)
    truth /= np.linalg.norm(truth)
    basis = np.linalg.svd(truth[None])[2][1:]  # eight vectors orthogonal to the truth
    system = rng.normal(size=(12, 8)) @ basis
    found = co.null_vector(system)
    assert found is not None
    assert min(np.abs(found - truth).max(), np.abs(found + truth).max()) < 1e-12
    assert co.null_vector(rng.normal(size=(12, 7)) @ basis[:7]) is None


def test_pair_estimator_delegates_to_its_solver() -> None:
    """The estimator fits subsets with its solver, and an unsolvable subset gives no model."""
    data = (np.arange(10.0).reshape(5, 2), np.arange(10.0, 20.0).reshape(5, 2))

    def solver(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64] | None:
        return None if len(a) < 3 else np.eye(3) * float(a.sum() + b.sum())

    def residual(model: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.full(5, float(model[0, 0]))

    estimator = co.PairEstimator(data, solver, residual, sample_size=3)
    assert (estimator.sample_size, estimator.size) == (3, 5)
    subset = np.array([0, 1, 2])
    (model,) = estimator.fit_minimal(subset)
    assert model[0, 0] == data[0][subset].sum() + data[1][subset].sum()
    assert estimator.fit_nonminimal(np.array([0, 1])) == []
    np.testing.assert_array_equal(estimator.residuals(model), np.full(5, model[0, 0]))


def test_unique_raises_on_no_solution() -> None:
    """No solution raises DegenerateError naming the matrix; a solution passes through."""
    with pytest.raises(DegenerateError, match="E: the correspondences"):
        co.unique(None, "E")
    assert co.unique(np.eye(3), "E")[0, 0] == 1.0
