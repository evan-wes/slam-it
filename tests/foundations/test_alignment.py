"""Umeyama alignment tests: parity with the mpmath fixture, recovery, and degeneracy."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from slam_it_all import backends
from slam_it_all.foundations import alignment, se3
from slam_it_all.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import assert_matches_fixture, load_fixture, require_function

FIXTURE = load_fixture("foundations/alignment.json")
SPEC = FIXTURE["functions"]["umeyama"]
THIN = load_fixture("foundations/ill_conditioned.json")["functions"]["umeyama"]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", SPEC["cases"], ids=[c["name"] for c in SPEC["cases"]])
def test_matches_fixture(backend: str, case: dict[str, Any]) -> None:
    """Each backend's similarity matrix matches the mpmath oracle."""
    chosen = require_function(backend, alignment.__name__, "umeyama")
    result = alignment.align(**case["input"], backend=chosen)
    assert_matches_fixture(result.as_matrix(), case["expected"], SPEC["tolerance"])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", THIN["cases"], ids=[c["name"] for c in THIN["cases"]])
def test_thin_triangles_match_fixture(backend: str, case: dict[str, Any]) -> None:
    """Three points on a thin triangle give the oracle's rotation (panel L1).

    The covariance has σ₂/σ₁ near 1e-3; a 3x3 SVD through ``MᵀM`` turned it into a rotation off
    by 0.7.
    """
    chosen = require_function(backend, alignment.__name__, "umeyama")
    result = alignment.align(**case["input"], backend=chosen)
    assert_matches_fixture(result.as_matrix(), case["expected"], THIN["tolerance"])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("with_scale", [False, True])
def test_recovers_a_known_transform(backend: str, with_scale: bool) -> None:
    """Noise-free correspondences recover R, t (and s) to rounding."""
    rng = np.random.default_rng(0)
    source = rng.normal(size=(40, 3))
    truth = se3.exp([0.5, -1.0, 2.0, 0.3, -2.1, 0.9])
    scale = 0.4 if with_scale else 1.0
    target = scale * source @ truth[:3, :3].T + truth[:3, 3]
    result = alignment.align(source, target, with_scale=with_scale, backend=backend)
    np.testing.assert_allclose(result.rotation, truth[:3, :3], rtol=0, atol=1e-13)
    np.testing.assert_allclose(result.translation, truth[:3, 3], rtol=0, atol=1e-13)
    assert result.scale == pytest.approx(scale, abs=1e-13)
    np.testing.assert_allclose(result.apply(source), target, rtol=0, atol=1e-12)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_degenerate_points_are_reported(backend: str) -> None:
    """Collinear or too few points do not determine the rotation: DegenerateError."""
    chosen = require_function(backend, alignment.__name__, "umeyama")
    line = np.outer(np.arange(6.0), [1.0, 2.0, -1.0])
    with pytest.raises(DegenerateError, match="collinear"):
        alignment.align(line, line, backend=chosen)
    pts = np.random.default_rng(1).normal(size=(5, 3))
    with pytest.raises(DegenerateError, match="fewer than three"):
        alignment.align(pts, pts, weights=[1.0, 1.0, 0.0, 0.0, 0.0], backend=chosen)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"source": np.zeros((5, 3)), "target": np.zeros((4, 3))}, "equal"),
        ({"source": np.zeros((5, 2)), "target": np.zeros((5, 2))}, "shape"),
        ({"source": np.eye(3), "target": np.eye(3), "weights": [1.0, -1.0, 1.0]}, "non-negative"),
        ({"source": np.eye(3), "target": np.eye(3), "weights": [1.0, 1.0]}, "non-negative"),
    ],
)
def test_rejects_invalid_input(kwargs: dict[str, Any], message: str) -> None:
    """Mismatched shapes and bad weights are rejected before any backend runs."""
    with pytest.raises(InvalidInputError, match=message):
        alignment.align(**kwargs)
