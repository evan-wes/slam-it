"""SE(3) tests: parity with the mpmath fixture, group properties, and validation."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import se3
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import (
    check_parity,
    load_fixture,
    numeric_right_jacobian,
    parity_params,
    protocol_members,
    random_rotation_vectors,
)

SE3 = load_fixture("foundations/se3.json")


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(SE3["functions"]) == protocol_members(se3)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(SE3))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend matches the mpmath oracle under the function's tolerance policy."""
    check_parity(se3, SE3, function, case, backend)


def _random_twists(n: int, seed: int = 0) -> NDArray[np.float64]:
    """Random twists: Gaussian translation parts, rotation angles in [0, pi)."""
    rho = np.random.default_rng(seed + 1000).normal(size=(n, 3))
    return np.concatenate([rho, random_rotation_vectors(n, seed)], axis=-1)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_group_properties(backend: str) -> None:
    """log(exp(ξ)) = ξ; T T⁻¹ = I; composition is associative; act matches the matrix."""
    xi = _random_twists(200)
    t = se3.exp(xi, backend=backend)
    np.testing.assert_allclose(se3.log(t, backend=backend), xi, rtol=0, atol=1e-12)
    identity = np.broadcast_to(np.eye(4), t.shape)
    np.testing.assert_allclose(
        se3.compose(t, se3.inverse(t, backend=backend), backend=backend),
        identity,
        rtol=0,
        atol=1e-13,
    )
    a, b, c = t[:60], t[60:120], t[120:180]
    np.testing.assert_allclose(
        se3.compose(se3.compose(a, b), c, backend=backend),
        se3.compose(a, se3.compose(b, c), backend=backend),
        rtol=0,
        atol=1e-13,
    )
    points = np.random.default_rng(1).normal(size=(200, 3))
    homogeneous = np.einsum("nij,nj->ni", t, np.concatenate([points, np.ones((200, 1))], axis=-1))
    np.testing.assert_allclose(se3.act(t, points, backend=backend), homogeneous[:, :3], atol=1e-13)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_adjoint_conjugates_the_exponential(backend: str) -> None:
    """T exp(ξ) T⁻¹ = exp(Ad_T ξ)."""
    t = se3.exp(_random_twists(50, seed=2))
    xi = _random_twists(50, seed=3)
    left = se3.compose(se3.compose(t, se3.exp(xi)), se3.inverse(t))
    right = se3.exp(np.einsum("nij,nj->ni", se3.adjoint(t, backend=backend), xi))
    np.testing.assert_allclose(left, right, rtol=0, atol=1e-12)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_right_jacobian_matches_finite_differences(backend: str) -> None:
    """exp(ξ + δ) ≈ exp(ξ) exp(Jr(ξ) δ): Jr against central differences of log."""
    for xi in _random_twists(20, seed=4) * 0.8:
        numeric = numeric_right_jacobian(
            xi, se3.exp, se3.log, lambda a, b: se3.compose(se3.inverse(a), b)
        )
        np.testing.assert_allclose(se3.right_jacobian(xi, backend=backend), numeric, atol=1e-8)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_inverse_jacobians_invert(backend: str) -> None:
    """Jr⁻¹ Jr = I and Jl⁻¹ Jl = I."""
    xi = _random_twists(100, seed=5)
    identity = np.broadcast_to(np.eye(6), (100, 6, 6))
    for j, j_inv in (
        (se3.right_jacobian, se3.right_jacobian_inverse),
        (se3.left_jacobian, se3.left_jacobian_inverse),
    ):
        np.testing.assert_allclose(
            j_inv(xi, backend=backend) @ j(xi, backend=backend), identity, rtol=0, atol=1e-12
        )


def test_binary_functions_reject_mismatched_batches() -> None:
    """Compose and act need equal leading shapes."""
    t = se3.exp(_random_twists(3))
    with pytest.raises(InvalidInputError, match="leading shapes"):
        se3.compose(t, t[:2])
    with pytest.raises(InvalidInputError, match="leading shapes"):
        se3.act(t, np.zeros((2, 3)))
