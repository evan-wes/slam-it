"""SO(3) tests: parity of every backend with the mpmath fixture, properties, and validation."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.foundations import so3
from slam_it.foundations.errors import InvalidInputError
from tests.conftest import (
    check_parity,
    load_fixture,
    numeric_right_jacobian,
    parity_params,
    protocol_members,
    random_rotation_vectors,
    require_function,
)

SO3 = load_fixture("foundations/so3.json")
FUNCTIONS: dict[str, Callable[..., NDArray[np.float64]]] = {
    name: getattr(so3, name) for name in SO3["functions"]
}
PARITY_CASES = [(name, case) for name, spec in SO3["functions"].items() for case in spec["cases"]]
FINITE_DIFFERENCE_STEP = 1e-6
"""Central-difference step for checking Jacobians; truncation and rounding both ~1e-10."""


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    assert set(SO3["functions"]) == protocol_members(so3)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(("function", "case"), parity_params(SO3))
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend matches the mpmath oracle under the function's tolerance policy."""
    check_parity(so3, SO3, function, case, backend)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_batches_keep_their_leading_shape(backend: str) -> None:
    """Every function accepts any leading batch shape and returns the same one."""
    rng = np.random.default_rng(0)
    omega = rng.normal(size=(2, 5, 3))
    for name in ("hat", "exp", "right_jacobian"):
        chosen = require_function(backend, so3.__name__, name)
        assert FUNCTIONS[name](omega, backend=chosen).shape == (2, 5, 3, 3)
    assert so3.log(so3.exp(omega), backend=backend).shape == (2, 5, 3)
    assert so3.to_quaternion(so3.exp(omega), backend=backend).shape == (2, 5, 4)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_round_trips(backend: str) -> None:
    """log(exp(w)) = w for |w| < pi; vee(hat(w)) = w; quaternions round-trip."""
    omega = random_rotation_vectors(500)
    np.testing.assert_allclose(
        so3.log(so3.exp(omega, backend=backend), backend=backend), omega, rtol=0.0, atol=1e-12
    )
    np.testing.assert_array_equal(so3.vee(so3.hat(omega, backend=backend), backend=backend), omega)
    rotation = so3.exp(omega)
    back = so3.from_quaternion(so3.to_quaternion(rotation, backend=backend), backend=backend)
    np.testing.assert_allclose(back, rotation, rtol=0.0, atol=1e-14)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_rotations_are_orthonormal(backend: str) -> None:
    """The exponential produces orthonormal matrices with determinant +1."""
    rotation = so3.exp(random_rotation_vectors(500) * 1.7, backend=backend)
    gram = np.einsum("nji,njk->nik", rotation, rotation)
    np.testing.assert_allclose(gram, np.broadcast_to(np.eye(3), gram.shape), rtol=0.0, atol=1e-14)
    np.testing.assert_allclose(np.linalg.det(rotation), 1.0, rtol=0.0, atol=1e-14)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_right_jacobian_matches_finite_differences(backend: str) -> None:
    """exp(w + d) ≈ exp(w) exp(Jr(w) d): Jr against central differences of log."""
    for omega in random_rotation_vectors(50, seed=3):
        numeric = numeric_right_jacobian(omega, so3.exp, so3.log, lambda a, b: a.T @ b)
        np.testing.assert_allclose(
            so3.right_jacobian(omega, backend=backend), numeric, rtol=0.0, atol=1e-8
        )


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_jacobian_identities(backend: str) -> None:
    """Jl(w) = Jr(-w), and the inverse Jacobians invert them."""
    omega = random_rotation_vectors(300) * 1.9  # up to ~1.9 pi, inside the 2 pi limit
    jr = so3.right_jacobian(omega, backend=backend)
    np.testing.assert_allclose(
        so3.left_jacobian(omega, backend=backend), so3.right_jacobian(-omega), rtol=0, atol=1e-15
    )
    identity = np.broadcast_to(np.eye(3), jr.shape)
    np.testing.assert_allclose(
        so3.right_jacobian_inverse(omega, backend=backend) @ jr, identity, rtol=0, atol=1e-12
    )


def test_cross_check_with_scipy_and_manif() -> None:
    """At moderate angles the reference agrees with SciPy and manif (secondary oracles)."""
    rotation_module = pytest.importorskip("scipy.spatial.transform")
    manifpy = pytest.importorskip("manifpy")
    omega = random_rotation_vectors(200, seed=5)
    expected = rotation_module.Rotation.from_rotvec(omega).as_matrix()
    np.testing.assert_allclose(so3.exp(omega), expected, rtol=0.0, atol=1e-14)
    for w in omega[np.linalg.norm(omega, axis=-1) > 0.1]:
        np.testing.assert_allclose(
            so3.right_jacobian(w), manifpy.SO3Tangent(w).rjac(), rtol=0.0, atol=1e-13
        )


def test_log_at_pi_uses_the_canonical_sign() -> None:
    """At exactly pi the largest-magnitude component is positive, in every backend."""
    for backend in backends.available():
        for axis in np.eye(3):
            rotation = 2.0 * np.outer(axis, axis) - np.eye(3)
            np.testing.assert_allclose(
                so3.log(rotation, backend=backend), np.pi * axis, rtol=0.0, atol=1e-15
            )


def test_quaternion_double_cover() -> None:
    """The quaternions q and -q give the same rotation."""
    q = so3.to_quaternion(so3.exp(random_rotation_vectors(100)))
    np.testing.assert_allclose(so3.from_quaternion(q), so3.from_quaternion(-q), rtol=0, atol=0)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_core_errors_reach_python_as_slam_it_errors(backend: str) -> None:
    """A core error crosses the binding as the SLAM It! exception of its kind."""
    compiled = backends.function(
        require_function(backend, so3.__name__, "log"), so3.__name__, "log"
    )
    with pytest.raises(InvalidInputError, match="orthonormal"):
        compiled(np.full((1, 3, 3), 0.5))


def test_unknown_backend_is_rejected() -> None:
    """An unknown backend name raises ValueError before any work is done."""
    with pytest.raises(ValueError, match="unknown backend"):
        so3.exp([1.0, 2.0, 3.0], backend="fortran")
