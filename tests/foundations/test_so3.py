"""SO(3) tests: parity of every backend with the mpmath fixture, properties, and validation."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import so3
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import assert_matches_fixture, load_fixture, require_function

SO3 = load_fixture("foundations/so3.json")
FUNCTIONS: dict[str, Callable[..., NDArray[np.float64]]] = {
    name: getattr(so3, name) for name in SO3["functions"]
}
PARITY_CASES = [(name, case) for name, spec in SO3["functions"].items() for case in spec["cases"]]
FINITE_DIFFERENCE_STEP = 1e-6
"""Central-difference step for checking Jacobians; truncation and rounding both ~1e-10."""


def test_fixture_covers_the_protocol() -> None:
    """Every function in the compiled-backend Protocol has fixture cases."""
    members = {m for m in vars(so3.Implementation) if not m.startswith("_")}
    assert set(SO3["functions"]) == members


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("function", "case"), PARITY_CASES, ids=[f"{n}:{c['name']}" for n, c in PARITY_CASES]
)
def test_matches_fixture(backend: str, function: str, case: dict[str, Any]) -> None:
    """Each backend matches the mpmath oracle under the function's tolerance policy."""
    chosen = require_function(backend, so3.__name__, function)
    got = FUNCTIONS[function](case["input"], backend=chosen)
    assert got.dtype == np.float64
    assert_matches_fixture(got, case["expected"], SO3["functions"][function]["tolerance"])


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_batches_keep_their_leading_shape(backend: str) -> None:
    """Every function accepts any leading batch shape and returns the same one."""
    rng = np.random.default_rng(0)
    omega = rng.normal(size=(2, 5, 3))
    for name in ("hat", "exp", "right_jacobian"):
        chosen = require_function(backend, so3.__name__, name)
        assert FUNCTIONS[name](omega, backend=chosen).shape == (2, 5, 3, 3)
    if backend in backends.LOCKSTEP_BACKENDS:
        assert so3.log(so3.exp(omega), backend=backend).shape == (2, 5, 3)
        assert so3.to_quaternion(so3.exp(omega), backend=backend).shape == (2, 5, 4)


def _random_omega(n: int, seed: int = 0) -> NDArray[np.float64]:
    """Random rotation vectors with angles spread over [0, pi)."""
    rng = np.random.default_rng(seed)
    axes = rng.normal(size=(n, 3))
    axes /= np.linalg.norm(axes, axis=-1, keepdims=True)
    return axes * rng.uniform(0.0, np.pi * 0.999, size=(n, 1))


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_round_trips(backend: str) -> None:
    """log(exp(w)) = w for |w| < pi; vee(hat(w)) = w; quaternions round-trip."""
    omega = _random_omega(500)
    np.testing.assert_allclose(
        so3.log(so3.exp(omega, backend=backend), backend=backend), omega, rtol=0.0, atol=1e-12
    )
    np.testing.assert_array_equal(so3.vee(so3.hat(omega, backend=backend), backend=backend), omega)
    rotation = so3.exp(omega)
    back = so3.from_quaternion(so3.to_quaternion(rotation, backend=backend), backend=backend)
    np.testing.assert_allclose(back, rotation, rtol=0.0, atol=1e-14)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_rotations_are_orthonormal(backend: str) -> None:
    """The exponential produces orthonormal matrices with determinant +1."""
    rotation = so3.exp(_random_omega(500) * 1.7, backend=backend)
    gram = np.einsum("nji,njk->nik", rotation, rotation)
    np.testing.assert_allclose(gram, np.broadcast_to(np.eye(3), gram.shape), rtol=0.0, atol=1e-14)
    np.testing.assert_allclose(np.linalg.det(rotation), 1.0, rtol=0.0, atol=1e-14)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_right_jacobian_matches_finite_differences(backend: str) -> None:
    """exp(w + d) ≈ exp(w) exp(Jr(w) d): Jr against central differences of log."""
    h = FINITE_DIFFERENCE_STEP
    for omega in _random_omega(50, seed=3):
        base_inv = so3.exp(omega).T
        numeric = np.empty((3, 3))
        for k in range(3):
            step = np.zeros(3)
            step[k] = h
            plus = so3.log(base_inv @ so3.exp(omega + step))
            minus = so3.log(base_inv @ so3.exp(omega - step))
            numeric[:, k] = (plus - minus) / (2.0 * h)
        np.testing.assert_allclose(
            so3.right_jacobian(omega, backend=backend), numeric, rtol=0.0, atol=1e-8
        )


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_jacobian_identities(backend: str) -> None:
    """Jl(w) = Jr(-w), and the inverse Jacobians invert them."""
    omega = _random_omega(300) * 1.9  # up to ~1.9 pi, inside the 2 pi limit
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
    omega = _random_omega(200, seed=5)
    expected = rotation_module.Rotation.from_rotvec(omega).as_matrix()
    np.testing.assert_allclose(so3.exp(omega), expected, rtol=0.0, atol=1e-14)
    for w in omega[np.linalg.norm(omega, axis=-1) > 0.1]:
        np.testing.assert_allclose(
            so3.right_jacobian(w), manifpy.SO3Tangent(w).rjac(), rtol=0.0, atol=1e-13
        )


def test_log_at_pi_uses_the_canonical_sign() -> None:
    """At exactly pi the largest-magnitude component is positive, in every lock-step backend."""
    for backend in sorted(backends.LOCKSTEP_BACKENDS):
        for axis in np.eye(3):
            rotation = 2.0 * np.outer(axis, axis) - np.eye(3)
            np.testing.assert_allclose(
                so3.log(rotation, backend=backend), np.pi * axis, rtol=0.0, atol=1e-15
            )


def test_quaternion_double_cover() -> None:
    """The quaternions q and -q give the same rotation."""
    q = so3.to_quaternion(so3.exp(_random_omega(100)))
    np.testing.assert_allclose(so3.from_quaternion(q), so3.from_quaternion(-q), rtol=0, atol=0)


BAD_INPUTS: list[tuple[str, object, str]] = [
    ("exp", [1.0, 2.0], "shape"),
    ("exp", [[1.0, 2.0, 3.0, 4.0]], "shape"),
    ("exp", [None, 1.0, 2.0], "real numbers"),
    ("exp", ["1", "2", "3"], "real numbers"),
    ("exp", [True, False, True], "real numbers"),
    ("exp", np.array([1 + 2j, 0, 0]), "real numbers"),
    ("exp", [np.nan, 0.0, 0.0], "finite"),
    ("exp", [0.0, np.inf, 0.0], "finite"),
    ("exp", np.ma.array([1.0, 2.0, 3.0], mask=[False, True, False]), "masked"),
    ("log", np.full((3, 3), 0.5), "orthonormal"),
    ("log", -np.eye(3), "determinant"),
    ("to_quaternion", np.eye(3) * 1.1, "orthonormal"),
    ("vee", np.eye(3), "skew-symmetric"),
    ("from_quaternion", [1.0, 1.0, 0.0, 0.0], "norm"),
    ("right_jacobian_inverse", [7.0, 0.0, 0.0], "2\\*pi"),
    ("left_jacobian_inverse", [0.0, 0.0, -6.3], "2\\*pi"),
]


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize(
    ("function", "bad", "message"), BAD_INPUTS, ids=[f"{f}:{m}" for f, _, m in BAD_INPUTS]
)
def test_rejects_invalid_input(backend: str, function: str, bad: object, message: str) -> None:
    """Every backend rejects bad input the same way: Python validates before dispatching."""
    with pytest.raises(InvalidInputError, match=message):
        FUNCTIONS[function](bad, backend=backend)


@pytest.mark.parametrize("backend", ["cpp"])
def test_core_errors_reach_python_as_slamlab_errors(backend: str) -> None:
    """A core error crosses the binding as the slamlab exception of its kind."""
    compiled = backends.function(
        require_function(backend, so3.__name__, "log"), so3.__name__, "log"
    )
    with pytest.raises(InvalidInputError, match="orthonormal"):
        compiled(np.full((1, 3, 3), 0.5))


def test_unknown_backend_is_rejected() -> None:
    """An unknown backend name raises ValueError before any work is done."""
    with pytest.raises(ValueError, match="unknown backend"):
        so3.exp([1.0, 2.0, 3.0], backend="fortran")
