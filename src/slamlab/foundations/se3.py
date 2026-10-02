"""SE(3), rigid-body transforms: the Python reference and backend dispatch.

Conventions (design doc, "Conventions"): a transform is a 4x4 homogeneous matrix
``T = [[R, t], [0, 1]]``; ``T_a_b`` maps frame b to frame a; the tangent is a 6-vector
``ξ = (ρ, θ)``, translation part first; perturbations are right (local),
``T ⊕ δ = T · exp(δ)``. Every function is batched over any leading dimensions.

The Jacobians use the block form of Barfoot (*State Estimation for Robotics*, §7.1.5)::

    Jl(ξ) = [[Jl(θ), Q(ρ, θ)], [0, Jl(θ)]],     Jr(ξ) = Jl(-ξ)

``Q``'s closed-form coefficients cancel catastrophically at small angles (the worst loses
about 60 eps / θ⁴), so below :data:`SERIES_THRESHOLD` they are evaluated from their exact
power series, summed to convergence; above it the closed form is accurate to 1e-14.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import so3
from slamlab.foundations.arrays import as_batch, unbatch
from slamlab.foundations.errors import InvalidInputError

SERIES_THRESHOLD = 1.0
"""Below this rotation angle, ``Q``'s three coefficients come from their power series."""

SERIES_TERMS = 10
"""Terms summed for each series; at θ = 1 the first omitted term is below 1e-19."""

BOTTOM_ROW_TOLERANCE = 1e-12
"""A transform's last row must equal ``[0, 0, 0, 1]`` within this."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.se3`` submodule provides.

    Each takes and returns float64 C-contiguous arrays with a leading batch dimension N.
    """

    def exp(self, xi: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 6) twists (ρ, θ) to (N, 4, 4) transforms."""
        ...

    def log(self, transform: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 4, 4) transforms to (N, 6) twists with rotation angle in [0, π]."""
        ...

    def inverse(self, transform: NDArray[np.float64]) -> NDArray[np.float64]:
        """Invert (N, 4, 4) transforms."""
        ...

    def compose(self, a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
        """Compose (N, 4, 4) transforms: ``a @ b``."""
        ...

    def act(
        self, transform: NDArray[np.float64], points: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Apply (N, 4, 4) transforms to (N, 3) points."""
        ...

    def adjoint(self, transform: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 4, 4) transforms to their (N, 6, 6) adjoints."""
        ...

    def right_jacobian(self, xi: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 6) twists to their (N, 6, 6) right Jacobians."""
        ...

    def left_jacobian(self, xi: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 6) twists to their (N, 6, 6) left Jacobians."""
        ...

    def right_jacobian_inverse(self, xi: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 6) twists to the inverses of their right Jacobians."""
        ...

    def left_jacobian_inverse(self, xi: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 6) twists to the inverses of their left Jacobians."""
        ...


# --- validation ---------------------------------------------------------------------------


def _check_transform(transform: NDArray[np.float64]) -> None:
    """Raise unless every (4, 4) matrix in the (N, 4, 4) batch is a rigid transform."""
    bottom = transform[:, 3, :] - np.array([0.0, 0.0, 0.0, 1.0])
    if np.any(np.abs(bottom) > BOTTOM_ROW_TOLERANCE):
        raise InvalidInputError("transform: last row is not [0, 0, 0, 1]")
    so3.check_rotation(transform[:, :3, :3])


def _transform_batch(
    value: ArrayLike, name: str = "transform"
) -> tuple[NDArray[np.float64], tuple[int, ...]]:
    """Validate and flatten a batch of transforms."""
    flat, lead = as_batch(value, (4, 4), name)
    _check_transform(flat)
    return flat, lead


def _xi_batch(
    value: ArrayLike, limit: float | None = None
) -> tuple[NDArray[np.float64], tuple[int, ...]]:
    """Validate and flatten a batch of twists, optionally bounding the rotation angle."""
    flat, lead = as_batch(value, (6,), "xi")
    if limit is not None and np.any(np.linalg.norm(flat[:, 3:], axis=-1) >= limit):
        raise InvalidInputError("xi: inverse Jacobian undefined at |theta| >= 2*pi")
    return flat, lead


# --- reference implementations (flat batches) ---------------------------------------------


def _series(theta: float, coefficient: Callable[[int], float]) -> float:
    """Sum ``Σ_k (-1)^k θ^{2k} c(k)`` for :data:`SERIES_TERMS` terms."""
    t2 = theta * theta
    total, power = 0.0, 1.0
    for k in range(SERIES_TERMS):
        total += (-1) ** k * power * coefficient(k)
        power *= t2
    return total


def _q_coefficients(theta: float) -> tuple[float, float, float]:
    """Compute ``Q``'s three coefficients: series below the threshold, closed form above.

    They are ``(θ - sin θ)/θ³``, ``(θ² + 2cos θ - 2)/(2θ⁴)`` and
    ``(2θ - 3 sin θ + θ cos θ)/(2θ⁵)``.
    """
    if theta < SERIES_THRESHOLD:
        return (
            _series(theta, lambda k: 1.0 / math.factorial(2 * k + 3)),
            _series(theta, lambda k: 1.0 / math.factorial(2 * k + 4)),
            _series(theta, lambda k: (k + 1) / math.factorial(2 * k + 5)),
        )
    s, c = math.sin(theta), math.cos(theta)
    return (
        (theta - s) / theta**3,
        (theta**2 + 2.0 * c - 2.0) / (2.0 * theta**4),
        (2.0 * theta - 3.0 * s + theta * c) / (2.0 * theta**5),
    )


def _q_matrix(rho: NDArray[np.float64], theta_vec: NDArray[np.float64]) -> NDArray[np.float64]:
    """Barfoot's ``Q(ρ, θ)`` for one twist (the coupling block of the left Jacobian)."""
    r = so3.hat(rho)
    p = so3.hat(theta_vec)
    c1, c2, c3 = _q_coefficients(float(np.linalg.norm(theta_vec)))
    pr, rp, prp = p @ r, r @ p, p @ r @ p
    q = (
        0.5 * r
        + c1 * (pr + rp + prp)
        + c2 * (p @ pr + rp @ p - 3.0 * prp)
        + c3 * (prp @ p + p @ prp)
    )
    return np.asarray(q, dtype=np.float64)


def _exp(xi: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute ``exp`` on an (N, 6) batch (reference): ``R = exp(θ)``, ``t = Jl(θ) ρ``."""
    out = np.zeros((xi.shape[0], 4, 4))
    out[:, :3, :3] = so3.exp(xi[:, 3:])
    out[:, :3, 3] = np.einsum("nij,nj->ni", so3.left_jacobian(xi[:, 3:]), xi[:, :3])
    out[:, 3, 3] = 1.0
    return out


def _log(transform: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute ``log`` on an (N, 4, 4) batch (reference): ``ρ = Jl(θ)⁻¹ t``."""
    theta = so3.log(transform[:, :3, :3])
    rho = np.einsum("nij,nj->ni", so3.left_jacobian_inverse(theta), transform[:, :3, 3])
    return np.concatenate([rho, theta], axis=-1)


def _inverse(transform: NDArray[np.float64]) -> NDArray[np.float64]:
    """Invert an (N, 4, 4) batch (reference): ``[Rᵀ, -Rᵀ t]``."""
    out = np.zeros_like(transform)
    rt = np.swapaxes(transform[:, :3, :3], 1, 2)
    out[:, :3, :3] = rt
    out[:, :3, 3] = -np.einsum("nij,nj->ni", rt, transform[:, :3, 3])
    out[:, 3, 3] = 1.0
    return out


def _compose(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compose (N, 4, 4) batches (reference), keeping the last row exact."""
    out = np.zeros_like(a)
    out[:, :3, :3] = a[:, :3, :3] @ b[:, :3, :3]
    out[:, :3, 3] = np.einsum("nij,nj->ni", a[:, :3, :3], b[:, :3, 3]) + a[:, :3, 3]
    out[:, 3, 3] = 1.0
    return out


def _act(transform: NDArray[np.float64], points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Apply transforms to points (reference): ``R p + t``."""
    moved = np.einsum("nij,nj->ni", transform[:, :3, :3], points) + transform[:, :3, 3]
    return np.asarray(moved, dtype=np.float64)


def _adjoint(transform: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute adjoints of an (N, 4, 4) batch (reference): ``[[R, t^ R], [0, R]]``."""
    rot = transform[:, :3, :3]
    out = np.zeros((transform.shape[0], 6, 6))
    out[:, :3, :3] = rot
    out[:, 3:, 3:] = rot
    out[:, :3, 3:] = so3.hat(transform[:, :3, 3]) @ rot
    return out


def _left_jacobian(xi: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the left Jacobian of an (N, 6) batch (reference)."""
    jl = so3.left_jacobian(xi[:, 3:])
    out = np.zeros((xi.shape[0], 6, 6))
    out[:, :3, :3] = jl
    out[:, 3:, 3:] = jl
    for i in range(xi.shape[0]):
        out[i, :3, 3:] = _q_matrix(xi[i, :3], xi[i, 3:])
    return out


def _right_jacobian(xi: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the right Jacobian of an (N, 6) batch (reference): ``Jr(ξ) = Jl(-ξ)``."""
    return _left_jacobian(-xi)


def _left_jacobian_inverse(xi: NDArray[np.float64]) -> NDArray[np.float64]:
    """Invert the left Jacobian blockwise (reference): ``[[J⁻¹, -J⁻¹ Q J⁻¹], [0, J⁻¹]]``."""
    jl_inv = so3.left_jacobian_inverse(xi[:, 3:])
    out = np.zeros((xi.shape[0], 6, 6))
    out[:, :3, :3] = jl_inv
    out[:, 3:, 3:] = jl_inv
    for i in range(xi.shape[0]):
        out[i, :3, 3:] = -jl_inv[i] @ _q_matrix(xi[i, :3], xi[i, 3:]) @ jl_inv[i]
    return out


def _right_jacobian_inverse(xi: NDArray[np.float64]) -> NDArray[np.float64]:
    """Invert the right Jacobian (reference): ``Jr(ξ)⁻¹ = Jl(-ξ)⁻¹``."""
    return _left_jacobian_inverse(-xi)


# --- public, batched API -------------------------------------------------------------------


def exp(xi: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map twists ``(ρ, θ)`` to rigid transforms.

    Parameters
    ----------
    xi : array_like, shape (..., 6)
        Twists: translation part ρ, then rotation vector θ.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 4, 4)
        ``[[exp(θ), Jl(θ) ρ], [0, 1]]``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `xi` is not a finite real array of shape (..., 6).
    """
    flat, lead = _xi_batch(xi)
    return unbatch(backends.dispatch(backend, __name__, "exp", _exp, flat), lead)


def log(transform: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map rigid transforms to twists ``(ρ, θ)``, rotation angle in [0, π].

    Parameters
    ----------
    transform : array_like, shape (..., 4, 4)
        Rigid transforms: a rotation block (see :func:`slamlab.foundations.so3.log`) and a
        last row of ``[0, 0, 0, 1]``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 6)
        Twists with ``exp(log(T)) == T``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `transform` is not a finite real (..., 4, 4) array of rigid transforms.
    """
    flat, lead = _transform_batch(transform)
    return unbatch(backends.dispatch(backend, __name__, "log", _log, flat), lead)


def inverse(transform: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Invert rigid transforms: ``T⁻¹ = [[Rᵀ, -Rᵀ t], [0, 1]]``.

    Parameters
    ----------
    transform : array_like, shape (..., 4, 4)
        Rigid transforms.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 4, 4)
        The inverses.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `transform` is not a finite real (..., 4, 4) array of rigid transforms.
    """
    flat, lead = _transform_batch(transform)
    return unbatch(backends.dispatch(backend, __name__, "inverse", _inverse, flat), lead)


def compose(a: ArrayLike, b: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compose rigid transforms: ``T_a_c = T_a_b @ T_b_c``.

    Parameters
    ----------
    a, b : array_like, shape (..., 4, 4)
        Rigid transforms with the same leading shape.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 4, 4)
        The products, with the last row exactly ``[0, 0, 0, 1]``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If either is not an array of rigid transforms, or their leading shapes differ.
    """
    flat_a, lead = _transform_batch(a, "a")
    flat_b, lead_b = _transform_batch(b, "b")
    if lead != lead_b:
        raise InvalidInputError(f"a and b: leading shapes differ, {lead} vs {lead_b}")
    result = backends.dispatch(backend, __name__, "compose", _compose, flat_a, flat_b)
    return unbatch(result, lead)


def act(transform: ArrayLike, points: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Apply rigid transforms to points: ``p_a = R p_b + t`` for ``T_a_b``.

    Parameters
    ----------
    transform : array_like, shape (..., 4, 4)
        Rigid transforms.
    points : array_like, shape (..., 3)
        Points, with the same leading shape as `transform`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3)
        The transformed points.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the inputs are invalid or their leading shapes differ.
    """
    flat_t, lead = _transform_batch(transform)
    flat_p, lead_p = as_batch(points, (3,), "points")
    if lead != lead_p:
        raise InvalidInputError(f"transform and points: leading shapes differ, {lead} vs {lead_p}")
    return unbatch(backends.dispatch(backend, __name__, "act", _act, flat_t, flat_p), lead)


def adjoint(transform: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute adjoints: ``Ad_T = [[R, hat(t) R], [0, R]]`` for twists ordered ``(ρ, θ)``.

    ``T exp(ξ) T⁻¹ = exp(Ad_T ξ)``.

    Parameters
    ----------
    transform : array_like, shape (..., 4, 4)
        Rigid transforms.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 6, 6)
        The adjoint matrices.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `transform` is not a finite real (..., 4, 4) array of rigid transforms.
    """
    flat, lead = _transform_batch(transform)
    return unbatch(backends.dispatch(backend, __name__, "adjoint", _adjoint, flat), lead)


def right_jacobian(xi: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the right Jacobian, with ``exp(ξ + δ) ≈ exp(ξ) exp(Jr(ξ) δ)``.

    Parameters
    ----------
    xi : array_like, shape (..., 6)
        Twists ``(ρ, θ)``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 6, 6)
        ``Jr(ξ) = Jl(-ξ)``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `xi` is not a finite real array of shape (..., 6).
    """
    flat, lead = _xi_batch(xi)
    result = backends.dispatch(backend, __name__, "right_jacobian", _right_jacobian, flat)
    return unbatch(result, lead)


def left_jacobian(xi: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the left Jacobian ``[[Jl(θ), Q(ρ, θ)], [0, Jl(θ)]]`` (Barfoot, §7.1.5).

    Parameters
    ----------
    xi : array_like, shape (..., 6)
        Twists ``(ρ, θ)``.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 6, 6)
        The left Jacobians.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `xi` is not a finite real array of shape (..., 6).
    """
    flat, lead = _xi_batch(xi)
    result = backends.dispatch(backend, __name__, "left_jacobian", _left_jacobian, flat)
    return unbatch(result, lead)


def right_jacobian_inverse(xi: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the inverse right Jacobian, defined for rotation angle ``|θ| < 2π``.

    Parameters
    ----------
    xi : array_like, shape (..., 6)
        Twists whose rotation angle is below :data:`slamlab.foundations.so3.INVERSE_JACOBIAN_LIMIT`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 6, 6)
        ``Jr(ξ)⁻¹``, computed blockwise.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `xi` is not a finite real array of shape (..., 6), or ``|θ| >= 2π``.
    """
    flat, lead = _xi_batch(xi, so3.INVERSE_JACOBIAN_LIMIT)
    result = backends.dispatch(
        backend, __name__, "right_jacobian_inverse", _right_jacobian_inverse, flat
    )
    return unbatch(result, lead)


def left_jacobian_inverse(xi: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the inverse left Jacobian ``[[J⁻¹, -J⁻¹ Q J⁻¹], [0, J⁻¹]]``.

    Parameters
    ----------
    xi : array_like, shape (..., 6)
        Twists whose rotation angle is below :data:`slamlab.foundations.so3.INVERSE_JACOBIAN_LIMIT`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 6, 6)
        ``Jl(ξ)⁻¹``.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `xi` is not a finite real array of shape (..., 6), or ``|θ| >= 2π``.
    """
    flat, lead = _xi_batch(xi, so3.INVERSE_JACOBIAN_LIMIT)
    result = backends.dispatch(
        backend, __name__, "left_jacobian_inverse", _left_jacobian_inverse, flat
    )
    return unbatch(result, lead)
