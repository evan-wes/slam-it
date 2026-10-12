"""SO(3), the group of 3D rotations: the Python reference and backend dispatch.

Conventions (design doc, "Conventions"): rotations are 3x3 matrices; the tangent space is
R^3 with ``hat``/``vee`` mapping to and from skew-symmetric matrices; perturbations are
right (local), ``R ⊕ δ = R · exp(δ)``; quaternions are Hamilton, stored ``(x, y, z, w)``.

Every function is batched: it accepts any number of leading dimensions and returns the
same leading dimensions. Each validates its input once, here, then computes with the
Python reference or calls the chosen compiled backend (``backend=``), which the parity
tests hold to the reference.

Numerics. Every formula uses a form free of cancellation where one exists
(``(1 - cos θ) / θ²`` as ``½ (sin(θ/2) / (θ/2))²``), switches to a Taylor series below a
named threshold where none does, takes the angle in :func:`log` from ``atan2`` (so rounding
cannot push ``arccos`` out of its domain), and extracts the axis from the symmetric part of
the rotation near θ = π, where the skew part vanishes. The thresholds are module constants
shared with the C++ core.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations.arrays import as_batch, unbatch
from slam_it.foundations.errors import InvalidInputError

EXP_TAYLOR_THRESHOLD = 1e-4
"""Below this angle, ``exp``'s coefficient ``sin θ / θ`` and ``log``'s ``θ / sin θ`` use
their series (truncation error below 1e-17 relative)."""

JACOBIAN_TAYLOR_THRESHOLD = 0.05
"""Below this angle, the Jacobians' ``(θ - sin θ) / θ³`` and the inverse Jacobians'
``1/θ² - cot(θ/2) / (2θ)`` use their series. Both closed forms lose about 6 eps / θ² to
cancellation, about 5e-13 here; the series' truncation error is below 1e-18."""

NEAR_PI_SINE = 1e-3
"""When ``sin θ`` is below this with ``cos θ < 0``, :func:`log` takes the axis from the
symmetric part of the rotation, because the skew part ``sin θ · n`` has lost its accuracy."""

ROTATION_TOLERANCE = 1e-6
"""A matrix is accepted as a rotation if ``|RᵀR - I|`` is within this (element-wise) and its
determinant is positive. Lenient enough for rotations that passed through float32."""

SKEW_TOLERANCE = 1e-9
"""A matrix is accepted as skew-symmetric if ``|W + Wᵀ|`` is within this, relative to
``max(1, |W|)``."""

QUATERNION_TOLERANCE = 1e-6
"""A quaternion is accepted if its norm is within this of 1; it is then normalised."""

INVERSE_JACOBIAN_LIMIT = 2.0 * np.pi - 1e-6
"""The inverse Jacobians are singular at ``|ω| = 2π``; larger angles are rejected."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.so3`` submodule provides.

    Each takes and returns float64 C-contiguous arrays with a leading batch dimension N.
    Python validates first; the cores check again (see the design doc's "Conventions").
    """

    def hat(self, omega: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3) vectors to (N, 3, 3) skew-symmetric matrices."""
        ...

    def vee(self, skew: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3, 3) skew-symmetric matrices to (N, 3) vectors."""
        ...

    def exp(self, omega: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3) rotation vectors to (N, 3, 3) rotation matrices."""
        ...

    def log(self, rotation: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3, 3) rotation matrices to (N, 3) rotation vectors with angle in [0, π]."""
        ...

    def right_jacobian(self, omega: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3) vectors to their (N, 3, 3) right Jacobians."""
        ...

    def left_jacobian(self, omega: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3) vectors to their (N, 3, 3) left Jacobians."""
        ...

    def right_jacobian_inverse(self, omega: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3) vectors to the (N, 3, 3) inverses of their right Jacobians."""
        ...

    def left_jacobian_inverse(self, omega: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3) vectors to the (N, 3, 3) inverses of their left Jacobians."""
        ...

    def from_quaternion(self, quaternion: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 4) xyzw quaternions to (N, 3, 3) rotation matrices."""
        ...

    def to_quaternion(self, rotation: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3, 3) rotation matrices to (N, 4) xyzw quaternions with w >= 0."""
        ...

    def project_to_so3(self, matrix: NDArray[np.float64]) -> NDArray[np.float64]:
        """Map (N, 3, 3) matrices to the nearest (N, 3, 3) rotations (Frobenius norm)."""
        ...


# --- validation ---------------------------------------------------------------------------


def check_rotation(rotation: NDArray[np.float64]) -> None:
    """Raise unless every matrix in a batch is a rotation within :data:`ROTATION_TOLERANCE`.

    Parameters
    ----------
    rotation : ndarray of float64, shape (N, 3, 3)
        A validated, flattened batch of matrices (see
        :func:`slam_it.foundations.arrays.as_batch`).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If any matrix is not orthonormal within the tolerance, or has a non-positive
        determinant.
    """
    gram = np.einsum("nji,njk->nik", rotation, rotation)
    if np.any(np.abs(gram - np.eye(3)) > ROTATION_TOLERANCE):
        raise InvalidInputError(
            f"rotation: not orthonormal within {ROTATION_TOLERANCE}; "
            "use project_to_so3 to repair it explicitly"
        )
    if np.any(np.linalg.det(rotation) <= 0.0):
        raise InvalidInputError("rotation: determinant is not positive (a reflection)")


def _check_skew(skew: NDArray[np.float64]) -> None:
    """Raise unless every (3, 3) matrix in the (N, 3, 3) batch is skew-symmetric."""
    scale = np.maximum(1.0, np.abs(skew).max(axis=(1, 2)))
    asym = np.abs(skew + np.swapaxes(skew, 1, 2)).max(axis=(1, 2))
    if np.any(asym > SKEW_TOLERANCE * scale):
        raise InvalidInputError(f"skew: not skew-symmetric within {SKEW_TOLERANCE}")


# --- reference implementations (flat batches) ---------------------------------------------


def _hat(omega: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute ``hat`` on an (N, 3) batch (reference)."""
    x, y, z = omega[:, 0], omega[:, 1], omega[:, 2]
    zero = np.zeros_like(x)
    return np.stack(
        [
            np.stack([zero, -z, y], axis=-1),
            np.stack([z, zero, -x], axis=-1),
            np.stack([-y, x, zero], axis=-1),
        ],
        axis=-2,
    )


def _vee(skew: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute ``vee`` on an (N, 3, 3) batch of validated skew matrices (reference)."""
    return np.stack([skew[:, 2, 1], skew[:, 0, 2], skew[:, 1, 0]], axis=-1)


def _sinc(theta: NDArray[np.float64]) -> NDArray[np.float64]:
    """``sin θ / θ``, with its series below :data:`EXP_TAYLOR_THRESHOLD`."""
    small = theta < EXP_TAYLOR_THRESHOLD
    safe = np.where(small, 1.0, theta)
    return np.where(small, 1.0 - theta**2 / 6.0, np.sin(safe) / safe)


def _half_versine_coefficient(theta: NDArray[np.float64]) -> NDArray[np.float64]:
    """``(1 - cos θ) / θ²`` as ``½ (sin(θ/2) / (θ/2))²``: no cancellation at small θ."""
    return 0.5 * _sinc(0.5 * theta) ** 2


def _third_coefficient(theta: NDArray[np.float64]) -> NDArray[np.float64]:
    """``(θ - sin θ) / θ³``, with its series below :data:`JACOBIAN_TAYLOR_THRESHOLD`."""
    small = theta < JACOBIAN_TAYLOR_THRESHOLD
    safe = np.where(small, 1.0, theta)
    t2 = theta**2
    series = 1.0 / 6.0 - t2 / 120.0 + t2**2 / 5040.0 - t2**3 / 362880.0
    return np.where(small, series, (safe - np.sin(safe)) / safe**3)


def _inverse_coefficient(theta: NDArray[np.float64]) -> NDArray[np.float64]:
    """``1/θ² - cot(θ/2) / (2θ)``, with its series below the Jacobian threshold."""
    small = theta < JACOBIAN_TAYLOR_THRESHOLD
    safe = np.where(small, 1.0, theta)
    t2 = theta**2
    series = 1.0 / 12.0 + t2 / 720.0 + t2**2 / 30240.0 + t2**3 / 1209600.0
    return np.where(small, series, 1.0 / safe**2 - 1.0 / (2.0 * safe * np.tan(0.5 * safe)))


def _exp(omega: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute ``exp`` (Rodrigues) on an (N, 3) batch (reference)."""
    theta = np.linalg.norm(omega, axis=-1)
    w = _hat(omega)
    a = _sinc(theta)[:, None, None]
    b = _half_versine_coefficient(theta)[:, None, None]
    return np.eye(3) + a * w + b * (w @ w)


def _log(rotation: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute ``log`` on an (N, 3, 3) batch of validated rotations (reference)."""
    v = 0.5 * _vee(rotation - np.swapaxes(rotation, 1, 2))  # sin θ · n
    sine = np.linalg.norm(v, axis=-1)
    cosine = np.clip(0.5 * (np.trace(rotation, axis1=1, axis2=2) - 1.0), -1.0, 1.0)
    theta = np.arctan2(sine, cosine)
    # General case: ω = θ · v / sin θ, with θ / sin θ from its series near zero.
    small = theta < EXP_TAYLOR_THRESHOLD
    safe_sine = np.where(sine > 0.0, sine, 1.0)
    factor = np.where(small, 1.0 + theta**2 / 6.0, theta / safe_sine)
    omega = v * factor[:, None]
    near_pi = (cosine < 0.0) & (sine < NEAR_PI_SINE)
    for i in np.flatnonzero(near_pi):
        omega[i] = theta[i] * _axis_near_pi(rotation[i], float(cosine[i]), v[i])
    return np.asarray(omega, dtype=np.float64)


def _axis_near_pi(
    rotation: NDArray[np.float64], cosine: float, v: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Find the rotation axis near θ = π, from ``(R + Rᵀ)/2 - cos θ · I = (1 - cos θ) n nᵀ``.

    The sign follows the skew part ``v = sin θ · n``; when that vanishes (θ = π exactly,
    where ``n`` and ``-n`` are the same rotation) the largest-magnitude component is made
    positive, so every backend returns the same vector.
    """
    outer = 0.5 * (rotation + rotation.T) - cosine * np.eye(3)
    k = int(np.argmax(np.diag(outer)))
    axis = outer[:, k] / np.sqrt(outer[k, k] * (1.0 - cosine))
    axis = axis / np.linalg.norm(axis)
    alignment = float(axis @ v)
    if alignment < 0.0 or (alignment == 0.0 and axis[int(np.argmax(np.abs(axis)))] < 0.0):
        axis = -axis
    return np.asarray(axis, dtype=np.float64)


def _jacobian(omega: NDArray[np.float64], sign: float) -> NDArray[np.float64]:
    """``I + sign · B W + C W²``: the left (sign +1) or right (sign -1) Jacobian."""
    theta = np.linalg.norm(omega, axis=-1)
    w = _hat(omega)
    b = _half_versine_coefficient(theta)[:, None, None]
    c = _third_coefficient(theta)[:, None, None]
    return np.eye(3) + sign * b * w + c * (w @ w)


def _jacobian_inverse(omega: NDArray[np.float64], sign: float) -> NDArray[np.float64]:
    """``I - sign · ½ W + D W²``: the left (sign +1) or right (sign -1) inverse Jacobian."""
    theta = np.linalg.norm(omega, axis=-1)
    w = _hat(omega)
    d = _inverse_coefficient(theta)[:, None, None]
    return np.eye(3) - sign * 0.5 * w + d * (w @ w)


def _right_jacobian(omega: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the right Jacobian on an (N, 3) batch (reference)."""
    return _jacobian(omega, -1.0)


def _left_jacobian(omega: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the left Jacobian on an (N, 3) batch (reference)."""
    return _jacobian(omega, 1.0)


def _right_jacobian_inverse(omega: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the inverse right Jacobian on an (N, 3) batch (reference)."""
    return _jacobian_inverse(omega, -1.0)


def _left_jacobian_inverse(omega: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the inverse left Jacobian on an (N, 3) batch (reference)."""
    return _jacobian_inverse(omega, 1.0)


def _from_quaternion(quaternion: NDArray[np.float64]) -> NDArray[np.float64]:
    """Convert an (N, 4) batch of near-unit xyzw quaternions to rotations (reference)."""
    q = quaternion / np.linalg.norm(quaternion, axis=-1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack(
        [
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
            np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
            np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
        ],
        axis=-2,
    )


def _to_quaternion(rotation: NDArray[np.float64]) -> NDArray[np.float64]:
    """Convert rotations to xyzw quaternions by Shepperd's method, w >= 0 (reference)."""
    out = np.empty((rotation.shape[0], 4))
    for i, r in enumerate(rotation):
        trace = r[0, 0] + r[1, 1] + r[2, 2]
        k = int(np.argmax((trace, r[0, 0], r[1, 1], r[2, 2])))
        if k == 0:
            s = 2.0 * np.sqrt(1.0 + trace)
            q = (r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], 0.25 * s * s)
        elif k == 1:
            s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            q = (0.25 * s * s, r[0, 1] + r[1, 0], r[0, 2] + r[2, 0], r[2, 1] - r[1, 2])
        elif k == 2:
            s = 2.0 * np.sqrt(1.0 - r[0, 0] + r[1, 1] - r[2, 2])
            q = (r[0, 1] + r[1, 0], 0.25 * s * s, r[1, 2] + r[2, 1], r[0, 2] - r[2, 0])
        else:
            s = 2.0 * np.sqrt(1.0 - r[0, 0] - r[1, 1] + r[2, 2])
            q = (r[0, 2] + r[2, 0], r[1, 2] + r[2, 1], 0.25 * s * s, r[1, 0] - r[0, 1])
        quat = np.asarray(q) / s
        out[i] = -quat if quat[3] < 0.0 else quat
    return out


def _project_to_so3(matrix: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute the nearest rotations (via an SVD) of an (N, 3, 3) batch (reference)."""
    u, _, vt = np.linalg.svd(matrix)
    d = np.ones((matrix.shape[0], 3))
    d[:, 2] = np.sign(np.linalg.det(u @ vt))
    return np.asarray((u * d[:, None, :]) @ vt, dtype=np.float64)


# --- public, batched API -------------------------------------------------------------------


def hat(omega: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map vectors to skew-symmetric matrices, so that ``hat(ω) @ u == cross(ω, u)``.

    Parameters
    ----------
    omega : array_like, shape (..., 3)
        Vectors in R^3, the tangent space of SO(3).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        The skew-symmetric matrices ``[ω]x``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `omega` is not a finite real array of shape (..., 3).

    Examples
    --------
    >>> hat([1.0, 2.0, 3.0])
    array([[ 0., -3.,  2.],
           [ 3.,  0., -1.],
           [-2.,  1.,  0.]])
    """
    flat, lead = as_batch(omega, (3,), "omega")
    return unbatch(backends.dispatch(backend, __name__, "hat", _hat, flat), lead)


def vee(skew: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map skew-symmetric matrices back to vectors; the inverse of :func:`hat`.

    Parameters
    ----------
    skew : array_like, shape (..., 3, 3)
        Skew-symmetric matrices.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3)
        The vectors ``ω`` with ``hat(ω) == skew``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `skew` is not finite, real, of shape (..., 3, 3), and skew-symmetric within
        :data:`SKEW_TOLERANCE`.
    """
    flat, lead = as_batch(skew, (3, 3), "skew")
    _check_skew(flat)
    return unbatch(backends.dispatch(backend, __name__, "vee", _vee, flat), lead)


def exp(omega: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map rotation vectors to rotation matrices (Rodrigues' formula).

    Parameters
    ----------
    omega : array_like, shape (..., 3)
        Rotation vectors: axis times angle in radians. Any angle is accepted; angles beyond
        π give the same rotation as their wrapped counterpart.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        The rotation matrices ``exp(hat(ω))``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `omega` is not a finite real array of shape (..., 3).
    """
    flat, lead = as_batch(omega, (3,), "omega")
    return unbatch(backends.dispatch(backend, __name__, "exp", _exp, flat), lead)


def log(rotation: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map rotation matrices to rotation vectors with angle in [0, π].

    At exactly θ = π the axis is ambiguous up to sign; the vector whose largest-magnitude
    component is positive is returned, the same in every backend.

    Parameters
    ----------
    rotation : array_like, shape (..., 3, 3)
        Rotation matrices: orthonormal within :data:`ROTATION_TOLERANCE`, determinant +1.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3)
        Rotation vectors ``ω`` with ``exp(ω) == rotation`` and ``|ω| <= π``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `rotation` is not a finite real (..., 3, 3) array of rotations. Repair a nearly
        orthonormal matrix explicitly with :func:`project_to_so3`.
    """
    flat, lead = as_batch(rotation, (3, 3), "rotation")
    check_rotation(flat)
    return unbatch(backends.dispatch(backend, __name__, "log", _log, flat), lead)


def _tangent_batch(
    omega: ArrayLike, limit: float | None = None
) -> tuple[NDArray[np.float64], tuple[int, ...]]:
    """Validate a batch of tangent vectors, optionally bounding their norm."""
    flat, lead = as_batch(omega, (3,), "omega")
    if limit is not None and np.any(np.linalg.norm(flat, axis=-1) >= limit):
        raise InvalidInputError("omega: inverse Jacobian undefined at |omega| >= 2*pi")
    return flat, lead


def right_jacobian(omega: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the right Jacobian ``Jr(ω)``, with ``exp(ω + δ) ≈ exp(ω) exp(Jr(ω) δ)``.

    Parameters
    ----------
    omega : array_like, shape (..., 3)
        Rotation vectors.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        ``Jr(ω) = I - (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²`` with ``W = hat(ω)``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `omega` is not a finite real array of shape (..., 3).
    """
    flat, lead = _tangent_batch(omega)
    result = backends.dispatch(backend, __name__, "right_jacobian", _right_jacobian, flat)
    return unbatch(result, lead)


def left_jacobian(omega: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the left Jacobian ``Jl(ω) = Jr(-ω)``, with ``exp(ω + δ) ≈ exp(Jl δ) exp(ω)``.

    Parameters
    ----------
    omega : array_like, shape (..., 3)
        Rotation vectors.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        ``Jl(ω) = I + (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `omega` is not a finite real array of shape (..., 3).
    """
    flat, lead = _tangent_batch(omega)
    result = backends.dispatch(backend, __name__, "left_jacobian", _left_jacobian, flat)
    return unbatch(result, lead)


def right_jacobian_inverse(omega: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the inverse of the right Jacobian, defined for ``|ω| < 2π``.

    Parameters
    ----------
    omega : array_like, shape (..., 3)
        Rotation vectors with norm below :data:`INVERSE_JACOBIAN_LIMIT`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        ``Jr⁻¹(ω) = I + ½ W + (1/θ² - cot(θ/2)/(2θ)) W²``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `omega` is not a finite real array of shape (..., 3), or ``|ω| >= 2π``.
    """
    flat, lead = _tangent_batch(omega, INVERSE_JACOBIAN_LIMIT)
    result = backends.dispatch(
        backend, __name__, "right_jacobian_inverse", _right_jacobian_inverse, flat
    )
    return unbatch(result, lead)


def left_jacobian_inverse(omega: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Compute the inverse of the left Jacobian, defined for ``|ω| < 2π``.

    Parameters
    ----------
    omega : array_like, shape (..., 3)
        Rotation vectors with norm below :data:`INVERSE_JACOBIAN_LIMIT`.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        ``Jl⁻¹(ω) = I - ½ W + (1/θ² - cot(θ/2)/(2θ)) W²``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `omega` is not a finite real array of shape (..., 3), or ``|ω| >= 2π``.
    """
    flat, lead = _tangent_batch(omega, INVERSE_JACOBIAN_LIMIT)
    result = backends.dispatch(
        backend, __name__, "left_jacobian_inverse", _left_jacobian_inverse, flat
    )
    return unbatch(result, lead)


def from_quaternion(quaternion: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map unit quaternions (Hamilton, xyzw) to rotation matrices.

    Parameters
    ----------
    quaternion : array_like, shape (..., 4)
        Quaternions ``(x, y, z, w)`` with norm within :data:`QUATERNION_TOLERANCE` of 1;
        they are normalised before use. ``q`` and ``-q`` give the same rotation.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        The rotation matrices.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `quaternion` is not a finite real array of shape (..., 4) of unit norm.
    """
    flat, lead = as_batch(quaternion, (4,), "quaternion")
    if np.any(np.abs(np.linalg.norm(flat, axis=-1) - 1.0) > QUATERNION_TOLERANCE):
        raise InvalidInputError(f"quaternion: norm not within {QUATERNION_TOLERANCE} of 1")
    result = backends.dispatch(backend, __name__, "from_quaternion", _from_quaternion, flat)
    return unbatch(result, lead)


def to_quaternion(rotation: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map rotation matrices to unit quaternions (Hamilton, xyzw), canonically ``w >= 0``.

    Parameters
    ----------
    rotation : array_like, shape (..., 3, 3)
        Rotation matrices (see :func:`log` for the accepted tolerance).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 4)
        Quaternions ``(x, y, z, w)`` with ``w >= 0``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `rotation` is not a finite real (..., 3, 3) array of rotations.
    """
    flat, lead = as_batch(rotation, (3, 3), "rotation")
    check_rotation(flat)
    result = backends.dispatch(backend, __name__, "to_quaternion", _to_quaternion, flat)
    return unbatch(result, lead)


def project_to_so3(matrix: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Map matrices to their nearest rotations in the Frobenius norm (via an SVD).

    This is the explicit repair for a nearly orthonormal matrix that :func:`log` rejects.

    Parameters
    ----------
    matrix : array_like, shape (..., 3, 3)
        Any real matrices.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (..., 3, 3)
        ``U diag(1, 1, det(U Vᵀ)) Vᵀ`` for ``matrix = U S Vᵀ``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `matrix` is not a finite real array of shape (..., 3, 3).
    """
    flat, lead = as_batch(matrix, (3, 3), "matrix")
    result = backends.dispatch(backend, __name__, "project_to_so3", _project_to_so3, flat)
    return unbatch(result, lead)
