"""Homographies: the plane-induced map between two views, by the normalised DLT.

When every point lies on one plane, or the camera only rotates, two views are related by a
**homography**: ``x₂ ∝ H x₁`` for homogeneous image points. For calibrated views (normalised
image points) and a plane ``nᵀ X = d`` in camera 1's frame,

    H = R + t nᵀ / d,

so H carries the relative pose and the plane, which :func:`decompose` recovers. A pure
rotation is the special case ``t = 0``. H is defined up to scale; here it is returned with unit
Frobenius norm, and may come back negated (the same homography).

**The DLT** (Hartley and Zisserman, algorithm 4.2). Each correspondence gives two equations
linear in the nine entries of H, from the first two rows of ``x₂ × H x₁ = 0``. Four or more
correspondences give ``A vec(H) = 0``, solved by the right singular vector of the smallest
singular value. Points are first **normalised** (Hartley: centroid to the origin, mean distance
√2 in each image) and the result mapped back. A system whose second-smallest singular value is
below :data:`slamlab.foundations.epipolar.RANK_TOLERANCE` of its largest (three collinear
points in a minimal sample) does not determine H.

**The Sampson error** is the first-order approximation of the squared distance the
correspondence must move, in both images together, to satisfy the two equations
(Hartley and Zisserman, section 4.2.6): with ``ε`` the two algebraic errors and ``J`` their
2x4 Jacobian with respect to ``(x₁, y₁, x₂, y₂)``,

    e² = εᵀ (J Jᵀ)⁻¹ ε.

It is in the input's units squared: pixels for pixels, the normalised plane for normalised
points. Because it measures the same thing as the essential matrix's Sampson error (a
distance in the joint image space), the two can be compared directly, which is what model
selection between them needs.

**RANSAC.** :func:`homography_ransac` runs the generic LO-MSAC loop with the DLT as both the
minimal (4 correspondences) and the least-squares solver.

The C++ core (``slamlab/foundations/homography.hpp``) implements every function.
"""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import ransac
from slamlab.foundations.epipolar import (
    Matrix3,
    PairEstimator,
    hartley_normalisation,
    homogeneous,
    null_vector,
    point_pairs,
    run_ransac,
)
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from slamlab.foundations.solve import Solution

SAMPLE_SIZE = 4
"""Correspondences in a minimal sample (and the fewest the DLT accepts)."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.homography`` submodule provides.

    ``homography_ransac`` takes :class:`slamlab.foundations.ransac.Options`' fields as one
    dict of scalars and returns a dict like the RANSAC functions of
    :class:`slamlab.foundations.epipolar.Implementation`.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def homography_dlt(
        self, points1: NDArray[np.float64], points2: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Estimate H from (N, 2) point pairs, N >= 4."""
        ...

    def sampson_homography(
        self,
        homography: NDArray[np.float64],
        points1: NDArray[np.float64],
        points2: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Return (N,) squared Sampson errors."""
        ...

    def homography_ransac(
        self,
        points1: NDArray[np.float64],
        points2: NDArray[np.float64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Run LO-MSAC with the normalised DLT."""
        ...


# --- the reference ------------------------------------------------------------------------


def _homography_dlt(points1: NDArray[np.float64], points2: NDArray[np.float64]) -> Matrix3 | None:
    """Fit H by the normalised DLT, unit norm (reference); None if undetermined."""
    t1, t2 = hartley_normalisation(points1), hartley_normalisation(points2)
    if t1 is None or t2 is None:
        return None
    x1 = homogeneous(points1) @ t1.T
    x2 = homogeneous(points2) @ t2.T
    n = x1.shape[0]
    system = np.zeros((2 * n, 9))
    # Rows of x₂ × H x₁ = 0 (with w₂ = 1): -(h₂ · x₁) + y₂ (h₃ · x₁), (h₁ · x₁) - x₂ (h₃ · x₁).
    system[0::2, 3:6] = -x1
    system[0::2, 6:9] = x2[:, 1:2] * x1
    system[1::2, 0:3] = x1
    system[1::2, 6:9] = -x2[:, 0:1] * x1
    solution = null_vector(system)
    if solution is None:
        return None
    homography = np.linalg.inv(t2) @ solution.reshape(3, 3) @ t1
    return np.asarray(homography / np.linalg.norm(homography), dtype=np.float64)


def sampson(
    homography: Matrix3, points1: NDArray[np.float64], points2: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Return the squared Sampson error of point pairs under H.

    Parameters
    ----------
    homography : ndarray of float64, shape (3, 3)
        H.
    points1, points2 : ndarray of float64, shape (N, 2)
        Point pairs.

    Returns
    -------
    ndarray of float64, shape (N,)
        ``εᵀ (J Jᵀ)⁻¹ ε``; infinity where ``J Jᵀ`` is singular.
    """
    h = homography
    mapped = homogeneous(points1) @ h.T  # (u, v, w) = H x₁
    u, v, w = mapped[:, 0], mapped[:, 1], mapped[:, 2]
    x2, y2 = points2[:, 0], points2[:, 1]
    eps1 = y2 * w - v
    eps2 = u - x2 * w
    # J = ∂(ε₁, ε₂)/∂(x₁, y₁, x₂, y₂).
    j1 = np.stack([y2 * h[2, 0] - h[1, 0], y2 * h[2, 1] - h[1, 1], np.zeros_like(w), w], axis=1)
    j2 = np.stack([h[0, 0] - x2 * h[2, 0], h[0, 1] - x2 * h[2, 1], -w, np.zeros_like(w)], axis=1)
    a = np.sum(j1 * j1, axis=1)
    b = np.sum(j1 * j2, axis=1)
    c = np.sum(j2 * j2, axis=1)
    determinant = a * c - b * b
    with np.errstate(divide="ignore", invalid="ignore"):
        error = (c * eps1**2 - 2.0 * b * eps1 * eps2 + a * eps2**2) / determinant
    return np.asarray(np.where(determinant > 0.0, error, np.inf), dtype=np.float64)


def homography_estimator(
    points1: NDArray[np.float64], points2: NDArray[np.float64]
) -> PairEstimator:
    """Return the normalised DLT with its Sampson error, as a :class:`ransac.Estimator`.

    Parameters
    ----------
    points1, points2 : ndarray of float64, shape (N, 2)
        Validated point pairs (pixels, or normalised image points).

    Returns
    -------
    slamlab.foundations.epipolar.PairEstimator
        The estimator.
    """
    return PairEstimator(
        (points1, points2), _homography_dlt, lambda m: sampson(m, points1, points2), SAMPLE_SIZE
    )


# --- validation and dispatch --------------------------------------------------------------


def _dlt_reference(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> Matrix3:
    """Run the normalised DLT; NaN if undetermined."""
    found = _homography_dlt(p1, p2)
    return np.full((3, 3), np.nan) if found is None else found


def homography_dlt(points1: ArrayLike, points2: ArrayLike, *, backend: str = "python") -> Matrix3:
    """Estimate the homography from four or more point pairs (normalised DLT).

    Parameters
    ----------
    points1, points2 : array_like, shape (N, 2)
        Point pairs (pixels, or normalised image points), N >= 4.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (3, 3)
        H with ``x₂ ∝ H x₁``, unit Frobenius norm, up to sign.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the points are not matching (N, 2) arrays, or N < 4.
    slamlab.foundations.errors.DegenerateError
        If the correspondences do not determine H (three collinear points in four).
    """
    p1, p2 = point_pairs(points1, points2, SAMPLE_SIZE, "points")
    found = backends.dispatch(backend, __name__, "homography_dlt", _dlt_reference, p1, p2)
    if np.all(np.isnan(found)):
        raise DegenerateError("H: the correspondences do not determine a unique solution")
    return found


def sampson_homography(
    homography: ArrayLike, points1: ArrayLike, points2: ArrayLike, *, backend: str = "python"
) -> NDArray[np.float64]:
    """Return each point pair's squared Sampson error under H.

    Parameters
    ----------
    homography : array_like, shape (3, 3)
        The homography.
    points1, points2 : array_like, shape (N, 2)
        Point pairs.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (N,)
        The squared errors, in the points' units squared.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the matrix is not (3, 3) or the points are invalid.
    """
    matrix = np.asarray(homography, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise InvalidInputError(f"homography: expected a finite (3, 3) matrix, got {matrix.shape}")
    p1, p2 = point_pairs(points1, points2, 0, "points")
    return backends.dispatch(
        backend, __name__, "sampson_homography", sampson, np.ascontiguousarray(matrix), p1, p2
    )


def homography_ransac(
    points1: ArrayLike, points2: ArrayLike, options: ransac.Options, *, backend: str = "python"
) -> Solution[ransac.Consensus[Matrix3], ransac.Summary]:
    """Estimate the homography robustly: LO-MSAC with the normalised DLT.

    Parameters
    ----------
    points1, points2 : array_like, shape (N, 2)
        Point pairs (pixels, or normalised image points), N >= 4.
    options : ransac.Options
        The settings; ``threshold`` is a Sampson distance in the points' units.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    slamlab.foundations.solve.Solution
        H and its inliers, and the run's :class:`ransac.Summary`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the points are invalid or fewer than 4.
    slamlab.foundations.errors.DegenerateError
        If no sample gave a homography.
    """
    p1, p2 = point_pairs(points1, points2, SAMPLE_SIZE, "points")
    estimator = homography_estimator(p1, p2)
    return run_ransac(
        estimator, (p1, p2), options, backend=backend, target=(__name__, "homography_ransac")
    )
