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
below :data:`slamlab.foundations.correspondences.RANK_TOLERANCE` of its largest (three collinear
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

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import ransac, so3
from slamlab.foundations.arrays import as_matrix3, as_point_pairs
from slamlab.foundations.correspondences import (
    RANK_TOLERANCE,
    Matrix3,
    PairEstimator,
    hartley_normalisation,
    homogeneous,
    null_vector,
)
from slamlab.foundations.errors import DegenerateError
from slamlab.foundations.solve import Solution

SAMPLE_SIZE = 4
"""Correspondences in a minimal sample (and the fewest the DLT accepts)."""

PURE_ROTATION_TOLERANCE = 1e-3
"""Below this largest entry of ``HᵀH - I`` (H scaled to a middle singular value of 1), H is a
rotation: the translation is zero and the plane is undetermined. The value is OpenCV's
(``HomographyDecompInria``)."""


@dataclass(frozen=True)
class PlanarMotions:
    """The motions a calibrated homography allows: ``H ∝ R + t nᵀ``.

    Attributes
    ----------
    poses : ndarray of float64, shape (K, 4, 4)
        Each candidate ``T_c2_c1``; its translation is ``t / d``, scaled by the plane's
        distance from camera 1.
    normals : ndarray of float64, shape (K, 3)
        Each candidate's unit plane normal ``n`` in camera 1's frame (zero for a pure
        rotation).
    """

    poses: NDArray[np.float64]
    normals: NDArray[np.float64]


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

    def decompose_homography(self, homography: NDArray[np.float64]) -> dict[str, Any]:
        """Return ``poses`` (K, 4, 4) and ``normals`` (K, 3), K = 4 or 1 (pure rotation)."""
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
    """Return the squared Sampson error of point pairs under H, unchecked.

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
    slamlab.foundations.correspondences.PairEstimator
        The estimator.
    """
    return PairEstimator(
        (points1, points2), _homography_dlt, lambda m: sampson(m, points1, points2), SAMPLE_SIZE
    )


# --- validation and dispatch --------------------------------------------------------------


def _dlt_reference(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> Matrix3:
    """Run the normalised DLT (reference); raise if the pairs do not determine H."""
    found = _homography_dlt(p1, p2)
    if found is None:
        raise DegenerateError("H: the correspondences do not determine a unique solution")
    return found


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
    p1, p2 = as_point_pairs(points1, points2, SAMPLE_SIZE, "points")
    return backends.dispatch(backend, __name__, "homography_dlt", _dlt_reference, p1, p2)


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
    matrix = as_matrix3(homography, "homography")
    p1, p2 = as_point_pairs(points1, points2, 0, "points")
    return backends.dispatch(backend, __name__, "sampson_homography", sampson, matrix, p1, p2)


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
    p1, p2 = as_point_pairs(points1, points2, SAMPLE_SIZE, "points")
    estimator = homography_estimator(p1, p2)
    return ransac.run(
        estimator, (p1, p2), options, backend=backend, target=(__name__, "homography_ransac")
    )


# --- decomposition (Malis and Vargas) -----------------------------------------------------


def _opposite_of_minor(m: Matrix3, row: int, col: int) -> float:
    """Return ``-M_ij``, the negated (row, col) minor of a symmetric 3x3 matrix."""
    rows = [r for r in range(3) if r != row]
    cols = [c for c in range(3) if c != col]
    a, b = rows
    c, d = cols
    return float(m[a, d] * m[b, c] - m[a, c] * m[b, d])


def _rotation_from(
    h: Matrix3, t_star: NDArray[np.float64], n: NDArray[np.float64], v: float
) -> Matrix3:
    """Return ``R = H (I - (2/v) t* nᵀ)``, made proper and projected onto SO(3).

    The closed form assumes an exact ``H = R + t nᵀ``; for an estimated (noisy) H the product
    is only near a rotation, so it is projected (the nearest rotation in Frobenius norm).
    """
    rotation = h @ (np.eye(3) - (2.0 / v) * np.outer(t_star, n))
    proper = rotation if np.linalg.det(rotation) > 0.0 else -rotation
    return so3.project_to_so3(proper)


_Minors = dict[tuple[int, int], float]
"""``-M_ij`` of S by index pair: the diagonal three and (0, 1), (0, 2), (1, 2)."""


def _minors(s: Matrix3) -> _Minors:
    """Return the negated minors of S that Malis and Vargas' formulas use."""
    pairs = ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))
    return {(i, j): _opposite_of_minor(s, i, j) for i, j in pairs}


def _plane_normals(
    s: Matrix3, minors: _Minors
) -> tuple[NDArray[np.float64], NDArray[np.float64], int]:
    """Return the two unit plane normals ``n'_a``, ``n'_b`` and the index of S's largest |s_kk|.

    Malis and Vargas (2007) give three equivalent formulas; the one built on the row of S with
    the largest diagonal entry is used, for numerical stability.
    """
    root = {k: np.sqrt(max(minors[(k, k)], 0.0)) for k in range(3)}
    sign = {k: 1.0 if minors[k] >= 0.0 else -1.0 for k in ((0, 1), (0, 2), (1, 2))}
    k = int(np.argmax(np.abs(np.diag(s))))
    if k == 0:
        na = np.array([s[0, 0], s[0, 1] + root[2], s[0, 2] + sign[(1, 2)] * root[1]])
        nb = np.array([s[0, 0], s[0, 1] - root[2], s[0, 2] - sign[(1, 2)] * root[1]])
    elif k == 1:
        na = np.array([s[0, 1] + root[2], s[1, 1], s[1, 2] - sign[(0, 2)] * root[0]])
        nb = np.array([s[0, 1] - root[2], s[1, 1], s[1, 2] + sign[(0, 2)] * root[0]])
    else:
        na = np.array([s[0, 2] + sign[(0, 1)] * root[1], s[1, 2] + root[0], s[2, 2]])
        nb = np.array([s[0, 2] - sign[(0, 1)] * root[1], s[1, 2] - root[0], s[2, 2]])
    return na / np.linalg.norm(na), nb / np.linalg.norm(nb), k


def _motion_scalars(s: Matrix3, minors: _Minors) -> tuple[float, float, float]:
    """Return Malis and Vargas' scalars ``ν``, ``ρ`` and ``‖t_e‖``.

    ``ν = 2 √(1 + tr S - M₁₁ - M₂₂ - M₃₃)``, ``ρ² = 2 + tr S + ν``, ``‖t_e‖² = 2 + tr S - ν``;
    each radicand is clamped at zero, since rounding takes a zero just below it.
    """
    trace = float(np.trace(s))
    nu = 2.0 * np.sqrt(max(1.0 + trace - minors[(0, 0)] - minors[(1, 1)] - minors[(2, 2)], 0.0))
    rho = float(np.sqrt(max(2.0 + trace + nu, 0.0)))
    t_norm = float(np.sqrt(max(2.0 + trace - nu, 0.0)))
    return float(nu), rho, t_norm


def decompose_reference(homography: Matrix3) -> dict[str, NDArray[np.float64]]:
    """Decompose a calibrated H by Malis and Vargas' closed form (the reference).

    Unchecked apart from its rank: a matrix of rank below 2 raises DegenerateError, as the C++
    core does. The names follow the paper: ``S = HᵀH - I``, the plane normals ``n'_a``,
    ``n'_b``, ``ε = sign(s_kk)``, and ``t*_a``, ``t*_b`` the translations before rotation.
    """
    singular = np.linalg.svd(homography, compute_uv=False)
    if not singular[1] > RANK_TOLERANCE * singular[0]:
        raise DegenerateError("homography: rank below 2")
    h = homography / singular[1]
    s = h.T @ h - np.eye(3)
    if np.abs(s).max() < PURE_ROTATION_TOLERANCE:
        # An estimated H is only near a rotation: project it, as for the four candidates.
        pose = np.eye(4)
        pose[:3, :3] = so3.project_to_so3(h if np.linalg.det(h) > 0.0 else -h)
        return {"poses": pose[None], "normals": np.zeros((1, 3))}
    minors = _minors(s)
    na, nb, k = _plane_normals(s, minors)
    nu, rho, t_norm = _motion_scalars(s, minors)
    epsilon = 1.0 if s[k, k] >= 0.0 else -1.0
    ta_star = 0.5 * t_norm * (epsilon * rho * nb - t_norm * na)
    tb_star = 0.5 * t_norm * (epsilon * rho * na - t_norm * nb)
    poses = np.tile(np.eye(4), (4, 1, 1))
    normals = np.empty((4, 3))
    for j, (t_star, n) in enumerate(((ta_star, na), (tb_star, nb))):
        rotation = _rotation_from(h, t_star, n, nu)
        t = rotation @ t_star
        for sgn, slot in ((1.0, 2 * j), (-1.0, 2 * j + 1)):
            poses[slot, :3, :3] = rotation
            poses[slot, :3, 3] = sgn * t
            normals[slot] = sgn * n
    return {"poses": poses, "normals": normals}


def decompose_homography(homography: ArrayLike, *, backend: str = "python") -> PlanarMotions:
    """Return the motions and planes a calibrated homography allows (Malis and Vargas).

    For ``H ∝ R + t nᵀ`` on normalised image points, Malis and Vargas ("Deeper understanding of
    the homography decomposition for vision-based control", INRIA RR-6303, 2007) give the
    candidates in closed form from ``S = HᵀH - I`` (with H scaled to a middle singular value
    of 1), without an SVD of H's structure. There are four: two pairs ``(R, t, n)`` and
    ``(R, -t, -n)``. Two put the plane behind the cameras; the remaining two are told apart by
    triangulating the inliers (:func:`slamlab.foundations.twoview.initialize`). A pure
    rotation (``S ≈ 0``) has one, with zero translation and no plane.

    Parameters
    ----------
    homography : array_like, shape (3, 3)
        A calibrated H (on normalised image points), any scale and sign.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    PlanarMotions
        The candidates' poses ``T_c2_c1`` (translation over plane distance) and normals.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the matrix is not a finite (3, 3) array.
    slamlab.foundations.errors.DegenerateError
        If H is singular (rank below 2).
    """
    matrix = as_matrix3(homography, "homography")
    backends.check_name(backend)
    raw = (
        decompose_reference(matrix)
        if backend == "python"
        else backends.function(backend, __name__, "decompose_homography")(matrix)
    )
    return PlanarMotions(
        poses=np.asarray(raw["poses"], dtype=np.float64),
        normals=np.asarray(raw["normals"], dtype=np.float64),
    )
