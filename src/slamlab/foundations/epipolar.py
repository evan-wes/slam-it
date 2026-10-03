"""Epipolar geometry: the essential and fundamental matrices, by the 8-point algorithm.

Two views of a static scene satisfy the **epipolar constraint**. For calibrated rays (bearing
vectors ``f₁``, ``f₂``) and the relative pose ``T_c2_c1 = [R | t]``,

    f₂ᵀ E f₁ = 0,   E = [t]ₓ R          (the essential matrix: rank 2, singular values (σ, σ, 0))

and for pixels ``u₁``, ``u₂`` of uncalibrated cameras, ``u₂ᵀ F u₁ = 0`` with the fundamental
matrix ``F = K₂⁻ᵀ E K₁⁻¹`` (rank 2). Both are defined up to scale; here E is returned with
singular values ``(1, 1, 0)`` and F with unit Frobenius norm. Either may come back negated,
which is the same matrix.

**The 8-point algorithm** (Longuet-Higgins 1981; Hartley, "In defense of the eight-point
algorithm", TPAMI 1997). Each correspondence gives one equation linear in the nine entries,
``(x₂ ⊗ x₁) · vec(M) = 0``; eight or more give a system ``A vec(M) = 0`` whose least-squares
solution is the right singular vector of ``A`` with the smallest singular value. The result
does not have the right singular values, so it is projected: E onto the essential manifold
(``U diag(1, 1, 0) Vᵀ``, the closest essential matrix in Frobenius norm), F onto rank 2
(``U diag(σ₁, σ₂, 0) Vᵀ``).

Both are solved on **normalised** points (Hartley): each image's points are shifted to their
centroid and scaled to a mean distance of √2, so the columns of ``A`` are of similar size,
and the result is mapped back (``M = T₂ᵀ M̂ T₁``). F works on pixels; E on the bearings'
normalised image points ``x = f / f_z``. E is projected after mapping back, because the
normalisation does not preserve the essential structure; F's rank is enforced before, as
Hartley recommends. Without the normalisation the 8-point algorithm on bearings is badly
conditioned: on a 200-pair scene with 1 px noise, fitting E to every inlier gave 0.19 error
in E's entries against 0.023 with it, and RANSAC's least-squares refits stalled.

A system whose second-smallest singular value is below :data:`RANK_TOLERANCE` of its largest
has more than one solution (too few points, or a degenerate configuration such as a planar
scene for F); the solvers then raise :class:`~slamlab.foundations.errors.DegenerateError`.

**The Sampson error** is the residual RANSAC thresholds: the first-order approximation of the
squared distance a correspondence must move (in both images together) to satisfy the
constraint (Hartley and Zisserman, section 11.4.3),

    e² = (x₂ᵀ M x₁)² / ((M x₁)₀² + (M x₁)₁² + (Mᵀ x₂)₀² + (Mᵀ x₂)₁²).

For F it is in pixels²; for E it is in the **normalised image plane** (``x = f / f_z``), so a
pixel threshold ``θ`` becomes ``θ / focal`` there. The essential-matrix functions therefore
need bearings with positive z (pinhole views), like :func:`~slamlab.foundations.triangulation.
two_view`.

**RANSAC.** :func:`essential_ransac` and :func:`fundamental_ransac` run the generic LO-MSAC
loop (:mod:`slamlab.foundations.ransac`) with the 8-point algorithm as both the minimal solver
(8 correspondences) and the least-squares one. The 5-point solver, a smaller minimal sample,
arrives in its own change.

The C++ core (``slamlab/foundations/epipolar.hpp``) implements every function.
"""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import ransac
from slamlab.foundations.arrays import as_bearing_pairs, as_matrix3, as_point_pairs
from slamlab.foundations.correspondences import (
    RANK_TOLERANCE,
    Matrix3,
    PairEstimator,
    hartley_normalisation,
    homogeneous,
    normalised_plane,
    null_vector,
)
from slamlab.foundations.errors import DegenerateError
from slamlab.foundations.solve import Solution

SAMPLE_SIZE = 8
"""Correspondences in a minimal sample (and the fewest the 8-point algorithm accepts)."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.epipolar`` submodule provides.

    The RANSAC functions take :class:`slamlab.foundations.ransac.Options`' fields as one dict
    of scalars and return a dict: ``model`` (3, 3), ``inliers`` (N,) bool, and the summary's
    fields with ``termination`` as its index in ``codes()["termination"]``.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def essential_8pt(
        self, bearings1: NDArray[np.float64], bearings2: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Estimate E from (N, 3) bearing pairs, N >= 8."""
        ...

    def fundamental_8pt(
        self, pixels1: NDArray[np.float64], pixels2: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Estimate F from (N, 2) pixel pairs, N >= 8."""
        ...

    def sampson_essential(
        self,
        essential: NDArray[np.float64],
        bearings1: NDArray[np.float64],
        bearings2: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Return (N,) squared Sampson errors in the normalised plane."""
        ...

    def sampson_fundamental(
        self,
        fundamental: NDArray[np.float64],
        pixels1: NDArray[np.float64],
        pixels2: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Return (N,) squared Sampson errors in pixels."""
        ...

    def decompose_essential(self, essential: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return E's four candidate poses ``T_c2_c1``, shape (4, 4, 4)."""
        ...

    def essential_ransac(
        self,
        bearings1: NDArray[np.float64],
        bearings2: NDArray[np.float64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Run LO-MSAC with the 8-point E solver."""
        ...

    def fundamental_ransac(
        self,
        pixels1: NDArray[np.float64],
        pixels2: NDArray[np.float64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Run LO-MSAC with the normalised 8-point F solver."""
        ...


# --- the reference ------------------------------------------------------------------------


def _epipolar_system(x1: NDArray[np.float64], x2: NDArray[np.float64]) -> NDArray[np.float64]:
    """Stack ``x₂ ⊗ x₁`` per correspondence: the rows of ``A vec(M) = 0`` (vec row-major)."""
    return np.asarray((x2[:, :, None] * x1[:, None, :]).reshape(-1, 9), dtype=np.float64)


def _normalised_8pt(
    points1: NDArray[np.float64], points2: NDArray[np.float64]
) -> tuple[Matrix3, Matrix3, Matrix3] | None:
    """Solve the 8-point system on Hartley-normalised 2D points.

    Returns the solution in normalised coordinates with the two similarities ``T₁``, ``T₂``,
    or None if every point coincides or the solution is not unique.
    """
    t1, t2 = hartley_normalisation(points1), hartley_normalisation(points2)
    if t1 is None or t2 is None:
        return None
    x1 = homogeneous(points1) @ t1.T
    x2 = homogeneous(points2) @ t2.T
    solution = null_vector(_epipolar_system(x1, x2))
    if solution is None:
        return None
    return solution.reshape(3, 3), t1, t2


def _essential_8pt(
    bearings1: NDArray[np.float64], bearings2: NDArray[np.float64]
) -> Matrix3 | None:
    """Fit E on normalised image points, then project it onto the essential manifold."""
    solved = _normalised_8pt(normalised_plane(bearings1)[:, :2], normalised_plane(bearings2)[:, :2])
    if solved is None:
        return None
    raw, t1, t2 = solved
    u, _, vt = np.linalg.svd(t2.T @ raw @ t1)
    return np.asarray(u @ np.diag([1.0, 1.0, 0.0]) @ vt, dtype=np.float64)


def _fundamental_8pt(pixels1: NDArray[np.float64], pixels2: NDArray[np.float64]) -> Matrix3 | None:
    """Fit F by the normalised 8-point algorithm, rank 2, unit norm (reference)."""
    solved = _normalised_8pt(pixels1, pixels2)
    if solved is None:
        return None
    raw, t1, t2 = solved
    u, singular, vt = np.linalg.svd(raw)
    rank2 = u @ np.diag([singular[0], singular[1], 0.0]) @ vt
    fundamental = t2.T @ rank2 @ t1
    return np.asarray(fundamental / np.linalg.norm(fundamental), dtype=np.float64)


def sampson(
    matrix: Matrix3, x1: NDArray[np.float64], x2: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Return the squared Sampson error of homogeneous point pairs (z = 1) under E or F, unchecked.

    Parameters
    ----------
    matrix : ndarray of float64, shape (3, 3)
        E (on normalised points) or F (on pixels).
    x1, x2 : ndarray of float64, shape (N, 3)
        Homogeneous points with z = 1.

    Returns
    -------
    ndarray of float64, shape (N,)
        ``(x₂ᵀ M x₁)² / ((M x₁)₀² + (M x₁)₁² + (Mᵀ x₂)₀² + (Mᵀ x₂)₁²)``; infinity where the
        denominator vanishes (a point at an epipole).
    """
    line2 = x1 @ matrix.T  # M x₁, the epipolar line in image 2
    line1 = x2 @ matrix  # Mᵀ x₂, the epipolar line in image 1
    algebraic = np.sum(x2 * line2, axis=1)
    gradient = line2[:, 0] ** 2 + line2[:, 1] ** 2 + line1[:, 0] ** 2 + line1[:, 1] ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        error = np.where(gradient > 0.0, algebraic**2 / gradient, np.inf)
    return np.asarray(error, dtype=np.float64)


def essential_estimator(
    bearings1: NDArray[np.float64], bearings2: NDArray[np.float64]
) -> PairEstimator:
    """Return the 8-point E solver with its normalised-plane Sampson error.

    Parameters
    ----------
    bearings1, bearings2 : ndarray of float64, shape (N, 3)
        Validated bearing pairs with positive z.

    Returns
    -------
    slamlab.foundations.correspondences.PairEstimator
        The estimator.
    """
    x1, x2 = normalised_plane(bearings1), normalised_plane(bearings2)
    return PairEstimator(
        (bearings1, bearings2), _essential_8pt, lambda m: sampson(m, x1, x2), SAMPLE_SIZE
    )


def fundamental_estimator(
    pixels1: NDArray[np.float64], pixels2: NDArray[np.float64]
) -> PairEstimator:
    """Return the normalised 8-point F solver with its Sampson error in pixels.

    Parameters
    ----------
    pixels1, pixels2 : ndarray of float64, shape (N, 2)
        Validated pixel pairs.

    Returns
    -------
    PairEstimator
        The estimator.
    """
    x1, x2 = homogeneous(pixels1), homogeneous(pixels2)
    return PairEstimator(
        (pixels1, pixels2), _fundamental_8pt, lambda m: sampson(m, x1, x2), SAMPLE_SIZE
    )


# --- validation and dispatch --------------------------------------------------------------


def _unique(found: Matrix3 | None, what: str) -> Matrix3:
    """Return a solver's matrix, or raise as the C++ core does when there is none."""
    if found is None:
        raise DegenerateError(f"{what}: the correspondences do not determine a unique solution")
    return found


def _essential_reference(b1: NDArray[np.float64], b2: NDArray[np.float64]) -> Matrix3:
    """Run the 8-point E solver (reference); raise if the pairs do not determine E."""
    return _unique(_essential_8pt(b1, b2), "E")


def _fundamental_reference(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> Matrix3:
    """Run the normalised 8-point F solver (reference); raise if the pairs do not determine F."""
    return _unique(_fundamental_8pt(p1, p2), "F")


def _sampson_essential(
    matrix: Matrix3, b1: NDArray[np.float64], b2: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Return the Sampson errors of bearing pairs in the normalised plane (reference)."""
    return sampson(matrix, normalised_plane(b1), normalised_plane(b2))


def _sampson_fundamental(
    matrix: Matrix3, p1: NDArray[np.float64], p2: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Return the Sampson errors of pixel pairs (reference)."""
    return sampson(matrix, homogeneous(p1), homogeneous(p2))


def essential_8pt(
    bearings1: ArrayLike, bearings2: ArrayLike, *, backend: str = "python"
) -> Matrix3:
    """Estimate the essential matrix from eight or more bearing pairs.

    Parameters
    ----------
    bearings1, bearings2 : array_like, shape (N, 3)
        Unit bearings of each correspondence in camera 1 and camera 2, N >= 8, positive z.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (3, 3)
        E with ``f₂ᵀ E f₁ ≈ 0`` and singular values (1, 1, 0), up to sign.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the bearings are not matching (N, 3) unit vectors with positive z, or N < 8.
    slamlab.foundations.errors.DegenerateError
        If the correspondences do not determine E uniquely.
    """
    b1, b2 = as_bearing_pairs(bearings1, bearings2, SAMPLE_SIZE)
    return backends.dispatch(backend, __name__, "essential_8pt", _essential_reference, b1, b2)


def fundamental_8pt(pixels1: ArrayLike, pixels2: ArrayLike, *, backend: str = "python") -> Matrix3:
    """Estimate the fundamental matrix from eight or more pixel pairs (normalised 8-point).

    Parameters
    ----------
    pixels1, pixels2 : array_like, shape (N, 2)
        Pixels of each correspondence in image 1 and image 2, N >= 8.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (3, 3)
        F with ``u₂ᵀ F u₁ ≈ 0``, rank 2 and unit Frobenius norm, up to sign.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the pixels are not matching (N, 2) arrays, or N < 8.
    slamlab.foundations.errors.DegenerateError
        If the correspondences do not determine F uniquely.
    """
    p1, p2 = as_point_pairs(pixels1, pixels2, SAMPLE_SIZE)
    return backends.dispatch(backend, __name__, "fundamental_8pt", _fundamental_reference, p1, p2)


def sampson_essential(
    essential: ArrayLike, bearings1: ArrayLike, bearings2: ArrayLike, *, backend: str = "python"
) -> NDArray[np.float64]:
    """Return each bearing pair's squared Sampson error under E, in the normalised plane.

    Parameters
    ----------
    essential : array_like, shape (3, 3)
        The essential matrix.
    bearings1, bearings2 : array_like, shape (N, 3)
        Unit bearing pairs with positive z.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (N,)
        The squared errors (normalised-plane units; a pixel threshold is ``θ / focal``).

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the matrix is not (3, 3) or the bearings are invalid.
    """
    matrix = as_matrix3(essential, "essential")
    b1, b2 = as_bearing_pairs(bearings1, bearings2, 0)
    return backends.dispatch(
        backend, __name__, "sampson_essential", _sampson_essential, matrix, b1, b2
    )


def sampson_fundamental(
    fundamental: ArrayLike, pixels1: ArrayLike, pixels2: ArrayLike, *, backend: str = "python"
) -> NDArray[np.float64]:
    """Return each pixel pair's squared Sampson error under F.

    Parameters
    ----------
    fundamental : array_like, shape (3, 3)
        The fundamental matrix.
    pixels1, pixels2 : array_like, shape (N, 2)
        Pixel pairs.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (N,)
        The squared errors, in pixels².

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the matrix is not (3, 3) or the pixels are invalid.
    """
    matrix = as_matrix3(fundamental, "fundamental")
    p1, p2 = as_point_pairs(pixels1, pixels2, 0)
    return backends.dispatch(
        backend, __name__, "sampson_fundamental", _sampson_fundamental, matrix, p1, p2
    )


_W = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
"""The rotation by 90 degrees about z in E's decomposition, ``R = U W Vᵀ`` or ``U Wᵀ Vᵀ``."""


def decompose_reference(essential: Matrix3) -> NDArray[np.float64]:
    """Return E's four candidate poses, shape (4, 4, 4): the reference, unchecked."""
    u, _, vt = np.linalg.svd(essential)
    # E = U diag(1, 1, 0) Vᵀ holds with either sign of U or V, so make both proper rotations.
    u = u if np.linalg.det(u) > 0.0 else -u
    vt = vt if np.linalg.det(vt) > 0.0 else -vt
    t = u[:, 2]
    poses = np.tile(np.eye(4), (4, 1, 1))
    for k, (rotation, sign) in enumerate(
        [(u @ _W @ vt, 1.0), (u @ _W @ vt, -1.0), (u @ _W.T @ vt, 1.0), (u @ _W.T @ vt, -1.0)]
    ):
        poses[k, :3, :3] = rotation
        poses[k, :3, 3] = sign * t
    return poses


def decompose_essential(essential: ArrayLike, *, backend: str = "python") -> NDArray[np.float64]:
    """Return the four relative poses an essential matrix allows.

    ``E = U diag(1, 1, 0) Vᵀ`` (with U and V proper rotations) is ``[t]ₓ R`` for
    ``R ∈ {U W Vᵀ, U Wᵀ Vᵀ}`` and ``t = ±u₃`` (Hartley and Zisserman, result 9.19), where W is
    a quarter turn about z. Exactly one of the four puts the scene in front of both cameras;
    :func:`slamlab.foundations.twoview.initialize` chooses it by triangulating.

    Parameters
    ----------
    essential : array_like, shape (3, 3)
        E (any scale; it is projected onto the essential manifold by the SVD).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (4, 4, 4)
        The candidates ``T_c2_c1``, with unit translation: ``(R₁, t)``, ``(R₁, -t)``,
        ``(R₂, t)``, ``(R₂, -t)``. Which of ``R₁``, ``R₂`` is which, and the sign of ``t``,
        depend on the SVD, so backends may order them differently.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the matrix is not a finite (3, 3) array.
    slamlab.foundations.errors.DegenerateError
        If its rank is below 2 (no translation direction).
    """
    matrix = as_matrix3(essential, "essential")
    singular = np.linalg.svd(matrix, compute_uv=False)
    if not singular[1] > RANK_TOLERANCE * singular[0]:
        raise DegenerateError("essential: rank below 2, no translation direction")
    return backends.dispatch(backend, __name__, "decompose_essential", decompose_reference, matrix)


def essential_ransac(
    bearings1: ArrayLike, bearings2: ArrayLike, options: ransac.Options, *, backend: str = "python"
) -> Solution[ransac.Consensus[Matrix3], ransac.Summary]:
    """Estimate the essential matrix robustly: LO-MSAC with the 8-point solver.

    Parameters
    ----------
    bearings1, bearings2 : array_like, shape (N, 3)
        Unit bearing pairs with positive z, N >= 8.
    options : ransac.Options
        The settings; ``threshold`` is a Sampson distance in the normalised plane (a pixel
        threshold divided by the focal length).
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    slamlab.foundations.solve.Solution
        E and its inliers, and the run's :class:`ransac.Summary`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the bearings are invalid or fewer than 8.
    slamlab.foundations.errors.DegenerateError
        If no sample gave an essential matrix.
    """
    b1, b2 = as_bearing_pairs(bearings1, bearings2, SAMPLE_SIZE)
    estimator = essential_estimator(b1, b2)
    return ransac.run(
        estimator, (b1, b2), options, backend=backend, target=(__name__, "essential_ransac")
    )


def fundamental_ransac(
    pixels1: ArrayLike, pixels2: ArrayLike, options: ransac.Options, *, backend: str = "python"
) -> Solution[ransac.Consensus[Matrix3], ransac.Summary]:
    """Estimate the fundamental matrix robustly: LO-MSAC with the normalised 8-point solver.

    Parameters
    ----------
    pixels1, pixels2 : array_like, shape (N, 2)
        Pixel pairs, N >= 8.
    options : ransac.Options
        The settings; ``threshold`` is a Sampson distance in pixels.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    slamlab.foundations.solve.Solution
        F and its inliers, and the run's :class:`ransac.Summary`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the pixels are invalid or fewer than 8.
    slamlab.foundations.errors.DegenerateError
        If no sample gave a fundamental matrix.
    """
    p1, p2 = as_point_pairs(pixels1, pixels2, SAMPLE_SIZE)
    estimator = fundamental_estimator(p1, p2)
    return ransac.run(
        estimator, (p1, p2), options, backend=backend, target=(__name__, "fundamental_ransac")
    )
