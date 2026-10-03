"""P3P: the camera pose from three world points and their bearings.

A calibrated camera sees world point ``Xᵢ`` along the unit bearing ``yᵢ``, at an unknown depth
``λᵢ > 0``: ``λᵢ yᵢ = R Xᵢ + t`` for the pose ``T_cam_world = [R | t]``. Three points
determine the pose up to a finite set, at most four real solutions, so P3P is the minimal
solver of absolute-pose RANSAC (:mod:`slamlab.foundations.pnp`).

**The depths.** A rigid motion keeps distances, so each pair of points gives one equation in
the depths alone (the law of cosines, with ``bᵢⱼ = yᵢ · yⱼ`` and ``aᵢⱼ = |Xᵢ - Xⱼ|²``):

    λᵢ² - 2 bᵢⱼ λᵢ λⱼ + λⱼ² = aᵢⱼ        (ij = 12, 13, 23).

Dividing one equation by another removes the scale, leaving two **conics**: quadratic curves
in two unknowns (depth ratios, or the depths up to scale). The solutions are where the conics
meet, at most four points. Both solvers here find them the same way, which is what makes
them fast and stable compared with the classic route through a quartic polynomial (Grunert,
1841; Kneip et al., CVPR 2011):

1. **A degenerate conic in the pencil.** Every combination ``C₁ + s C₂`` of the two conics
   passes through their intersections. For three values of ``s``, the roots of the cubic
   ``det(C₁ + s C₂) = 0``, the combination is degenerate: a pair of lines. One real root
   suffices, found in closed form.
2. **Split it into its two lines**, and intersect each line with one of the conics: a
   quadratic in one depth ratio. Its positive roots give the depths, up to the scale that one
   distance equation fixes.
3. **Polish the depths** by Gauss-Newton on the three distance equations (to convergence,
   :data:`POLISH_MAX_STEPS`, :data:`POLISH_TOLERANCE`), as both papers do with a few steps.
4. **The pose** follows from the two triangles, ``λᵢ yᵢ`` in the camera and ``Xᵢ`` in the
   world: with ``Y = [v₁, v₂, v₁ × v₂]`` for ``v₁ = λ₁y₁ - λ₂y₂``, ``v₂ = λ₁y₁ - λ₃y₃``, and
   ``X`` the same from the world points, ``R = Y X⁻¹`` and ``t = λ₁y₁ - R X₁``.

**Lambda Twist** (Persson and Nordberg, "Lambda Twist: An Accurate Fast Robust Perspective
Three Point (P3P) Solver", ECCV 2018) works on the depths themselves. Its conics are the
homogeneous quadratic forms ``D₁ = a₂₃ M₁₂ - a₁₂ M₂₃`` and ``D₂ = a₂₃ M₁₃ - a₁₃ M₂₃``
(``Mᵢⱼ`` the left-hand side above as a matrix), and it splits ``D₀ = D₁ + g D₂`` with its
eigen-decomposition: ``D₀`` has a zero eigenvalue, so ``Λᵀ D₀ Λ = σ₁ (e₁·Λ)² + σ₂ (e₂·Λ)²``,
which vanishes on the lines ``(e₁ ± s e₂)·Λ = 0``, ``s = √(-σ₂/σ₁)``. It takes the cubic's
largest real root.

**Ding et al.** (Ding, Yang, Larsson, Olsson and Åström, "Revisiting the P3P Problem", CVPR
2023) works on the depth ratios ``x = λ₁/λ₃`` and ``y = λ₂/λ₃``, with the points reordered so
the reference distance ``a₁₂`` between the second and third is the longest, which keeps the
conics' coefficients bounded. It splits the degenerate conic without an eigen-decomposition:
for a line pair ``C = p qᵀ + q pᵀ``, ``-adj(C) = w wᵀ`` with ``w = p × q``, and ``C + [w]ₓ`` is
the rank-one ``2 q pᵀ``, whose column and row are the lines. If the cubic has only one real
root, the real intersections both lie on one line (the other holds a complex-conjugate
pair), so the second line is skipped once the first gave a solution. It is the default:
it is the more stable of the two on near-degenerate samples (``docs/notes/absolute-pose.md``).

Both follow PoseLib's implementations (Larsson et al., BSD-3), restated here in slamlab's
terms. Two departures, the same in both: the cubic's root and the depths are polished to
convergence rather than for a fixed number of steps, and the line pair is split on its
largest entry (PoseLib's ``MAX_ABS`` strategy) rather than its first row and column.

A sample whose world points are collinear has no unique pose and gives no solution.

The C++ core (``slamlab/foundations/p3p.hpp``) implements both.
"""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import so3
from slamlab.foundations.arrays import as_correspondences
from slamlab.foundations.correspondences import Matrix3
from slamlab.foundations.errors import InvalidInputError

SAMPLE_SIZE = 3
"""Correspondences a P3P solver takes."""

SOLVERS = ("ding", "lambda_twist")
"""The P3P solvers, in code order."""

POLISH_MAX_STEPS = 10
"""At most this many Newton steps polish the cubic's root, and the depths."""
POLISH_TOLERANCE = 1e-14
"""Polishing stops when a step is below this, relative to the value polished (at least 1 for
the cubic's root)."""

REAL_TOLERANCE = 1e-12
"""A negative discriminant above this times ``-(b² + |4ac|)`` counts as zero (a double root), so
rounding cannot turn a tangent line's one intersection into none. A positive one, however
small, gives two roots: near the danger cylinder two true poses can be 1e-6 apart."""

COLLINEAR_TOLERANCE = 1e-10
"""The world points are collinear when ``|X₁₂ × X₁₃|`` is below this times ``|X₁₂| |X₁₃|``."""

Vector3 = NDArray[np.float64]
Pose = NDArray[np.float64]


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.p3p`` submodule provides."""

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order: ``{"solver": [...]}``."""
        ...

    def p3p(
        self, points: NDArray[np.float64], bearings: NDArray[np.float64], solver: int
    ) -> NDArray[np.float64]:
        """Return every pose ``T_cam_world`` three correspondences allow, shape (K, 4, 4)."""
        ...


# --- shared pieces ------------------------------------------------------------------------


def _polish_cubic(root: float, c2: float, c1: float, c0: float) -> float:
    """Polish a root of the monic cubic ``z³ + c₂z² + c₁z + c₀`` by Newton's method.

    A step is kept only if it lowers ``|f|``; polishing stops when a step is below
    :data:`POLISH_TOLERANCE` relative to the root (at least 1).
    """
    value = ((root + c2) * root + c1) * root + c0
    for _ in range(POLISH_MAX_STEPS):
        slope = (3.0 * root + 2.0 * c2) * root + c1
        if slope == 0.0:
            break
        step = value / slope
        moved = root - step
        moved_value = ((moved + c2) * moved + c1) * moved + c0
        if not abs(moved_value) < abs(value):
            break
        root, value = moved, moved_value
        if abs(step) <= POLISH_TOLERANCE * max(1.0, abs(root)):
            break
    return root


def cubic_root(c2: float, c1: float, c0: float) -> tuple[float, bool]:
    """Return the largest real root of ``z³ + c₂z² + c₁z + c₀``, and whether it is the only one.

    In closed form: with ``z = u - c₂/3`` the cubic is ``u³ + p u + q``; when
    ``Δ = q²/4 + p³/27 > 0`` it has one real root (Cardano), otherwise three, the largest
    ``2 √(-p/3) cos(φ/3)`` (Viète). The root is then polished by Newton's method.

    Parameters
    ----------
    c2, c1, c0 : float
        The monic cubic's coefficients.

    Returns
    -------
    tuple of (float, bool)
        The root, and True when it is the cubic's only real root.
    """
    p = c1 - c2 * c2 / 3.0
    q = (2.0 * c2 * c2 * c2 - 9.0 * c2 * c1) / 27.0 + c0
    discriminant = q * q / 4.0 + p * p * p / 27.0
    if discriminant >= 0.0:
        root = math.sqrt(discriminant)
        u = float(np.cbrt(-0.5 * q + root) + np.cbrt(-0.5 * q - root))
        single = discriminant > 0.0
    else:
        cosine = min(1.0, max(-1.0, 1.5 * q / p * math.sqrt(-3.0 / p)))
        u = 2.0 * math.sqrt(-p / 3.0) * math.cos(math.acos(cosine) / 3.0)
        single = False
    return _polish_cubic(u - c2 / 3.0, c2, c1, c0), single


def pencil_cubic(first: Matrix3, second: Matrix3) -> tuple[float, float, float, float]:
    """Return ``det(first + s second)`` as a cubic in ``s``, from the highest power down.

    For symmetric 3x3 matrices, ``det(A + sB) = det A + s ⟨B, cof A⟩ + s² ⟨A, cof B⟩ +
    s³ det B``, with ``cof`` the cofactor matrix and ``⟨·,·⟩`` the entrywise inner product.

    Parameters
    ----------
    first, second : ndarray of float64, shape (3, 3)
        The two conics, symmetric.

    Returns
    -------
    tuple of float
        ``(det B, ⟨A, cof B⟩, ⟨B, cof A⟩, det A)``.
    """

    def cofactor(matrix: Matrix3) -> Matrix3:
        columns = (matrix[:, 1], matrix[:, 2], matrix[:, 0])
        return np.column_stack([np.cross(columns[k], columns[(k + 1) % 3]) for k in range(3)])

    cof_first, cof_second = cofactor(first), cofactor(second)
    return (
        float(second[:, 0] @ cof_second[:, 0]),
        float(np.sum(first * cof_second)),
        float(np.sum(second * cof_first)),
        float(first[:, 0] @ cof_first[:, 0]),
    )


def quadratic_roots(a: float, b: float, c: float) -> list[float]:
    """Return the real roots of ``a τ² + b τ + c``, computed without cancellation.

    A discriminant slightly below zero (within :data:`REAL_TOLERANCE`) is a double root; a
    positive one gives two roots, however close.

    Parameters
    ----------
    a, b, c : float
        The coefficients.

    Returns
    -------
    list of float
        Zero, one (a double root, or a linear equation) or two roots.
    """
    discriminant = b * b - 4.0 * a * c
    if discriminant < -REAL_TOLERANCE * (b * b + abs(4.0 * a * c)):
        return []
    if a == 0.0:
        return [-c / b] if b != 0.0 else []
    if discriminant < 0.0:
        return [-0.5 * b / a]
    half = -0.5 * (b + math.copysign(math.sqrt(discriminant), b))
    return [half / a, c / half] if half != 0.0 else [half / a]


def _polish_depths(depths: Vector3, cosines: Vector3, squared: Vector3) -> Vector3:
    """Polish the depths by Newton's method on the three distance equations.

    ``rᵢⱼ = λᵢ² - 2 bᵢⱼ λᵢ λⱼ + λⱼ² - aᵢⱼ`` for ``ij = 12, 13, 23``, with `cosines` the ``bᵢⱼ``
    and `squared` the ``aᵢⱼ`` in that order. A step is kept only if it lowers ``|r|``.
    """
    pairs = ((0, 1), (0, 2), (1, 2))

    def residuals(lam: Vector3) -> Vector3:
        return np.array(
            [
                lam[i] ** 2 - 2.0 * cosines[k] * lam[i] * lam[j] + lam[j] ** 2 - squared[k]
                for k, (i, j) in enumerate(pairs)
            ]
        )

    current = residuals(depths)
    for _ in range(POLISH_MAX_STEPS):
        jacobian = np.zeros((3, 3))
        for k, (i, j) in enumerate(pairs):
            jacobian[k, i] = 2.0 * (depths[i] - cosines[k] * depths[j])
            jacobian[k, j] = 2.0 * (depths[j] - cosines[k] * depths[i])
        try:
            step = np.linalg.solve(jacobian, current)
        except np.linalg.LinAlgError:
            break
        moved = depths - step
        moved_residuals = residuals(moved)
        if not np.linalg.norm(moved_residuals) < np.linalg.norm(current):
            break
        depths, current = moved, moved_residuals
        if np.linalg.norm(step) <= POLISH_TOLERANCE * np.linalg.norm(depths):
            break
    return depths


def _pose(depths: Vector3, bearings: NDArray[np.float64], points: NDArray[np.float64]) -> Pose:
    """Return ``T_cam_world`` from the depths: ``R = Y X⁻¹``, ``t = λ₁y₁ - R X₁``."""
    rays = depths[:, None] * bearings
    v1, v2 = rays[0] - rays[1], rays[0] - rays[2]
    d1, d2 = points[0] - points[1], points[0] - points[2]
    camera = np.column_stack([v1, v2, np.cross(v1, v2)])
    world = np.column_stack([d1, d2, np.cross(d1, d2)])
    rotation = camera @ np.linalg.inv(world)
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = rays[0] - rotation @ points[0]
    return pose


def _collinear(points: NDArray[np.float64]) -> bool:
    """Whether three world points are (nearly) collinear: no unique pose."""
    d1, d2 = points[0] - points[1], points[0] - points[2]
    area = float(np.linalg.norm(np.cross(d1, d2)))
    return not area > COLLINEAR_TOLERANCE * float(np.linalg.norm(d1) * np.linalg.norm(d2))


def _finish(
    depths: Vector3,
    bearings: NDArray[np.float64],
    points: NDArray[np.float64],
    geometry: tuple[Vector3, Vector3],
) -> Pose | None:
    """Polish candidate depths and build the pose; None unless all three stay positive."""
    cosines, squared = geometry
    polished = _polish_depths(depths, cosines, squared)
    if not np.all(polished > 0.0):
        return None
    pose = _pose(polished, bearings, points)
    return pose if np.all(np.isfinite(pose)) else None


# --- Lambda Twist -------------------------------------------------------------------------


def _eigen_known_zero(matrix: Matrix3) -> tuple[Matrix3, float, float] | None:
    """Eigen-decompose a symmetric 3x3 matrix with a zero eigenvalue, in closed form.

    The other two eigenvalues are the roots of ``σ² + p₁σ + p₀`` (``p₁ = -tr``, ``p₀`` the sum
    of principal 2x2 minors), ordered ``|σ₁| ≥ |σ₂|``; each eigenvector solves the first two
    rows of ``(M - σI) e = 0`` with its last entry 1, then is normalised (Persson and Nordberg,
    with the sign of ``M₀₀`` in ``p₀`` corrected, as PoseLib notes).

    Returns
    -------
    tuple or None
        The eigenvectors as columns (the third unused, left zero), ``σ₁`` and ``σ₂``; None if
        the eigenvalues are not real or an eigenvector's last entry vanishes.
    """
    m = matrix
    p1 = -np.trace(m)
    p0 = (
        -m[0, 1] ** 2 - m[0, 2] ** 2 - m[1, 2] ** 2
        + m[0, 0] * (m[1, 1] + m[2, 2]) + m[1, 1] * m[2, 2]
    )  # fmt: skip
    inside = p1 * p1 / 4.0 - p0
    if not inside >= 0.0:
        return None
    half = math.sqrt(inside)
    sigma1, sigma2 = -p1 / 2.0 + half, -p1 / 2.0 - half
    if abs(sigma1) < abs(sigma2):
        sigma1, sigma2 = sigma2, sigma1
    vectors = np.zeros((3, 3))
    for column, sigma in enumerate((sigma1, sigma2)):
        c = sigma * sigma + m[0, 0] * m[1, 1] - sigma * (m[0, 0] + m[1, 1]) - m[0, 1] ** 2
        if c == 0.0:
            return None
        a1 = (sigma * m[0, 2] + m[0, 1] * m[1, 2] - m[0, 2] * m[1, 1]) / c
        a2 = (sigma * m[1, 2] + m[0, 1] * m[0, 2] - m[0, 0] * m[1, 2]) / c
        vectors[:, column] = np.array([a1, a2, 1.0]) / math.sqrt(1.0 + a1 * a1 + a2 * a2)
    return vectors, sigma1, sigma2


def _lambda_twist(bearings: NDArray[np.float64], points: NDArray[np.float64]) -> list[Pose]:
    """Solve P3P by Lambda Twist (reference)."""
    a12, a13, a23 = (
        float(np.sum((points[i] - points[j]) ** 2)) for i, j in ((0, 1), (0, 2), (1, 2))
    )
    b12, b13, b23 = (float(bearings[i] @ bearings[j]) for i, j in ((0, 1), (0, 2), (1, 2)))
    d1 = np.array(
        [[a23, -a23 * b12, 0.0], [-a23 * b12, a23 - a12, a12 * b23], [0.0, a12 * b23, -a12]]
    )
    d2 = np.array(
        [[a23, 0.0, -a23 * b13], [0.0, -a13, a13 * b23], [-a23 * b13, a13 * b23, a23 - a13]]
    )
    c3, c2, c1, c0 = pencil_cubic(d1, d2)
    if c3 == 0.0:
        return []
    gamma, _ = cubic_root(c2 / c3, c1 / c3, c0 / c3)
    decomposed = _eigen_known_zero(d1 + gamma * d2)
    if decomposed is None or not -decomposed[2] / decomposed[1] >= 0.0:
        return []
    vectors, sigma1, sigma2 = decomposed
    s = math.sqrt(-sigma2 / sigma1)
    geometry = (np.array([b12, b13, b23]), np.array([a12, a13, a23]))
    poses: list[Pose] = []
    for sign in (1.0, -1.0):
        for depths in _twist_depths(vectors[:, 0] - sign * s * vectors[:, 1], geometry):
            if (pose := _finish(depths, bearings, points, geometry)) is not None:
                poses.append(pose)
    return poses


def _twist_depths(u: Vector3, geometry: tuple[Vector3, Vector3]) -> list[Vector3]:
    """Intersect one of Lambda Twist's lines, ``u · Λ = 0``, with the distance conics.

    The depth with the larger coefficient in ``u`` is eliminated (``λ₂`` if ``|u₁| < |u₂|``,
    else ``λ₁``), leaving a quadratic in the ratio ``τ`` of the other two; one distance
    equation then fixes the scale.

    Returns
    -------
    list of ndarray of float64, shape (3,)
        The candidate depths, every one positive.
    """
    (b12, b13, b23), (a12, a13, a23) = geometry
    candidates: list[Vector3] = []
    if abs(u[0]) < abs(u[1]):
        # λ₂ = w₀ λ₁ + w₁ λ₃ and τ = λ₃ / λ₁, from a₁₃ (λ₂² - 2b₂₃λ₂λ₃ + λ₃²) = a₂₃ (λ₁² - …).
        w0, w1 = -u[0] / u[1], -u[2] / u[1]
        taus = quadratic_roots(
            a13 * w1 * w1 - 2.0 * a13 * b23 * w1 + a13 - a23,
            2.0 * a13 * w0 * w1 - 2.0 * a13 * b23 * w0 + 2.0 * a23 * b13,
            a13 * w0 * w0 - a23,
        )
        for tau in (t for t in taus if t > 0.0):
            lam1 = math.sqrt(a13 / (tau * (tau - 2.0 * b13) + 1.0))
            lam3 = tau * lam1
            candidates.append(np.array([lam1, w0 * lam1 + w1 * lam3, lam3]))
    else:
        # λ₁ = w₀ λ₂ + w₁ λ₃ and τ = λ₃ / λ₂, from a₁₃ (λ₁² - 2b₁₂λ₁λ₂ + λ₂²) = a₁₂ (λ₁² - …).
        w0, w1 = -u[1] / u[0], -u[2] / u[0]
        taus = quadratic_roots(
            (a13 - a12) * w1 * w1 + 2.0 * a12 * b13 * w1 - a12,
            2.0 * (a13 - a12) * w0 * w1 - 2.0 * a13 * b12 * w1 + 2.0 * a12 * b13 * w0,
            (a13 - a12) * w0 * w0 - 2.0 * a13 * b12 * w0 + a13,
        )
        for tau in (t for t in taus if t > 0.0):
            lam2 = math.sqrt(a23 / (tau * (tau - 2.0 * b23) + 1.0))
            lam3 = tau * lam2
            candidates.append(np.array([w0 * lam2 + w1 * lam3, lam2, lam3]))
    return [d for d in candidates if np.all(d > 0.0)]


# --- Ding et al. --------------------------------------------------------------------------


def _line_pair(conic: Matrix3) -> tuple[Vector3, Vector3] | None:
    """Split a degenerate conic ``C = p qᵀ + q pᵀ`` into its lines ``p`` and ``q``.

    ``-adj(C) = w wᵀ`` with ``w = p × q``: its column with the largest diagonal entry, over that
    entry's square root, is ``±w``. Then ``C + [w]ₓ`` has rank one; the column and row through
    its largest entry are the lines. None if ``-adj(C)`` has no positive diagonal entry (the
    lines are complex).
    """
    c = conic
    adjugate = -np.array(
        [
            [c[1, 1] * c[2, 2] - c[1, 2] ** 2, c[0, 2] * c[1, 2] - c[0, 1] * c[2, 2],
             c[0, 1] * c[1, 2] - c[0, 2] * c[1, 1]],
            [c[0, 2] * c[1, 2] - c[0, 1] * c[2, 2], c[0, 0] * c[2, 2] - c[0, 2] ** 2,
             c[0, 1] * c[0, 2] - c[0, 0] * c[1, 2]],
            [c[0, 1] * c[1, 2] - c[0, 2] * c[1, 1], c[0, 1] * c[0, 2] - c[0, 0] * c[1, 2],
             c[0, 0] * c[1, 1] - c[0, 1] ** 2],
        ]
    )  # fmt: skip
    k = int(np.argmax(np.diag(adjugate)))
    if not adjugate[k, k] > 0.0:
        return None
    w = adjugate[:, k] / math.sqrt(adjugate[k, k])
    rank_one = c + so3.hat(w)
    row, column = np.unravel_index(int(np.argmax(np.abs(rank_one))), (3, 3))
    return rank_one[:, column], rank_one[row, :]


def _ding(bearings: NDArray[np.float64], points: NDArray[np.float64]) -> list[Pose]:
    """Solve P3P by Ding et al.'s method (reference)."""
    order = [0, 1, 2]
    squared = [float(np.sum((points[i] - points[j]) ** 2)) for i, j in ((0, 1), (0, 2), (1, 2))]
    # Reorder so the distance between the second and third points, a₁₂, is the longest.
    if squared[0] > squared[1]:
        if squared[0] > squared[2]:
            order = [2, 1, 0]
    elif squared[1] > squared[2]:
        order = [1, 0, 2]
    x, world = bearings[order], points[order]
    a01, a02, a12 = (float(np.sum((world[i] - world[j]) ** 2)) for i, j in ((0, 1), (0, 2), (1, 2)))
    m01, m02, m12 = (float(x[i] @ x[j]) for i, j in ((0, 1), (0, 2), (1, 2)))
    a, b = a01 / a12, a02 / a12
    # The conics over (1, x, y) = (λ₃, λ₁, λ₂) / λ₃ (0-based: depths d0, d1 over d2):
    # x² - 2m₀₁xy + (1-a)y² + 2am₁₂y - a = 0, and x² - 2m₀₂x + 1 - b(y² - 2m₁₂y + 1) = 0.
    first = np.array([[-a, 0.0, a * m12], [0.0, 1.0, -m01], [a * m12, -m01, 1.0 - a]])
    second = np.array([[1.0 - b, -m02, b * m12], [-m02, 1.0, 0.0], [b * m12, 0.0, -b]])
    c3, c2, c1, c0 = pencil_cubic(first, second)
    if c3 == 0.0:
        return []
    s, single = cubic_root(c2 / c3, c1 / c3, c0 / c3)
    lines = _line_pair(first + s * second)
    if lines is None:
        return []
    geometry = (np.array([m01, m02, m12]), np.array([a01, a02, a12]))
    poses: list[Pose] = []
    for line in lines:
        for depths in _ding_depths(line, geometry):
            if (pose := _finish(depths, x, world, geometry)) is not None:
                poses.append(pose)
        if poses and single:
            break
    return poses


def _ding_depths(p: Vector3, geometry: tuple[Vector3, Vector3]) -> list[Vector3]:
    """Intersect one of Ding et al.'s lines, ``p · (λ₃, λ₁, λ₂) = 0``, with a conic.

    In 0-based depths ``d0, d1, d2`` (the reordered points'): ``d0`` is eliminated if
    ``|p₀| <= |p₁|``, else ``d2``, leaving a quadratic in ``τ = d1/d2`` (resp. ``d1/d0``); one
    distance equation then fixes the scale.

    Returns
    -------
    list of ndarray of float64, shape (3,)
        The candidate depths, every one positive.
    """
    (m01, m02, m12), (a01, a02, a12) = geometry
    a, b = a01 / a12, a02 / a12
    candidates: list[Vector3] = []
    if abs(p[0]) <= abs(p[1]):
        # d0 = w0 d2 + w1 d1 and τ = d1 / d2, from the second conic.
        w0, w1 = -p[0] / p[1], -p[2] / p[1]
        taus = quadratic_roots(
            w1 * w1 - b, 2.0 * (b * m12 - m02 * w1 + w0 * w1), w0 * w0 - 2.0 * m02 * w0 - b + 1.0
        )
        for tau in (t for t in taus if t > 0.0):
            d2 = math.sqrt(a12 / (tau * (tau - 2.0 * m12) + 1.0))
            d1 = tau * d2
            candidates.append(np.array([w0 * d2 + w1 * d1, d1, d2]))
    else:
        # d2 = w0 d0 + w1 d1 and τ = d1 / d0, from the first conic.
        w0, w1 = -p[1] / p[0], -p[2] / p[0]
        taus = quadratic_roots(
            -a * w1 * w1 + 2.0 * a * m12 * w1 - a + 1.0,
            2.0 * (a * m12 * w0 - m01 - a * w0 * w1),
            1.0 - a * w0 * w0,
        )
        for tau in (t for t in taus if t > 0.0):
            d0 = math.sqrt(a01 / (tau * (tau - 2.0 * m01) + 1.0))
            d1 = tau * d0
            candidates.append(np.array([d0, d1, w0 * d0 + w1 * d1]))
    return [d for d in candidates if np.all(d > 0.0)]


def solve_reference(
    points: NDArray[np.float64], bearings: NDArray[np.float64], solver: str
) -> list[Pose]:
    """Run one P3P solver on validated correspondences, unchecked.

    Parameters
    ----------
    points : ndarray of float64, shape (3, 3)
        Three world points.
    bearings : ndarray of float64, shape (3, 3)
        Their unit bearings.
    solver : {"ding", "lambda_twist"}
        The solver.

    Returns
    -------
    list of ndarray of float64, shape (4, 4)
        Every pose ``T_cam_world`` with all three depths positive (none if the points are
        collinear).

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If `solver` is unknown.
    """
    if solver not in SOLVERS:
        raise InvalidInputError(f"solver: expected one of {SOLVERS}, got {solver!r}")
    if _collinear(points):
        return []
    if solver == "ding":
        return _ding(bearings, points)
    return _lambda_twist(bearings, points)


def _reference(
    points: NDArray[np.float64], bearings: NDArray[np.float64], code: int
) -> NDArray[np.float64]:
    """Stack the reference's solutions as (K, 4, 4)."""
    found = solve_reference(points, bearings, SOLVERS[code])
    return np.array(found) if found else np.empty((0, 4, 4))


def p3p(
    points: ArrayLike,
    bearings: ArrayLike,
    *,
    solver: str = "ding",
    backend: str = "python",
) -> NDArray[np.float64]:
    """Return every camera pose three world points and their bearings allow.

    Parameters
    ----------
    points : array_like, shape (3, 3)
        Three world points.
    bearings : array_like, shape (3, 3)
        Their unit bearings in the camera (any sign of z: the solvers use bearings, not image
        points).
    solver : {"ding", "lambda_twist"}, optional
        Ding et al.'s solver (default) or Lambda Twist.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (K, 4, 4)
        The poses ``T_cam_world`` with every point at a positive depth, K <= 4; K = 0 for
        collinear points.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the input is not three finite points and unit bearings, or `solver` is unknown.
    """
    if solver not in SOLVERS:
        raise InvalidInputError(f"solver: expected one of {SOLVERS}, got {solver!r}")
    world, rays = as_correspondences(points, bearings, SAMPLE_SIZE, positive_z=False)
    if world.shape[0] != SAMPLE_SIZE:
        raise InvalidInputError(f"points: expected exactly 3, got {world.shape[0]}")
    code = SOLVERS.index(solver)
    found = backends.dispatch(backend, __name__, "p3p", _reference, world, rays, code)
    return np.asarray(found.reshape(-1, 4, 4), dtype=np.float64)
