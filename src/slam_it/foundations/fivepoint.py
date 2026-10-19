"""The 5-point essential matrix: Nistér's and Stewénius' minimal solvers.

E has five degrees of freedom, so five correspondences determine it up to a finite set: at
most ten real solutions. A minimal solver needs the fewest correspondences RANSAC can sample,
which makes an all-inlier sample far likelier (``w⁵`` rather than the 8-point algorithm's
``w⁸``), and it uses the essential constraints exactly rather than projecting a
least-squares solution onto them, so its models are far less sensitive to noise.

**The shared front end.** Each bearing pair gives ``f₂ᵀ E f₁ = 0``, one linear equation in
E's nine entries. Five give a 5x9 system whose null space is four-dimensional (its right
singular vectors ``X, Y, Z, W``), so

    E = x X + y Y + z Z + W

for unknown scalars ``x, y, z`` (W's coefficient fixed to 1, fixing E's scale). E must also be
essential: ``det E = 0`` and ``2 E Eᵀ E - tr(E Eᵀ) E = 0`` (Demazure's constraints). Substituting
gives ten cubic equations in ``x, y, z``, a 10x20 matrix of coefficients over the twenty
monomials of degree up to three.

**Nistér** ("An efficient solution to the five-point relative pose problem", TPAMI 26(6),
2004). With the monomials ordered ``x³ y³ x²y xy² x²z x² y²z y² xyz xy | xz² xz x yz² yz y z³
z² z 1``, Gauss-Jordan elimination of the first ten columns leaves each of the last six rows
(leading ``x²z, x², y²z, y², xyz, xy``) as its leading monomial plus terms in ``x``, ``y`` and
``1`` times powers of ``z``. Subtracting ``z`` times the second of each pair cancels the leading
monomials, leaving three equations ``kₓ(z) x + k_y(z) y + k₁(z) = 0``: a 3x3 matrix ``B(z)``
of polynomials in ``z`` that must be singular. ``det B(z)`` is a polynomial of degree ten; its
real roots are isolated with a **Sturm sequence** and refined by bisection, as Nistér did.
For each root, ``(x, y, 1)`` is B's null vector, the cross product of two of its rows.
Forming ``det B(z)`` loses digits on some samples.

**Stewénius** (Stewénius, Engels and Nistér, "Recent developments on direct relative
orientation", ISPRS J. 60(4), 2006). With the monomials in graded reverse lexicographic order,
``x³ x²y x²z xy² xyz xz² y³ y²z yz² z³ | x² xy xz y² yz z² x y z 1``, Gauss-Jordan
elimination expresses the ten cubic monomials in the ten lower ones, ``b``. That gives the
10x10 **action matrix** of multiplication by ``x`` on ``b``: ``x·b = A b`` at every solution.
So every solution is an eigenvector of A, with ``x`` its eigenvalue, and ``(x, y, z)`` read off
the eigenvector's last four entries. A Gröbner-basis method: one eigen-decomposition instead
of a polynomial and its roots. It is the default: near degeneracy (a small baseline, forward
motion) it keeps the true solution far more often than Nistér's polynomial, whose rounding can
push a real root off the real axis.

**Polishing.** Each solution of either method is polished by Gauss-Newton on the original ten
cubic constraints, to convergence (:data:`POLISH_MAX_STEPS`, :data:`POLISH_TOLERANCE`). Near
degeneracy two roots can polish to the same solution; the copy is dropped
(:data:`DUPLICATE_TOLERANCE`). The measured precision and recall are in
``docs/notes/five-point.md``.

Both return every real solution, scaled to singular values ``(1, 1, 0)``; each is
``±[t̂]ₓ R``. Five pairs whose 5x9 system has rank below five (a repeated or degenerate
correspondence) give no solution. A sample with no baseline is fitted by every ``[t̂]ₓ R``:
the constraints have a continuum of solutions instead of ten points, so the roots the solvers
report are arbitrary, and some do not even give an essential matrix. Those are dropped
(:data:`ESSENTIAL_TOLERANCE`); the rest fit, with an arbitrary ``t``, which RANSAC's scoring
and two-view's homography handle.

**Canonical order.** The solvers find the same solutions in every language, but in an order
that depends on each library's null-space basis and Schur form. So every core returns them
canonically: each E signed so its largest-magnitude entry is positive, then sorted
lexicographically by its row-major entries. Entries within :data:`SIGN_TIE_RTOL` of the
largest count as tied, and the first of them in row-major order decides the sign: a pure
translation's E is skew-symmetric, so its two largest entries are equal with opposite signs,
and roundoff alone would otherwise choose. RANSAC keeps the first of
equally good models, so this keeps the cores' decisions alike; only solutions that agree to
roundoff in their leading entries could still be ordered differently.

The C++ core (``slam_it/foundations/fivepoint.hpp``) implements both.
"""

from __future__ import annotations

import itertools
from typing import Protocol

import numpy as np
from numpy.polynomial import polynomial as npoly
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations.arrays import as_bearing_pairs
from slam_it.foundations.correspondences import RANK_TOLERANCE, Matrix3
from slam_it.foundations.errors import InvalidInputError

SAMPLE_SIZE = 5
"""Correspondences a 5-point solver takes."""

SOLVERS = ("nister", "stewenius")
"""The 5-point solvers, in code order."""

REAL_TOLERANCE = 1e-10
"""An action-matrix eigenvalue counts as real when its imaginary part is below this, relative
to its modulus (at least 1)."""

POLISH_MAX_STEPS = 10
"""At most this many Gauss-Newton steps polish each solution on the ten cubic constraints."""
POLISH_TOLERANCE = 1e-14
"""Polishing stops when a step is below this, relative to the solution (at least 1)."""

ROOT_TOLERANCE = 1e-14
"""Bisection stops when a root's bracket is below this, relative to the root (at least 1)."""

DUPLICATE_TOLERANCE = 1e-8
"""A solution within this (max-abs, up to sign) of one already kept is the same solution: two
roots that polishing merged."""

ESSENTIAL_TOLERANCE = 1e-6
"""A solution is kept only if, scaled to norm √2, ``σ1 - σ2`` and ``σ3`` are both below this.
The solvers are accurate to about 1e-9 on well-posed samples (docs/notes/five-point.md)."""

_Monomial = tuple[int, int, int]

NISTER_ORDER: tuple[_Monomial, ...] = (
    (3, 0, 0), (0, 3, 0), (2, 1, 0), (1, 2, 0), (2, 0, 1), (2, 0, 0), (0, 2, 1), (0, 2, 0),
    (1, 1, 1), (1, 1, 0), (1, 0, 2), (1, 0, 1), (1, 0, 0), (0, 1, 2), (0, 1, 1), (0, 1, 0),
    (0, 0, 3), (0, 0, 2), (0, 0, 1), (0, 0, 0),
)  # fmt: skip
"""Nistér's monomial order, exponents of ``(x, y, z)``."""

GREVLEX_ORDER: tuple[_Monomial, ...] = (
    (3, 0, 0), (2, 1, 0), (2, 0, 1), (1, 2, 0), (1, 1, 1), (1, 0, 2), (0, 3, 0), (0, 2, 1),
    (0, 1, 2), (0, 0, 3), (2, 0, 0), (1, 1, 0), (1, 0, 1), (0, 2, 0), (0, 1, 1), (0, 0, 2),
    (1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0),
)  # fmt: skip
"""Graded reverse lexicographic order, exponents of ``(x, y, z)``."""

_NISTER_ROW_PAIRS = ((4, 5), (6, 7), (8, 9))
"""Rows of Nistér's reduced matrix led by ``x²z, x²``, ``y²z, y²`` and ``xyz, xy``: the first of
each pair minus ``z`` times the second cancels the leading monomials, giving B(z)'s rows."""
_DETERMINANT_TERMS = (
    ((0, 1, 2), 1), ((1, 2, 0), 1), ((2, 0, 1), 1),
    ((0, 2, 1), -1), ((1, 0, 2), -1), ((2, 1, 0), -1),
)  # fmt: skip
"""A 3x3 determinant's six terms: the column taken from each row, and the permutation's sign."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.fivepoint`` submodule provides."""

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order: ``{"solver": [...]}``."""
        ...

    def essential_5pt(
        self, bearings1: NDArray[np.float64], bearings2: NDArray[np.float64], solver: int
    ) -> NDArray[np.float64]:
        """Return every essential matrix five bearing pairs allow, shape (K, 3, 3)."""
        ...


# --- polynomials in (x, y, z) of degree up to three, as (4, 4, 4) coefficient arrays ------


def _multiply(p: NDArray[np.float64], q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Multiply two polynomials, dropping nothing (the product must stay within degree 3)."""
    out = np.zeros((4, 4, 4))
    for a, b, c in zip(*np.nonzero(p), strict=True):
        for d, e, f in zip(*np.nonzero(q), strict=True):
            if a + b + c + d + e + f <= 3:
                out[a + d, b + e, c + f] += p[a, b, c] * q[d, e, f]
    return out


def _null_space(
    bearings1: NDArray[np.float64], bearings2: NDArray[np.float64]
) -> NDArray[np.float64] | None:
    """Return the 5x9 system's null-space basis ``X, Y, Z, W`` as (4, 3, 3), or None."""
    system = (bearings2[:, :, None] * bearings1[:, None, :]).reshape(5, 9)
    _, singular, vt = np.linalg.svd(system, full_matrices=True)
    if not singular[4] > RANK_TOLERANCE * singular[0]:
        return None
    return np.asarray(vt[5:].reshape(4, 3, 3), dtype=np.float64)


def _constraints(basis: NDArray[np.float64], order: tuple[_Monomial, ...]) -> NDArray[np.float64]:
    """Return the ten cubic constraints on ``E = xX + yY + zZ + W`` as a 10x20 matrix."""
    e = np.zeros((3, 3, 4, 4, 4))
    for i, j in itertools.product(range(3), range(3)):
        e[i, j, 1, 0, 0], e[i, j, 0, 1, 0], e[i, j, 0, 0, 1], e[i, j, 0, 0, 0] = basis[:, i, j]
    eet = np.zeros((3, 3, 4, 4, 4))
    for i, j, k in itertools.product(range(3), range(3), range(3)):
        eet[i, j] += _multiply(e[i, k], e[j, k])
    trace = eet[0, 0] + eet[1, 1] + eet[2, 2]
    rows = []
    for i, j in itertools.product(range(3), range(3)):
        cubic = -_multiply(trace, e[i, j])
        for k in range(3):
            cubic += 2.0 * _multiply(eet[i, k], e[k, j])
        rows.append(cubic)
    cofactors = (
        _multiply(e[1, 1], e[2, 2]) - _multiply(e[1, 2], e[2, 1]),
        _multiply(e[1, 2], e[2, 0]) - _multiply(e[1, 0], e[2, 2]),
        _multiply(e[1, 0], e[2, 1]) - _multiply(e[1, 1], e[2, 0]),
    )
    determinant = np.zeros((4, 4, 4))
    for k in range(3):
        determinant += _multiply(e[0, k], cofactors[k])
    rows.insert(0, determinant)
    return np.array([[row[m] for m in order] for row in rows])


def _gauss_jordan(matrix: NDArray[np.float64]) -> NDArray[np.float64] | None:
    """Reduce a 10x20 matrix to ``[I | B]`` by partial pivoting; return B, or None if singular."""
    m = matrix.copy()
    for col in range(10):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))
        if not abs(m[pivot, col]) > 0.0:
            return None
        m[[col, pivot]] = m[[pivot, col]]
        m[col] /= m[col, col]
        for row in range(10):
            if row != col:
                m[row] -= m[row, col] * m[col]
    return np.asarray(m[:, 10:], dtype=np.float64)


def _keep(solutions: list[Matrix3], essential: NDArray[np.float64]) -> None:
    """Scale E to singular values (1, 1, 0), Frobenius norm √2, and keep it if it is essential.

    A solution of a well-posed sample is essential to the solver's precision. A sample with no
    baseline (or nearly none) leaves the cubic constraints a continuum of solutions, and some
    roots the solvers find there give no essential matrix: anything not within
    :data:`ESSENTIAL_TOLERANCE` of singular values (1, 1, 0) is dropped, and so is a copy of a
    solution already kept (:data:`DUPLICATE_TOLERANCE`).
    """
    scaled = np.asarray(essential * (np.sqrt(2.0) / np.linalg.norm(essential)), dtype=np.float64)
    singular = np.linalg.svd(scaled, compute_uv=False)
    if not max(singular[0] - singular[1], singular[2]) <= ESSENTIAL_TOLERANCE:
        return
    for kept in solutions:
        if min(np.abs(kept - scaled).max(), np.abs(kept + scaled).max()) <= DUPLICATE_TOLERANCE:
            return
    solutions.append(scaled)


# --- Nistér: a degree-10 polynomial and its Sturm sequence ---------------------------------


def _sturm_chain(poly: NDArray[np.float64]) -> list[NDArray[np.float64]]:
    """Return the Sturm sequence of a polynomial (coefficients ascending)."""
    chain = [poly, np.asarray(npoly.polyder(poly), dtype=np.float64)]
    while len(chain[-1]) > 1 or chain[-1][0] != 0.0:
        _, remainder = npoly.polydiv(chain[-2], chain[-1])
        remainder = np.asarray(npoly.polytrim(-remainder, tol=0.0), dtype=np.float64)
        if len(remainder) == 1 and remainder[0] == 0.0:
            break
        chain.append(remainder)
        if len(remainder) == 1:
            break
    return chain


def _sign_changes(chain: list[NDArray[np.float64]], z: float) -> int:
    """Count the sign changes of a Sturm sequence at ``z``."""
    signs = [s for s in (np.sign(npoly.polyval(z, p)) for p in chain) if s != 0.0]
    return sum(1 for a, b in itertools.pairwise(signs) if a != b)


def _bisect(poly: NDArray[np.float64], low: float, high: float) -> float:
    """Refine a root bracketed by a sign change of ``poly`` on ``[low, high]``."""
    f_low = npoly.polyval(low, poly)
    while high - low > ROOT_TOLERANCE * max(1.0, abs(low), abs(high)):
        middle = 0.5 * (low + high)
        f_middle = npoly.polyval(middle, poly)
        if f_middle == 0.0:
            return middle
        if (f_middle < 0.0) == (f_low < 0.0):
            low, f_low = middle, f_middle
        else:
            high = middle
    return 0.5 * (low + high)


def real_roots(poly: ArrayLike) -> NDArray[np.float64]:
    """Return a polynomial's distinct real roots, ascending, by Sturm sequences and bisection.

    Sturm's theorem counts the real roots in an interval by the drop in sign changes of the
    sequence ``p, p', -rem(p, p'), …`` between its ends. Starting from Cauchy's bound on the
    roots' size, intervals are halved until each holds one root, which bisection then refines.

    Parameters
    ----------
    poly : array_like, shape (n + 1,)
        Coefficients, ascending (``poly[k]`` multiplies ``zᵏ``).

    Returns
    -------
    ndarray of float64, shape (r,)
        The real roots, ascending.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the coefficients are not a finite 1-D array.
    """
    coefficients = np.asarray(poly, dtype=np.float64)
    if coefficients.ndim != 1 or not np.all(np.isfinite(coefficients)):
        raise InvalidInputError(
            f"poly: expected finite coefficients, shape (n + 1,), got shape {coefficients.shape}"
        )
    p = npoly.polytrim(coefficients, tol=0.0)
    if len(p) < 2:
        return np.empty(0)
    p = p / p[-1]
    bound = 1.0 + float(np.max(np.abs(p[:-1])))
    chain = _sturm_chain(p)
    roots: list[float] = []
    stack = [(-bound, bound, _sign_changes(chain, -bound), _sign_changes(chain, bound))]
    while stack:
        low, high, s_low, s_high = stack.pop()
        count = s_low - s_high
        if count <= 0:
            continue
        if count == 1 and np.sign(npoly.polyval(low, p)) != np.sign(npoly.polyval(high, p)):
            roots.append(_bisect(p, low, high))
            continue
        if high - low < ROOT_TOLERANCE * max(1.0, abs(low)):
            roots.append(0.5 * (low + high))  # a multiple root, or roots closer than resolvable
            continue
        middle = 0.5 * (low + high)
        s_middle = _sign_changes(chain, middle)
        stack.append((middle, high, s_middle, s_high))
        stack.append((low, middle, s_low, s_middle))
    return np.sort(np.array(roots))


def _z_polynomials(tail: NDArray[np.float64]) -> tuple[NDArray[np.float64], ...]:
    """Split a row's tail into the polynomials in z multiplying ``x``, ``y`` and ``1``.

    The tail is over ``xz² xz x yz² yz y z³ z² z 1``; the polynomials are ascending in z.
    """
    return (
        np.array([tail[2], tail[1], tail[0]]),
        np.array([tail[5], tail[4], tail[3]]),
        np.array([tail[9], tail[8], tail[7], tail[6]]),
    )


def _residual(
    constraints: NDArray[np.float64], exponents: NDArray[np.int64], xyz: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Evaluate the ten cubic constraints at ``(x, y, z)``."""
    return np.asarray(constraints @ np.prod(xyz[None, :] ** exponents, axis=1), dtype=np.float64)


def _polish(
    constraints: NDArray[np.float64], order: tuple[_Monomial, ...], xyz: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Refine ``(x, y, z)`` by Gauss-Newton on the ten cubic constraints, to convergence.

    A step is taken only if it lowers the constraints' residual; the polish stops when a step
    is below :data:`POLISH_TOLERANCE` of the solution, does not help, or after
    :data:`POLISH_MAX_STEPS`.
    """
    exponents = np.array(order)
    residual = _residual(constraints, exponents, xyz)
    for _ in range(POLISH_MAX_STEPS):
        derivative = np.zeros((20, 3))
        for k in range(3):
            lowered = exponents.copy()
            lowered[:, k] = np.maximum(lowered[:, k] - 1, 0)
            derivative[:, k] = exponents[:, k] * np.prod(xyz[None, :] ** lowered, axis=1)
        step = np.linalg.lstsq(constraints @ derivative, -residual, rcond=None)[0]
        if not np.all(np.isfinite(step)):
            break
        moved = xyz + step
        moved_residual = _residual(constraints, exponents, moved)
        if not np.linalg.norm(moved_residual) < np.linalg.norm(residual):
            break
        xyz, residual = moved, moved_residual
        if np.linalg.norm(step) <= POLISH_TOLERANCE * max(1.0, float(np.linalg.norm(xyz))):
            break
    return xyz


def _nister(bearings1: NDArray[np.float64], bearings2: NDArray[np.float64]) -> list[Matrix3]:
    """Solve five bearing pairs by Nistér's method (reference)."""
    basis = _null_space(bearings1, bearings2)
    if basis is None:
        return []
    constraints = _constraints(basis, NISTER_ORDER)
    reduced = _gauss_jordan(constraints)
    if reduced is None:
        return []
    rows = []
    for upper, lower in _NISTER_ROW_PAIRS:
        a, b = _z_polynomials(reduced[upper]), _z_polynomials(reduced[lower])
        rows.append([npoly.polysub(p, npoly.polymulx(q)) for p, q in zip(a, b, strict=True)])
    determinant = np.zeros(1)
    for (i, j, k), sign in _DETERMINANT_TERMS:
        term = npoly.polymul(npoly.polymul(rows[0][i], rows[1][j]), rows[2][k])
        determinant = np.asarray(npoly.polyadd(determinant, sign * term), dtype=np.float64)
    solutions: list[Matrix3] = []
    for root in real_roots(determinant):
        matrix = np.array([[npoly.polyval(root, entry) for entry in row] for row in rows])
        v = np.cross(matrix[0], matrix[1])
        if not abs(v[2]) > 0.0:
            continue
        x, y, z = _polish(constraints, NISTER_ORDER, np.array([v[0] / v[2], v[1] / v[2], root]))
        _keep(solutions, x * basis[0] + y * basis[1] + z * basis[2] + basis[3])
    return solutions


# --- Stewénius: the action matrix of multiplication by x -----------------------------------

_ACTION_ROWS = (0, 1, 2, 3, 4, 5)
"""Rows of B giving ``x·b`` for ``b = x², xy, xz, y², yz, z²`` (the cubics ``x³ … xz²``)."""
_ACTION_SHIFTS = ((6, 0), (7, 1), (8, 2), (9, 6))
"""``x·x = x², x·y = xy, x·z = xz, x·1 = x``: (row of A, index into b)."""


def _stewenius(bearings1: NDArray[np.float64], bearings2: NDArray[np.float64]) -> list[Matrix3]:
    """Solve five bearing pairs by Stewénius' action matrix (reference)."""
    basis = _null_space(bearings1, bearings2)
    if basis is None:
        return []
    constraints = _constraints(basis, GREVLEX_ORDER)
    reduced = _gauss_jordan(constraints)
    if reduced is None:
        return []
    action = np.zeros((10, 10))
    action[: len(_ACTION_ROWS)] = -reduced[list(_ACTION_ROWS)]
    for row, index in _ACTION_SHIFTS:
        action[row, index] = 1.0
    values, vectors = np.linalg.eig(action)
    solutions: list[Matrix3] = []
    for value, vector in zip(values, vectors.T, strict=True):
        if abs(value.imag) > REAL_TOLERANCE * max(1.0, abs(value)) or not abs(vector[9]) > 0.0:
            continue
        x, y, z = _polish(constraints, GREVLEX_ORDER, (vector[6:9] / vector[9]).real)
        _keep(solutions, x * basis[0] + y * basis[1] + z * basis[2] + basis[3])
    return solutions


SIGN_TIE_RTOL = 1e-6
"""Entries of E within this relative margin of the largest magnitude are tied for its sign.

Wider than the cores' disagreement on a solution (about 1e-10 relative), so they choose the
same entry; see the module's "Canonical order"."""


def _canonical(solutions: list[Matrix3]) -> list[Matrix3]:
    """Put the solutions in canonical order (see the module's "Canonical order").

    Each is signed so that its first entry, in row-major order, within :data:`SIGN_TIE_RTOL`
    of the largest magnitude is positive; then all are sorted lexicographically by their
    row-major entries.
    """
    signed = []
    for essential in solutions:
        flat = essential.ravel()
        magnitude = np.abs(flat)
        first = int(np.flatnonzero(magnitude >= magnitude.max() * (1.0 - SIGN_TIE_RTOL))[0])
        signed.append(essential if flat[first] > 0.0 else -essential)
    return sorted(signed, key=lambda e: tuple(e.ravel()))


def solve_reference(
    bearings1: NDArray[np.float64], bearings2: NDArray[np.float64], solver: str
) -> list[Matrix3]:
    """Run one 5-point solver on validated bearing pairs, unchecked.

    Parameters
    ----------
    bearings1, bearings2 : ndarray of float64, shape (5, 3)
        Five unit bearing pairs.
    solver : {"nister", "stewenius"}
        The solver.

    Returns
    -------
    list of ndarray of float64, shape (3, 3)
        Every real solution that is essential (none for a degenerate sample), in canonical
        order (see the module's "Canonical order").

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `solver` is unknown.
    """
    if solver == "nister":
        return _canonical(_nister(bearings1, bearings2))
    if solver == "stewenius":
        return _canonical(_stewenius(bearings1, bearings2))
    raise InvalidInputError(f"solver: expected one of {SOLVERS}, got {solver!r}")


def _reference(b1: NDArray[np.float64], b2: NDArray[np.float64], code: int) -> NDArray[np.float64]:
    """Stack the reference's solutions as (K, 3, 3)."""
    found = solve_reference(b1, b2, SOLVERS[code])
    return np.array(found) if found else np.empty((0, 3, 3))


def essential_5pt(
    bearings1: ArrayLike,
    bearings2: ArrayLike,
    *,
    solver: str = "stewenius",
    backend: str = "python",
) -> NDArray[np.float64]:
    """Return every essential matrix five bearing pairs allow.

    Parameters
    ----------
    bearings1, bearings2 : array_like, shape (5, 3)
        Five unit bearing pairs (any sign of z: the solvers use bearings, not image points).
    solver : {"nister", "stewenius"}, optional
        Stewénius' action matrix (default) or Nistér's polynomial and Sturm sequence.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation computes the result (default: the Python reference).

    Returns
    -------
    ndarray of float64, shape (K, 3, 3)
        The real solutions, K <= 10, each with singular values (1, 1, 0), in canonical order
        (see the module's "Canonical order"); K = 0 for a degenerate sample.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the bearings are not five unit pairs, or `solver` is unknown.
    """
    if solver not in SOLVERS:
        raise InvalidInputError(f"solver: expected one of {SOLVERS}, got {solver!r}")
    b1, b2 = as_bearing_pairs(bearings1, bearings2, SAMPLE_SIZE, positive_z=False)
    if b1.shape[0] != SAMPLE_SIZE:
        raise InvalidInputError(f"bearings: expected exactly 5 pairs, got {b1.shape[0]}")
    code = SOLVERS.index(solver)
    found = backends.dispatch(backend, __name__, "essential_5pt", _reference, b1, b2, code)
    return np.asarray(found.reshape(-1, 3, 3), dtype=np.float64)
