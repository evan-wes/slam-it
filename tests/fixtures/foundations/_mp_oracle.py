"""High-precision (mpmath) helpers shared by the fixture generators.

Nothing here imports slam_it_all: the fixtures' expected values must come from algorithms
independent of the code they test. Matrices are lists of rows of mpmath numbers. The
working precision is set by each generator (``with mp.workdps(PRECISION)``), not here, so
importing this module has no side effects.
"""

from __future__ import annotations

from typing import Any

import mpmath as mp

PRECISION = 50
"""Decimal digits the generators work in."""

Matrix = list[list[Any]]


def mpf_vector(values: list[float]) -> list[Any]:
    """Convert floats to mpmath numbers."""
    return [mp.mpf(x) for x in values]


def mpf_matrix(rows: object) -> Matrix:
    """Convert a float matrix (nested lists or ndarray) to mpmath numbers."""
    return [[mp.mpf(float(x)) for x in row] for row in rows]  # type: ignore[attr-defined]


def to_float(matrix: Matrix) -> list[list[float]]:
    """Round an mpmath matrix to float64 nested lists."""
    return [[float(x) for x in row] for row in matrix]


def identity(n: int = 3) -> Matrix:
    """Return the n x n identity."""
    return [[mp.mpf(1) if i == j else mp.mpf(0) for j in range(n)] for i in range(n)]


def hat(v: list[Any]) -> Matrix:
    """Return the skew-symmetric matrix of a 3-vector (mpmath or float; exact for floats)."""
    zero = 0.0 if isinstance(v[0], float) else mp.mpf(0)
    return [[zero, -v[2], v[1]], [v[2], zero, -v[0]], [-v[1], v[0], zero]]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    """Multiply matrices."""
    return [
        [sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
        for i in range(len(a))
    ]


def combine(*terms: tuple[Any, Matrix]) -> Matrix:
    """Sum scalar-weighted matrices of equal shape."""
    rows, cols = len(terms[0][1]), len(terms[0][1][0])
    return [[sum(c * m[i][j] for c, m in terms) for j in range(cols)] for i in range(rows)]


def so3_exp(v: list[Any]) -> Matrix:
    """Rodrigues' formula."""
    th = mp.sqrt(sum(x * x for x in v))
    if th == 0:
        return identity()
    w = hat(v)
    return combine((1, identity()), (mp.sin(th) / th, w), ((1 - mp.cos(th)) / th**2, matmul(w, w)))


def so3_jacobian(v: list[Any], sign: int) -> Matrix:
    """Left (sign +1) or right (sign -1) SO(3) Jacobian, closed form."""
    th = mp.sqrt(sum(x * x for x in v))
    if th == 0:
        return identity()
    w = hat(v)
    return combine(
        (1, identity()),
        (sign * (1 - mp.cos(th)) / th**2, w),
        ((th - mp.sin(th)) / th**3, matmul(w, w)),
    )


def inverse(m: Matrix) -> Matrix:
    """Invert a square matrix."""
    inv = mp.inverse(mp.matrix(m))
    out: Matrix = [[inv[i, j] for j in range(len(m))] for i in range(len(m))]
    return out


def to_quaternion(r: Matrix) -> list[Any]:
    """Rotation to quaternion (xyzw) by Shepperd's method, w >= 0.

    The largest of (trace, r00, r11, r22) picks the component taken from the diagonal; the
    others come from off-diagonal sums, which keep small rotations' information.
    """
    trace = r[0][0] + r[1][1] + r[2][2]
    candidates = [trace, r[0][0], r[1][1], r[2][2]]
    k = max(range(4), key=lambda i: candidates[i])
    if k == 0:
        s = 2 * mp.sqrt(1 + trace)
        q = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1], s * s / 4]
    elif k == 1:
        s = 2 * mp.sqrt(1 + r[0][0] - r[1][1] - r[2][2])
        q = [s * s / 4, r[0][1] + r[1][0], r[0][2] + r[2][0], r[2][1] - r[1][2]]
    elif k == 2:
        s = 2 * mp.sqrt(1 - r[0][0] + r[1][1] - r[2][2])
        q = [r[0][1] + r[1][0], s * s / 4, r[1][2] + r[2][1], r[0][2] - r[2][0]]
    else:
        s = 2 * mp.sqrt(1 - r[0][0] - r[1][1] + r[2][2])
        q = [r[0][2] + r[2][0], r[1][2] + r[2][1], s * s / 4, r[1][0] - r[0][1]]
    q = [c / s for c in q]
    n = mp.sqrt(sum(c * c for c in q))
    q = [c / n for c in q]
    return [-c for c in q] if q[3] < 0 else q


def so3_log(r: Matrix) -> list[Any]:
    """Rotation to rotation vector via the quaternion: ``2 atan2(|q_v|, q_w) q_v / |q_v|``."""
    x, y, z, w = to_quaternion(r)
    nv = mp.sqrt(x * x + y * y + z * z)
    if nv == 0:
        return [mp.mpf(0)] * 3
    angle = 2 * mp.atan2(nv, w)
    return [angle * c / nv for c in (x, y, z)]


def case(name: str, value: object, expected: object, **extra: object) -> dict[str, object]:
    """Return one fixture case."""
    return {"name": name, "input": value, "expected": expected, **extra}
