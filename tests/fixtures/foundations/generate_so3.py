"""Generate ``so3.json``, the SO(3) parity fixture, from a 50-digit mpmath oracle.

Run from the repository root after changing the cases (the fixture is committed)::

    python tests/fixtures/foundations/generate_so3.py

Every expected value is computed in 50-digit arithmetic, with algorithms independent of
``slamlab.foundations.so3``: Rodrigues' formula for ``exp``; a quaternion route for ``log``
(``2 atan2(|q_v|, q_w)``); the closed-form Jacobians evaluated in high precision (no series
needed); mpmath's own SVD for ``project_to_so3``. Nothing here imports slamlab. manif and
SciPy are deliberately not the oracle: manif's Jacobians lose about 1e-11 near θ = 1e-6.

The inputs are float64. Each case's expected value is the exact (to 50 digits) result for
that float64 input, rounded to float64, so the tolerance only has to cover the
implementation's own rounding.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np

mp.mp.dps = 50
OUT = Path(__file__).with_name("so3.json")
AXIS = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])

ANGLES: dict[str, float] = {
    "zero": 0.0,
    "tiny 1e-12": 1e-12,
    "small 1e-8": 1e-8,
    "below exp Taylor switch": 1e-4 * (1 - 1e-9),
    "above exp Taylor switch": 1e-4 * (1 + 1e-9),
    "below Jacobian Taylor switch": 0.05 * (1 - 1e-9),
    "above Jacobian Taylor switch": 0.05 * (1 + 1e-9),
    "moderate 0.7": 0.7,
    "large 2.5": 2.5,
    "near pi, sin below the switch": math.pi - 5e-4,
    "near pi 1e-6": math.pi - 1e-6,
    "near pi 1e-12": math.pi - 1e-12,
    "above pi (wraps)": 4.0,
    "near 2 pi": 6.0,
}


def _mat(rows: list[list[Any]]) -> list[list[float]]:
    """Round an mpmath matrix (as rows) to float64 nested lists."""
    return [[float(x) for x in row] for row in rows]


def _hat(v: list[Any]) -> list[list[Any]]:
    """Skew-symmetric matrix of a 3-vector (mpmath or float; with floats it is exact)."""
    zero = 0.0 if isinstance(v[0], float) else mp.mpf(0)
    return [[zero, -v[2], v[1]], [v[2], zero, -v[0]], [-v[1], v[0], zero]]


def _matmul(a: list[list[Any]], b: list[list[Any]]) -> list[list[Any]]:
    """3x3 matrix product."""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _combine(*terms: tuple[Any, list[list[Any]]]) -> list[list[Any]]:
    """Sum of scalar-weighted 3x3 matrices."""
    return [[sum(c * m[i][j] for c, m in terms) for j in range(3)] for i in range(3)]


def _identity() -> list[list[Any]]:
    """Return the 3x3 identity in mpmath."""
    return [[mp.mpf(1) if i == j else mp.mpf(0) for j in range(3)] for i in range(3)]


def _exp(v: list[Any]) -> list[list[Any]]:
    """Rodrigues' formula in high precision."""
    th = mp.sqrt(sum(x * x for x in v))
    w = _hat(v)
    if th == 0:
        return _identity()
    return _combine(
        (1, _identity()), (mp.sin(th) / th, w), ((1 - mp.cos(th)) / th**2, _matmul(w, w))
    )


def _jacobian(v: list[Any], sign: int) -> list[list[Any]]:
    """Left (sign +1) or right (sign -1) Jacobian, closed form in high precision."""
    th = mp.sqrt(sum(x * x for x in v))
    if th == 0:
        return _identity()
    w = _hat(v)
    b = (1 - mp.cos(th)) / th**2
    c = (th - mp.sin(th)) / th**3
    return _combine((1, _identity()), (sign * b, w), (c, _matmul(w, w)))


def _jacobian_inverse(v: list[Any], sign: int) -> list[list[Any]]:
    """Inverse of the left (sign +1) or right (sign -1) Jacobian, by matrix inversion."""
    inv = mp.inverse(mp.matrix(_jacobian(v, sign)))
    return [[inv[i, j] for j in range(3)] for i in range(3)]


def _to_quaternion(r: list[list[Any]]) -> list[Any]:
    """Rotation to quaternion (xyzw) by Shepperd's method in high precision, w >= 0.

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


def _log(r: list[list[Any]]) -> list[Any]:
    """Rotation to rotation vector via the quaternion: ``2 atan2(|q_v|, q_w) q_v / |q_v|``."""
    x, y, z, w = _to_quaternion(r)
    nv = mp.sqrt(x * x + y * y + z * z)
    if nv == 0:
        return [mp.mpf(0)] * 3
    angle = 2 * mp.atan2(nv, w)
    return [angle * c / nv for c in (x, y, z)]


def _project(m: list[list[float]]) -> list[list[Any]]:
    """Nearest rotation via mpmath's SVD."""
    u, _, vt = mp.svd_r(mp.matrix(m))
    d = mp.diag([1, 1, mp.sign(mp.det(u * vt))])
    p = u * d * vt
    return [[p[i, j] for j in range(3)] for i in range(3)]


def _case(name: str, value: object, expected: object, **extra: object) -> dict[str, object]:
    """One fixture case."""
    return {"name": name, "input": value, "expected": expected, **extra}


def main() -> None:
    """Write ``so3.json``."""
    vectors = {name: (theta * AXIS).tolist() for name, theta in ANGLES.items()}
    vectors["general"] = [0.1, 1 / 3, -0.7]
    vectors["float32-inexact"] = [0.1, 1 / 3, 1e-300]
    exact_pi = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    rotations = {
        name: np.asarray(_mat(_exp([mp.mpf(x) for x in v]))) for name, v in vectors.items()
    }

    def mpv(v: list[float]) -> list[Any]:
        return [mp.mpf(x) for x in v]

    def mpm(m: object) -> list[list[Any]]:
        return [[mp.mpf(float(x)) for x in row] for row in np.asarray(m)]

    fx: dict[str, Any] = {}
    fx["hat"] = {
        "tolerance": {
            "comparison": "exact",
            "rtol": 0.0,
            "atol": 0.0,
            "reason": "hat copies and negates; no rounding",
        },
        # Built from the float input with float negation: exact, and it keeps signed zeros.
        "cases": [_case(n, v, _hat([float(x) for x in v])) for n, v in vectors.items()],
    }
    fx["vee"] = {
        "tolerance": {
            "comparison": "exact",
            "rtol": 0.0,
            "atol": 0.0,
            "reason": "vee copies entries; no rounding",
        },
        "cases": [_case(n, _hat([float(x) for x in v]), v) for n, v in vectors.items()],
    }
    fx["exp"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 2e-15,
            "reason": "entries <= 1; a few roundings in Rodrigues' formula",
        },
        "cases": [_case(n, v, _mat(_exp(mpv(v)))) for n, v in vectors.items()],
    }
    log_cases = [
        _case(n, r.tolist(), [float(x) for x in _log(mpm(r))]) for n, r in rotations.items()
    ]
    log_cases.append(_case("exactly pi (canonical sign)", exact_pi, [math.pi, 0.0, 0.0]))
    fx["log"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 1e-14,
            "reason": "|omega| <= pi; the axis near pi comes from the symmetric part",
        },
        "cases": log_cases,
    }
    for name, sign in (("right_jacobian", -1), ("left_jacobian", 1)):
        fx[name] = {
            "tolerance": {
                "comparison": "close",
                "rtol": 0.0,
                "atol": 2e-15,
                "reason": "entries O(1); series below the switch, closed form above",
            },
            "cases": [_case(n, v, _mat(_jacobian(mpv(v), sign))) for n, v in vectors.items()],
        }
    for name, sign in (("right_jacobian_inverse", -1), ("left_jacobian_inverse", 1)):
        fx[name] = {
            "tolerance": {
                "comparison": "close",
                "rtol": 0.0,
                "atol": 2e-14,
                "reason": "entries grow toward 2 pi (cot(theta/2)); relative 1e-15",
            },
            "cases": [
                _case(n, v, _mat(_jacobian_inverse(mpv(v), sign))) for n, v in vectors.items()
            ],
        }
    quats = {n: [float(x) for x in _to_quaternion(mpm(r))] for n, r in rotations.items()}
    fx["to_quaternion"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 2e-15,
            "reason": "unit quaternion; Shepperd's method",
            "double_cover": True,
        },
        "cases": [_case(n, rotations[n].tolist(), q) for n, q in quats.items()],
    }
    fx["from_quaternion"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 2e-15,
            "reason": "entries <= 1; quadratic in the quaternion",
        },
        "cases": [
            _case(n, q, _mat(_exp(mpv(vectors[n]))))
            for n, q in quats.items()
            if np.isclose(np.linalg.norm(q), 1.0)
        ],
    }
    rng = np.random.default_rng(7)
    project_inputs = {
        "rotation + noise 1e-3": rotations["general"] + 1e-3 * rng.normal(size=(3, 3)),
        "rotation + noise 0.3": rotations["moderate 0.7"] + 0.3 * rng.normal(size=(3, 3)),
        "reflection-like": np.diag([1.0, 1.0, -1.0]) + 0.1 * rng.normal(size=(3, 3)),
        "already a rotation": rotations["large 2.5"],
    }
    fx["project_to_so3"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 1e-13,
            "reason": "an SVD; the reflection-like case flips the weakest singular "
            "direction, amplifying rounding by about 1/(s2 - s3)",
        },
        "cases": [
            _case(n, m.tolist(), _mat(_project(m.tolist()))) for n, m in project_inputs.items()
        ],
    }
    doc = {
        "description": __doc__.split("\n\n")[0],
        "oracle": "mpmath, 50 digits, independent algorithms (see generate_so3.py)",
        "functions": fx,
    }
    OUT.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote {OUT} ({sum(len(f['cases']) for f in fx.values())} cases)")


if __name__ == "__main__":
    main()
