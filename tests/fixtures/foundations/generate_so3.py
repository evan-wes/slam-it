"""Generate ``so3.json``, the SO(3) parity fixture, from a 50-digit mpmath oracle.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_so3

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

from tests.fixtures.foundations import _mp_oracle as o

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


def _jacobian_inverse(v: list[Any], sign: int) -> list[list[Any]]:
    """Invert the left (sign +1) or right (sign -1) Jacobian by matrix inversion."""
    return o.inverse(o.so3_jacobian(v, sign))


def _project(m: list[list[float]]) -> list[list[Any]]:
    """Nearest rotation via mpmath's SVD."""
    u, _, vt = mp.svd_r(mp.matrix(m))
    d = mp.diag([1, 1, mp.sign(mp.det(u * vt))])
    p = u * d * vt
    return [[p[i, j] for j in range(3)] for i in range(3)]


def main() -> None:
    """Write ``so3.json``."""
    vectors = {name: (theta * AXIS).tolist() for name, theta in ANGLES.items()}
    vectors["general"] = [0.1, 1 / 3, -0.7]
    vectors["float32-inexact"] = [0.1, 1 / 3, 1e-300]
    exact_pi = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    rotations = {
        name: np.asarray(o.to_float(o.so3_exp(o.mpf_vector(v)))) for name, v in vectors.items()
    }

    mpv, mpm = o.mpf_vector, o.mpf_matrix

    fx: dict[str, Any] = {}
    fx["hat"] = {
        "tolerance": {
            "comparison": "exact",
            "rtol": 0.0,
            "atol": 0.0,
            "reason": "hat copies and negates; no rounding",
        },
        # Built from the float input with float negation: exact, and it keeps signed zeros.
        "cases": [o.case(n, v, o.hat([float(x) for x in v])) for n, v in vectors.items()],
    }
    fx["vee"] = {
        "tolerance": {
            "comparison": "exact",
            "rtol": 0.0,
            "atol": 0.0,
            "reason": "vee copies entries; no rounding",
        },
        "cases": [o.case(n, o.hat([float(x) for x in v]), v) for n, v in vectors.items()],
    }
    fx["exp"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 2e-15,
            "reason": "entries <= 1; a few roundings in Rodrigues' formula",
        },
        "cases": [o.case(n, v, o.to_float(o.so3_exp(mpv(v)))) for n, v in vectors.items()],
    }
    log_cases = [
        o.case(n, r.tolist(), [float(x) for x in o.so3_log(mpm(r))]) for n, r in rotations.items()
    ]
    log_cases.append(o.case("exactly pi (canonical sign)", exact_pi, [math.pi, 0.0, 0.0]))
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
            "cases": [
                o.case(n, v, o.to_float(o.so3_jacobian(mpv(v), sign))) for n, v in vectors.items()
            ],
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
                o.case(n, v, o.to_float(_jacobian_inverse(mpv(v), sign)))
                for n, v in vectors.items()
            ],
        }
    quats = {n: [float(x) for x in o.to_quaternion(mpm(r))] for n, r in rotations.items()}
    fx["to_quaternion"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 2e-15,
            "reason": "unit quaternion; Shepperd's method",
            "double_cover": True,
        },
        "cases": [o.case(n, rotations[n].tolist(), q) for n, q in quats.items()],
    }
    fx["from_quaternion"] = {
        "tolerance": {
            "comparison": "close",
            "rtol": 0.0,
            "atol": 2e-15,
            "reason": "entries <= 1; quadratic in the quaternion",
        },
        "cases": [
            o.case(n, q, o.to_float(o.so3_exp(mpv(vectors[n]))))
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
            o.case(n, m.tolist(), o.to_float(_project(m.tolist())))
            for n, m in project_inputs.items()
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
    with mp.workdps(o.PRECISION):
        main()
