"""Generate ``ill_conditioned.json``: inputs where a weaker SVD gives a different answer.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_ill_conditioned

PR 6's panel (L1, L2) found nalgebra 0.35's SVDs not backward stable, and the parity fixtures
did not notice: their inputs are well conditioned. These cases are not, and every backend must
still match the 50-digit oracle:

- ``essential_8pt`` and ``fundamental_8pt``: eight near-planar, noisy 20-match scenes from a
  seeded stream (numpy ``default_rng(0)``, the panel's probe), the ones where the old Rust SVD
  was off by 1.6e-6 to 6e-3. Expected: the 8-point definitions of ``generate_twoview``;
- ``decompose_essential``: a matrix with singular values (1, 1e-6, 0), its candidates from
  mpmath's SVD by the decomposition's definition;
- ``umeyama``: three points on thin triangles (a third point 1e-3 off the line of the other
  two), where a 3x3 SVD through ``MᵀM`` loses the rotation. Expected: ``generate_se3``'s
  50-digit Umeyama.

Each tolerance is about ten times the largest error any backend showed against the oracle
when the fixture was made (LAPACK, Eigen and faer agree to 1e-10 or better), and orders of
magnitude below the old Rust errors (1.6e-6 to 6e-3 for E and F, about 0.7 for Umeyama).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np
from scipy.spatial.transform import Rotation

from tests.fixtures.foundations import _mp_oracle as o
from tests.fixtures.foundations.generate_se3 import _umeyama_mp
from tests.fixtures.foundations.generate_twoview import (
    _essential_8pt,
    _float,
    _fundamental_8pt,
    _pose_list,
)

HERE = Path(__file__).parent
BAD_SCENES = (1400, 1718, 2150, 4094, 5786, 9398, 11552, 17228)
"""The probe's scene indices where nalgebra's SVD gave a wrong E or F."""


def _scenes() -> dict[int, dict[str, list[list[float]]]]:
    """Rebuild the probe's scene stream and keep :data:`BAD_SCENES` (bearings and pixels)."""
    rng = np.random.default_rng(0)
    kept = {}
    for t in range(max(BAD_SCENES) + 1):
        rot = Rotation.from_rotvec(rng.uniform(-0.3, 0.3, 3)).as_matrix()
        tr = rng.uniform(-1, 1, 3) * [1, 1, 0.2]
        tr /= np.linalg.norm(tr)
        flat = 1e-3 if t % 2 == 0 else 1.0
        noise = [0.0, 1e-4, 1e-3][t % 3]
        x = np.column_stack(
            [rng.uniform(-1, 1, 20), rng.uniform(-1, 1, 20), 4 + flat * rng.uniform(-1, 1, 20)]
        )
        y = x @ rot.T + tr
        x1 = x[:, :2] / x[:, 2:] + noise * rng.uniform(-1, 1, (20, 2))
        x2 = y[:, :2] / y[:, 2:] + noise * rng.uniform(-1, 1, (20, 2))
        if t in BAD_SCENES:
            b1 = np.column_stack([x1, np.ones(20)])
            b2 = np.column_stack([x2, np.ones(20)])
            kept[t] = {
                "bearings1": (b1 / np.linalg.norm(b1, axis=1, keepdims=True)).tolist(),
                "bearings2": (b2 / np.linalg.norm(b2, axis=1, keepdims=True)).tolist(),
                "pixels1": (500 * x1 + 320).tolist(),
                "pixels2": (500 * x2 + 240).tolist(),
            }
    return kept


def _decompose_mp(e: mp.matrix) -> list[list[list[float]]]:
    """E's four candidate poses by the decomposition's definition, in 50 digits."""
    u, _, vt = mp.svd_r(e)
    if mp.det(u) < 0:
        u = -u
    if mp.det(vt) < 0:
        vt = -vt
    w = mp.matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    t = u[:, 2]
    return [_pose_list(r, sign * t) for r in (u * w * vt, u * w.T * vt) for sign in (1, -1)]


def _epipolar_cases() -> dict[str, list[dict[str, object]]]:
    """Build the 8-point scenes and the near-rank-deficient decomposition."""
    scenes = _scenes()
    essential = [
        o.case(
            f"near-planar scene {t}",
            {"bearings1": s["bearings1"], "bearings2": s["bearings2"]},
            _float(_essential_8pt(s["bearings1"], s["bearings2"])),
        )
        for t, s in scenes.items()
    ]
    fundamental = [
        o.case(
            f"near-planar scene {t}",
            {"pixels1": s["pixels1"], "pixels2": s["pixels2"]},
            _float(_fundamental_8pt(s["pixels1"], s["pixels2"])),
        )
        for t, s in scenes.items()
    ]
    rng = np.random.default_rng(3)
    decompose = []
    for k in range(4):
        a = o.so3_exp(o.mpf_vector(rng.uniform(-2, 2, 3).tolist()))
        b = o.so3_exp(o.mpf_vector(rng.uniform(-2, 2, 3).tolist()))
        e = mp.matrix(a) * mp.diag([1, mp.mpf("1e-6"), 0]) * mp.matrix(b)
        decompose.append(
            o.case(f"singular values (1, 1e-6, 0), {k}", {"essential": _float(e)}, _decompose_mp(e))
        )
    return {
        "essential_8pt": essential,
        "fundamental_8pt": fundamental,
        "decompose_essential": decompose,
    }


def _umeyama_cases() -> list[dict[str, object]]:
    """Build rigid alignments of thin triangles."""
    rng = np.random.default_rng(7)
    cases = []
    for k in range(6):
        base = rng.normal(size=3)
        side = np.cross(base, rng.normal(size=3))
        side /= np.linalg.norm(side)
        source = np.array([[0.0, 0.0, 0.0], base, 0.5 * base + 1e-3 * side]) + rng.normal(size=3)
        rot = o.so3_exp(o.mpf_vector(rng.uniform(-2, 2, 3).tolist()))
        t = rng.normal(size=3)
        xm = o.mpf_matrix(source)
        target = [
            [float(sum(rot[r][c] * xm[i][c] for c in range(3)) + t[r]) for r in range(3)]
            for i in range(3)
        ]
        expected = _umeyama_mp(xm, o.mpf_matrix(target), o.mpf_vector([1.0] * 3), False)
        cases.append(
            o.case(
                f"thin triangle (1e-3), {k}",
                {
                    "source": source.tolist(),
                    "target": target,
                    "weights": [1.0, 1.0, 1.0],
                    "with_scale": False,
                },
                o.to_float(expected),
            )
        )
    return cases


def main() -> None:
    """Write ``ill_conditioned.json``."""
    tol: dict[str, Any] = {"comparison": "close", "rtol": 0.0, "double_cover": True}
    with mp.workdps(o.PRECISION):
        epipolar = _epipolar_cases()
        umeyama = _umeyama_cases()
    doc = {
        "description": "Ill-conditioned inputs: parity fixture from mpmath "
        "(generate_ill_conditioned.py).",
        "oracle": "50-digit 8-point definitions, SVD decomposition and Umeyama",
        "functions": {
            "essential_8pt": {
                "tolerance": {**tol, "atol": 1e-11, "reason": "near-planar 8-point systems"},
                "cases": epipolar["essential_8pt"],
            },
            "fundamental_8pt": {
                "tolerance": {**tol, "atol": 1e-10, "reason": "near-planar 8-point systems"},
                "cases": epipolar["fundamental_8pt"],
            },
            "decompose_essential": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-9,
                    "reason": "σ₂ = 1e-6: the third singular pair is known to about ε / σ₂;"
                    " compared as an unordered set of four poses",
                },
                "cases": epipolar["decompose_essential"],
            },
            "umeyama": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 3e-9,
                    "reason": "a thin triangle's covariance has σ₂ / σ₁ near 1e-3",
                },
                "cases": umeyama,
            },
        },
    }
    path = HERE / "ill_conditioned.json"
    path.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
