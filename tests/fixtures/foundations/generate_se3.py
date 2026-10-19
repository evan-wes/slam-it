"""Generate ``se3.json`` and ``alignment.json`` from a 50-digit mpmath oracle.

Run from the repository root after changing the cases (the fixtures are committed)::

    python -m tests.fixtures.foundations.generate_se3

Independent methods, nothing imported from slam_it:

- ``exp``: mpmath's matrix exponential of the 4x4 twist matrix;
- ``log``: the rotation vector by the quaternion route, then ``ρ = Jl(θ)⁻¹ t`` by matrix
  inversion;
- the Jacobians: the series ``Jl(ξ) = Σ adⁿ / (n+1)!`` (``Jr(ξ) = Jl(-ξ)``), inverses by
  matrix inversion -- no ``Q`` formula involved;
- ``inverse``, ``compose``, ``act``, ``adjoint``: their definitions in high precision;
- ``umeyama``: exact synthetic similarity transforms (target computed in 50 digits from a
  known s, R, t), plus a weighted noisy case solved by mpmath's own SVD.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np

from tests.fixtures.foundations import _mp_oracle as o

HERE = Path(__file__).parent
SERIES_TERMS = 150
"""Terms of ``Σ adⁿ/(n+1)!``; the twists here have spectral radius below 10, so the tail is
far below 1e-50."""

ANGLES: dict[str, float] = {
    "zero rotation": 0.0,
    "tiny 1e-10": 1e-10,
    "small 1e-4": 1e-4,
    "below series switch": 1.0 * (1 - 1e-9),
    "above series switch": 1.0 * (1 + 1e-9),
    "large 2.5": 2.5,
    "near pi 1e-6": math.pi - 1e-6,
    "above pi": 4.0,
}


def _twist_matrix(xi: list[Any]) -> mp.matrix:
    """Build the 4x4 twist matrix ``[[hat(θ), ρ], [0, 0]]``."""
    m = mp.zeros(4, 4)
    w = o.hat(xi[3:])
    for i in range(3):
        for j in range(3):
            m[i, j] = w[i][j]
        m[i, 3] = xi[i]
    return m


def _ad(xi: list[Any]) -> o.Matrix:
    """Build the 6x6 ``ad(ξ) = [[hat(θ), hat(ρ)], [0, hat(θ)]]``."""
    r, p = o.hat(xi[:3]), o.hat(xi[3:])
    out = [[mp.mpf(0)] * 6 for _ in range(6)]
    for i in range(3):
        for j in range(3):
            out[i][j] = p[i][j]
            out[3 + i][3 + j] = p[i][j]
            out[i][3 + j] = r[i][j]
    return out


def _left_jacobian(xi: list[Any]) -> o.Matrix:
    """``Jl(ξ) = Σ_n adⁿ / (n+1)!``."""
    a = mp.matrix(_ad(xi))
    term = mp.eye(6)
    total = mp.eye(6)
    for n in range(1, SERIES_TERMS):
        term = term * a / (n + 1)
        total += term
    return [[total[i, j] for j in range(6)] for i in range(6)]


def _exp(xi: list[Any]) -> o.Matrix:
    """``exp`` by mpmath's matrix exponential."""
    e = mp.expm(_twist_matrix(xi))
    return [[e[i, j] for j in range(4)] for i in range(4)]


def _log(t: o.Matrix) -> list[Any]:
    """``log``: the quaternion-route rotation vector, then ``ρ = Jl(θ)⁻¹ t``."""
    theta = o.so3_log([row[:3] for row in t[:3]])
    jl_inv = o.inverse(o.so3_jacobian(theta, 1))
    rho: list[Any] = [sum(jl_inv[i][k] * t[k][3] for k in range(3)) for i in range(3)]
    return rho + list(theta)


def _inverse(t: o.Matrix) -> o.Matrix:
    """``[[Rᵀ, -Rᵀ t], [0, 1]]``."""
    rt = [[t[j][i] for j in range(3)] for i in range(3)]
    tr = [-sum(rt[i][k] * t[k][3] for k in range(3)) for i in range(3)]
    return [rt[i] + [tr[i]] for i in range(3)] + [[0, 0, 0, 1]]


def _adjoint(t: o.Matrix) -> o.Matrix:
    """``[[R, hat(t) R], [0, R]]``."""
    r = [row[:3] for row in t[:3]]
    tr = o.matmul(o.hat([t[0][3], t[1][3], t[2][3]]), r)
    out = [[mp.mpf(0)] * 6 for _ in range(6)]
    for i in range(3):
        for j in range(3):
            out[i][j] = r[i][j]
            out[3 + i][3 + j] = r[i][j]
            out[i][3 + j] = tr[i][j]
    return out


def _se3_fixture() -> dict[str, Any]:
    """Build the SE(3) fixture."""
    rng = np.random.default_rng(11)
    twists: dict[str, list[float]] = {}
    for name, angle in ANGLES.items():
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        twists[name] = [*rng.normal(size=3).tolist(), *(angle * axis).tolist()]
    twists["pure translation"] = [1.5, -0.25, 3.0, 0.0, 0.0, 0.0]
    transforms = {n: np.asarray(o.to_float(_exp(o.mpf_vector(x)))) for n, x in twists.items()}
    # Bottom rows from expm are exact zeros/ones in float; force them to be exactly so.
    for t in transforms.values():
        t[3] = [0.0, 0.0, 0.0, 1.0]
    mpm = o.mpf_matrix
    names = list(transforms)
    close = {"comparison": "close", "rtol": 0.0}
    fx: dict[str, Any] = {
        "exp": {
            "tolerance": {**close, "atol": 1e-14, "reason": "entries O(|rho|); a few roundings"},
            "cases": [o.case(n, x, o.to_float(_exp(o.mpf_vector(x)))) for n, x in twists.items()],
        },
        "log": {
            "tolerance": {
                **close,
                "atol": 1e-13,
                "reason": "rho = Jl(theta)^-1 t amplifies t's rounding near pi",
            },
            "cases": [
                o.case(n, t.tolist(), [float(v) for v in _log(mpm(t))])
                for n, t in transforms.items()
            ],
        },
        "inverse": {
            "tolerance": {**close, "atol": 1e-14, "reason": "transpose and one product"},
            "cases": [
                o.case(n, t.tolist(), o.to_float(_inverse(mpm(t)))) for n, t in transforms.items()
            ],
        },
        "adjoint": {
            "tolerance": {**close, "atol": 1e-14, "reason": "one 3x3 product"},
            "cases": [
                o.case(n, t.tolist(), o.to_float(_adjoint(mpm(t)))) for n, t in transforms.items()
            ],
        },
        "compose": {
            "unpack": True,
            "tolerance": {**close, "atol": 1e-14, "reason": "one 4x4 product"},
            "cases": [
                o.case(
                    f"{a} . {b}",
                    [transforms[a].tolist(), transforms[b].tolist()],
                    o.to_float(o.matmul(mpm(transforms[a]), mpm(transforms[b]))),
                )
                for a, b in zip(names, names[1:] + names[:1], strict=True)
            ],
        },
        "act": {
            "unpack": True,
            "tolerance": {**close, "atol": 1e-14, "reason": "one 3x3 product and a sum"},
            "cases": [
                o.case(
                    n,
                    [t.tolist(), p := rng.normal(size=3).tolist()],
                    [
                        float(
                            sum(mp.mpf(float(t[i][k])) * mp.mpf(p[k]) for k in range(3))
                            + mp.mpf(float(t[i][3]))
                        )
                        for i in range(3)
                    ],
                )
                for n, t in transforms.items()
            ],
        },
    }
    for name, sign in (("left_jacobian", 1), ("right_jacobian", -1)):
        fx[name] = {
            "tolerance": {
                **close,
                "atol": 1e-14,
                "reason": "Q from its series below theta = 1, closed form above",
            },
            "cases": [
                o.case(n, x, o.to_float(_left_jacobian(o.mpf_vector([sign * v for v in x]))))
                for n, x in twists.items()
            ],
        }
    for name, sign in (("left_jacobian_inverse", 1), ("right_jacobian_inverse", -1)):
        fx[name] = {
            "tolerance": {
                **close,
                "atol": 1e-13,
                "reason": "blockwise -J^-1 Q J^-1; entries grow with |rho|",
            },
            "cases": [
                o.case(
                    n, x, o.to_float(o.inverse(_left_jacobian(o.mpf_vector([sign * v for v in x]))))
                )
                for n, x in twists.items()
                if np.linalg.norm(x[3:]) < 2 * math.pi - 1e-6
            ],
        }
    return {
        "description": "SE(3): parity fixture from a 50-digit mpmath oracle (generate_se3.py).",
        "oracle": "mpmath expm; the series sum ad^n/(n+1)!; matrix inversion",
        "functions": fx,
    }


def _umeyama_mp(x: o.Matrix, y: o.Matrix, w: list[Any], with_scale: bool) -> o.Matrix:
    """Weighted Umeyama in high precision, by mpmath's SVD."""
    total = sum(w)
    wn = [v / total for v in w]
    mux = [sum(wn[i] * x[i][k] for i in range(len(x))) for k in range(3)]
    muy = [sum(wn[i] * y[i][k] for i in range(len(y))) for k in range(3)]
    cov = mp.zeros(3, 3)
    var = mp.mpf(0)
    for i in range(len(x)):
        xc = [x[i][k] - mux[k] for k in range(3)]
        yc = [y[i][k] - muy[k] for k in range(3)]
        var += wn[i] * sum(c * c for c in xc)
        for r in range(3):
            for c in range(3):
                cov[r, c] += wn[i] * yc[r] * xc[c]
    u, d, vt = mp.svd_r(cov)
    sign = 1 if mp.det(u) * mp.det(vt) > 0 else -1
    s_mat = mp.diag([1, 1, sign])
    rot = u * s_mat * vt
    scale = (d[0] + d[1] + sign * d[2]) / var if with_scale else mp.mpf(1)
    t = [muy[r] - scale * sum(rot[r, c] * mux[c] for c in range(3)) for r in range(3)]
    return [[scale * rot[r, c] for c in range(3)] + [t[r]] for r in range(3)] + [[0, 0, 0, 1]]


def _alignment_fixture() -> dict[str, Any]:
    """Build the alignment (Umeyama) fixture."""
    rng = np.random.default_rng(13)
    cases = []
    for name, with_scale, noise, planar in (
        ("rigid, exact", False, 0.0, False),
        ("similarity, exact", True, 0.0, False),
        ("similarity, planar points", True, 0.0, True),
        ("similarity, weighted and noisy", True, 1e-2, False),
    ):
        x = rng.normal(size=(30, 3))
        if planar:
            x[:, 2] = 0.0
        axis = rng.normal(size=3)
        rot = o.so3_exp(o.mpf_vector((1.3 * axis / np.linalg.norm(axis)).tolist()))
        scale = mp.mpf("2.5") if with_scale else mp.mpf(1)
        t = o.mpf_vector(rng.normal(size=3).tolist())
        xm = o.mpf_matrix(x)
        y = [
            [scale * sum(rot[r][k] * xm[i][k] for k in range(3)) + t[r] for r in range(3)]
            for i in range(len(x))
        ]
        y_float = np.asarray(o.to_float(y)) + noise * rng.normal(size=(30, 3))
        w = rng.uniform(0.2, 1.0, size=30) if noise else np.ones(30)
        expected = _umeyama_mp(xm, o.mpf_matrix(y_float), o.mpf_vector(w.tolist()), with_scale)
        cases.append(
            o.case(
                name,
                {
                    "source": x.tolist(),
                    "target": y_float.tolist(),
                    "weights": w.tolist(),
                    "with_scale": with_scale,
                },
                o.to_float(expected),
            )
        )
    return {
        "description": "Umeyama alignment: parity fixture from mpmath (generate_se3.py).",
        "oracle": "exact synthetic transforms; mpmath SVD for the noisy case",
        "functions": {
            "umeyama": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-13,
                    "reason": "an SVD of a well-conditioned covariance",
                },
                "cases": cases,
            }
        },
    }


def main() -> None:
    """Write ``se3.json`` and ``alignment.json``."""
    for name, doc in (("se3.json", _se3_fixture()), ("alignment.json", _alignment_fixture())):
        (HERE / name).write_text(json.dumps(doc, indent=1) + "\n")
        print(f"wrote {name} ({sum(len(f['cases']) for f in doc['functions'].values())} cases)")


if __name__ == "__main__":
    with mp.workdps(o.PRECISION):
        main()
