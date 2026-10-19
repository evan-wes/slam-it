"""Generate ``epipolar.json``, ``fivepoint.json``, ``homography.json`` and ``twoview.json``.

Run from the repository root after changing the cases (the fixtures are committed)::

    python -m tests.fixtures.foundations.generate_twoview

Independent methods, nothing imported from slam_it:

- noise-free cases expect the true matrix, built in 50 digits from the pose (and plane):
  ``E = [t̂]ₓ R``, ``F = K⁻ᵀ E K⁻¹``, ``H = R + t nᵀ / d``, each scaled as the solvers scale
  them;
- noisy cases expect each solver's defining solution, computed in 50 digits: the smallest
  eigenvector of ``AᵀA`` for the system the docstring states (on Hartley-normalised points),
  then mpmath's SVD for the projection onto the essential manifold or rank 2;
- Sampson errors are their closed forms in 50 digits;
- RANSAC runs use noise-free inliers and outliers far from any model, so the expected model
  is the true one and the expected inliers are exactly the clean correspondences, for every
  minimal solver, refined or not;
- the 5-point solvers on exact samples must return the true E among their solutions;
- the Sampson refinement on exact pairs, from a perturbed start, must reach the true E (the
  cost's minimum, zero).

Everything is computed in 50 digits (mpmath) and rounded once.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np

from tests.fixtures.foundations import _mp_oracle as o

HERE = Path(__file__).parent
K = [[520.0, 0.0, 320.0], [0.0, 515.0, 240.0], [0.0, 0.0, 1.0]]
"""The pinhole intrinsics of the fundamental-matrix cases."""


def _pose(omega: list[float], t: list[float]) -> tuple[mp.matrix, mp.matrix]:
    """Return a 50-digit rotation ``exp(ω)`` and translation."""
    return mp.matrix(o.so3_exp(o.mpf_vector(omega))), mp.matrix(o.mpf_vector(t))


def _unit(v: mp.matrix) -> list[float]:
    """Round a 50-digit vector's direction to float64."""
    n = mp.norm(v)
    return [float(x / n) for x in v]


def _scene(rng: np.random.Generator, n: int) -> list[mp.matrix]:
    """Draw points 4-8 m ahead, as 50-digit vectors."""
    pts = rng.uniform([-2.0, -2.0, 4.0], [2.0, 2.0, 8.0], size=(n, 3))
    return [mp.matrix(p.tolist()) for p in pts]


def _jitter(f: list[float], rng: np.random.Generator, sigma: float) -> list[float]:
    """Perturb a bearing's normalised image point by Gaussian noise, then renormalise."""
    x = [f[0] / f[2] + float(rng.normal(scale=sigma)), f[1] / f[2] + float(rng.normal(scale=sigma))]
    return _unit(mp.matrix([x[0], x[1], 1.0]))


def _float(m: mp.matrix) -> list[list[float]]:
    """Round a 50-digit 3x3 matrix to float64 lists."""
    return [[float(m[i, j]) for j in range(3)] for i in range(3)]


def _smallest(system: mp.matrix) -> mp.matrix:
    """Return the smallest eigenvector of ``AᵀA`` as a 3x3 matrix (row-major vec)."""
    values, vectors = mp.eigsy(system.T * system)
    k = min(range(values.rows), key=lambda i: values[i])
    v = vectors[:, k]
    return mp.matrix([[v[3 * i + j] for j in range(3)] for i in range(3)])


def _epipolar_rows(x1: list[mp.matrix], x2: list[mp.matrix]) -> mp.matrix:
    """Return the 8-point system: one ``x₂ ⊗ x₁`` row per correspondence."""
    return mp.matrix(
        [[a[i] * b[j] for i in range(3) for j in range(3)] for b, a in zip(x1, x2, strict=True)]
    )


def _hartley(points: list[list[float]]) -> mp.matrix:
    """Return Hartley's normalising similarity in 50 digits."""
    pts = [mp.matrix(p) for p in points]
    c = sum(pts[1:], pts[0]) / len(pts)
    spread = sum(mp.norm(p - c) for p in pts) / len(pts)
    s = mp.sqrt(2) / spread
    return mp.matrix([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]])


def _homogeneous(t: mp.matrix, points: list[list[float]]) -> list[mp.matrix]:
    """Apply a similarity to 2D points, as homogeneous 50-digit vectors."""
    return [t * mp.matrix([p[0], p[1], 1]) for p in points]


def _essential_8pt(b1: list[list[float]], b2: list[list[float]]) -> mp.matrix:
    """Solve the normalised 8-point E by its definition, then project it (essential manifold)."""
    p1 = [[mp.mpf(f[0]) / f[2], mp.mpf(f[1]) / f[2]] for f in b1]
    p2 = [[mp.mpf(f[0]) / f[2], mp.mpf(f[1]) / f[2]] for f in b2]
    t1, t2 = _hartley(p1), _hartley(p2)
    raw = _smallest(_epipolar_rows(_homogeneous(t1, p1), _homogeneous(t2, p2)))
    u, _, v = mp.svd_r(t2.T * raw * t1)
    return u * mp.diag([1, 1, 0]) * v


def _fundamental_8pt(p1: list[list[float]], p2: list[list[float]]) -> mp.matrix:
    """Solve the normalised 8-point F by its definition: rank 2, unit Frobenius norm."""
    t1, t2 = _hartley(p1), _hartley(p2)
    raw = _smallest(_epipolar_rows(_homogeneous(t1, p1), _homogeneous(t2, p2)))
    u, s, v = mp.svd_r(raw)
    f = t2.T * (u * mp.diag([s[0], s[1], 0]) * v) * t1
    return f / mp.mnorm(f, "f")


def _homography_dlt(p1: list[list[float]], p2: list[list[float]]) -> mp.matrix:
    """Solve the normalised DLT by its definition: unit Frobenius norm."""
    t1, t2 = _hartley(p1), _hartley(p2)
    x1, x2 = _homogeneous(t1, p1), _homogeneous(t2, p2)
    rows = []
    for p, q in zip(x1, x2, strict=True):
        a = p / p[2]
        b = q / q[2]
        rows.append([0, 0, 0, -a[0], -a[1], -a[2], b[1] * a[0], b[1] * a[1], b[1] * a[2]])
        rows.append([a[0], a[1], a[2], 0, 0, 0, -b[0] * a[0], -b[0] * a[1], -b[0] * a[2]])
    h = mp.inverse(t2) * _smallest(mp.matrix(rows)) * t1
    return h / mp.mnorm(h, "f")


def _sampson_epipolar(m: mp.matrix, x1: list[float], x2: list[float]) -> float:
    """Return the squared Sampson error of homogeneous points (z = 1) under E or F."""
    a, b = mp.matrix(x1), mp.matrix(x2)
    l2, l1 = m * a, m.T * b
    e = (b.T * l2)[0]
    return float(e**2 / (l2[0] ** 2 + l2[1] ** 2 + l1[0] ** 2 + l1[1] ** 2))


def _sampson_homography(h: mp.matrix, p1: list[float], p2: list[float]) -> float:
    """Return the squared Sampson error ``εᵀ (J Jᵀ)⁻¹ ε`` under H."""
    u, v, w = h * mp.matrix([p1[0], p1[1], 1])
    x2, y2 = p2
    eps = mp.matrix([y2 * w - v, u - x2 * w])
    j = mp.matrix(
        [
            [y2 * h[2, 0] - h[1, 0], y2 * h[2, 1] - h[1, 1], 0, w],
            [h[0, 0] - x2 * h[2, 0], h[0, 1] - x2 * h[2, 1], -w, 0],
        ]
    )
    return float((eps.T * mp.inverse(j * j.T) * eps)[0])


def _bearing_pairs(
    rng: np.random.Generator, n: int, rot: mp.matrix, t: mp.matrix, sigma: float
) -> tuple[list[list[float]], list[list[float]]]:
    """Return bearing pairs of a random scene through ``[R | t]``, optionally noisy."""
    b1, b2 = [], []
    for x in _scene(rng, n):
        f1, f2 = _unit(x), _unit(rot * x + t)
        if sigma > 0.0:
            f1, f2 = _jitter(f1, rng, sigma), _jitter(f2, rng, sigma)
        b1.append(f1)
        b2.append(f2)
    return b1, b2


def _pose_list(rot: mp.matrix, t: mp.matrix) -> list[list[float]]:
    """Round a 50-digit ``[R | t]`` to a float (4, 4) list."""
    rows = [[float(rot[i, j]) for j in range(3)] + [float(t[i])] for i in range(3)]
    return [*rows, [0.0, 0.0, 0.0, 1.0]]


def _essential_candidates(rot: mp.matrix, t: mp.matrix) -> list[list[list[float]]]:
    """Return E's four poses in closed form: ``(R, ±t̂)`` and the twisted pair ``(R', ±t̂)``.

    The twisted pair turns R by half a turn about the baseline: ``R' = (2 t̂ t̂ᵀ - I) R``
    (Hartley and Zisserman, section 9.6.3).
    """
    unit = t / mp.norm(t)
    twisted = (2 * unit * unit.T - mp.eye(3)) * rot
    return [_pose_list(r, sign * unit) for r in (rot, twisted) for sign in (1, -1)]


def _true_essential(rot: mp.matrix, t: mp.matrix) -> mp.matrix:
    """Return ``[t̂]ₓ R``: singular values (1, 1, 0)."""
    return mp.matrix(o.hat(list(t / mp.norm(t)))) * rot


def _pixels(bearings: list[list[float]]) -> list[list[float]]:
    """Project bearings through K (rounded)."""
    k = mp.matrix(K)
    return [
        [float(c) for c in (k * mp.matrix([f[0] / f[2], f[1] / f[2], 1]))[:2]] for f in bearings
    ]


def _with_outliers(
    rng: np.random.Generator, b2: list[list[float]], fraction: float
) -> tuple[list[list[float]], list[bool]]:
    """Replace a fraction of camera-2 bearings by random directions; return the inlier mask."""
    out, inliers = [], []
    for f in b2:
        if rng.random() < fraction:
            g = rng.uniform(-0.6, 0.6, size=2)
            out.append(_unit(mp.matrix([float(g[0]), float(g[1]), 1.0])))
            inliers.append(False)
        else:
            out.append(f)
            inliers.append(True)
    return out, inliers


def _epipolar_fixture() -> dict[str, Any]:
    """Build the essential- and fundamental-matrix cases."""
    rng = np.random.default_rng(11)
    rot, t = _pose([0.02, -0.1, 0.03], [0.5, -0.1, 0.05])
    fwd_rot, fwd_t = _pose([0.01, 0.02, 0.0], [0.02, 0.01, 0.6])
    e_true = _true_essential(rot, t)
    clean1, clean2 = _bearing_pairs(rng, 20, rot, t, 0.0)
    eight1, eight2 = clean1[:8], clean2[:8]
    noisy1, noisy2 = _bearing_pairs(rng, 50, rot, t, 1e-3)
    fwd1, fwd2 = _bearing_pairs(rng, 20, fwd_rot, fwd_t, 0.0)
    essential = [
        o.case("noise-free, 20 pairs", {"bearings1": clean1, "bearings2": clean2}, _float(e_true)),
        o.case("noise-free, exactly 8", {"bearings1": eight1, "bearings2": eight2}, _float(e_true)),
        o.case(
            "forward motion",
            {"bearings1": fwd1, "bearings2": fwd2},
            _float(_true_essential(fwd_rot, fwd_t)),
        ),
        o.case(
            "noisy 1e-3, 50 pairs",
            {"bearings1": noisy1, "bearings2": noisy2},
            _float(_essential_8pt(noisy1, noisy2)),
        ),
    ]
    k_inv = mp.inverse(mp.matrix(K))
    f_true = k_inv.T * e_true * k_inv
    f_true = f_true / mp.mnorm(f_true, "f")
    px1, px2 = _pixels(clean1), _pixels(clean2)
    noisy_px1 = [
        [u + float(rng.normal(scale=0.5)), v + float(rng.normal(scale=0.5))]
        for u, v in _pixels(noisy1)
    ]
    noisy_px2 = [
        [u + float(rng.normal(scale=0.5)), v + float(rng.normal(scale=0.5))]
        for u, v in _pixels(noisy2)
    ]
    fundamental = [
        o.case("noise-free, 20 pairs", {"pixels1": px1, "pixels2": px2}, _float(f_true)),
        o.case(
            "noisy 0.5 px, 50 pairs",
            {"pixels1": noisy_px1, "pixels2": noisy_px2},
            _float(_fundamental_8pt(noisy_px1, noisy_px2)),
        ),
    ]
    sampson_e = [
        o.case(
            "true E, noisy pairs",
            {"essential": _float(e_true), "bearings1": noisy1, "bearings2": noisy2},
            [
                _sampson_epipolar(
                    mp.matrix(_float(e_true)),
                    [a[0] / a[2], a[1] / a[2], 1],
                    [b[0] / b[2], b[1] / b[2], 1],
                )
                for a, b in zip(noisy1, noisy2, strict=True)
            ],
        )
    ]
    f_float = _float(f_true)
    sampson_f = [
        o.case(
            "true F, noisy pixels",
            {"fundamental": f_float, "pixels1": noisy_px1, "pixels2": noisy_px2},
            [
                _sampson_epipolar(mp.matrix(f_float), [a[0], a[1], 1], [b[0], b[1], 1])
                for a, b in zip(noisy_px1, noisy_px2, strict=True)
            ],
        )
    ]
    run1, run2 = _bearing_pairs(rng, 120, rot, t, 0.0)
    run2, mask = _with_outliers(rng, run2, 0.3)
    ransac = [
        o.case(
            f"30% outliers, noise-free inliers, {solver}{', refined' if refine else ''}",
            {
                "bearings1": run1,
                "bearings2": run2,
                "threshold": 1e-4,
                "seed": 5,
                "minimal_solver": solver,
                "refine": refine,
            },
            {"model": _float(e_true), "inliers": mask},
            summary={"refinement_ran": refine},
        )
        for solver, refine in (("stewenius", True), ("nister", True), ("8pt", False))
    ]
    nudged = mp.matrix(o.so3_exp(o.mpf_vector([0.0, 0.02, 0.0]))) * (t / mp.norm(t))
    start = mp.matrix(o.hat(list(nudged))) * mp.matrix(o.so3_exp(o.mpf_vector([0.01, 0.0, 0.0])))
    refine = [
        o.case(
            "noise-free, 20 pairs, a perturbed start",
            {
                "essential": _float(start * rot),
                "bearings1": clean1,
                "bearings2": clean2,
                "scale": 1e-3,
            },
            _float(e_true),
            summary={"converged": True},
        )
    ]
    tol: dict[str, Any] = {"comparison": "close", "rtol": 0.0, "double_cover": True}
    return {
        "description": "Epipolar geometry: parity fixture from mpmath (generate_twoview.py).",
        "oracle": "true matrices; 50-digit definitions of the 8-point solvers and Sampson errors",
        "functions": {
            "essential_8pt": {
                "tolerance": {
                    **tol,
                    "atol": 1e-10,
                    "reason": "an SVD of the 8-point system; up to sign",
                },
                "cases": essential,
            },
            "fundamental_8pt": {
                "tolerance": {
                    **tol,
                    "atol": 1e-10,
                    "reason": "normalised 8-point and rank-2 SVD; up to sign",
                },
                "cases": fundamental,
            },
            "sampson_essential": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 1e-10,
                    "atol": 0.0,
                    "reason": "a closed form; x2^T E x1 cancels for near-inliers",
                },
                "cases": sampson_e,
            },
            "sampson_fundamental": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 1e-10,
                    "atol": 0.0,
                    "reason": "a closed form in pixels",
                },
                "cases": sampson_f,
            },
            "decompose_essential": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-12,
                    "reason": "an SVD of E; compared as an unordered set of four poses",
                },
                "cases": [
                    o.case(
                        "sideways motion",
                        {"essential": _float(e_true)},
                        _essential_candidates(rot, t),
                    ),
                    o.case(
                        "forward motion",
                        {"essential": _float(_true_essential(fwd_rot, fwd_t))},
                        _essential_candidates(fwd_rot, fwd_t),
                    ),
                ],
            },
            "essential_ransac": {
                "tolerance": {
                    **tol,
                    "atol": 1e-10,
                    "reason": "noise-free inliers: the minimal solvers and LO's 8-point refits "
                    "are exact to rounding, and the refinement starts at the cost's minimum",
                },
                "cases": ransac,
            },
            "refine_essential": {
                "tolerance": {
                    **tol,
                    "atol": 1e-8,
                    "reason": "exact pairs: the Sampson cost's minimum is the true E; LM converges "
                    "to it from a 1-degree perturbation and stops on its function tolerance, "
                    "measured 2.5e-9 away",
                },
                "cases": refine,
            },
        },
    }


def _fivepoint_fixture() -> dict[str, Any]:
    """Build the 5-point cases: exact five-pair samples, the true E among the solutions."""
    rng = np.random.default_rng(17)
    motions = {
        "sideways": _pose([0.02, -0.1, 0.03], [0.5, -0.1, 0.05]),
        "forward": _pose([0.01, 0.02, 0.0], [0.02, 0.01, 0.6]),
        "oblique": _pose([-0.1, 0.05, 0.2], [0.3, 0.4, -0.2]),
    }
    samples = []
    for name, (rot, t) in motions.items():
        samples.append((name, rot, t, _bearing_pairs(rng, 5, rot, t, 0.0)))
    # A 5 cm forward baseline: samples whose unpolished solutions miss the true E by 6e-8 to
    # 3e-3 (both solvers), so a port without the converged polish fails the tolerance.
    near_rot, near_t = _pose([0.01, -0.015, 0.005], [0.004, -0.003, 0.05])
    for seed in (1, 15, 20):
        pairs = _bearing_pairs(np.random.default_rng(seed), 5, near_rot, near_t, 0.0)
        samples.append((f"forward, 5 cm baseline, sample {seed}", near_rot, near_t, pairs))
    cases = []
    for name, rot, t, (b1, b2) in samples:
        for solver in ("stewenius", "nister"):
            cases.append(
                o.case(
                    f"{name}, {solver}",
                    {"bearings1": b1, "bearings2": b2, "solver": solver},
                    [_float(_true_essential(rot, t))],
                )
            )
    return {
        "description": "The 5-point solvers: parity fixture from mpmath (generate_twoview.py).",
        "oracle": "the true E of exact samples, built in 50 digits from the pose",
        "functions": {
            "essential_5pt": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-11,
                    "double_cover": True,
                    "reason": "the expected E must be among the solutions, up to sign; the "
                    "solutions are polished to convergence on the cubic constraints (worst "
                    "measured 1.6e-12 here, 3.8e-12 on 300 generic samples); unpolished, the "
                    "near-degenerate samples miss by 6e-8 to 3e-3",
                },
                "cases": cases,
            }
        },
    }


def _homography_fixture() -> dict[str, Any]:
    """Build the homography cases: a plane seen from two calibrated views."""
    rng = np.random.default_rng(12)
    rot, t = _pose([0.05, -0.08, 0.02], [0.4, 0.1, -0.05])
    normal = mp.matrix(_unit(mp.matrix([0.1, -0.2, 1.0])))
    depth = mp.mpf(5)
    h_true = rot + t * normal.T / depth
    h_true = h_true / mp.mnorm(h_true, "f")

    def plane_pairs(n: int, sigma: float) -> tuple[list[list[float]], list[list[float]]]:
        p1, p2 = [], []
        for xy in rng.uniform(-0.5, 0.5, size=(n, 2)):
            ray = mp.matrix([float(xy[0]), float(xy[1]), 1.0])
            x = ray * depth / (normal.T * ray)[0]  # on nᵀ X = d
            y = rot * x + t
            a = [float(x[0] / x[2]), float(x[1] / x[2])]
            b = [float(y[0] / y[2]), float(y[1] / y[2])]
            if sigma > 0.0:
                a = [a[0] + float(rng.normal(scale=sigma)), a[1] + float(rng.normal(scale=sigma))]
                b = [b[0] + float(rng.normal(scale=sigma)), b[1] + float(rng.normal(scale=sigma))]
            p1.append(a)
            p2.append(b)
        return p1, p2

    clean1, clean2 = plane_pairs(20, 0.0)
    noisy1, noisy2 = plane_pairs(40, 1e-3)
    run1, run2 = plane_pairs(100, 0.0)
    mask = []
    for i in range(len(run2)):
        if rng.random() < 0.4:
            run2[i] = [float(c) for c in rng.uniform(-0.6, 0.6, size=2)]
            mask.append(False)
        else:
            mask.append(True)
    h_float = _float(h_true)
    tol: dict[str, Any] = {"comparison": "close", "rtol": 0.0, "double_cover": True}
    return {
        "description": "Homographies: parity fixture from mpmath (generate_twoview.py).",
        "oracle": "true plane homography; the normalised DLT and Sampson error in 50 digits",
        "functions": {
            "homography_dlt": {
                "tolerance": {
                    **tol,
                    "atol": 1e-10,
                    "reason": "an SVD of the DLT system; up to sign",
                },
                "cases": [
                    o.case(
                        "noise-free plane, 20 pairs",
                        {"points1": clean1, "points2": clean2},
                        h_float,
                    ),
                    o.case(
                        "noise-free, exactly 4",
                        {"points1": clean1[:4], "points2": clean2[:4]},
                        h_float,
                    ),
                    o.case(
                        "noisy 1e-3, 40 pairs",
                        {"points1": noisy1, "points2": noisy2},
                        _float(_homography_dlt(noisy1, noisy2)),
                    ),
                ],
            },
            "sampson_homography": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 1e-10,
                    "atol": 0.0,
                    "reason": "a closed form",
                },
                "cases": [
                    o.case(
                        "true H, noisy pairs",
                        {"homography": h_float, "points1": noisy1, "points2": noisy2},
                        [
                            _sampson_homography(mp.matrix(h_float), a, b)
                            for a, b in zip(noisy1, noisy2, strict=True)
                        ],
                    )
                ],
            },
            "decompose_homography": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-10,
                    "reason": "Malis-Vargas closed form; the expected candidates must be among "
                    "those returned",
                },
                "cases": [
                    o.case(
                        "plane, scaled and negated H",
                        {"homography": _float(-2.5 * (rot + t * normal.T / depth))},
                        {
                            "contains_poses": [
                                _pose_list(rot, t / depth),
                                _pose_list(rot, -t / depth),
                            ],
                            "contains_normals": [
                                [float(c) for c in normal],
                                [-float(c) for c in normal],
                            ],
                        },
                    ),
                    o.case(
                        "pure rotation",
                        {"homography": _float(rot)},
                        {
                            "contains_poses": [_pose_list(rot, mp.matrix([0, 0, 0]))],
                            "contains_normals": [[0.0, 0.0, 0.0]],
                        },
                    ),
                ],
            },
            "homography_ransac": {
                "tolerance": {
                    **tol,
                    "atol": 1e-10,
                    "reason": "LO refits all clean inliers exactly",
                },
                "cases": [
                    o.case(
                        "40% outliers, noise-free inliers",
                        {"points1": run1, "points2": run2, "threshold": 1e-4, "seed": 6},
                        {"model": h_float, "inliers": mask},
                    )
                ],
            },
        },
    }


OUTLIER_MARGIN = 10 * 2.0 / 520.0
"""Planted outliers lie at least ten 2 px thresholds (at f = 520) from the true model, so no
estimator could call them inliers."""


def _true_distance(motion: tuple[mp.matrix, mp.matrix], f1: list[float], f2: list[float]) -> float:
    """Return a match's distance to the true model in the normalised plane.

    The Sampson distance under the true E, or the transfer distance under R for a rotation.
    """
    rot, t = motion
    a = mp.matrix([f1[0] / f1[2], f1[1] / f1[2], 1])
    b = mp.matrix([f2[0] / f2[2], f2[1] / f2[2], 1])
    if mp.norm(t) == 0:
        y = rot * a
        return float(mp.norm(mp.matrix([b[0] - y[0] / y[2], b[1] - y[1] / y[2]])))
    return float(mp.sqrt(_sampson_epipolar(_true_essential(rot, t), list(a), list(b))))


def _far_outliers(
    rng: np.random.Generator,
    b1: list[list[float]],
    b2: list[list[float]],
    motion: tuple[mp.matrix, mp.matrix],
    fraction: float,
) -> tuple[list[list[float]], list[bool]]:
    """Replace a fraction of camera-2 bearings by random directions far from the true model."""
    out, inliers = [], []
    for f1, f2 in zip(b1, b2, strict=True):
        if rng.random() >= fraction:
            out.append(f2)
            inliers.append(True)
            continue
        while True:
            g = rng.uniform(-0.6, 0.6, size=2)
            candidate = _unit(mp.matrix([float(g[0]), float(g[1]), 1.0]))
            if _true_distance(motion, f1, candidate) > OUTLIER_MARGIN:
                break
        out.append(candidate)
        inliers.append(False)
    return out, inliers


def _initialize_case(
    name: str,
    points: list[mp.matrix],
    motion: tuple[mp.matrix, mp.matrix],
    *,
    expected: tuple[str, str],
    seed: int,
) -> dict[str, object]:
    """Build one ``initialize`` case from exact bearings, with a third replaced by outliers.

    The expected motion is the true one with unit translation; the expected points are the
    true ones over the baseline's length, for the clean matches with at least one degree of
    parallax (none for a pure rotation).
    """
    rng = np.random.default_rng(seed)
    rot, t = motion
    b1 = [_unit(x) for x in points]
    b2 = [_unit(rot * x + t) for x in points]
    b2, clean = _far_outliers(rng, b1, b2, motion, 1 / 3)
    baseline = mp.norm(t)
    centre2 = -(rot.T * t)
    unit = t / baseline if baseline > 0 else t
    rows, kept = [], []
    for x, ok in zip(points, clean, strict=True):
        if baseline == 0 or not ok:
            kept.append(False)
            continue
        parallax = mp.degrees(
            mp.acos((x.T * (x - centre2))[0] / (mp.norm(x) * mp.norm(x - centre2)))
        )
        kept.append(bool(parallax >= 1))
        if kept[-1]:
            rows.append([float(c / baseline) for c in x])
    return o.case(
        name,
        {"bearings1": b1, "bearings2": b2, "focal": 520.0, "seed": seed},
        {
            "status": expected[0],
            "model": expected[1],
            "relative_pose": _pose_list(rot, unit),
            "inliers": clean,
            "triangulated": kept,
            "points": rows,
        },
    )


def _initialize_fixture() -> dict[str, Any]:
    """Build the two-view initialisation cases: exact matches, a third of them outliers."""
    rng = np.random.default_rng(13)
    general = _scene(rng, 150)
    planar = []
    for xy in rng.uniform(-2.0, 2.0, size=(150, 2)):
        planar.append(mp.matrix([float(xy[0]), float(xy[1]), 5.0]))
    sideways = _pose([0.02, -0.1, 0.03], [0.5, -0.1, 0.05])
    forward = _pose([0.01, 0.02, 0.0], [0.02, 0.01, 0.6])
    rotation = _pose([0.05, -0.1, 0.02], [0.0, 0.0, 0.0])
    return {
        "description": "Two-view initialisation: ground truth fixture (generate_twoview.py).",
        "oracle": "the true motion and points of exact matches; a third replaced by outliers",
        "functions": {
            "initialize": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-9,
                    "reason": "exact inliers: the 5-point solver, the 8-point refits, the "
                    "Sampson refinement and Lindstrom are exact to rounding; depths up to 8 "
                    "over a baseline of 0.5 amplify it",
                },
                "cases": [
                    _initialize_case(
                        "general scene, sideways",
                        general,
                        sideways,
                        expected=("initialized", "essential"),
                        seed=21,
                    ),
                    _initialize_case(
                        "general scene, forward",
                        general,
                        forward,
                        expected=("initialized", "essential"),
                        seed=22,
                    ),
                    _initialize_case(
                        "plane: E degenerate, H chosen",
                        planar,
                        sideways,
                        expected=("initialized", "homography"),
                        seed=23,
                    ),
                    _initialize_case(
                        "pure rotation",
                        general,
                        rotation,
                        expected=("pure_rotation", "homography"),
                        seed=24,
                    ),
                ],
            }
        },
    }


def main() -> None:
    """Write ``epipolar.json``, ``fivepoint.json``, ``homography.json`` and ``twoview.json``."""
    for name, doc in (
        ("epipolar.json", _epipolar_fixture()),
        ("fivepoint.json", _fivepoint_fixture()),
        ("homography.json", _homography_fixture()),
        ("twoview.json", _initialize_fixture()),
    ):
        (HERE / name).write_text(json.dumps(doc, indent=1) + "\n")
        functions: dict[str, dict[str, Any]] = doc["functions"]
        count = sum(len(f["cases"]) for f in functions.values())
        print(f"wrote {name} ({count} cases)")


if __name__ == "__main__":
    with mp.workdps(o.PRECISION):
        main()
