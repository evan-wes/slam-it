"""Generate ``triangulation.json`` from a 50-digit mpmath oracle.

Run from the repository root after changing the cases (the fixtures are committed)::

    python -m tests.fixtures.foundations.generate_triangulation

Independent methods, nothing imported from slam_it_all:

- ``two_view``: noise-free cases expect the true points. Noisy cases expect the minimiser of
  the squared reprojection error in both normalised image planes (the quantity Lindstrom's
  correction minimises), found by Gauss-Newton in 50 digits from the true point, not by any
  closed form.
- ``n_view``: noise-free cases expect the true points. Noisy cases expect the DLT's defining
  solution, the smallest eigenvector of ``AᵀA`` for the normalised system the DLT states,
  computed by mpmath's symmetric eigensolver.
- Cheirality and parallax are evaluated in 50 digits at the expected point.
"""

from __future__ import annotations

import json
from pathlib import Path

import mpmath as mp
import numpy as np
from numpy.typing import NDArray

from tests.fixtures.foundations import _mp_oracle as o

HERE = Path(__file__).parent
GAUSS_NEWTON_STEPS = 40
"""Gauss-Newton steps for the noisy two-view optimum; the residuals are about 1e-3, so each
step gains about three digits and 40 reach far below 1e-50."""


def _pose(omega: list[float], t: list[float]) -> NDArray[np.float64]:
    """Build a float (4, 4) ``T = [exp(ω) | t]``, the rotation rounded from 50 digits."""
    pose = np.eye(4)
    pose[:3, :3] = np.asarray(o.to_float(o.so3_exp(o.mpf_vector(omega))))
    pose[:3, 3] = t
    return pose


def _mp_pose(pose: NDArray[np.float64]) -> tuple[mp.matrix, mp.matrix]:
    """Return the rotation and translation of a float pose as mpmath matrices."""
    return mp.matrix(pose[:3, :3].tolist()), mp.matrix(pose[:3, 3].tolist())


def _bearing(point: mp.matrix) -> list[float]:
    """Round a 50-digit point's unit direction to float64."""
    return [float(x) for x in point / mp.norm(point)]


def _noisy_bearing(point: mp.matrix, rng: np.random.Generator, sigma: float) -> list[float]:
    """Perturb a point's normalised image coordinates by Gaussian noise, then normalise."""
    x = [float(point[0] / point[2]), float(point[1] / point[2]), 1.0]
    x[0] += float(rng.normal(scale=sigma))
    x[1] += float(rng.normal(scale=sigma))
    return _bearing(mp.matrix(x))


def _angle(a: mp.matrix, b: mp.matrix) -> mp.matrix:
    """Return the angle between two mpmath vectors."""
    cross = mp.matrix(
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    )
    return mp.atan2(mp.norm(cross), (a.T * b)[0])


def _row(point: mp.matrix, views: list[tuple[mp.matrix, mp.matrix, list[float]]]) -> list[float]:
    """Pack the expectation: point, cheirality over the views, largest pairwise parallax."""
    in_front = all(((rot * point + t).T * mp.matrix(f))[0] > 0 for rot, t, f in views)
    centres = [-(rot.T * t) for rot, t, _ in views]
    parallax = max(
        _angle(point - centres[i], point - centres[j])
        for i in range(len(centres))
        for j in range(i + 1, len(centres))
    )
    return [
        float(point[0]),
        float(point[1]),
        float(point[2]),
        1.0 if in_front else 0.0,
        float(parallax),
    ]


def _optimum(
    rot: mp.matrix, t: mp.matrix, f1: list[float], f2: list[float], start: mp.matrix
) -> mp.matrix:
    """Minimise the squared reprojection error in both normalised planes (Gauss-Newton)."""
    x1 = [mp.mpf(f1[0]) / f1[2], mp.mpf(f1[1]) / f1[2]]
    x2 = [mp.mpf(f2[0]) / f2[2], mp.mpf(f2[1]) / f2[2]]
    point = start
    for _ in range(GAUSS_NEWTON_STEPS):
        q = rot * point + t
        residual = mp.matrix(
            [
                point[0] / point[2] - x1[0],
                point[1] / point[2] - x1[1],
                q[0] / q[2] - x2[0],
                q[1] / q[2] - x2[1],
            ]
        )
        j1 = mp.matrix(
            [
                [1 / point[2], 0, -point[0] / point[2] ** 2],
                [0, 1 / point[2], -point[1] / point[2] ** 2],
            ]
        )
        j2 = mp.matrix([[1 / q[2], 0, -q[0] / q[2] ** 2], [0, 1 / q[2], -q[1] / q[2] ** 2]]) * rot
        jac = mp.matrix(4, 3)
        for i in range(2):
            for k in range(3):
                jac[i, k] = j1[i, k]
                jac[i + 2, k] = j2[i, k]
        point = point - mp.lu_solve(jac.T * jac, jac.T * residual)
    return point


def _midpoint(rot: mp.matrix, t: mp.matrix, f1: list[float], f2: list[float]) -> mp.matrix:
    """Return the rays' closest approach (a start for Gauss-Newton), in 50 digits."""
    a = rot * mp.matrix(f1)
    b = mp.matrix(f2)
    # Minimise |d1 a + t - d2 b|² over (d1, d2).
    m = mp.matrix([[(a.T * a)[0], -(a.T * b)[0]], [-(a.T * b)[0], (b.T * b)[0]]])
    rhs = mp.matrix([-(a.T * t)[0], (b.T * t)[0]])
    d = mp.lu_solve(m, rhs)
    return d[0] * mp.matrix(f1)


def _two_view_case(
    name: str,
    pose: NDArray[np.float64],
    points: NDArray[np.float64],
    *,
    sigma: float,
    seed: int,
    given_pose: NDArray[np.float64] | None = None,
) -> dict[str, object]:
    """Build one ``two_view`` case: bearings from `pose`, triangulated through `given_pose`."""
    rng = np.random.default_rng(seed)
    rot, t = _mp_pose(pose)
    used = pose if given_pose is None else given_pose
    rot_u, t_u = _mp_pose(used)
    b1, b2, rows = [], [], []
    for p in points:
        x = mp.matrix(p.tolist())
        f1 = _bearing(x) if sigma == 0.0 else _noisy_bearing(x, rng, sigma)
        f2 = _bearing(rot * x + t) if sigma == 0.0 else _noisy_bearing(rot * x + t, rng, sigma)
        best = (
            x
            if sigma == 0.0 and given_pose is None
            else _optimum(rot_u, t_u, f1, f2, _midpoint(rot_u, t_u, f1, f2))
        )
        identity = (mp.eye(3), mp.matrix([0, 0, 0]))
        rows.append(_row(best, [(*identity, f1), (rot_u, t_u, f2)]))
        b1.append(f1)
        b2.append(f2)
    inputs = {"relative_pose": used.tolist(), "bearings1": b1, "bearings2": b2}
    return o.case(name, inputs, rows)


def _dlt(poses: list[NDArray[np.float64]], rays: list[list[float]]) -> mp.matrix:
    """Solve the normalised DLT by its definition, the smallest eigenvector of AᵀA."""
    mats = [_mp_pose(p) for p in poses]
    centres = [-(r.T * t) for r, t in mats]
    mean = sum(centres[1:], centres[0]) / len(centres)
    scale = sum(mp.norm(c - mean) for c in centres) / len(centres)
    rows = []
    for (rot, t), f in zip(mats, rays, strict=True):
        skew = mp.matrix(o.hat(o.mpf_vector(f)))
        t_norm = (t + rot * mean) / scale
        left, right = skew * rot, skew * t_norm
        rows.extend([[left[i, 0], left[i, 1], left[i, 2], right[i]] for i in range(3)])
    a = mp.matrix(rows)
    values, vectors = mp.eigsy(a.T * a)
    k = min(range(4), key=lambda i: values[i])
    h = vectors[:, k]
    return mean + scale * mp.matrix([h[0], h[1], h[2]]) / h[3]


def _n_view_case(
    name: str,
    poses: list[NDArray[np.float64]],
    points: NDArray[np.float64],
    observed: NDArray[np.bool_],
    *,
    sigma: float,
    seed: int,
) -> dict[str, object]:
    """Build one ``n_view`` case."""
    rng = np.random.default_rng(seed)
    mats = [_mp_pose(p) for p in poses]
    bearings, rows = [], []
    for p, seen in zip(points, observed, strict=True):
        x = mp.matrix(p.tolist())
        rays = [
            (_bearing(r * x + t) if sigma == 0.0 else _noisy_bearing(r * x + t, rng, sigma))
            for r, t in mats
        ]
        views = [v for v in range(len(poses)) if seen[v]]
        best = x if sigma == 0.0 else _dlt([poses[v] for v in views], [rays[v] for v in views])
        rows.append(_row(best, [(*mats[v], rays[v]) for v in views]))
        bearings.append(rays)
    inputs = {
        "poses": [p.tolist() for p in poses],
        "bearings": bearings,
        "observed": observed.tolist(),
    }
    return o.case(name, inputs, rows)


def main() -> None:
    """Write ``triangulation.json``."""
    rng = np.random.default_rng(7)
    points = rng.uniform([-2.0, -2.0, 4.0], [2.0, 2.0, 8.0], size=(12, 3))
    pose = _pose([0.02, -0.1, 0.03], [0.5, -0.1, 0.05])
    reversed_pose = pose.copy()
    reversed_pose[:3, 3] *= -1.0
    two_view = [
        _two_view_case("noise-free", pose, points, sigma=0.0, seed=0),
        _two_view_case("noisy 1e-3", pose, points, sigma=1e-3, seed=1),
        _two_view_case(
            "low parallax (5 mm baseline)",
            _pose([0.0, 0.01, 0.0], [0.005, 0.0, 0.0]),
            points,
            sigma=1e-4,
            seed=2,
        ),
        _two_view_case(
            "translation reversed: behind",
            pose,
            points[:4],
            sigma=1e-3,
            seed=3,
            given_pose=reversed_pose,
        ),
    ]
    poses = [
        np.eye(4),
        pose,
        _pose([-0.05, 0.08, 0.0], [-0.4, 0.2, 0.1]),
        _pose([0.1, 0.05, -0.02], [0.1, 0.6, -0.2]),
    ]
    observed = np.ones((12, 4), dtype=bool)
    observed[::3, 1] = False
    observed[1::4, 3] = False
    far = [p.copy() for p in poses]
    for p in far:
        p[:3, 3] += p[:3, :3] @ np.array([1000.0, -500.0, 200.0])
    far_points = points + np.array([-1000.0, 500.0, -200.0])
    n_view = [
        _n_view_case("noise-free, partly observed", poses, points, observed, sigma=0.0, seed=4),
        _n_view_case("noisy 1e-3, partly observed", poses, points, observed, sigma=1e-3, seed=5),
        _n_view_case("far from the origin, noisy", far, far_points, observed, sigma=1e-3, seed=6),
    ]
    doc = {
        "description": "Triangulation: parity fixture from mpmath (generate_triangulation.py).",
        "oracle": "true points; 50-digit Gauss-Newton optimum; 50-digit eigenvector of the DLT",
        "functions": {
            "two_view": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-9,
                    "reason": "niter2 reaches the optimum to about 1e-12 in cost; depths up "
                    "to 8 and the low-parallax case amplify it",
                },
                "cases": two_view,
            },
            "n_view": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-9,
                    "reason": "an SVD of a normalised 12x4 system; far cases add 1e3 offsets",
                },
                "cases": n_view,
            },
        },
    }
    (HERE / "triangulation.json").write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote triangulation.json ({len(two_view) + len(n_view)} cases)")


if __name__ == "__main__":
    with mp.workdps(o.PRECISION):
        main()
