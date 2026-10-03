"""Generate ``p3p.json``: the absolute-pose parity fixtures.

Run from the repository root after changing the cases (the fixtures are committed)::

    python -m tests.fixtures.foundations.generate_pose

Independent methods, nothing imported from slamlab: each case is an exact sample built in 50
digits (mpmath) from a chosen pose ``T_cam_world``. The world points are chosen in float64,
mapped into the camera in 50 digits, and their bearings rounded once; the expected pose is the
chosen one, which the solvers must return among their solutions.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np

from tests.fixtures.foundations import _mp_oracle as o

HERE = Path(__file__).parent

P3P_SOLVERS = ("ding", "lambda_twist")
"""The P3P solvers the fixture covers (slamlab.foundations.p3p.SOLVERS)."""


def _pose(omega: list[float], t: list[float]) -> tuple[mp.matrix, mp.matrix]:
    """Return a 50-digit rotation ``exp(ω)`` and translation."""
    return mp.matrix(o.so3_exp(o.mpf_vector(omega))), mp.matrix(o.mpf_vector(t))


def _pose_list(rot: mp.matrix, t: mp.matrix) -> list[list[float]]:
    """Round ``[R | t]`` to a float64 4x4 pose."""
    rows = [[float(rot[i, j]) for j in range(3)] + [float(t[i])] for i in range(3)]
    return [*rows, [0.0, 0.0, 0.0, 1.0]]


def _bearings(rot: mp.matrix, t: mp.matrix, points: list[list[float]]) -> list[list[float]]:
    """Map world points into the camera in 50 digits; round their unit bearings once."""
    out = []
    for p in points:
        c = rot * mp.matrix(p) + t
        n = mp.norm(c)
        out.append([float(x / n) for x in c])
    return out


def _cylinder_sample(margin: float) -> tuple[mp.matrix, mp.matrix, list[list[float]]]:
    """Return a pose and three points with the camera near their danger cylinder.

    The points lie on a circle of radius 1 in the plane ``z = 0``; the camera centre is at
    ``(1 - margin, 0, 2)``, inside the cylinder over that circle by `margin`. On the cylinder
    two of P3P's solutions coincide, so the problem is ill-conditioned close to it.
    """
    angles = (0.4, 2.3, 4.1)
    points = [[float(np.cos(a)), float(np.sin(a)), 0.0] for a in angles]
    centre = mp.matrix([1 - mp.mpf(margin), 0, 2])
    # Look down at the circle's centre: the camera's z towards (0, 0, 0) from the centre.
    forward = -centre / mp.norm(centre)
    side = mp.matrix([0, 1, 0])
    side = side - (side.T * forward)[0] * forward
    side = side / mp.norm(side)
    down = mp.matrix([forward[1] * side[2] - forward[2] * side[1],
                      forward[2] * side[0] - forward[0] * side[2],
                      forward[0] * side[1] - forward[1] * side[0]])  # fmt: skip
    rot = mp.matrix([[side[k] for k in range(3)], [down[k] for k in range(3)],
                     [forward[k] for k in range(3)]])  # fmt: skip
    return rot, -(rot * centre), points


def _p3p_fixture() -> dict[str, Any]:
    """Build the P3P cases: exact three-point samples, the true pose among the solutions."""
    rng = np.random.default_rng(23)
    samples: list[tuple[str, mp.matrix, mp.matrix, list[list[float]]]] = []
    generic = {
        "generic": _pose([0.3, -0.2, 0.1], [0.2, -0.1, 4.0]),
        "rotated": _pose([1.2, 0.4, -0.8], [-0.5, 0.3, 6.0]),
        "far": _pose([0.05, 0.1, -0.02], [1.0, -2.0, 30.0]),
    }
    for name, (rot, t) in generic.items():
        points = rng.uniform(-2.0, 2.0, size=(3, 3)).tolist()
        samples.append((name, rot, t, points))
    # Ding et al. reorder the points so the longest side is the reference; each ordering once.
    rot, t = _pose([0.1, 0.2, -0.1], [0.1, 0.2, 5.0])
    base = [[0.0, 0.0, 0.0], [0.3, 0.1, 0.2], [-0.2, 0.25, -0.1]]
    for longest, points in (
        ("first and second", [base[0], [2.5, 0.2, 0.1], base[2]]),
        ("first and third", [base[0], base[1], [-2.4, 0.3, -0.2]]),
        ("second and third", [[0.1, -0.2, 0.0], [1.3, 0.2, 0.1], [-1.2, 0.3, -0.2]]),
    ):
        samples.append((f"longest side between the {longest} points", rot, t, points))
    # 1e-6 inside, two true poses are 1e-6 apart: a solver that merges a quadratic's close roots
    # into one returns a pose between them, 4e-7 from each.
    for margin in (1e-2, 1e-6):
        rot, t, points = _cylinder_sample(margin)
        samples.append((f"camera {margin:g} inside the danger cylinder", rot, t, points))
    cases = []
    for name, rot, t, points in samples:
        bearings = _bearings(rot, t, points)
        for solver in P3P_SOLVERS:
            cases.append(
                o.case(
                    f"{name}, {solver}",
                    {"points": points, "bearings": bearings, "solver": solver},
                    [_pose_list(rot, t)],
                )
            )
    return {
        "description": "The P3P solvers: parity fixture from mpmath (generate_pose.py).",
        "oracle": "the true pose of exact samples, built in 50 digits",
        "functions": {
            "p3p": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-8,
                    "reason": "the expected pose must be among the solutions; worst measured "
                    "1e-9, 1e-6 inside the danger cylinder (the problem's condition number there "
                    "is about 1e6); 2e-12 elsewhere",
                },
                "cases": cases,
            }
        },
    }


def main() -> None:
    """Write ``p3p.json``."""
    for name, build in (("p3p.json", _p3p_fixture),):
        with mp.workdps(o.PRECISION):
            doc = build()
        (HERE / name).write_text(json.dumps(doc, indent=1) + "\n")
        functions: dict[str, dict[str, Any]] = doc["functions"]
        count = sum(len(f["cases"]) for f in functions.values())
        print(f"wrote {name} ({count} cases)")


if __name__ == "__main__":
    main()
