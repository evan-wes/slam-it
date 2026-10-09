"""Generate ``camera.json``, the camera-model parity fixture, from a 50-digit mpmath oracle.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_camera

Independent methods, nothing imported from slam_it_all: projection by its defining formula in
high precision; Jacobians by mpmath's numerical differentiation of that formula (not the
analytic derivatives under test); unprojection by mpmath's root finder on the distortion
equations, then normalised.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np

from tests.fixtures.foundations import _mp_oracle as o

OUT = Path(__file__).with_name("camera.json")
MODELS = {"pinhole": 0, "radtan": 1, "radial": 2}
PINHOLE = [520.9, 521.0, 325.1, 249.7]
# A strongly distorting lens (TUM RGB-D fr1-like) and a mild one.
RADTAN = {
    "strong": [520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172],
    "mild": [
        458.654,
        457.296,
        367.215,
        248.375,
        -0.28340811,
        0.07395907,
        0.00019359,
        1.76187114e-05,
        0.0,
    ],
}


# A BAL-like camera: one focal length, two radial terms (Ladybug-scale values).
RADIAL = [499.4, 320.0, 240.0, -0.31, 0.12]


def _distort(coefficients: list[Any], x: Any, y: Any) -> tuple[Any, Any]:  # noqa: ANN401 -- mpmath numbers
    """Apply radtan distortion ``(k1, k2, p1, p2, k3)`` to a normalised point."""
    k1, k2, p1, p2, k3 = coefficients
    r2 = x * x + y * y
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    return (
        x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
        y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y,
    )


def _pixel(model: str, values: list[Any], component: int) -> Any:  # noqa: ANN401 -- mpmath number
    """One pixel coordinate of the projection, from point (X, Y, Z) and intrinsics."""
    x_, y_, z_ = values[:3]
    if model == "radial":  # BAL's formula: f r(p) p + c, with r = 1 + k1 |p|² + k2 |p|⁴
        f, cx, cy, k1, k2 = values[3:8]
        x, y = x_ / z_, y_ / z_
        r2 = x * x + y * y
        scale = f * (1 + k1 * r2 + k2 * r2 * r2)
        return scale * x + cx if component == 0 else scale * y + cy
    fx, fy, cx, cy = values[3:7]
    x, y = x_ / z_, y_ / z_
    xd, yd = _distort(values[7:12], x, y) if model == "radtan" else (x, y)
    return fx * xd + cx if component == 0 else fy * yd + cy


def _project(model: str, k: list[float], points: list[list[float]]) -> list[list[float]]:
    """Project points in high precision."""
    out = []
    for p in points:
        values = o.mpf_vector(p + k)
        out.append([float(_pixel(model, values, c)) for c in (0, 1)])
    return out


def _jacobians(model: str, k: list[float], points: list[list[float]]) -> list[Any]:
    """d(pixel)/d(point, intrinsics) by mpmath's numerical differentiation."""
    out = []
    for p in points:
        base = o.mpf_vector(p + k)
        rows = []
        for c in (0, 1):
            row = []
            for j in range(len(base)):

                def f(t: Any, j: int = j, c: int = c, base: list[Any] = base) -> Any:  # noqa: ANN401 -- mpmath numbers
                    return _pixel(model, [*base[:j], t, *base[j + 1 :]], c)

                row.append(float(mp.diff(f, base[j])))
            rows.append(row)
        out.append(rows)
    return out


def _distortion_equations(kk: list[Any], xd: Any, yd: Any) -> Callable[..., list[Any]]:  # noqa: ANN401 -- mpmath numbers
    """Return the radtan equations ``distort(x, y) - (xd, yd) = 0`` as a function."""

    def equations(x: Any, y: Any) -> list[Any]:  # noqa: ANN401 -- mpmath numbers
        dx, dy = _distort(kk[4:9], x, y)
        return [dx - xd, dy - yd]

    return equations


def _unproject(model: str, k: list[float], pixels: list[list[float]]) -> list[list[float]]:
    """Bearing vectors: solve the distortion equations with mpmath's root finder."""
    kk = o.mpf_vector(k)
    if model == "radial":  # as radtan with fx = fy = f and no tangential or k3 terms
        kk = [kk[0], kk[0], kk[1], kk[2], kk[3], kk[4], mp.mpf(0), mp.mpf(0), mp.mpf(0)]
        model = "radtan"
    out = []
    for u, v in pixels:
        xd = (mp.mpf(u) - kk[2]) / kk[0]
        yd = (mp.mpf(v) - kk[3]) / kk[1]
        if model == "radtan":
            x, y = mp.findroot(_distortion_equations(kk, xd, yd), (xd, yd))
        else:
            x, y = xd, yd
        n = mp.sqrt(x * x + y * y + 1)
        out.append([float(x / n), float(y / n), float(1 / n)])
    return out


def main() -> None:
    """Write ``camera.json``."""
    rng = np.random.default_rng(17)
    points = {
        "on the optical axis": [[0.0, 0.0, 2.0]],
        "general": np.column_stack(
            [rng.uniform(-0.8, 0.8, 6), rng.uniform(-0.6, 0.6, 6), rng.uniform(1.0, 6.0, 6)]
        ).tolist(),
        "near the image corner": [[0.9, 0.7, 1.5], [-0.95, -0.72, 1.6]],
        "very close": [[0.01, -0.02, 0.05]],
        "far": [[3.0, -2.0, 400.0]],
    }
    cameras = (
        {"pinhole": ("pinhole", PINHOLE)}
        | {f"radtan {n}": ("radtan", k) for n, k in RADTAN.items()}
        | {"radial": ("radial", RADIAL)}
    )
    fx: dict[str, Any] = {
        name: {"unpack": True, "cases": []}
        for name in ("project", "project_jacobians", "unproject")
    }
    fx["project"]["tolerance"] = {
        "comparison": "close",
        "rtol": 1e-15,
        "atol": 1e-12,
        "reason": "pixels O(1e3); a few roundings",
    }
    fx["project_jacobians"]["tolerance"] = {
        "comparison": "close",
        "rtol": 1e-13,
        "atol": 1e-11,
        "reason": "analytic vs numerical differentiation; entries up to O(fx / Z)",
    }
    fx["unproject"]["tolerance"] = {
        "comparison": "close",
        "rtol": 0.0,
        "atol": 1e-14,
        "reason": "unit vectors; Newton to 1e-15",
    }
    for cam_name, (model, k) in cameras.items():
        for pts_name, pts in points.items():
            name = f"{cam_name}, {pts_name}"
            fx["project"]["cases"].append(o.case(name, [model, k, pts], _project(model, k, pts)))
            fx["project_jacobians"]["cases"].append(
                o.case(name, [model, k, pts], _jacobians(model, k, pts))
            )
            pixels = _project(model, k, pts)
            fx["unproject"]["cases"].append(
                o.case(name, [model, k, pixels], _unproject(model, k, pixels))
            )
    doc = {
        "description": "Camera models: parity fixture from mpmath (generate_camera.py).",
        "oracle": "the defining formula in 50 digits; mp.diff; mp.findroot",
        "functions": fx,
    }
    OUT.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote {OUT} ({sum(len(f['cases']) for f in fx.values())} cases)")


if __name__ == "__main__":
    with mp.workdps(o.PRECISION):
        main()
