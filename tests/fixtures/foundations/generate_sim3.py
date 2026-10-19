"""Generate ``sim3.json``: the Sim(3) RANSAC parity fixture.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_sim3

Independent methods, nothing imported from slam_it: each case is an exact sample built in 50
digits (mpmath) from a chosen similarity ``S_12 = [[s R, t], [0, 1]]``. Camera 2's points are
chosen in float64, carried into camera 1 in 50 digits and rounded once; a third of the matches
then get camera-1 points far from their partners'. RANSAC must return the chosen similarity,
with exactly the clean matches as inliers: scaled (monocular), rigid (stereo and RGB-D), and
through a distorting camera. Each outlier's error is checked here to be far past the bound in
image 1, with a pinhole projection written out below (and radtan's distortion for that case).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mpmath as mp
import numpy as np

from tests.fixtures.foundations import _layout
from tests.fixtures.foundations import _mp_oracle as o

HERE = Path(__file__).parent

CHI2 = 9.21
"""The inlier bound, two degrees of freedom at 99% (ORB-SLAM's)."""

PINHOLE = [500.0, 500.0, 320.0, 240.0]
"""A pinhole camera's intrinsics."""

RADTAN = [517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633]
"""A radtan camera's intrinsics (TUM fr1's): fx, fy, cx, cy, k1, k2, p1, p2, k3."""

MATCHES = 60
"""The matches in each case."""


def _project(intrinsics: list[float], point: list[float]) -> np.ndarray:
    """Project a camera-frame point (float64), with radtan's distortion when given its terms."""
    x, y = point[0] / point[2], point[1] / point[2]
    if len(intrinsics) == len(RADTAN):
        k1, k2, p1, p2, k3 = intrinsics[4:]
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2**2 + k3 * r2**3
        x, y = (
            x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
            y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
        )
    fx, fy, cx, cy = intrinsics[:4]
    return np.array([fx * x + cx, fy * y + cy])


def _case(
    name: str,
    omega: list[float],
    t: list[float],
    scale: float,
    *,
    model: str,
    intrinsics: list[float],
    seed: int,
) -> dict[str, Any]:
    """Build one case: the similarity, its exact inliers and its far outliers."""
    rng = np.random.default_rng(seed)
    rot = mp.matrix(o.so3_exp(o.mpf_vector(omega)))
    s = mp.mpf(scale)
    shift = mp.matrix(o.mpf_vector(t))
    expected = [[float(s * rot[i, j]) for j in range(3)] + [float(shift[i])] for i in range(3)]
    expected.append([0.0, 0.0, 0.0, 1.0])
    # Near the axis (|x/z|, |y/z| at most 0.2), where radtan's distortion is monotone.
    points2 = np.column_stack(
        [rng.uniform(-0.6, 0.6, (MATCHES, 2)), rng.uniform(3.0, 8.0, MATCHES)]
    ).tolist()
    points1 = [[float(x) for x in s * rot * mp.matrix(p) + shift] for p in points2]
    clean = [True] * MATCHES
    for k in rng.choice(MATCHES, MATCHES // 3, replace=False).tolist():
        clean[k] = False
        true = _project(intrinsics, points1[k])
        while True:
            candidate = [*rng.uniform(-0.5, 0.5, 2).tolist(), float(rng.uniform(2.5, 7.0))]
            candidate[0] *= candidate[2]
            candidate[1] *= candidate[2]
            # Far past the bound at the coarsest level's variance (1.2^14 px²).
            if np.sum((_project(intrinsics, candidate) - true) ** 2) > 1e3 * CHI2 * 1.2**14:
                break
        points1[k] = candidate
    levels = rng.integers(0, 8, (2, MATCHES))
    return o.case(
        name,
        {
            "points1": points1,
            "points2": points2,
            "sigma2_1": [1.2 ** (2 * int(level)) for level in levels[0]],
            "sigma2_2": [1.2 ** (2 * int(level)) for level in levels[1]],
            "model": model,
            "intrinsics": intrinsics,
            "fixed_scale": scale == 1.0,
            "threshold": float(np.sqrt(CHI2)),
            "seed": seed,
        },
        {"model": expected, "inliers": clean},
    )


def _fixture() -> dict[str, Any]:
    """Build the cases."""
    cases = [
        _case(
            "a third outliers, exact inliers, scaled",
            [0.1, -0.2, 0.05], [0.3, -0.1, 0.2], 0.8,
            model="pinhole", intrinsics=PINHOLE, seed=5,
        ),
        _case(
            "a third outliers, exact inliers, rigid",
            [-0.15, 0.1, 0.2], [-0.2, 0.05, 0.4], 1.0,
            model="pinhole", intrinsics=PINHOLE, seed=6,
        ),
        _case(
            "a third outliers, exact inliers, scaled, through radtan's distortion",
            [0.05, 0.12, -0.08], [0.1, 0.2, -0.3], 1.3,
            model="radtan", intrinsics=RADTAN, seed=7,
        ),
    ]  # fmt: skip
    return {
        "description": "Sim(3) RANSAC: parity fixture from mpmath (generate_sim3.py).",
        "oracle": "the chosen similarity of exact samples, built in 50 digits",
        "functions": {
            "sim3_ransac": {
                "tolerance": {
                    "comparison": "close",
                    "rtol": 0.0,
                    "atol": 1e-10,
                    "reason": "exact inliers: Umeyama's alignment of any three is the similarity "
                    "to rounding, and LO's refits of all of them too",
                },
                "cases": cases,
            }
        },
    }


def main() -> None:
    """Write ``sim3.json``."""
    with mp.workdps(o.PRECISION):
        doc = _fixture()
    (HERE / "sim3.json").write_text(_layout.dumps(doc))
    print(f"wrote sim3.json ({len(doc['functions']['sim3_ransac']['cases'])} cases)")


if __name__ == "__main__":
    main()
