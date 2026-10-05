"""Generate ``stereo.json``, the stereo-rectification parity fixture.

Run from the repository root after changing the cases (the fixture is committed)::

    python -m tests.fixtures.foundations.generate_stereo

Independent of the module under test, nothing imported from slamlab: the rectifying rotations
are OpenCV's ``stereoRectify`` (Bouguet's algorithm, the same construction); the rectified
camera and the baseline follow from their definitions; each map pixel is computed from the
pinhole inverse and the Brown-Conrady formula written out here. Rotations agree to rounding
(``1e-12``), maps to the distortion polynomial's rounding (``1e-9`` pixels).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from tests.fixtures.foundations._layout import dumps

OUT = Path(__file__).with_name("stereo.json")
OUTSIDE = -2.0
MIN_DEPTH = 1e-9

# EuRoC MAV's cam0 and cam1 (sensor.yaml): radtan with k3 = 0, and their body poses.
EUROC_LEFT = [458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359,
              1.76187114e-05, 0.0]  # fmt: skip
EUROC_RIGHT = [457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473,
               -3.55590700e-05, 0.0]  # fmt: skip
EUROC_B_LEFT = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                0.0, 0.0, 0.0, 1.0]  # fmt: skip
EUROC_B_RIGHT = [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
                 0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
                 -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
                 0.0, 0.0, 0.0, 1.0]  # fmt: skip


def _pose(rotvec: list[float], translation: list[float]) -> NDArray[np.float64]:
    """Build a rigid transform from a rotation vector and a translation."""
    out = np.eye(4)
    out[:3, :3] = cv2.Rodrigues(np.asarray(rotvec, dtype=np.float64))[0]
    out[:3, 3] = translation
    return out


def _matrix(model: str, k: list[float]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """OpenCV's camera matrix and distortion vector for a model's intrinsics."""
    if model == "radial":
        f, cx, cy, k1, k2 = k
        return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]]), np.array([k1, k2, 0, 0, 0.0])
    fx, fy, cx, cy = k[:4]
    distortion = np.array(k[4:9] if model == "radtan" else [0.0] * 5)
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]), distortion


def _focal(model: str, k: list[float]) -> float:
    """Return the camera's smallest focal length."""
    return k[0] if model == "radial" else min(k[0], k[1])


def _rectify(case: dict[str, Any]) -> dict[str, Any]:
    """Rectify with OpenCV's rotations; the camera and baseline from their definitions."""
    left, right = case["left"], case["right"]
    transform = np.asarray(case["T_right_left"])
    rows, cols = case["shape"]
    k1, d1 = _matrix(*left)
    k2, d2 = _matrix(*right)
    r1, r2, *_ = cv2.stereoRectify(
        k1, d1, k2, d2, (cols, rows),
        transform[:3, :3].copy(), transform[:3, 3].reshape(3, 1).copy(), alpha=0,
    )  # fmt: skip
    focal = min(_focal(*left), _focal(*right))
    return {
        "rotation_left": r1.tolist(),
        "rotation_right": r2.tolist(),
        "intrinsics": [focal, focal, (cols - 1) / 2.0, (rows - 1) / 2.0],
        "baseline": float(np.linalg.norm(transform[:3, 3])),
    }


def _map(
    model: str,
    k: list[float],
    rotation: list[list[float]],
    rectified: list[float],
    shape: list[int],
) -> list[Any]:
    """Each rectified pixel's source: unproject through the pinhole, turn, project, distort."""
    rows, cols = shape
    f, _, cx, cy = rectified
    r = np.asarray(rotation)
    out = []
    for v in range(rows):
        row = []
        for u in range(cols):
            ray = r.T @ np.array([(u - cx) / f, (v - cy) / f, 1.0])
            if ray[2] <= MIN_DEPTH:
                row.append([OUTSIDE, OUTSIDE])
                continue
            x, y = ray[0] / ray[2], ray[1] / ray[2]
            if model == "radial":
                fx = fy = k[0]
                c = (k[1], k[2])
                k1, k2, p1, p2, k3 = k[3], k[4], 0.0, 0.0, 0.0
            else:
                fx, fy, c = k[0], k[1], (k[2], k[3])
                k1, k2, p1, p2, k3 = k[4:9] if model == "radtan" else (0.0,) * 5
            r2 = x * x + y * y
            radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
            xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            row.append([fx * xd + c[0], fy * yd + c[1]])
        out.append(row)
    return out


def main() -> None:
    """Write the fixture."""
    euroc = np.linalg.inv(np.reshape(EUROC_B_RIGHT, (4, 4))) @ np.reshape(EUROC_B_LEFT, (4, 4))
    pairs = [
        {
            "name": "EuRoC MAV cam0 and cam1",
            "left": ["radtan", EUROC_LEFT],
            "right": ["radtan", EUROC_RIGHT],
            "T_right_left": euroc.tolist(),
            "shape": [480, 752],
        },
        {
            "name": "pinhole pair, right camera to the left",
            "left": ["pinhole", [500.0, 505.0, 320.0, 240.0]],
            "right": ["pinhole", [498.0, 502.0, 318.0, 243.0]],
            "T_right_left": _pose([0.01, -0.03, 0.02], [0.12, 0.004, -0.01]).tolist(),
            "shape": [480, 640],
        },
        {
            "name": "small radial and radtan cameras, for the maps",
            "left": ["radial", [11.0, 7.4, 5.6, -0.05, 0.01]],
            "right": ["radtan", [10.0, 10.5, 7.6, 5.4, -0.04, 0.012, 0.001, -0.002, 0.003]],
            "T_right_left": _pose([0.02, 0.05, -0.01], [-0.3, 0.01, 0.02]).tolist(),
            "shape": [12, 16],
        },
    ]
    rectify_cases = []
    map_cases = []
    for pair in pairs:
        expected = _rectify(pair)
        rectify_cases.append(
            {
                "name": pair["name"],
                "input": [*pair["left"], *pair["right"], pair["T_right_left"], pair["shape"]],
                "expected": expected,
            }
        )
        if pair["shape"][0] * pair["shape"][1] <= 400:  # keep the fixture small
            rectification = {**expected, "shape": pair["shape"]}
            map_cases.append(
                {
                    "name": pair["name"],
                    "input": [pair["left"], pair["right"], rectification],
                    "expected": [
                        _map(
                            pair["left"][0],
                            pair["left"][1],
                            expected["rotation_left"],
                            expected["intrinsics"],
                            pair["shape"],
                        ),
                        _map(
                            pair["right"][0],
                            pair["right"][1],
                            expected["rotation_right"],
                            expected["intrinsics"],
                            pair["shape"],
                        ),
                    ],
                }
            )
    functions: dict[str, Any] = {
        "rectify": {
            "unpack": True,
            "tolerance": {
                "comparison": "close",
                "rtol": 0.0,
                "atol": 1e-12,
                "reason": "rotations from SO(3)'s exp and log",
            },
            "cases": rectify_cases,
        },
        "rectification_map": {
            "unpack": True,
            "tolerance": {
                "comparison": "close",
                "rtol": 0.0,
                "atol": 1e-9,
                "reason": "pixels O(10); a distortion polynomial's rounding",
            },
            "cases": map_cases,
        },
    }
    fixture = {
        "description": "Stereo rectification: parity fixture (generate_stereo.py).",
        "oracle": "OpenCV's stereoRectify; the maps from their defining formulas",
        "functions": functions,
    }
    OUT.write_text(dumps(fixture))


if __name__ == "__main__":
    main()
