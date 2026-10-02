"""Tests for the BA readers (BAL, COLMAP text), and our BA against COLMAP's Ceres BA.

The readers are checked against independent implementations: BAL's projection equation
written out here, and pycolmap reading the same COLMAP files. The Ceres comparison runs
COLMAP's bundle adjuster (Ceres underneath) on the same problem.
"""

from __future__ import annotations

import bz2
from dataclasses import replace
from pathlib import Path

import numpy as np
import pycolmap
import pytest
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from slamlab import backends
from slamlab.eval import colmap_reference
from slamlab.foundations import ba, camera, nls, se3
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import require_backend

Array = NDArray[np.float64]


def _bal_scene(seed: int = 0) -> tuple[Array, Array, list[tuple[int, int]]]:
    """Return BAL cameras (N, 9), points, and (camera, point) pairs with the point in front."""
    rng = np.random.default_rng(seed)
    cams = np.zeros((4, 9))
    cams[:, :3] = rng.normal(0.0, 0.1, (4, 3))
    cams[:, 3:6] = np.column_stack([0.5 * np.arange(4), np.zeros(4), np.full(4, -6.0)])
    cams[:, 6] = rng.uniform(450, 550, 4)
    cams[:, 7:9] = [-0.1, 0.02]
    points = rng.uniform([-2, -2, -2], [2, 2, 2], (30, 3))
    pairs = [(c, p) for c in range(4) for p in range(30)]
    return cams, points, pairs


def _bal_pixel(cam: Array, point: Array) -> Array:
    """Project with BAL's own equations: ``P = R X + t``, ``p = -P/P_z``, ``f r(p) p``."""
    rotated = Rotation.from_rotvec(cam[:3]).apply(point) + cam[3:6]
    p = -rotated[:2] / rotated[2]
    r2 = p @ p
    return np.asarray(cam[6] * (1.0 + cam[7] * r2 + cam[8] * r2 * r2) * p)


def _write_bal(path: Path, cams: Array, points: Array, pairs: list[tuple[int, int]]) -> None:
    """Write a BAL file (bz2 if the name says so)."""
    lines = [f"{len(cams)} {len(points)} {len(pairs)}"]
    lines += [
        f"{c} {p} {x:.17g} {y:.17g}" for c, p in pairs for x, y in [_bal_pixel(cams[c], points[p])]
    ]
    lines += [f"{v:.17g}" for v in cams.ravel()]
    lines += [f"{v:.17g}" for v in points.ravel()]
    text = "\n".join(lines) + "\n"
    if path.suffix == ".bz2":
        path.write_bytes(bz2.compress(text.encode()))
    else:
        path.write_text(text)


def _reprojections(problem: ba.BAProblem) -> Array:
    """Project every observation's point with the problem's own variables."""
    t_cw = se3.inverse(problem.poses)
    cam, pt = problem.cam_idx, problem.pt_idx
    in_cam = np.einsum("mij,mj->mi", t_cw[cam, :3, :3], problem.points[pt]) + t_cw[cam, :3, 3]
    out = np.zeros((cam.size, 2))
    for c in np.unique(cam):
        sel = cam == c
        k = problem.intrinsics[problem.intrinsics_idx[c]]
        out[sel] = camera.project(problem.camera_model, k, in_cam[sel])
    return out


@pytest.mark.parametrize("name", ["problem.txt", "problem.txt.bz2"])
def test_bal_reader_matches_bal_projection(tmp_path: Path, name: str) -> None:
    """Read back, our model reproduces BAL's projection (with y flipped) to 1e-10 px."""
    cams, points, pairs = _bal_scene()
    path = tmp_path / name
    _write_bal(path, cams, points, pairs)
    problem = ba.read_bal(path)
    assert problem.camera_model == "radial"
    np.testing.assert_allclose(_reprojections(problem), problem.observations, rtol=0, atol=1e-10)
    np.testing.assert_array_equal(problem.fixed_intrinsics[0], [False, True, True, False, False])


def test_bal_reader_rejects_truncated_files(tmp_path: Path) -> None:
    """A file whose counts do not match its contents is rejected."""
    path = tmp_path / "bad.txt"
    path.write_text("2 1 1\n0 0 1.0 2.0\n")
    with pytest.raises(InvalidInputError, match="expected"):
        ba.read_bal(path)


def _write_colmap(root: Path, model: str, params: list[float], seed: int = 0) -> None:
    """Write a small COLMAP text model: two images, and a third with no keypoints."""
    rng = np.random.default_rng(seed)
    points = rng.uniform([-1, -1, 4], [1, 1, 6], (12, 3))
    lines = ["# cameras", f"1 {model} 640 480 " + " ".join(f"{v:.17g}" for v in params)]
    (root / "cameras.txt").write_text("\n".join(lines) + "\n")
    rec = pycolmap.Reconstruction()
    cam = pycolmap.Camera(model=model, width=640, height=480, params=params, camera_id=1)
    image_lines = ["# images"]
    tracks: dict[int, list[str]] = {p: [] for p in range(12)}
    for i in range(2):
        rotation = Rotation.from_rotvec(rng.normal(0, 0.05, 3))
        translation = np.array([-0.3 * i, 0.0, 0.0])
        qx, qy, qz, qw = rotation.as_quat()
        header = (
            f"{i + 1} {qw:.17g} {qx:.17g} {qy:.17g} {qz:.17g} "
            + " ".join(f"{v:.17g}" for v in translation)
            + f" 1 im{i}.png"
        )
        keypoints = []
        for p, x in enumerate(points):
            uv = cam.img_from_cam(rotation.apply(x) + translation)
            keypoints.append(f"{uv[0]:.17g} {uv[1]:.17g} {p + 1}")
            tracks[p].append(f"{i + 1} {p}")
        image_lines += [header, " ".join(keypoints)]
    image_lines.append("3 1 0 0 0 0 0 0 1 empty.png")
    image_lines.append("")
    (root / "images.txt").write_text("\n".join(image_lines) + "\n")
    point_lines = ["# points"] + [
        f"{p + 1} " + " ".join(f"{v:.17g}" for v in x) + " 0 0 0 0.5 " + " ".join(tracks[p])
        for p, x in enumerate(points)
    ]
    (root / "points3D.txt").write_text("\n".join(point_lines) + "\n")
    del rec


@pytest.mark.parametrize(
    ("model", "params", "ours", "refinable"),
    [
        ("SIMPLE_RADIAL", [500.0, 320.0, 240.0, -0.05], "radial", [1, 1, 1, 1, 0]),
        ("RADIAL", [500.0, 320.0, 240.0, -0.05, 0.01], "radial", [1, 1, 1, 1, 1]),
        ("PINHOLE", [500.0, 505.0, 320.0, 240.0], "pinhole", [1, 1, 1, 1]),
        ("OPENCV", [500.0, 505.0, 320.0, 240.0, -0.05, 0.01, 1e-3, -1e-3], "radtan", [1] * 8 + [0]),
    ],
)
def test_colmap_reader_matches_pycolmap(
    tmp_path: Path, model: str, params: list[float], ours: str, refinable: list[int]
) -> None:
    """Pixels pycolmap projects (COLMAP's pixel centres) come back -0.5 px under our model."""
    _write_colmap(tmp_path, model, params)
    read = ba.read_colmap_text(tmp_path)
    assert read.problem.camera_model == ours
    np.testing.assert_array_equal(read.refinable[0], np.array(refinable, dtype=bool))
    assert read.image_names == ["im0.png", "im1.png", "empty.png"]
    np.testing.assert_allclose(
        _reprojections(read.problem), read.problem.observations, rtol=0, atol=1e-9
    )
    reference = pycolmap.Reconstruction(str(tmp_path))
    np.testing.assert_allclose(
        np.array([reference.points3D[int(i)].xyz for i in read.point_ids]), read.problem.points
    )


def test_colmap_reader_rejects_mixed_models(tmp_path: Path) -> None:
    """Cameras from different families cannot share one model."""
    _write_colmap(tmp_path, "RADIAL", [500.0, 320.0, 240.0, 0.0, 0.0])
    with (tmp_path / "cameras.txt").open("a") as handle:
        handle.write("2 OPENCV 640 480 500 500 320 240 0 0 0 0\n")
    with pytest.raises(InvalidInputError, match="unsupported model mix"):
        ba.read_colmap_text(tmp_path)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_matches_ceres_on_a_bal_style_problem(tmp_path: Path, backend: str) -> None:
    """On a BAL problem with f, k1, k2 free, our optimum matches COLMAP's Ceres BA.

    COLMAP's solution is re-evaluated with our own camera model, so the two optima are compared
    as RMS reprojection error under one cost, and the focal lengths directly.
    """
    require_backend(backend)
    cams, points, pairs = _bal_scene(seed=4)
    rng = np.random.default_rng(5)
    path = tmp_path / "noisy.txt"
    _write_bal(path, cams, points, pairs)
    problem = ba.read_bal(path)
    noisy = problem.observations + rng.normal(0.0, 0.5, problem.observations.shape)
    start_intrinsics = problem.intrinsics * [1.02, 1.0, 1.0, 1.0, 1.0]
    problem = ba.BAProblem.create(
        "radial",
        start_intrinsics,
        problem.poses,
        problem.points,
        noisy,
        problem.cam_idx,
        problem.pt_idx,
        intrinsics_idx=problem.intrinsics_idx,
        fixed_intrinsics=problem.fixed_intrinsics,
    )
    options = ba.BAOptions(
        solver=nls.Options(max_iterations=200, function_tolerance=1e-14), gauge="none"
    )
    ours = ba.solve(problem, options, backend=backend)
    reference = colmap_reference.solve(problem, tolerance=1e-14)
    colmap_focal = reference.intrinsics[:, 0]
    rows = 2 * problem.observations.shape[0]
    colmap_cost = 0.5 * float(
        np.sum(
            (
                _reprojections(
                    replace(
                        problem,
                        poses=reference.poses,
                        intrinsics=reference.intrinsics,
                        points=reference.points,
                    )
                )
                - problem.observations
            )
            ** 2
        )
    )
    np.testing.assert_allclose(
        np.sqrt(2.0 * ours.summary.solve.final_cost / rows),
        np.sqrt(2.0 * colmap_cost / rows),
        rtol=1e-8,
    )
    np.testing.assert_allclose(ours.intrinsics[:, 0], colmap_focal, rtol=1e-7)
