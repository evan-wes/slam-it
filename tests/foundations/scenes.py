"""Synthetic scenes shared by the triangulation, epipolar, homography and two-view tests."""

from __future__ import annotations

from types import ModuleType
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it_all.foundations import nls, ransac, se3
from tests.conftest import assert_matches_fixture

POSE = se3.exp([0.5, -0.1, 0.05, 0.02, -0.1, 0.03])
"""A sideways-moving relative pose ``T_c2_c1`` with some rotation."""


def random_pose(rng: np.random.Generator, translation: float) -> NDArray[np.float64]:
    """Draw a pose: a rotation vector of unit scale, a translation of scale `translation`."""
    return se3.exp(np.concatenate([rng.normal(size=3), rng.normal(scale=translation, size=3)]))


def posed(
    rng: np.random.Generator, n: int
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Draw a camera and n points 2-8 m ahead of it, for absolute pose.

    Parameters
    ----------
    rng : numpy.random.Generator
        The random generator.
    n : int
        How many points.

    Returns
    -------
    tuple of ndarray
        The world points (n, 3), their exact unit bearings (n, 3) and ``T_cam_world`` (4, 4).
    """
    pose = random_pose(rng, 0.5)
    in_camera = np.column_stack([rng.uniform(-3.0, 3.0, (n, 2)), rng.uniform(2.0, 8.0, n)])
    return (in_camera - pose[:3, 3]) @ pose[:3, :3], bearings(in_camera), pose


def bearings(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the unit directions of points."""
    return np.asarray(points / np.linalg.norm(points, axis=-1, keepdims=True), dtype=np.float64)


def in_camera(pose: NDArray[np.float64], points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Map points into a camera's frame: ``R X + t``."""
    return np.asarray(points @ pose[:3, :3].T + pose[:3, 3], dtype=np.float64)


def scene(n: int, seed: int = 0) -> NDArray[np.float64]:
    """Draw points 4-8 m in front of camera 1."""
    return np.random.default_rng(seed).uniform([-2.0, -2.0, 4.0], [2.0, 2.0, 8.0], size=(n, 3))


def plane(n: int, seed: int = 0, depth: float = 5.0) -> NDArray[np.float64]:
    """Draw points on the plane z = `depth` in front of camera 1."""
    xy = np.random.default_rng(seed).uniform(-2.0, 2.0, size=(n, 2))
    return np.column_stack([xy, np.full(n, depth)])


def noisy(rays: NDArray[np.float64], sigma: float, seed: int) -> NDArray[np.float64]:
    """Perturb bearings' normalised image points by Gaussian noise; return unit bearings."""
    x = rays / rays[:, 2:3]
    x[:, :2] += np.random.default_rng(seed).normal(scale=sigma, size=(len(x), 2))
    return bearings(x)


def pairs(
    points: NDArray[np.float64], pose: NDArray[np.float64] = POSE, sigma: float = 0.0, seed: int = 0
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the bearing pairs of points seen from camera 1 and from `pose`."""
    b1, b2 = bearings(points), bearings(in_camera(pose, points))
    if sigma > 0.0:
        b1, b2 = noisy(b1, sigma, seed), noisy(b2, sigma, seed + 1)
    return b1, b2


def with_outliers(
    rays: NDArray[np.float64], fraction: float, seed: int
) -> tuple[NDArray[np.float64], NDArray[np.bool_]]:
    """Replace a fraction of bearings by random directions; return them and the inlier mask."""
    rng = np.random.default_rng(seed)
    out = rays.copy()
    outlier = rng.random(len(rays)) < fraction
    random = np.column_stack([rng.uniform(-0.6, 0.6, (outlier.sum(), 2)), np.ones(outlier.sum())])
    out[outlier] = bearings(random)
    return out, ~outlier


def plane_xy(rays: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return bearings' normalised image points ``(x, y)``."""
    return np.ascontiguousarray(rays[:, :2] / rays[:, 2:3])


def check_fixture(module: ModuleType, fixture: dict[str, Any], function: str, backend: str) -> None:
    """Hold one backend to every case of one function in a fixture.

    A RANSAC function (name ending in ``_ransac``) takes ``threshold`` and ``seed`` from each
    case's input beside the correspondences, and is held to the expected model and inliers. A
    refinement (name starting ``refine_``) is held to the expected state and to whether it
    converged, and, where the case records one, to its final cost; a case may carry its own
    tolerance (with ``cost_rtol`` for the cost).

    Parameters
    ----------
    module : module
        The slam_it_all module under test.
    fixture : dict
        The loaded fixture.
    function : str
        The function's name, in the module and in the fixture.
    backend : str
        The backend, already checked with ``require_function``.
    """
    spec: dict[str, Any] = fixture["functions"][function]
    for case in spec["cases"]:
        inputs = dict(case["input"])
        if function.startswith("refine_"):
            solved = getattr(module, function)(**inputs, backend=backend)
            tolerance = case.get("tolerance", spec["tolerance"])
            pose_tolerance = {k: v for k, v in tolerance.items() if k != "cost_rtol"}
            assert_matches_fixture(solved.state, case["expected"], pose_tolerance)
            assert solved.summary.converged == case["summary"]["converged"]
            if "cost" in case["summary"]:
                cost, rtol = case["summary"]["cost"], tolerance["cost_rtol"]
                assert abs(solved.summary.final_cost - cost) <= rtol * abs(cost)
            continue
        if not function.endswith("_ransac"):
            got = getattr(module, function)(**inputs, backend=backend)
            assert_matches_fixture(got, case["expected"], spec["tolerance"])
            continue
        options = ransac.Options(threshold=inputs.pop("threshold"), seed=inputs.pop("seed"))
        result = getattr(module, function)(**inputs, options=options, backend=backend)
        assert_matches_fixture(result.state.model, case["expected"]["model"], spec["tolerance"])
        np.testing.assert_array_equal(result.state.inliers, case["expected"]["inliers"])


def assert_refined_codes(codes: dict[str, list[str]]) -> None:
    """Assert a refined-RANSAC module's termination tables are RANSAC's and NLS', in order.

    Parameters
    ----------
    codes : dict of str to list of str
        The compiled module's ``codes()``.
    """
    assert tuple(codes["ransac_termination"]) == tuple(ransac.Termination)
    assert tuple(codes["nls_termination"]) == tuple(nls.Termination)


def assert_contains(found: NDArray[np.float64], expected: ArrayLike, atol: float) -> None:
    """Assert every expected item is among the found ones (an unordered comparison).

    Parameters
    ----------
    found : ndarray, shape (K, ...)
        The items a function returned.
    expected : array_like, shape (M, ...)
        Items that must each match one found item within `atol`.
    atol : float
        The absolute tolerance.
    """
    for item in np.asarray(expected, dtype=np.float64):
        distances = [float(np.abs(candidate - item).max()) for candidate in found]
        assert min(distances) <= atol, f"no match within {atol}: closest {min(distances):.3g}"
