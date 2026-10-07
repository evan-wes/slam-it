"""Homography tests: the mpmath fixture, plane and rotation scenes, RANSAC, validation."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import correspondences, homography, ransac, se3
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

FIXTURE = load_fixture("foundations/homography.json")


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", ["homography_dlt", "sampson_homography"])
def test_matches_fixture(backend: str, function: str) -> None:
    """Each backend matches the 50-digit oracle (H up to sign)."""
    chosen = require_function(backend, homography.__name__, function)
    sc.check_fixture(homography, FIXTURE, function, chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_ransac_matches_fixture(backend: str) -> None:
    """With noise-free inliers, RANSAC finds the true H and exactly the clean pairs."""
    chosen = require_function(backend, homography.__name__, "homography_ransac")
    sc.check_fixture(homography, FIXTURE, "homography_ransac", chosen)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_pure_rotation_is_a_homography(backend: str) -> None:
    """Without translation any scene maps by H = R (normalised)."""
    rotation = se3.exp([0.0, 0.0, 0.0, 0.05, -0.1, 0.02])
    b1, b2 = sc.pairs(sc.scene(30), pose=rotation)
    found = homography.homography_dlt(sc.plane_xy(b1), sc.plane_xy(b2), backend=backend)
    truth = rotation[:3, :3] / np.linalg.norm(rotation[:3, :3])
    assert min(np.abs(found - truth).max(), np.abs(found + truth).max()) < 1e-13


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_collinear_points_are_degenerate(backend: str) -> None:
    """Three collinear points in four do not determine H."""
    p1 = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [0.0, 1.0]])
    with pytest.raises(DegenerateError, match="unique"):
        homography.homography_dlt(p1, p1 + 0.5, backend=backend)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_sampson_is_zero_on_the_model(backend: str) -> None:
    """Pairs mapped exactly by H have zero error; a shifted pair has about its shift squared."""
    h = np.array([[1.0, 0.1, 0.2], [0.0, 0.9, -0.1], [0.01, 0.0, 1.0]])
    p1 = np.random.default_rng(0).uniform(-0.5, 0.5, size=(10, 2))
    mapped = np.column_stack([p1, np.ones(10)]) @ h.T
    p2 = mapped[:, :2] / mapped[:, 2:3]
    assert np.all(homography.sampson_homography(h, p1, p2, backend=backend) < 1e-28)


def test_ransac_backends_agree_run_for_run() -> None:
    """Same seed, same samples: every backend stops on the same iteration, same inliers."""
    b1, b2 = sc.pairs(sc.plane(120, seed=2), sigma=2e-3, seed=3)
    b2, _ = sc.with_outliers(b2, 0.4, seed=4)
    options = ransac.Options(threshold=4e-3, seed=2)
    runs = [
        homography.homography_ransac(
            sc.plane_xy(b1),
            sc.plane_xy(b2),
            options,
            backend=require_function(b, homography.__name__, "homography_ransac"),
        )
        for b in sorted(backends.LOCKSTEP_BACKENDS)
    ]
    for other in runs[1:]:
        assert other.summary.iterations == runs[0].summary.iterations
        np.testing.assert_array_equal(other.state.inliers, runs[0].state.inliers)


@pytest.mark.parametrize(
    ("function", "args", "message"),
    [
        ("homography_dlt", (np.ones((3, 2)), np.ones((3, 2))), "at least 4"),
        ("homography_dlt", (np.ones((4, 2)), np.ones((5, 2))), "equal"),
        ("sampson_homography", (np.eye(2), np.ones((1, 2)), np.ones((1, 2))), "homography"),
        (
            "sampson_homography",
            (np.full((3, 3), np.nan), np.ones((1, 2)), np.ones((1, 2))),
            "finite",
        ),
    ],
)
def test_rejects_invalid_input(function: str, args: tuple[Any, ...], message: str) -> None:
    """Bad shapes and sizes are rejected before any backend runs."""
    with pytest.raises(InvalidInputError, match=message):
        getattr(homography, function)(*args)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_decomposition_matches_fixture(backend: str) -> None:
    """The true motion and plane are among the candidates; a rotation has one candidate."""
    spec: dict[str, Any] = FIXTURE["functions"]["decompose_homography"]
    chosen = require_function(backend, homography.__name__, "decompose_homography")
    for case in spec["cases"]:
        found = homography.decompose_homography(**case["input"], backend=chosen)
        atol = spec["tolerance"]["atol"]
        sc.assert_contains(found.poses, case["expected"]["contains_poses"], atol)
        sc.assert_contains(found.normals, case["expected"]["contains_normals"], atol)
    rotation = homography.decompose_homography(np.eye(3), backend=chosen)
    assert rotation.poses.shape == (1, 4, 4)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_every_candidate_reproduces_h(backend: str) -> None:
    """Over random motions and planes, each of the four candidates rebuilds H up to scale."""
    rng = np.random.default_rng(0)
    for _ in range(25):
        pose = se3.exp(np.r_[rng.normal(scale=0.5, size=3), rng.normal(scale=0.2, size=3)])
        normal = rng.normal(size=3)
        normal /= np.linalg.norm(normal)
        h = pose[:3, :3] + np.outer(pose[:3, 3], normal) / rng.uniform(2.0, 8.0)
        motions = homography.decompose_homography(h, backend=backend)
        assert motions.poses.shape == (4, 4, 4)
        target = h / np.linalg.norm(h)
        for candidate, n in zip(motions.poses, motions.normals, strict=True):
            rebuilt = candidate[:3, :3] + np.outer(candidate[:3, 3], n)
            rebuilt /= np.linalg.norm(rebuilt)
            assert min(np.abs(rebuilt - target).max(), np.abs(rebuilt + target).max()) < 1e-10


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_the_dlt_refuses_a_singular_fit(backend: str) -> None:
    """Two points matched to one: the only exact fit is singular, so there is no homography.

    Four correspondences determine H, but with two of image 1's points on one point of image 2
    the exact fit maps their line onto that point: rank below 3, which no two views of a plane
    give. The DLT, H-RANSAC's minimal and least-squares solver, refuses it.
    """
    chosen = require_function(backend, homography.__name__, "homography_dlt")
    points1 = [[0.0, 0.0], [100.0, 0.0], [0.0, 100.0], [100.0, 100.0]]
    points2 = [[10.0, 10.0], [10.0, 10.0], [20.0, 120.0], [120.0, 110.0]]
    with pytest.raises(DegenerateError, match="H: the correspondences"):
        homography.homography_dlt(points1, points2, backend=chosen)
    moved = [[10.0, 10.0], [110.0, 12.0], [20.0, 120.0], [120.0, 110.0]]
    assert homography.homography_dlt(points1, moved, backend=chosen).shape == (3, 3)


def _tilted_view(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray] | None:
    """Return a minimal sample of a steep plane seen in two wide views, in pixels, or None.

    A plane tilted up to 85 degrees from facing camera 1, at 1 to 10 m, and four points on it
    anywhere in camera 1's 60-degree field of view; camera 2 rotated up to 30 degrees about
    each axis and moved up to the plane's distance along each. None if a ray misses the
    plane's front, or a point is behind a camera or outside camera 2's view.
    """
    focal, half = 320.0, np.tan(np.radians(30.0))
    depth = rng.uniform(1.0, 10.0)
    tilt, azimuth = np.radians(rng.uniform(0.0, 85.0)), rng.uniform(0.0, 2.0 * np.pi)
    normal = np.array(
        [np.sin(tilt) * np.cos(azimuth), np.sin(tilt) * np.sin(azimuth), -np.cos(tilt)]
    )
    rays = np.column_stack([rng.uniform(-half, half, (4, 2)), np.ones(4)])
    along = rays @ normal
    if np.any(along >= -1e-6):
        return None
    points = rays * (normal[2] * depth / along)[:, None]  # the plane passes through (0, 0, d)
    pose = se3.exp(np.r_[rng.uniform(-1.0, 1.0, 3) * depth, rng.uniform(-1.0, 1.0, 3) * 0.5236])
    seen = points @ pose[:3, :3].T + pose[:3, 3]
    if np.any(points[:, 2] <= 0.0) or np.any(seen[:, 2] <= 0.0):
        return None
    image1, image2 = focal * points[:, :2] / points[:, 2:], focal * seen[:, :2] / seen[:, 2:]
    if np.any(np.abs(image2) > focal * half):
        return None
    return image1, image2


def _condition(points1: np.ndarray, points2: np.ndarray, h: np.ndarray) -> float:
    """Return H's smallest over largest singular value in the Hartley-normalised frame."""
    t1 = correspondences.hartley_normalisation(points1)
    t2 = correspondences.hartley_normalisation(points2)
    assert t1 is not None
    assert t2 is not None
    singular = np.linalg.svd(t2 @ h @ np.linalg.inv(t1), compute_uv=False)
    return float(singular[2] / singular[0])


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_steep_planes_keep_the_rank_margin(backend: str) -> None:
    """Minimal samples of steep planes under wide motions all fit (review panel finding L10).

    The DLT refuses an H below :data:`CONDITION_TOLERANCE` of its largest singular value; on
    these 2000 scenes the least is 4.5e-4, so a threshold raised into the range real scenes
    reach refuses one of them and fails here.
    """
    chosen = require_function(backend, homography.__name__, "homography_dlt")
    rng = np.random.default_rng(31)
    least, fitted = np.inf, 0
    for _ in range(20_000):  # bounded: the seeded draws give 2000 views well within it
        view = _tilted_view(rng)
        if view is None:
            continue
        least = min(least, _condition(*view, homography.homography_dlt(*view, backend=chosen)))
        fitted += 1
        if fitted == 2000:
            break
    assert fitted == 2000
    assert least > homography.CONDITION_TOLERANCE


def test_singular_homographies_are_rejected() -> None:
    """A rank-1 matrix is no homography: DegenerateError."""
    with pytest.raises(DegenerateError, match="rank"):
        homography.decompose_homography(np.outer([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_candidates_of_a_noisy_h_are_rotations(backend: str) -> None:
    """An estimated H is not exactly R + t nᵀ; every candidate rotation is still in SO(3).

    Found on TUM fr1/xyz: unprojected, the closed form's rotation was off by more than 1e-6,
    and triangulating with it was rejected.
    """
    b1, b2 = sc.pairs(sc.plane(60, seed=7), sigma=2e-3, seed=8)
    h = homography.homography_dlt(sc.plane_xy(b1), sc.plane_xy(b2))
    for pose in homography.decompose_homography(h, backend=backend).poses:
        rotation = pose[:3, :3]
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-12)
        assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-12)


def test_the_reference_decomposition_checks_rank_itself() -> None:
    """A singular H raises in the reference that twoview calls, as in C++ (panel H1)."""
    with pytest.raises(DegenerateError, match="rank"):
        homography.decompose_reference(np.outer([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
