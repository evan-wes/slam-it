"""Epipolar geometry tests: the mpmath fixture, the matrices' structure, RANSAC, validation."""

from __future__ import annotations

from dataclasses import asdict, replace
from typing import Any

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import epipolar, ransac, so3
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

FIXTURE = load_fixture("foundations/epipolar.json")
DIRECT = ["essential_8pt", "fundamental_8pt", "sampson_essential", "sampson_fundamental"]
K = np.array([[520.0, 0.0, 320.0], [0.0, 515.0, 240.0], [0.0, 0.0, 1.0]])


def _pixels(rays: np.ndarray) -> np.ndarray:
    """Project bearings through K."""
    return np.ascontiguousarray((rays / rays[:, 2:3] @ K.T)[:, :2])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", DIRECT)
def test_matches_fixture(backend: str, function: str) -> None:
    """Each backend matches the 50-digit oracle (matrices up to sign)."""
    chosen = require_function(backend, epipolar.__name__, function)
    sc.check_fixture(epipolar, FIXTURE, function, chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_ransac_matches_fixture(backend: str) -> None:
    """With noise-free inliers, RANSAC finds the true E and exactly the clean pairs."""
    chosen = require_function(backend, epipolar.__name__, "essential_ransac")
    sc.check_fixture(epipolar, FIXTURE, "essential_ransac", chosen)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_essential_has_the_right_structure(backend: str) -> None:
    """E from noisy pairs has singular values (1, 1, 0); F has rank 2 and unit norm."""
    b1, b2 = sc.pairs(sc.scene(60), sigma=1e-3)
    singular = np.linalg.svd(epipolar.essential_8pt(b1, b2, backend=backend), compute_uv=False)
    np.testing.assert_allclose(singular, [1.0, 1.0, 0.0], rtol=0, atol=1e-14)
    f = epipolar.fundamental_8pt(_pixels(b1), _pixels(b2), backend=backend)
    assert np.linalg.norm(f) == pytest.approx(1.0, abs=1e-14)
    assert abs(np.linalg.det(f)) < 1e-14


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_planar_scenes_are_degenerate_for_the_8_point(backend: str) -> None:
    """Coplanar points leave the 8-point system a family of solutions: DegenerateError."""
    b1, b2 = sc.pairs(sc.plane(30))
    with pytest.raises(DegenerateError, match="unique"):
        epipolar.essential_8pt(b1, b2, backend=backend)
    with pytest.raises(DegenerateError, match="unique"):
        epipolar.fundamental_8pt(_pixels(b1), _pixels(b2), backend=backend)


def test_essential_is_t_cross_r() -> None:
    """Noise-free E equals ``[t̂]ₓ R`` up to sign."""
    b1, b2 = sc.pairs(sc.scene(20))
    t = sc.POSE[:3, 3] / np.linalg.norm(sc.POSE[:3, 3])
    truth = so3.hat(t) @ sc.POSE[:3, :3]
    found = epipolar.essential_8pt(b1, b2)
    assert min(np.abs(found - truth).max(), np.abs(found + truth).max()) < 1e-13


@pytest.mark.parametrize("function", ["essential_ransac", "fundamental_ransac"])
def test_ransac_backends_agree_run_for_run(function: str) -> None:
    """Same seed, same samples: every backend stops on the same iteration, same inliers."""
    b1, b2 = sc.pairs(sc.scene(150, seed=3), sigma=2e-3, seed=4)
    b2, _ = sc.with_outliers(b2, 0.4, seed=5)
    if function == "essential_ransac":
        args: tuple[np.ndarray, np.ndarray] = (b1, b2)
        options = ransac.Options(threshold=4e-3, seed=9)
    else:
        args = (_pixels(b1), _pixels(b2))
        options = ransac.Options(threshold=2.0, seed=9)
    runs = [
        getattr(epipolar, function)(
            *args, options, backend=require_function(b, epipolar.__name__, function)
        )
        for b in sorted(backends.LOCKSTEP_BACKENDS)
    ]
    first = runs[0]
    for other in runs[1:]:
        assert other.summary.iterations == first.summary.iterations
        assert other.summary.local_optimizations == first.summary.local_optimizations
        np.testing.assert_array_equal(other.state.inliers, first.state.inliers)
        m, n = other.state.model, first.state.model
        assert min(np.abs(m - n).max(), np.abs(m + n).max()) < 1e-12


def test_local_optimization_reaches_the_8_point_noise_floor() -> None:
    """LO-RANSAC's E reaches the 8-point fit to the true inliers, and beats plain RANSAC.

    Over ten scenes with 1 px noise and 40% outliers: the median error is within 1.5 times the
    floor (the 8-point fit to the true inliers), recall is above 85%, and plain RANSAC's median
    error is more than twice LO's.

    The 8-point algorithm as a minimal solver is noise-sensitive; local optimisation's
    least-squares refits are what bring the result to the floor (docs/notes/two-view.md).
    """
    truth = so3.hat(sc.POSE[:3, 3] / np.linalg.norm(sc.POSE[:3, 3])) @ sc.POSE[:3, :3]

    def error(e: np.ndarray) -> float:
        return float(min(np.abs(e - truth).max(), np.abs(e + truth).max()))

    ratios, plain_ratios, recalls = [], [], []
    for seed in range(10):
        b1, b2 = sc.pairs(sc.scene(200, seed=seed), sigma=1 / 520, seed=seed + 100)
        b2, clean = sc.with_outliers(b2, 0.4, seed=seed + 200)
        floor = error(epipolar.essential_8pt(b1[clean], b2[clean]))
        options = ransac.Options(threshold=2 / 520, seed=seed)
        result = epipolar.essential_ransac(b1, b2, options)
        plain = epipolar.essential_ransac(b1, b2, replace(options, local_optimization=False))
        ratios.append(error(result.state.model) / floor)
        plain_ratios.append(error(plain.state.model) / floor)
        recalls.append((result.state.inliers & clean).sum() / clean.sum())
    assert np.median(ratios) < 1.5
    assert np.median(plain_ratios) > 2.0 * np.median(ratios)
    assert np.median(recalls) > 0.85


@pytest.mark.parametrize(
    ("function", "args", "message"),
    [
        (
            "essential_8pt",
            (np.ones((7, 3)) / np.sqrt(3), np.ones((7, 3)) / np.sqrt(3)),
            "at least 8",
        ),
        ("essential_8pt", (np.ones((8, 3)), np.ones((8, 3))), "unit"),
        (
            "essential_8pt",
            (-np.ones((8, 3)) / np.sqrt(3), np.ones((8, 3)) / np.sqrt(3)),
            "positive z",
        ),
        ("fundamental_8pt", (np.ones((8, 2)), np.ones((9, 2))), "equal"),
        ("fundamental_8pt", (np.ones((7, 2)), np.ones((7, 2))), "at least 8"),
        ("sampson_essential", (np.eye(4), np.ones((1, 3)), np.ones((1, 3))), "essential"),
    ],
)
def test_rejects_invalid_input(function: str, args: tuple[Any, ...], message: str) -> None:
    """Bad shapes, sizes and bearings are rejected before any backend runs."""
    with pytest.raises(InvalidInputError, match=message):
        getattr(epipolar, function)(*args)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS - {"python"}))
def test_core_checks_its_own_input(backend: str) -> None:
    """The C++ core rejects what the entry point rejects, and its option dict is strict."""
    impl = backends.implementation(backend, epipolar.__name__)
    with pytest.raises(InvalidInputError, match="at least 8"):
        impl.essential_8pt(np.ones((7, 3)) / np.sqrt(3), np.ones((7, 3)) / np.sqrt(3))
    b1, b2 = sc.pairs(sc.scene(20))
    with pytest.raises(ValueError, match="options"):
        impl.essential_ransac(b1, b2, {"threshold": 1.0})
    with pytest.raises(InvalidInputError, match="threshold"):
        impl.essential_ransac(b1, b2, {**_options_dict(), "threshold": -1.0})


def _options_dict() -> dict[str, Any]:
    """Return default RANSAC options as the dict the bindings take."""
    return asdict(ransac.Options(threshold=1.0))
