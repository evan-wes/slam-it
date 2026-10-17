"""Sim(3) RANSAC tests: the fixture, noise and outliers, the backends' agreement, and refusals."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it import backends
from slam_it.foundations import ransac, se3, sim3
from slam_it.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

FIXTURE = load_fixture("foundations/sim3.json")
PINHOLE = np.array([500.0, 500.0, 320.0, 240.0])
CHI2 = 9.21
"""ORB-SLAM's bound: two degrees of freedom at 99%."""

Array = NDArray[np.float64]


def _matches(
    seed: int, n: int, *, scale: float, noise: float = 0.0, outliers: float = 0.0
) -> tuple[Array, Array, Array, NDArray[np.bool_]]:
    """Return a scene: ``S_12``, the matches' points in cameras 1 and 2, and the clean mask.

    Camera 2's points lie 3-8 m ahead; camera 1's are theirs carried by ``S_12``, `noise` metres
    of Gaussian noise added, and a fraction `outliers` replaced by unrelated points ahead.
    """
    rng = np.random.default_rng(seed)
    truth = se3.exp(np.concatenate([rng.uniform(-0.3, 0.3, 3), rng.uniform(-0.2, 0.2, 3)]))
    truth[:3, :3] *= scale
    points2 = np.column_stack([rng.uniform(-2, 2, (n, 2)), rng.uniform(3, 8, n)])
    points1 = points2 @ truth[:3, :3].T + truth[:3, 3] + rng.normal(0.0, noise, (n, 3))
    clean = np.ones(n, dtype=bool)
    bad = rng.choice(n, int(outliers * n), replace=False)
    clean[bad] = False
    points1[bad] = np.column_stack([rng.uniform(-2, 2, (len(bad), 2)), rng.uniform(3, 8, len(bad))])
    return truth, points1, points2, clean


def _settings(seed: int = 0) -> ransac.Options:
    """Return ORB-SLAM's RANSAC settings: the bound's square root, 0.99, at most 300 samples."""
    return ransac.Options(threshold=np.sqrt(CHI2), confidence=0.99, max_iterations=300, seed=seed)


# --- fixtures -----------------------------------------------------------------------------


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matches_fixture(backend: str) -> None:
    """Each backend returns the 50-digit similarity and exactly the clean inliers."""
    chosen = require_function(backend, sim3.__name__, "sim3_ransac")
    sc.check_fixture(sim3, FIXTURE, "sim3_ransac", chosen)


# --- the estimate -------------------------------------------------------------------------


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("fixed_scale", [False, True])
def test_the_similarity_is_recovered_through_noise_and_outliers(
    backend: str, fixed_scale: bool
) -> None:
    """With 40% outliers and 1 cm of noise the similarity is within the noise.

    No outlier is an inlier, and most clean matches are; the scale is held at 1 when fixed.
    """
    chosen = require_function(backend, sim3.__name__, "sim3_ransac")
    for seed in range(5):
        scale = 1.0 if fixed_scale else 0.6 + 0.2 * seed
        truth, points1, points2, clean = _matches(seed, 150, scale=scale, noise=0.01, outliers=0.4)
        found = sim3.sim3_ransac(
            points1, points2, np.ones(150), np.ones(150), _settings(seed), model="pinhole",
            intrinsics=PINHOLE, fixed_scale=fixed_scale, backend=chosen,
        )  # fmt: skip
        np.testing.assert_allclose(found.state.model, truth, atol=0.02)
        if fixed_scale:
            assert np.cbrt(np.linalg.det(found.state.model[:3, :3])) == pytest.approx(1.0)
        assert not np.any(found.state.inliers & ~clean)
        assert np.count_nonzero(found.state.inliers) >= 0.8 * np.count_nonzero(clean)


def test_the_inlier_test_is_the_bound_in_both_images() -> None:
    """A match is an inlier exactly when it projects within the bound in each image.

    Image 1's error alone, image 2's alone, and both are each tested against a variance
    scaled so the bound falls on either side of them.
    """
    truth, points1, points2, _ = _matches(0, 20, scale=1.0)
    estimator = sim3.SimilarityEstimator.create(
        points1, points2, np.ones(20), np.ones(20), model="pinhole", intrinsics=PINHOLE,
        fixed_scale=True,
    )  # fmt: skip
    nudged = truth.copy()
    nudged[0, 3] += 0.002  # every point moves ~0.2-0.3 px in both images
    errors = estimator.residuals(nudged)
    assert np.all((errors > 0.0) & (errors < CHI2))
    ratio = float(np.median(errors))
    for sigma2_1, sigma2_2 in ((ratio / CHI2 / 2, 1.0), (1.0, ratio / CHI2 / 2)):
        tight = sim3.SimilarityEstimator.create(
            points1, points2, np.full(20, sigma2_1), np.full(20, sigma2_2), model="pinhole",
            intrinsics=PINHOLE, fixed_scale=True,
        )  # fmt: skip
        assert np.count_nonzero(tight.residuals(nudged) < CHI2) < 20


def test_a_point_carried_behind_the_other_camera_is_never_an_inlier() -> None:
    """Carried behind either camera, a match's residual is infinite."""
    _, points1, points2, _ = _matches(1, 10, scale=1.0)
    estimator = sim3.SimilarityEstimator.create(
        points1, points2, np.ones(10), np.ones(10), model="pinhole", intrinsics=PINHOLE,
        fixed_scale=False,
    )  # fmt: skip
    behind = np.eye(4)
    behind[2, 3] = -20.0
    assert np.all(np.isinf(estimator.residuals(behind)))


def test_the_inverse_undoes_the_similarity() -> None:
    """``S⁻¹ S`` is the identity, scale and all."""
    truth, *_ = _matches(2, 3, scale=1.7)
    np.testing.assert_allclose(sim3.inverse(truth) @ truth, np.eye(4), atol=1e-14)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_backends_agree_on_ransac(backend: str) -> None:
    """Every compiled core draws the same samples: same inliers, iterations and similarity."""
    chosen = require_function(backend, sim3.__name__, "sim3_ransac")
    for seed in range(5):
        _, points1, points2, _ = _matches(seed, 120, scale=0.9, noise=0.005, outliers=0.3)
        variances = 1.2 ** (2.0 * (np.arange(120) % 8))
        reference, compiled = (
            sim3.sim3_ransac(
                points1, points2, variances, variances[::-1].copy(), _settings(seed),
                model="pinhole", intrinsics=PINHOLE, fixed_scale=seed % 2 == 0, backend=name,
            )
            for name in ("python", chosen)
        )  # fmt: skip
        np.testing.assert_array_equal(compiled.state.inliers, reference.state.inliers)
        run, expected = compiled.summary, reference.summary
        assert (run.iterations, run.termination, run.inlier_count) == (
            expected.iterations,
            expected.termination,
            expected.inlier_count,
        )
        assert run.score == pytest.approx(expected.score, rel=1e-9)
        np.testing.assert_allclose(compiled.state.model, reference.state.model, atol=1e-10)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_collinear_matches_are_degenerate(backend: str) -> None:
    """Points on one line determine no rotation: a Degenerate error, not a guess."""
    chosen = require_function(backend, sim3.__name__, "sim3_ransac")
    line = np.column_stack([np.linspace(-1, 1, 10), np.zeros(10), np.full(10, 5.0)])
    with pytest.raises(DegenerateError):
        sim3.sim3_ransac(
            line, line, np.ones(10), np.ones(10), _settings(), model="pinhole",
            intrinsics=PINHOLE, fixed_scale=False, backend=chosen,
        )  # fmt: skip


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_bad_matches_are_refused(backend: str) -> None:
    """Too few matches, unequal sets, a point behind its camera, a bad variance or camera."""
    chosen = require_function(backend, sim3.__name__, "sim3_ransac")
    _, points1, points2, _ = _matches(3, 12, scale=1.0)
    ones = np.ones(12)

    def run(one: Array, two: Array, first: Array, *, intrinsics: Array = PINHOLE) -> None:
        sim3.sim3_ransac(
            one, two, first, np.ones(len(first)), _settings(), model="pinhole",
            intrinsics=intrinsics, fixed_scale=False, backend=chosen,
        )  # fmt: skip

    with pytest.raises(InvalidInputError):
        run(points1[:2], points2[:2], ones[:2])
    with pytest.raises(InvalidInputError):
        run(points1, points2[:11], ones)
    behind = points2.copy()
    behind[4, 2] = -1.0
    with pytest.raises(InvalidInputError, match="in front"):
        run(points1, behind, ones)
    zero = ones.copy()
    zero[0] = 0.0
    with pytest.raises(InvalidInputError, match="sigma2_1"):
        run(points1, points2, zero)
    with pytest.raises(InvalidInputError):
        run(points1, points2, ones, intrinsics=np.array([1.0, 2.0]))


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_the_compiled_codes_match(backend: str) -> None:
    """The termination codes Python decodes are each compiled core's, in the same order."""
    chosen = require_function(backend, sim3.__name__, "codes")
    codes = backends.implementation(chosen, sim3.__name__).codes()
    assert tuple(codes["ransac_termination"]) == tuple(ransac.Termination)
