"""Two-view initialisation tests: ground truth, noisy scenes, statuses, GRIC, validation."""

from __future__ import annotations

import math
from dataclasses import asdict, replace
from typing import Any

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import se3, so3, twoview
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

FIXTURE = load_fixture("foundations/twoview.json")
OPTIONS = twoview.Options(focal=520.0)
ROTATION = se3.exp([0.0, 0.0, 0.0, 0.05, -0.1, 0.02])


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matches_fixture(backend: str) -> None:
    """On exact matches with planted outliers, the true motion, inliers and points."""
    spec: dict[str, Any] = FIXTURE["functions"]["initialize"]
    chosen = require_function(backend, twoview.__name__, "initialize")
    atol = spec["tolerance"]["atol"]
    for case in spec["cases"]:
        inputs = dict(case["input"])
        options = twoview.Options(focal=inputs.pop("focal"), seed=inputs.pop("seed"))
        result = twoview.initialize(**inputs, options=options, backend=chosen)
        expected = case["expected"]
        assert (result.status, result.model) == (expected["status"], expected["model"])
        np.testing.assert_allclose(
            result.relative_pose, expected["relative_pose"], rtol=0, atol=atol
        )
        np.testing.assert_array_equal(result.inliers, expected["inliers"])
        np.testing.assert_array_equal(result.triangulated, expected["triangulated"])
        found = result.points[result.triangulated]
        np.testing.assert_allclose(
            found, np.reshape(expected["points"], found.shape), rtol=0, atol=atol
        )
        assert np.all(np.isnan(result.points[~result.triangulated]))


def _noisy(points: np.ndarray, pose: np.ndarray, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Return 1 px noisy matches of `points` through `pose`, a third replaced by outliers."""
    b1, b2 = sc.pairs(points, pose=pose, sigma=1 / 520, seed=seed)
    b2, _ = sc.with_outliers(b2, 1 / 3, seed=seed + 1)
    return b1, b2


def _rotation_error_deg(found: np.ndarray, truth: np.ndarray) -> float:
    """Return the angle between two rotations, in degrees."""
    return math.degrees(float(np.linalg.norm(so3.log(found[:3, :3].T @ truth[:3, :3]))))


@pytest.mark.parametrize(
    ("name", "points", "pose", "model"),
    [
        ("general", sc.scene(300, seed=1), sc.POSE, twoview.Model.ESSENTIAL),
        ("plane", sc.plane(300, seed=2), sc.POSE, twoview.Model.HOMOGRAPHY),
        (
            "forward",
            sc.scene(300, seed=5),
            se3.exp([0.02, 0.01, 0.6, 0.01, 0.02, 0.0]),
            twoview.Model.ESSENTIAL,
        ),
    ],
)
def test_noisy_scenes_initialise_with_the_right_model(
    name: str, points: np.ndarray, pose: np.ndarray, model: twoview.Model
) -> None:
    """With 1 px noise and a third outliers: initialised, the right model, an accurate motion."""
    result = twoview.initialize(*_noisy(points, pose, seed=0), OPTIONS)
    assert result.status is twoview.Status.INITIALIZED, name
    assert result.model is model, name
    assert _rotation_error_deg(result.relative_pose, pose) < 1.0
    t = pose[:3, 3] / np.linalg.norm(pose[:3, 3])
    assert math.degrees(math.acos(min(1.0, float(result.relative_pose[:3, 3] @ t)))) < 6.0
    assert np.count_nonzero(result.triangulated) >= OPTIONS.min_points


def test_a_rotation_does_not_initialise() -> None:
    """A noisy pure rotation picks H and has no parallax to triangulate."""
    result = twoview.initialize(*_noisy(sc.scene(300, seed=3), ROTATION, seed=0), OPTIONS)
    assert result.model is twoview.Model.HOMOGRAPHY
    assert result.status in {twoview.Status.PURE_ROTATION, twoview.Status.LOW_PARALLAX}
    assert not result.triangulated.any()
    assert _rotation_error_deg(result.relative_pose, ROTATION) < 0.5


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_a_well_estimated_rotation_returns_a_rigid_pose(backend: str) -> None:
    """With a long focal length and many matches a noisy rotation reaches ``pure_rotation``.

    Its H is still only near a rotation; the returned pose must be rigid (panel L1).
    """
    b1, b2 = sc.pairs(sc.scene(1000, seed=3), pose=ROTATION, sigma=1 / 2000, seed=0)
    result = twoview.initialize(b1, b2, twoview.Options(focal=2000.0), backend=backend)
    assert result.status is twoview.Status.PURE_ROTATION
    se3.check_transform(result.relative_pose[None])
    assert _rotation_error_deg(result.relative_pose, ROTATION) < 0.1


def test_a_tiny_baseline_has_low_parallax() -> None:
    """A 1 cm baseline at 4-8 m leaves too little parallax."""
    tiny = se3.exp([0.01, 0.0, 0.0, 0.01, 0.0, 0.0])
    result = twoview.initialize(*_noisy(sc.scene(300, seed=4), tiny, seed=0), OPTIONS)
    assert result.status is twoview.Status.LOW_PARALLAX


def test_a_strict_ambiguity_ratio_reports_ambiguity() -> None:
    """On a plane, the runner-up candidate has some points in front: a ratio of 1% flags it."""
    b1, b2 = _noisy(sc.plane(300, seed=2), sc.POSE, seed=0)
    result = twoview.initialize(b1, b2, replace(OPTIONS, ambiguity_ratio=0.01))
    assert result.status is twoview.Status.AMBIGUOUS
    assert result.in_front[1] > 0.01 * result.in_front[0]


def _desk(seed: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return a desk-like scene: 80% of points on a plane 2 m away, the rest in front of it.

    With a 20 cm baseline and 1 px noise, H wins GRIC; its runner-up candidate has more than
    half its best's support, so with ``ambiguity_ratio=0.5`` H is ambiguous and E decides.
    """
    pose = se3.exp([0.2, 0.0, 0.02, 0.0, 0.02, 0.0])
    points = np.vstack([sc.plane(240, seed=1, depth=2.0), sc.scene(60, seed=2) * [0.5, 0.5, 0.4]])
    b1, b2 = sc.pairs(points, pose=pose, sigma=1 / 520, seed=3 + seed)
    b2, _ = sc.with_outliers(b2, 0.2, seed=4 + seed)
    return b1, b2, pose


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_an_ambiguous_homography_falls_back_to_e(backend: str) -> None:
    """H wins but is ambiguous; E alone is unambiguous, so E decides and the flag says so."""
    b1, b2, pose = _desk()
    strict = replace(OPTIONS, ambiguity_ratio=0.5)
    result = twoview.initialize(b1, b2, strict, backend=backend)
    assert result.gric_homography < result.gric_essential
    assert result.status is twoview.Status.INITIALIZED
    assert result.model is twoview.Model.ESSENTIAL
    assert result.fallback
    assert _rotation_error_deg(result.relative_pose, pose) < 1.0
    default = twoview.initialize(b1, b2, OPTIONS, backend=backend)
    assert (default.model, default.fallback) == (twoview.Model.HOMOGRAPHY, False)


@pytest.mark.parametrize("seed", range(3))
def test_backends_agree_run_for_run(seed: int) -> None:
    """Every backend reaches the same decision, inliers and points on noisy data."""
    b1, b2 = _noisy(sc.scene(200, seed=10 + seed), sc.POSE, seed=seed)
    runs = [
        twoview.initialize(
            b1, b2, OPTIONS, backend=require_function(b, twoview.__name__, "initialize")
        )
        for b in sorted(backends.LOCKSTEP_BACKENDS)
    ]
    for other in runs[1:]:
        assert (other.status, other.model, other.fallback) == (
            runs[0].status,
            runs[0].model,
            runs[0].fallback,
        )
        np.testing.assert_array_equal(other.inliers, runs[0].inliers)
        np.testing.assert_array_equal(other.triangulated, runs[0].triangulated)
        np.testing.assert_allclose(other.relative_pose, runs[0].relative_pose, rtol=0, atol=1e-10)
        assert other.in_front.tolist() == runs[0].in_front.tolist()
        assert other.gric_essential == pytest.approx(runs[0].gric_essential, rel=1e-12)
        assert other.essential is not None
        assert runs[0].essential is not None
        assert other.essential.iterations == runs[0].essential.iterations


def test_gric_terms() -> None:
    """Torr's formula: the capped fit term plus the dimension and parameter penalties."""
    errors = np.array([0.0, 1.0, 100.0])  # in units of sigma²: 0, 1, capped at 2 (E)
    expected = (0.0 + 1.0 + 2.0) + math.log(4) * 3 * 3 + math.log(4 * 3) * 5
    assert twoview.gric(errors, 1.0, 3, 5) == pytest.approx(expected)
    assert twoview.gric(errors, 1.0, 2, 8) == pytest.approx(
        (0.0 + 1.0 + 4.0) + math.log(4) * 2 * 3 + math.log(12) * 8
    )


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"focal": 0.0}, "focal"),
        ({"focal": 520.0, "sigma_px": -1.0}, "sigma_px"),
        ({"focal": 520.0, "min_points": 0}, "min_points"),
        (
            {"focal": 520.0, "ambiguity_ratio": 1.5},
            r"ambiguity_ratio: must be in \(0, 1\], got 1.5",
        ),
        ({"focal": 520.0, "min_parallax_deg": math.nan}, "min_parallax_deg"),
    ],
)
def test_rejects_invalid_options(kwargs: dict[str, float], message: str) -> None:
    """Out-of-range settings are rejected when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        twoview.Options(**kwargs)  # type: ignore[arg-type]


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS - {"python"}))
def test_core_checks_its_own_input(backend: str) -> None:
    """The C++ core rejects bad settings and too few matches itself."""
    impl = backends.implementation(backend, twoview.__name__)
    b1, b2 = sc.pairs(sc.scene(20))
    with pytest.raises(InvalidInputError, match="focal"):
        impl.initialize(b1, b2, {**asdict(OPTIONS), "focal": -1.0})
    with pytest.raises(InvalidInputError, match="at least 8"):
        impl.initialize(b1[:7], b2[:7], asdict(OPTIONS))
    with pytest.raises(ValueError, match="options"):
        impl.initialize(b1, b2, {"focal": 1.0})
