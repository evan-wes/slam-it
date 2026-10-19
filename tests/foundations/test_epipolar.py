"""Epipolar geometry tests: the mpmath fixture, the matrices' structure, RANSAC, validation."""

from __future__ import annotations

from dataclasses import asdict, replace
from typing import Any

import numpy as np
import pytest

from slam_it import backends
from slam_it.foundations import correspondences, epipolar, ransac, so3
from slam_it.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

FIXTURE = load_fixture("foundations/epipolar.json")
ILL_CONDITIONED = load_fixture("foundations/ill_conditioned.json")
DIRECT = ["essential_8pt", "fundamental_8pt", "sampson_essential", "sampson_fundamental"]
K = np.array([[520.0, 0.0, 320.0], [0.0, 515.0, 240.0], [0.0, 0.0, 1.0]])


def _pixels(rays: np.ndarray) -> np.ndarray:
    """Project bearings through K."""
    return np.ascontiguousarray((rays / rays[:, 2:3] @ K.T)[:, :2])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", ["essential_8pt", "fundamental_8pt"])
def test_ill_conditioned_8pt_matches_fixture(backend: str, function: str) -> None:
    """Near-planar, noisy scenes, where a weaker SVD gave E and F off by up to 6e-3 (panel L2)."""
    chosen = require_function(backend, epipolar.__name__, function)
    sc.check_fixture(epipolar, ILL_CONDITIONED, function, chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_ill_conditioned_decomposition_matches_fixture(backend: str) -> None:
    """Singular values (1, 1e-6, 0): every backend's candidates are the oracle's (panel L1).

    A 3x3 SVD through ``MᵀM`` loses the third singular pair here.
    """
    spec: dict[str, Any] = ILL_CONDITIONED["functions"]["decompose_essential"]
    chosen = require_function(backend, epipolar.__name__, "decompose_essential")
    for case in spec["cases"]:
        found = epipolar.decompose_essential(**case["input"], backend=chosen)
        sc.assert_contains(found, case["expected"], spec["tolerance"]["atol"])


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("scale", [1e-200, 1e200])
def test_decomposition_does_not_depend_on_scale(backend: str, scale: float) -> None:
    """E scaled far from 1 gives E's own candidates (panel run 2, L8).

    LAPACK and Eigen scale a matrix into a safe range before its SVD; an SVD that does not
    loses the factors to underflow below about 1e-154, or fails above about 1e154.
    """
    chosen = require_function(backend, epipolar.__name__, "decompose_essential")
    essential = _true_essential()
    reference = epipolar.decompose_essential(essential, backend="python")
    scaled = epipolar.decompose_essential(scale * essential, backend=chosen)
    sc.assert_contains(scaled, reference, 1e-12)


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", DIRECT)
def test_matches_fixture(backend: str, function: str) -> None:
    """Each backend matches the 50-digit oracle (matrices up to sign)."""
    chosen = require_function(backend, epipolar.__name__, function)
    sc.check_fixture(epipolar, FIXTURE, function, chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_ransac_matches_fixture(backend: str) -> None:
    """With noise-free inliers, every minimal solver finds the true E and the clean pairs."""
    chosen = require_function(backend, epipolar.__name__, "essential_ransac")
    sc.check_fixture(epipolar, FIXTURE, "essential_ransac", chosen)
    for case in FIXTURE["functions"]["essential_ransac"]["cases"]:
        inputs = dict(case["input"])
        options = ransac.Options(threshold=inputs.pop("threshold"), seed=inputs.pop("seed"))
        found = epipolar.essential_ransac(**inputs, options=options, backend=chosen)
        ran = found.summary.refinement is not None
        assert ran == case["summary"]["refinement_ran"], case["name"]


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_refinement_matches_fixture(backend: str) -> None:
    """On exact pairs the refinement converges from a perturbed start to the true E."""
    chosen = require_function(backend, epipolar.__name__, "refine_essential")
    sc.check_fixture(epipolar, FIXTURE, "refine_essential", chosen)


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


def _contaminated() -> tuple[np.ndarray, np.ndarray]:
    """Return 150 noisy bearing pairs, 40% of them outliers, for the run-for-run tests."""
    b1, b2 = sc.pairs(sc.scene(150, seed=3), sigma=2e-3, seed=4)
    b2, _ = sc.with_outliers(b2, 0.4, seed=5)
    return b1, b2


def test_fundamental_ransac_backends_agree_run_for_run() -> None:
    """Same seed, same samples: every backend stops on the same iteration, same inliers.

    E-RANSAC's counterpart, for every minimal solver, is
    ``test_ransac_backends_agree_for_every_solver``.
    """
    b1, b2 = _contaminated()
    runs = [
        epipolar.fundamental_ransac(
            _pixels(b1),
            _pixels(b2),
            ransac.Options(threshold=2.0, seed=9),
            backend=require_function(b, epipolar.__name__, "fundamental_ransac"),
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
        result = epipolar.essential_ransac(b1, b2, options, minimal_solver="8pt", refine=False)
        plain = epipolar.essential_ransac(
            b1,
            b2,
            replace(options, local_optimization=False),
            minimal_solver="8pt",
            refine=False,
        )
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
        impl.essential_ransac(b1, b2, {"threshold": 1.0}, 0, True)
    with pytest.raises(InvalidInputError, match="threshold"):
        impl.essential_ransac(b1, b2, {**_options_dict(), "threshold": -1.0}, 0, True)
    with pytest.raises(ValueError, match="minimal_solver"):
        impl.essential_ransac(b1, b2, _options_dict(), len(epipolar.MINIMAL_SOLVERS), True)
    with pytest.raises(InvalidInputError, match="at least 5"):
        impl.refine_essential(b1[:4], b2[:4], np.eye(3), 1.0)
    with pytest.raises(InvalidInputError, match="scale"):
        impl.refine_essential(b1, b2, so3.hat([1.0, 0.0, 0.0]), 0.0)
    with pytest.raises(DegenerateError, match="rank"):
        impl.refine_essential(b1, b2, np.diag([1.0, 0.0, 0.0]), 1.0)


def _options_dict() -> dict[str, Any]:
    """Return default RANSAC options as the dict the bindings take."""
    return asdict(ransac.Options(threshold=1.0))


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_decomposition_matches_fixture(backend: str) -> None:
    """The four poses are the closed-form ones (R, ±t̂) and the twisted pair, in any order."""
    spec: dict[str, Any] = FIXTURE["functions"]["decompose_essential"]
    chosen = require_function(backend, epipolar.__name__, "decompose_essential")
    for case in spec["cases"]:
        found = epipolar.decompose_essential(**case["input"], backend=chosen)
        assert found.shape == (4, 4, 4)
        sc.assert_contains(found, case["expected"], spec["tolerance"]["atol"])


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_every_candidate_explains_e(backend: str) -> None:
    """Each candidate's ``[t]ₓ R`` equals E up to sign, for E of any scale."""
    b1, b2 = sc.pairs(sc.scene(30), sigma=1e-3)
    essential = epipolar.essential_8pt(b1, b2)
    for pose in epipolar.decompose_essential(-3.0 * essential, backend=backend):
        rebuilt = so3.hat(pose[:3, 3]) @ pose[:3, :3]
        assert min(np.abs(rebuilt - essential).max(), np.abs(rebuilt + essential).max()) < 1e-12


def test_rank_one_matrices_are_not_essential() -> None:
    """A rank-1 matrix has no translation direction: DegenerateError."""
    with pytest.raises(DegenerateError, match="rank"):
        epipolar.decompose_essential(np.outer([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))


# --- the 5-point minimal solvers in RANSAC, and the Sampson refinement --------------------


def _true_essential() -> np.ndarray:
    """Return ``[t̂]ₓ R`` for the shared pose."""
    return so3.hat(sc.POSE[:3, 3] / np.linalg.norm(sc.POSE[:3, 3])) @ sc.POSE[:3, :3]


def _essential_error(found: np.ndarray, truth: np.ndarray) -> float:
    """Return the max-abs difference of two Es, up to sign."""
    return float(min(np.abs(found - truth).max(), np.abs(found + truth).max()))


@pytest.mark.parametrize("minimal_solver", epipolar.MINIMAL_SOLVERS)
@pytest.mark.parametrize("refine", [False, True])
def test_ransac_backends_agree_for_every_solver(minimal_solver: str, refine: bool) -> None:
    """Every minimal solver, refined or not, runs sample for sample the same in each backend."""
    b1, b2 = _contaminated()
    options = ransac.Options(threshold=4e-3, seed=9)
    runs = [
        epipolar.essential_ransac(
            b1,
            b2,
            options,
            minimal_solver=minimal_solver,
            refine=refine,
            backend=require_function(b, epipolar.__name__, "essential_ransac"),
        )
        for b in sorted(backends.LOCKSTEP_BACKENDS)
    ]
    for other in runs[1:]:
        first, summary = runs[0].summary, other.summary
        assert summary.iterations == first.iterations
        assert summary.run.local_optimizations == first.run.local_optimizations
        assert summary.run.score == pytest.approx(first.run.score, rel=1e-12)
        assert summary.refined == first.refined
        assert summary.rounds == first.rounds
        assert (summary.refinement is None) == (first.refinement is None) == (not refine)
        if summary.refinement is not None and first.refinement is not None:
            assert summary.refinement.iterations == first.refinement.iterations
            assert summary.refinement.termination == first.refinement.termination
        np.testing.assert_array_equal(other.state.inliers, runs[0].state.inliers)
        assert _essential_error(other.state.model, runs[0].state.model) < 1e-12


def test_the_5_point_needs_fewer_samples_and_refining_beats_the_floor() -> None:
    """The 5-point needs fewer samples; refining takes E below the 8-point floor.

    Over ten scenes with 1 px noise and 40% outliers (docs/notes/five-point.md): the 5-point
    solvers stop after fewer than half the 8-point's samples, and refining the
    consensus on its Sampson distances takes the median error below the floor (the 8-point
    fit to the true inliers) -- which the unrefined runs only reach.
    """
    truth = _true_essential()
    ratios: dict[tuple[str, bool], list[float]] = {}
    iterations: dict[str, list[int]] = {}
    for seed in range(10):
        b1, b2 = sc.pairs(sc.scene(200, seed=seed), sigma=1 / 520, seed=seed + 100)
        b2, clean = sc.with_outliers(b2, 0.4, seed=seed + 200)
        floor = _essential_error(epipolar.essential_8pt(b1[clean], b2[clean]), truth)
        options = ransac.Options(threshold=2 / 520, seed=seed)
        for solver in epipolar.MINIMAL_SOLVERS:
            for refine in (False, True):
                run = epipolar.essential_ransac(
                    b1, b2, options, minimal_solver=solver, refine=refine
                )
                ratio = _essential_error(run.state.model, truth) / floor
                ratios.setdefault((solver, refine), []).append(ratio)
                if not refine:
                    iterations.setdefault(solver, []).append(run.summary.iterations)
    for solver in ("nister", "stewenius"):
        assert np.median(iterations[solver]) < 0.5 * np.median(iterations["8pt"])
        assert np.median(ratios[solver, True]) < 0.8
        assert np.median(ratios[solver, True]) < np.median(ratios[solver, False])


def test_refining_recovers_the_true_essential_from_a_perturbed_start() -> None:
    """On exact data the Sampson cost's minimum is the true E: refining reaches it."""
    b1, b2 = sc.pairs(sc.scene(40, seed=1))
    truth = _true_essential()
    start = so3.hat(so3.exp([0.0, 0.02, 0.0]) @ sc.POSE[:3, 3]) @ so3.exp([0.01, 0.0, 0.0])
    for backend in sorted(backends.LOCKSTEP_BACKENDS):
        solved = epipolar.refine_essential(
            start @ sc.POSE[:3, :3], b1, b2, scale=1e-3, backend=backend
        )
        assert solved.summary.converged
        assert solved.summary.final_cost < 1e-20
        assert _essential_error(solved.state, truth) < 1e-9


def test_the_sampson_gradient_matches_finite_differences() -> None:
    """The analytic Jacobian's gradient ``Jᵀ r`` matches central differences of ``½‖r‖²``."""
    b1, b2 = sc.pairs(sc.scene(30, seed=2), sigma=1e-3, seed=3)
    problem = epipolar.SampsonProblem(
        correspondences.normalised_plane(b1), correspondences.normalised_plane(b2)
    )
    state = epipolar.start_motion(epipolar.essential_8pt(b1, b2) + 0.01 * np.eye(3))
    system = problem.linearize(state, np.ones(len(b1)))
    step = 1e-6
    numeric = np.zeros(5)
    for k in range(5):
        delta = np.zeros(5)
        delta[k] = step
        plus = problem.residuals(problem.retract(state, delta))
        minus = problem.residuals(problem.retract(state, -delta))
        numeric[k] = (0.5 * plus @ plus - 0.5 * minus @ minus) / (2.0 * step)
    np.testing.assert_allclose(system.gradient, numeric, rtol=1e-6, atol=1e-12)


def test_refining_matches_scipy_least_squares() -> None:
    """From RANSAC's consensus, the refinement reaches SciPy's minimum of the same problem.

    SciPy solves the same 5-DOF problem (the same retraction, Cauchy loss of the same scale)
    to tolerances of 1e-15. Ours stops when the cost's relative decrease is below its
    function tolerance; the cost is flat there, so the costs agree to 1e-6 and E to ~1e-5.
    """
    optimize = pytest.importorskip("scipy.optimize")
    for seed in range(3):
        b1, b2 = sc.pairs(sc.scene(120, seed=5 + seed), sigma=1 / 520, seed=6 + seed)
        b2, _ = sc.with_outliers(b2, 0.1, seed=7 + seed)
        scale = 2 / 520
        run = epipolar.essential_ransac(
            b1, b2, ransac.Options(threshold=scale, seed=seed), refine=False
        )
        b1, b2 = b1[run.state.inliers], b2[run.state.inliers]
        problem = epipolar.SampsonProblem(
            correspondences.normalised_plane(b1), correspondences.normalised_plane(b2)
        )
        origin = epipolar.start_motion(run.state.model)
        fitted = optimize.least_squares(
            lambda delta, p=problem, o=origin: p.residuals(p.retract(o, delta)),
            np.zeros(5),
            loss="cauchy",
            f_scale=scale,
            xtol=1e-15,
            ftol=1e-15,
            gtol=1e-15,
        )
        expected = problem.retract(origin, fitted.x).essential
        for backend in sorted(backends.LOCKSTEP_BACKENDS):
            solved = epipolar.refine_essential(
                run.state.model, b1, b2, scale=scale, backend=backend
            )
            assert solved.summary.converged
            assert solved.summary.final_cost == pytest.approx(fitted.cost, rel=1e-6)
            assert _essential_error(solved.state, expected) < 1e-4


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_refine_rejects_invalid_input(backend: str) -> None:
    """Too few pairs, a non-positive scale and a rank-1 E are rejected."""
    b1, b2 = sc.pairs(sc.scene(20))
    with pytest.raises(InvalidInputError, match="at least 5"):
        epipolar.refine_essential(np.eye(3), b1[:4], b2[:4], scale=1.0, backend=backend)
    with pytest.raises(InvalidInputError, match="scale"):
        epipolar.refine_essential(_true_essential(), b1, b2, scale=-1.0, backend=backend)
    with pytest.raises(DegenerateError, match="rank"):
        epipolar.refine_essential(np.diag([1.0, 0.0, 0.0]), b1, b2, scale=1.0, backend=backend)
    with pytest.raises(InvalidInputError, match="minimal_solver"):
        epipolar.essential_ransac(
            b1, b2, ransac.Options(threshold=1.0), minimal_solver="sturm", backend=backend
        )


@pytest.mark.filterwarnings("ignore:invalid value encountered in divide:RuntimeWarning")
@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_refine_rejects_a_match_at_both_epipoles(backend: str) -> None:
    """A match at both epipoles has a 0 / 0 Sampson distance: an error, not a crash.

    Forward translation with no rotation puts both epipoles exactly at (0, 0, 1), so E maps
    the match to two zero lines in floating point, not merely small ones.
    """
    points = np.array([[np.sin(1.7 * k), np.cos(0.9 * k), 4.0 + np.sin(0.3 * k)] for k in range(8)])
    moved = points + np.array([0.0, 0.0, 1.0])
    forward = np.array([0.0, 0.0, 1.0])
    b1 = np.vstack([points / np.linalg.norm(points, axis=1, keepdims=True), forward])
    b2 = np.vstack([moved / np.linalg.norm(moved, axis=1, keepdims=True), forward])
    essential = so3.hat(forward)
    with pytest.raises(InvalidInputError, match="initial cost is not finite"):
        epipolar.refine_essential(essential, b1, b2, scale=1e-3, backend=backend)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_the_compiled_code_tables_match(backend: str) -> None:
    """The codes Python sends and decodes are each compiled core's, in the same order."""
    chosen = require_function(backend, epipolar.__name__, "codes")
    codes = backends.implementation(chosen, epipolar.__name__).codes()
    sc.assert_refined_codes(codes)
    assert tuple(codes["minimal_solver"]) == epipolar.MINIMAL_SOLVERS


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_the_summary_reports_the_refinement(backend: str) -> None:
    """With refine on, the summary holds the refinement's solve and whether it was kept."""
    b1, b2 = sc.pairs(sc.scene(200, seed=1), sigma=1 / 520, seed=2)
    b2, _ = sc.with_outliers(b2, 0.3, seed=3)
    options = ransac.Options(threshold=2 / 520, seed=4)
    plain = epipolar.essential_ransac(b1, b2, options, refine=False, backend=backend)
    refined = epipolar.essential_ransac(b1, b2, options, backend=backend)
    assert plain.summary.refinement is None
    assert not plain.summary.refined
    assert refined.summary.refinement is not None
    assert refined.summary.refinement.converged
    assert refined.summary.refined
    assert refined.summary.run.score < plain.summary.run.score
    assert refined.summary.iterations == plain.summary.iterations


def test_start_motion_rejects_what_does_not_decompose() -> None:
    """A non-finite or rank-1 E has no start motion: errors, as in the compiled cores."""
    with pytest.raises(InvalidInputError):
        epipolar.start_motion(np.full((3, 3), np.nan))
    with pytest.raises(DegenerateError, match="rank"):
        epipolar.start_motion(np.diag([1.0, 0.0, 0.0]))
