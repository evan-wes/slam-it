"""5-point tests: the true E among the solutions, the two solvers agree, PoseLib as an oracle."""

from __future__ import annotations

import numpy as np
import pytest

from slamlab import backends
from slamlab.foundations import fivepoint, so3
from slamlab.foundations.errors import InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

TRIALS = 50
FIXTURE = load_fixture("foundations/fivepoint.json")


def _sample(seed: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return five exact bearing pairs under a random motion, and the true E (unit t)."""
    rng = np.random.default_rng(seed)
    pose = sc.random_pose(rng, 0.3)
    b1, b2 = sc.pairs(sc.scene(5, seed=seed), pose=pose)
    t = pose[:3, 3] / np.linalg.norm(pose[:3, 3])
    return b1, b2, so3.hat(t) @ pose[:3, :3]


def _distance(found: np.ndarray, expected: np.ndarray) -> float:
    """Return the closest solution's distance to `expected`, both up to sign."""
    if len(found) == 0:
        return np.inf
    return float(min(min(np.abs(e - expected).max(), np.abs(e + expected).max()) for e in found))


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_matches_fixture(backend: str) -> None:
    """Each backend finds the 50-digit true E among its solutions, up to sign."""
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    spec = FIXTURE["functions"]["essential_5pt"]
    for case in spec["cases"]:
        found = fivepoint.essential_5pt(**case["input"], backend=chosen)
        for expected in case["expected"]:
            assert _distance(found, np.asarray(expected)) <= spec["tolerance"]["atol"]


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("solver", fivepoint.SOLVERS)
def test_the_true_essential_is_among_the_solutions(backend: str, solver: str) -> None:
    """On exact data, one solution is the true E; every solution is essential and fits."""
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    for seed in range(TRIALS):
        b1, b2, truth = _sample(seed)
        found = fivepoint.essential_5pt(b1, b2, solver=solver, backend=chosen)
        assert 1 <= len(found) <= 10
        assert _distance(found, truth) < 1e-8
        for e in found:
            singular = np.linalg.svd(e, compute_uv=False)
            np.testing.assert_allclose(singular, [1.0, 1.0, 0.0], atol=1e-8)
            residuals = np.einsum("ij,jk,ik->i", b2, e, b1)
            assert np.abs(residuals).max() < 1e-9


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_nister_and_stewenius_find_the_same_solutions(backend: str) -> None:
    """The polynomial and the action matrix are two routes to the same ten-degree system."""
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    for seed in range(TRIALS):
        b1, b2, _ = _sample(seed)
        nister = fivepoint.essential_5pt(b1, b2, solver="nister", backend=chosen)
        stewenius = fivepoint.essential_5pt(b1, b2, solver="stewenius", backend=chosen)
        assert len(nister) == len(stewenius)
        for e in nister:
            assert _distance(stewenius, e) < 1e-9


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
@pytest.mark.parametrize("solver", fivepoint.SOLVERS)
def test_backends_agree(backend: str, solver: str) -> None:
    """Every compiled core returns the reference's solutions, signed and ordered alike.

    The canonical order (the module's "Canonical order") makes the order independent of each
    library's null-space basis and Schur form, so the lists match element for element.
    """
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    for seed in range(TRIALS):
        b1, b2, _ = _sample(seed)
        reference = fivepoint.essential_5pt(b1, b2, solver=solver)
        compiled = fivepoint.essential_5pt(b1, b2, solver=solver, backend=chosen)
        assert compiled.shape == reference.shape
        np.testing.assert_allclose(compiled, reference, rtol=0, atol=1e-10)


def test_poselib_solutions_are_among_ours() -> None:
    """Every accurate solution PoseLib's 5-point solver returns is one of ours.

    Any matrix in the sample's null space satisfies the five epipolar equations; what makes it
    a solution is the cubic constraint ``2 E Eᵀ E - tr(E Eᵀ) E = 0``. PoseLib is less precise
    than our polished solvers (some of its matrices miss that constraint by ~1e-3), so only
    the ones that satisfy it are compared.
    """
    poselib = pytest.importorskip("poselib")
    compared = 0
    for seed in range(TRIALS):
        b1, b2, _ = _sample(seed)
        ours = fivepoint.essential_5pt(b1, b2)
        for e in poselib.essential_matrix_5pt(b1, b2):
            e_unit = np.asarray(e) * np.sqrt(2.0) / np.linalg.norm(e)
            demazure = 2.0 * e_unit @ e_unit.T @ e_unit - np.trace(e_unit @ e_unit.T) * e_unit
            if np.abs(demazure).max() > 1e-9:
                continue
            compared += 1
            assert _distance(ours, e_unit) < 1e-8
    assert compared >= 2 * TRIALS


def test_real_roots_matches_numpy() -> None:
    """Sturm sequences and bisection find exactly the real roots numpy's companion matrix does."""
    roots = [-2.0, 0.5, 1.0, 3.5]
    poly = np.polynomial.polynomial.polyfromroots(roots)
    poly = np.polynomial.polynomial.polymul(poly, [1.0, 0.0, 1.0])  # z^2 + 1: no real roots
    np.testing.assert_allclose(fivepoint.real_roots(poly), roots, atol=1e-12)
    assert len(fivepoint.real_roots([1.0, 0.0, 1.0])) == 0


@pytest.mark.parametrize("poly", [[np.nan, 1.0], [1.0, np.inf, 1.0], [[1.0, 2.0]]])
def test_real_roots_rejects_bad_coefficients(poly: list[float]) -> None:
    """Non-finite or non-1-D coefficients are an error, not "no real roots"."""
    with pytest.raises(InvalidInputError, match="poly"):
        fivepoint.real_roots(poly)


def _near_degenerate(seed: int, baseline: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return five exact pairs under mostly forward motion with a small baseline, and E."""
    rng = np.random.default_rng(seed)
    depth = rng.uniform(2.0, 8.0, 5)
    points = np.column_stack([rng.uniform(-0.5, 0.5, (5, 2)) * depth[:, None], depth])
    rotation = so3.exp(rng.normal(scale=0.02, size=3))
    direction = np.array([0.0, 0.0, 1.0]) + rng.normal(scale=0.2, size=3)
    t = direction / np.linalg.norm(direction)
    b1 = sc.bearings(points)
    b2 = sc.bearings(points @ rotation.T + baseline * t)
    return b1, b2, so3.hat(t) @ rotation


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_near_degenerate_samples_keep_the_true_essential(backend: str) -> None:
    """Forward motion at a 5% baseline (docs/notes/five-point.md, "Near degeneracy").

    On 100 exact samples, Stewénius finds the true E on all but at most two, Nistér on all but
    at most twelve: its degree-10 polynomial can round a real root into a complex pair. Nistér's
    count is measured (6 in Python, 7 in C++) and depends on rounding, so on the platform and
    compiler; the bound leaves room for that.
    """
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    misses = dict.fromkeys(fivepoint.SOLVERS, 0)
    for seed in range(100):
        b1, b2, truth = _near_degenerate(seed, 0.05)
        for solver in fivepoint.SOLVERS:
            found = fivepoint.essential_5pt(b1, b2, solver=solver, backend=chosen)
            misses[solver] += _distance(found, truth) > 1e-4
    assert misses["stewenius"] <= 2
    assert misses["nister"] <= 12


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("solver", fivepoint.SOLVERS)
def test_a_repeated_pair_gives_no_solution(backend: str, solver: str) -> None:
    """Five pairs with one repeated leave a rank-4 system: no isolated solution, none returned."""
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    b1, b2, _ = _sample(0)
    b1[4], b2[4] = b1[3], b2[3]
    assert fivepoint.essential_5pt(b1, b2, solver=solver, backend=chosen).shape == (0, 3, 3)


def test_the_compiled_solver_codes_match() -> None:
    """The solver codes Python sends are the compiled core's."""
    chosen = require_function("cpp", fivepoint.__name__, "essential_5pt")
    codes = backends.implementation(chosen, fivepoint.__name__).codes()
    assert tuple(codes["solver"]) == fivepoint.SOLVERS


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_a_pure_rotation_returns_only_essential_matrices(backend: str) -> None:
    """With no baseline every ``[t̂]ₓ R`` fits: the roots are arbitrary, but none is junk.

    Without the essential check, some of these samples return matrices with singular values
    like (1.19, 0.71, 0.30).
    """
    chosen = require_function(backend, fivepoint.__name__, "essential_5pt")
    for seed in range(10):
        b1 = sc.bearings(sc.scene(5, seed=seed))
        b2 = b1 @ so3.exp([0.0, 0.1, 0.02]).T
        for solver in fivepoint.SOLVERS:
            for e in fivepoint.essential_5pt(b1, b2, solver=solver, backend=chosen):
                singular = np.linalg.svd(e, compute_uv=False)
                np.testing.assert_allclose(
                    singular, [1.0, 1.0, 0.0], atol=fivepoint.ESSENTIAL_TOLERANCE
                )
                assert np.abs(np.einsum("ij,jk,ik->i", b2, e, b1)).max() < 1e-6


def test_rejects_invalid_input() -> None:
    """Other than five unit pairs, or an unknown solver, is rejected."""
    b1, b2, _ = _sample(0)
    with pytest.raises(InvalidInputError, match="solver"):
        fivepoint.essential_5pt(b1, b2, solver="sturm")
    with pytest.raises(InvalidInputError):
        fivepoint.essential_5pt(b1[:4], b2[:4])
    with pytest.raises(InvalidInputError):
        fivepoint.essential_5pt(np.vstack([b1, b1[:1]]), np.vstack([b2, b2[:1]]))
