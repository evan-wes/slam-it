"""PnP tests: EPnP, the pose refinement and PnP RANSAC, against truth, OpenCV and SciPy."""

from __future__ import annotations

import numpy as np
import pytest

from slam_it_all import backends
from slam_it_all.foundations import nls, p3p, pnp, ransac
from slam_it_all.foundations.errors import DegenerateError, InvalidInputError
from tests.conftest import load_fixture, require_function
from tests.foundations import scenes as sc

FIXTURE = load_fixture("foundations/pnp.json")
FOCAL = 520.0
"""The focal length the noise and thresholds are stated in, pixels."""


def _correspondences(
    seed: int, n: int, sigma_px: float = 0.0, outliers: float = 0.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return world points, their bearings, the true ``T_cam_world`` and the inlier mask.

    The points are drawn 2-8 m ahead of a random camera; `sigma_px` of pixel noise (at
    :data:`FOCAL`) perturbs the bearings, and a fraction `outliers` is replaced by random
    directions in the field of view.
    """
    rng = np.random.default_rng(seed)
    world, rays, pose = sc.posed(rng, n)
    if sigma_px > 0.0:
        rays = sc.noisy(rays, sigma_px / FOCAL, seed)
    mask = np.ones(n, dtype=bool)
    bad = rng.choice(n, int(outliers * n), replace=False)
    mask[bad] = False
    rays[bad] = sc.bearings(
        np.column_stack([rng.uniform(-0.6, 0.6, (len(bad), 2)), np.ones(len(bad))])
    )
    return world, rays, pose, mask


def _pose_error(found: np.ndarray, truth: np.ndarray) -> tuple[float, float]:
    """Return the rotation error (degrees) and the camera-centre error (metres)."""
    relative = found[:3, :3] @ truth[:3, :3].T
    angle = np.degrees(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))
    centre = -found[:3, :3].T @ found[:3, 3] + truth[:3, :3].T @ truth[:3, 3]
    return float(angle), float(np.linalg.norm(centre))


def _lengths(pose: np.ndarray, world: np.ndarray, image: np.ndarray) -> np.ndarray:
    """Return each correspondence's reprojection error length in the normalised plane."""
    in_camera = world @ pose[:3, :3].T + pose[:3, 3]
    return np.asarray(np.linalg.norm(in_camera[:, :2] / in_camera[:, 2:3] - image, axis=1))


# --- fixtures -----------------------------------------------------------------------------


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("function", ["epnp", "pnp_ransac"])
def test_matches_fixture(backend: str, function: str) -> None:
    """Each backend returns the 50-digit true pose (and, for RANSAC, the clean inliers)."""
    chosen = require_function(backend, pnp.__name__, function)
    sc.check_fixture(pnp, FIXTURE, function, chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_refinement_matches_fixture(backend: str) -> None:
    """On exact correspondences the refinement converges from a perturbed start to the truth."""
    chosen = require_function(backend, pnp.__name__, "refine_pose")
    sc.check_fixture(pnp, FIXTURE, "refine_pose", chosen)


@pytest.mark.parametrize("backend", backends.BACKENDS)
def test_ransac_fixture_records_whether_the_refinement_ran(backend: str) -> None:
    """A port that skips the refinement is caught even when its model is right."""
    chosen = require_function(backend, pnp.__name__, "pnp_ransac")
    for case in FIXTURE["functions"]["pnp_ransac"]["cases"]:
        inputs = dict(case["input"])
        options = ransac.Options(threshold=inputs.pop("threshold"), seed=inputs.pop("seed"))
        found = pnp.pnp_ransac(**inputs, options=options, backend=chosen)
        assert (found.summary.refinement is not None) == case["summary"]["refinement_ran"]


# --- EPnP ---------------------------------------------------------------------------------


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("n", [5, 6, 20, 200])
def test_epnp_is_exact_on_exact_data(backend: str, n: int) -> None:
    """From its minimum of five points up, EPnP returns the true pose of exact data."""
    chosen = require_function(backend, pnp.__name__, "epnp")
    for seed in range(50):
        world, rays, truth, _ = _correspondences(seed, n)
        np.testing.assert_allclose(pnp.epnp(world, rays, backend=chosen), truth, atol=1e-9)


def test_epnp_matches_opencv_on_noisy_data() -> None:
    """With 1 px of noise, EPnP's accuracy matches OpenCV's EPnP on the same correspondences."""
    cv2 = pytest.importorskip("cv2")
    ours, theirs = [], []
    for seed in range(40):
        world, rays, truth, _ = _correspondences(seed, 100, sigma_px=1.0)
        ours.append(_pose_error(pnp.epnp(world, rays), truth)[0])
        image = sc.plane_xy(rays)
        _, rvec, tvec = cv2.solvePnP(world, image, np.eye(3), None, flags=cv2.SOLVEPNP_EPNP)
        reference = np.eye(4)
        reference[:3, :3], reference[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
        theirs.append(_pose_error(reference, truth)[0])
    assert np.median(ours) < 1.5 * np.median(theirs)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_epnp_rejects_planar_points(backend: str) -> None:
    """On a plane the third control point is undetermined: a Degenerate error, not a guess."""
    chosen = require_function(backend, pnp.__name__, "epnp")
    world = sc.plane(20)
    with pytest.raises(DegenerateError, match="planar"):
        pnp.epnp(world, sc.bearings(world), backend=chosen)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_epnp_rejects_too_few_points(backend: str) -> None:
    """Four correspondences are below EPnP's minimum (its linearisations are approximate)."""
    chosen = require_function(backend, pnp.__name__, "epnp")
    world, rays, _, _ = _correspondences(0, 4)
    with pytest.raises(InvalidInputError, match="at least 5"):
        pnp.epnp(world, rays, backend=chosen)


# --- the refinement -----------------------------------------------------------------------


@pytest.mark.parametrize("weighted", [False, True])
def test_refining_matches_scipy_least_squares(weighted: bool) -> None:
    """From RANSAC's consensus, the refinement reaches SciPy's minimum of the same problem.

    SciPy solves the same 6-DOF problem (the same retraction, a Cauchy loss of the same scale)
    to tolerances of 1e-15; ours stops on its function tolerance, where the cost is flat. SciPy
    applies its loss to each residual, ours to each block's squared norm, so SciPy is given one
    residual per correspondence, the reprojection error's length: the same cost. Weighted, as
    the edge rung weights keypoints by their pyramid level, each length is scaled by its
    weight's square root.
    """
    optimize = pytest.importorskip("scipy.optimize")
    scale = 2.0 / FOCAL
    for seed in range(3):
        world, rays, _, _ = _correspondences(seed, 150, sigma_px=1.0, outliers=0.2)
        run = pnp.pnp_ransac(world, rays, ransac.Options(threshold=scale, seed=seed), refine=False)
        world, rays = world[run.state.inliers], rays[run.state.inliers]
        problem = pnp.ReprojectionProblem(world, sc.plane_xy(rays))
        origin = run.state.model

        image = sc.plane_xy(rays)
        levels = np.random.default_rng(seed).integers(0, 8, len(world))
        weights = 1.2 ** (-2.0 * levels) if weighted else np.ones(len(world))
        root = np.sqrt(weights)
        fitted = optimize.least_squares(
            lambda delta, o=origin, w=world, x=image, r=root: (
                r * _lengths(pnp.ReprojectionProblem.retract(o, delta), w, x)
            ),
            np.zeros(6),
            loss="cauchy",
            f_scale=scale,
            xtol=1e-15,
            ftol=1e-15,
            gtol=1e-15,
        )
        expected = problem.retract(origin, fitted.x)
        for backend in sorted(backends.LOCKSTEP_BACKENDS):
            solved = pnp.refine_pose(
                origin, world, rays, scale=scale, weights=weights, backend=backend
            )
            assert solved.summary.converged
            assert solved.summary.final_cost == pytest.approx(fitted.cost, rel=1e-6)
            np.testing.assert_allclose(solved.state, expected, atol=1e-5)


def test_the_reprojection_jacobian_matches_finite_differences() -> None:
    """The analytic Jacobian is the numeric one of the residuals through the retraction."""
    world, rays, truth, _ = _correspondences(3, 12, sigma_px=1.0)
    problem = pnp.ReprojectionProblem(world, sc.plane_xy(rays))
    start = problem.retract(truth, np.array([0.01, -0.02, 0.03, 0.01, 0.02, -0.01]))

    def residuals(pose: np.ndarray) -> np.ndarray:
        in_camera = world @ pose[:3, :3].T + pose[:3, 3]
        return np.asarray((in_camera[:, :2] / in_camera[:, 2:3] - sc.plane_xy(rays)).ravel())

    numeric = nls.numeric_jacobian(residuals, start, problem.retract, 6)
    ones = np.ones(len(world))
    analytic = problem.linearize(start, ones)
    np.testing.assert_allclose(analytic.hessian, numeric.T @ numeric, rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(
        analytic.gradient, numeric.T @ residuals(start), rtol=1e-6, atol=1e-10
    )


def test_a_point_behind_the_camera_has_an_infinite_cost() -> None:
    """The refinement's cost, like RANSAC's, is infinite for a point behind the camera.

    Its projection ``x / z`` with both negative can land on the observation, so without this
    an LM step could move a point behind the camera at no cost.
    """
    world, rays, truth, _ = _correspondences(0, 10)
    mirrored = world.copy()
    mirrored[0] = 2.0 * (-truth[:3, :3].T @ truth[:3, 3]) - world[0]
    problem = pnp.ReprojectionProblem(mirrored, sc.plane_xy(rays))
    norms = problem.squared_norms(truth)
    assert norms[0] == np.inf
    assert np.all(norms[1:] < 1e-20)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_refine_rejects_invalid_input(backend: str) -> None:
    """A pose that is not rigid, too few points, a bad scale or weight, a point behind."""
    world, rays, truth, _ = _correspondences(0, 10)
    stretched = truth.copy()
    stretched[:3, :3] *= 2.0
    with pytest.raises(InvalidInputError, match="rotation"):
        pnp.refine_pose(stretched, world, rays, scale=1e-3, backend=backend)
    with pytest.raises(InvalidInputError, match="at least 3"):
        pnp.refine_pose(truth, world[:2], rays[:2], scale=1e-3, backend=backend)
    with pytest.raises(InvalidInputError, match="scale"):
        pnp.refine_pose(truth, world, rays, scale=0.0, backend=backend)
    for weights in (np.zeros(len(world)), np.ones(len(world) - 1), np.full(len(world), np.inf)):
        with pytest.raises(InvalidInputError, match="weights"):
            pnp.refine_pose(truth, world, rays, scale=1e-3, weights=weights, backend=backend)
    behind = truth.copy()
    behind[2, 3] -= 20.0
    with pytest.raises(InvalidInputError, match="behind"):
        pnp.refine_pose(behind, world, rays, scale=1e-3, backend=backend)


# --- RANSAC -------------------------------------------------------------------------------


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize("solver", p3p.SOLVERS)
def test_ransac_recovers_the_pose_through_outliers(backend: str, solver: str) -> None:
    """With 40% outliers and 1 px of noise the pose is within the noise.

    The inliers are the clean correspondences, bar a few the noise pushes over the threshold.
    """
    chosen = require_function(backend, pnp.__name__, "pnp_ransac")
    for seed in range(5):
        world, rays, truth, mask = _correspondences(seed, 200, sigma_px=1.0, outliers=0.4)
        found = pnp.pnp_ransac(
            world, rays, ransac.Options(threshold=3.0 / FOCAL, seed=seed),
            minimal_solver=solver, backend=chosen,
        )  # fmt: skip
        rotation, centre = _pose_error(found.state.model, truth)
        assert rotation < 0.2
        assert centre < 0.02
        assert not np.any(found.state.inliers & ~mask)
        assert np.count_nonzero(found.state.inliers) >= 0.95 * np.count_nonzero(mask)
        assert found.summary.converged


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_ransac_on_a_planar_scene_reaches_the_floor(backend: str) -> None:
    """On a plane, refining and re-collecting the inliers brings the pose to the noise floor.

    EPnP refuses planar points, so LO's refits cannot run; one refinement left the pose 2x off
    the floor (p90 8x). The floor is the refinement started at the true pose on the true inliers.
    """
    chosen = require_function(backend, pnp.__name__, "pnp_ransac")
    threshold = 2.0 / FOCAL
    ratios = []
    for seed in range(10):
        world, rays, truth, mask = _correspondences(seed, 200, sigma_px=1.0, outliers=0.4)
        in_camera = sc.in_camera(truth, world)
        in_camera[:, 2] = in_camera[:, 2].mean()
        world = (in_camera - truth[:3, 3]) @ truth[:3, :3]
        rays[mask] = sc.noisy(sc.bearings(in_camera[mask]), 1.0 / FOCAL, seed)
        floor = pnp.refine_pose(truth, world[mask], rays[mask], scale=threshold).state
        found = pnp.pnp_ransac(
            world, rays, ransac.Options(threshold=threshold, seed=seed), backend=chosen
        )
        ratios.append(_pose_error(found.state.model, truth)[0] / _pose_error(floor, truth)[0])
    # The median is the robust bound: over 40 seeds the worst ratio is 6.4 (panel L9).
    assert np.median(ratios) < 1.6


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_the_refinement_lowers_the_error(backend: str) -> None:
    """Refining the consensus lowers the median rotation error below LO's EPnP refits'."""
    chosen = require_function(backend, pnp.__name__, "pnp_ransac")
    errors: dict[bool, list[float]] = {True: [], False: []}
    for seed in range(20):
        world, rays, truth, _ = _correspondences(seed, 150, sigma_px=1.0, outliers=0.3)
        for refine in (True, False):
            found = pnp.pnp_ransac(
                world, rays, ransac.Options(threshold=3.0 / FOCAL, seed=seed), refine=refine,
                backend=chosen,
            )  # fmt: skip
            errors[refine].append(_pose_error(found.state.model, truth)[0])
    assert np.median(errors[True]) < np.median(errors[False])


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_backends_agree_on_ransac(backend: str) -> None:
    """Every compiled core draws the same samples: same inliers, iterations, refinement, pose."""
    chosen = require_function(backend, pnp.__name__, "pnp_ransac")
    for seed in range(5):
        world, rays, _, _ = _correspondences(seed, 120, sigma_px=1.0, outliers=0.3)
        options = ransac.Options(threshold=2.0 / FOCAL, seed=seed)
        reference = pnp.pnp_ransac(world, rays, options)
        compiled = pnp.pnp_ransac(world, rays, options, backend=chosen)
        np.testing.assert_array_equal(compiled.state.inliers, reference.state.inliers)
        run, expected = compiled.summary.run, reference.summary.run
        assert (run.iterations, run.termination, run.inlier_count) == (
            expected.iterations,
            expected.termination,
            expected.inlier_count,
        )
        assert run.score == pytest.approx(expected.score, rel=1e-12)
        assert compiled.summary.refined == reference.summary.refined
        assert compiled.summary.rounds == reference.summary.rounds
        np.testing.assert_allclose(compiled.state.model, reference.state.model, atol=1e-10)


def test_a_point_behind_the_camera_is_never_an_inlier() -> None:
    """The residual of a point behind the camera is infinite, whatever its bearing."""
    world, rays, truth, _ = _correspondences(0, 10)
    flipped = world.copy()
    flipped[0] = 2.0 * (-truth[:3, :3].T @ truth[:3, 3]) - world[0]  # mirrored through the centre
    errors = pnp.reprojection_errors(truth, flipped, rays)
    assert errors[0] == np.inf
    assert np.all(errors[1:] < 1e-20)


@pytest.mark.parametrize("backend", backends.COMPILED_BACKENDS)
def test_the_compiled_codes_match(backend: str) -> None:
    """The solver codes Python sends are P3P's; the termination codes RANSAC's and NLS'."""
    chosen = require_function(backend, pnp.__name__, "pnp_ransac")
    codes = backends.implementation(chosen, pnp.__name__).codes()
    assert tuple(codes["minimal_solver"]) == p3p.SOLVERS
    sc.assert_refined_codes(codes)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_ransac_rejects_invalid_input(backend: str) -> None:
    """An unknown solver, two correspondences, or a bearing behind the camera."""
    world, rays, _, _ = _correspondences(0, 20)
    options = ransac.Options(threshold=1e-3)
    with pytest.raises(InvalidInputError, match="minimal_solver"):
        pnp.pnp_ransac(world, rays, options, minimal_solver="grunert", backend=backend)
    with pytest.raises(InvalidInputError, match="at least 3"):
        pnp.pnp_ransac(world[:2], rays[:2], options, backend=backend)
    with pytest.raises(InvalidInputError, match="positive z"):
        pnp.pnp_ransac(world, -rays, options, backend=backend)
