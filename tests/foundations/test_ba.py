"""Tests for slamlab.foundations.ba: the Schur system, the optimum, gauge, degeneracy, parity."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray
from scipy.optimize import least_squares

from slamlab import backends
from slamlab.foundations import ba, camera, nls, se3
from slamlab.foundations.ba.bundle import Bundle, Kind, State
from slamlab.foundations.ba.layout import make_layout
from slamlab.foundations.ba.schur import Reduction
from slamlab.foundations.ba.solve import ENUM_TABLES, MODULE, backend_arguments
from slamlab.foundations.errors import InvalidInputError, NotConvergedError
from tests.conftest import assert_matches_fixture, load_fixture, require_backend, require_function
from tests.fixtures.foundations.generate_ba import options_from, problem_from

Array = NDArray[np.float64]

PINHOLE = np.array([500.0, 505.0, 320.0, 240.0])
RADTAN = np.array([500.0, 505.0, 320.0, 240.0, -0.08, 0.01, 0.001, -0.0005, 0.0])
TIGHT = nls.Options(max_iterations=100, function_tolerance=1e-12, gradient_tolerance=1e-10)
"""Tolerances tight enough that a solve reaches the optimum's rounding floor."""

TO_FLOOR = replace(TIGHT, function_tolerance=0.0, max_iterations=300)
"""Run until the step itself is negligible: for comparing optima in badly conditioned scenes,
where a 1e-12 relative cost decrease still leaves a gradient near 1e-4 along depth."""


@dataclass(frozen=True)
class Scene:
    """A synthetic scene: the truth and a perturbed problem built from it."""

    poses: Array
    points: Array
    intrinsics: Array
    inverse_depth: Array
    """Each observation's true inverse depth, for building depth priors."""
    problem: ba.BAProblem


def _scene(
    *,
    cameras: int = 5,
    points: int = 60,
    noise: float = 0.0,
    perturb: float = 0.01,
    model: str = "pinhole",
    seed: int = 0,
    rotation: float = 0.04,
    groups: Array | None = None,
    **create: object,
) -> Scene:
    """Build cameras along a gentle arc looking at a box of points, with every point seen.

    ``groups`` overrides the problem's starting intrinsics (the truth projects with one);
    ``rotation`` is the spread of the cameras' orientations (0 for pure translation).
    """
    rng = np.random.default_rng(seed)
    intrinsics = PINHOLE if model == "pinhole" else RADTAN
    world = rng.uniform([-2.0, -2.0, 5.0], [2.0, 2.0, 9.0], (points, 3))
    twists = np.concatenate(
        [
            np.stack(
                [0.4 * np.arange(cameras), 0.1 * np.sin(np.arange(cameras)), np.zeros(cameras)], 1
            ),
            rng.normal(0.0, 0.04, (cameras, 3)) * (rotation / 0.04),
        ],
        axis=1,
    )
    poses = se3.exp(twists)
    t_cam_world = se3.inverse(poses)
    in_camera = (
        np.einsum("cij,nj->cni", t_cam_world[:, :3, :3], world) + t_cam_world[:, None, :3, 3]
    )
    pixels = camera.project(model, intrinsics, in_camera.reshape(-1, 3))
    pixels = pixels + rng.normal(0.0, noise, pixels.shape) if noise else pixels
    cam_idx, pt_idx = np.divmod(np.arange(cameras * points), points)
    start_poses = se3.compose(poses, se3.exp(rng.normal(0.0, perturb, (cameras, 6))))
    start_points = world + rng.normal(0.0, 10 * perturb, world.shape)
    problem = ba.BAProblem.create(
        model,
        intrinsics if groups is None else groups,
        start_poses,
        start_points,
        pixels,
        cam_idx,
        pt_idx,
        **create,  # type: ignore[arg-type]
    )
    inverse_depth = 1.0 / in_camera.reshape(-1, 3)[:, 2]
    return Scene(poses, world, intrinsics, inverse_depth, problem)


FIXTURE = load_fixture("foundations/ba.json")
OUTPUTS = ("poses", "intrinsics", "points", "depth_affine", "inverse_depth", "intrinsics_std")


@pytest.mark.parametrize("backend", backends.BACKENDS)
@pytest.mark.parametrize("case", FIXTURE["cases"], ids=[c["name"] for c in FIXTURE["cases"]])
def test_matches_fixture(backend: str, case: dict[str, Any]) -> None:
    """Each backend reproduces the reference's solve: outputs, costs, codes and counts.

    The cases cover every gauge, Gauss-Newton, robust losses, degenerate drops, the radial
    model, priors and anchored points (see ``generate_ba.py``).
    """
    chosen = require_function(backend, MODULE, "solve")
    problem = problem_from(case["input"]["problem"])
    result = ba.solve(problem, options_from(case["input"]["options"]), backend=chosen)
    expected, tolerance = case["expected"], FIXTURE["tolerance"]
    for name in OUTPUTS:
        assert_matches_fixture(getattr(result, name), _nan(expected[name]), tolerance[name])
    summary = result.summary
    for name, value in (
        ("initial_cost", summary.solve.initial_cost),
        ("final_cost", summary.solve.final_cost),
        ("variance_factor", summary.variance_factor),
    ):
        assert_matches_fixture(value, _nan(expected[name]), tolerance["cost"])
    codes = {
        "gauge": summary.gauge,
        "termination": str(summary.solve.termination),
        "iterations": summary.solve.iterations,
        **{name: getattr(summary, name) for name in ba.COUNT_NAMES},
    }
    assert tolerance["codes"]["comparison"] == "exact"
    assert codes == expected["codes"]


def _nan(value: object) -> Array:
    """Return a fixture value as floats, its nulls as NaN."""
    return np.array(value, dtype=np.float64)


def _dense_normal_equations(bundle: Bundle, state: State, weights: Array) -> tuple[Array, Array]:
    """Assemble the full weighted normal equations ``(H, g)`` in the free coordinates."""
    reduction = bundle.reduction(state)
    real = bundle._real
    hessian = 0.0
    gradient = 0.0
    splits = np.cumsum([k.residuals.shape[0] for k in bundle.kinds(state)])[:-1]
    for kind, w in zip(bundle.kinds(state), np.split(weights, splits), strict=True):
        jacobian = _kind_jacobian(kind, reduction, real)
        weighted = jacobian * w[:, None, None]
        hessian = hessian + np.einsum("mki,mkj->ij", weighted, jacobian)
        gradient = gradient + np.einsum("mki,mk->i", weighted, kind.residuals)
    return np.asarray(hessian), np.asarray(gradient)


def _kind_jacobian(kind: Kind, reduction: Reduction, real: NDArray[np.bool_]) -> Array:
    """Return one kind of block's Jacobian in the free coordinates, shape (M, d, n).

    ``real`` marks the point dimensions that are variables (an anchored point has one).
    """
    m, d = kind.residuals.shape
    full = np.zeros((m, d, reduction.full_size))
    np.add.at(
        full,
        (np.arange(m)[:, None, None], np.arange(d)[None, :, None], kind.columns[:, None, :]),
        kind.camera_side,
    )
    cam = np.concatenate(
        [full[:, :, reduction.free], full[:, :, reduction.rows] @ reduction.basis], axis=2
    )
    pts = np.zeros((m, d, real.shape[0]))
    if kind.point is not None:
        on = np.flatnonzero(kind.slots >= 0)
        for k in range(3):
            pts[on, :, 3 * kind.slots[on] + k] = kind.point[on, :, k]
    return np.concatenate([cam, pts[:, :, real]], axis=2)


def _residual_vector(bundle: Bundle, state: State) -> Array:
    """Return every block's weighted residual rows, stacked."""
    return np.concatenate([part.reshape(-1) for part in bundle.residuals(state)])


def _bundle(problem: ba.BAProblem, options: ba.BAOptions | None = None) -> tuple[Bundle, State]:
    """Return the reference's NLS problem and its start state."""
    layout = make_layout(problem, options or ba.BAOptions())
    bundle = Bundle(problem, layout, ba.DenseCholesky())
    return bundle, bundle.start()


def _every_kind(scene: Scene) -> ba.BAProblem:
    """Return a scene's problem with depth priors and pose priors added.

    Half the observations get a depth prior, with free affines on two cameras; two cameras get
    a pose prior, one of them with no rotation weight; every third point is anchored as an
    inverse depth (one of them at an observation that also carries a depth prior).
    """
    p = scene.problem
    n_cams = p.poses.shape[0]
    prior = np.where(np.arange(p.cam_idx.size) % 2 == 0, scene.inverse_depth * 1.02, np.nan)
    pose_weights = np.zeros((n_cams, 2))
    pose_weights[1] = [4.0, 0.0]
    pose_weights[2] = [1.0, 9.0]
    return replace(
        p,
        depth_prior=prior,
        depth_prior_weights=np.full(p.cam_idx.size, 25.0),
        fixed_depth_affine=np.arange(n_cams) >= 2,
        depth_affine=np.tile([1.1, 0.01], (n_cams, 1)),
        pose_prior=se3.compose(scene.poses, se3.exp(np.full((n_cams, 6), 0.01))),
        pose_prior_weights=pose_weights,
        point_anchor=np.where(
            np.arange(p.points.shape[0]) % 3 == 0, ba.anchor_points(p).point_anchor, -1
        ),
    )


def test_schur_step_matches_dense_solve() -> None:
    """Eliminating the points gives the same damped step as solving the full system.

    Every kind of block takes part: reprojection, depth prior and pose prior.
    """
    scene = _scene(noise=0.5, model="radtan", fixed_intrinsics=[False])
    problem = _every_kind(scene)
    bundle, state = _bundle(problem, ba.BAOptions(gauge="similarity"))
    loss = nls.BlockLoss(
        (nls.Loss("huber", 2.0), nls.Loss("cauchy", 0.5), nls.Loss()), bundle.loss_index
    )
    weights = loss.evaluate(bundle.squared_norms(state))[1]
    schur = bundle.linearize(state, weights)
    hessian, gradient = _dense_normal_equations(bundle, state, weights)
    damping = 1e-3 * np.clip(np.diagonal(hessian), 1e-6, 1e32)
    np.testing.assert_allclose(schur.gradient, gradient, rtol=1e-12, atol=1e-9)
    np.testing.assert_allclose(schur.hessian_diagonal, np.diagonal(hessian), rtol=1e-12)
    step_schur = schur.solve(damping)
    step_dense = np.linalg.solve(hessian + np.diag(damping), -gradient)
    assert step_schur is not None
    np.testing.assert_allclose(
        step_schur, step_dense, rtol=0, atol=1e-10 * np.abs(step_dense).max()
    )


def test_jacobian_matches_numeric() -> None:
    """The analytic Jacobian of every kind of block matches central differences."""
    scene = _scene(cameras=3, points=8, model="radtan", fixed_intrinsics=[False])
    bundle, state = _bundle(_every_kind(scene), ba.BAOptions(gauge="similarity"))
    reduction = bundle.reduction(state)
    real = bundle._real
    analytic = np.concatenate(
        [
            _kind_jacobian(k, reduction, real).reshape(-1, reduction.size + int(real.sum()))
            for k in bundle.kinds(state)
        ]
    )
    numeric = nls.numeric_jacobian(
        lambda s: _residual_vector(bundle, s), state, bundle.retract, analytic.shape[1]
    )
    scale = np.abs(analytic).max(axis=0)
    np.testing.assert_allclose(analytic / scale, numeric / scale, rtol=0, atol=1e-6)


def test_noise_free_scene_recovers_truth() -> None:
    """From a perturbed start, noise-free observations bring back the true structure."""
    scene = _scene()
    result = ba.solve(scene.problem, ba.BAOptions(solver=TIGHT))
    assert result.summary.solve.converged
    assert result.summary.solve.final_cost < 1e-18
    projected = camera.project(
        "pinhole",
        PINHOLE,
        np.einsum("cij,nj->cni", se3.inverse(result.poses)[:, :3, :3], result.points).reshape(-1, 3)
        + np.repeat(se3.inverse(result.poses)[:, :3, 3], scene.points.shape[0], axis=0),
    )
    np.testing.assert_allclose(projected, scene.problem.observations, rtol=0, atol=1e-8)


def test_gradient_vanishes_and_chi_squared_matches_noise() -> None:
    """At convergence the gradient is ~0, and 2·cost/dof ≈ σ² for σ = 1 px noise."""
    scene = _scene(cameras=6, points=120, noise=1.0, seed=3)
    result = ba.solve(scene.problem, ba.BAOptions(solver=TIGHT))
    summary = result.summary.solve
    assert summary.converged
    final_gradient = summary.trace[-1].gradient_norm if summary.trace else 0.0
    assert final_gradient < 1e-6 * summary.trace[0].gradient_norm
    n_obs = scene.problem.observations.shape[0]
    dof = 2 * n_obs - (6 * (6 - 1) - 1 + 3 * 120)
    assert 0.85 < 2.0 * summary.final_cost / dof < 1.15


def _offsets_problem(cameras: int, points: int) -> tuple[ba.BAProblem, NDArray[np.bool_]]:
    """Return a noisy scene whose first two poses are fixed, so the gauge needs no help."""
    fixed = np.arange(cameras) < 2
    return _scene(
        cameras=cameras, points=points, noise=1.0, seed=5, fixed_poses=fixed
    ).problem, fixed


def _offset_residuals(problem: ba.BAProblem, fixed: NDArray[np.bool_], x: Array) -> Array:
    """Reprojection errors (M, 2) with free poses ``T0 · Exp(δ)`` and points ``X0 + d``.

    Written independently of the module: plain numpy plus the camera model.
    """
    t0 = se3.inverse(problem.poses)
    free = np.flatnonzero(~fixed)
    t_cw = t0.copy()
    t_cw[free] = se3.compose(t0[free], se3.exp(x[: 6 * free.size].reshape(-1, 6)))
    pts = problem.points + x[6 * free.size :].reshape(-1, 3)
    cam, pt = problem.cam_idx, problem.pt_idx
    in_cam = np.einsum("mij,mj->mi", t_cw[cam, :3, :3], pts[pt]) + t_cw[cam, :3, 3]
    return camera.project("pinhole", PINHOLE, in_cam) - problem.observations


def _offsets_of(problem: ba.BAProblem, fixed: NDArray[np.bool_], result: ba.BAResult) -> Array:
    """Express a solution as offsets from the start (the inverse of ``_offset_residuals``)."""
    free = np.flatnonzero(~fixed)
    t0, t1 = se3.inverse(problem.poses[free]), se3.inverse(result.poses[free])
    twists = se3.log(se3.compose(se3.inverse(t0), t1))
    return np.concatenate([twists.reshape(-1), (result.points - problem.points).reshape(-1)])


def test_least_squares_optimum_matches_scipy() -> None:
    """With the trivial loss, the optimum matches SciPy's trf over ``T0 · Exp(δ)``.

    SciPy is the independent optimiser here; with a robust loss it is the wrong oracle (on
    this scene its trf exhausts its evaluations at a cost 3x ours), so the robust optimum is
    checked by its own gradient below instead.
    """
    problem, fixed = _offsets_problem(cameras=4, points=12)
    ours = ba.solve(problem, ba.BAOptions(solver=TO_FLOOR, gauge="none"))
    x0 = np.zeros(6 * int(np.count_nonzero(~fixed)) + 3 * problem.points.shape[0])
    scipy = least_squares(
        lambda x: _offset_residuals(problem, fixed, x).reshape(-1),
        x0,
        method="trf",
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    assert ours.summary.solve.converged
    np.testing.assert_allclose(ours.summary.solve.final_cost, scipy.cost, rtol=1e-9)
    # Compared where the data constrains them: as reprojections (depths are nearly flat).
    np.testing.assert_allclose(
        _offset_residuals(problem, fixed, _offsets_of(problem, fixed, ours)),
        _offset_residuals(problem, fixed, scipy.x),
        rtol=0,
        atol=1e-5,
    )


def test_robust_optimum_is_stationary() -> None:
    """At the Huber optimum, an independently written robust cost has zero gradient."""
    problem, fixed = _offsets_problem(cameras=4, points=12)
    loss = nls.Loss("huber", 1.5)
    ours = ba.solve(problem, ba.BAOptions(solver=replace(TO_FLOOR, loss=loss), gauge="none"))
    x_star = _offsets_of(problem, fixed, ours)

    def robust_cost(x: Array) -> float:
        norm = np.linalg.norm(_offset_residuals(problem, fixed, x), axis=1)
        return 0.5 * float(np.sum(np.where(norm <= 1.5, norm**2, 2 * 1.5 * norm - 1.5**2)))

    h = 1e-6
    gradient = np.array(
        [
            (robust_cost(x_star + h * e) - robust_cost(x_star - h * e)) / (2 * h)
            for e in np.eye(x_star.size)
        ]
    )
    assert ours.summary.solve.converged
    np.testing.assert_allclose(robust_cost(x_star), ours.summary.solve.final_cost, rtol=1e-12)
    # IRLS converges linearly on Huber's outlier blocks; at the step floor the gradient is
    # ~1e-5 against ~1e4 at the start.
    assert np.abs(gradient).max() < 1e-8 * ours.summary.solve.trace[0].gradient_norm


def test_perturbed_focal_is_recovered() -> None:
    """Freed intrinsics started 3% off converge back to the truth."""
    scene = _scene(cameras=6, points=100, perturb=0.005)
    off = scene.problem.intrinsics * np.array([1.03, 0.97, 1.0, 1.0])
    problem = replace(scene.problem, intrinsics=off, fixed_intrinsics=np.zeros((1, 4), dtype=bool))
    result = ba.solve(problem, ba.BAOptions(solver=TIGHT))
    assert result.summary.solve.converged
    np.testing.assert_allclose(result.intrinsics[0], PINHOLE, rtol=1e-7)


def test_similarity_gauge() -> None:
    """The gauge pose stays put, the scale pose keeps its baseline, and undamped GN converges."""
    scene = _scene(noise=0.5)
    options = ba.BAOptions(solver=nls.Options(method="gauss_newton"))
    result = ba.solve(scene.problem, options)
    summary = result.summary
    assert summary.solve.converged
    assert (summary.gauge_pose, summary.confined_pose) == (0, 4)
    np.testing.assert_array_equal(result.poses[0], scene.problem.poses[0])
    before = np.linalg.norm(scene.problem.poses[4, :3, 3] - scene.problem.poses[0, :3, 3])
    after = np.linalg.norm(result.poses[4, :3, 3] - result.poses[0, :3, 3])
    assert abs(after - before) < 1e-4 * before  # held to first order; drifts at second


def test_explicitly_fixed_poses_define_the_gauge() -> None:
    """Two fixed poses already pin rotation, translation and scale: no gauge pose is added."""
    fixed = np.array([True, False, False, True, False])
    scene = _scene(fixed_poses=fixed)
    result = ba.solve(scene.problem)
    assert (result.summary.gauge_pose, result.summary.confined_pose) == (-1, -1)
    np.testing.assert_array_equal(result.poses[fixed], scene.problem.poses[fixed])


def test_degenerate_data_is_dropped_and_counted() -> None:
    """A single-view point, a no-parallax point and an unobserved camera are set aside."""
    scene = _scene(cameras=4, points=20)
    p = scene.problem
    far = np.array([[0.0, 0.0, 1e7]])
    points = np.concatenate([p.points, p.points[:1], far])
    poses = np.concatenate([p.poses, p.poses[:1]])
    extra_obs = [(0, 20), (0, 21), (1, 21)]
    t_cw = se3.inverse(poses)
    pixels = [
        camera.project("pinhole", PINHOLE, t_cw[c, :3, :3] @ points[k] + t_cw[c, :3, 3])
        for c, k in extra_obs
    ]
    problem = ba.BAProblem.create(
        "pinhole",
        PINHOLE,
        poses,
        points,
        np.concatenate([p.observations, pixels]),
        np.concatenate([p.cam_idx, [c for c, _ in extra_obs]]),
        np.concatenate([p.pt_idx, [k for _, k in extra_obs]]),
    )
    result = ba.solve(problem, ba.BAOptions(min_parallax_deg=1.0))
    summary = result.summary
    assert (summary.dropped_points, summary.dropped_observations, summary.unobserved_poses) == (
        2,
        3,
        1,
    )
    np.testing.assert_array_equal(result.points[20:], points[20:])
    np.testing.assert_array_equal(result.poses[4], poses[4])


@pytest.mark.parametrize("backend", sorted(set(backends.LOCKSTEP_BACKENDS) - {"python"}))
def test_backend_parity(backend: str) -> None:
    """A compiled backend matches the reference on every kind of block.

    Radtan, two intrinsics groups, a Cauchy loss, depth priors with free affines, and pose
    priors.
    """
    require_backend(backend)
    scene = _scene(
        cameras=5,
        points=50,
        noise=0.8,
        model="radtan",
        intrinsics_idx=np.array([0, 0, 1, 1, 1]),
        fixed_intrinsics=np.array([False, False]),
        seed=7,
        groups=np.stack([RADTAN, RADTAN * 1.001]),
    )
    problem = _every_kind(scene)
    options = ba.BAOptions(
        solver=replace(TIGHT, loss=nls.Loss("cauchy", 2.0)), depth_loss=nls.Loss("huber", 0.5)
    )
    ref = ba.solve(problem, options)
    out = ba.solve(problem, options, backend=backend)
    assert out.summary.solve.termination == ref.summary.solve.termination
    assert out.summary.solve.iterations == ref.summary.solve.iterations
    same = replace(
        out.summary, solve=ref.summary.solve, variance_factor=ref.summary.variance_factor
    )
    assert same == ref.summary
    np.testing.assert_allclose(out.summary.variance_factor, ref.summary.variance_factor, rtol=1e-9)
    np.testing.assert_allclose(out.poses, ref.poses, rtol=0, atol=1e-9)
    np.testing.assert_allclose(out.points, ref.points, rtol=0, atol=1e-8)
    # Distortion is poorly conditioned in this scene (k3 ends near -6): 1e-8 relative.
    np.testing.assert_allclose(out.intrinsics, ref.intrinsics, rtol=1e-8)
    np.testing.assert_allclose(out.depth_affine, ref.depth_affine, rtol=0, atol=1e-9)
    np.testing.assert_allclose(out.intrinsics_std, ref.intrinsics_std, rtol=1e-6)
    np.testing.assert_allclose(
        out.summary.solve.final_cost, ref.summary.solve.final_cost, rtol=1e-9
    )


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_strict_mode_raises(backend: str) -> None:
    """At the iteration cap, strict mode raises in every backend."""
    require_backend(backend)
    scene = _scene(noise=0.5)
    options = ba.BAOptions(solver=replace(TIGHT, max_iterations=1, strict=True))
    with pytest.raises(NotConvergedError, match="max_iterations"):
        ba.solve(scene.problem, options, backend=backend)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"camera_model": "fisheye"}, "camera_model"),
        ({"cam_idx": np.array([0, 9])}, "cam_idx"),
        ({"pt_idx": np.array([0.0, 1.0])}, "pt_idx"),
        ({"weights": np.array([1.0, -1.0])}, "weights"),
        ({"fixed_points": np.array([1, 0])}, "fixed_points"),
        ({"poses": np.stack([np.eye(4), np.diag([2.0, 2.0, 2.0, 1.0])])}, "rotation"),
        ({"intrinsics": np.array([0.0, 500.0, 320.0, 240.0])}, "focal"),
        ({"depth_prior": np.array([0.2, -0.1])}, "depth_prior"),
        ({"pose_prior_weights": np.ones((2, 3))}, "pose_prior_weights"),
        ({"fixed_depth_affine": np.array([True])}, "fixed_depth_affine"),
    ],
)
def test_invalid_problems_are_rejected(change: dict[str, object], message: str) -> None:
    """Every malformed array is rejected when the problem is built."""
    poses = se3.exp(np.array([[0.0, 0, 0, 0, 0, 0], [1.0, 0, 0, 0, 0, 0]]))
    args: dict[str, object] = {
        "camera_model": "pinhole",
        "intrinsics": PINHOLE,
        "poses": poses,
        "points": np.array([[0.0, 0.0, 5.0], [1.0, 0.0, 5.0]]),
        "observations": np.array([[320.0, 240.0], [400.0, 240.0]]),
        "cam_idx": np.array([0, 1]),
        "pt_idx": np.array([0, 1]),
    }
    args.update(change)
    with pytest.raises(InvalidInputError, match=message):
        ba.BAProblem.create(**args)  # type: ignore[arg-type]


@pytest.mark.parametrize("backend", sorted(set(backends.LOCKSTEP_BACKENDS) - {"python"}))
def test_core_checks_indices_too(backend: str) -> None:
    """A compiled core rejects an out-of-range index even when Python's checks are bypassed."""
    require_backend(backend)
    arguments = backend_arguments(_scene(cameras=3, points=5).problem, ba.BAOptions())
    arguments["pt_idx"] = np.where(
        np.arange(arguments["pt_idx"].size) == 0, 99, arguments["pt_idx"]
    )
    with pytest.raises(InvalidInputError, match="pt_idx"):
        backends.function(backend, "slamlab.foundations.ba", "solve")(**arguments)


def test_construction_validates_however_built() -> None:
    """Direct construction and ``dataclasses.replace`` validate like ``create``.

    So the reference and the compiled core reject the same bad problem the same way.
    """
    problem = _scene(cameras=3, points=5).problem
    with pytest.raises(InvalidInputError, match="pt_idx"):
        replace(problem, pt_idx=np.where(np.arange(problem.pt_idx.size) == 0, 99, problem.pt_idx))
    with pytest.raises(InvalidInputError, match="fixed point cannot be anchored"):
        replace(
            ba.anchor_points(problem),
            fixed_points=np.ones(problem.points.shape[0], dtype=bool),
        )


def test_a_checked_problem_cannot_change() -> None:
    """A problem owns read-only copies: neither its fields nor the caller's arrays move it."""
    problem = _scene(cameras=3, points=5).problem
    anchors = ba.anchor_points(problem).point_anchor.copy()
    anchored = replace(problem, point_anchor=anchors)
    anchors[0] = -1
    assert anchored.point_anchor[0] != -1
    with pytest.raises(ValueError, match="read-only"):
        anchored.pt_idx[0] = 99


@pytest.mark.parametrize("backend", sorted(set(backends.LOCKSTEP_BACKENDS) - {"python"}))
def test_enum_tables_match_the_compiled_side(backend: str) -> None:
    """Every enum crossing the boundary has the same names in the same code order."""
    require_backend(backend)
    compiled = backends.implementation(backend, "slamlab.foundations.ba").codes()
    assert {k: tuple(v) for k, v in compiled.items()} == ENUM_TABLES


def test_invalid_options() -> None:
    """An unknown gauge, the report-only "partial", or a negative threshold is rejected."""
    with pytest.raises(InvalidInputError, match="gauge"):
        ba.BAOptions(gauge="partial")
    with pytest.raises(InvalidInputError, match="gauge"):
        ba.BAOptions(gauge="sim3")
    with pytest.raises(InvalidInputError, match="min_parallax_deg"):
        ba.BAOptions(min_parallax_deg=-1.0)
    block = nls.BlockLoss((nls.Loss(),), np.zeros(1, dtype=np.int64))
    with pytest.raises(InvalidInputError, match="block_loss"):
        ba.BAOptions(solver=nls.Options(block_loss=block))


def _scaled_about_first_camera(problem: ba.BAProblem, factor: float) -> ba.BAProblem:
    """Scale a problem's scene about its first camera: reprojections are unchanged."""
    centre = problem.poses[0, :3, 3]
    poses = problem.poses.copy()
    poses[:, :3, 3] = centre + factor * (poses[:, :3, 3] - centre)
    return replace(problem, poses=poses, points=centre + factor * (problem.points - centre))


def test_metric_depth_prior_fixes_scale() -> None:
    """Metric depth priors fix the scale that reprojection leaves free.

    From a start scaled 1.5x, the solve returns the true scene. Without them, the similarity
    gauge keeps the 1.5x scale.
    """
    scene = _scene(perturb=0.0)
    scaled = _scaled_about_first_camera(scene.problem, 1.5)
    with_prior = ba.solve(
        replace(scaled, depth_prior=scene.inverse_depth), ba.BAOptions(solver=TIGHT)
    )
    assert with_prior.summary.gauge == "first_pose"
    np.testing.assert_allclose(with_prior.points, scene.points, rtol=0, atol=1e-6)
    without = ba.solve(scaled, ba.BAOptions(solver=TIGHT))
    assert without.summary.gauge == "similarity"
    np.testing.assert_allclose(without.points, scaled.points, rtol=0, atol=1e-9)


def test_relative_depth_prior_recovers_affine() -> None:
    """With free affines, a prior known only up to scale and shift is calibrated.

    That is a relative-depth network's disparity: the solve finds each camera's
    (scale, shift).
    """
    scene = _scene(perturb=0.0)
    scale, shift = 2.0, 0.05
    problem = replace(
        scene.problem,
        depth_prior=(scene.inverse_depth - shift) / scale,
        fixed_depth_affine=np.zeros(5, dtype=bool),
    )
    result = ba.solve(problem, ba.BAOptions(solver=TIGHT))
    assert result.summary.gauge == "similarity"
    assert result.summary.solve.converged
    np.testing.assert_allclose(result.depth_affine, np.tile([scale, shift], (5, 1)), rtol=1e-8)


def test_pose_priors_anchor_the_gauge() -> None:
    """With pose priors, the auto gauge fixes nothing; priors at the truth return it."""
    scene = _scene()
    problem = replace(
        scene.problem, pose_prior=scene.poses, pose_prior_weights=np.full((5, 2), 100.0)
    )
    result = ba.solve(problem, ba.BAOptions(solver=TIGHT))
    assert (result.summary.gauge, result.summary.gauge_pose) == ("none", -1)
    np.testing.assert_allclose(result.poses, scene.poses, rtol=0, atol=1e-8)


def test_focal_observability() -> None:
    """Free focal lengths come with calibrated deviations; unpinned ones are flagged.

    The deviations are scaled by the variance factor, so they hold for the 0.5 px noise
    whatever the weights. With 0.3 rad of rotation between views, σ_f is under 1 px and the
    truth lies within 3σ. With 0.02 rad, σ_f is 8 px (1.7%): weakly observable, and flagged.
    Under pure translation, focal length and depth trade off exactly (σ_f ~ 700 px).
    """
    free = np.array([False])
    flagged = []
    for rotation in (0.3, 0.02, 0.0):
        scene = _scene(cameras=6, points=80, noise=0.5, rotation=rotation, fixed_intrinsics=free)
        result = ba.solve(scene.problem, ba.BAOptions(solver=TIGHT))
        flagged.append(result.summary.unobservable_focal)
        if rotation == 0.3:
            std = result.intrinsics_std[0]
            assert np.all(std[:2] < 0.01 * PINHOLE[:2])
            assert np.all(np.abs(result.intrinsics[0] - PINHOLE) < 3.0 * std)
    assert flagged == [0, 1, 1]


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_variance_factor_calibrates_the_deviations(backend: str) -> None:
    """The variance factor estimates the noise the weights leave out; it scales σ.

    With 0.5 px noise and unit weights the factor is near 0.25; with the matching weights
    (1 / 0.5²) near 1, and the scaled deviations agree either way. The unscaled ones keep
    whatever the weights say.
    """
    require_backend(backend)
    scene = _scene(
        cameras=6, points=80, noise=0.5, rotation=0.3, fixed_intrinsics=np.array([False])
    )
    matched = replace(scene.problem, weights=np.full_like(scene.problem.weights, 4.0))
    unit = ba.solve(scene.problem, ba.BAOptions(solver=TIGHT), backend=backend)
    exact = ba.solve(matched, ba.BAOptions(solver=TIGHT), backend=backend)
    raw = ba.solve(
        scene.problem, ba.BAOptions(solver=TIGHT, scale_deviations=False), backend=backend
    )
    assert 0.2 < unit.summary.variance_factor < 0.3
    np.testing.assert_allclose(exact.summary.variance_factor, 4.0 * unit.summary.variance_factor)
    np.testing.assert_allclose(exact.intrinsics_std, unit.intrinsics_std, rtol=1e-6)
    np.testing.assert_allclose(
        unit.intrinsics_std, np.sqrt(unit.summary.variance_factor) * raw.intrinsics_std
    )


def test_variance_factor_without_redundancy() -> None:
    """With no more residuals than free parameters, the factor is NaN and σ is unscaled."""
    scene = _scene(cameras=2, points=3, fixed_intrinsics=np.array([False]))
    result = ba.solve(scene.problem, ba.BAOptions(solver=TIGHT))
    assert np.isnan(result.summary.variance_factor)


def test_anchored_points_reach_the_same_optimum() -> None:
    """Anchoring every point as an inverse depth still reaches zero cost on noise-free data.

    Each inverse depth is its point's 1/z in its anchor camera.
    """
    scene = _scene()
    anchored = ba.anchor_points(scene.problem)
    result = ba.solve(anchored, ba.BAOptions(solver=TIGHT))
    assert result.summary.solve.converged
    assert result.summary.solve.final_cost < 1e-18
    host = se3.inverse(result.poses)[anchored.cam_idx[anchored.point_anchor]]
    in_host = np.einsum("pij,pj->pi", host[:, :3, :3], result.points) + host[:, :3, 3]
    np.testing.assert_allclose(result.inverse_depth, 1.0 / in_host[:, 2], rtol=1e-12)
    plain = ba.solve(scene.problem, ba.BAOptions(solver=TIGHT))
    assert np.all(np.isnan(plain.inverse_depth))


def test_anchor_validation_and_dropped_anchor() -> None:
    """An anchor must be one of the point's own observations, and one behind its camera drops it.

    A point whose anchor sits behind its camera at the start has no ray to be placed on.
    """
    p = _scene(cameras=3, points=6).problem
    wrong = np.full(6, -1)
    wrong[0] = 1  # observation 1 sees point 1, not point 0
    with pytest.raises(InvalidInputError, match="point_anchor"):
        ba.BAProblem.create(
            "pinhole",
            PINHOLE,
            p.poses,
            p.points,
            p.observations,
            p.cam_idx,
            p.pt_idx,
            point_anchor=wrong,
        )
    flipped = p.poses.copy()
    flipped[0] = se3.compose(p.poses[0], se3.exp(np.array([0.0, 0, 0, np.pi, 0, 0])))
    anchored = replace(
        ba.anchor_points(p), poses=flipped, fixed_poses=np.array([False, True, True])
    )
    result = ba.solve(anchored, ba.BAOptions(gauge="none"))
    assert result.summary.dropped_points == 6


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_auto_gauge_motion_only(backend: str) -> None:
    """With every point fixed (motion-only BA), the points pin the gauge: auto fixes nothing.

    The first version froze a pose at its perturbed start here (panel finding L1).
    """
    require_backend(backend)
    scene = _scene(cameras=4, points=40, perturb=0.01, seed=1)
    problem = replace(scene.problem, points=scene.points, fixed_points=np.ones(40, dtype=bool))
    result = ba.solve(problem, ba.BAOptions(solver=TIGHT), backend=backend)
    assert (result.summary.gauge, result.summary.gauge_pose) == ("none", -1)
    assert result.summary.solve.final_cost < 1e-16
    np.testing.assert_allclose(result.poses, scene.poses, rtol=0, atol=1e-8)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_auto_gauge_single_pose_prior(backend: str) -> None:
    """A full pose prior on one camera pins rigid motion, not scale: auto confines the scale.

    The first version left scale free here, so the focal length read as unobservable.
    """
    require_backend(backend)
    scene = _scene(
        cameras=5, points=60, noise=0.5, seed=3, rotation=0.3, fixed_intrinsics=np.array([False])
    )
    weights = np.zeros((5, 2))
    weights[0] = 1e4
    problem = replace(scene.problem, pose_prior=scene.poses, pose_prior_weights=weights)
    result = ba.solve(problem, ba.BAOptions(solver=TIGHT), backend=backend)
    assert result.summary.gauge == "scale"
    assert result.summary.confined_pose >= 0
    assert result.summary.unobservable_focal == 0
    assert np.all(np.isfinite(result.intrinsics_std[0, :2]))


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_scale_gauge_needs_an_anchor(backend: str) -> None:
    """An explicit scale gauge without a fixed pose or full prior is rejected, in both cores."""
    require_backend(backend)
    with pytest.raises(InvalidInputError, match="scale"):
        ba.solve(_scene().problem, ba.BAOptions(gauge="scale"), backend=backend)


def _partially_anchored(case: str) -> ba.BAProblem:
    """Return a noise-free scene whose priors or fixed points pin part of the similarity."""
    scene = _scene(cameras=5, points=60, perturb=0.02, seed=4, rotation=0.3)
    p = scene.problem
    weights = np.zeros((5, 2))
    if case == "translation priors on two cameras":  # free: rotation about their baseline
        weights[[0, 3], 0] = 1e4
        return replace(p, pose_prior=scene.poses, pose_prior_weights=weights)
    if case == "rotation priors on every camera":  # free: translation and scale
        weights[:, 1] = 1e4
        return replace(p, pose_prior=scene.poses, pose_prior_weights=weights)
    n_fixed = 1 if case == "one fixed point" else 2  # free: rotation and scale about it; or
    fixed = np.arange(60) < n_fixed  # the rotation about the line through the two
    points = np.where(fixed[:, None], scene.points, p.points)
    return replace(p, points=points, fixed_points=fixed)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
@pytest.mark.parametrize(
    "case",
    [
        "translation priors on two cameras",
        "rotation priors on every camera",
        "one fixed point",
        "two fixed points",
    ],
)
def test_auto_gauge_removes_only_what_is_free(backend: str, case: str) -> None:
    """Partial anchoring: auto removes the rest of the similarity, and no more.

    Fixing a whole pose here (the first version did) held it at its perturbed start while
    the priors and fixed points pulled elsewhere: costs of 5 to 29 on noise-free data.
    Removing exactly the free part reaches the optimum, as no gauge at all does.
    """
    require_backend(backend)
    result = ba.solve(_partially_anchored(case), ba.BAOptions(solver=TIGHT), backend=backend)
    assert result.summary.gauge == "partial"
    assert result.summary.gauge_pose == -1
    assert result.summary.confined_pose >= 0
    assert result.summary.solve.final_cost < 1e-18


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_prior_on_an_unobserved_camera_pins_nothing(backend: str) -> None:
    """A prior on a camera nothing observes cannot anchor the scene: the gauge ignores it.

    The first version took it as the rigid anchor, left rigid motion free, and read the
    focal length as unobservable (σ_f infinite).
    """
    require_backend(backend)
    scene = _scene(
        cameras=5, points=60, noise=0.5, seed=3, rotation=0.3, fixed_intrinsics=np.array([False])
    )
    unseen = replace(scene.problem, weights=np.where(scene.problem.cam_idx == 4, 0.0, 1.0))
    weights = np.zeros((5, 2))
    weights[4] = 1e4
    prior = replace(unseen, pose_prior=scene.poses, pose_prior_weights=weights)
    plain = ba.solve(unseen, ba.BAOptions(solver=TIGHT), backend=backend)
    result = ba.solve(prior, ba.BAOptions(solver=TIGHT), backend=backend)
    assert result.summary.gauge == plain.summary.gauge == "similarity"
    np.testing.assert_allclose(result.intrinsics_std, plain.intrinsics_std, rtol=1e-6)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_variance_factor_counts_only_weighted_prior_rows(backend: str) -> None:
    """A translation-only prior adds 3 rows to m, not 6: its rotation rows carry nothing.

    Under the trivial loss ``σ̂₀² (m - n) = 2 cost``; with and without the priors the
    redundancy differs by 3 per prior camera that moves. The gauge fixes pose 0, so its
    prior is a constant and left out: 4 cameras count.
    """
    require_backend(backend)
    scene = _scene(cameras=5, points=60, noise=0.5, seed=5, rotation=0.3)
    weights = np.zeros((5, 2))
    weights[:, 0] = 1e-12  # weighted, but too weakly to move anything
    priors = replace(scene.problem, pose_prior=scene.poses, pose_prior_weights=weights)
    options = ba.BAOptions(solver=TIGHT, gauge="similarity")
    plain = ba.solve(scene.problem, options, backend=backend).summary
    primed = ba.solve(priors, options, backend=backend).summary
    redundancy = [2 * s.solve.final_cost / s.variance_factor for s in (plain, primed)]
    assert plain.gauge_pose == 0
    assert redundancy[1] - redundancy[0] == pytest.approx(3 * 4, abs=1e-3)


@pytest.mark.parametrize("backend", sorted(backends.LOCKSTEP_BACKENDS))
def test_variance_factor_under_a_robust_loss(backend: str) -> None:
    """Outliers count as much as their IRLS weight lets them, not by their ρ.

    0.5 px noise (σ̂₀² = 0.25) with 15% of the pixels moved 20 px, under Cauchy: the factor
    stays near 0.25. Taking Σρ instead gave 0.78.
    """
    require_backend(backend)
    scene = _scene(cameras=6, points=80, noise=0.5, seed=6, rotation=0.3)
    rng = np.random.default_rng(6)
    pixels = scene.problem.observations.copy()
    outliers = np.asarray(rng.random(pixels.shape[0]) < 0.15)
    pixels[outliers] += rng.choice([-20.0, 20.0], size=(int(np.count_nonzero(outliers)), 2))
    problem = replace(scene.problem, observations=pixels)
    options = ba.BAOptions(solver=replace(TIGHT, loss=nls.Loss("cauchy", 1.0)))
    factor = ba.solve(problem, options, backend=backend).summary.variance_factor
    assert 0.15 < factor < 0.35


def _undistortion_scene(k1: float) -> ba.BAProblem:
    """Anchored points on radtan with free distortion, started at a strong k1."""
    scene = _scene(cameras=4, points=30, model="radtan", seed=2, perturb=0.02)
    fixed = np.zeros((1, 9), dtype=bool)
    fixed[0, 2:4] = True
    start = RADTAN.copy()
    start[4] = k1
    return replace(
        ba.anchor_points(scene.problem), intrinsics=start[None, :], fixed_intrinsics=fixed
    )


@pytest.mark.parametrize(("k1", "dropped"), [(-1.0, 0), (-1.1, 1)])
def test_anchor_pixels_outside_the_invertible_region(k1: float, dropped: int) -> None:
    """Both backends behave alike when an anchor pixel has no ray (panel finding L2).

    From k1 = -1.0, trial steps push anchor pixels out of the invertible region; they are
    rejected, not raised (Python used to raise). From k1 = -1.1, one anchor has no ray at the
    start; both backends drop it and count it. Both recover the true k1 = -0.08.
    """
    require_backend("cpp")
    problem = _undistortion_scene(k1)
    for backend in ("python", "cpp"):
        result = ba.solve(problem, ba.BAOptions(solver=TIGHT), backend=backend)
        assert result.summary.dropped_points == dropped
        assert result.summary.solve.converged
        assert any(not it.accepted for it in result.summary.solve.trace)
        np.testing.assert_allclose(result.intrinsics[0, 4], RADTAN[4], rtol=0, atol=1e-9)
