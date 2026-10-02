"""Tests for slamlab.foundations.ba: the Schur system, the optimum, gauge, degeneracy, parity."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
import pytest
from numpy.typing import NDArray
from scipy.optimize import least_squares

from slamlab import backends
from slamlab.foundations import ba, camera, nls, se3
from slamlab.foundations.errors import InvalidInputError, NotConvergedError
from tests.conftest import require_backend

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
    problem: ba.BAProblem


def _scene(
    *,
    cameras: int = 5,
    points: int = 60,
    noise: float = 0.0,
    perturb: float = 0.01,
    model: str = "pinhole",
    seed: int = 0,
    groups: Array | None = None,
    **create: object,
) -> Scene:
    """Build cameras along a gentle arc looking at a box of points, with every point seen.

    ``groups`` overrides the problem's starting intrinsics (the truth projects with one).
    """
    rng = np.random.default_rng(seed)
    intrinsics = PINHOLE if model == "pinhole" else RADTAN
    world = rng.uniform([-2.0, -2.0, 5.0], [2.0, 2.0, 9.0], (points, 3))
    twists = np.concatenate(
        [
            np.stack(
                [0.4 * np.arange(cameras), 0.1 * np.sin(np.arange(cameras)), np.zeros(cameras)], 1
            ),
            rng.normal(0.0, 0.04, (cameras, 3)),
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
    return Scene(poses, world, intrinsics, problem)


def _dense_jacobian(bundle: ba._Bundle, state: ba._State) -> tuple[Array, Array]:
    """Assemble the full Jacobian in the free coordinates from the per-observation blocks."""
    lin = bundle._linearise(state)
    reduction = bundle.reduction(state)
    n_obs = lin.residuals.shape[0]
    full = np.zeros((n_obs, 2, reduction.full_size))
    np.add.at(
        full,
        (np.arange(n_obs)[:, None, None], np.arange(2)[None, :, None], lin.columns[:, None, :]),
        lin.camera_side,
    )
    cam = np.concatenate(
        [full[:, :, reduction.free], full[:, :, reduction.rows] @ reduction.basis], axis=2
    )
    slots = bundle._layout.point_slot[bundle._pt]
    n_free = int(slots.max()) + 1
    pts = np.zeros((n_obs, 2, 3 * n_free))
    for k in range(3):
        rows = np.flatnonzero(slots >= 0)
        pts[rows, :, 3 * slots[rows] + k] = lin.point[rows, :, k]
    return np.concatenate([cam, pts], axis=2), lin.residuals


def _bundle(
    problem: ba.BAProblem, options: ba.BAOptions | None = None
) -> tuple[ba._Bundle, ba._State]:
    """Return the reference's NLS problem and its start state."""
    layout = ba._layout(problem, options or ba.BAOptions())
    start = ba._State(se3.inverse(problem.poses), problem.intrinsics.copy(), problem.points.copy())
    return ba._Bundle(problem, layout, ba.DenseCholesky()), start


def test_schur_step_matches_dense_solve() -> None:
    """Eliminating the points gives the same damped step as solving the full system."""
    scene = _scene(noise=0.5, model="radtan", fixed_intrinsics=[False])
    bundle, state = _bundle(scene.problem)
    weights = nls.Loss("huber", 2.0).evaluate(bundle.squared_norms(state))[1]
    schur = bundle.linearize(state, weights)
    jacobian, residuals = _dense_jacobian(bundle, state)
    dense = nls.DenseSystem.build(jacobian, residuals, weights)
    damping = 1e-3 * np.clip(dense.hessian_diagonal, 1e-6, 1e32)
    np.testing.assert_allclose(schur.gradient, dense.gradient, rtol=1e-12, atol=1e-9)
    np.testing.assert_allclose(schur.hessian_diagonal, dense.hessian_diagonal, rtol=1e-12)
    step_schur, step_dense = schur.solve(damping), dense.solve(damping)
    assert step_schur is not None
    assert step_dense is not None
    np.testing.assert_allclose(
        step_schur, step_dense, rtol=0, atol=1e-10 * np.abs(step_dense).max()
    )


def test_jacobian_matches_numeric() -> None:
    """The analytic Jacobian (poses, radtan intrinsics, points) matches central differences."""
    scene = _scene(cameras=3, points=8, model="radtan", fixed_intrinsics=[False])
    bundle, state = _bundle(scene.problem)
    analytic, _ = _dense_jacobian(bundle, state)
    numeric = nls.numeric_jacobian(
        lambda s: bundle._residuals(s).reshape(-1), state, bundle.retract, analytic.shape[2]
    )
    scale = np.abs(analytic).max(axis=(0, 1))
    np.testing.assert_allclose(
        analytic.reshape(-1, analytic.shape[2]) / scale, numeric / scale, rtol=0, atol=1e-6
    )


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
    problem = replace(scene.problem, intrinsics=off, fixed_intrinsics=np.array([False]))
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
    assert (summary.gauge_pose, summary.scale_pose) == (0, 4)
    np.testing.assert_array_equal(result.poses[0], scene.problem.poses[0])
    before = np.linalg.norm(scene.problem.poses[4, :3, 3] - scene.problem.poses[0, :3, 3])
    after = np.linalg.norm(result.poses[4, :3, 3] - result.poses[0, :3, 3])
    assert abs(after - before) < 1e-4 * before  # held to first order; drifts at second


def test_explicitly_fixed_poses_define_the_gauge() -> None:
    """Two fixed poses already pin rotation, translation and scale: no gauge pose is added."""
    fixed = np.array([True, False, False, True, False])
    scene = _scene(fixed_poses=fixed)
    result = ba.solve(scene.problem)
    assert (result.summary.gauge_pose, result.summary.scale_pose) == (-1, -1)
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
    result = ba.solve(problem)
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
    """A compiled backend matches the reference: radtan, two intrinsics groups, Cauchy loss."""
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
    problem = scene.problem
    options = ba.BAOptions(solver=replace(TIGHT, loss=nls.Loss("cauchy", 2.0)))
    ref = ba.solve(problem, options)
    out = ba.solve(problem, options, backend=backend)
    assert out.summary.solve.termination == ref.summary.solve.termination
    assert out.summary.solve.iterations == ref.summary.solve.iterations
    assert replace(out.summary, solve=ref.summary.solve) == ref.summary
    np.testing.assert_allclose(out.poses, ref.poses, rtol=0, atol=1e-9)
    np.testing.assert_allclose(out.points, ref.points, rtol=0, atol=1e-8)
    np.testing.assert_allclose(out.intrinsics, ref.intrinsics, rtol=1e-10)
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
    p = _scene(cameras=3, points=5).problem
    bad = replace(p, pt_idx=np.where(np.arange(p.pt_idx.size) == 0, 99, p.pt_idx))
    with pytest.raises(InvalidInputError, match="pt_idx"):
        ba.solve(bad, backend=backend)


def test_invalid_options() -> None:
    """An unknown gauge or a negative parallax threshold is rejected."""
    with pytest.raises(InvalidInputError, match="gauge"):
        ba.BAOptions(gauge="sim3")
    with pytest.raises(InvalidInputError, match="min_parallax_deg"):
        ba.BAOptions(min_parallax_deg=-1.0)
