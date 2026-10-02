"""Bundle adjustment's entry point: the Python reference, or a compiled backend."""

from __future__ import annotations

from dataclasses import replace
from typing import Protocol

import numpy as np

from slamlab import backends
from slamlab.foundations import camera, nls, se3
from slamlab.foundations.ba.bundle import Bundle, State
from slamlab.foundations.ba.layout import make_layout
from slamlab.foundations.ba.problem import (
    COUNT_NAMES,
    GAUGES,
    Array,
    BAOptions,
    BAProblem,
    BAResult,
    BASummary,
    IndexArray,
    Mask,
)
from slamlab.foundations.ba.schur import DenseCholesky

MODULE = "slamlab.foundations.ba"
"""The module a compiled backend's implementation mirrors (``_cpp.foundations.ba``)."""

BackendResult = tuple[Array, Array, Array, Array, Array, Array, Array, IndexArray, Array]
"""What a compiled backend returns: poses, intrinsics, points, depth affines, inverse depths,
intrinsics deviations, the trace, the counts and the costs."""


class Implementation(Protocol):
    """The function a compiled backend's ``foundations.ba`` submodule provides."""

    def solve(  # noqa: PLR0913, PLR0917 -- a problem crosses as a struct of arrays
        self,
        intrinsics: Array,
        intrinsics_idx: IndexArray,
        poses: Array,
        points: Array,
        observations: Array,
        cam_idx: IndexArray,
        pt_idx: IndexArray,
        weights: Array,
        fixed_poses: Mask,
        fixed_intrinsics: Mask,
        fixed_points: Mask,
        depth_prior: Array,
        depth_prior_weights: Array,
        depth_affine: Array,
        fixed_depth_affine: Mask,
        pose_prior: Array,
        pose_prior_weights: Array,
        point_anchor: IndexArray,
        gauge: int,
        settings: Array,
        codes: IndexArray,
        tolerances: Array,
    ) -> BackendResult:
        """Solve; see :func:`_backend_arguments` for the packed ``settings`` and ``codes``."""
        ...


# --- entry point ----------------------------------------------------------------------------


def _intrinsics_std(bundle: Bundle, state: State, loss: nls.BlockLoss) -> Array:
    """Return the free intrinsics' marginal standard deviations at ``state`` (NaN if fixed)."""
    out = np.full(state.intrinsics.shape, np.nan)
    groups, params, positions = bundle.intrinsics_positions()
    if groups.size == 0:
        return out
    weights = loss.evaluate(bundle.squared_norms(state))[1]
    variances = bundle.linearize(state, weights).camera_variances(positions)
    out[groups, params] = np.sqrt(variances)
    return out


def _unobservable_focal(model: str, intrinsics: Array, std: Array, ratio: float) -> int:
    """Count free groups whose focal length is uncertain by more than ``ratio`` of itself."""
    focal = list(camera.FOCAL_INDICES[model])
    relative = std[:, focal] / intrinsics[:, focal]
    return int(np.count_nonzero(np.any(relative > ratio, axis=1)))


def _solve_reference(problem: BAProblem, options: BAOptions) -> BAResult:
    """Run bundle adjustment with the Python reference."""
    layout = make_layout(problem, options)
    bundle = Bundle(problem, layout, DenseCholesky())
    block_loss = nls.BlockLoss(
        (options.solver.loss, options.depth_loss, nls.Loss()), bundle.loss_index
    )
    start = bundle.start()
    solution = nls.solve(bundle, start, replace(options.solver, block_loss=block_loss))
    final = solution.state
    poses = se3.inverse(final.poses)
    unmoved = np.all(final.poses == start.poses, axis=(1, 2))
    poses[unmoved] = problem.poses[unmoved]  # bit-exact, not a round trip through the inverse
    std = _intrinsics_std(bundle, final, block_loss)
    summary = BASummary(
        solve=solution.summary,
        gauge=layout.gauge,
        unobservable_focal=_unobservable_focal(
            problem.camera_model, final.intrinsics, std, options.focal_std_ratio
        ),
        **layout.summary,
    )
    return BAResult(
        poses,
        final.intrinsics,
        bundle.world_points(final),
        final.depth_affine,
        final.inverse_depth,
        std,
        summary,
    )


def _result_from_backend(raw: BackendResult) -> BAResult:
    """Assemble a compiled backend's arrays into a :class:`BAResult`."""
    poses, intrinsics, points, affine, inverse_depth, std, trace, counts, costs = raw
    records = tuple(
        nls.Iteration(
            iteration=int(row[0]),
            cost=float(row[1]),
            gradient_norm=float(row[2]),
            step_norm=float(row[3]),
            damping=float(row[4]),
            relative_decrease=float(row[5]),
            accepted=bool(row[6]),
        )
        for row in trace
    )
    termination = list(nls.Termination)[int(counts[0])]
    solve_summary = nls.SolveSummary(
        initial_cost=float(costs[0]),
        final_cost=float(costs[1]),
        iterations=len(records),
        termination=termination,
        converged=termination in nls.CONVERGED,
        trace=records,
    )
    summary = BASummary(
        solve_summary,
        GAUGES[int(counts[1])],
        **{name: int(v) for name, v in zip(COUNT_NAMES, counts[2:], strict=True)},
    )
    return BAResult(poses, intrinsics, points, affine, inverse_depth, std, summary)


def _backend_arguments(problem: BAProblem, options: BAOptions) -> tuple[object, ...]:
    """Pack a problem and its options for a compiled backend.

    Scalars travel packed: ``settings`` is ``[min_parallax_deg, focal_std_ratio, loss_scale,
    depth_loss_scale]``; ``codes`` is ``[model, method, loss, depth_loss, max_iterations,
    strict]``; ``tolerances`` is the eight NLS tolerances and limits in ``Options`` order.
    """
    solver = options.solver
    return (
        problem.intrinsics,
        problem.intrinsics_idx,
        problem.poses,
        problem.points,
        problem.observations,
        problem.cam_idx,
        problem.pt_idx,
        problem.weights,
        problem.fixed_poses,
        np.ascontiguousarray(problem.fixed_intrinsics.ravel()),
        problem.fixed_points,
        problem.depth_prior,
        problem.depth_prior_weights,
        problem.depth_affine,
        problem.fixed_depth_affine,
        problem.pose_prior,
        problem.pose_prior_weights,
        problem.point_anchor,
        GAUGES.index(options.gauge),
        np.array(
            [
                options.min_parallax_deg,
                options.focal_std_ratio,
                solver.loss.scale,
                options.depth_loss.scale,
            ]
        ),
        np.array(
            [
                camera.MODELS[problem.camera_model][0],
                nls.METHODS.index(solver.method),
                nls.LOSSES.index(solver.loss.kind),
                nls.LOSSES.index(options.depth_loss.kind),
                solver.max_iterations,
                int(solver.strict),
            ],
            dtype=np.int64,
        ),
        np.array(
            [
                solver.function_tolerance,
                solver.gradient_tolerance,
                solver.step_tolerance,
                solver.initial_damping,
                solver.max_damping,
                solver.min_diagonal,
                solver.max_diagonal,
                solver.min_relative_decrease,
            ]
        ),
    )


def solve(
    problem: BAProblem, options: BAOptions | None = None, *, backend: str = "python"
) -> BAResult:
    """Refine a bundle-adjustment problem's poses, intrinsics, points and depth affines.

    Parameters
    ----------
    problem : BAProblem
        The problem (see :meth:`BAProblem.create`).
    options : BAOptions, optional
        Solver settings, losses, gauge and thresholds (default: ``BAOptions()``).
    backend : {"python", "cpp"}, optional
        Which implementation solves it (default: the Python reference).

    Returns
    -------
    BAResult
        The refined variables and the summary.

    Raises
    ------
    slamlab.foundations.errors.NotConvergedError
        If ``options.solver.strict`` and the solve stopped without converging.
    """
    opts = options if options is not None else BAOptions()
    backends.check_name(backend)
    if backend == "python":
        return _solve_reference(problem, opts)
    raw = backends.function(backend, MODULE, "solve")(*_backend_arguments(problem, opts))
    return _result_from_backend(raw)
