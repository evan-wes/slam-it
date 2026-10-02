"""Bundle adjustment's entry point: the Python reference, or a compiled backend."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Protocol

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

BackendResult = dict[str, Any]
"""What a compiled backend returns: named arrays and scalars (see :class:`Implementation`)."""

ENUM_TABLES: dict[str, tuple[str, ...]] = {
    "model": tuple(camera.MODELS),
    "gauge": GAUGES,
    "method": nls.METHODS,
    "loss": nls.LOSSES,
    "termination": tuple(t.value for t in nls.Termination),
}
"""Each enum crossing the boundary, as names in code order. A compiled backend's ``codes()``
must return the same tables; a test holds the two sides together."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.ba`` submodule provides.

    Every argument crosses by keyword. Enums cross as their index in :data:`ENUM_TABLES`.
    ``solve`` returns a dict with ``poses``, ``intrinsics``, ``points``, ``depth_affine``,
    ``inverse_depth``, ``intrinsics_std``, ``trace`` (rows of the :class:`nls.Iteration`
    fields), ``initial_cost``, ``final_cost``, ``variance_factor``, ``termination`` and
    ``gauge`` codes, and each of :data:`COUNT_NAMES`.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def solve(  # noqa: PLR0913 -- a problem and its options cross by name
        self,
        *,
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
        model: int,
        gauge: int,
        method: int,
        loss: int,
        loss_scale: float,
        depth_loss: int,
        depth_loss_scale: float,
        max_iterations: int,
        function_tolerance: float,
        gradient_tolerance: float,
        step_tolerance: float,
        initial_damping: float,
        max_damping: float,
        min_diagonal: float,
        max_diagonal: float,
        min_relative_decrease: float,
        strict: bool,
        min_parallax_deg: float,
        focal_std_ratio: float,
        scale_deviations: bool,
    ) -> BackendResult:
        """Solve; return the named results."""
        ...


# --- entry point ----------------------------------------------------------------------------


def _variance_factor(bundle: Bundle, state: State, cost: float) -> float:
    """Return the a-posteriori variance factor ``2 cost / (m - n)``, or NaN if m <= n."""
    redundancy = bundle.residual_rows() - bundle.free_size(state)
    return 2.0 * cost / redundancy if redundancy > 0 else float("nan")


def _intrinsics_std(bundle: Bundle, state: State, loss: nls.BlockLoss, factor: float) -> Array:
    """Return the free intrinsics' marginal standard deviations at ``state`` (NaN if fixed).

    ``factor`` scales the variances (the a-posteriori variance factor, or 1).
    """
    out = np.full(state.intrinsics.shape, np.nan)
    groups, params, positions = bundle.intrinsics_positions()
    if groups.size == 0:
        return out
    weights = loss.evaluate(bundle.squared_norms(state))[1]
    variances = bundle.linearize(state, weights).camera_variances(positions)
    out[groups, params] = np.sqrt(factor * variances)
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
    factor = _variance_factor(bundle, final, solution.summary.final_cost)
    scaling = factor if options.scale_deviations and np.isfinite(factor) else 1.0
    std = _intrinsics_std(bundle, final, block_loss, scaling)
    summary = BASummary(
        solve=solution.summary,
        gauge=layout.gauge,
        unobservable_focal=_unobservable_focal(
            problem.camera_model, final.intrinsics, std, options.focal_std_ratio
        ),
        variance_factor=factor,
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
    """Assemble a compiled backend's named results into a :class:`BAResult`."""
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
        for row in raw["trace"]
    )
    termination = nls.Termination(ENUM_TABLES["termination"][int(raw["termination"])])
    solve_summary = nls.SolveSummary(
        initial_cost=float(raw["initial_cost"]),
        final_cost=float(raw["final_cost"]),
        iterations=len(records),
        termination=termination,
        converged=termination in nls.CONVERGED,
        trace=records,
    )
    summary = BASummary(
        solve_summary,
        GAUGES[int(raw["gauge"])],
        variance_factor=float(raw["variance_factor"]),
        **{name: int(raw[name]) for name in COUNT_NAMES},
    )
    return BAResult(
        raw["poses"],
        raw["intrinsics"],
        raw["points"],
        raw["depth_affine"],
        raw["inverse_depth"],
        raw["intrinsics_std"],
        summary,
    )


def backend_arguments(problem: BAProblem, options: BAOptions) -> dict[str, Any]:
    """Return a problem and its options as a compiled backend's keyword arguments.

    Parameters
    ----------
    problem : BAProblem
        The problem.
    options : BAOptions
        The options.

    Returns
    -------
    dict
        Every argument of :meth:`Implementation.solve`, by name.
    """
    solver = options.solver
    return {
        "intrinsics": problem.intrinsics,
        "intrinsics_idx": problem.intrinsics_idx,
        "poses": problem.poses,
        "points": problem.points,
        "observations": problem.observations,
        "cam_idx": problem.cam_idx,
        "pt_idx": problem.pt_idx,
        "weights": problem.weights,
        "fixed_poses": problem.fixed_poses,
        "fixed_intrinsics": np.ascontiguousarray(problem.fixed_intrinsics.ravel()),
        "fixed_points": problem.fixed_points,
        "depth_prior": problem.depth_prior,
        "depth_prior_weights": problem.depth_prior_weights,
        "depth_affine": problem.depth_affine,
        "fixed_depth_affine": problem.fixed_depth_affine,
        "pose_prior": problem.pose_prior,
        "pose_prior_weights": problem.pose_prior_weights,
        "point_anchor": problem.point_anchor,
        "model": ENUM_TABLES["model"].index(problem.camera_model),
        "gauge": ENUM_TABLES["gauge"].index(options.gauge),
        "method": ENUM_TABLES["method"].index(solver.method),
        "loss": ENUM_TABLES["loss"].index(solver.loss.kind),
        "loss_scale": solver.loss.scale,
        "depth_loss": ENUM_TABLES["loss"].index(options.depth_loss.kind),
        "depth_loss_scale": options.depth_loss.scale,
        "max_iterations": solver.max_iterations,
        "function_tolerance": solver.function_tolerance,
        "gradient_tolerance": solver.gradient_tolerance,
        "step_tolerance": solver.step_tolerance,
        "initial_damping": solver.initial_damping,
        "max_damping": solver.max_damping,
        "min_diagonal": solver.min_diagonal,
        "max_diagonal": solver.max_diagonal,
        "min_relative_decrease": solver.min_relative_decrease,
        "strict": solver.strict,
        "min_parallax_deg": options.min_parallax_deg,
        "focal_std_ratio": options.focal_std_ratio,
        "scale_deviations": options.scale_deviations,
    }


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
    ValueError
        If ``backend`` is not a known backend name.
    slamlab.backends.BackendNotBuiltError
        If ``backend`` names a compiled backend that is not built here.
    slamlab.foundations.errors.InvalidInputError
        If the problem and options do not fit together: a ``"scale"`` gauge with nothing to
        measure scale from. The compiled backend also re-checks the problem itself, which
        :class:`BAProblem` already did on construction.
    slamlab.foundations.errors.NotConvergedError
        If ``options.solver.strict`` and the solve stopped without converging.
    """
    opts = options if options is not None else BAOptions()
    backends.check_name(backend)
    if backend == "python":
        return _solve_reference(problem, opts)
    raw = backends.function(backend, MODULE, "solve")(**backend_arguments(problem, opts))
    return _result_from_backend(raw)
