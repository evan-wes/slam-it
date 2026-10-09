"""Bundle adjustment's entry point: the Python reference, or a compiled backend."""

from __future__ import annotations

from dataclasses import asdict, replace
from typing import Any, Protocol

import numpy as np

from slam_it_all import backends
from slam_it_all.foundations import camera, nls, se3
from slam_it_all.foundations.ba.bundle import Bundle, State
from slam_it_all.foundations.ba.layout import make_layout
from slam_it_all.foundations.ba.problem import (
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
from slam_it_all.foundations.ba.schur import DenseCholesky

MODULE = "slam_it_all.foundations.ba"
"""The module a compiled backend's implementation mirrors (``_cpp.foundations.ba``)."""

BackendResult = dict[str, Any]
"""What a compiled backend returns: named arrays and scalars (see :class:`Implementation`)."""

ENUM_TABLES: dict[str, tuple[str, ...]] = {
    "model": tuple(camera.MODELS),
    "gauge": GAUGES,
    "method": nls.METHODS,
    "loss": nls.LOSSES,
    "nls_termination": tuple(t.value for t in nls.Termination),
}
"""Each enum the binding knows, as names in code order. The camera model and the result's
gauge and termination cross as codes; the options' choices (gauge, method, loss) cross by
name and the binding looks them up in these same tables. A compiled backend's ``codes()``
must return them; a test holds the two sides together."""


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.ba`` submodule provides.

    Every argument crosses by keyword: the problem's arrays, the camera model as its index in
    :data:`ENUM_TABLES`, and ``options``, :class:`BAOptions`' fields as one dict
    (``dataclasses.asdict``: nested for ``solver`` and the losses, choices by name).
    ``solve`` returns a dict with ``poses``, ``intrinsics``, ``points``, ``depth_affine``,
    ``inverse_depth``, ``intrinsics_std``, ``solve`` (the least-squares summary, read by
    :func:`nls.summary_from_backend`), ``variance_factor``, the ``gauge`` code, and each of
    :data:`COUNT_NAMES`.
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
        options: dict[str, Any],
    ) -> BackendResult:
        """Solve; return the named results."""
        ...


# --- entry point ----------------------------------------------------------------------------


def _variance_factor(bundle: Bundle, state: State, loss: nls.BlockLoss) -> float:
    """Return the a-posteriori variance factor ``Σ w s / (m - n)``, or NaN if m <= n.

    ``s`` are the blocks' squared weighted norms and ``w`` their final IRLS weights: the
    numerator that pairs with the information matrix ``Jᵀ W J`` the deviations come from.
    Under the trivial loss it is ``2 cost``; under a robust one it does not count outliers'
    ``ρ``, which grows with their size.
    """
    redundancy = bundle.residual_rows() - bundle.free_size(state)
    if redundancy <= 0:
        return float("nan")
    squared = bundle.squared_norms(state)
    return float(np.sum(loss.evaluate(squared)[1] * squared)) / redundancy


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


def centring_origin(problem: BAProblem) -> Array:
    """Return the point the solve is centred on, shape (3,).

    The mean of the centres of the cameras with a positive-weight observation: an unobserved
    placeholder (an unregistered camera left at the identity) must not pull it off the
    scene. All cameras if none is observed. Summed in index order (a cumulative sum), so both
    cores get the same bits.
    """
    seen = np.zeros(problem.poses.shape[0], dtype=bool)
    seen[problem.cam_idx[problem.weights > 0.0]] = True
    centres = problem.poses[seen if np.any(seen) else slice(None), :3, 3]
    if centres.shape[0] == 0:
        return np.zeros(3)
    return np.asarray(np.cumsum(centres, axis=0)[-1] / centres.shape[0], dtype=np.float64)


def _centred(problem: BAProblem, origin: Array) -> BAProblem:
    """Return the problem shifted by ``-origin``: camera centres, points and prior centres.

    Exact for the optimum: every residual is unchanged by a shift of the world. It keeps
    the solve's own steps well conditioned far from the origin, where a step's rotation
    about the world origin would swing a camera by about ``|θ|² |C| / 2``.
    """
    poses = problem.poses.copy()
    poses[:, :3, 3] -= origin
    prior = problem.pose_prior.copy()
    prior[:, :3, 3] -= origin
    return replace(problem, poses=poses, points=problem.points - origin, pose_prior=prior)


def _solve_reference(original: BAProblem, options: BAOptions) -> BAResult:
    """Run bundle adjustment with the Python reference, in a world centred on the cameras."""
    origin = centring_origin(original)
    problem = _centred(original, origin)
    layout = make_layout(problem, options)
    bundle = Bundle(problem, layout, DenseCholesky())
    block_loss = nls.BlockLoss(
        (options.solver.loss, options.depth_loss, nls.Loss()), bundle.loss_index
    )
    start = bundle.start()
    solution = nls.solve(bundle, start, replace(options.solver, block_loss=block_loss))
    final = solution.state
    poses = se3.inverse(final.poses)
    poses[:, :3, 3] += origin
    unmoved = np.all(final.poses == start.poses, axis=(1, 2))
    poses[unmoved] = original.poses[unmoved]  # bit-exact, not a round trip through the inverse
    points = bundle.world_points(final)
    still = np.all(points == problem.points, axis=1)  # fixed or dropped
    points = points + origin
    points[still] = original.points[still]
    factor = _variance_factor(bundle, final, block_loss)
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
        points,
        final.depth_affine,
        final.inverse_depth,
        std,
        summary,
    )


def _result_from_backend(raw: BackendResult, codes: backends.Codes) -> BAResult:
    """Assemble a compiled backend's named results into a :class:`BAResult`."""
    solve_summary = nls.summary_from_backend(raw["solve"], codes)
    summary = BASummary(
        solve_summary,
        backends.decode_name(codes, "gauge", raw["gauge"]),
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
        "options": asdict(options),
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
    backend : {"python", "cpp", "rust"}, optional
        Which implementation solves it (default: the Python reference).

    Returns
    -------
    BAResult
        The refined variables and the summary.

    Raises
    ------
    ValueError
        If ``backend`` is not a known backend name.
    slam_it_all.backends.BackendNotBuiltError
        If ``backend`` names a compiled backend that is not built here.
    slam_it_all.foundations.errors.InvalidInputError
        If the problem and options do not fit together: a ``"scale"`` gauge with nothing to
        measure scale from. The compiled backend also re-checks the problem itself, which
        :class:`BAProblem` already did on construction.
    slam_it_all.foundations.errors.NotConvergedError
        If ``options.solver.strict`` and the solve stopped without converging.
    """
    opts = options if options is not None else BAOptions()
    return backends.call(
        backend,
        MODULE,
        "solve",
        reference=lambda: _solve_reference(problem, opts),
        arguments=backend_arguments(problem, opts),
        decode=_result_from_backend,
    )
