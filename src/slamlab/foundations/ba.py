"""Bundle adjustment: refine camera poses, intrinsics and 3D points against their images.

The cost is a robust sum over three kinds of residual block,

    cost = ½ Σ_m ρ(w_m |π(K_g(m), T_c(m) X_p(m)) - u_m|²)          reprojection, 2 rows
         + ½ Σ_d ρ_d(w_d (1/z_c(d)(X_p(d)) - (s_c d̃_d + t_c))²)      depth prior, 1 row
         + ½ Σ_c |W_c log(T̄_c⁻¹ T_c)|²                                pose prior, 6 rows

minimised by :func:`slamlab.foundations.nls.solve`, each kind with its own loss.

**Reprojection.** ``T_c`` is camera c's world-to-camera pose, ``K_g`` its intrinsics group,
``X_p`` a world point, ``u_m`` the observed pixel and ``w_m`` the observation's weight (an
information, ``1/σ²`` in px⁻²).

**Depth priors.** An observation can carry a prior inverse depth ``d̃`` (from a depth network
or a depth sensor) with its own weight and robust loss. The prior is mapped through a
per-camera affine ``(s_c, t_c)``:

- for a metric prior, keep the affine fixed at ``(1, 0)``;
- for an affine-invariant one (a relative-depth network's disparity), free it, so the solve
  finds each frame's scale and shift.

Inverse depth, not depth, because networks predict it and it stays finite for far points.

**Pose priors.** A camera can carry a prior pose ``T̄_c`` (from odometry or a previous solve),
weighted separately in translation and rotation. The error is ``log(T̄_c⁻¹ T_c)`` with
``T_c`` world-to-camera, so it is expressed in the world frame like the poses' steps.

**Parameters.**
- Poses are perturbed on the right of ``T_cam_world``, ``T ⊕ δ = T exp(δ)``, so ``δ = (ρ, θ)``
  is expressed in the world frame.
- Points are plain 3-vectors.
- Intrinsics are shared through ``intrinsics_idx``: several cameras can use one group, or
  each its own.
- Depth affines are per camera.

Any pose, intrinsics group, affine or point can be held fixed. Callers pass and receive
camera-to-world poses ``T_world_cam`` (design doc, "Conventions").

**The Schur complement.** Points are eliminated before the solve. Each point's 3x3 block
is inverted, and the reduced camera system (poses, intrinsics and affines) is factored by a
:class:`ReducedSystemSolver` (dense Cholesky here; a sparse solver arrives with larger
problems). The points then follow by back-substitution.

**Gauge.** Reprojection errors don't change if the whole scene moves rigidly, or, for a
monocular camera, scales. Those directions are removed, not left to the damping:

- ``"first_pose"`` (SE(3) gauge): if no pose is fixed, the first free observed pose is.
- ``"similarity"`` (Sim(3) gauge): as ``"first_pose"``. Then, unless two fixed poses
  already pin the scale, the free pose farthest from a fixed one may not move its centre
  along that baseline. Its 6-dimensional step is confined to the 5-dimensional subspace
  orthogonal (to first order) to the change ``dc = -ρ + cross(c, θ)`` of its centre ``c``
  along the baseline. The basis of that subspace is a Householder reflection, deterministic
  in every backend.
- ``"none"``: the caller's fixed variables or priors define the gauge.
- ``"auto"`` (the default) picks by what the data observes:
  - ``"none"`` when any pose has a prior (it anchors the scene);
  - ``"first_pose"`` when a metric depth prior (a fixed affine) fixes the scale;
  - ``"similarity"`` otherwise.

**Degenerate data is dropped, and counted:**
- observations of a point behind its camera at the start;
- every observation of a free point seen fewer than twice, or with less than
  ``min_parallax_deg`` between its rays.

A point with no parallax has an undetermined depth. Freezing it at a wrong depth would bias
the cameras that see it. Free poses, intrinsics groups and affines left with nothing
observing them are held fixed.

**Focal observability.** When intrinsics are free, the result reports their standard
deviations from the marginal covariance at the solution (the inverse of the reduced camera
system, under the final robust weights). A free group whose focal length is uncertain by
more than ``focal_std_ratio`` of itself is counted as unobservable: the data does not pin
it. Those deviations assume the weights are true informations.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import camera, nls, se3, so3
from slamlab.foundations.arrays import as_real
from slamlab.foundations.errors import InvalidInputError

Array = NDArray[np.float64]
IndexArray = NDArray[np.int64]
Mask = NDArray[np.bool_]

GAUGES = ("none", "first_pose", "similarity", "auto")
"""The gauge choices :class:`BAOptions` accepts, in the order of their integer codes."""

POSE_DIM = 6
"""A pose's tangent dimension."""

AFFINE_DIM = 2
"""A depth affine's dimension, ``(scale, shift)``."""

PAIR_CHUNK = 4096
"""Observation pairs processed at once when forming the Schur complement (bounds memory)."""

BackendResult = tuple[Array, Array, Array, Array, Array, Array, IndexArray, Array]
"""What a compiled backend returns: poses, intrinsics, points, depth affines, intrinsics
deviations, the trace, the counts and the costs."""


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
        gauge: int,
        settings: Array,
        codes: IndexArray,
        tolerances: Array,
    ) -> BackendResult:
        """Solve; see :func:`_backend_arguments` for the packed ``settings`` and ``codes``."""
        ...


class ReducedSystemSolver(Protocol):
    """Solves the reduced camera system ``S x = b``, S symmetric positive definite."""

    def solve(self, matrix: Array, rhs: Array) -> Array | None:
        """Return ``x``, or ``None`` if ``matrix`` is not positive definite."""
        ...


class DenseCholesky:
    """The dense reduced solver: Cholesky on the full matrix."""

    def solve(self, matrix: Array, rhs: Array) -> Array | None:
        """Solve ``matrix x = rhs`` by Cholesky.

        Parameters
        ----------
        matrix : ndarray of float64, shape (n, n)
            Symmetric positive definite.
        rhs : ndarray of float64, shape (n,)
            The right-hand side.

        Returns
        -------
        ndarray of float64, shape (n,), or None
            The solution, or ``None`` if ``matrix`` is not positive definite.
        """
        return nls.cholesky_solve(matrix, rhs)


@dataclass(frozen=True)
class BAProblem:
    """A bundle-adjustment problem, as a struct of arrays (build it with :meth:`create`).

    Attributes
    ----------
    camera_model : str
        A model from :data:`slamlab.foundations.camera.MODELS`.
    intrinsics : ndarray of float64, shape (K, P)
        One intrinsics vector per group.
    poses : ndarray of float64, shape (C, 4, 4)
        Camera-to-world poses ``T_world_cam``.
    points : ndarray of float64, shape (N, 3)
        World points.
    observations : ndarray of float64, shape (M, 2)
        Observed pixels.
    cam_idx, pt_idx : ndarray of int64, shape (M,)
        Each observation's camera and point.
    intrinsics_idx : ndarray of int64, shape (C,)
        Each camera's intrinsics group.
    weights : ndarray of float64, shape (M,)
        Each observation's weight (0 excludes it).
    fixed_poses, fixed_intrinsics, fixed_points : ndarray of bool
        Which variables are held fixed, shapes (C,), (K,) and (N,).
    depth_prior : ndarray of float64, shape (M,)
        Each observation's prior inverse depth (NaN for none).
    depth_prior_weights : ndarray of float64, shape (M,)
        Each depth prior's weight, an information in inverse-depth units.
    depth_affine : ndarray of float64, shape (C, 2)
        Each camera's prior-to-model map ``(scale, shift)``.
    fixed_depth_affine : ndarray of bool, shape (C,)
        Which affines are held fixed (a metric prior's stays at ``(1, 0)``).
    pose_prior : ndarray of float64, shape (C, 4, 4)
        Each camera's prior camera-to-world pose (ignored where its weights are zero).
    pose_prior_weights : ndarray of float64, shape (C, 2)
        Each pose prior's ``(translation, rotation)`` information.
    """

    camera_model: str
    intrinsics: Array
    poses: Array
    points: Array
    observations: Array
    cam_idx: IndexArray
    pt_idx: IndexArray
    intrinsics_idx: IndexArray
    weights: Array
    fixed_poses: Mask
    fixed_intrinsics: Mask
    fixed_points: Mask
    depth_prior: Array
    depth_prior_weights: Array
    depth_affine: Array
    fixed_depth_affine: Mask
    pose_prior: Array
    pose_prior_weights: Array

    @classmethod
    def create(  # noqa: PLR0913, PLR0917 -- every array of a problem, in the order it reads
        cls,
        camera_model: str,
        intrinsics: ArrayLike,
        poses: ArrayLike,
        points: ArrayLike,
        observations: ArrayLike,
        cam_idx: ArrayLike,
        pt_idx: ArrayLike,
        *,
        intrinsics_idx: ArrayLike | None = None,
        weights: ArrayLike | None = None,
        fixed_poses: ArrayLike | None = None,
        fixed_intrinsics: ArrayLike | None = None,
        fixed_points: ArrayLike | None = None,
        depth_prior: ArrayLike | None = None,
        depth_prior_weights: ArrayLike | None = None,
        depth_affine: ArrayLike | None = None,
        fixed_depth_affine: ArrayLike | None = None,
        pose_prior: ArrayLike | None = None,
        pose_prior_weights: ArrayLike | None = None,
    ) -> BAProblem:
        """Validate a problem and fill in the defaults.

        Parameters
        ----------
        camera_model : str
            A model from :data:`slamlab.foundations.camera.MODELS`.
        intrinsics : array_like, shape (K, P) or (P,)
            The intrinsics groups; a single vector is one group shared by every camera.
        poses : array_like, shape (C, 4, 4)
            Camera-to-world poses.
        points : array_like, shape (N, 3)
            World points.
        observations : array_like, shape (M, 2)
            Observed pixels.
        cam_idx, pt_idx : array_like of int, shape (M,)
            Each observation's camera and point.
        intrinsics_idx : array_like of int, shape (C,), optional
            Each camera's intrinsics group (default: group 0 for all).
        weights : array_like, shape (M,), optional
            Non-negative observation weights (default: 1).
        fixed_poses : array_like of bool, shape (C,), optional
            Poses held fixed (default: none).
        fixed_intrinsics : array_like of bool, shape (K,), optional
            Intrinsics groups held fixed (default: all, as for a calibrated camera).
        fixed_points : array_like of bool, shape (N,), optional
            Points held fixed (default: none).
        depth_prior : array_like, shape (M,), optional
            Prior inverse depths, positive, NaN where there is none (default: none).
        depth_prior_weights : array_like, shape (M,), optional
            Non-negative depth-prior weights (default: 1).
        depth_affine : array_like, shape (C, 2), optional
            Starting ``(scale, shift)`` per camera (default: ``(1, 0)``).
        fixed_depth_affine : array_like of bool, shape (C,), optional
            Affines held fixed (default: all, for metric priors).
        pose_prior : array_like, shape (C, 4, 4), optional
            Prior camera-to-world poses (default: the poses themselves).
        pose_prior_weights : array_like, shape (C, 2), optional
            Non-negative ``(translation, rotation)`` informations (default: zero, no prior).

        Returns
        -------
        BAProblem
            The validated problem.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If any array has the wrong shape, type or values, an index is out of range, a
            pose is not a rigid transform, or an intrinsics vector is invalid for the model.
        """
        if camera_model not in camera.MODELS:
            raise InvalidInputError(f"camera_model: unknown model {camera_model!r}")
        k = np.atleast_2d(as_real(intrinsics, "intrinsics"))
        for group in k:
            camera.check_intrinsics(camera_model, group)
        t = _transforms(poses, "poses")
        x = _shaped(points, 3, "points")
        u = _shaped(observations, 2, "observations")
        n_cams, n_obs = t.shape[0], u.shape[0]
        return cls(
            camera_model=camera_model,
            intrinsics=k,
            poses=t,
            points=x,
            observations=u,
            cam_idx=_indices(cam_idx, n_obs, n_cams, "cam_idx"),
            pt_idx=_indices(pt_idx, n_obs, x.shape[0], "pt_idx"),
            intrinsics_idx=(
                np.zeros(n_cams, dtype=np.int64)
                if intrinsics_idx is None
                else _indices(intrinsics_idx, n_cams, k.shape[0], "intrinsics_idx")
            ),
            weights=_weights(weights, (n_obs,), 1.0, "weights"),
            fixed_poses=_mask(fixed_poses, n_cams, False, "fixed_poses"),
            fixed_intrinsics=_mask(fixed_intrinsics, k.shape[0], True, "fixed_intrinsics"),
            fixed_points=_mask(fixed_points, x.shape[0], False, "fixed_points"),
            depth_prior=_depth_prior(depth_prior, n_obs),
            depth_prior_weights=_weights(depth_prior_weights, (n_obs,), 1.0, "depth_prior_weights"),
            depth_affine=(
                np.tile([1.0, 0.0], (n_cams, 1))
                if depth_affine is None
                else _shaped(depth_affine, AFFINE_DIM, "depth_affine", rows=n_cams)
            ),
            fixed_depth_affine=_mask(fixed_depth_affine, n_cams, True, "fixed_depth_affine"),
            pose_prior=t.copy()
            if pose_prior is None
            else _transforms(pose_prior, "pose_prior", n_cams),
            pose_prior_weights=_weights(pose_prior_weights, (n_cams, 2), 0.0, "pose_prior_weights"),
        )


@dataclass(frozen=True)
class BAOptions:
    """Settings for :func:`solve`.

    Attributes
    ----------
    solver : slamlab.foundations.nls.Options
        The NLS method, tolerances and limits; its ``loss`` is the reprojection loss. Its
        ``block_loss`` must be unset: bundle adjustment assigns the per-block losses.
    depth_loss : slamlab.foundations.nls.Loss
        The depth priors' robust loss.
    gauge : {"auto", "similarity", "first_pose", "none"}
        How the gauge freedom is removed (see the module docstring).
    min_parallax_deg : float
        A free point whose rays span less than this angle is dropped.
    focal_std_ratio : float
        A free focal length whose standard deviation exceeds this fraction of it is counted
        as unobservable.
    """

    solver: nls.Options = field(default_factory=nls.Options)
    depth_loss: nls.Loss = field(default_factory=nls.Loss)
    gauge: str = "auto"
    min_parallax_deg: float = 1.0
    focal_std_ratio: float = 0.01

    def __post_init__(self) -> None:
        """Reject an unknown gauge, a bad threshold, or a per-block loss set by the caller."""
        if self.gauge not in GAUGES:
            raise InvalidInputError(f"gauge: expected one of {GAUGES}, got {self.gauge!r}")
        for name in ("min_parallax_deg", "focal_std_ratio"):
            value = getattr(self, name)
            if not (np.isfinite(value) and value >= 0.0):
                raise InvalidInputError(f"{name}: must be non-negative and finite")
        if self.solver.block_loss is not None:
            raise InvalidInputError("solver.block_loss: set by bundle adjustment, not the caller")


@dataclass(frozen=True)
class BASummary:
    """How a bundle adjustment went.

    Attributes
    ----------
    solve : slamlab.foundations.nls.SolveSummary
        The NLS solve's summary; its costs are the full robust cost.
    gauge : str
        The gauge applied (``"auto"`` resolved).
    gauge_pose : int
        The pose fixed to remove the rigid gauge, or -1 if none was.
    scale_pose : int
        The pose whose step was confined to remove the scale gauge, or -1.
    dropped_points : int
        Free points dropped as degenerate (too few views or too little parallax).
    dropped_observations : int
        Observations left out: those of dropped points and those behind their camera.
    unobserved_poses, unobserved_intrinsics, unobserved_depth_affines : int
        Free poses, intrinsics groups and affines held fixed because nothing observes them.
    unobservable_focal : int
        Free intrinsics groups whose focal length the data does not pin (see
        ``BAOptions.focal_std_ratio``).
    """

    solve: nls.SolveSummary
    gauge: str
    gauge_pose: int
    scale_pose: int
    dropped_points: int
    dropped_observations: int
    unobserved_poses: int
    unobserved_intrinsics: int
    unobserved_depth_affines: int
    unobservable_focal: int


COUNT_NAMES = (
    "gauge_pose",
    "scale_pose",
    "dropped_points",
    "dropped_observations",
    "unobserved_poses",
    "unobserved_intrinsics",
    "unobserved_depth_affines",
    "unobservable_focal",
)
"""The summary's counts, in the order a compiled backend returns them (after the
termination and gauge codes)."""


@dataclass(frozen=True)
class BAResult:
    """The refined variables and the summary.

    Attributes
    ----------
    poses : ndarray of float64, shape (C, 4, 4)
        Refined camera-to-world poses.
    intrinsics : ndarray of float64, shape (K, P)
        Refined intrinsics.
    points : ndarray of float64, shape (N, 3)
        Refined points (dropped points are returned unchanged).
    depth_affine : ndarray of float64, shape (C, 2)
        Refined depth affines.
    intrinsics_std : ndarray of float64, shape (K, P)
        Free intrinsics' standard deviations (NaN where held fixed; inf if unobservable).
    summary : BASummary
        How the solve went.
    """

    poses: Array
    intrinsics: Array
    points: Array
    depth_affine: Array
    intrinsics_std: Array
    summary: BASummary


# --- validation ---------------------------------------------------------------------------


def _shaped(value: ArrayLike, columns: int, name: str, rows: int | None = None) -> Array:
    """Return a finite real 2D array with the given column (and optionally row) count."""
    arr = as_real(value, name)
    if arr.ndim != 2 or arr.shape[1] != columns or (rows is not None and arr.shape[0] != rows):
        expected = f"({'n' if rows is None else rows}, {columns})"
        raise InvalidInputError(f"{name}: expected shape {expected}, got {arr.shape}")
    return arr


def _transforms(value: ArrayLike, name: str, count: int | None = None) -> Array:
    """Return a (C, 4, 4) batch of rigid transforms."""
    t = as_real(value, name)
    if t.ndim != 3 or t.shape[1:] != (4, 4) or (count is not None and t.shape[0] != count):
        raise InvalidInputError(f"{name}: expected shape (C, 4, 4), got {t.shape}")
    se3.check_transform(t)
    return t


def _indices(value: ArrayLike, length: int, bound: int, name: str) -> IndexArray:
    """Return an int64 index array of the given length with every entry in ``[0, bound)``."""
    arr = np.asarray(value)
    if arr.shape != (length,) or not np.issubdtype(arr.dtype, np.integer):
        raise InvalidInputError(f"{name}: expected {length} integers, got {arr.dtype} {arr.shape}")
    if length and (arr.min() < 0 or arr.max() >= bound):
        raise InvalidInputError(f"{name}: indices must lie in [0, {bound})")
    return arr.astype(np.int64)


def _weights(value: ArrayLike | None, shape: tuple[int, ...], default: float, name: str) -> Array:
    """Return non-negative weights of the given shape (default: filled with ``default``)."""
    if value is None:
        return np.full(shape, default)
    w = as_real(value, name)
    if w.shape != shape or np.any(w < 0.0):
        raise InvalidInputError(f"{name}: expected non-negative values of shape {shape}")
    return w


def _depth_prior(value: ArrayLike | None, length: int) -> Array:
    """Return prior inverse depths: positive, or NaN for none."""
    if value is None:
        return np.full(length, np.nan)
    arr = np.asarray(value, dtype=np.float64)
    given = ~np.isnan(arr)
    if arr.shape != (length,) or np.any(~np.isfinite(arr[given])) or np.any(arr[given] <= 0.0):
        raise InvalidInputError(f"depth_prior: expected {length} positive values (NaN for none)")
    return arr


def _mask(value: ArrayLike | None, length: int, default: bool, name: str) -> Mask:
    """Return a bool mask of the given length."""
    if value is None:
        return np.full(length, default)
    arr = np.asarray(value)
    if arr.shape != (length,) or arr.dtype != np.bool_:
        raise InvalidInputError(f"{name}: expected {length} booleans, got {arr.dtype} {arr.shape}")
    return arr.copy()


# --- setup: what is optimised ---------------------------------------------------------------


@dataclass(frozen=True)
class _Layout:
    """Which blocks count and which variables move, decided once from the start."""

    obs: IndexArray
    """Indices of the observations kept."""
    depth: IndexArray
    """Positions in ``obs`` of the kept observations with a depth prior."""
    prior_cams: IndexArray
    """Cameras with a pose prior."""
    pose_free: Mask
    """Poses with all six directions free (not fixed, observed, not the gauge poses)."""
    scale_pose: int
    scale_reference: int
    intrinsics_free: Mask
    affine_free: Mask
    point_slot: IndexArray
    """Each point's index among the free points, or -1."""
    gauge: str
    summary: dict[str, int]


def _ray_parallax(centres: Array, points: Array, cam: IndexArray, pt: IndexArray) -> Array:
    """Return each point's largest angle between its first ray and any other (radians)."""
    rays = centres[cam] - points[pt]
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    first = np.full(points.shape[0], -1, dtype=np.int64)
    order = np.arange(cam.shape[0])[::-1]
    first[pt[order]] = order  # the last write per point is its lowest observation index
    reference = rays[first[pt]]
    angle = np.arctan2(
        np.linalg.norm(np.cross(rays, reference), axis=-1), np.sum(rays * reference, axis=-1)
    )
    parallax = np.zeros(points.shape[0])
    np.maximum.at(parallax, pt, angle)
    return parallax


def _resolve_gauge(gauge: str, has_pose_prior: bool, has_metric_depth: bool) -> str:
    """Resolve ``"auto"`` by what anchors the scene; other choices pass through."""
    if gauge != "auto":
        return gauge
    if has_pose_prior:
        return "none"
    return "first_pose" if has_metric_depth else "similarity"


def _gauge(
    problem: BAProblem, pose_free: Mask, observed: Mask, gauge: str
) -> tuple[Mask, int, int, int]:
    """Fix the gauge; return the free poses, the gauge pose, the scale pose and its reference."""
    pose_free = pose_free.copy()
    gauge_pose = scale_pose = scale_reference = -1
    if gauge == "none":
        return pose_free, gauge_pose, scale_pose, scale_reference
    anchors = np.flatnonzero(~problem.fixed_poses & observed)
    fixed_observed = np.flatnonzero(problem.fixed_poses & observed)
    if fixed_observed.size == 0 and anchors.size:
        gauge_pose = int(anchors[0])
        pose_free[gauge_pose] = False
        fixed_observed = np.array([gauge_pose])
    if gauge == "first_pose" or fixed_observed.size == 0:
        return pose_free, gauge_pose, scale_pose, scale_reference
    centres = problem.poses[:, :3, 3]
    spread = np.linalg.norm(centres[fixed_observed] - centres[fixed_observed[0]], axis=-1)
    candidates = np.flatnonzero(pose_free)
    scale_tolerance = 1e-12 * (1.0 + np.linalg.norm(centres[fixed_observed[0]]))
    if np.any(spread > scale_tolerance) or candidates.size == 0:
        return pose_free, gauge_pose, scale_pose, scale_reference
    distance = np.linalg.norm(centres[candidates] - centres[fixed_observed[0]], axis=-1)
    if distance.max() > scale_tolerance:
        scale_pose = int(candidates[np.argmax(distance)])
        scale_reference = int(fixed_observed[0])
        pose_free[scale_pose] = False
    return pose_free, gauge_pose, scale_pose, scale_reference


def _kept_observations(problem: BAProblem, options: BAOptions) -> tuple[IndexArray, Mask, int]:
    """Return the kept observations, the degenerate points, and how many were dropped."""
    t_cam_world = se3.inverse(problem.poses)
    obs = np.flatnonzero(problem.weights > 0.0)
    cam, pt = problem.cam_idx[obs], problem.pt_idx[obs]
    depth = (
        np.einsum("mj,mj->m", t_cam_world[cam, 2, :3], problem.points[pt]) + t_cam_world[cam, 2, 3]
    )
    in_front = depth > camera.MIN_DEPTH
    obs, cam, pt = obs[in_front], cam[in_front], pt[in_front]
    views = np.bincount(pt, minlength=problem.points.shape[0])
    parallax = _ray_parallax(problem.poses[:, :3, 3], problem.points, cam, pt)
    degenerate = (
        ~problem.fixed_points
        & (views > 0)
        & ((views < 2) | (parallax < np.deg2rad(options.min_parallax_deg)))
    )
    keep = ~degenerate[pt]
    dropped = int(np.count_nonzero(~in_front)) + int(np.count_nonzero(~keep))
    return obs[keep], degenerate, dropped


def _layout(problem: BAProblem, options: BAOptions) -> _Layout:
    """Drop degenerate observations, hold unobserved variables fixed, and fix the gauge."""
    n_cams, n_points = problem.poses.shape[0], problem.points.shape[0]
    obs, degenerate, dropped = _kept_observations(problem, options)
    cam = problem.cam_idx[obs]
    depth = np.flatnonzero(
        ~np.isnan(problem.depth_prior[obs]) & (problem.depth_prior_weights[obs] > 0.0)
    )
    observed = np.bincount(cam, minlength=n_cams) > 0
    intrinsics_observed = (
        np.bincount(problem.intrinsics_idx[cam], minlength=problem.intrinsics.shape[0]) > 0
    )
    affine_observed = np.bincount(cam[depth], minlength=n_cams) > 0
    prior_cams = np.flatnonzero(np.any(problem.pose_prior_weights > 0.0, axis=1))
    gauge = _resolve_gauge(
        options.gauge,
        has_pose_prior=prior_cams.size > 0,
        has_metric_depth=bool(np.any(problem.fixed_depth_affine[cam[depth]])),
    )
    pose_free, gauge_pose, scale_pose, scale_reference = _gauge(
        problem, ~problem.fixed_poses & observed, observed, gauge
    )
    point_free = (
        ~problem.fixed_points
        & ~degenerate
        & (np.bincount(problem.pt_idx[obs], minlength=n_points) > 0)
    )
    point_slot = np.full(n_points, -1, dtype=np.int64)
    point_slot[point_free] = np.arange(np.count_nonzero(point_free))
    summary = {
        "gauge_pose": gauge_pose,
        "scale_pose": scale_pose,
        "dropped_points": int(np.count_nonzero(degenerate)),
        "dropped_observations": dropped,
        "unobserved_poses": int(np.count_nonzero(~problem.fixed_poses & ~observed)),
        "unobserved_intrinsics": int(
            np.count_nonzero(~problem.fixed_intrinsics & ~intrinsics_observed)
        ),
        "unobserved_depth_affines": int(
            np.count_nonzero(~problem.fixed_depth_affine & ~affine_observed)
        ),
    }
    return _Layout(
        obs=obs,
        depth=depth,
        prior_cams=prior_cams,
        pose_free=pose_free,
        scale_pose=scale_pose,
        scale_reference=scale_reference,
        intrinsics_free=~problem.fixed_intrinsics & intrinsics_observed,
        affine_free=~problem.fixed_depth_affine & affine_observed,
        point_slot=point_slot,
        gauge=gauge,
        summary=summary,
    )


# --- the reduced camera-side coordinates ----------------------------------------------------


def _householder_complement(normal: Array) -> Array:
    """Return an orthonormal (n, n-1) basis of the complement of ``normal``, deterministically.

    The Householder reflection mapping ``normal`` onto its largest axis keeps every other axis
    orthogonal to it; those columns are the basis.
    """
    unit = normal / np.linalg.norm(normal)
    k = int(np.argmax(np.abs(unit)))
    v = unit.copy()
    v[k] += 1.0 if unit[k] >= 0.0 else -1.0
    reflection = np.eye(unit.shape[0]) - 2.0 * np.outer(v, v) / (v @ v)
    return np.delete(reflection, k, axis=1)


@dataclass(frozen=True)
class _Reduction:
    """Maps between the full camera-side vector and the free one.

    The full vector holds 6 entries per pose, P per intrinsics group and 2 per depth affine;
    the free coordinates are the free full entries, in order, then the scale pose's five.
    """

    free: IndexArray
    rows: IndexArray
    basis: Array
    full_size: int

    @property
    def size(self) -> int:
        """The number of free camera-side coordinates."""
        return int(self.free.shape[0] + self.basis.shape[1])

    def reduce_vector(self, vector: Array) -> Array:
        """Project a full vector (a gradient) onto the free coordinates."""
        return np.concatenate([vector[self.free], self.basis.T @ vector[self.rows]])

    def reduce_matrix(self, matrix: Array) -> Array:
        """Project a full symmetric matrix onto the free coordinates: ``Bᵀ M B``."""
        top = matrix[np.ix_(self.free, self.free)]
        if self.basis.shape[1] == 0:
            return top
        side = matrix[np.ix_(self.free, self.rows)] @ self.basis
        corner = self.basis.T @ matrix[np.ix_(self.rows, self.rows)] @ self.basis
        return np.block([[top, side], [side.T, corner]])

    def reduce_diagonal(self, matrix: Array) -> Array:
        """Return the diagonal of ``reduce_matrix(matrix)`` without forming it."""
        corner = self.basis.T @ matrix[np.ix_(self.rows, self.rows)] @ self.basis
        return np.concatenate([np.diagonal(matrix)[self.free], np.diagonal(corner)])

    def expand(self, reduced: Array) -> Array:
        """Map free coordinates (a step) to the full vector."""
        full = np.zeros(self.full_size)
        n_free = self.free.shape[0]
        full[self.free] = reduced[:n_free]
        full[self.rows] += self.basis @ reduced[n_free:]
        return full


# --- the normal equations with the points eliminated ----------------------------------------


@dataclass(frozen=True)
class Blocks:
    """The blocks of the normal equations, formed once per linearisation.

    Attributes
    ----------
    hessian_cc, gradient_c : ndarray of float64
        The full camera-side Hessian (n, n) and gradient (n,).
    hessian_pp, gradient_p : ndarray of float64
        Each free point's 3x3 Hessian block (Nf, 3, 3) and gradient (Nf, 3).
    coupling : ndarray of float64, shape (Mf, w, 3)
        Each point-coupled block's camera-point block ``J_cᵀ W J_p``, zero-padded to the
        widest kind of block.
    columns : ndarray of int64, shape (Mf, w)
        Those blocks' camera-side columns (padding points at column 0, with zero coupling).
    slots : ndarray of int64, shape (Mf,)
        Those blocks' free-point slots.
    """

    hessian_cc: Array
    gradient_c: Array
    hessian_pp: Array
    gradient_p: Array
    coupling: Array
    columns: IndexArray
    slots: IndexArray


class SchurSystem:
    """The normal equations with the points eliminated: an NLS ``LinearSystem``."""

    def __init__(self, reduction: _Reduction, blocks: Blocks, solver: ReducedSystemSolver) -> None:
        """Hold the blocks; see :meth:`solve` for how they combine.

        Parameters
        ----------
        reduction : _Reduction
            The free camera-side coordinates.
        blocks : Blocks
            The normal equations' blocks.
        solver : ReducedSystemSolver
            Factors the reduced camera system.
        """
        self._reduction = reduction
        self._blocks = blocks
        self._solver = solver
        slots = blocks.slots
        order = np.argsort(slots, kind="stable")
        counts = np.bincount(slots[order], minlength=blocks.hessian_pp.shape[0])
        per_block = counts[slots[order]]
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])[slots[order]]
        offsets = np.arange(int(per_block.sum())) - np.repeat(
            np.cumsum(per_block) - per_block, per_block
        )
        self._pairs = (
            np.repeat(order, per_block),
            order[np.repeat(starts, per_block) + offsets],
        )

    @property
    def gradient(self) -> Array:
        """The gradient in the free coordinates: cameras, then points."""
        return np.concatenate(
            [
                self._reduction.reduce_vector(self._blocks.gradient_c),
                self._blocks.gradient_p.reshape(-1),
            ]
        )

    @property
    def hessian_diagonal(self) -> Array:
        """The Hessian's diagonal in the free coordinates."""
        return np.concatenate(
            [
                self._reduction.reduce_diagonal(self._blocks.hessian_cc),
                np.diagonal(self._blocks.hessian_pp, axis1=1, axis2=2).reshape(-1),
            ]
        )

    def _eliminate(self, damping: Array) -> tuple[Array, Array, Array] | None:
        """Eliminate the points from the damped system.

        Returns the reduced matrix, the reduced right-hand side and the damped point blocks'
        inverses; or ``None`` if a point block is indefinite.
        """
        b = self._blocks
        n_cam = self._reduction.size
        damped_pp = b.hessian_pp + damping[n_cam:].reshape(-1, 3)[:, :, None] * np.eye(3)
        try:
            lower = np.linalg.cholesky(damped_pp)
        except np.linalg.LinAlgError:
            return None
        lower_inv = np.linalg.inv(lower)
        inverse_pp = np.einsum("pki,pkj->pij", lower_inv, lower_inv)
        weighted = np.einsum("mij,mjk->mik", b.coupling, inverse_pp[b.slots])
        schur = b.hessian_cc.copy()
        first, second = self._pairs
        for start in range(0, first.shape[0], PAIR_CHUNK):
            i, j = first[start : start + PAIR_CHUNK], second[start : start + PAIR_CHUNK]
            pair_blocks = np.einsum("pik,pjk->pij", weighted[i], b.coupling[j])
            np.subtract.at(schur, (b.columns[i][:, :, None], b.columns[j][:, None, :]), pair_blocks)
        rhs = b.gradient_c.copy()
        np.subtract.at(rhs, b.columns, np.einsum("mik,mk->mi", weighted, b.gradient_p[b.slots]))
        reduced = self._reduction.reduce_matrix(schur) + np.diag(damping[:n_cam])
        return reduced, self._reduction.reduce_vector(rhs), inverse_pp

    def solve(self, damping: Array) -> Array | None:
        """Solve the damped normal equations by eliminating the points.

        Parameters
        ----------
        damping : ndarray of float64
            Added to the Hessian's diagonal, in the free coordinates.

        Returns
        -------
        ndarray of float64, or None
            The step, or ``None`` if a point block or the reduced system is indefinite.
        """
        eliminated = self._eliminate(damping)
        if eliminated is None:
            return None
        reduced, rhs, inverse_pp = eliminated
        step_c = self._solver.solve(reduced, -rhs)
        if step_c is None:
            return None
        b = self._blocks
        full_c = self._reduction.expand(step_c)
        back = b.gradient_p.copy()
        np.add.at(back, b.slots, np.einsum("mij,mi->mj", b.coupling, full_c[b.columns]))
        step_p = -np.einsum("pij,pj->pi", inverse_pp, back)
        return np.concatenate([step_c, step_p.reshape(-1)])

    def camera_variances(self, positions: IndexArray) -> Array:
        """Return marginal variances of camera-side free coordinates: ``diag(S⁻¹)``.

        Parameters
        ----------
        positions : ndarray of int64
            The free coordinates wanted.

        Returns
        -------
        ndarray of float64
            Their variances, or infinity for all if the reduced system is singular.
        """
        undamped = self._eliminate(np.zeros(self._reduction.size + self._blocks.gradient_p.size))
        if undamped is None:
            return np.full(positions.shape, np.inf)
        reduced = undamped[0]
        try:
            lower = np.linalg.cholesky(reduced)
        except np.linalg.LinAlgError:
            return np.full(positions.shape, np.inf)
        lower_inv = np.linalg.inv(lower)
        return np.asarray(np.sum(lower_inv[:, positions] ** 2, axis=0), dtype=np.float64)


# --- the problem the NLS loop drives -------------------------------------------------------


@dataclass(frozen=True)
class _State:
    """The variables: world-to-camera poses, intrinsics groups, points, depth affines."""

    poses: Array
    intrinsics: Array
    points: Array
    depth_affine: Array


@dataclass(frozen=True)
class _Kind:
    """One kind of residual block linearised at a state (rows already scaled by √w)."""

    residuals: Array
    """Shape (M, d)."""
    camera_side: Array
    """d r / d(camera-side variables), shape (M, d, w)."""
    columns: IndexArray
    """Each block's camera-side columns in the full vector, shape (M, w)."""
    point: Array | None
    """d r / d point, shape (M, d, 3); None for blocks that touch no point."""
    slots: IndexArray
    """Each block's free-point slot (-1 for none)."""


def _pose_columns(cam: IndexArray) -> IndexArray:
    """Return each camera's six pose columns in the full vector."""
    return np.asarray(POSE_DIM * cam[:, None] + np.arange(POSE_DIM), dtype=np.int64)


class _Bundle:
    """The bundle-adjustment problem as an NLS ``Problem`` over :class:`_State`."""

    def __init__(self, problem: BAProblem, layout: _Layout, solver: ReducedSystemSolver) -> None:
        """Hold the problem, its layout and the reduced solver; index every block."""
        self._problem = problem
        self._layout = layout
        self._solver = solver
        n_cams, (n_groups, n_params) = problem.poses.shape[0], problem.intrinsics.shape
        self._intrinsics_offset = POSE_DIM * n_cams
        self._affine_offset = self._intrinsics_offset + n_params * n_groups
        self._full_size = self._affine_offset + AFFINE_DIM * n_cams
        obs = layout.obs
        self._cam = problem.cam_idx[obs]
        self._pt = problem.pt_idx[obs]
        self._group = problem.intrinsics_idx[self._cam]
        self._pixels = problem.observations[obs]
        self._sqrt_w = np.sqrt(problem.weights[obs])
        intrinsics_columns = self._intrinsics_offset + n_params * self._group[:, None]
        self._columns = np.concatenate(
            [_pose_columns(self._cam), intrinsics_columns + np.arange(n_params)], axis=1
        )
        self._d_cam, self._d_pt = self._cam[layout.depth], self._pt[layout.depth]
        self._d_prior = problem.depth_prior[obs[layout.depth]]
        self._d_sqrt_w = np.sqrt(problem.depth_prior_weights[obs[layout.depth]])
        affine_columns = self._affine_offset + AFFINE_DIM * self._d_cam[:, None]
        self._d_columns = np.concatenate(
            [_pose_columns(self._d_cam), affine_columns + np.arange(AFFINE_DIM)], axis=1
        )
        cams = layout.prior_cams
        self._p_cam = cams
        self._p_prior = problem.pose_prior[cams]  # T̄_world_cam = T̄_cam_world⁻¹
        self._p_sqrt_w = np.sqrt(np.repeat(problem.pose_prior_weights[cams], 3, axis=1))
        self._p_columns = _pose_columns(cams)
        free = np.concatenate(
            [
                np.repeat(layout.pose_free, POSE_DIM),
                np.repeat(layout.intrinsics_free, n_params),
                np.repeat(layout.affine_free, AFFINE_DIM),
            ]
        )
        self._free = np.flatnonzero(free)
        self._scale_rows = (
            POSE_DIM * layout.scale_pose + np.arange(POSE_DIM)
            if layout.scale_pose >= 0
            else np.zeros(0, dtype=np.int64)
        )

    @property
    def loss_index(self) -> IndexArray:
        """Each block's loss: 0 reprojection, 1 depth prior, 2 pose prior."""
        sizes = (self._cam.size, self._d_cam.size, self._p_cam.size)
        return np.repeat(np.arange(3, dtype=np.int64), sizes)

    def intrinsics_positions(self) -> tuple[IndexArray, IndexArray]:
        """Return the free intrinsics' (group, parameter) pairs and their free coordinates."""
        n_params = self._problem.intrinsics.shape[1]
        groups = np.flatnonzero(self._layout.intrinsics_free)
        full = (self._intrinsics_offset + n_params * groups[:, None] + np.arange(n_params)).ravel()
        return groups, np.searchsorted(self._free, full)

    def reduction(self, state: _State) -> _Reduction:
        """Return the free camera-side coordinates at ``state`` (the scale basis moves)."""
        layout = self._layout
        if layout.scale_pose < 0:
            basis = np.zeros((0, 0))
        else:
            centres = _centres(state.poses[[layout.scale_pose, layout.scale_reference]])
            baseline = centres[0] - centres[1]
            baseline /= np.linalg.norm(baseline)
            basis = _householder_complement(
                np.concatenate([-baseline, np.cross(baseline, centres[0])])
            )
        return _Reduction(self._free, self._scale_rows, basis, self._full_size)

    @staticmethod
    def _in_camera(state: _State, cam: IndexArray, pt: IndexArray) -> Array:
        """Return each point in its camera's frame."""
        in_camera = np.einsum("mij,mj->mi", state.poses[cam, :3, :3], state.points[pt])
        return np.asarray(in_camera + state.poses[cam, :3, 3], dtype=np.float64)

    def _reprojection(self, state: _State) -> Array:
        """Return the weighted reprojection errors (rows of NaN for points behind)."""
        in_camera = self._in_camera(state, self._cam, self._pt)
        out = np.full((in_camera.shape[0], 2), np.nan)
        valid = in_camera[:, 2] > camera.MIN_DEPTH
        for group, params in enumerate(state.intrinsics):
            sel = valid & (self._group == group)
            if np.any(sel):
                out[sel] = camera.project(self._problem.camera_model, params, in_camera[sel])
        return (out - self._pixels) * self._sqrt_w[:, None]

    def _depth(self, state: _State) -> Array:
        """Return the weighted inverse-depth prior errors, shape (Md, 1) (NaN if behind)."""
        z = self._in_camera(state, self._d_cam, self._d_pt)[:, 2]
        inverse = 1.0 / np.where(z > camera.MIN_DEPTH, z, np.nan)
        scale, shift = state.depth_affine[self._d_cam].T
        error = (inverse - (scale * self._d_prior + shift)) * self._d_sqrt_w
        return np.asarray(error[:, None], dtype=np.float64)

    def _pose_error(self, state: _State) -> Array:
        """Return the unweighted pose-prior errors ``log(T̄⁻¹ T)``, shape (Cp, 6)."""
        if self._p_cam.size == 0:
            return np.zeros((0, POSE_DIM))
        return se3.log(se3.compose(self._p_prior, state.poses[self._p_cam]))

    def _pose_prior(self, state: _State) -> Array:
        """Return the weighted pose-prior errors, shape (Cp, 6)."""
        return np.asarray(self._pose_error(state) * self._p_sqrt_w, dtype=np.float64)

    def squared_norms(self, state: _State) -> Array:
        """Return every block's squared weighted error (inf if its point is behind)."""
        squared = np.concatenate(
            [
                np.sum(self._reprojection(state) ** 2, axis=1),
                np.sum(self._depth(state) ** 2, axis=1),
                np.sum(self._pose_prior(state) ** 2, axis=1),
            ]
        )
        return np.where(np.isnan(squared), np.inf, squared)

    def _reprojection_kind(self, state: _State) -> _Kind:
        """Linearise the reprojection blocks."""
        in_camera = self._in_camera(state, self._cam, self._pt)
        n_obs, n_params = in_camera.shape[0], state.intrinsics.shape[1]
        d_point = np.zeros((n_obs, 2, 3))
        d_intrinsics = np.zeros((n_obs, 2, n_params))
        for group, params in enumerate(state.intrinsics):
            sel = self._group == group
            if np.any(sel):
                d_point[sel], d_intrinsics[sel] = camera.project_jacobians(
                    self._problem.camera_model, params, in_camera[sel]
                )
        scale = self._sqrt_w[:, None, None]
        point = scale * np.einsum("mij,mjk->mik", d_point, state.poses[self._cam, :3, :3])
        rotation_part = -np.einsum("mij,mjk->mik", point, so3.hat(state.points[self._pt]))
        camera_side = np.concatenate([point, rotation_part, scale * d_intrinsics], axis=2)
        slots = self._layout.point_slot[self._pt]
        return _Kind(self._reprojection(state), camera_side, self._columns, point, slots)

    def _depth_kind(self, state: _State) -> _Kind:
        """Linearise the depth-prior blocks: ``d(1/z) = -dz / z²``."""
        z = self._in_camera(state, self._d_cam, self._d_pt)[:, 2]
        d_inverse = np.zeros((z.shape[0], 1, 3))
        d_inverse[:, 0, 2] = -self._d_sqrt_w / z**2
        point = np.einsum("mij,mjk->mik", d_inverse, state.poses[self._d_cam, :3, :3])
        rotation_part = -np.einsum("mij,mjk->mik", point, so3.hat(state.points[self._d_pt]))
        d_affine = (
            -self._d_sqrt_w[:, None, None]
            * np.stack([self._d_prior, np.ones_like(self._d_prior)], axis=-1)[:, None, :]
        )
        camera_side = np.concatenate([point, rotation_part, d_affine], axis=2)
        slots = self._layout.point_slot[self._d_pt]
        return _Kind(self._depth(state), camera_side, self._d_columns, point, slots)

    def _pose_prior_kind(self, state: _State) -> _Kind:
        """Linearise the pose-prior blocks: ``d log(E exp(δ)) = Jr⁻¹(e) dδ``."""
        error = self._pose_error(state)
        residuals = np.asarray(error * self._p_sqrt_w, dtype=np.float64)
        jacobian = (
            se3.right_jacobian_inverse(error)
            if error.shape[0]
            else np.zeros((0, POSE_DIM, POSE_DIM))
        )
        camera_side = self._p_sqrt_w[:, :, None] * jacobian
        empty = np.zeros(0, dtype=np.int64)
        return _Kind(residuals, camera_side, self._p_columns, None, empty)

    def kinds(self, state: _State) -> tuple[_Kind, _Kind, _Kind]:
        """Linearise every block at ``state``: reprojection, depth prior, pose prior."""
        return (
            self._reprojection_kind(state),
            self._depth_kind(state),
            self._pose_prior_kind(state),
        )

    def linearize(self, state: _State, weights: Array) -> SchurSystem:
        """Form the normal equations at ``state`` and eliminate the points."""
        kinds = self.kinds(state)
        n_free = int(self._layout.point_slot.max(initial=-1)) + 1
        hessian_cc = np.zeros((self._full_size, self._full_size))
        gradient_c = np.zeros(self._full_size)
        hessian_pp = np.zeros((n_free, 3, 3))
        gradient_p = np.zeros((n_free, 3))
        width = max(k.columns.shape[1] for k in kinds if k.point is not None)
        coupling, columns, slots = [], [], []
        splits = np.cumsum([k.residuals.shape[0] for k in kinds])[:-1]
        for kind, w in zip(kinds, np.split(weights, splits), strict=True):
            weighted_c = kind.camera_side * w[:, None, None]
            np.add.at(
                hessian_cc,
                (kind.columns[:, :, None], kind.columns[:, None, :]),
                np.einsum("mki,mkj->mij", weighted_c, kind.camera_side),
            )
            np.add.at(gradient_c, kind.columns, np.einsum("mki,mk->mi", weighted_c, kind.residuals))
            if kind.point is None:
                continue
            on = kind.slots >= 0
            weighted_p = kind.point[on] * w[on, None, None]
            np.add.at(
                hessian_pp, kind.slots[on], np.einsum("mki,mkj->mij", weighted_p, kind.point[on])
            )
            np.add.at(
                gradient_p, kind.slots[on], np.einsum("mki,mk->mi", weighted_p, kind.residuals[on])
            )
            pad = width - kind.columns.shape[1]
            block = np.einsum("mki,mkj->mij", weighted_c[on], kind.point[on])
            coupling.append(np.pad(block, ((0, 0), (0, pad), (0, 0))))
            columns.append(np.pad(kind.columns[on], ((0, 0), (0, pad))))
            slots.append(kind.slots[on])
        blocks = Blocks(
            hessian_cc=hessian_cc,
            gradient_c=gradient_c,
            hessian_pp=hessian_pp,
            gradient_p=gradient_p,
            coupling=np.concatenate(coupling),
            columns=np.concatenate(columns),
            slots=np.concatenate(slots),
        )
        return SchurSystem(self.reduction(state), blocks, self._solver)

    def retract(self, state: _State, delta: Array) -> _State:
        """Apply a step in the free coordinates."""
        reduction = self.reduction(state)
        full = reduction.expand(delta[: reduction.size])
        n_cams = state.poses.shape[0]
        twists = full[: self._intrinsics_offset].reshape(n_cams, POSE_DIM)
        moving = np.flatnonzero(np.any(twists != 0.0, axis=1))
        poses = state.poses.copy()
        if moving.size:
            poses[moving] = se3.compose(state.poses[moving], se3.exp(twists[moving]))
        intrinsics = state.intrinsics + full[self._intrinsics_offset : self._affine_offset].reshape(
            state.intrinsics.shape
        )
        affine = state.depth_affine + full[self._affine_offset :].reshape(n_cams, AFFINE_DIM)
        points = state.points.copy()
        free = self._layout.point_slot >= 0
        points[free] += delta[reduction.size :].reshape(-1, 3)
        return _State(poses, intrinsics, points, affine)


def _centres(t_cam_world: Array) -> Array:
    """Return the camera centres ``-Rᵀ t`` of world-to-camera poses."""
    return np.asarray(
        -np.einsum("cji,cj->ci", t_cam_world[:, :3, :3], t_cam_world[:, :3, 3]), dtype=np.float64
    )


# --- entry point ----------------------------------------------------------------------------


def _intrinsics_std(bundle: _Bundle, state: _State, loss: nls.BlockLoss) -> Array:
    """Return the free intrinsics' marginal standard deviations at ``state`` (NaN if fixed)."""
    out = np.full(state.intrinsics.shape, np.nan)
    groups, positions = bundle.intrinsics_positions()
    if groups.size == 0:
        return out
    weights = loss.evaluate(bundle.squared_norms(state))[1]
    variances = bundle.linearize(state, weights).camera_variances(positions)
    out[groups] = np.sqrt(variances).reshape(groups.size, -1)
    return out


def _unobservable_focal(intrinsics: Array, std: Array, ratio: float) -> int:
    """Count free groups whose focal length is uncertain by more than ``ratio`` of itself."""
    relative = std[:, :2] / intrinsics[:, :2]
    return int(np.count_nonzero(np.any(relative > ratio, axis=1)))


def _solve_reference(problem: BAProblem, options: BAOptions) -> BAResult:
    """Run bundle adjustment with the Python reference."""
    layout = _layout(problem, options)
    bundle = _Bundle(problem, layout, DenseCholesky())
    block_loss = nls.BlockLoss(
        (options.solver.loss, options.depth_loss, nls.Loss()), bundle.loss_index
    )
    start = _State(
        se3.inverse(problem.poses),
        problem.intrinsics.copy(),
        problem.points.copy(),
        problem.depth_affine.copy(),
    )
    solution = nls.solve(bundle, start, replace(options.solver, block_loss=block_loss))
    final = solution.state
    poses = se3.inverse(final.poses)
    unmoved = np.all(final.poses == start.poses, axis=(1, 2))
    poses[unmoved] = problem.poses[unmoved]  # bit-exact, not a round trip through the inverse
    std = _intrinsics_std(bundle, final, block_loss)
    summary = BASummary(
        solve=solution.summary,
        gauge=layout.gauge,
        unobservable_focal=_unobservable_focal(final.intrinsics, std, options.focal_std_ratio),
        **layout.summary,
    )
    return BAResult(poses, final.intrinsics, final.points, final.depth_affine, std, summary)


def _result_from_backend(raw: BackendResult) -> BAResult:
    """Assemble a compiled backend's arrays into a :class:`BAResult`."""
    poses, intrinsics, points, affine, std, trace, counts, costs = raw
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
    return BAResult(poses, intrinsics, points, affine, std, summary)


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
        problem.fixed_intrinsics,
        problem.fixed_points,
        problem.depth_prior,
        problem.depth_prior_weights,
        problem.depth_affine,
        problem.fixed_depth_affine,
        problem.pose_prior,
        problem.pose_prior_weights,
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
    raw = backends.function(backend, __name__, "solve")(*_backend_arguments(problem, opts))
    return _result_from_backend(raw)
