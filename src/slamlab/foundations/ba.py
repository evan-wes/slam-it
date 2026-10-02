"""Bundle adjustment: refine camera poses, intrinsics and 3D points against their images.

The cost is the robust sum of reprojection errors, one 2-row block per observation,

    cost = ½ Σ_m ρ(w_m |π(K_g(m), T_c(m) X_p(m)) - u_m|²),

where ``T_c`` is camera c's world-to-camera pose, ``K_g`` its intrinsics group's parameters,
``X_p`` a world point, ``u_m`` the observed pixel and ``w_m`` the observation's weight (an
information, ``1/σ²`` in px⁻²). It is minimised by :func:`slamlab.foundations.nls.solve`.

**Parameters.**
- Poses are perturbed on the right of ``T_cam_world``, ``T ⊕ δ = T exp(δ)``, so ``δ = (ρ, θ)``
  is expressed in the world frame.
- Points are plain 3-vectors.
- Intrinsics are shared through ``intrinsics_idx``: several cameras can use one group, or
  each its own.

Any pose, intrinsics group or point can be held fixed. Callers pass and receive
camera-to-world poses ``T_world_cam`` (design doc, "Conventions").

**The Schur complement.** Points are eliminated before the solve. Each point's 3x3 block
is inverted, and the reduced camera system, poses and intrinsics only, is factored by a
:class:`ReducedSystemSolver` (dense Cholesky here; a sparse solver arrives with larger
problems). The points then follow by back-substitution. This is the
:class:`~slamlab.foundations.nls.LinearSystem` that bundle adjustment hands the NLS loop.

**Gauge.** Reprojection errors don't change if the whole scene moves rigidly, or, for a
monocular camera, scales. Those directions are removed, not left to the damping:

- ``"first_pose"`` (SE(3) gauge): if no pose is fixed, the first free observed pose is.
- ``"similarity"`` (Sim(3) gauge, the default for monocular data): as ``"first_pose"``. Then,
  unless two fixed poses already pin the scale, the free pose farthest from a fixed one may
  not move its centre along that baseline. Its 6-dimensional step is confined to the
  5-dimensional subspace orthogonal (to first order) to the change ``dc = -ρ + cross(c, θ)`` of
  its centre ``c`` along the baseline. The basis of that subspace is a Householder
  reflection, deterministic in every backend.
- ``"none"``: the caller's fixed variables define the gauge.

**Degenerate data is dropped, and counted:**
- observations of a point behind its camera at the start;
- every observation of a free point seen fewer than twice, or with less than
  ``min_parallax_deg`` between its rays.

A point with no parallax has an undetermined depth. Freezing it at a wrong depth would bias
the cameras that see it. Free poses and intrinsics groups left with no observations are
held fixed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
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

GAUGES = ("none", "first_pose", "similarity")
"""The gauge choices :class:`BAOptions` accepts, in the order of their integer codes."""

POSE_DIM = 6
"""A pose's tangent dimension."""

PAIR_CHUNK = 4096
"""Observation pairs processed at once when forming the Schur complement (bounds memory)."""


class Implementation(Protocol):
    """The function a compiled backend's ``foundations.ba`` submodule provides."""

    def solve(  # noqa: PLR0913, PLR0917 -- a problem crosses as a struct of arrays
        self,
        model: int,
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
        gauge: int,
        min_parallax_deg: float,
        method: int,
        loss: int,
        loss_scale: float,
        max_iterations: int,
        tolerances: Array,
        strict: bool,
    ) -> tuple[Array, Array, Array, Array, IndexArray, Array]:
        """Solve; return poses, intrinsics, points, the trace, counts and costs."""
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
        t = as_real(poses, "poses")
        if t.ndim != 3 or t.shape[1:] != (4, 4):
            raise InvalidInputError(f"poses: expected shape (C, 4, 4), got {t.shape}")
        se3.check_transform(t)
        x = _shaped(points, (-1, 3), "points")
        u = _shaped(observations, (-1, 2), "observations")
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
            weights=_weights(weights, n_obs),
            fixed_poses=_mask(fixed_poses, n_cams, False, "fixed_poses"),
            fixed_intrinsics=_mask(fixed_intrinsics, k.shape[0], True, "fixed_intrinsics"),
            fixed_points=_mask(fixed_points, x.shape[0], False, "fixed_points"),
        )


@dataclass(frozen=True)
class BAOptions:
    """Settings for :func:`solve`.

    Attributes
    ----------
    solver : slamlab.foundations.nls.Options
        The NLS method, loss, tolerances and limits.
    gauge : {"similarity", "first_pose", "none"}
        How the gauge freedom is removed (see the module docstring).
    min_parallax_deg : float
        A free point whose rays span less than this angle is dropped.
    """

    solver: nls.Options = field(default_factory=nls.Options)
    gauge: str = "similarity"
    min_parallax_deg: float = 1.0

    def __post_init__(self) -> None:
        """Reject an unknown gauge or a negative parallax threshold."""
        if self.gauge not in GAUGES:
            raise InvalidInputError(f"gauge: expected one of {GAUGES}, got {self.gauge!r}")
        if not (np.isfinite(self.min_parallax_deg) and self.min_parallax_deg >= 0.0):
            raise InvalidInputError("min_parallax_deg: must be non-negative and finite")


@dataclass(frozen=True)
class BASummary:
    """How a bundle adjustment went.

    Attributes
    ----------
    solve : slamlab.foundations.nls.SolveSummary
        The NLS solve's summary; its costs are the robust reprojection cost.
    gauge_pose : int
        The pose fixed to remove the rigid gauge, or -1 if none was.
    scale_pose : int
        The pose whose step was confined to remove the scale gauge, or -1.
    dropped_points : int
        Free points dropped as degenerate (too few views or too little parallax).
    dropped_observations : int
        Observations left out: those of dropped points and those behind their camera.
    unobserved_poses, unobserved_intrinsics : int
        Free poses and intrinsics groups held fixed because nothing observes them.
    """

    solve: nls.SolveSummary
    gauge_pose: int
    scale_pose: int
    dropped_points: int
    dropped_observations: int
    unobserved_poses: int
    unobserved_intrinsics: int


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
    summary : BASummary
        How the solve went.
    """

    poses: Array
    intrinsics: Array
    points: Array
    summary: BASummary


# --- validation ---------------------------------------------------------------------------


def _shaped(value: ArrayLike, shape: tuple[int, int], name: str) -> Array:
    """Return a finite real 2D array with the given column count."""
    arr = as_real(value, name)
    if arr.ndim != 2 or arr.shape[1] != shape[1]:
        raise InvalidInputError(f"{name}: expected shape (n, {shape[1]}), got {arr.shape}")
    return arr


def _indices(value: ArrayLike, length: int, bound: int, name: str) -> IndexArray:
    """Return an int64 index array of the given length with every entry in ``[0, bound)``."""
    arr = np.asarray(value)
    if arr.shape != (length,) or not np.issubdtype(arr.dtype, np.integer):
        raise InvalidInputError(f"{name}: expected {length} integers, got {arr.dtype} {arr.shape}")
    if length and (arr.min() < 0 or arr.max() >= bound):
        raise InvalidInputError(f"{name}: indices must lie in [0, {bound})")
    return arr.astype(np.int64)


def _weights(value: ArrayLike | None, length: int) -> Array:
    """Return non-negative weights of the given length (default: ones)."""
    if value is None:
        return np.ones(length)
    w = as_real(value, "weights")
    if w.shape != (length,) or np.any(w < 0.0):
        raise InvalidInputError(f"weights: expected {length} non-negative values")
    return w


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
    """Which observations count and which variables move, decided once from the start."""

    obs: NDArray[np.int64]
    """Indices of the observations kept."""
    pose_free: Mask
    """Poses with all six directions free (not fixed, observed, not the gauge poses)."""
    scale_pose: int
    scale_reference: int
    intrinsics_free: Mask
    point_slot: NDArray[np.int64]
    """Each point's index among the free points, or -1."""
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


def _layout(problem: BAProblem, options: BAOptions) -> _Layout:
    """Drop degenerate observations, hold unobserved variables fixed, and fix the gauge."""
    n_cams, n_points = problem.poses.shape[0], problem.points.shape[0]
    t_cam_world = se3.inverse(problem.poses)
    obs = np.flatnonzero(problem.weights > 0.0)
    cam, pt = problem.cam_idx[obs], problem.pt_idx[obs]
    depth = (
        np.einsum("mj,mj->m", t_cam_world[cam, 2, :3], problem.points[pt]) + t_cam_world[cam, 2, 3]
    )
    in_front = depth > camera.MIN_DEPTH
    behind = int(np.count_nonzero(~in_front))
    obs, cam, pt = obs[in_front], cam[in_front], pt[in_front]
    views = np.bincount(pt, minlength=n_points)
    parallax = _ray_parallax(problem.poses[:, :3, 3], problem.points, cam, pt)
    degenerate = ~problem.fixed_points & (
        (views < 2) | (parallax < np.deg2rad(options.min_parallax_deg))
    )
    degenerate &= views > 0
    keep = ~degenerate[pt]
    obs, cam, pt = obs[keep], cam[keep], pt[keep]
    observed = np.bincount(cam, minlength=n_cams) > 0
    intrinsics_observed = (
        np.bincount(problem.intrinsics_idx[cam], minlength=problem.intrinsics.shape[0]) > 0
    )
    pose_free, gauge_pose, scale_pose, scale_reference = _gauge(
        problem, ~problem.fixed_poses & observed, observed, options.gauge
    )
    point_free = ~problem.fixed_points & ~degenerate & (views > 0)
    point_slot = np.full(n_points, -1, dtype=np.int64)
    point_slot[point_free] = np.arange(np.count_nonzero(point_free))
    summary = {
        "gauge_pose": gauge_pose,
        "scale_pose": scale_pose,
        "dropped_points": int(np.count_nonzero(degenerate)),
        "dropped_observations": behind + int(np.count_nonzero(~keep)),
        "unobserved_poses": int(np.count_nonzero(~problem.fixed_poses & ~observed)),
        "unobserved_intrinsics": int(
            np.count_nonzero(~problem.fixed_intrinsics & ~intrinsics_observed)
        ),
    }
    return _Layout(
        obs=obs,
        pose_free=pose_free,
        scale_pose=scale_pose,
        scale_reference=scale_reference,
        intrinsics_free=~problem.fixed_intrinsics & intrinsics_observed,
        point_slot=point_slot,
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
    """Maps between the full camera-side vector (6 per pose, P per group) and the free one.

    The free coordinates are the free full entries, in order, then the scale pose's five.
    """

    free: NDArray[np.int64]
    rows: NDArray[np.int64]
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


# --- the problem the NLS loop drives -------------------------------------------------------


@dataclass(frozen=True)
class _State:
    """The variables: world-to-camera poses, intrinsics groups, points."""

    poses: Array
    intrinsics: Array
    points: Array


@dataclass(frozen=True)
class _Linearisation:
    """Per-observation Jacobians and residuals at a state (rows already scaled by √w)."""

    residuals: Array
    camera_side: Array
    """d r / d(pose, intrinsics), shape (M, 2, 6 + P)."""
    point: Array
    """d r / d point, shape (M, 2, 3)."""
    columns: NDArray[np.int64]
    """Each observation's camera-side columns in the full vector, shape (M, 6 + P)."""


@dataclass(frozen=True)
class Blocks:
    """The blocks of the normal equations, formed once per linearisation.

    Attributes
    ----------
    hessian_cc, gradient_c : ndarray of float64
        The full camera-side Hessian (n, n) and gradient (n,).
    hessian_pp, gradient_p : ndarray of float64
        Each free point's 3x3 Hessian block (Nf, 3, 3) and gradient (Nf, 3).
    coupling : ndarray of float64, shape (Mf, 6 + P, 3)
        Each free-point observation's camera-point block ``J_cᵀ W J_p``.
    columns : ndarray of int64, shape (Mf, 6 + P)
        Those observations' camera-side columns.
    slots : ndarray of int64, shape (Mf,)
        Those observations' free-point slots.
    """

    hessian_cc: Array
    gradient_c: Array
    hessian_pp: Array
    gradient_p: Array
    coupling: Array
    columns: NDArray[np.int64]
    slots: NDArray[np.int64]


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
        self._hessian_cc = blocks.hessian_cc
        self._gradient_c = blocks.gradient_c
        self._hessian_pp = blocks.hessian_pp
        self._gradient_p = blocks.gradient_p
        self._coupling = blocks.coupling
        self._columns = blocks.columns
        self._slots = blocks.slots
        self._solver = solver
        slots = blocks.slots
        hessian_pp = blocks.hessian_pp
        order = np.argsort(slots, kind="stable")
        counts = np.bincount(slots[order], minlength=hessian_pp.shape[0])
        per_obs = counts[slots[order]]
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])[slots[order]]
        offsets = np.arange(int(per_obs.sum())) - np.repeat(np.cumsum(per_obs) - per_obs, per_obs)
        self._pairs = (
            np.repeat(order, per_obs),
            order[np.repeat(starts, per_obs) + offsets],
        )

    @property
    def gradient(self) -> Array:
        """The gradient in the free coordinates: cameras, then points."""
        return np.concatenate(
            [self._reduction.reduce_vector(self._gradient_c), self._gradient_p.reshape(-1)]
        )

    @property
    def hessian_diagonal(self) -> Array:
        """The Hessian's diagonal in the free coordinates."""
        return np.concatenate(
            [
                self._reduction.reduce_diagonal(self._hessian_cc),
                np.diagonal(self._hessian_pp, axis1=1, axis2=2).reshape(-1),
            ]
        )

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
        n_cam = self._reduction.size
        damped_pp = self._hessian_pp + damping[n_cam:].reshape(-1, 3)[:, :, None] * np.eye(3)
        try:
            lower = np.linalg.cholesky(damped_pp)
        except np.linalg.LinAlgError:
            return None
        lower_inv = np.linalg.inv(lower)
        inverse_pp = np.einsum("pki,pkj->pij", lower_inv, lower_inv)
        weighted = np.einsum("mij,mjk->mik", self._coupling, inverse_pp[self._slots])
        schur = self._hessian_cc.copy()
        first, second = self._pairs
        for start in range(0, first.shape[0], PAIR_CHUNK):
            a, b = first[start : start + PAIR_CHUNK], second[start : start + PAIR_CHUNK]
            blocks = np.einsum("pik,pjk->pij", weighted[a], self._coupling[b])
            np.subtract.at(
                schur, (self._columns[a][:, :, None], self._columns[b][:, None, :]), blocks
            )
        rhs = self._gradient_c.copy()
        np.subtract.at(
            rhs, self._columns, np.einsum("mik,mk->mi", weighted, self._gradient_p[self._slots])
        )
        reduced = self._reduction.reduce_matrix(schur) + np.diag(damping[:n_cam])
        step_c = self._solver.solve(reduced, -self._reduction.reduce_vector(rhs))
        if step_c is None:
            return None
        full_c = self._reduction.expand(step_c)
        back = self._gradient_p.copy()
        np.add.at(back, self._slots, np.einsum("mij,mi->mj", self._coupling, full_c[self._columns]))
        step_p = -np.einsum("pij,pj->pi", inverse_pp, back)
        return np.concatenate([step_c, step_p.reshape(-1)])


class _Bundle:
    """The bundle-adjustment problem as an NLS ``Problem`` over :class:`_State`."""

    def __init__(self, problem: BAProblem, layout: _Layout, solver: ReducedSystemSolver) -> None:
        """Hold the problem, its layout and the reduced solver."""
        self._problem = problem
        self._layout = layout
        self._solver = solver
        obs = layout.obs
        self._cam = problem.cam_idx[obs]
        self._pt = problem.pt_idx[obs]
        self._group = problem.intrinsics_idx[self._cam]
        self._pixels = problem.observations[obs]
        self._sqrt_w = np.sqrt(problem.weights[obs])
        n_cams, (n_groups, n_params) = problem.poses.shape[0], problem.intrinsics.shape
        self._full_size = POSE_DIM * n_cams + n_params * n_groups
        pose_columns = POSE_DIM * self._cam[:, None] + np.arange(POSE_DIM)
        intrinsics_columns = POSE_DIM * n_cams + n_params * self._group[:, None]
        self._columns = np.concatenate(
            [pose_columns, intrinsics_columns + np.arange(n_params)], axis=1
        )
        pose_entries = np.repeat(layout.pose_free, POSE_DIM)
        free = np.concatenate([pose_entries, np.repeat(layout.intrinsics_free, n_params)])
        self._free = np.flatnonzero(free)
        self._scale_rows = (
            POSE_DIM * layout.scale_pose + np.arange(POSE_DIM)
            if layout.scale_pose >= 0
            else np.zeros(0, dtype=np.int64)
        )

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

    def _camera_points(self, state: _State) -> Array:
        """Return every kept observation's point in its camera's frame."""
        rotation = state.poses[self._cam, :3, :3]
        in_camera = np.einsum("mij,mj->mi", rotation, state.points[self._pt])
        return np.asarray(in_camera + state.poses[self._cam, :3, 3], dtype=np.float64)

    def _residuals(self, state: _State) -> Array:
        """Return the weighted reprojection errors (rows of NaN for points behind)."""
        in_camera = self._camera_points(state)
        out = np.full((in_camera.shape[0], 2), np.nan)
        valid = in_camera[:, 2] > camera.MIN_DEPTH
        for group, params in enumerate(state.intrinsics):
            sel = valid & (self._group == group)
            if np.any(sel):
                out[sel] = camera.project(self._problem.camera_model, params, in_camera[sel])
        return (out - self._pixels) * self._sqrt_w[:, None]

    def squared_norms(self, state: _State) -> Array:
        """Return each observation's squared weighted reprojection error (inf if behind)."""
        squared = np.sum(self._residuals(state) ** 2, axis=1)
        return np.where(np.isnan(squared), np.inf, squared)

    def _linearise(self, state: _State) -> _Linearisation:
        """Return the residuals and Jacobians of every kept observation."""
        in_camera = self._camera_points(state)
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
        return _Linearisation(self._residuals(state), camera_side, point, self._columns)

    def linearize(self, state: _State, weights: Array) -> SchurSystem:
        """Form the normal equations at ``state`` and eliminate the points."""
        lin = self._linearise(state)
        weighted_c = lin.camera_side * weights[:, None, None]
        hessian_cc = np.zeros((self._full_size, self._full_size))
        np.add.at(
            hessian_cc,
            (lin.columns[:, :, None], lin.columns[:, None, :]),
            np.einsum("mki,mkj->mij", weighted_c, lin.camera_side),
        )
        gradient_c = np.zeros(self._full_size)
        np.add.at(gradient_c, lin.columns, np.einsum("mki,mk->mi", weighted_c, lin.residuals))
        slots = self._layout.point_slot[self._pt]
        on_free = slots >= 0
        n_free = int(self._layout.point_slot.max(initial=-1)) + 1
        weighted_p = lin.point[on_free] * weights[on_free, None, None]
        hessian_pp = np.zeros((n_free, 3, 3))
        np.add.at(
            hessian_pp,
            slots[on_free],
            np.einsum("mki,mkj->mij", weighted_p, lin.point[on_free]),
        )
        gradient_p = np.zeros((n_free, 3))
        np.add.at(
            gradient_p,
            slots[on_free],
            np.einsum("mki,mk->mi", weighted_p, lin.residuals[on_free]),
        )
        coupling = np.einsum("mki,mkj->mij", weighted_c[on_free], lin.point[on_free])
        blocks = Blocks(
            hessian_cc=hessian_cc,
            gradient_c=gradient_c,
            hessian_pp=hessian_pp,
            gradient_p=gradient_p,
            coupling=coupling,
            columns=lin.columns[on_free],
            slots=slots[on_free],
        )
        return SchurSystem(self.reduction(state), blocks, self._solver)

    def retract(self, state: _State, delta: Array) -> _State:
        """Apply a step in the free coordinates."""
        reduction = self.reduction(state)
        full = reduction.expand(delta[: reduction.size])
        n_cams = state.poses.shape[0]
        moving = np.flatnonzero(
            np.any(full[: POSE_DIM * n_cams].reshape(n_cams, POSE_DIM) != 0.0, axis=1)
        )
        poses = state.poses.copy()
        if moving.size:
            twists = full[: POSE_DIM * n_cams].reshape(n_cams, POSE_DIM)[moving]
            poses[moving] = se3.compose(state.poses[moving], se3.exp(twists))
        intrinsics = state.intrinsics + full[POSE_DIM * n_cams :].reshape(state.intrinsics.shape)
        points = state.points.copy()
        free = self._layout.point_slot >= 0
        points[free] += delta[reduction.size :].reshape(-1, 3)
        return _State(poses, intrinsics, points)


def _centres(t_cam_world: Array) -> Array:
    """Return the camera centres ``-Rᵀ t`` of world-to-camera poses."""
    return np.asarray(
        -np.einsum("cji,cj->ci", t_cam_world[:, :3, :3], t_cam_world[:, :3, 3]), dtype=np.float64
    )


# --- entry point ----------------------------------------------------------------------------


def _solve_reference(problem: BAProblem, options: BAOptions) -> BAResult:
    """Run bundle adjustment with the Python reference."""
    layout = _layout(problem, options)
    bundle = _Bundle(problem, layout, DenseCholesky())
    start = _State(se3.inverse(problem.poses), problem.intrinsics.copy(), problem.points.copy())
    solution = nls.solve(bundle, start, options.solver)
    poses = se3.inverse(solution.state.poses)
    unmoved = np.all(solution.state.poses == start.poses, axis=(1, 2))
    poses[unmoved] = problem.poses[unmoved]  # bit-exact, not a round trip through the inverse
    return BAResult(
        poses=poses,
        intrinsics=solution.state.intrinsics,
        points=solution.state.points,
        summary=BASummary(solve=solution.summary, **layout.summary),
    )


def _result_from_backend(raw: tuple[Array, Array, Array, Array, IndexArray, Array]) -> BAResult:
    """Assemble a compiled backend's arrays into a :class:`BAResult`."""
    poses, intrinsics, points, trace, counts, costs = raw
    terminations = list(nls.Termination)
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
    termination = terminations[int(counts[0])]
    solve_summary = nls.SolveSummary(
        initial_cost=float(costs[0]),
        final_cost=float(costs[1]),
        iterations=len(records),
        termination=termination,
        converged=termination in nls.CONVERGED,
        trace=records,
    )
    names = (
        "gauge_pose",
        "scale_pose",
        "dropped_points",
        "dropped_observations",
        "unobserved_poses",
        "unobserved_intrinsics",
    )
    summary = BASummary(
        solve_summary, **{n: int(v) for n, v in zip(names, counts[1:], strict=True)}
    )
    return BAResult(poses=poses, intrinsics=intrinsics, points=points, summary=summary)


def solve(
    problem: BAProblem, options: BAOptions | None = None, *, backend: str = "python"
) -> BAResult:
    """Refine a bundle-adjustment problem's poses, intrinsics and points.

    Parameters
    ----------
    problem : BAProblem
        The problem (see :meth:`BAProblem.create`).
    options : BAOptions, optional
        Solver settings, gauge and degeneracy threshold (default: ``BAOptions()``).
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
    solver = opts.solver
    raw = backends.function(backend, __name__, "solve")(
        camera.MODELS[problem.camera_model][0],
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
        GAUGES.index(opts.gauge),
        opts.min_parallax_deg,
        nls.METHODS.index(solver.method),
        nls.LOSSES.index(solver.loss.kind),
        solver.loss.scale,
        solver.max_iterations,
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
        solver.strict,
    )
    return _result_from_backend(raw)
