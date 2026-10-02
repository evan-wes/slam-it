"""Bundle adjustment's inputs and outputs: the problem, its options, the result and summary.

A problem crosses every boundary as a struct of arrays (design doc, "Conventions");
:meth:`BAProblem.create` validates it once, so the solvers can trust it.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab.foundations import camera, nls, se3
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
    fixed_poses, fixed_points : ndarray of bool
        Which poses (C,) and points (N,) are held fixed.
    fixed_intrinsics : ndarray of bool, shape (K, P)
        Which intrinsics parameters are held fixed, per group and parameter.
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
    point_anchor : ndarray of int64, shape (N,)
        Each point's anchor observation, or -1. An anchored point is one inverse depth along
        its anchor pixel's ray, in the anchor's camera (see :func:`anchor_points`).
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
    point_anchor: IndexArray

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
        point_anchor: ArrayLike | None = None,
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
        fixed_intrinsics : array_like of bool, shape (K,) or (K, P), optional
            Intrinsics held fixed, per group or per parameter (default: all, as for a
            calibrated camera). BAL, for example, frees ``f, k1, k2`` but not ``cx, cy``.
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
        point_anchor : array_like of int, shape (N,), optional
            Each point's anchor observation (one of its own), or -1 for a plain 3D point
            (default: all -1).

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
            fixed_intrinsics=_intrinsics_mask(fixed_intrinsics, k.shape),
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
            point_anchor=_anchors(point_anchor, x.shape[0], np.asarray(pt_idx)),
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
        A free point whose rays span less than this angle is dropped (default 0: only points
        seen fewer than twice are). Bundle adjustment refines low-parallax points well;
        raise it only to discard poorly triangulated points deliberately.
    focal_std_ratio : float
        A free focal length whose standard deviation exceeds this fraction of it is counted
        as unobservable.
    """

    solver: nls.Options = field(default_factory=nls.Options)
    depth_loss: nls.Loss = field(default_factory=nls.Loss)
    gauge: str = "auto"
    min_parallax_deg: float = 0.0
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
    inverse_depth : ndarray of float64, shape (N,)
        Anchored points' refined inverse depths in their anchor camera (NaN for the others).
    intrinsics_std : ndarray of float64, shape (K, P)
        Free intrinsics' standard deviations (NaN where held fixed; inf if unobservable).
    summary : BASummary
        How the solve went.
    """

    poses: Array
    intrinsics: Array
    points: Array
    depth_affine: Array
    inverse_depth: Array
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


def _anchors(value: ArrayLike | None, length: int, pt_idx: IndexArray) -> IndexArray:
    """Return anchor observations: -1, or an observation of the anchored point itself."""
    if value is None:
        return np.full(length, -1, dtype=np.int64)
    arr = np.asarray(value)
    if arr.shape != (length,) or not np.issubdtype(arr.dtype, np.integer):
        raise InvalidInputError(f"point_anchor: expected {length} integers, got {arr.shape}")
    anchored = np.flatnonzero(arr >= 0)
    if np.any(arr < -1) or np.any(arr[anchored] >= pt_idx.shape[0]):
        raise InvalidInputError("point_anchor: expected -1 or an observation index")
    if np.any(pt_idx[arr[anchored]] != anchored):
        raise InvalidInputError("point_anchor: a point's anchor must be one of its observations")
    return arr.astype(np.int64)


def anchor_points(problem: BAProblem) -> BAProblem:
    """Anchor every free point at its first weighted observation, as an inverse depth.

    Parameters
    ----------
    problem : BAProblem
        The problem.

    Returns
    -------
    BAProblem
        The same problem with ``point_anchor`` set for each free point that has a weighted
        observation; fixed points and unobserved points stay plain 3D points.
    """
    anchor = np.full(problem.points.shape[0], -1, dtype=np.int64)
    weighted = np.flatnonzero(problem.weights > 0.0)[::-1]
    anchor[problem.pt_idx[weighted]] = weighted  # the last write per point is its first
    anchor[problem.fixed_points] = -1
    return replace(problem, point_anchor=anchor)


def _intrinsics_mask(value: ArrayLike | None, shape: tuple[int, int]) -> Mask:
    """Return a (K, P) mask of fixed intrinsics from a per-group or per-parameter one."""
    if value is None:
        return np.ones(shape, dtype=bool)
    arr = np.asarray(value)
    if arr.dtype != np.bool_ or arr.shape not in {(shape[0],), shape}:
        raise InvalidInputError(
            f"fixed_intrinsics: expected booleans of shape ({shape[0]},) or {shape}, "
            f"got {arr.dtype} {arr.shape}"
        )
    return np.broadcast_to(arr.reshape(shape[0], -1), shape).copy()


def _mask(value: ArrayLike | None, length: int, default: bool, name: str) -> Mask:
    """Return a bool mask of the given length."""
    if value is None:
        return np.full(length, default)
    arr = np.asarray(value)
    if arr.shape != (length,) or arr.dtype != np.bool_:
        raise InvalidInputError(f"{name}: expected {length} booleans, got {arr.dtype} {arr.shape}")
    return arr.copy()
