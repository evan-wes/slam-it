"""The bundle-adjustment problem as an NLS ``Problem``: its blocks, Jacobians and retraction."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from slamlab.foundations import camera, se3, so3
from slamlab.foundations.ba.layout import Layout
from slamlab.foundations.ba.problem import AFFINE_DIM, POSE_DIM, Array, BAProblem, IndexArray
from slamlab.foundations.ba.schur import (
    Blocks,
    ReducedSystemSolver,
    Reduction,
    SchurSystem,
    householder_complement,
)

# --- the problem the NLS loop drives -------------------------------------------------------


@dataclass(frozen=True)
class State:
    """The variables: world-to-camera poses, intrinsics groups, points, depth affines."""

    poses: Array
    intrinsics: Array
    points: Array
    depth_affine: Array


@dataclass(frozen=True)
class Kind:
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


class Bundle:
    """The bundle-adjustment problem as an NLS ``Problem`` over :class:`State`."""

    def __init__(self, problem: BAProblem, layout: Layout, solver: ReducedSystemSolver) -> None:
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

    def reduction(self, state: State) -> Reduction:
        """Return the free camera-side coordinates at ``state`` (the scale basis moves)."""
        layout = self._layout
        if layout.scale_pose < 0:
            basis = np.zeros((0, 0))
        else:
            moving, fixed = centres(state.poses[[layout.scale_pose, layout.scale_reference]])
            baseline = (moving - fixed) / np.linalg.norm(moving - fixed)
            basis = householder_complement(np.concatenate([-baseline, np.cross(baseline, moving)]))
        return Reduction(self._free, self._scale_rows, basis, self._full_size)

    @staticmethod
    def _in_camera(state: State, cam: IndexArray, pt: IndexArray) -> Array:
        """Return each point in its camera's frame."""
        in_camera = np.einsum("mij,mj->mi", state.poses[cam, :3, :3], state.points[pt])
        return np.asarray(in_camera + state.poses[cam, :3, 3], dtype=np.float64)

    def _reprojection(self, state: State) -> Array:
        """Return the weighted reprojection errors (rows of NaN for points behind)."""
        in_camera = self._in_camera(state, self._cam, self._pt)
        out = np.full((in_camera.shape[0], 2), np.nan)
        valid = in_camera[:, 2] > camera.MIN_DEPTH
        for group, params in enumerate(state.intrinsics):
            sel = valid & (self._group == group)
            if np.any(sel):
                out[sel] = camera.project(self._problem.camera_model, params, in_camera[sel])
        return (out - self._pixels) * self._sqrt_w[:, None]

    def _depth(self, state: State) -> Array:
        """Return the weighted inverse-depth prior errors, shape (Md, 1) (NaN if behind)."""
        z = self._in_camera(state, self._d_cam, self._d_pt)[:, 2]
        inverse = 1.0 / np.where(z > camera.MIN_DEPTH, z, np.nan)
        scale, shift = state.depth_affine[self._d_cam].T
        error = (inverse - (scale * self._d_prior + shift)) * self._d_sqrt_w
        return np.asarray(error[:, None], dtype=np.float64)

    def _pose_error(self, state: State) -> Array:
        """Return the unweighted pose-prior errors ``log(T̄⁻¹ T)``, shape (Cp, 6)."""
        if self._p_cam.size == 0:
            return np.zeros((0, POSE_DIM))
        return se3.log(se3.compose(self._p_prior, state.poses[self._p_cam]))

    def _pose_prior(self, state: State) -> Array:
        """Return the weighted pose-prior errors, shape (Cp, 6)."""
        return np.asarray(self._pose_error(state) * self._p_sqrt_w, dtype=np.float64)

    def squared_norms(self, state: State) -> Array:
        """Return every block's squared weighted error (inf if its point is behind)."""
        squared = np.concatenate(
            [
                np.sum(self._reprojection(state) ** 2, axis=1),
                np.sum(self._depth(state) ** 2, axis=1),
                np.sum(self._pose_prior(state) ** 2, axis=1),
            ]
        )
        return np.where(np.isnan(squared), np.inf, squared)

    def _reprojection_kind(self, state: State) -> Kind:
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
        return Kind(self._reprojection(state), camera_side, self._columns, point, slots)

    def _depth_kind(self, state: State) -> Kind:
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
        return Kind(self._depth(state), camera_side, self._d_columns, point, slots)

    def _pose_prior_kind(self, state: State) -> Kind:
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
        return Kind(residuals, camera_side, self._p_columns, None, empty)

    def kinds(self, state: State) -> tuple[Kind, Kind, Kind]:
        """Linearise every block at ``state``: reprojection, depth prior, pose prior."""
        return (
            self._reprojection_kind(state),
            self._depth_kind(state),
            self._pose_prior_kind(state),
        )

    def linearize(self, state: State, weights: Array) -> SchurSystem:
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

    def retract(self, state: State, delta: Array) -> State:
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
        return State(poses, intrinsics, points, affine)


def centres(t_cam_world: Array) -> Array:
    """Return the camera centres ``-Rᵀ t`` of world-to-camera poses."""
    return np.asarray(
        -np.einsum("cji,cj->ci", t_cam_world[:, :3, :3], t_cam_world[:, :3, 3]), dtype=np.float64
    )
