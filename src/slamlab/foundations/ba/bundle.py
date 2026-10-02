"""The bundle-adjustment problem as an NLS ``Problem``: its blocks, Jacobians and retraction.

**Anchored points.** A point with an anchor observation ``a`` (camera h, pixel ``u_a``) is one
inverse depth ``ρ`` along that pixel's ray. The ray ``m = (x, y, 1)`` lies on h's normalised
image plane, undistorted with h's current intrinsics, and

    X_world = T_h⁻¹ (m / ρ).

Every block that sees the point depends on it through ``X_world``. So the world-point
Jacobian ``J_X`` of any block is chained three ways:

- through ``ρ``: ``-R_hᵀ m / ρ²``;
- through the host pose (right perturbation, world frame): ``[-I, hat(X_world)]``;
- through the host intrinsics: ``R_hᵀ (dm/dK) / ρ``, where ``dm/dK = -(∂π/∂m)⁻¹ ∂π/∂K``
  by the implicit function theorem on ``π(K, m) = u_a``.

The anchor observation's own reprojection error is identically zero (it defines the ray),
so it has no block. A depth prior on it remains, and is a prior on ``ρ`` itself.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from slamlab.foundations import camera, se3, so3
from slamlab.foundations.ba.layout import Layout, bearings_or_nan
from slamlab.foundations.ba.problem import AFFINE_DIM, POSE_DIM, Array, BAProblem, IndexArray
from slamlab.foundations.ba.schur import (
    Blocks,
    ReducedSystemSolver,
    Reduction,
    SchurSystem,
    householder_complement,
)


@dataclass(frozen=True)
class State:
    """The variables: world-to-camera poses, intrinsics, points, affines, inverse depths.

    ``points`` holds the plain 3D points (anchored rows are unused); ``inverse_depth`` holds
    the anchored points' ``ρ`` (NaN for the others).
    """

    poses: Array
    intrinsics: Array
    points: Array
    depth_affine: Array
    inverse_depth: Array


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
    """d r / d(point variables), shape (M, d, 3): xyz, or (ρ, 0, 0) for an anchored point;
    None for blocks that touch no point."""
    slots: IndexArray
    """Each block's free-point slot (-1 for none)."""


@dataclass(frozen=True)
class _Anchors:
    """The anchored points' world positions and their Jacobians at a state."""

    world: Array
    """Shape (Na, 3)."""
    d_rho: Array
    """d X_world / d ρ, shape (Na, 3)."""
    d_pose: Array
    """d X_world / d(host pose), shape (Na, 3, 6)."""
    d_intrinsics: Array
    """d X_world / d(host intrinsics), shape (Na, 3, P)."""


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
        obs = layout.obs[layout.reprojection]
        self._cam = problem.cam_idx[obs]
        self._pt = problem.pt_idx[obs]
        self._group = problem.intrinsics_idx[self._cam]
        self._pixels = problem.observations[obs]
        self._sqrt_w = np.sqrt(problem.weights[obs])
        self._columns = np.concatenate(
            [_pose_columns(self._cam), self._intrinsics_columns(self._cam)], axis=1
        )
        depth_obs = layout.obs[layout.depth]
        self._d_cam, self._d_pt = problem.cam_idx[depth_obs], problem.pt_idx[depth_obs]
        self._d_prior = problem.depth_prior[depth_obs]
        self._d_sqrt_w = np.sqrt(problem.depth_prior_weights[depth_obs])
        affine_columns = self._affine_offset + AFFINE_DIM * self._d_cam[:, None]
        self._d_columns = np.concatenate(
            [_pose_columns(self._d_cam), affine_columns + np.arange(AFFINE_DIM)], axis=1
        )
        cams = layout.prior_cams
        self._p_cam = cams
        self._p_prior = problem.pose_prior[cams]  # T̄_world_cam = T̄_cam_world⁻¹
        self._p_sqrt_w = np.sqrt(np.repeat(problem.pose_prior_weights[cams], 3, axis=1))
        self._p_columns = _pose_columns(cams)
        self._a_points = np.flatnonzero(layout.anchored)
        anchor_obs = problem.point_anchor[self._a_points]
        self._a_host = problem.cam_idx[anchor_obs]
        self._a_pixels = problem.observations[anchor_obs]
        self._a_index = np.full(problem.points.shape[0], -1, dtype=np.int64)
        self._a_index[self._a_points] = np.arange(self._a_points.size)
        self._a_columns = np.concatenate(
            [_pose_columns(self._a_host), self._intrinsics_columns(self._a_host)], axis=1
        )
        n_slots = int(layout.point_slot.max(initial=-1)) + 1
        anchored_slot = np.zeros(n_slots, dtype=bool)
        anchored_slot[layout.point_slot[self._a_points][layout.point_slot[self._a_points] >= 0]] = (
            True
        )
        self._anchored_slots = np.flatnonzero(anchored_slot)
        self._real = np.repeat(~anchored_slot, 3)
        self._real[3 * self._anchored_slots] = True
        free = np.concatenate(
            [
                np.repeat(layout.pose_free, POSE_DIM),
                layout.intrinsics_free.ravel(),
                np.repeat(layout.affine_free, AFFINE_DIM),
            ]
        )
        self._free = np.flatnonzero(free)
        self._scale_rows = (
            POSE_DIM * layout.scale_pose + np.arange(POSE_DIM)
            if layout.scale_pose >= 0
            else np.zeros(0, dtype=np.int64)
        )

    def _intrinsics_columns(self, cam: IndexArray) -> IndexArray:
        """Return each camera's intrinsics-group columns in the full vector."""
        n_params = self._problem.intrinsics.shape[1]
        group = self._problem.intrinsics_idx[cam]
        start = self._intrinsics_offset + n_params * group[:, None]
        return np.asarray(start + np.arange(n_params), dtype=np.int64)

    @property
    def loss_index(self) -> IndexArray:
        """Each block's loss: 0 reprojection, 1 depth prior, 2 pose prior."""
        sizes = (self._cam.size, self._d_cam.size, self._p_cam.size)
        return np.repeat(np.arange(3, dtype=np.int64), sizes)

    def start(self) -> State:
        """Return the starting state: anchored points placed on their rays at their depth."""
        p = self._problem
        poses = se3.inverse(p.poses)
        inverse_depth = np.full(p.points.shape[0], np.nan)
        host = poses[self._a_host]
        in_host = (
            np.einsum("aij,aj->ai", host[:, :3, :3], p.points[self._a_points]) + host[:, :3, 3]
        )
        inverse_depth[self._a_points] = 1.0 / in_host[:, 2]
        return State(
            poses, p.intrinsics.copy(), p.points.copy(), p.depth_affine.copy(), inverse_depth
        )

    def intrinsics_positions(self) -> tuple[IndexArray, IndexArray, IndexArray]:
        """Return the free intrinsics' groups, parameters, and free coordinates."""
        n_params = self._problem.intrinsics.shape[1]
        groups, params = np.nonzero(self._layout.intrinsics_free)
        full = self._intrinsics_offset + n_params * groups + params
        return groups, params, np.searchsorted(self._free, full)

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

    def _rays(self, state: State) -> Array:
        """Return each anchor pixel's ray on its host's normalised plane, ``(x, y, 1)``.

        A pixel the current intrinsics cannot undistort gives a NaN ray: its point becomes
        invalid and the step that led there is rejected.
        """
        rays = np.zeros((self._a_points.size, 3))
        group = self._problem.intrinsics_idx[self._a_host]
        for g, params in enumerate(state.intrinsics):
            sel = group == g
            if np.any(sel):
                bearing = bearings_or_nan(self._problem.camera_model, params, self._a_pixels[sel])
                rays[sel] = bearing / bearing[:, 2:3]
        return rays

    def world_points(self, state: State) -> Array:
        """Return every point in the world frame (NaN rows for ``ρ ≤ 0``)."""
        out = state.points.copy()
        if self._a_points.size:
            rho = state.inverse_depth[self._a_points]
            host = state.poses[self._a_host]
            in_host = self._rays(state) / np.where(rho > 0.0, rho, np.nan)[:, None]
            out[self._a_points] = np.einsum("aji,aj->ai", host[:, :3, :3], in_host - host[:, :3, 3])
        return out

    def _anchors(self, state: State) -> _Anchors:
        """Return the anchored points and their Jacobians at ``state``."""
        n_params = state.intrinsics.shape[1]
        n_anchors = self._a_points.size
        rays = self._rays(state)
        rho = state.inverse_depth[self._a_points]
        rotation_t = np.swapaxes(state.poses[self._a_host, :3, :3], 1, 2)
        world = self.world_points(state)[self._a_points]
        d_rho = -np.einsum("aij,aj->ai", rotation_t, rays) / rho[:, None] ** 2
        d_pose = np.concatenate(
            [-np.broadcast_to(np.eye(3), (n_anchors, 3, 3)), so3.hat(world)], axis=2
        )
        d_ray = np.zeros((n_anchors, 3, n_params))
        group = self._problem.intrinsics_idx[self._a_host]
        for g, params in enumerate(state.intrinsics):
            sel = group == g
            if np.any(sel):
                d_point, d_k = camera.project_jacobians(
                    self._problem.camera_model, params, rays[sel]
                )
                d_ray[sel, :2] = -np.linalg.solve(d_point[:, :, :2], d_k)
        d_intrinsics = np.einsum("aij,ajk->aik", rotation_t, d_ray) / rho[:, None, None]
        return _Anchors(world, d_rho, d_pose, d_intrinsics)

    @staticmethod
    def _in_camera(state: State, world: Array, cam: IndexArray, pt: IndexArray) -> Array:
        """Return each point in its camera's frame."""
        in_camera = np.einsum("mij,mj->mi", state.poses[cam, :3, :3], world[pt])
        return np.asarray(in_camera + state.poses[cam, :3, 3], dtype=np.float64)

    def _reprojection(self, state: State, world: Array) -> Array:
        """Return the weighted reprojection errors (rows of NaN for points behind)."""
        in_camera = self._in_camera(state, world, self._cam, self._pt)
        out = np.full((in_camera.shape[0], 2), np.nan)
        valid = in_camera[:, 2] > camera.MIN_DEPTH
        for group, params in enumerate(state.intrinsics):
            sel = valid & (self._group == group)
            if np.any(sel):
                out[sel] = camera.project(self._problem.camera_model, params, in_camera[sel])
        return (out - self._pixels) * self._sqrt_w[:, None]

    def _depth(self, state: State, world: Array) -> Array:
        """Return the weighted inverse-depth prior errors, shape (Md, 1) (NaN if behind)."""
        z = self._in_camera(state, world, self._d_cam, self._d_pt)[:, 2]
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

    def residuals(self, state: State) -> tuple[Array, Array, Array]:
        """Return every block's weighted error: reprojection, depth prior, pose prior."""
        world = self.world_points(state)
        return self._reprojection(state, world), self._depth(state, world), self._pose_prior(state)

    def squared_norms(self, state: State) -> Array:
        """Return every block's squared weighted error (inf if its point is behind)."""
        squared = np.concatenate([np.sum(r**2, axis=1) for r in self.residuals(state)])
        return np.where(np.isnan(squared), np.inf, squared)

    def _couple(
        self, base: Kind, j_world: Array, pt: IndexArray, state: State, anchors: _Anchors
    ) -> Kind:
        """Chain world-point Jacobians through anchored points; widen every block alike.

        Each block gains the host's pose and intrinsics columns (zero, at column 0, for
        blocks on plain points), and an anchored point's Jacobian becomes ``(d/dρ, 0, 0)``.
        """
        m, d = base.residuals.shape
        n_host = POSE_DIM + state.intrinsics.shape[1]
        extra = np.zeros((m, d, n_host))
        extra_columns = np.zeros((m, n_host), dtype=np.int64)
        point = j_world.copy()
        index = self._a_index[pt]
        on = np.flatnonzero(index >= 0)
        if on.size:
            a = index[on]
            jx = j_world[on]
            point[on] = 0.0
            point[on, :, 0] = np.einsum("mij,mj->mi", jx, anchors.d_rho[a])
            extra[on, :, :POSE_DIM] = np.einsum("mij,mjk->mik", jx, anchors.d_pose[a])
            extra[on, :, POSE_DIM:] = np.einsum("mij,mjk->mik", jx, anchors.d_intrinsics[a])
            extra_columns[on] = self._a_columns[a]
        return replace(
            base,
            camera_side=np.concatenate([base.camera_side, extra], axis=2),
            columns=np.concatenate([base.columns, extra_columns], axis=1),
            point=point,
        )

    def _reprojection_kind(self, state: State, world: Array, anchors: _Anchors) -> Kind:
        """Linearise the reprojection blocks."""
        in_camera = self._in_camera(state, world, self._cam, self._pt)
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
        j_world = scale * np.einsum("mij,mjk->mik", d_point, state.poses[self._cam, :3, :3])
        rotation_part = -np.einsum("mij,mjk->mik", j_world, so3.hat(world[self._pt]))
        camera_side = np.concatenate([j_world, rotation_part, scale * d_intrinsics], axis=2)
        slots = self._layout.point_slot[self._pt]
        base = Kind(self._reprojection(state, world), camera_side, self._columns, None, slots)
        return self._couple(base, j_world, self._pt, state, anchors)

    def _depth_kind(self, state: State, world: Array, anchors: _Anchors) -> Kind:
        """Linearise the depth-prior blocks: ``d(1/z) = -dz / z²``."""
        z = self._in_camera(state, world, self._d_cam, self._d_pt)[:, 2]
        d_inverse = np.zeros((z.shape[0], 1, 3))
        d_inverse[:, 0, 2] = -self._d_sqrt_w / z**2
        j_world = np.einsum("mij,mjk->mik", d_inverse, state.poses[self._d_cam, :3, :3])
        rotation_part = -np.einsum("mij,mjk->mik", j_world, so3.hat(world[self._d_pt]))
        d_affine = (
            -self._d_sqrt_w[:, None, None]
            * np.stack([self._d_prior, np.ones_like(self._d_prior)], axis=-1)[:, None, :]
        )
        camera_side = np.concatenate([j_world, rotation_part, d_affine], axis=2)
        slots = self._layout.point_slot[self._d_pt]
        base = Kind(self._depth(state, world), camera_side, self._d_columns, None, slots)
        return self._couple(base, j_world, self._d_pt, state, anchors)

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
        world = self.world_points(state)
        anchors = self._anchors(state)
        return (
            self._reprojection_kind(state, world, anchors),
            self._depth_kind(state, world, anchors),
            self._pose_prior_kind(state),
        )

    def linearize(self, state: State, weights: Array) -> SchurSystem:
        """Form the normal equations at ``state`` and eliminate the points."""
        kinds = self.kinds(state)
        n_free = self._real.shape[0] // 3
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
        # An anchored point's two unused dimensions: inert, so their step is exactly zero.
        hessian_pp[self._anchored_slots, 1, 1] = 1.0
        hessian_pp[self._anchored_slots, 2, 2] = 1.0
        blocks = Blocks(
            hessian_cc=hessian_cc,
            gradient_c=gradient_c,
            hessian_pp=hessian_pp,
            gradient_p=gradient_p,
            coupling=np.concatenate(coupling),
            columns=np.concatenate(columns),
            slots=np.concatenate(slots),
            real=self._real,
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
        padded = np.zeros(self._real.shape[0])
        padded[self._real] = delta[reduction.size :]
        step = padded.reshape(-1, 3)
        slot = self._layout.point_slot
        plain = (slot >= 0) & ~self._layout.anchored
        points = state.points.copy()
        points[plain] += step[slot[plain]]
        inverse_depth = state.inverse_depth.copy()
        free_anchored = self._a_points[slot[self._a_points] >= 0]
        inverse_depth[free_anchored] += step[slot[free_anchored], 0]
        return State(poses, intrinsics, points, affine, inverse_depth)


def centres(t_cam_world: Array) -> Array:
    """Return the camera centres ``-Rᵀ t`` of world-to-camera poses."""
    return np.asarray(
        -np.einsum("cji,cj->ci", t_cam_world[:, :3, :3], t_cam_world[:, :3, 3]), dtype=np.float64
    )
