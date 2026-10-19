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

from slam_it.foundations import camera, se3, so3
from slam_it.foundations.arrays import group_members
from slam_it.foundations.ba.gauge import confinement
from slam_it.foundations.ba.layout import Layout
from slam_it.foundations.ba.problem import AFFINE_DIM, POSE_DIM, Array, BAProblem, IndexArray
from slam_it.foundations.ba.schur import (
    Blocks,
    ReducedSystemSolver,
    Reduction,
    SchurSystem,
)


@dataclass(frozen=True)
class State:
    """The variables: world-to-camera poses, intrinsics, points, affines, inverse depths.

    ``points`` holds the plain 3D points (anchored rows are unused); ``inverse_depth`` holds
    the anchored points' ``ρ`` (NaN for the others).
    """

    poses: Array
    """World-to-camera poses ``T_cam_world``, shape (C, 4, 4) (the solver's convention)."""
    intrinsics: Array
    """Intrinsics groups, shape (K, P)."""
    points: Array
    """Plain points in the world frame, shape (N, 3)."""
    depth_affine: Array
    """Per-camera depth affines ``(scale, shift)``, shape (C, 2)."""
    inverse_depth: Array
    """Anchored points' ``ρ``, the inverse of their depth in the host camera, shape (N,)."""


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


def _by_group(group: IndexArray, n_groups: int) -> list[tuple[int, IndexArray]]:
    """Return each non-empty group with the positions of its members."""
    return [(g, m) for g, m in enumerate(group_members(group, n_groups)) if m.size]


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
        self._index_reprojection()
        self._index_depth()
        self._index_pose_priors()
        self._index_anchors()
        self._index_free()

    def _index_reprojection(self) -> None:
        """Index the reprojection blocks: cameras, points, groups, pixels, weights, columns."""
        problem, layout = self._problem, self._layout
        obs = layout.obs[layout.reprojection]
        self._cam = problem.cam_idx[obs]
        self._pt = problem.pt_idx[obs]
        self._by_group = _by_group(problem.intrinsics_idx[self._cam], problem.intrinsics.shape[0])
        self._pixels = problem.observations[obs]
        self._sqrt_w = np.sqrt(problem.weights[obs])
        self._columns = np.concatenate(
            [_pose_columns(self._cam), self._intrinsics_columns(self._cam)], axis=1
        )

    def _index_depth(self) -> None:
        """Index the depth-prior blocks: cameras, points, priors, weights, columns."""
        problem, layout = self._problem, self._layout
        depth_obs = layout.obs[layout.depth]
        self._d_cam, self._d_pt = problem.cam_idx[depth_obs], problem.pt_idx[depth_obs]
        self._d_prior = problem.depth_prior[depth_obs]
        self._d_sqrt_w = np.sqrt(problem.depth_prior_weights[depth_obs])
        affine_columns = self._affine_offset + AFFINE_DIM * self._d_cam[:, None]
        self._d_columns = np.concatenate(
            [_pose_columns(self._d_cam), affine_columns + np.arange(AFFINE_DIM)], axis=1
        )

    def _index_pose_priors(self) -> None:
        """Index the pose-prior blocks: cameras, priors, weights (per row), columns."""
        cams = self._layout.prior_cams
        self._p_cam = cams
        self._p_prior = self._problem.pose_prior[cams]  # T̄_world_cam: centre, orientation
        self._p_sqrt_w = np.sqrt(np.repeat(self._problem.pose_prior_weights[cams], 3, axis=1))
        self._p_columns = _pose_columns(cams)

    def _index_anchors(self) -> None:
        """Index the anchored points: hosts, pixels, columns, and which point slots they own.

        An anchored point's slot keeps one real dimension (its inverse depth) of three.
        """
        problem, layout = self._problem, self._layout
        self._a_points = np.flatnonzero(layout.anchored)
        anchor_obs = problem.point_anchor[self._a_points]
        self._a_host = problem.cam_idx[anchor_obs]
        self._a_pixels = problem.observations[anchor_obs]
        self._a_by_group = _by_group(
            problem.intrinsics_idx[self._a_host], problem.intrinsics.shape[0]
        )
        self._a_index = np.full(problem.points.shape[0], -1, dtype=np.int64)
        self._a_index[self._a_points] = np.arange(self._a_points.size)
        self._a_columns = np.concatenate(
            [_pose_columns(self._a_host), self._intrinsics_columns(self._a_host)], axis=1
        )
        n_slots = int(layout.point_slot.max(initial=-1)) + 1
        slots = layout.point_slot[self._a_points]
        anchored_slot = np.zeros(n_slots, dtype=bool)
        anchored_slot[slots[slots >= 0]] = True
        self._anchored_slots = np.flatnonzero(anchored_slot)
        self._real = np.repeat(~anchored_slot, 3)
        self._real[3 * self._anchored_slots] = True

    def _index_free(self) -> None:
        """Index the free camera-side coordinates and the confined poses' rows."""
        layout = self._layout
        free = np.concatenate(
            [
                np.repeat(layout.pose_free, POSE_DIM),
                layout.intrinsics_free.ravel(),
                np.repeat(layout.affine_free, AFFINE_DIM),
            ]
        )
        self._free = np.flatnonzero(free)
        confined = layout.gauge_plan.confined
        self._confined_rows = (POSE_DIM * confined[:, None] + np.arange(POSE_DIM)).ravel()

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
        """Return the starting state: anchored points placed on their rays at their depth.

        Poses are inverted to ``T_cam_world``; each anchored point's ``ρ`` is ``1/z`` of its
        starting position in its host camera.
        """
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

    def residual_rows(self) -> int:
        """Return the number of weighted residual rows.

        2 per reprojection, 1 per depth prior, and 3 per weighted half of a pose prior (a
        translation-only prior's rotation rows carry no information).
        """
        return int(2 * self._cam.size + self._d_cam.size + np.count_nonzero(self._p_sqrt_w > 0.0))

    def free_size(self, state: State) -> int:
        """Return the number of free parameters (the solver's step length) at ``state``."""
        return self.reduction(state).size + int(np.count_nonzero(self._real))

    def intrinsics_positions(self) -> tuple[IndexArray, IndexArray, IndexArray]:
        """Return the free intrinsics' groups, parameters, and free coordinates.

        Three index arrays of equal length: a free entry ``intrinsics[g, k]`` and its position
        in the step (the reduced camera-side vector).
        """
        n_params = self._problem.intrinsics.shape[1]
        groups, params = np.nonzero(self._layout.intrinsics_free)
        full = self._intrinsics_offset + n_params * groups + params
        return groups, params, np.searchsorted(self._free, full)

    def reduction(self, state: State) -> Reduction:
        """Return the free camera-side coordinates at ``state``.

        Each confined pose's six rows map to a basis of the steps that leave the gauge alone
        (see :func:`gauge.confinement`); the bases move with the poses, so they are rebuilt
        at every state.
        """
        layout = self._layout
        bases = confinement(layout.gauge_anchors, centres(state.poses), layout.gauge_plan)
        basis = np.zeros((POSE_DIM * len(bases), sum(b.shape[1] for b in bases)))
        column = 0
        for i, block in enumerate(bases):
            basis[POSE_DIM * i : POSE_DIM * (i + 1), column : column + block.shape[1]] = block
            column += block.shape[1]
        return Reduction(self._free, self._confined_rows, basis, self._full_size)

    def _rays(self, state: State) -> Array:
        """Return each anchor pixel's ray on its host's normalised plane, ``(x, y, 1)``.

        Shape (Na, 3), in the host camera's frame.

        A pixel the current intrinsics cannot undistort gives a NaN ray: its point becomes
        invalid and the step that led there is rejected.
        """
        rays = np.zeros((self._a_points.size, 3))
        for g, sel in self._a_by_group:
            bearing = camera.unproject_or_nan(
                self._problem.camera_model, state.intrinsics[g], self._a_pixels[sel]
            )
            rays[sel] = bearing / bearing[:, 2:3]
        return rays

    def world_points(self, state: State) -> Array:
        """Return every point in the world frame, shape (N, 3) (NaN rows for ``ρ ≤ 0``).

        Plain points as stored; anchored points ``R_hostᵀ (ray / ρ - t_host)``.
        """
        out = state.points.copy()
        if self._a_points.size:
            rho = state.inverse_depth[self._a_points]
            host = state.poses[self._a_host]
            in_host = self._rays(state) / np.where(rho > 0.0, rho, np.nan)[:, None]
            out[self._a_points] = np.einsum("aji,aj->ai", host[:, :3, :3], in_host - host[:, :3, 3])
        return out

    def _anchors(self, state: State) -> _Anchors:
        """Return the anchored points and their Jacobians at ``state``.

        The intrinsics Jacobian goes through the ray: the undistorted ray ``m(K)`` satisfies
        ``project(K, m) = pixel``, so by the implicit-function theorem
        ``dm/dK = -(∂project/∂m)⁻¹ ∂project/∂K``.
        """
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
        for g, sel in self._a_by_group:
            d_point, d_k = camera.project_jacobians(
                self._problem.camera_model, state.intrinsics[g], rays[sel]
            )
            d_ray[sel, :2] = -np.linalg.solve(d_point[:, :, :2], d_k)
        d_intrinsics = np.einsum("aij,ajk->aik", rotation_t, d_ray) / rho[:, None, None]
        return _Anchors(world, d_rho, d_pose, d_intrinsics)

    @staticmethod
    def _in_camera(state: State, world: Array, cam: IndexArray, pt: IndexArray) -> Array:
        """Return ``world[pt]`` in camera ``cam``'s frame, shape (M, 3)."""
        in_camera = np.einsum("mij,mj->mi", state.poses[cam, :3, :3], world[pt])
        return np.asarray(in_camera + state.poses[cam, :3, 3], dtype=np.float64)

    def _reprojection(self, state: State, world: Array) -> Array:
        """Return the weighted reprojection errors ``√w (project - pixel)``, shape (M, 2).

        In pixels scaled by ``√w``; rows of NaN for points behind their camera.
        """
        in_camera = self._in_camera(state, world, self._cam, self._pt)
        out = np.full((in_camera.shape[0], 2), np.nan)
        valid = in_camera[:, 2] > camera.MIN_DEPTH
        for g, members in self._by_group:
            sel = members[valid[members]]
            out[sel] = camera.project(
                self._problem.camera_model, state.intrinsics[g], in_camera[sel]
            )
        return (out - self._pixels) * self._sqrt_w[:, None]

    def _depth(self, state: State, world: Array) -> Array:
        """Return the weighted inverse-depth prior errors, shape (Md, 1) (NaN if behind)."""
        z = self._in_camera(state, world, self._d_cam, self._d_pt)[:, 2]
        inverse = 1.0 / np.where(z > camera.MIN_DEPTH, z, np.nan)
        scale, shift = state.depth_affine[self._d_cam].T
        error = (inverse - (scale * self._d_prior + shift)) * self._d_sqrt_w
        return np.asarray(error[:, None], dtype=np.float64)

    def _pose_error(self, state: State) -> Array:
        """Return the unweighted pose-prior errors ``(C - C̄, log(R̄_wcᵀ R_wc))``, shape (Cp, 6).

        Decoupled: the translation half compares world camera centres, so it pins the centre
        whatever rotation the prior stores (a GNSS-style prior), and the rotation half the
        camera's world orientation.
        """
        if self._p_cam.size == 0:
            return np.zeros((0, POSE_DIM))
        t_cam_world = state.poses[self._p_cam]
        r_world_cam = np.swapaxes(t_cam_world[:, :3, :3], 1, 2)
        centre_error = centres(t_cam_world) - self._p_prior[:, :3, 3]
        rotation_error = so3.log(np.einsum("cji,cjk->cik", self._p_prior[:, :3, :3], r_world_cam))
        return np.concatenate([centre_error, rotation_error], axis=1)

    def _pose_prior(self, state: State) -> Array:
        """Return the weighted pose-prior errors, shape (Cp, 6)."""
        return np.asarray(self._pose_error(state) * self._p_sqrt_w, dtype=np.float64)

    def residuals(self, state: State) -> tuple[Array, Array, Array]:
        """Return every block's weighted error: reprojection, depth prior, pose prior.

        Shapes (M, 2), (Md, 1) and (Cp, 6), for the kept blocks in layout order.
        """
        world = self.world_points(state)
        return self._reprojection(state, world), self._depth(state, world), self._pose_prior(state)

    def squared_norms(self, state: State) -> Array:
        """Return every block's squared weighted error, shape (M + Md + Cp,).

        In :meth:`residuals` order; inf for a block whose point is behind its camera.
        """
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
        """Linearise the reprojection blocks.

        Camera side: ``[d/dt, d/dθ]`` of the right-perturbed ``T_cam_world`` (translation
        first), then the camera's intrinsics group. Point side: d/d(world point), or d/dρ
        for an anchored point; :meth:`_couple` adds the anchor's host terms.
        """
        in_camera = self._in_camera(state, world, self._cam, self._pt)
        n_obs, n_params = in_camera.shape[0], state.intrinsics.shape[1]
        d_point = np.zeros((n_obs, 2, 3))
        d_intrinsics = np.zeros((n_obs, 2, n_params))
        for g, sel in self._by_group:
            d_point[sel], d_intrinsics[sel] = camera.project_jacobians(
                self._problem.camera_model, state.intrinsics[g], in_camera[sel]
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
        """Linearise the pose-prior blocks.

        Under ``T_cw exp(δ)``, ``δ = (ρ, θ)``: the centre moves by ``-ρ + hat(C) θ``, and
        ``R_wc`` by ``exp(-θ) R_wc``, so the rotation error ``log(E exp(-R_cw θ))`` moves by
        ``-Jr⁻¹(e_r) R_cw θ``.
        """
        error = self._pose_error(state)
        residuals = np.asarray(error * self._p_sqrt_w, dtype=np.float64)
        n_priors = error.shape[0]
        jacobian = np.zeros((n_priors, POSE_DIM, POSE_DIM))
        if n_priors:
            t_cam_world = state.poses[self._p_cam]
            jacobian[:, :3, :3] = -np.eye(3)
            jacobian[:, :3, 3:] = so3.hat(centres(t_cam_world))
            jacobian[:, 3:, 3:] = -np.einsum(
                "cij,cjk->cik", so3.right_jacobian_inverse(error[:, 3:]), t_cam_world[:, :3, :3]
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
        """Form the normal equations at ``state`` and eliminate the points.

        ``weights`` are the IRLS weights, one per block in :meth:`squared_norms` order. The
        camera-side system is over the full vector (poses, intrinsics, affines); the
        :class:`SchurSystem` reduces it to the free coordinates.
        """
        kinds = self.kinds(state)
        n_free = self._real.shape[0] // 3
        hessian_cc = np.zeros((self._full_size, self._full_size))
        gradient_c = np.zeros(self._full_size)
        hessian_pp = np.zeros((n_free, 3, 3))
        gradient_p = np.zeros((n_free, 3))
        width = max(k.columns.shape[1] for k in kinds if k.point is not None)
        couplings = []
        splits = np.cumsum([k.residuals.shape[0] for k in kinds])[:-1]
        for kind, w in zip(kinds, np.split(weights, splits), strict=True):
            weighted_c = kind.camera_side * w[:, None, None]
            _add_camera(kind, weighted_c, hessian_cc, gradient_c)
            if kind.point is not None:
                _add_point(kind, kind.point, w, hessian_pp, gradient_p)
                couplings.append(_coupling(kind, kind.point, weighted_c, width))
        # An anchored point's two unused dimensions: inert, so their step is exactly zero.
        hessian_pp[self._anchored_slots, 1, 1] = 1.0
        hessian_pp[self._anchored_slots, 2, 2] = 1.0
        coupling, columns, slots = (np.concatenate(part) for part in zip(*couplings, strict=True))
        blocks = Blocks(
            hessian_cc=hessian_cc,
            gradient_c=gradient_c,
            hessian_pp=hessian_pp,
            gradient_p=gradient_p,
            coupling=coupling,
            columns=columns,
            slots=slots,
            real=self._real,
        )
        return SchurSystem(self.reduction(state), blocks, self._solver)

    def retract(self, state: State, delta: Array) -> State:
        """Apply a step in the free coordinates.

        ``delta`` holds the reduced camera-side step (:meth:`reduction`), then each free
        point's real coordinates (xyz, or ρ for an anchored point). Poses move by
        ``T exp(δ)`` (right perturbation); everything else adds.
        """
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


def _add_camera(kind: Kind, weighted_c: Array, hessian_cc: Array, gradient_c: Array) -> None:
    """Add a kind's camera-side blocks ``J_cᵀ W J_c`` and ``J_cᵀ W r`` in place."""
    np.add.at(
        hessian_cc,
        (kind.columns[:, :, None], kind.columns[:, None, :]),
        np.einsum("mki,mkj->mij", weighted_c, kind.camera_side),
    )
    np.add.at(gradient_c, kind.columns, np.einsum("mki,mk->mi", weighted_c, kind.residuals))


def _add_point(
    kind: Kind, point: Array, weights: Array, hessian_pp: Array, gradient_p: Array
) -> None:
    """Add a kind's point blocks ``J_pᵀ W J_p`` and ``J_pᵀ W r`` per free point, in place.

    ``point`` is the kind's point Jacobian, ``kind.point``, known to be present.
    """
    on = kind.slots >= 0
    weighted_p = point[on] * weights[on, None, None]
    np.add.at(hessian_pp, kind.slots[on], np.einsum("mki,mkj->mij", weighted_p, point[on]))
    np.add.at(gradient_p, kind.slots[on], np.einsum("mki,mk->mi", weighted_p, kind.residuals[on]))


def _coupling(
    kind: Kind, point: Array, weighted_c: Array, width: int
) -> tuple[Array, IndexArray, IndexArray]:
    """Return a kind's coupling blocks ``J_cᵀ W J_p``, columns and slots on free points.

    The columns are padded to ``width``, the widest kind's, so all kinds stack; padded
    columns carry zero blocks. ``point`` is ``kind.point``, known to be present.
    """
    on = kind.slots >= 0
    pad = width - kind.columns.shape[1]
    block = np.einsum("mki,mkj->mij", weighted_c[on], point[on])
    return (
        np.pad(block, ((0, 0), (0, pad), (0, 0))),
        np.pad(kind.columns[on], ((0, 0), (0, pad))),
        kind.slots[on],
    )


def centres(t_cam_world: Array) -> Array:
    """Return the camera centres ``-Rᵀ t`` of world-to-camera poses."""
    return np.asarray(
        -np.einsum("cji,cj->ci", t_cam_world[:, :3, :3], t_cam_world[:, :3, 3]), dtype=np.float64
    )
