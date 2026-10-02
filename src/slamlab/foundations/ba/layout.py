"""What a bundle adjustment optimises: kept blocks, free variables and the gauge."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from slamlab.foundations import camera, se3
from slamlab.foundations.ba.problem import Array, BAOptions, BAProblem, IndexArray, Mask

# --- setup: what is optimised ---------------------------------------------------------------


@dataclass(frozen=True)
class Layout:
    """Which blocks count and which variables move, decided once from the start."""

    obs: IndexArray
    """Indices of the observations kept."""
    reprojection: IndexArray
    """Positions in ``obs`` with a reprojection block: all but the anchors, whose error is
    identically zero (they define their point's ray)."""
    anchored: Mask
    """Points parameterised as an inverse depth along their anchor's ray."""
    depth: IndexArray
    """Positions in ``obs`` of the kept observations with a depth prior."""
    prior_cams: IndexArray
    """Cameras with a pose prior."""
    pose_free: Mask
    """Poses with all six directions free (not fixed, observed, not the gauge poses)."""
    scale_pose: int
    scale_reference: int
    intrinsics_free: Mask
    """Intrinsics parameters that move, shape (K, P)."""
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
    # An anchored point whose anchor was dropped has no ray to live on.
    kept = np.zeros(problem.observations.shape[0], dtype=bool)
    kept[obs] = True
    anchors = problem.point_anchor
    degenerate |= (anchors >= 0) & ~kept[np.maximum(anchors, 0)] & (views > 0)
    keep = ~degenerate[pt]
    dropped = int(np.count_nonzero(~in_front)) + int(np.count_nonzero(~keep))
    return obs[keep], degenerate, dropped


def make_layout(problem: BAProblem, options: BAOptions) -> Layout:
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
    used = ~degenerate & (np.bincount(problem.pt_idx[obs], minlength=n_points) > 0)
    point_free = ~problem.fixed_points & used
    anchored = (problem.point_anchor >= 0) & used
    reprojection = np.flatnonzero(~np.isin(obs, problem.point_anchor[anchored]))
    point_slot = np.full(n_points, -1, dtype=np.int64)
    point_slot[point_free] = np.arange(np.count_nonzero(point_free))
    summary = {
        "gauge_pose": gauge_pose,
        "scale_pose": scale_pose,
        "dropped_points": int(np.count_nonzero(degenerate)),
        "dropped_observations": dropped,
        "unobserved_poses": int(np.count_nonzero(~problem.fixed_poses & ~observed)),
        "unobserved_intrinsics": int(
            np.count_nonzero(np.any(~problem.fixed_intrinsics, axis=1) & ~intrinsics_observed)
        ),
        "unobserved_depth_affines": int(
            np.count_nonzero(~problem.fixed_depth_affine & ~affine_observed)
        ),
    }
    return Layout(
        obs=obs,
        reprojection=reprojection,
        anchored=anchored,
        depth=depth,
        prior_cams=prior_cams,
        pose_free=pose_free,
        scale_pose=scale_pose,
        scale_reference=scale_reference,
        intrinsics_free=~problem.fixed_intrinsics & intrinsics_observed[:, None],
        affine_free=~problem.fixed_depth_affine & affine_observed,
        point_slot=point_slot,
        gauge=gauge,
        summary=summary,
    )
