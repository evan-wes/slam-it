"""What a bundle adjustment optimises: kept blocks, free variables and the gauge.

The gauge is the scene's free similarity: rigid motion, and scale unless metric data pins it.
Fixed poses (two or more apart), full pose priors and metric depth priors anchor parts of
it. The gauge applied removes what is left, and no more (Triggs et al. 1999, section 9):

- ``"first_pose"`` fixes one observed pose (the first fixed one, else the first free one);
- ``"scale"`` confines the free pose farthest from an anchored one to steps that keep their
  baseline's length, through a Householder basis of the other five directions;
- ``"similarity"`` does both;
- ``"none"`` fixes nothing;
- ``"auto"`` resolves to the minimal one of these for what is anchored.

Kept blocks: observations of dropped points (too few views, too little parallax, or an
anchor that cannot be placed) are left out, and the counts reported.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from slamlab.foundations import camera, se3
from slamlab.foundations.ba.problem import Array, BAOptions, BAProblem, IndexArray, Mask
from slamlab.foundations.errors import InvalidInputError, NotConvergedError

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
    first = np.full(points.shape[0], cam.shape[0], dtype=np.int64)
    np.minimum.at(first, pt, np.arange(cam.shape[0]))  # each point's lowest observation
    reference = rays[first[pt]]
    angle = np.arctan2(
        np.linalg.norm(np.cross(rays, reference), axis=-1), np.sum(rays * reference, axis=-1)
    )
    parallax = np.zeros(points.shape[0])
    np.maximum.at(parallax, pt, angle)
    return parallax


ANCHOR_TOLERANCE = 1e-9
"""Known positions closer than this (relative to their magnitude) count as one."""


@dataclass(frozen=True)
class _Anchoring:
    """What the fixed variables and priors already pin, before any gauge is applied."""

    rigid: bool
    """Rotation and translation: a fixed pose, a full pose prior, or three non-collinear
    known positions."""
    scale: bool
    """Scale: a metric depth prior, or two distinct known positions."""
    reference: int
    """The camera the rigid motion is pinned to (a fixed or fully prior-constrained pose),
    or -1."""


def _spread(positions: Array) -> tuple[bool, bool]:
    """Return whether known world positions are (at least two) distinct, and non-collinear."""
    if positions.shape[0] < 2:
        return False, False
    singular = np.linalg.svd(positions - positions.mean(axis=0), compute_uv=False)
    tolerance = ANCHOR_TOLERANCE * (1.0 + float(np.abs(positions).max()))
    return bool(singular[0] > tolerance), bool(singular[1] > tolerance)


def _anchoring(
    problem: BAProblem, observed: Mask, fixed_points_used: Mask, metric_depth: bool
) -> _Anchoring:
    """Count what fixed poses, fixed points, pose priors and metric depth already pin.

    Known world positions are fixed observed camera centres, translation-prior centres and
    fixed observed points. A translation-only prior on one camera pins neither rotation nor
    scale; it is not counted as an anchor.
    """
    fixed_cams = np.flatnonzero(problem.fixed_poses & observed)
    weights = problem.pose_prior_weights
    translation = np.flatnonzero(weights[:, 0] > 0.0)
    full = np.flatnonzero((weights[:, 0] > 0.0) & (weights[:, 1] > 0.0))
    positions = np.concatenate(
        [
            problem.poses[fixed_cams, :3, 3],
            problem.pose_prior[translation, :3, 3],
            problem.points[fixed_points_used],
        ]
    )
    distinct, non_collinear = _spread(positions)
    if fixed_cams.size:
        reference = int(fixed_cams[0])
    elif full.size:
        reference = int(full[0])
    else:
        reference = -1
    return _Anchoring(
        rigid=reference >= 0 or non_collinear, scale=metric_depth or distinct, reference=reference
    )


def _resolve_gauge(gauge: str, anchoring: _Anchoring) -> str:
    """Resolve ``"auto"`` to the minimal gauge the anchoring leaves; others pass through.

    Minimal, as Triggs et al. (1999, section 9) require: fixing more than the free directions
    biases the solution, fixing fewer leaves the system singular.
    """
    if gauge != "auto":
        return gauge
    if anchoring.rigid:
        return "none" if anchoring.scale else "scale"
    return "first_pose" if anchoring.scale else "similarity"


def _confine_scale(problem: BAProblem, pose_free: Mask, reference: int) -> tuple[Mask, int, int]:
    """Confine the free pose farthest from ``reference`` to keep its baseline (scale gauge)."""
    centres = problem.poses[:, :3, 3]
    candidates = np.flatnonzero(pose_free)
    tolerance = 1e-12 * (1.0 + np.linalg.norm(centres[reference]))
    if candidates.size == 0:
        return pose_free, -1, -1
    distance = np.linalg.norm(centres[candidates] - centres[reference], axis=-1)
    if distance.max() <= tolerance:
        return pose_free, -1, -1
    scale_pose = int(candidates[np.argmax(distance)])
    pose_free = pose_free.copy()
    pose_free[scale_pose] = False
    return pose_free, scale_pose, reference


def _gauge(
    problem: BAProblem, pose_free: Mask, observed: Mask, gauge: str, anchoring: _Anchoring
) -> tuple[Mask, int, int, int]:
    """Fix the gauge; return the free poses, the gauge pose, the scale pose and its reference."""
    pose_free = pose_free.copy()
    if gauge == "none":
        return pose_free, -1, -1, -1
    if gauge == "scale":
        if anchoring.reference < 0:
            raise InvalidInputError(
                "gauge: 'scale' needs a fixed pose or a full pose prior to measure scale from"
            )
        pose_free, scale_pose, reference = _confine_scale(problem, pose_free, anchoring.reference)
        return pose_free, -1, scale_pose, reference
    gauge_pose = -1
    fixed_observed = np.flatnonzero(problem.fixed_poses & observed)
    if fixed_observed.size == 0:
        anchors = np.flatnonzero(~problem.fixed_poses & observed)
        if anchors.size == 0:
            return pose_free, -1, -1, -1
        gauge_pose = int(anchors[0])
        pose_free[gauge_pose] = False
        fixed_observed = np.array([gauge_pose])
    if gauge == "first_pose":
        return pose_free, gauge_pose, -1, -1
    centres = problem.poses[:, :3, 3]
    tolerance = 1e-12 * (1.0 + np.linalg.norm(centres[fixed_observed[0]]))
    spread = np.linalg.norm(centres[fixed_observed] - centres[fixed_observed[0]], axis=-1)
    if np.any(spread > tolerance):
        return pose_free, gauge_pose, -1, -1  # two fixed poses apart already pin the scale
    pose_free, scale_pose, reference = _confine_scale(problem, pose_free, int(fixed_observed[0]))
    return pose_free, gauge_pose, scale_pose, reference


def bearings_or_nan(model: str, intrinsics: Array, pixels: Array) -> Array:
    """Unproject pixels to bearings; rows whose pixel cannot be undistorted are NaN.

    A pixel beyond the region where the lens model is invertible has no ray. For an anchor
    pixel, that makes its point invalid: at the start it is dropped, during a solve the step
    that caused it is rejected (its blocks' cost becomes infinite), as in the C++ core.
    """
    try:
        return camera.unproject(model, intrinsics, pixels)
    except NotConvergedError:
        out = np.full((pixels.shape[0], 3), np.nan)
        for i, pixel in enumerate(pixels):
            try:
                out[i] = camera.unproject(model, intrinsics, pixel)
            except NotConvergedError:
                continue
        return out


def _unplaceable_anchors(problem: BAProblem, candidates: IndexArray) -> IndexArray:
    """Return the anchored points among ``candidates`` whose anchor pixel has no ray."""
    anchor_obs = problem.point_anchor[candidates]
    group = problem.intrinsics_idx[problem.cam_idx[anchor_obs]]
    bad = np.zeros(candidates.size, dtype=bool)
    for g in np.unique(group):
        sel = group == g
        rays = bearings_or_nan(
            problem.camera_model, problem.intrinsics[g], problem.observations[anchor_obs[sel]]
        )
        bad[sel] = np.isnan(rays[:, 0])
    return candidates[bad]


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
    # So has one whose anchor pixel cannot be undistorted with the starting intrinsics.
    degenerate[
        _unplaceable_anchors(problem, np.flatnonzero((anchors >= 0) & ~degenerate & (views > 0)))
    ] = True
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
    used = ~degenerate & (np.bincount(problem.pt_idx[obs], minlength=n_points) > 0)
    anchoring = _anchoring(
        problem,
        observed,
        problem.fixed_points & used,
        metric_depth=bool(np.any(problem.fixed_depth_affine[cam[depth]])),
    )
    gauge = _resolve_gauge(options.gauge, anchoring)
    pose_free, gauge_pose, scale_pose, scale_reference = _gauge(
        problem, ~problem.fixed_poses & observed, observed, gauge, anchoring
    )
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
