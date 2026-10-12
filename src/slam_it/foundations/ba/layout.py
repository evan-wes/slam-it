"""What a bundle adjustment optimises: kept blocks, free variables and the gauge.

The gauge is the scene's free similarity (see :mod:`slam_it.foundations.ba.gauge`). Fixed
poses, fixed points, pose priors and metric depth pin parts of it; the gauge choice removes
the rest:

- ``"auto"`` removes exactly what the fixed variables and priors leave free, and no more
  (Triggs et al. 1999, section 9). It reports what that was: ``"none"``, ``"first_pose"``
  (the rigid motions), ``"scale"``, ``"similarity"``, or ``"partial"`` (any other part, such
  as the rotation about the line through two fixed points);
- ``"similarity"`` counts only fixed poses as anchors: it fixes the first observed pose if
  none is fixed, and confines the farthest from it to keep their baseline, unless two fixed
  poses apart pin the scale already;
- ``"scale"`` removes only the scale about a reference (the first fixed pose, else the first
  pose with a full prior), confining the pose farthest from it;
- ``"first_pose"`` fixes the first observed pose if none is fixed, nothing else;
- ``"none"`` fixes nothing.

Kept blocks: observations of dropped points (too few views, too little parallax, or an
anchor that cannot be placed) are left out, and the counts reported.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from slam_it.foundations import camera, se3
from slam_it.foundations.arrays import group_members
from slam_it.foundations.ba.gauge import (
    NO_PLAN,
    Anchors,
    Frame,
    Plan,
    classify,
    plan,
    remaining,
)
from slam_it.foundations.ba.problem import Array, BAOptions, BAProblem, IndexArray, Mask
from slam_it.foundations.errors import InvalidInputError

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
    """Cameras with a pose prior that can move (a prior on a pose that cannot is constant)."""
    pose_free: Mask
    """Poses with all six directions free (not fixed, observed, not the gauge poses)."""
    gauge_anchors: Anchors
    """What pins the similarity, the poses the gauge fixed included."""
    gauge_plan: Plan
    """The poses that remove the gauge: fixed ones (out of ``pose_free``), and confined ones
    in order with how many directions each removes."""
    intrinsics_free: Mask
    """Intrinsics parameters that move, shape (K, P)."""
    affine_free: Mask
    """Depth affines that move, shape (C,): free and observed by a depth prior."""
    point_slot: IndexArray
    """Each point's index among the free points, or -1."""
    gauge: str
    """The gauge applied (``auto`` resolved to what it found)."""
    summary: dict[str, int]
    """The layout's counts, by :data:`~slam_it.foundations.ba.problem.COUNT_NAMES` name."""


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


def _anchors(
    problem: BAProblem, observed: Mask, fixed_points_used: Mask, metric_depth: bool, gauge: str
) -> Anchors:
    """Return what a gauge choice counts as pinned.

    ``auto`` counts everything: fixed observed poses, priors on observed poses (translation
    and rotation separately), fixed observed points and metric depth. ``similarity`` counts
    only the fixed observed poses; ``scale`` only its reference pose. Priors on unobserved
    poses pin nothing the scene can feel, so they never count.
    """
    weights = problem.pose_prior_weights
    fixed = np.flatnonzero(problem.fixed_poses & observed)
    prior = observed & ~problem.fixed_poses
    points = problem.points[fixed_points_used]
    # The rows' frame: centred on the scene and scaled by its spread (see gauge.Frame).
    frame = Frame.of(np.concatenate([problem.poses[observed, :3, 3], points]))
    none = np.zeros((0, 3))
    if gauge == "scale":
        full = np.flatnonzero(prior & (weights[:, 0] > 0.0) & (weights[:, 1] > 0.0))
        reference = np.concatenate([fixed, full])[:1]
        if reference.size == 0:
            raise InvalidInputError(
                "gauge: 'scale' needs a fixed pose or a full pose prior to measure scale from"
            )
        return Anchors(none, reference, reference, metric=False, frame=frame)
    if gauge == "similarity":
        return Anchors(none, fixed, fixed, metric=False, frame=frame)
    return Anchors(
        points=points,
        centre_cams=np.concatenate([fixed, np.flatnonzero(prior & (weights[:, 0] > 0.0))]),
        orientation_cams=np.concatenate([fixed, np.flatnonzero(prior & (weights[:, 1] > 0.0))]),
        metric=metric_depth,
        frame=frame,
    )


@dataclass(frozen=True)
class _Gauge:
    """The gauge as applied: its name, the poses that remove it, what pins the rest."""

    name: str
    """The gauge applied: the request, or for ``auto`` what it found."""
    pose_free: Mask
    """Poses with all six directions free once the gauge is removed."""
    anchors: Anchors
    """What pins the similarity, the plan's fixed poses included."""
    plan: Plan
    """The poses that remove the gauge."""


def _fix_gauge(
    problem: BAProblem, observed: Mask, fixed_points_used: Mask, metric_depth: bool, gauge: str
) -> _Gauge:
    """Resolve the gauge choice and plan its removal (:mod:`slam_it.foundations.ba.gauge`)."""
    pose_free = ~problem.fixed_poses & observed
    candidates = np.flatnonzero(pose_free)
    anchors = _anchors(problem, observed, fixed_points_used, metric_depth, gauge)
    centres = problem.poses[:, :3, 3]
    if gauge in {"none", "first_pose"}:
        # Fixed recipes: whatever they leave of the gauge stays in the problem, and is counted.
        fixed = np.zeros(0, dtype=np.int64)
        if gauge == "first_pose" and not np.any(problem.fixed_poses & observed) and candidates.size:
            fixed = candidates[:1]
            anchors = anchors.with_fixed(int(fixed[0]))
        free = int(remaining(anchors, centres).shape[1])
        return _Gauge(
            gauge,
            pose_free & ~np.isin(np.arange(pose_free.size), fixed),
            anchors,
            replace(NO_PLAN, fixed=fixed, free=free),
        )
    name = classify(remaining(anchors, centres)) if gauge == "auto" else gauge
    chosen = plan(anchors, centres, candidates)
    for c in chosen.fixed:
        anchors = anchors.with_fixed(int(c))
    pose_free = pose_free.copy()
    pose_free[np.concatenate([chosen.fixed, chosen.confined])] = False
    return _Gauge(name, pose_free, anchors, chosen)


def _unplaceable_anchors(problem: BAProblem, candidates: IndexArray) -> IndexArray:
    """Return the anchored points among ``candidates`` whose anchor pixel has no ray."""
    anchor_obs = problem.point_anchor[candidates]
    group = problem.intrinsics_idx[problem.cam_idx[anchor_obs]]
    bad = np.zeros(candidates.size, dtype=bool)
    for g, sel in enumerate(group_members(group, problem.intrinsics.shape[0])):
        if sel.size == 0:
            continue
        rays = camera.unproject_or_nan(
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
    used = ~degenerate & (np.bincount(problem.pt_idx[obs], minlength=n_points) > 0)
    gauge = _fix_gauge(
        problem,
        observed,
        problem.fixed_points & used,
        metric_depth=bool(np.any(problem.fixed_depth_affine[cam[depth]])),
        gauge=options.gauge,
    )
    point_free = ~problem.fixed_points & used
    anchored = (problem.point_anchor >= 0) & used
    reprojection = np.flatnonzero(~np.isin(obs, problem.point_anchor[anchored]))
    point_slot = np.full(n_points, -1, dtype=np.int64)
    point_slot[point_free] = np.arange(np.count_nonzero(point_free))
    summary = {
        "gauge_pose": int(gauge.plan.fixed[0]) if gauge.plan.fixed.size else -1,
        "confined_pose": int(gauge.plan.confined[0]) if gauge.plan.confined.size else -1,
        "free_gauge": gauge.plan.free,
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
        # A prior on a pose that never moves (fixed, unobserved, or fixed by the gauge) is a
        # constant: it informs nothing and is left out, like the views of a dropped point.
        prior_cams=np.flatnonzero(
            np.any(problem.pose_prior_weights > 0.0, axis=1)
            & (gauge.pose_free | np.isin(np.arange(n_cams), gauge.plan.confined))
        ),
        pose_free=gauge.pose_free,
        gauge_anchors=gauge.anchors,
        gauge_plan=gauge.plan,
        intrinsics_free=~problem.fixed_intrinsics & intrinsics_observed[:, None],
        affine_free=~problem.fixed_depth_affine & affine_observed,
        point_slot=point_slot,
        gauge=gauge.name,
        summary=summary,
    )
