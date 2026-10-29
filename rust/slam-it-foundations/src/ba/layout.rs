//! Bundle adjustment internals: the layout -- which observations count and which variables
//! move: degeneracy, unobserved variables, the gauge.

use nalgebra::Vector3;

use super::gauge::{self, Anchors, Plan};
use super::{Gauge, Options, Problem, at, invert, to_camera};
use crate::camera;
use crate::error::{ErrorKind, Result, fail};
use crate::se3;

/// The counts a layout reports, before the solve.
#[derive(Debug, Clone, Default)]
pub struct Counts {
    /// The gauge applied (`Auto` resolved).
    pub gauge: Option<Gauge>,
    /// The pose fixed to remove the rigid gauge.
    pub gauge_pose: Option<usize>,
    /// The first pose whose step was confined.
    pub confined_pose: Option<usize>,
    /// Gauge directions left in the problem.
    pub free_gauge: usize,
    /// Free points dropped as degenerate.
    pub dropped_points: usize,
    /// Observations left out (of dropped points, or behind their camera).
    pub dropped_observations: usize,
    /// Free poses held fixed because nothing observes them.
    pub unobserved_poses: usize,
    /// Free intrinsics groups held fixed because nothing observes them.
    pub unobserved_intrinsics: usize,
    /// Free depth affines held fixed because no depth prior observes them.
    pub unobserved_depth_affines: usize,
}

/// Which observations count and which variables move, decided once from the start.
#[derive(Debug, Clone, Default)]
pub struct Layout {
    /// The observations kept.
    pub obs: Vec<usize>,
    /// Positions in `obs` with a reprojection block: all but the anchors, whose error is
    /// identically zero.
    pub reprojection: Vec<usize>,
    /// Points parameterised as an inverse depth along their anchor's ray.
    pub anchored: Vec<bool>,
    /// Positions in `obs` of the kept observations with a depth prior.
    pub depth: Vec<usize>,
    /// Cameras with a pose prior that can move (a prior on a pose that cannot is constant).
    pub prior_cams: Vec<usize>,
    /// Depth affines that move.
    pub affine_free: Vec<bool>,
    /// Poses with all six directions free.
    pub pose_free: Vec<bool>,
    /// What pins the similarity, the poses the gauge fixed included.
    pub gauge_anchors: Anchors,
    /// The poses that remove the gauge: fixed ones (already out of `pose_free`), and
    /// confined ones in order with how many directions each removes.
    pub gauge_plan: Plan,
    /// Intrinsics parameters that move, per group and parameter (K x P, row-major).
    pub intrinsics_free: Vec<bool>,
    /// Each point's index among the free points.
    pub point_slot: Vec<Option<usize>>,
    /// The number of free points.
    pub free_points: usize,
    /// The layout's counts.
    pub counts: Counts,
}

/// A camera's world centre, from its camera-to-world pose.
///
/// # Arguments
///
/// * `t_world_cam` - The pose.
///
/// # Returns
///
/// Its translation.
fn centre_of(t_world_cam: &nalgebra::Matrix4<f64>) -> Vector3<f64> {
    se3::translation(t_world_cam)
}

/// Each point's largest angle between its first kept ray and any other (radians).
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `obs` - The kept observations.
///
/// # Returns
///
/// The parallax per point.
fn ray_parallax(problem: &Problem, obs: &[usize]) -> Vec<f64> {
    let n_points = problem.points.len();
    let mut reference: Vec<Option<Vector3<f64>>> = vec![None; n_points];
    let mut parallax = vec![0.0_f64; n_points];
    for &m in obs {
        let p = at(problem.pt_idx[m]);
        let camera = centre_of(&problem.poses[at(problem.cam_idx[m])]);
        let ray = (camera - problem.points[p]).normalize();
        match reference[p] {
            None => reference[p] = Some(ray),
            Some(first) => {
                let angle = ray.cross(&first).norm().atan2(ray.dot(&first));
                parallax[p] = parallax[p].max(angle);
            }
        }
    }
    parallax
}

/// The observed poses sorted by what pins them.
#[derive(Debug, Default)]
struct PinnedPoses {
    /// Fixed observed poses.
    fixed: Vec<usize>,
    /// Free observed poses with a translation prior.
    translation: Vec<usize>,
    /// Free observed poses with a rotation prior.
    rotation: Vec<usize>,
    /// Free observed poses with both.
    full: Vec<usize>,
    /// Every observed pose's centre.
    centres: Vec<Vector3<f64>>,
}

/// Sorts the observed poses by what pins them.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `observed` - Which poses are observed.
///
/// # Returns
///
/// The poses.
fn pinned_poses(problem: &Problem, observed: &[bool]) -> PinnedPoses {
    let mut out = PinnedPoses::default();
    for (c, pose) in problem.poses.iter().enumerate() {
        if !observed[c] {
            continue;
        }
        out.centres.push(centre_of(pose));
        if problem.fixed_poses[c] {
            out.fixed.push(c);
            continue;
        }
        let by_translation = problem.pose_prior_weights[c][0] > 0.0;
        let by_rotation = problem.pose_prior_weights[c][1] > 0.0;
        if by_translation {
            out.translation.push(c);
        }
        if by_rotation {
            out.rotation.push(c);
        }
        if by_translation && by_rotation {
            out.full.push(c);
        }
    }
    out
}

/// What a gauge choice counts as pinned.
///
/// `Auto` everything (fixed observed poses, priors on observed poses, fixed observed points,
/// metric depth), `Similarity` the fixed observed poses, `Scale` its reference pose. Priors on
/// unobserved poses pin nothing the scene can feel.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `observed` - Which poses are observed.
/// * `metric_depth` - Whether a metric depth prior (a fixed affine) is present.
/// * `fixed_points_used` - Which fixed points are observed and kept.
/// * `gauge` - The requested gauge.
///
/// # Returns
///
/// The anchors.
///
/// # Errors
///
/// `InvalidInput` for a scale gauge with nothing to measure from.
fn pinned(
    problem: &Problem,
    observed: &[bool],
    metric_depth: bool,
    fixed_points_used: &[bool],
    gauge: Gauge,
) -> Result<Anchors> {
    let poses = pinned_poses(problem, observed);
    let points: Vec<Vector3<f64>> = problem
        .points
        .iter()
        .zip(fixed_points_used)
        .filter(|(_, used)| **used)
        .map(|(p, _)| *p)
        .collect();
    // The frame's positions: observed centres, then points.
    let scene: Vec<Vector3<f64>> = poses.centres.iter().chain(&points).copied().collect();
    let mut out = Anchors {
        frame: gauge::frame_of(&scene),
        ..Anchors::default()
    };
    if gauge == Gauge::Scale {
        let Some(&reference) = poses.fixed.first().or(poses.full.first()) else {
            return fail(
                ErrorKind::InvalidInput,
                "gauge: 'scale' needs a fixed pose or a full pose prior to measure scale from",
            );
        };
        out.centre_cams = vec![reference];
        out.orientation_cams = vec![reference];
        return Ok(out);
    }
    out.centre_cams.clone_from(&poses.fixed);
    out.orientation_cams.clone_from(&poses.fixed);
    if gauge == Gauge::Similarity {
        return Ok(out);
    }
    out.centre_cams.extend(&poses.translation);
    out.orientation_cams.extend(&poses.rotation);
    out.points = points;
    out.metric = metric_depth;
    Ok(out)
}

/// The gauge as applied: its name, the poses that remove it, what pins the rest.
struct GaugeChoice {
    /// The gauge applied: the request, or for `Auto` what it found.
    name: Gauge,
    /// Poses with all six directions free once the gauge is removed.
    pose_free: Vec<bool>,
    /// What pins the similarity, the plan's fixed poses included.
    anchors: Anchors,
    /// The poses that remove the gauge.
    plan: Plan,
}

/// Resolves the gauge choice and plans its removal (see the gauge module).
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `observed` - Which poses are observed.
/// * `metric_depth` - Whether a metric depth prior is present.
/// * `fixed_points_used` - Which fixed points are observed and kept.
/// * `gauge` - The requested gauge.
///
/// # Returns
///
/// The choice.
///
/// # Errors
///
/// `InvalidInput` for a scale gauge with nothing to measure from; `Degenerate` if the
/// gauge's SVD fails.
fn fix_gauge(
    problem: &Problem,
    observed: &[bool],
    metric_depth: bool,
    fixed_points_used: &[bool],
    gauge: Gauge,
) -> Result<GaugeChoice> {
    let mut anchors = pinned(problem, observed, metric_depth, fixed_points_used, gauge)?;
    let mut pose_free: Vec<bool> = (0..problem.poses.len())
        .map(|c| !problem.fixed_poses[c] && observed[c])
        .collect();
    let candidates: Vec<usize> = (0..problem.poses.len()).filter(|&c| pose_free[c]).collect();
    let any_fixed = (0..problem.poses.len()).any(|c| observed[c] && problem.fixed_poses[c]);
    let centres: Vec<Vector3<f64>> = problem.poses.iter().map(centre_of).collect();
    let mut name = gauge;
    let mut plan = Plan::default();
    if gauge == Gauge::FirstPose {
        if let (false, Some(&first)) = (any_fixed, candidates.first()) {
            plan.fixed.push(first);
        }
    } else if gauge != Gauge::None {
        if gauge == Gauge::Auto {
            name = gauge::classify(&gauge::remaining(&anchors, &centres)?);
        }
        plan = gauge::plan(&anchors, &centres, &candidates)?;
    }
    for &c in &plan.fixed {
        anchors.centre_cams.push(c);
        anchors.orientation_cams.push(c);
        pose_free[c] = false;
    }
    for &c in &plan.confined {
        pose_free[c] = false;
    }
    if matches!(gauge, Gauge::None | Gauge::FirstPose) {
        // Fixed recipes: whatever they leave of the gauge stays in the problem, and is
        // counted.
        plan.free = gauge::remaining(&anchors, &centres)?.ncols();
    }
    Ok(GaugeChoice {
        name,
        pose_free,
        anchors,
        plan,
    })
}

/// The weighted observations whose point is in front of its camera at the start.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `counts` - Incremented for each observation behind its camera.
///
/// # Returns
///
/// The kept observations.
fn in_front(problem: &Problem, counts: &mut Counts) -> Vec<usize> {
    let mut kept = Vec::new();
    for (m, &w) in problem.weights.iter().enumerate() {
        if w <= 0.0 {
            continue;
        }
        let t_world_cam = &problem.poses[at(problem.cam_idx[m])];
        let point = problem.points[at(problem.pt_idx[m])];
        // The bundle's own depth (its state's inverted pose, `R p + t`), so that the two agree
        // at `camera::MIN_DEPTH`: a separate formula, `R_world_cam`'s column 2 dotted with the
        // offset from the centre, rounded differently, and kept an observation 1e-9 m in front
        // of its camera that the projection then refused, an infinite starting cost (advio-01,
        // B1).
        let depth = to_camera(&invert(t_world_cam), &point).z;
        if depth > camera::MIN_DEPTH {
            kept.push(m);
        } else {
            counts.dropped_observations += 1;
        }
    }
    kept
}

/// Points that cannot be placed: free points with fewer than two views or too little
/// parallax, and anchored points whose anchor was dropped or cannot be undistorted.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `kept` - The kept observations.
/// * `min_parallax_deg` - The parallax threshold (degrees).
/// * `views` - Each point's number of kept observations.
///
/// # Returns
///
/// Which points are degenerate.
fn degenerate_points(
    problem: &Problem,
    kept: &[usize],
    min_parallax_deg: f64,
    views: &[usize],
) -> Vec<bool> {
    let parallax = ray_parallax(problem, kept);
    let min_parallax = min_parallax_deg.to_radians();
    let mut is_kept = vec![false; problem.observations.len()];
    for &m in kept {
        is_kept[m] = true;
    }
    (0..views.len())
        .map(|p| {
            let seen = views[p] > 0;
            let mut degenerate =
                !problem.fixed_points[p] && seen && (views[p] < 2 || parallax[p] < min_parallax);
            if let Ok(anchor) = usize::try_from(problem.point_anchor[p]) {
                // An anchored point whose anchor was dropped has no ray to live on.
                degenerate = degenerate || (seen && !is_kept[anchor]);
                // So has one whose anchor pixel the starting intrinsics cannot undistort.
                if seen && !degenerate {
                    let cam = at(problem.cam_idx[anchor]);
                    let group = at(problem.intrinsics_idx[cam]);
                    degenerate = camera::unproject(
                        problem.model,
                        problem.intrinsics.group(group),
                        &problem.observations[anchor],
                    )
                    .is_err();
                }
            }
            degenerate
        })
        .collect()
}

/// Attaches the depth priors and frees the observed affines.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `layout` - The layout to update (its kept observations already chosen).
///
/// # Returns
///
/// Whether a metric depth prior (a fixed affine) is present.
fn attach_priors(problem: &Problem, layout: &mut Layout) -> bool {
    let n_cams = problem.poses.len();
    let mut affine_observed = vec![false; n_cams];
    let mut metric_depth = false;
    for (i, &m) in layout.obs.iter().enumerate() {
        if problem.depth_prior[m].is_nan() || problem.depth_prior_weights[m] <= 0.0 {
            continue;
        }
        let c = at(problem.cam_idx[m]);
        layout.depth.push(i);
        affine_observed[c] = true;
        metric_depth = metric_depth || problem.fixed_depth_affine[c];
    }
    layout.affine_free = (0..n_cams)
        .map(|c| !problem.fixed_depth_affine[c] && affine_observed[c])
        .collect();
    layout.counts.unobserved_depth_affines = (0..n_cams)
        .filter(|&c| !problem.fixed_depth_affine[c] && !affine_observed[c])
        .count();
    metric_depth
}

/// Gives each used free point a slot, marks the anchored points, and lists the observations
/// that get a reprojection block (all but the anchors).
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `degenerate` - Which points were dropped.
/// * `views` - Each point's number of kept observations.
/// * `layout` - The layout to update (its kept observations already chosen).
fn assign_points(problem: &Problem, degenerate: &[bool], views: &[usize], layout: &mut Layout) {
    let n_points = views.len();
    layout.point_slot = vec![None; n_points];
    layout.anchored = vec![false; n_points];
    let mut is_anchor = vec![false; problem.observations.len()];
    for p in 0..n_points {
        let used = !degenerate[p] && views[p] > 0;
        if used && !problem.fixed_points[p] {
            layout.point_slot[p] = Some(layout.free_points);
            layout.free_points += 1;
        }
        if let (true, Ok(anchor)) = (used, usize::try_from(problem.point_anchor[p])) {
            layout.anchored[p] = true;
            is_anchor[anchor] = true;
        }
    }
    layout.reprojection = (0..layout.obs.len())
        .filter(|&i| !is_anchor[layout.obs[i]])
        .collect();
}

/// Keeps the observations of points that are not degenerate, and marks what they see.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `kept` - The observations in front of their camera.
/// * `degenerate` - Which points were dropped.
/// * `layout` - The layout to update: its observations and dropped-observation count.
///
/// # Returns
///
/// Which poses and which intrinsics groups are observed.
fn observe(
    problem: &Problem,
    kept: &[usize],
    degenerate: &[bool],
    layout: &mut Layout,
) -> (Vec<bool>, Vec<bool>) {
    let mut observed = vec![false; problem.poses.len()];
    let mut group_observed = vec![false; problem.intrinsics.groups()];
    for &m in kept {
        if degenerate[at(problem.pt_idx[m])] {
            layout.counts.dropped_observations += 1;
            continue;
        }
        let c = at(problem.cam_idx[m]);
        layout.obs.push(m);
        observed[c] = true;
        group_observed[at(problem.intrinsics_idx[c])] = true;
    }
    (observed, group_observed)
}

/// Frees the observed intrinsics parameters, and counts the unobserved free groups.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `group_observed` - Which intrinsics groups are observed.
/// * `layout` - The layout to update.
fn free_intrinsics(problem: &Problem, group_observed: &[bool], layout: &mut Layout) {
    let n_params = problem.intrinsics.params;
    layout.intrinsics_free = problem
        .fixed_intrinsics
        .iter()
        .enumerate()
        .map(|(k, &fixed)| !fixed && group_observed[k / n_params])
        .collect();
    layout.counts.unobserved_intrinsics = (0..problem.intrinsics.groups())
        .filter(|&g| {
            let any_free = problem.fixed_intrinsics[g * n_params..(g + 1) * n_params]
                .iter()
                .any(|&fixed| !fixed);
            any_free && !group_observed[g]
        })
        .count();
}

/// Attaches the pose priors on poses that move.
///
/// A prior on a pose that never moves (fixed, unobserved, or fixed by the gauge) is a
/// constant: it informs nothing and is left out, like the views of a dropped point.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `layout` - The layout to update (its gauge already applied).
fn attach_pose_priors(problem: &Problem, layout: &mut Layout) {
    layout.prior_cams = (0..problem.poses.len())
        .filter(|&c| {
            let moves = layout.pose_free[c] || layout.gauge_plan.confined.contains(&c);
            moves && problem.pose_prior_weights[c].iter().any(|&w| w > 0.0)
        })
        .collect();
}

/// Drops degenerate observations, holds unobserved variables fixed, and fixes the gauge.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `options` - The settings.
///
/// # Returns
///
/// The layout.
///
/// # Errors
///
/// `InvalidInput` for a gauge the problem cannot support.
pub fn make_layout(problem: &Problem, options: &Options) -> Result<Layout> {
    let n_points = problem.points.len();
    let mut layout = Layout::default();
    let kept = in_front(problem, &mut layout.counts);
    let mut views = vec![0_usize; n_points];
    for &m in &kept {
        views[at(problem.pt_idx[m])] += 1;
    }
    let degenerate = degenerate_points(problem, &kept, options.min_parallax_deg, &views);
    layout.counts.dropped_points = degenerate.iter().filter(|&&d| d).count();
    let (observed, group_observed) = observe(problem, &kept, &degenerate, &mut layout);
    layout.counts.unobserved_poses = (0..problem.poses.len())
        .filter(|&c| !problem.fixed_poses[c] && !observed[c])
        .count();
    free_intrinsics(problem, &group_observed, &mut layout);
    let metric_depth = attach_priors(problem, &mut layout);
    let fixed_points_used: Vec<bool> = (0..n_points)
        .map(|p| problem.fixed_points[p] && !degenerate[p] && views[p] > 0)
        .collect();
    let gauge = fix_gauge(
        problem,
        &observed,
        metric_depth,
        &fixed_points_used,
        options.gauge,
    )?;
    layout.counts.gauge = Some(gauge.name);
    layout.counts.gauge_pose = gauge.plan.fixed.first().copied();
    layout.counts.confined_pose = gauge.plan.confined.first().copied();
    layout.counts.free_gauge = gauge.plan.free;
    layout.pose_free = gauge.pose_free;
    layout.gauge_anchors = gauge.anchors;
    layout.gauge_plan = gauge.plan;
    attach_pose_priors(problem, &mut layout);
    assign_points(problem, &degenerate, &views, &mut layout);
    Ok(layout)
}
