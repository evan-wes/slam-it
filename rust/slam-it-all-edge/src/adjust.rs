//! Bundle adjustment over the map's keyframes: the Rust core's mirror of
//! `slam_it_all.edge.adjust`, whose docstring states the problem.
//!
//! The free keyframes, the points they observe and every keyframe observing those points go
//! into one `ba::Problem`, observations weighted by their levels' information and given an
//! inverse-depth prior where they have depth. Two solves bracket an outlier pass (a point left
//! with fewer than two inlier views restarts the second from where it began); the
//! observations still failing the chi-square test are forgotten, and the surviving points
//! refreshed.

use std::collections::BTreeSet;

use nalgebra::{Matrix4, Vector2, Vector3};
use slam_it_all_foundations::ba::{self, Intrinsics};
use slam_it_all_foundations::error::{ErrorKind, Result, fail};
use slam_it_all_foundations::{nls, se3};

use crate::frames::{Camera, check_setup};
use crate::map::{self, Map};
use crate::options::Options;
use crate::residuals::{chi2, sigma2, threshold};

/// The BA problem built from the map, and where its rows came from.
struct Adjustment {
    /// The problem.
    problem: ba::Problem,
    /// Each observation's keypoint row, ascending.
    rows: Vec<usize>,
    /// Each camera's keyframe row, ascending.
    observing: Vec<usize>,
    /// Each observation's point, as an index into the problem's points.
    point_of: Vec<usize>,
}

/// Builds the BA problem (`slam_it_all.edge.adjust._problem`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `keyframes` - The keyframes to adjust.
/// * `fixed` - The keyframes held fixed.
/// * `points` - The points the keyframes observe, by row, ascending.
///
/// # Returns
///
/// The problem.
///
/// # Errors
///
/// An error if a pose cannot be inverted.
fn build(
    m: &Map,
    camera: &Camera,
    keyframes: &[usize],
    fixed: &[bool],
    points: &[usize],
) -> Result<Adjustment> {
    let mut free = vec![false; m.keyframes()];
    for &k in keyframes {
        free[k] = !fixed[k];
    }
    // (keypoint row, point index): the observations in keypoint order, as Python's scan finds
    // them.
    let mut seen: Vec<(usize, usize)> = points
        .iter()
        .enumerate()
        .flat_map(|(i, &p)| m.pt_observations[p].iter().map(move |&r| (r, i)))
        .collect();
    seen.sort_unstable();
    let rows: Vec<usize> = seen.iter().map(|&(r, _)| r).collect();
    let point_of: Vec<usize> = seen.iter().map(|&(_, i)| i).collect();
    let keyframe_of: Vec<usize> = rows.iter().map(|&r| m.keyframe_of(r)).collect();
    let observing: BTreeSet<usize> = keyframe_of.iter().copied().collect();
    let observing: Vec<usize> = observing.into_iter().collect();
    let mut camera_index = vec![0; m.keyframes()];
    let mut poses = Vec::with_capacity(observing.len());
    for (c, &k) in observing.iter().enumerate() {
        camera_index[k] = to_i64(c);
        poses.push(se3::inverse(&m.kf_pose[k])?);
    }
    let mut problem = ba::Problem::with_defaults(
        camera.model,
        Intrinsics {
            values: camera.intrinsics.clone(),
            params: camera.intrinsics.len(),
        },
        poses,
        points.iter().map(|&p| m.pt_xyz[p]).collect(),
        rows.iter()
            .map(|&r| Vector2::new(m.kp_xy[r][0], m.kp_xy[r][1]))
            .collect(),
        keyframe_of.iter().map(|&k| camera_index[k]).collect(),
        point_of.iter().map(|&i| to_i64(i)).collect(),
    );
    let bf = camera.focal() * camera.baseline;
    for (i, &r) in rows.iter().enumerate() {
        let variance = sigma2(m, m.kp_level[r]);
        problem.weights[i] = 1.0 / variance;
        let depth = m.kp_depth[r];
        if depth.is_nan() {
            problem.depth_prior_weights[i] = 0.0;
        } else {
            problem.depth_prior[i] = 1.0 / depth;
            problem.depth_prior_weights[i] = bf * bf / variance;
        }
    }
    for (c, &k) in observing.iter().enumerate() {
        problem.fixed_poses[c] = !free[k];
    }
    Ok(Adjustment {
        problem,
        rows,
        observing,
        point_of,
    })
}

/// An index as the BA problem's `i64`.
///
/// # Arguments
///
/// * `index` - The index (below `isize::MAX`).
///
/// # Returns
///
/// It.
#[allow(
    clippy::cast_possible_wrap,
    reason = "indices into allocated arrays are below isize::MAX"
)]
const fn to_i64(index: usize) -> i64 {
    index as i64
}

/// The BA settings: Levenberg-Marquardt with a Huber loss, capped iterations.
///
/// # Arguments
///
/// * `options` - The edge settings (the loss's scale).
/// * `iterations` - The cap.
///
/// # Returns
///
/// The settings.
#[allow(clippy::cast_possible_wrap, reason = "settings are at most 2^30")]
fn solver_options(options: &Options, iterations: usize) -> ba::Options {
    ba::Options {
        solver: nls::Options {
            loss: nls::Loss {
                kind: nls::LossKind::Huber,
                scale: options.chi2_mono.sqrt(),
            },
            max_iterations: iterations as i64,
            ..nls::Options::default()
        },
        ..ba::Options::default()
    }
}

/// Which observations fail the chi-square test, or are behind their camera
/// (`slam_it_all.edge.adjust._outliers`).
///
/// # Arguments
///
/// * `m` - The map (the keypoints).
/// * `camera` - The camera.
/// * `problem` - The problem, at the estimate to test.
/// * `rows` - Each observation's keypoint row.
/// * `options` - The thresholds.
///
/// # Returns
///
/// One flag per observation.
///
/// # Errors
///
/// An error if a pose cannot be inverted.
#[allow(
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    reason = "the problem's indices were built from usize rows"
)]
fn outliers(
    m: &Map,
    camera: &Camera,
    problem: &ba::Problem,
    rows: &[usize],
    options: &Options,
) -> Result<Vec<bool>> {
    let t_cam_world: Vec<Matrix4<f64>> = problem
        .poses
        .iter()
        .map(se3::inverse)
        .collect::<Result<_>>()?;
    let local: Vec<Vector3<f64>> = (0..rows.len())
        .map(|i| {
            let t = &t_cam_world[problem.cam_idx[i] as usize];
            let point = problem.points[problem.pt_idx[i] as usize];
            t.fixed_view::<3, 3>(0, 0) * point + t.fixed_view::<3, 1>(0, 3)
        })
        .collect();
    let xy: Vec<[f64; 2]> = rows.iter().map(|&r| m.kp_xy[r]).collect();
    let right: Vec<f64> = rows.iter().map(|&r| m.kp_right[r]).collect();
    let variance: Vec<f64> = rows.iter().map(|&r| sigma2(m, m.kp_level[r])).collect();
    let error = chi2(camera, &local, &xy, &right, &variance)?;
    Ok(error
        .iter()
        .zip(&right)
        .map(|(&e, &u)| !(e < threshold(u, options)))
        .collect())
}

/// Bundle-adjusts keyframes and the points they observe
/// (`slam_it_all.edge.adjust.bundle_adjust`, ORB-SLAM's `LocalBundleAdjustment`).
///
/// # Arguments
///
/// * `m` - The map, changed in place: the free keyframes' poses and the points adjusted, the
///   outlier observations forgotten.
/// * `camera` - The camera.
/// * `keyframes` - The rows of the keyframes to adjust; every other keyframe observing their
///   points is held fixed.
/// * `fixed` - One flag per keyframe: true to hold it fixed even among `keyframes`.
/// * `options` - The iterations and thresholds.
///
/// # Errors
///
/// `InvalidInput` for a keyframe row out of range, flags that are not one per keyframe, or an
/// invalid camera or settings.
pub fn bundle_adjust(
    m: &mut Map,
    camera: &Camera,
    keyframes: &[usize],
    fixed: &[bool],
    options: &Options,
) -> Result<()> {
    check_setup(camera, options)?;
    if keyframes.iter().any(|&k| k >= m.keyframes()) {
        return fail(
            ErrorKind::InvalidInput,
            format!("keyframes: indices must lie in [0, {})", m.keyframes()),
        );
    }
    if fixed.len() != m.keyframes() {
        return fail(
            ErrorKind::InvalidInput,
            "fixed: expected one flag per keyframe",
        );
    }
    let points = m.observed_by(keyframes);
    if points.is_empty() {
        return Ok(());
    }
    let Adjustment {
        mut problem,
        rows,
        observing,
        point_of,
    } = build(m, camera, keyframes, fixed, &points)?;
    let first = ba::solve::solve(&problem, &solver_options(options, options.ba_iterations))?;
    let start = std::mem::replace(&mut problem.points, first.points);
    problem.poses = first.poses;
    let outlier = outliers(m, camera, &problem, &rows, options)?;
    let mut refined = problem.clone();
    let mut inlier_views = vec![0_usize; points.len()];
    for (i, &out) in outlier.iter().enumerate() {
        if out {
            refined.weights[i] = 0.0;
            refined.depth_prior_weights[i] = 0.0;
        } else {
            inlier_views[point_of[i]] += 1;
        }
    }
    // A point left with fewer than two inlier views is left out of the second solve: it goes
    // back to where it was, not where its outliers pulled it.
    for (p, &views) in inlier_views.iter().enumerate() {
        if views < 2 {
            refined.points[p] = start[p];
        }
    }
    let second = ba::solve::solve(
        &refined,
        &solver_options(options, options.ba_refine_iterations),
    )?;
    problem.poses.clone_from(&second.poses);
    problem.points.clone_from(&second.points);
    let still = outliers(m, camera, &problem, &rows, options)?;
    for (c, &k) in observing.iter().enumerate() {
        if !problem.fixed_poses[c] {
            m.kf_pose[k] = se3::inverse(&second.poses[c])?;
        }
    }
    let ids: Vec<i64> = points.iter().map(|&p| m.pt_id[p]).collect();
    for (i, &p) in points.iter().enumerate() {
        m.pt_xyz[p] = second.points[i];
    }
    let dropped: Vec<usize> = rows
        .iter()
        .zip(&still)
        .filter_map(|(&r, &out)| out.then_some(r))
        .collect();
    map::forget_observations(m, &dropped)?;
    let kept: Vec<usize> = ids.iter().filter_map(|&id| m.point_row(id)).collect();
    map::refresh_points(m, &kept)
}
