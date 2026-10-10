//! Tracking, placing each frame against the map: the Rust core's mirror of
//! `slam_it_all.edge.tracking` (ORB-SLAM's `Tracking`, lite), whose docstring states the stages.
//! Each stage reports how it went beside its result, which a run records per frame.

use nalgebra::{Matrix4, Vector3};
use slam_it_all_foundations::error::{ErrorKind, Result, fail};
use slam_it_all_foundations::matching::{
    self, Match, Projection, SearchOptions, match_descriptors,
};
use slam_it_all_foundations::p3p::Solver;
use slam_it_all_foundations::{p3p, pnp, ransac, so3};

use crate::frames::{Camera, Frame, Tracked, check_setup, search};
use crate::map::{Map, camera_centre, in_distance_range, predict_level, to_camera, viewing_cos};
use crate::options::{Mode, Options};
use crate::residuals::{chi2, sigma2, threshold};

/// Map points in a camera's view (`slam_it_all.edge.tracking.InView`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct InView {
    /// The points' rows.
    pub rows: Vec<usize>,
    /// Their projections.
    pub xy: Vec<[f64; 2]>,
    /// Their predicted right columns (NaN without a baseline).
    pub right: Vec<f64>,
    /// Their predicted levels.
    pub level: Vec<usize>,
    /// The cosine between each viewing ray and the point's mean viewing direction.
    pub view_cos: Vec<f64>,
}

/// How tracking the local map went (`slam_it_all.edge.tracking.LocalMapSummary`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocalMapSummary {
    /// The points in view: those the frame had matched, and the local map's others in view.
    pub in_view: usize,
    /// The keypoints matched before the pose was refined.
    pub matches: usize,
    /// The matches the refinement kept.
    pub inliers: usize,
}

/// A frame tracked against the local map, and the counts it adds to
/// (`slam_it_all.edge.tracking.LocalTracking`).
#[derive(Debug, Clone, PartialEq)]
pub struct LocalTracking {
    /// The refined pose and the inlier matches.
    pub tracked: Tracked,
    /// The ids of the points that were in view.
    pub visible: Vec<i64>,
    /// The ids of the inliers.
    pub found: Vec<i64>,
    /// The id of the map point each keypoint matched before the pose was refined, -1 where none.
    pub matched: Vec<i64>,
}

impl LocalTracking {
    /// How the stage went (`slam_it_all.edge.tracking.LocalTracking.summary`).
    ///
    /// # Returns
    ///
    /// The points in view, the matches and the inliers.
    #[must_use]
    pub fn summary(&self) -> LocalMapSummary {
        LocalMapSummary {
            in_view: self.visible.len(),
            matches: count_matched(&self.matched),
            inliers: self.tracked.inliers(),
        }
    }
}

/// How tracking the last frame went (`slam_it_all.edge.tracking.MotionSummary`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionSummary {
    /// The keypoints matched in the last window searched.
    pub matches: usize,
    /// Whether the window was doubled because too few matched in the first.
    pub widened: bool,
    /// The matches the refinement kept, unless too few matched to refine.
    pub inliers: Option<usize>,
}

/// Tracking the last frame: the frame, if it was tracked, and how it went
/// (`slam_it_all.edge.tracking.Motion`).
#[derive(Debug, Clone, PartialEq)]
pub struct Motion {
    /// The tracked frame, if enough points matched and survived refinement.
    pub tracked: Option<Tracked>,
    /// How it went.
    pub summary: MotionSummary,
    /// The id of the map point each keypoint matched before the pose was refined, -1 where none.
    pub matched: Vec<i64>,
}

/// How the PnP fallback went (`slam_it_all.edge.tracking.PnpSummary`); each count `None` when
/// the fallback stopped before reaching it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PnpSummary {
    /// The local map's points.
    pub candidates: usize,
    /// The keypoints matched to them by descriptor, with a bearing in front.
    pub matches: Option<usize>,
    /// RANSAC's samples, if it found a pose.
    pub iterations: Option<usize>,
    /// Why RANSAC stopped, if it found a pose.
    pub termination: Option<ransac::Termination>,
    /// Whether RANSAC found no pose.
    pub degenerate: bool,
    /// RANSAC's inliers.
    pub inliers: Option<usize>,
    /// The inliers the refinement kept.
    pub refined: Option<usize>,
}

/// The PnP fallback: the frame, if it was placed, and how it went
/// (`slam_it_all.edge.tracking.Relocalization`).
#[derive(Debug, Clone, PartialEq)]
pub struct Relocalization {
    /// The frame placed and refined, if it was.
    pub tracked: Option<Tracked>,
    /// How it went.
    pub summary: PnpSummary,
    /// The id of the map point each keypoint matched by descriptor (with a bearing in front), -1
    /// where none: all -1 if the fallback stopped before matching.
    pub matched: Vec<i64>,
}

/// Checks the camera, the settings, and a frame's matches: one point id (or -1) per keypoint.
///
/// # Arguments
///
/// * `camera` - The camera.
/// * `options` - The settings.
/// * `points` - The matches.
/// * `frame` - The frame.
/// * `name` - The matches' name, for the message.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check_tracking(
    camera: &Camera,
    options: &Options,
    points: &[i64],
    frame: &Frame,
    name: &str,
) -> Result<()> {
    check_setup(camera, options)?;
    if points.len() != frame.size() || points.iter().any(|&id| id < -1) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected one point id (or -1) per keypoint"),
        );
    }
    Ok(())
}

/// The point id each keypoint matched, from a search's matches.
///
/// # Arguments
///
/// * `m` - The map.
/// * `frame` - The frame.
/// * `found` - The `(projection, keypoint)` matches.
/// * `rows` - Each projection's point row.
///
/// # Returns
///
/// One id (or -1) per keypoint.
fn matched_ids(m: &Map, frame: &Frame, found: &[Match], rows: &[usize]) -> Vec<i64> {
    let mut out = vec![-1; frame.size()];
    for found in found {
        out[found.second] = m.pt_id[rows[found.first]];
    }
    out
}

/// How many keypoints matched.
///
/// # Arguments
///
/// * `points` - One id (or -1) per keypoint.
///
/// # Returns
///
/// The count.
fn count_matched(points: &[i64]) -> usize {
    points.iter().filter(|&&id| id >= 0).count()
}

/// Predicts a frame's pose by the constant-velocity model (`slam_it_all.edge.tracking.predict`),
/// its rotation projected back onto SO(3).
///
/// # Arguments
///
/// * `last_pose` - The last frame's `T_cam_world`.
/// * `velocity` - The last motion `T_last_previous`, or `None` before there is one.
///
/// # Returns
///
/// `velocity * last_pose`, its rotation the nearest one; or the last pose.
#[must_use]
pub fn predict(last_pose: &Matrix4<f64>, velocity: Option<&Matrix4<f64>>) -> Matrix4<f64> {
    velocity.map_or(*last_pose, |v| {
        let mut out = v * last_pose;
        let rotation = so3::project_to_so3(&out.fixed_view::<3, 3>(0, 0).into_owned());
        out.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
        out
    })
}

/// Refines a pose (`slam_it_all.edge.tracking._optimize_pose`), unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `tracked` - The starting pose and the matches.
/// * `options` - The settings.
///
/// # Returns
///
/// The refined frame.
///
/// # Errors
///
/// The refinement's error.
fn refine(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    tracked: &Tracked,
    options: &Options,
) -> Result<Tracked> {
    let points = &tracked.points;
    let n = frame.size();
    let mut xyz = vec![Vector3::zeros(); n];
    let mut matched = Vec::new();
    for i in 0..n {
        if let Some(p) = m.point_row(points[i])
            && frame.bearings[i].z > 0.0
        {
            matched.push(i);
            xyz[i] = m.pt_xyz[p];
        }
    }
    let keypoints = &frame.features.keypoints;
    let xy: Vec<[f64; 2]> = matched.iter().map(|&i| keypoints[i].xy).collect();
    let right: Vec<f64> = matched.iter().map(|&i| frame.right[i]).collect();
    let variance: Vec<f64> = matched
        .iter()
        .map(|&i| sigma2(m, keypoints[i].level))
        .collect();
    let scale = options.chi2_mono.sqrt() / camera.focal();
    let mut used = vec![false; n];
    for &i in &matched {
        used[i] = true;
    }
    let mut current = tracked.pose;
    for _ in 0..options.pose_rounds {
        let chosen: Vec<usize> = matched
            .iter()
            .copied()
            .filter(|&i| used[i] && to_camera(&current, &xyz[i]).z > 0.0)
            .collect();
        // refine_pose's minimum
        if chosen.len() < p3p::SAMPLE_SIZE {
            used.fill(false);
            for &i in &chosen {
                used[i] = true;
            }
            break;
        }
        let world: Vec<Vector3<f64>> = chosen.iter().map(|&i| xyz[i]).collect();
        let rays: Vec<Vector3<f64>> = chosen.iter().map(|&i| frame.bearings[i]).collect();
        let weights: Vec<f64> = chosen
            .iter()
            .map(|&i| 1.0 / sigma2(m, keypoints[i].level))
            .collect();
        current = pnp::refine_pose(&current, &world, &rays, scale, &weights)?.state;
        let moved: Vec<Vector3<f64>> = matched
            .iter()
            .map(|&i| to_camera(&current, &xyz[i]))
            .collect();
        let error = chi2(camera, &moved, &xy, &right, &variance)?;
        for (j, &i) in matched.iter().enumerate() {
            used[i] = error[j] < threshold(right[j], options);
        }
    }
    Ok(Tracked {
        pose: current,
        points: (0..n)
            .map(|i| if used[i] { points[i] } else { -1 })
            .collect(),
    })
}

/// Selects the points in view (`slam_it_all.edge.tracking._in_view`), unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `pose` - The camera's pose.
/// * `rows` - The candidates.
/// * `options` - The settings.
///
/// # Returns
///
/// The points in view.
///
/// # Errors
///
/// The level prediction's error.
fn select_in_view(
    m: &Map,
    camera: &Camera,
    pose: &Matrix4<f64>,
    rows: &[usize],
    options: &Options,
) -> Result<InView> {
    let centre = camera_centre(pose);
    let bf = camera.focal() * camera.baseline;
    let mut out = InView::default();
    let mut distances = Vec::new();
    for &p in rows {
        let at = to_camera(pose, &m.pt_xyz[p]);
        if !(at.z > 0.0) {
            continue;
        }
        let pixel = camera.project(&at);
        let offset = m.pt_xyz[p] - centre;
        let distance = offset.norm();
        if !(camera.inside(&pixel) && in_distance_range(m, p, distance, options)) {
            continue;
        }
        let view_cos = viewing_cos(m, p, &offset, distance);
        if !(view_cos >= options.view_cos_limit) {
            continue;
        }
        out.rows.push(p);
        out.xy.push([pixel.x, pixel.y]);
        out.right.push(if camera.baseline > 0.0 {
            pixel.x - bf / at.z
        } else {
            f64::NAN
        });
        out.view_cos.push(view_cos);
        distances.push(distance);
    }
    out.level = predict_level(m, &out.rows, &distances)?;
    Ok(out)
}

/// Refines a frame's pose against its matched points, dropping outliers
/// (`slam_it_all.edge.tracking.optimize_pose`, ORB-SLAM's `PoseOptimization`): each match
/// weighted by its level's information `1/s^(2l)`.
///
/// # Arguments
///
/// * `m` - The map (the points' positions).
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `tracked` - The starting `T_cam_world`, and the point id each keypoint matched (or -1).
/// * `options` - The rounds and chi-square thresholds.
///
/// # Returns
///
/// The refined pose and the inlier matches.
///
/// # Errors
///
/// `InvalidInput` for matches that are not one per keypoint, or an invalid camera or settings.
pub fn optimize_pose(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    tracked: &Tracked,
    options: &Options,
) -> Result<Tracked> {
    check_tracking(camera, options, &tracked.points, frame, "tracked.points")?;
    refine(m, camera, frame, tracked, options)
}

/// Selects the points a camera sees, at a distance and angle they are matchable from
/// (`slam_it_all.edge.tracking.in_view`, ORB-SLAM's `isInFrustum`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `pose` - Its `T_cam_world`.
/// * `rows` - The candidate points' rows.
/// * `options` - The viewing-angle limit.
///
/// # Returns
///
/// The points in view, in the order of `rows`.
///
/// # Errors
///
/// `InvalidInput` for a row out of range, or an invalid camera or settings.
pub fn in_view(
    m: &Map,
    camera: &Camera,
    pose: &Matrix4<f64>,
    rows: &[usize],
    options: &Options,
) -> Result<InView> {
    check_setup(camera, options)?;
    if rows.iter().any(|&r| r >= m.points()) {
        return fail(
            ErrorKind::InvalidInput,
            format!("rows: rows must lie in [0, {})", m.points()),
        );
    }
    select_in_view(m, camera, pose, rows, options)
}

/// The level range a last-frame point is searched on (`TrackWithMotionModel`): its level and
/// the next ones, or with depth, its level and the coarser ones moving forward, the finer
/// ones moving back.
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `last_pose` - The last frame's pose.
/// * `predicted` - The predicted pose.
/// * `mode` - The mode.
/// * `level` - The last keypoint's level.
///
/// # Returns
///
/// The lowest and highest level.
#[allow(clippy::cast_possible_wrap, reason = "levels are at most MAX_LEVELS")]
fn level_range(
    m: &Map,
    camera: &Camera,
    last_pose: &Matrix4<f64>,
    predicted: &Matrix4<f64>,
    mode: Mode,
    level: usize,
) -> (i64, i64) {
    let level = level as i64;
    if mode != Mode::Monocular {
        // The current camera's centre in the last camera's frame (ORB-SLAM's tlc).
        let ahead = last_pose
            .fixed_view::<1, 3>(2, 0)
            .dot(&camera_centre(predicted).transpose())
            + last_pose[(2, 3)];
        if ahead > camera.baseline {
            return (level, m.levels as i64 - 1);
        }
        if -ahead > camera.baseline {
            return (0, level);
        }
    }
    (level - 1, level + 1)
}

/// Tracks the last frame's points into this one (`slam_it_all.edge.tracking.track_last_frame`,
/// ORB-SLAM's `TrackWithMotionModel`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame to track.
/// * `last` - The last frame.
/// * `last_tracked` - Its pose and matches.
/// * `predicted` - The predicted pose.
/// * `mode` - The pipeline's mode.
/// * `options` - The windows and minimums.
///
/// # Returns
///
/// The tracked frame, none if too few points matched or survived refinement, how it went, and
/// each keypoint's match before refinement.
///
/// # Errors
///
/// `InvalidInput` for the last frame's matches not one per keypoint, or an invalid camera or
/// settings.
#[allow(
    clippy::too_many_arguments,
    clippy::cast_precision_loss,
    reason = "the stage's state, as the reference takes it; levels are small"
)]
pub fn track_last_frame(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    last: &Frame,
    last_tracked: &Tracked,
    predicted: &Matrix4<f64>,
    mode: Mode,
    options: &Options,
) -> Result<Motion> {
    check_tracking(
        camera,
        options,
        &last_tracked.points,
        last,
        "last_tracked.points",
    )?;
    let bf = camera.focal() * camera.baseline;
    let window = if mode == Mode::Stereo {
        options.stereo_motion_window
    } else {
        options.motion_window
    };
    let mut rows = Vec::new();
    let mut levels = Vec::new();
    let mut projections = Vec::new();
    for (i, k) in last.features.keypoints.iter().enumerate() {
        let Some(p) = m.point_row(last_tracked.points[i]) else {
            continue;
        };
        let at = to_camera(predicted, &m.pt_xyz[p]);
        if !(at.z > 0.0) {
            continue;
        }
        let pixel = camera.project(&at);
        if !camera.inside(&pixel) {
            continue;
        }
        let (low, high) = level_range(m, camera, &last_tracked.pose, predicted, mode, k.level);
        rows.push(p);
        levels.push(k.level);
        projections.push(Projection {
            xy: [pixel.x, pixel.y],
            radius: 0.0,
            min_level: low,
            max_level: high,
            descriptor: m.pt_descriptor[p],
            angle: k.angle,
            right: if mode == Mode::Monocular {
                f64::NAN
            } else {
                pixel.x - bf / at.z
            },
        });
    }
    let searched = SearchOptions {
        max_distance: options.th_high,
        ratio: 1.0,
        check_orientation: true,
    };
    let mut points = vec![-1; frame.size()];
    let mut widened = false;
    for factor in [1.0, 2.0] {
        widened = factor > 1.0;
        for (projection, &level) in projections.iter_mut().zip(&levels) {
            projection.radius = factor * window * m.scale_factor.powf(level as f64);
        }
        let found = search(frame, &projections, &searched, &[])?;
        points = matched_ids(m, frame, &found, &rows);
        if count_matched(&points) >= options.min_motion_matches {
            break;
        }
    }
    let matches = count_matched(&points);
    if matches < options.min_motion_matches {
        return Ok(Motion {
            tracked: None,
            summary: MotionSummary {
                matches,
                widened,
                inliers: None,
            },
            matched: points,
        });
    }
    let tracked = refine(
        m,
        camera,
        frame,
        &Tracked {
            pose: *predicted,
            points: points.clone(),
        },
        options,
    )?;
    let kept = tracked.inliers();
    Ok(Motion {
        tracked: (kept >= options.min_motion_inliers).then_some(tracked),
        summary: MotionSummary {
            matches,
            widened,
            inliers: Some(kept),
        },
        matched: points,
    })
}

/// Matches the local map's points in view and refines the pose
/// (`slam_it_all.edge.tracking.track_local_map`, ORB-SLAM's `TrackLocalMap`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `tracked` - Its pose and matches so far.
/// * `mode` - The pipeline's mode.
/// * `options` - The window, thresholds and minimums.
/// * `recovering` - Whether the frame is a recovery's or the next: the window is then
///   `recovery_local_window`.
///
/// # Returns
///
/// The refined frame, the ids of the points in view and of the inliers, and each keypoint's
/// match before refinement.
///
/// # Errors
///
/// `InvalidInput` for matches that are not one per keypoint, or an invalid camera or
/// settings.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    reason = "levels are small"
)]
pub fn track_local_map(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    tracked: &Tracked,
    mode: Mode,
    options: &Options,
    recovering: bool,
) -> Result<LocalTracking> {
    check_tracking(camera, options, &tracked.points, frame, "tracked.points")?;
    let local_points = m.observed_by(&m.window(options.window));
    let already: Vec<usize> = tracked
        .points
        .iter()
        .filter_map(|&id| m.point_row(id))
        .collect();
    let mut sorted_already = already.clone();
    sorted_already.sort_unstable();
    let candidates: Vec<usize> = local_points
        .into_iter()
        .filter(|p| sorted_already.binary_search(p).is_err())
        .collect();
    let view = select_in_view(m, camera, &tracked.pose, &candidates, options)?;
    let th = if recovering {
        options.recovery_local_window
    } else if mode == Mode::Rgbd {
        options.rgbd_local_window
    } else {
        options.local_window
    };
    let projections: Vec<Projection> = (0..view.rows.len())
        .map(|j| {
            let narrow = if view.view_cos[j] > options.aligned_view_cos {
                options.aligned_view_radius
            } else {
                options.oblique_view_radius
            };
            let level = view.level[j];
            Projection {
                xy: view.xy[j],
                radius: th * narrow * m.scale_factor.powf(level as f64),
                min_level: level as i64 - 1,
                max_level: level as i64,
                descriptor: m.pt_descriptor[view.rows[j]],
                angle: 0,
                right: view.right[j],
            }
        })
        .collect();
    let taken: Vec<bool> = tracked.points.iter().map(|&id| id >= 0).collect();
    let found = search(
        frame,
        &projections,
        &SearchOptions {
            max_distance: options.th_high,
            ratio: options.local_map_ratio,
            check_orientation: false,
        },
        &taken,
    )?;
    let mut points = matched_ids(m, frame, &found, &view.rows);
    for (point, &so_far) in points.iter_mut().zip(&tracked.points) {
        if so_far >= 0 {
            *point = so_far;
        }
    }
    let refined = refine(
        m,
        camera,
        frame,
        &Tracked {
            pose: tracked.pose,
            points: points.clone(),
        },
        options,
    )?;
    let visible = already
        .iter()
        .chain(&view.rows)
        .map(|&p| m.pt_id[p])
        .collect();
    let found = refined
        .points
        .iter()
        .copied()
        .filter(|&id| id >= 0)
        .collect();
    Ok(LocalTracking {
        tracked: refined,
        visible,
        found,
        matched: points,
    })
}

/// Places a frame against the local map by descriptors and PnP RANSAC
/// (`slam_it_all.edge.tracking.relocalize_locally`, the fallback).
///
/// # Arguments
///
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `options` - The RANSAC threshold, minimum inliers and seed.
///
/// # Returns
///
/// The frame placed and refined, none if too few inliers, how far it got, and each keypoint's
/// match by descriptor.
///
/// # Errors
///
/// `InvalidInput` for an invalid camera or settings.
pub fn relocalize_locally(
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    options: &Options,
) -> Result<Relocalization> {
    check_setup(camera, options)?;
    let unplaced = |summary, matched| {
        Ok(Relocalization {
            tracked: None,
            summary,
            matched,
        })
    };
    let local_points = m.observed_by(&m.window(options.window));
    let mut summary = PnpSummary {
        candidates: local_points.len(),
        ..PnpSummary::default()
    };
    let mut matched = vec![-1; frame.size()];
    if local_points.len() < options.pnp_min_inliers {
        return unplaced(summary, matched);
    }
    let descriptors: Vec<_> = local_points.iter().map(|&p| m.pt_descriptor[p]).collect();
    let angles: Vec<usize> = frame.features.keypoints.iter().map(|k| k.angle).collect();
    let found = match_descriptors(
        &frame.features.descriptors,
        &descriptors,
        &angles,
        &vec![0; local_points.len()],
        &matching::Options {
            max_distance: options.th_high,
            ratio: options.relocalize_ratio,
            check_orientation: false,
        },
    )?;
    let pairs: Vec<Match> = found
        .into_iter()
        .filter(|pair| frame.bearings[pair.first].z > 0.0)
        .collect();
    for pair in &pairs {
        matched[pair.first] = m.pt_id[local_points[pair.second]];
    }
    summary.matches = Some(pairs.len());
    if pairs.len() < options.pnp_min_inliers {
        return unplaced(summary, matched);
    }
    let world: Vec<Vector3<f64>> = pairs
        .iter()
        .map(|pair| m.pt_xyz[local_points[pair.second]])
        .collect();
    let rays: Vec<Vector3<f64>> = pairs
        .iter()
        .map(|pair| frame.bearings[pair.first])
        .collect();
    let settings = ransac::Options {
        threshold: options.pnp_threshold / camera.focal(),
        seed: options.seed,
        ..ransac::Options::default()
    };
    let solved = match pnp::pnp_ransac(&world, &rays, &settings, Solver::Ding, true) {
        Ok(solved) => solved,
        Err(e) if e.kind == ErrorKind::Degenerate => {
            summary.degenerate = true;
            return unplaced(summary, matched);
        }
        Err(e) => return Err(e),
    };
    let consensus = solved.state;
    let sampled = consensus.inliers.iter().filter(|&&inlier| inlier).count();
    summary.iterations = Some(solved.summary.run.iterations);
    summary.termination = Some(solved.summary.run.termination);
    summary.inliers = Some(sampled);
    if sampled < options.pnp_min_inliers {
        return unplaced(summary, matched);
    }
    let mut points = vec![-1; frame.size()];
    for (pair, &inlier) in pairs.iter().zip(&consensus.inliers) {
        if inlier {
            points[pair.first] = m.pt_id[local_points[pair.second]];
        }
    }
    let tracked = refine(
        m,
        camera,
        frame,
        &Tracked {
            pose: consensus.model,
            points,
        },
        options,
    )?;
    let kept = tracked.inliers();
    summary.refined = Some(kept);
    Ok(Relocalization {
        tracked: (kept >= options.pnp_min_inliers).then_some(tracked),
        summary,
        matched,
    })
}
