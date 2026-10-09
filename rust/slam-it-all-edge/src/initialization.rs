//! Initialisation, starting the map: the Rust core's mirror of `slam_it_all.edge.initialization`
//! (ORB-SLAM's `StereoInitialization` and `MonocularInitialization`).

use nalgebra::{Matrix4, Vector3};
use slam_it_all_foundations::error::{ErrorKind, Result, fail};
use slam_it_all_foundations::matching::{Match, Projection, SearchOptions};
use slam_it_all_foundations::triangulation;
use slam_it_all_foundations::twoview::{self, Status};

use crate::adjust::bundle_adjust;
use crate::frames::{Camera, Frame, Tracked, check_camera, search};
use crate::map::{self, Map};
use crate::options::{Options, check_options};

/// A started map and the frame it was started on, tracked.
#[derive(Debug, Clone, PartialEq)]
pub struct Started {
    /// The map.
    pub map: Map,
    /// The frame (the map's last keyframe), tracked.
    pub tracked: Tracked,
}

/// Checks the settings and a frame's pyramid against them.
///
/// # Arguments
///
/// * `frame` - The frame.
/// * `options` - The settings.
/// * `name` - The frame's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for invalid settings, or a pyramid that is not theirs.
#[allow(
    clippy::float_cmp,
    reason = "the frame's scale factor must be the settings' exactly, as in Python"
)]
fn check_frame(frame: &Frame, options: &Options, name: &str) -> Result<()> {
    check_options(options)?;
    if frame.features.levels != options.levels
        || frame.features.scale_factor != options.scale_factor
    {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: its pyramid must be the settings'"),
        );
    }
    Ok(())
}

/// Adds a keyframe that observes no point yet.
///
/// # Arguments
///
/// * `m` - The map.
/// * `frame` - The frame.
/// * `pose` - Its `T_cam_world`.
///
/// # Errors
///
/// The map's error.
fn insert_unobserving(m: &mut Map, frame: &Frame, pose: &Matrix4<f64>) -> Result<()> {
    map::insert_keyframe(m, frame, pose, &vec![-1; frame.size()])
}

/// Scales the map's translations and points, and refreshes the points (a monocular map's
/// gauge, `slam_it_all.edge.initialization._scaled`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `factor` - The scale.
///
/// # Errors
///
/// The map's error.
fn scale(m: &mut Map, factor: f64) -> Result<()> {
    for pose in &mut m.kf_pose {
        let mut translation = pose.fixed_view_mut::<3, 1>(0, 3);
        translation *= factor;
    }
    for x in &mut m.pt_xyz {
        *x *= factor;
    }
    let rows: Vec<usize> = (0..m.points()).collect();
    map::refresh_points(m, &rows)
}

/// The median of some values: the mean of the middle two for an even count.
///
/// # Arguments
///
/// * `values` - The values (not empty; reordered).
///
/// # Returns
///
/// The median.
fn median(values: &mut [f64]) -> f64 {
    let odd = values.len() % 2 == 1;
    let (lower, upper, _) = values.select_nth_unstable_by(values.len() / 2, f64::total_cmp);
    if odd {
        return *upper;
    }
    let lower = lower.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    f64::midpoint(lower, *upper)
}

/// Starts the map from a stereo or RGB-D frame
/// (`slam_it_all.edge.initialization.initialize_from_depth`).
///
/// # Arguments
///
/// * `frame` - The frame.
/// * `options` - The minimum keypoints, and the pyramid.
///
/// # Returns
///
/// The map of one keyframe at the origin with a point at every keypoint with depth (and a
/// bearing), or `None` unless it has more than `min_init_features_depth` keypoints and
/// `min_init_depth_points` with depth.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
pub fn initialize_from_depth(frame: &Frame, options: &Options) -> Result<Option<Started>> {
    check_frame(frame, options, "frame")?;
    if frame.size() <= options.min_init_features_depth {
        return Ok(None);
    }
    let has_depth: Vec<usize> = (0..frame.size())
        .filter(|&i| !frame.depth[i].is_nan() && !frame.bearings[i].x.is_nan())
        .collect();
    if has_depth.len() < options.min_init_depth_points {
        return Ok(None);
    }
    let mut m = Map::empty(options.levels, options.scale_factor);
    insert_unobserving(&mut m, frame, &Matrix4::identity())?;
    let xyz: Vec<Vector3<f64>> = has_depth
        .iter()
        .map(|&i| {
            let b = frame.bearings[i];
            b * (frame.depth[i] / b.z)
        })
        .collect();
    let observers: Vec<Vec<usize>> = has_depth.iter().map(|&i| vec![i]).collect();
    map::add_points(&mut m, &xyz, &observers)?;
    let mut points = vec![-1; frame.size()];
    for (j, &i) in has_depth.iter().enumerate() {
        points[i] = m.pt_id[j];
    }
    Ok(Some(Started {
        map: m,
        tracked: Tracked {
            pose: Matrix4::identity(),
            points,
        },
    }))
}

/// Matches a monocular reference frame to a later one
/// (`slam_it_all.edge.initialization.match_for_initialization`, ORB-SLAM's
/// `SearchForInitialization`).
///
/// # Arguments
///
/// * `reference` - The reference frame.
/// * `frame` - The later frame.
/// * `previous` - Where each reference keypoint was last matched, in pixels (its own position
///   until it is matched; ORB-SLAM's `vbPrevMatched`).
/// * `options` - The window and ratio.
///
/// # Returns
///
/// `(reference keypoint, frame keypoint)` matches, by reference keypoint.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', `previous` that is not one
/// finite pixel per reference keypoint, or invalid settings.
pub fn match_for_initialization(
    reference: &Frame,
    frame: &Frame,
    previous: &[[f64; 2]],
    options: &Options,
) -> Result<Vec<Match>> {
    check_frame(reference, options, "reference")?;
    check_frame(frame, options, "frame")?;
    if previous.len() != reference.size()
        || !previous
            .iter()
            .all(|xy| xy[0].is_finite() && xy[1].is_finite())
    {
        return fail(
            ErrorKind::InvalidInput,
            "previous: expected one finite pixel per reference keypoint",
        );
    }
    let rows: Vec<usize> = (0..reference.size())
        .filter(|&i| reference.features.keypoints[i].level == 0)
        .collect();
    let projections: Vec<Projection> = rows
        .iter()
        .map(|&i| {
            let k = &reference.features.keypoints[i];
            Projection {
                xy: previous[i],
                radius: options.init_window,
                min_level: 0,
                max_level: 0,
                descriptor: reference.features.descriptors[i],
                angle: k.angle,
                right: f64::NAN,
            }
        })
        .collect();
    let mut found = search(
        frame,
        &projections,
        &SearchOptions {
            max_distance: options.th_low,
            ratio: options.init_ratio,
            check_orientation: true,
        },
        &[],
    )?;
    for m in &mut found {
        m.first = rows[m.first];
    }
    Ok(found)
}

/// The matches a monocular start keeps, and their points
/// (`slam_it_all.edge.initialization._kept_points`, ORB-SLAM's `CheckRT`): the chosen motion's
/// inliers in front of both views with rays whose cosine is below `init_parallax_cos`.
///
/// # Arguments
///
/// * `result` - The two-view initialisation.
/// * `bearings1` - The matches' bearings in the reference.
/// * `bearings2` - In the later frame.
/// * `options` - The parallax.
///
/// # Returns
///
/// The kept matches' indices and their points.
///
/// # Errors
///
/// The triangulation's error.
fn kept_points(
    result: &twoview::Initialization,
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    options: &Options,
) -> Result<(Vec<usize>, Vec<Vector3<f64>>)> {
    let inliers: Vec<usize> = (0..result.inliers.len())
        .filter(|&i| result.inliers[i])
        .collect();
    let rays1: Vec<Vector3<f64>> = inliers.iter().map(|&i| bearings1[i]).collect();
    let rays2: Vec<Vector3<f64>> = inliers.iter().map(|&i| bearings2[i]).collect();
    let seen = triangulation::two_view(&result.relative_pose, &rays1, &rays2)?;
    Ok(inliers
        .iter()
        .enumerate()
        .filter(|&(j, _)| seen.in_front[j] && seen.parallax[j].cos() < options.init_parallax_cos)
        .map(|(j, &i)| (i, seen.points[j]))
        .unzip())
}

/// Starts the map from two monocular frames
/// (`slam_it_all.edge.initialization.initialize_monocular`).
///
/// # Arguments
///
/// * `camera` - The camera.
/// * `reference` - The reference frame (the first keyframe, at the origin).
/// * `frame` - The later frame.
/// * `matches` - `(reference keypoint, frame keypoint)` pairs.
/// * `options` - The minimum matches, the parallax, the seed, and the pyramid.
///
/// # Returns
///
/// The map of two keyframes and their points (the chosen motion's inliers in front of both
/// views with rays whose cosine is below `init_parallax_cos`, `CheckRT`), scaled so the
/// first's median depth is 1, or `None` if too few keypoints match, the views do not
/// initialise, or too few points survive the adjustment.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', a pair that is not a
/// keypoint of each frame, or an invalid camera or settings.
pub fn initialize_monocular(
    camera: &Camera,
    reference: &Frame,
    frame: &Frame,
    matches: &[[usize; 2]],
    options: &Options,
) -> Result<Option<Started>> {
    check_camera(camera)?;
    check_frame(reference, options, "reference")?;
    check_frame(frame, options, "frame")?;
    if matches
        .iter()
        .any(|p| p[0] >= reference.size() || p[1] >= frame.size())
    {
        return fail(
            ErrorKind::InvalidInput,
            "matches.pairs: each must be a keypoint of the reference and of the frame",
        );
    }
    let pairs: Vec<[usize; 2]> = matches
        .iter()
        .copied()
        .filter(|p| !reference.bearings[p[0]].x.is_nan() && !frame.bearings[p[1]].x.is_nan())
        .collect();
    if pairs.len() < options.init_min_matches {
        return Ok(None);
    }
    let bearings1: Vec<Vector3<f64>> = pairs.iter().map(|p| reference.bearings[p[0]]).collect();
    let bearings2: Vec<Vector3<f64>> = pairs.iter().map(|p| frame.bearings[p[1]]).collect();
    let result = match twoview::initialize(
        &bearings1,
        &bearings2,
        &twoview::Options {
            focal: camera.focal(),
            seed: options.seed,
            ..twoview::Options::default()
        },
    ) {
        Ok(result) => result,
        Err(e) if e.kind == ErrorKind::Degenerate => return Ok(None),
        Err(e) => return Err(e),
    };
    if result.status != Status::Initialized {
        return Ok(None);
    }
    let mut m = Map::empty(options.levels, options.scale_factor);
    insert_unobserving(&mut m, reference, &Matrix4::identity())?;
    insert_unobserving(&mut m, frame, &result.relative_pose)?;
    let second = m.kf_start[1];
    let (kept, xyz) = kept_points(&result, &bearings1, &bearings2, options)?;
    let observers: Vec<Vec<usize>> = kept
        .iter()
        .map(|&i| vec![pairs[i][0], second + pairs[i][1]])
        .collect();
    map::add_points(&mut m, &xyz, &observers)?;
    let adjusted = Options {
        ba_iterations: options.init_ba_iterations,
        ..*options
    };
    bundle_adjust(&mut m, camera, &[0, 1], &[true, false], &adjusted)?;
    if m.points() < options.init_min_matches {
        return Ok(None); // ORB-SLAM resets with fewer than 100 tracked points
    }
    let first = m.kf_pose[0];
    let mut depths: Vec<f64> = m
        .pt_xyz
        .iter()
        .map(|x| first.fixed_view::<1, 3>(2, 0).dot(&x.transpose()) + first[(2, 3)])
        .collect();
    let middle = median(&mut depths);
    if !(middle > 0.0) {
        return Ok(None);
    }
    scale(&mut m, 1.0 / middle)?;
    let tracked = Tracked {
        pose: m.kf_pose[1],
        points: m.kp_point[m.kf_start[1]..m.kf_start[2]].to_vec(),
    };
    Ok(Some(Started { map: m, tracked }))
}
