//! Initialisation, starting the map: the Rust core's mirror of `slamlab.edge.initialization`
//! (ORB-SLAM's `StereoInitialization` and `MonocularInitialization`).

use nalgebra::{Matrix4, Vector3};
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::matching::{Match, Projection, SearchOptions};
use slamlab_foundations::twoview::{self, Status};

use crate::adjust::bundle_adjust;
use crate::frames::{Camera, Frame, Tracked, check_camera, search};
use crate::map::{self, Map};
use crate::options::{Options, TH_LOW, check_options};

/// The monocular map's first bundle adjustment's iterations (ORB-SLAM's
/// `GlobalBundleAdjustemnt(map, 20)`).
pub const INIT_BA_ITERATIONS: usize = 20;

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
/// gauge, `slamlab.edge.initialization._scaled`).
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
/// (`slamlab.edge.initialization.initialize_from_depth`).
///
/// # Arguments
///
/// * `frame` - The frame.
/// * `options` - The minimum keypoints with depth, and the pyramid.
///
/// # Returns
///
/// The map of one keyframe at the origin with a point at every keypoint with depth (and a
/// bearing), or `None` if fewer than `min_depth_features` have depth.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
pub fn initialize_from_depth(frame: &Frame, options: &Options) -> Result<Option<Started>> {
    check_frame(frame, options, "frame")?;
    let has_depth: Vec<usize> = (0..frame.size())
        .filter(|&i| !frame.depth[i].is_nan() && !frame.bearings[i].x.is_nan())
        .collect();
    if has_depth.len() < options.min_depth_features {
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
/// (`slamlab.edge.initialization.match_for_initialization`, ORB-SLAM's
/// `SearchForInitialization`).
///
/// # Arguments
///
/// * `reference` - The reference frame.
/// * `frame` - The later frame.
/// * `options` - The window and ratio.
///
/// # Returns
///
/// `(reference keypoint, frame keypoint)` matches, by reference keypoint.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
pub fn match_for_initialization(
    reference: &Frame,
    frame: &Frame,
    options: &Options,
) -> Result<Vec<Match>> {
    check_frame(reference, options, "reference")?;
    check_frame(frame, options, "frame")?;
    let rows: Vec<usize> = (0..reference.size())
        .filter(|&i| reference.features.keypoints[i].level == 0)
        .collect();
    let projections: Vec<Projection> = rows
        .iter()
        .map(|&i| {
            let k = &reference.features.keypoints[i];
            Projection {
                xy: k.xy,
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
            max_distance: TH_LOW,
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

/// Starts the map from two monocular frames
/// (`slamlab.edge.initialization.initialize_monocular`).
///
/// # Arguments
///
/// * `camera` - The camera.
/// * `reference` - The reference frame (the first keyframe, at the origin).
/// * `frame` - The later frame.
/// * `options` - The window, the minimum matches, the seed, and the pyramid.
///
/// # Returns
///
/// The map of two keyframes and their points, scaled so the first's median depth is 1, or
/// `None` if too few keypoints match, the views do not initialise, or too few points survive
/// the adjustment.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', or an invalid camera or
/// settings.
pub fn initialize_monocular(
    camera: &Camera,
    reference: &Frame,
    frame: &Frame,
    options: &Options,
) -> Result<Option<Started>> {
    check_camera(camera)?;
    let pairs: Vec<Match> = match_for_initialization(reference, frame, options)?
        .into_iter()
        .filter(|p| !reference.bearings[p.first].x.is_nan() && !frame.bearings[p.second].x.is_nan())
        .collect();
    if pairs.len() < options.init_min_matches {
        return Ok(None);
    }
    let bearings1: Vec<Vector3<f64>> = pairs.iter().map(|p| reference.bearings[p.first]).collect();
    let bearings2: Vec<Vector3<f64>> = pairs.iter().map(|p| frame.bearings[p.second]).collect();
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
    let (xyz, observers): (Vec<Vector3<f64>>, Vec<Vec<usize>>) = pairs
        .iter()
        .enumerate()
        .filter(|&(i, _)| result.triangulated[i])
        .map(|(i, p)| (result.points[i], vec![p.first, second + p.second]))
        .unzip();
    map::add_points(&mut m, &xyz, &observers)?;
    let adjusted = Options {
        ba_iterations: INIT_BA_ITERATIONS,
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
