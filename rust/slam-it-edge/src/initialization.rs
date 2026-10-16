//! Initialisation, starting the map: the Rust core's mirror of `slam_it.edge.initialization`
//! (ORB-SLAM's `StereoInitialization` and `MonocularInitialization`). Each attempt also says how
//! it went, which a run records for every frame before the map exists.

use nalgebra::{Matrix4, Vector3};
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::matching::{Match, Projection, SearchOptions};
use slam_it_foundations::triangulation;
use slam_it_foundations::twoview::{self, Status};

use crate::adjust::{Solve, bundle_adjust};
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

/// How a frame went before the map existed, with the codes it crosses as
/// (`slam_it.edge.initialization.Outcome`, whose members document each).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// It started the map.
    Initialized = 0,
    /// Too few keypoints.
    TooFewFeatures = 1,
    /// It became the monocular reference: there was none.
    ReferenceSet = 2,
    /// Too few keypoints matched the reference, so it became the reference.
    ReferenceReplaced = 3,
    /// Too few of the matches had bearings in both views.
    TooFewMatches = 4,
    /// Two-view found no motion.
    Degenerate = 5,
    /// Two-view: the camera only rotated.
    PureRotation = 6,
    /// Two-view: two candidate motions fit about as well.
    Ambiguous = 7,
    /// Two-view: too few points had enough parallax.
    LowParallax = 8,
    /// Too few points survived the initial bundle adjustment.
    TooFewPoints = 9,
    /// The first keyframe's median depth was not positive.
    BadScale = 10,
}

impl Outcome {
    /// Every outcome, in code order.
    pub const ALL: [Self; 11] = [
        Self::Initialized,
        Self::TooFewFeatures,
        Self::ReferenceSet,
        Self::ReferenceReplaced,
        Self::TooFewMatches,
        Self::Degenerate,
        Self::PureRotation,
        Self::Ambiguous,
        Self::LowParallax,
        Self::TooFewPoints,
        Self::BadScale,
    ];

    /// The outcome's name, as `slam_it.edge.initialization.Outcome` spells it.
    ///
    /// # Returns
    ///
    /// A static string.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Initialized => "initialized",
            Self::TooFewFeatures => "too_few_features",
            Self::ReferenceSet => "reference_set",
            Self::ReferenceReplaced => "reference_replaced",
            Self::TooFewMatches => "too_few_matches",
            Self::Degenerate => "degenerate",
            Self::PureRotation => "pure_rotation",
            Self::Ambiguous => "ambiguous",
            Self::LowParallax => "low_parallax",
            Self::TooFewPoints => "too_few_points",
            Self::BadScale => "bad_scale",
        }
    }
}

/// How a frame went before the map existed
/// (`slam_it.edge.initialization.InitSummary`).
#[derive(Debug, Clone, PartialEq)]
pub struct InitSummary {
    /// What happened.
    pub outcome: Outcome,
    /// A monocular frame's matches to the reference, once there is one.
    pub matches: Option<usize>,
    /// The initial bundle adjustment's solves, if it ran.
    pub solves: Vec<Solve>,
}

impl InitSummary {
    /// An attempt's summary with no matches and no solves.
    ///
    /// # Arguments
    ///
    /// * `outcome` - What happened.
    ///
    /// # Returns
    ///
    /// The summary.
    #[must_use]
    pub const fn of(outcome: Outcome) -> Self {
        Self {
            outcome,
            matches: None,
            solves: Vec::new(),
        }
    }
}

/// An attempt to start the map (`slam_it.edge.initialization.Initialization`).
#[derive(Debug, Clone, PartialEq)]
pub struct Initialization {
    /// The started map and frame, if the map started.
    pub started: Option<Started>,
    /// How the attempt went.
    pub summary: InitSummary,
}

impl Initialization {
    /// An attempt that did not start the map.
    ///
    /// # Arguments
    ///
    /// * `outcome` - Why.
    /// * `matches` - The pairs given, if any.
    /// * `solves` - The adjustment's solves, if it ran.
    ///
    /// # Returns
    ///
    /// The attempt.
    #[must_use]
    pub const fn failed(outcome: Outcome, matches: Option<usize>, solves: Vec<Solve>) -> Self {
        Self {
            started: None,
            summary: InitSummary {
                outcome,
                matches,
                solves,
            },
        }
    }
}

/// The outcome a two-view status that did not initialise stands for.
///
/// # Arguments
///
/// * `status` - The status.
///
/// # Returns
///
/// The outcome of the same name.
const fn outcome_of(status: Status) -> Outcome {
    match status {
        Status::PureRotation => Outcome::PureRotation,
        Status::Ambiguous => Outcome::Ambiguous,
        Status::LowParallax => Outcome::LowParallax,
        Status::Initialized => Outcome::Initialized,
    }
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
    if frame.features.levels != options.features.levels
        || frame.features.scale_factor != options.features.scale_factor
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
/// gauge, `slam_it.edge.initialization._scaled`).
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
/// (`slam_it.edge.initialization.initialize_from_depth`).
///
/// # Arguments
///
/// * `frame` - The frame.
/// * `options` - The minimum keypoints, and the pyramid.
///
/// # Returns
///
/// The map of one keyframe at the origin with a point at every keypoint with depth (and a
/// bearing), or none unless it has more than `min_init_features_depth` keypoints; and the
/// outcome.
///
/// # Errors
///
/// `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
pub fn initialize_from_depth(frame: &Frame, options: &Options) -> Result<Initialization> {
    check_frame(frame, options, "frame")?;
    if frame.size() <= options.initialization.min_features_depth {
        return Ok(Initialization::failed(
            Outcome::TooFewFeatures,
            None,
            Vec::new(),
        ));
    }
    let has_depth: Vec<usize> = (0..frame.size())
        .filter(|&i| !frame.depth[i].is_nan() && !frame.bearings[i].x.is_nan())
        .collect();
    let mut m = Map::empty(options.features.levels, options.features.scale_factor);
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
    Ok(Initialization {
        started: Some(Started {
            map: m,
            tracked: Tracked {
                pose: Matrix4::identity(),
                points,
            },
        }),
        summary: InitSummary::of(Outcome::Initialized),
    })
}

/// Matches a monocular reference frame to a later one
/// (`slam_it.edge.initialization.match_for_initialization`, ORB-SLAM's
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
                radius: options.initialization.window,
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
            max_distance: options.matching.th_low,
            ratio: options.initialization.ratio,
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
/// (`slam_it.edge.initialization._kept_points`, ORB-SLAM's `CheckRT`): the chosen motion's
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
        .filter(|&(j, _)| {
            seen.in_front[j] && seen.parallax[j].cos() < options.initialization.parallax_cos
        })
        .map(|(j, &i)| (i, seen.points[j]))
        .unzip())
}

/// Starts the map from two monocular frames
/// (`slam_it.edge.initialization.initialize_monocular`).
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
/// first's median depth is 1, or none if too few keypoints match, the views do not
/// initialise, or too few points survive the adjustment; with the pairs given, the outcome and
/// the adjustment's solves.
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
) -> Result<Initialization> {
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
    let given = Some(matches.len());
    let failed = |outcome, solves| Ok(Initialization::failed(outcome, given, solves));
    if pairs.len() < options.initialization.min_matches {
        return failed(Outcome::TooFewMatches, Vec::new());
    }
    let bearings1: Vec<Vector3<f64>> = pairs.iter().map(|p| reference.bearings[p[0]]).collect();
    let bearings2: Vec<Vector3<f64>> = pairs.iter().map(|p| frame.bearings[p[1]]).collect();
    let result = match twoview::initialize(
        &bearings1,
        &bearings2,
        &twoview::Options {
            focal: camera.focal(),
            seed: options.seed,
            max_iterations: i64::try_from(options.initialization.ransac_iterations)
                .unwrap_or(i64::MAX),
            ..twoview::Options::default()
        },
    ) {
        Ok(result) => result,
        Err(e) if e.kind == ErrorKind::Degenerate => {
            return failed(Outcome::Degenerate, Vec::new());
        }
        Err(e) => return Err(e),
    };
    if result.status != Status::Initialized {
        return failed(outcome_of(result.status), Vec::new());
    }
    let mut m = Map::empty(options.features.levels, options.features.scale_factor);
    insert_unobserving(&mut m, reference, &Matrix4::identity())?;
    insert_unobserving(&mut m, frame, &result.relative_pose)?;
    let second = m.kf_start[1];
    let (kept, xyz) = kept_points(&result, &bearings1, &bearings2, options)?;
    let observers: Vec<Vec<usize>> = kept
        .iter()
        .map(|&i| vec![pairs[i][0], second + pairs[i][1]])
        .collect();
    map::add_points(&mut m, &xyz, &observers)?;
    let mut adjusted = *options;
    adjusted.adjustment.iterations = options.initialization.ba_iterations;
    let adjustment = bundle_adjust(&mut m, camera, &[0, 1], &[true, false], &adjusted)?;
    if m.points() < options.initialization.min_matches {
        // ORB-SLAM resets with fewer than 100 tracked points
        return failed(Outcome::TooFewPoints, adjustment.solves);
    }
    let first = m.kf_pose[0];
    let mut depths: Vec<f64> = m
        .pt_xyz
        .iter()
        .map(|x| first.fixed_view::<1, 3>(2, 0).dot(&x.transpose()) + first[(2, 3)])
        .collect();
    let middle = median(&mut depths);
    if !(middle > 0.0) {
        return failed(Outcome::BadScale, adjustment.solves);
    }
    scale(&mut m, 1.0 / middle)?;
    let tracked = Tracked {
        pose: m.kf_pose[1],
        points: m.kp_point[m.kf_start[1]..m.kf_start[2]].to_vec(),
    };
    Ok(Initialization {
        started: Some(Started { map: m, tracked }),
        summary: InitSummary {
            outcome: Outcome::Initialized,
            matches: given,
            solves: adjustment.solves,
        },
    })
}
