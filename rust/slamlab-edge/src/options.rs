//! The edge pipeline's settings: the Rust core's mirror of `slamlab.edge.options`.
//!
//! Every threshold named and defaulted to ORB-SLAM2's; the Python class's docstring says which
//! ORB-SLAM quantity each stands for.

use slamlab_foundations::error::{ErrorKind, Result, check_range, fail, fail_setting};
use slamlab_foundations::{matching, orb, stereo};

/// The pipeline's sensor modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// One camera.
    Monocular,
    /// A rectified stereo pair.
    Stereo,
    /// A camera and a registered depth map.
    Rgbd,
}

impl Mode {
    /// Every mode, in code order.
    pub const ALL: [Self; 3] = [Self::Monocular, Self::Stereo, Self::Rgbd];

    /// The mode's name, as `slamlab.edge.options.MODES` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Monocular => "monocular",
            Self::Stereo => "stereo",
            Self::Rgbd => "rgbd",
        }
    }
}

/// The largest an integer setting may be (Python's `1 << 30`).
const MAX_SETTING: usize = 1 << 30;

/// The edge pipeline's settings (`slamlab.edge.options.Options`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// ORB features per frame.
    pub n_features: usize,
    /// Pyramid levels.
    pub levels: usize,
    /// The pyramid's scale factor.
    pub scale_factor: f64,
    /// The FAST threshold a cell tries first.
    pub fast_threshold: i64,
    /// The FAST threshold for cells with no corner at the first.
    pub min_fast_threshold: i64,
    /// The stricter descriptor distance: initialisation, triangulation, fusion (`TH_LOW`).
    pub th_low: i64,
    /// The looser one: tracking (`TH_HIGH`).
    pub th_high: i64,
    /// The stereo row matching's Hamming distance must be below this.
    pub stereo_max_distance: i64,
    /// Its row band, in multiples of the right keypoint's level scale.
    pub stereo_band: f64,
    /// Its SAD patch's half-width.
    pub stereo_window: usize,
    /// Its SAD search's half-range.
    pub stereo_search: usize,
    /// Its matches whose best sum exceeds this times the median are dropped.
    pub stereo_outlier_factor: f64,
    /// Keypoints nearer than this many baselines are close.
    pub close_depth: f64,
    /// A monocular frame's features are this many times `n_features` before the map.
    pub init_feature_factor: usize,
    /// A monocular frame takes part in initialisation with more than this many keypoints.
    pub min_init_features: usize,
    /// A stereo or RGB-D frame starts the map with more than this many keypoints...
    pub min_init_features_depth: usize,
    /// ...and at least this many with depth (lite: ORB-SLAM resets a map lost early).
    pub min_init_depth_points: usize,
    /// The monocular initialisation's search window, in pixels.
    pub init_window: f64,
    /// Its ratio test.
    pub init_ratio: f64,
    /// Matches it needs (fewer: the reference frame is replaced).
    pub init_min_matches: usize,
    /// A monocular start keeps the points whose rays' cosine is below this.
    pub init_parallax_cos: f64,
    /// The motion model's search window, in pixels at level 0.
    pub motion_window: f64,
    /// The same for stereo.
    pub stereo_motion_window: f64,
    /// Matches the motion model needs before refining.
    pub min_motion_matches: usize,
    /// Inliers the motion model must keep after refinement.
    pub min_motion_inliers: usize,
    /// The local map's search window factor.
    pub local_window: f64,
    /// The same for RGB-D.
    pub rgbd_local_window: f64,
    /// The same on a recovery's frame and the next, in every mode.
    pub recovery_local_window: f64,
    /// The frames, the recovery's included, searched in the recovery window.
    pub recovery_window_frames: usize,
    /// Points seen at a cosine above this to their viewing direction get the narrower window.
    pub aligned_view_cos: f64,
    /// The narrower window's factor.
    pub aligned_view_radius: f64,
    /// The wider window's factor.
    pub oblique_view_radius: f64,
    /// The local map search's ratio test.
    pub local_map_ratio: f64,
    /// Inliers the local map must give for the frame to be tracked.
    pub min_tracked: usize,
    /// Inliers a frame needs just after a recovery.
    pub recovery_tracked: usize,
    /// The outlier test for a 2D observation (chi-square, 2 dof, 95%).
    pub chi2_mono: f64,
    /// The same for a 3D (left and right) one.
    pub chi2_stereo: f64,
    /// Rounds of pose refinement and outlier classification.
    pub pose_rounds: usize,
    /// A point is in view only within this cosine of its mean viewing direction.
    pub view_cos_limit: f64,
    /// A point is in view from this fraction of its minimum distance...
    pub min_distance_margin: f64,
    /// ...to this multiple of its maximum.
    pub max_distance_margin: f64,
    /// Frames after a recovery that need more inliers, and that make no keyframe once the map
    /// holds more than this many (the frame rate).
    pub max_frames: usize,
    /// Stereo and RGB-D keyframe ratio of the reference keyframe's points.
    pub ref_ratio: f64,
    /// The same for monocular.
    pub mono_ref_ratio: f64,
    /// The keyframe ratio while the map has a single keyframe.
    pub first_ref_ratio: f64,
    /// The reference keyframe's points counted have this many observations...
    pub ref_min_observations: usize,
    /// ...this many while the map holds two keyframes or fewer.
    pub ref_min_observations_early: usize,
    /// A keyframe needs more inliers than this.
    pub min_keyframe_inliers: usize,
    /// Close points tracked below which stereo and RGB-D may need a keyframe.
    pub close_tracked: usize,
    /// Untracked close points above which they do.
    pub close_untracked: usize,
    /// The sliding window's keyframes.
    pub window: usize,
    /// Points a stereo or RGB-D keyframe creates from depth, at least.
    pub min_new_points: usize,
    /// Keyframes a new stereo or RGB-D keyframe triangulates against.
    pub triangulation_neighbours: usize,
    /// The same for monocular.
    pub mono_triangulation_neighbours: usize,
    /// A triangulation match's chi-square bound to its epipolar line (1 dof).
    pub epipolar_chi2: f64,
    /// Without depth, its least squared distance from the epipole over the level scale s^l.
    pub min_epipole_distance2: f64,
    /// A monocular keyframe's least baseline to a neighbour, over its median depth.
    pub min_baseline_depth: f64,
    /// Rays whose cosine is above this are too parallel to triangulate.
    pub max_parallax_cos: f64,
    /// The scale-consistency factor of a triangulated point.
    pub scale_ratio: f64,
    /// The fusion search window, in pixels at level 0.
    pub fuse_window: f64,
    /// Local BA's iterations before dropping outliers.
    pub ba_iterations: usize,
    /// And after.
    pub ba_refine_iterations: usize,
    /// Local BA runs once the map holds this many keyframes.
    pub ba_min_keyframes: usize,
    /// The monocular map's first bundle adjustment's iterations.
    pub init_ba_iterations: usize,
    /// A new point found in less than this fraction of its views is culled.
    pub found_ratio: f64,
    /// A point is culled on evidence until this many keyframes after its creation.
    pub recent_point_keyframes: usize,
    /// A recent point this many keyframes old is culled on too few observations.
    pub point_cull_age: usize,
    /// Monocular points this weakly observed two keyframes on are culled.
    pub min_point_observations: usize,
    /// The same for stereo and RGB-D.
    pub min_point_observations_depth: usize,
    /// A keyframe is redundant when this fraction of its points are seen elsewhere.
    pub redundant_fraction: f64,
    /// By at least this many other keyframes.
    pub redundant_observers: usize,
    /// The PnP fallback's RANSAC threshold, in pixels.
    pub pnp_threshold: f64,
    /// Inliers the PnP fallback needs.
    pub pnp_min_inliers: usize,
    /// The fallback's descriptor ratio test.
    pub relocalize_ratio: f64,
    /// Failed frames in a row before the track is lost.
    pub lost_frames: usize,
    /// Every RANSAC's seed.
    pub seed: u64,
}

impl Default for Options {
    /// ORB-SLAM2's settings (and the design note's window of 10 keyframes).
    fn default() -> Self {
        Self {
            n_features: 1000,
            levels: 8,
            scale_factor: 1.2,
            fast_threshold: 20,
            min_fast_threshold: 7,
            th_low: 50,
            th_high: 100,
            stereo_max_distance: 75,
            stereo_band: 2.0,
            stereo_window: 5,
            stereo_search: 5,
            stereo_outlier_factor: 2.1,
            close_depth: 40.0,
            init_feature_factor: 2,
            min_init_features: 100,
            min_init_features_depth: 500,
            min_init_depth_points: 100,
            init_window: 100.0,
            init_ratio: 0.9,
            init_min_matches: 100,
            init_parallax_cos: 0.99998,
            motion_window: 15.0,
            stereo_motion_window: 7.0,
            min_motion_matches: 20,
            min_motion_inliers: 10,
            local_window: 1.0,
            rgbd_local_window: 3.0,
            recovery_local_window: 5.0,
            recovery_window_frames: 2,
            aligned_view_cos: 0.998,
            aligned_view_radius: 2.5,
            oblique_view_radius: 4.0,
            local_map_ratio: 0.8,
            min_tracked: 30,
            recovery_tracked: 50,
            chi2_mono: 5.991,
            chi2_stereo: 7.815,
            pose_rounds: 4,
            view_cos_limit: 0.5,
            min_distance_margin: 0.8,
            max_distance_margin: 1.2,
            max_frames: 30,
            ref_ratio: 0.75,
            mono_ref_ratio: 0.9,
            first_ref_ratio: 0.4,
            ref_min_observations: 3,
            ref_min_observations_early: 2,
            min_keyframe_inliers: 15,
            close_tracked: 100,
            close_untracked: 70,
            window: 10,
            min_new_points: 100,
            triangulation_neighbours: 10,
            mono_triangulation_neighbours: 20,
            epipolar_chi2: 3.84,
            min_epipole_distance2: 100.0,
            min_baseline_depth: 0.01,
            max_parallax_cos: 0.9998,
            scale_ratio: 1.5,
            fuse_window: 3.0,
            ba_iterations: 5,
            ba_refine_iterations: 10,
            ba_min_keyframes: 3,
            init_ba_iterations: 20,
            found_ratio: 0.25,
            recent_point_keyframes: 3,
            point_cull_age: 2,
            min_point_observations: 2,
            min_point_observations_depth: 3,
            redundant_fraction: 0.9,
            redundant_observers: 3,
            pnp_threshold: 4.0,
            pnp_min_inliers: 30,
            relocalize_ratio: 0.75,
            lost_frames: 5,
            seed: 0,
        }
    }
}

impl Options {
    /// The ORB extraction settings.
    ///
    /// # Arguments
    ///
    /// * `initializing` - For a monocular frame before the map exists: `init_feature_factor`
    ///   times the features.
    ///
    /// # Returns
    ///
    /// The settings.
    #[must_use]
    pub fn extraction(&self, initializing: bool) -> orb::Options {
        let factor = if initializing {
            self.init_feature_factor
        } else {
            1
        };
        orb::Options {
            n_features: (self.n_features * factor).min(orb::MAX_FEATURES),
            levels: self.levels,
            scale_factor: self.scale_factor,
            fast_threshold: self.fast_threshold,
            min_fast_threshold: self.min_fast_threshold,
        }
    }

    /// The stereo pair's row-matching settings.
    ///
    /// # Returns
    ///
    /// The `stereo_*` settings.
    #[must_use]
    pub fn stereo_matching(&self) -> stereo::MatchOptions {
        stereo::MatchOptions {
            max_distance: self.stereo_max_distance,
            band: self.stereo_band,
            window: self.stereo_window,
            search: self.stereo_search,
            outlier_factor: self.stereo_outlier_factor,
        }
    }
}

/// A setting's value, by its kind: what a command line gives, and what a run records.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Setting {
    /// An integer setting's.
    Integer(i64),
    /// A real setting's.
    Real(f64),
}

/// Implements [`Options::settings`] and [`Options::set`] over every field, each with its
/// kind. `settings` destructures the struct without `..`, so a field missing from the list
/// fails to build.
macro_rules! named_settings {
    ($($field:ident: $kind:ident),+ $(,)?) => {
        impl Options {
            /// Every setting's name, as the Python class's field, and value, in its order.
            ///
            /// # Returns
            ///
            /// The settings.
            #[must_use]
            pub fn settings(&self) -> Vec<(&'static str, Setting)> {
                let Self { $($field),+ } = *self;
                vec![$((stringify!($field), named_settings!(@value $kind $field))),+]
            }

            /// Sets one setting by its name, as the Python class's field.
            ///
            /// # Arguments
            ///
            /// * `name` - The setting.
            /// * `value` - Its value, of its kind.
            ///
            /// # Errors
            ///
            /// `InvalidInput` for an unknown name, a value of the other kind, or an integer
            /// the setting cannot hold (negative, for a count).
            pub fn set(&mut self, name: &str, value: Setting) -> Result<()> {
                match name {
                    $(stringify!($field) => named_settings!(@set $kind self.$field, name, value),)+
                    _ => fail(ErrorKind::InvalidInput, format!("options: no setting {name:?}")),
                }
            }
        }
    };
    (@value Integer $value:ident) => {
        Setting::Integer(i64::try_from($value).unwrap_or(i64::MAX))
    };
    (@value Real $value:ident) => {
        Setting::Real($value)
    };
    (@set Integer $slot:expr, $name:ident, $value:ident) => {
        match $value {
            Setting::Integer(integer) => match integer.try_into() {
                Ok(held) => {
                    $slot = held;
                    Ok(())
                }
                Err(_) => fail_setting($name, "out of range", integer),
            },
            Setting::Real(real) => fail_setting($name, "expected an integer", real),
        }
    };
    (@set Real $slot:expr, $name:ident, $value:ident) => {
        match $value {
            Setting::Real(real) => {
                $slot = real;
                Ok(())
            }
            Setting::Integer(integer) => fail_setting($name, "expected a real", integer),
        }
    };
}

named_settings!(
    n_features: Integer, levels: Integer, scale_factor: Real, fast_threshold: Integer,
    min_fast_threshold: Integer, th_low: Integer, th_high: Integer,
    stereo_max_distance: Integer, stereo_band: Real, stereo_window: Integer,
    stereo_search: Integer, stereo_outlier_factor: Real, close_depth: Real,
    init_feature_factor: Integer,
    min_init_features: Integer, min_init_features_depth: Integer,
    min_init_depth_points: Integer, init_window: Real,
    init_ratio: Real, init_min_matches: Integer, init_parallax_cos: Real, motion_window: Real,
    stereo_motion_window: Real, min_motion_matches: Integer, local_window: Real,
    rgbd_local_window: Real, recovery_local_window: Real, min_tracked: Integer, chi2_mono: Real, chi2_stereo: Real,
    pose_rounds: Integer, view_cos_limit: Real, max_frames: Integer, ref_ratio: Real,
    mono_ref_ratio: Real, min_keyframe_inliers: Integer, close_tracked: Integer,
    close_untracked: Integer, window: Integer, min_new_points: Integer, max_parallax_cos: Real,
    scale_ratio: Real, fuse_window: Real, ba_iterations: Integer, ba_refine_iterations: Integer,
    ba_min_keyframes: Integer,
    found_ratio: Real, min_point_observations: Integer, min_point_observations_depth: Integer,
    redundant_fraction: Real, redundant_observers: Integer, pnp_threshold: Real,
    min_motion_inliers: Integer,
    recovery_window_frames: Integer,
    aligned_view_cos: Real,
    aligned_view_radius: Real,
    oblique_view_radius: Real,
    local_map_ratio: Real,
    recovery_tracked: Integer,
    min_distance_margin: Real,
    max_distance_margin: Real,
    first_ref_ratio: Real,
    ref_min_observations: Integer,
    ref_min_observations_early: Integer,
    triangulation_neighbours: Integer,
    mono_triangulation_neighbours: Integer,
    epipolar_chi2: Real,
    min_epipole_distance2: Real,
    min_baseline_depth: Real,
    recent_point_keyframes: Integer,
    point_cull_age: Integer,
    init_ba_iterations: Integer,
    relocalize_ratio: Real,
    pnp_min_inliers: Integer, lost_frames: Integer, seed: Integer,
);

/// Checks the descriptor distances and the stereo matching's settings, named as `Options`' fields.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
fn check_matching(options: &Options) -> Result<()> {
    if let Err(error) = stereo::check_options(&options.stereo_matching()) {
        return fail(error.kind, format!("stereo_{}", error.message));
    }
    for (name, value) in [("th_low", options.th_low), ("th_high", options.th_high)] {
        check_range(name, value, 1, matching::MAX_DISTANCE)?;
    }
    Ok(())
}

/// Checks the settings (`Options.__post_init__`).
///
/// # Arguments
///
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
pub fn check_options(options: &Options) -> Result<()> {
    orb::check_options(&options.extraction(false))?;
    check_matching(options)?;
    for (name, value) in [
        ("min_init_features", options.min_init_features),
        ("min_init_features_depth", options.min_init_features_depth),
        ("min_init_depth_points", options.min_init_depth_points),
    ] {
        check_range(name, value, 0, MAX_SETTING)?;
    }
    check_range("seed", options.seed, 0, MAX_SETTING as u64)?;
    for (name, value) in [
        ("n_features", options.n_features),
        ("levels", options.levels),
        ("init_feature_factor", options.init_feature_factor),
        ("init_min_matches", options.init_min_matches),
        ("min_motion_matches", options.min_motion_matches),
        ("min_tracked", options.min_tracked),
        ("pose_rounds", options.pose_rounds),
        ("max_frames", options.max_frames),
        ("min_keyframe_inliers", options.min_keyframe_inliers),
        ("close_tracked", options.close_tracked),
        ("close_untracked", options.close_untracked),
        ("window", options.window),
        ("min_new_points", options.min_new_points),
        ("ba_iterations", options.ba_iterations),
        ("ba_refine_iterations", options.ba_refine_iterations),
        ("ba_min_keyframes", options.ba_min_keyframes),
        ("min_point_observations", options.min_point_observations),
        (
            "min_point_observations_depth",
            options.min_point_observations_depth,
        ),
        ("redundant_observers", options.redundant_observers),
        ("pnp_min_inliers", options.pnp_min_inliers),
        ("lost_frames", options.lost_frames),
        ("min_motion_inliers", options.min_motion_inliers),
        ("recovery_window_frames", options.recovery_window_frames),
        ("recovery_tracked", options.recovery_tracked),
        ("ref_min_observations", options.ref_min_observations),
        (
            "ref_min_observations_early",
            options.ref_min_observations_early,
        ),
        ("triangulation_neighbours", options.triangulation_neighbours),
        (
            "mono_triangulation_neighbours",
            options.mono_triangulation_neighbours,
        ),
        ("recent_point_keyframes", options.recent_point_keyframes),
        ("point_cull_age", options.point_cull_age),
        ("init_ba_iterations", options.init_ba_iterations),
    ] {
        check_range(name, value, 1, MAX_SETTING)?;
    }
    for (name, value) in [
        ("scale_factor", options.scale_factor),
        ("close_depth", options.close_depth),
        ("init_window", options.init_window),
        ("motion_window", options.motion_window),
        ("stereo_motion_window", options.stereo_motion_window),
        ("local_window", options.local_window),
        ("rgbd_local_window", options.rgbd_local_window),
        ("recovery_local_window", options.recovery_local_window),
        ("chi2_mono", options.chi2_mono),
        ("chi2_stereo", options.chi2_stereo),
        ("scale_ratio", options.scale_ratio),
        ("fuse_window", options.fuse_window),
        ("pnp_threshold", options.pnp_threshold),
        ("aligned_view_radius", options.aligned_view_radius),
        ("oblique_view_radius", options.oblique_view_radius),
        ("max_distance_margin", options.max_distance_margin),
        ("epipolar_chi2", options.epipolar_chi2),
        ("min_epipole_distance2", options.min_epipole_distance2),
    ] {
        if !(value.is_finite() && value > 0.0) {
            return fail_setting(name, "must be finite and positive", value);
        }
    }
    for (name, value) in [
        ("init_ratio", options.init_ratio),
        ("ref_ratio", options.ref_ratio),
        ("mono_ref_ratio", options.mono_ref_ratio),
        ("found_ratio", options.found_ratio),
        ("redundant_fraction", options.redundant_fraction),
        ("view_cos_limit", options.view_cos_limit),
        ("max_parallax_cos", options.max_parallax_cos),
        ("init_parallax_cos", options.init_parallax_cos),
        ("aligned_view_cos", options.aligned_view_cos),
        ("local_map_ratio", options.local_map_ratio),
        ("min_distance_margin", options.min_distance_margin),
        ("first_ref_ratio", options.first_ref_ratio),
        ("min_baseline_depth", options.min_baseline_depth),
        ("relocalize_ratio", options.relocalize_ratio),
    ] {
        if !(value.is_finite() && value > 0.0 && value <= 1.0) {
            return fail_setting(name, "must be in (0, 1]", value);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Native tests of the settings' checks and extraction.

    use super::*;

    /// Settings are read and set by name; an unknown name, a value of the other kind, or a
    /// count out of range is refused.
    #[test]
    fn settings_by_name() {
        let mut options = Options::default();
        let settings = options.settings();
        assert_eq!(settings.len(), 76);
        assert_eq!(settings[0], ("n_features", Setting::Integer(1000)));
        assert_eq!(settings[2], ("scale_factor", Setting::Real(1.2)));
        options.set("window", Setting::Integer(7)).unwrap();
        options.set("close_depth", Setting::Real(35.5)).unwrap();
        assert_eq!(options.window, 7);
        assert!((options.close_depth - 35.5).abs() < f64::EPSILON);
        for (name, value) in [
            ("windows", Setting::Integer(1)),
            ("window", Setting::Real(1.0)),
            ("close_depth", Setting::Integer(1)),
            ("window", Setting::Integer(-1)),
        ] {
            assert_eq!(
                options.set(name, value).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
    }

    /// The defaults pass; settings out of range are `InvalidInput`.
    #[test]
    fn checks_ranges() {
        check_options(&Options::default()).unwrap();
        for bad in [
            Options {
                window: 0,
                ..Options::default()
            },
            Options {
                ref_ratio: 1.5,
                ..Options::default()
            },
            Options {
                chi2_mono: f64::NAN,
                ..Options::default()
            },
        ] {
            assert_eq!(
                check_options(&bad).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
    }

    /// Every setting refuses a bad value by name: an integer above 2^30, a real that is not a
    /// number (review panel finding H21; Python's options test holds the same).
    #[test]
    fn every_setting_refuses_a_bad_value_by_name() {
        let unnamed: Vec<&str> = Options::default()
            .settings()
            .into_iter()
            .filter_map(|(name, setting)| {
                let bad = match setting {
                    Setting::Integer(_) => Setting::Integer((1 << 30) + 1),
                    Setting::Real(_) => Setting::Real(f64::NAN),
                };
                let mut options = Options::default();
                options.set(name, bad).unwrap();
                let refused = check_options(&options)
                    .is_err_and(|error| error.message.starts_with(&format!("{name}: ")));
                (!refused).then_some(name)
            })
            .collect();
        assert!(unnamed.is_empty(), "not refused by name: {unnamed:?}");
    }

    /// Initialisation's extraction has `init_feature_factor` times the features.
    #[test]
    fn initializing_doubles_the_features() {
        let options = Options::default();
        assert_eq!(options.extraction(true).n_features, 2 * options.n_features);
        assert_eq!(Mode::ALL.map(Mode::name), ["monocular", "stereo", "rgbd"]);
    }
}
