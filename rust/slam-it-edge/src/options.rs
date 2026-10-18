//! The edge pipeline's settings: the Rust core's mirror of `slam_it.edge.options`.
//!
//! Every threshold named and defaulted to ORB-SLAM2's; the Python class's docstring says which
//! ORB-SLAM quantity each stands for.

use slam_it_foundations::error::{ErrorKind, Result, check_range, fail, fail_setting};
use slam_it_foundations::{matching, orb, stereo};

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

    /// The mode's name, as `slam_it.edge.options.MODES` spells it.
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

/// ORB extraction's settings (`Options.features`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeatureOptions {
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
    /// A monocular frame's features are this many times `features.n_features` before the map.
    pub init_factor: usize,
}

impl Default for FeatureOptions {
    fn default() -> Self {
        Self {
            n_features: 1000,
            levels: 8,
            scale_factor: 1.2,
            fast_threshold: 20,
            min_fast_threshold: 7,
            init_factor: 2,
        }
    }
}

/// The descriptor distances matching takes in every stage (`Options.matching`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchingOptions {
    /// The stricter descriptor distance: initialisation, triangulation, fusion (`TH_LOW`).
    pub th_low: i64,
    /// The looser one: tracking (`TH_HIGH`).
    pub th_high: i64,
}

impl Default for MatchingOptions {
    fn default() -> Self {
        Self {
            th_low: 50,
            th_high: 100,
        }
    }
}

/// A stereo pair's row matching, and the close depth of stereo and RGB-D (`Options.stereo`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StereoOptions {
    /// The stereo row matching's Hamming distance must be below this.
    pub max_distance: i64,
    /// Its row band, in multiples of the right keypoint's level scale.
    pub band: f64,
    /// Its SAD patch's half-width.
    pub window: usize,
    /// Its SAD search's half-range.
    pub search: usize,
    /// Its matches whose best sum exceeds this times the median are dropped.
    pub outlier_factor: f64,
    /// Keypoints nearer than this many baselines are close.
    pub close_depth: f64,
}

impl Default for StereoOptions {
    fn default() -> Self {
        Self {
            max_distance: 75,
            band: 2.0,
            window: 5,
            search: 5,
            outlier_factor: 2.1,
            close_depth: 40.0,
        }
    }
}

/// Starting the map (`Options.initialization`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitializationOptions {
    /// A monocular frame takes part in initialisation with more than this many keypoints.
    pub min_features: usize,
    /// A stereo or RGB-D frame starts the map with more than this many keypoints.
    pub min_features_depth: usize,
    /// The monocular initialisation's search window, in pixels.
    pub window: f64,
    /// Its ratio test.
    pub ratio: f64,
    /// Matches it needs (fewer: the reference frame is replaced).
    pub min_matches: usize,
    /// A monocular start keeps the points whose rays' cosine is below this.
    pub parallax_cos: f64,
    /// The monocular map's first bundle adjustment's iterations.
    pub ba_iterations: usize,
    /// The most samples a monocular start's two-view RANSAC draws, per model.
    pub ransac_iterations: usize,
}

impl Default for InitializationOptions {
    fn default() -> Self {
        Self {
            min_features: 100,
            min_features_depth: 500,
            window: 100.0,
            ratio: 0.9,
            min_matches: 100,
            parallax_cos: 0.99998,
            ba_iterations: 20,
            ransac_iterations: 200,
        }
    }
}

/// Placing each frame: the motion model, the local map, the PnP fallback
/// (`Options.tracking`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingOptions {
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
    /// Frames after a recovery that need more inliers, and that make no keyframe once the map
    /// holds more than this many (the frame rate).
    pub max_frames: usize,
    /// A point is in view only within this cosine of its mean viewing direction.
    pub view_cos_limit: f64,
    /// A point is in view from this fraction of its minimum distance...
    pub min_distance_margin: f64,
    /// ...to this multiple of its maximum.
    pub max_distance_margin: f64,
    /// The PnP fallback's RANSAC threshold, in pixels.
    pub pnp_threshold: f64,
    /// Inliers the PnP fallback needs.
    pub pnp_min_inliers: usize,
}

impl Default for TrackingOptions {
    fn default() -> Self {
        Self {
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
            max_frames: 30,
            view_cos_limit: 0.5,
            min_distance_margin: 0.8,
            max_distance_margin: 1.2,
            pnp_threshold: 4.0,
            pnp_min_inliers: 30,
        }
    }
}

/// Pose optimisation and local BA (`Options.adjustment`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustmentOptions {
    /// The outlier test for a 2D observation (chi-square, 2 dof, 95%).
    pub chi2_mono: f64,
    /// The same for a 3D (left and right) one.
    pub chi2_stereo: f64,
    /// Rounds of pose refinement and outlier classification.
    pub pose_rounds: usize,
    /// Local BA's iterations before dropping outliers.
    pub iterations: usize,
    /// And after.
    pub refine_iterations: usize,
    /// Local BA runs once the map holds this many keyframes.
    pub min_keyframes: usize,
}

impl Default for AdjustmentOptions {
    fn default() -> Self {
        Self {
            chi2_mono: 5.991,
            chi2_stereo: 7.815,
            pose_rounds: 4,
            iterations: 5,
            refine_iterations: 10,
            min_keyframes: 3,
        }
    }
}

/// The keyframe decision (`Options.keyframes`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyframeOptions {
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
    pub min_inliers: usize,
    /// Close points tracked below which stereo and RGB-D may need a keyframe.
    pub close_tracked: usize,
    /// Untracked close points above which they do.
    pub close_untracked: usize,
    /// With local mapping busy, in threads of their own, a stereo or RGB-D frame tracking fewer
    /// than this fraction of its reference keyframe's points still asks for a keyframe
    /// (`NeedNewKeyFrame`'s `c1c`).
    pub busy_ref_ratio: f64,
}

impl Default for KeyframeOptions {
    fn default() -> Self {
        Self {
            ref_ratio: 0.75,
            mono_ref_ratio: 0.9,
            first_ref_ratio: 0.4,
            ref_min_observations: 3,
            ref_min_observations_early: 2,
            min_inliers: 15,
            close_tracked: 100,
            close_untracked: 70,
            busy_ref_ratio: 0.25,
        }
    }
}

/// Local mapping: points from depth, triangulation, fusion and culling (`Options.mapping`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MappingOptions {
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
    /// A new stereo or RGB-D keyframe's points are fused with its best this many links'.
    pub fuse_neighbours: usize,
    /// The same for monocular.
    pub mono_fuse_neighbours: usize,
    /// And with each one's best this many links'.
    pub fuse_second_neighbours: usize,
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
    /// With local mapping busy, in threads of their own, a stereo or RGB-D keyframe is queued
    /// only while fewer than this many wait (`KeyframesInQueue() < 3`).
    pub max_queued: usize,
}

impl Default for MappingOptions {
    fn default() -> Self {
        Self {
            min_new_points: 100,
            triangulation_neighbours: 10,
            mono_triangulation_neighbours: 20,
            epipolar_chi2: 3.84,
            min_epipole_distance2: 100.0,
            min_baseline_depth: 0.01,
            max_parallax_cos: 0.9998,
            scale_ratio: 1.5,
            fuse_window: 3.0,
            fuse_neighbours: 10,
            mono_fuse_neighbours: 20,
            fuse_second_neighbours: 5,
            found_ratio: 0.25,
            recent_point_keyframes: 3,
            point_cull_age: 2,
            min_point_observations: 2,
            min_point_observations_depth: 3,
            redundant_fraction: 0.9,
            redundant_observers: 3,
            max_queued: 3,
        }
    }
}

/// The covisibility graph (`Options.covisibility`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CovisibilityOptions {
    /// Keyframes observing at least this many of the same points are linked; one sharing this
    /// many with none is linked to the one it shares most with.
    pub min_points: usize,
    /// Tracking's local map stops adding K1's neighbours once it holds more than this many
    /// keyframes.
    pub local_keyframes: usize,
    /// Of a K1 keyframe's best this many links, the first not yet in the local map joins.
    pub local_neighbours: usize,
    /// Keyframes sharing at least this many points are strongly linked.
    pub strong_points: usize,
}

impl Default for CovisibilityOptions {
    fn default() -> Self {
        Self {
            min_points: 15,
            local_keyframes: 80,
            local_neighbours: 10,
            strong_points: 100,
        }
    }
}

/// Relocalising a failed frame against the whole map (`Options.relocalization`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelocalizationOptions {
    /// Relocalise a failed frame through the keyframe database (needs a vocabulary).
    pub enabled: bool,
    /// The descriptor ratio test of the fallback and of relocalisation's matching by word.
    pub ratio: f64,
    /// Relocalisation scores keyframes sharing more than this fraction of the most words.
    pub common_words: f64,
    /// A scored keyframe's score is accumulated over this many best covisible keyframes.
    pub neighbours: usize,
    /// Groups above this fraction of the best accumulated score give a candidate.
    pub group_score: f64,
    /// A candidate needs this many matches by word.
    pub min_matches: usize,
    /// A candidate's PnP RANSAC needs this fraction of its matches as inliers.
    pub inlier_ratio: f64,
    /// ...and at least this many, before and after the pose is optimised.
    pub min_inliers: usize,
    /// The confidence its samples are drawn for, in (0, 1).
    pub confidence: f64,
    /// The most samples it draws.
    pub ransac_iterations: usize,
    /// The window, in pixels at the predicted level, of the search by projection.
    pub window: f64,
    /// With more inliers than this but too few still, the search is repeated narrower.
    pub narrow_inliers: usize,
    /// That window.
    pub narrow_window: f64,
    /// Its descriptor distance.
    pub narrow_distance: i64,
}

impl Default for RelocalizationOptions {
    fn default() -> Self {
        Self {
            enabled: true,
            ratio: 0.75,
            common_words: 0.8,
            neighbours: 10,
            group_score: 0.75,
            min_matches: 15,
            inlier_ratio: 0.5,
            min_inliers: 10,
            confidence: 0.99,
            ransac_iterations: 300,
            window: 10.0,
            narrow_inliers: 30,
            narrow_window: 3.0,
            narrow_distance: 64,
        }
    }
}

/// Loop closure: detection, the loop's similarity, its correction (`Options.loops`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopOptions {
    /// Close loops in the map being tracked (needs the keyframe database).
    pub enabled: bool,
    /// A map of fewer keyframes is not searched.
    pub min_keyframes: usize,
    /// The best groups of keyframes the database gives.
    pub candidates: usize,
    /// The database scores keyframes sharing more than this fraction of the most words.
    pub common_words: f64,
    /// A candidate's group: it and its best this many links; and the confirming links.
    pub neighbours: usize,
    /// The ratio test of the matching by word with a group.
    pub ratio: f64,
    /// A group needs this many matches by word.
    pub min_matches: usize,
    /// The loop's Sim(3) RANSAC needs this many inliers...
    pub min_inliers: usize,
    /// ...drawing its samples with this confidence, in (0, 1)...
    pub confidence: f64,
    /// ...at most this many.
    pub ransac_iterations: usize,
    /// A match is the RANSAC's inlier within this many variances in both keyframes.
    pub chi2: f64,
    /// The window of the search through the RANSAC's similarity...
    pub coarse_window: f64,
    /// ...its descriptor distance...
    pub coarse_distance: i64,
    /// ...and the matches it must give.
    pub coarse_matches: usize,
    /// The refinement's Huber loss sits at this chi-square's root; above it a match is dropped.
    pub huber_chi2: f64,
    /// The refinement's first round, and its second when it dropped no match...
    pub refine_iterations: usize,
    /// ...or this many when it did.
    pub refine_more: usize,
    /// The refinement fails when fewer matches survive its first round.
    pub min_kept: usize,
    /// The refined similarity needs this many inliers...
    pub min_refined: usize,
    /// ...and the search through it, in this window...
    pub fine_window: f64,
    /// ...within this distance...
    pub fine_distance: i64,
    /// ...must give this many matches.
    pub fine_matches: usize,
    /// A confirmation's search window...
    pub confirm_window: f64,
    /// ...its distance...
    pub confirm_distance: i64,
    /// ...and the matches it must give.
    pub confirm_matches: usize,
    /// Confirmations that close the loop.
    pub confirmations: usize,
    /// Later keyframes in a row that fail to confirm a loop drop it.
    pub misses: usize,
    /// A later keyframe's refinement must keep more inliers than this...
    pub temporal_refined: usize,
    /// ...and its search through the refined similarity give this many.
    pub temporal_matches: usize,
    /// The loop's points are fused into the corrected keyframes in this window.
    pub fuse_window: f64,
    /// The essential graph links keyframes sharing this many points.
    pub essential_points: usize,
    /// Its Levenberg-Marquardt iterations...
    pub essential_iterations: usize,
    /// ...from this damping.
    pub essential_damping: f64,
    /// Bundle-adjust the whole map after each loop.
    pub global_ba: bool,
    /// Its iterations.
    pub global_iterations: usize,
    /// A merge's welding window: a keyframe and its best this many links, widened to this many
    /// when it holds fewer (`numTemporalKFs`)...
    pub weld_neighbours: usize,
    /// ...each member's best this many links taken in (`numTemporalKFs / 2`)...
    pub weld_widen: usize,
    /// ...at most this many times (`nMaxTries`); 0 widens none.
    pub weld_rounds: usize,
}

impl Default for LoopOptions {
    fn default() -> Self {
        Self {
            enabled: true,
            min_keyframes: 12,
            candidates: 3,
            common_words: 0.8,
            neighbours: 10,
            ratio: 0.9,
            min_matches: 20,
            min_inliers: 15,
            confidence: 0.99,
            ransac_iterations: 300,
            chi2: 9.21,
            coarse_window: 8.0,
            coarse_distance: 75,
            coarse_matches: 50,
            huber_chi2: 10.0,
            refine_iterations: 5,
            refine_more: 10,
            min_kept: 10,
            min_refined: 20,
            fine_window: 5.0,
            fine_distance: 50,
            fine_matches: 80,
            confirm_window: 3.0,
            confirm_distance: 75,
            confirm_matches: 30,
            confirmations: 3,
            misses: 2,
            temporal_refined: 50,
            temporal_matches: 100,
            fuse_window: 4.0,
            essential_points: 100,
            essential_iterations: 20,
            essential_damping: 1e-16,
            global_ba: true,
            global_iterations: 10,
            weld_neighbours: 25,
            weld_widen: 12,
            weld_rounds: 5,
        }
    }
}

/// When a lost track's map is closed (`Options.loss`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LossOptions {
    /// A recently lost track is lost once more than this has passed since its first failed
    /// frame, in seconds.
    pub relocalize_seconds: f64,
    /// A map of `loss.reset_keyframes` keyframes or fewer is lost after this many failed frames in
    /// a row instead.
    pub lost_frames: usize,
    /// A map lost with this many keyframes or fewer is reset rather than kept.
    pub reset_keyframes: usize,
}

impl Default for LossOptions {
    fn default() -> Self {
        Self {
            relocalize_seconds: 3.0,
            lost_frames: 5,
            reset_keyframes: 10,
        }
    }
}

/// The edge pipeline's settings (`slam_it.edge.options.Options`), grouped by the stage that
/// reads them; the defaults are ORB-SLAM2's.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Options {
    /// ORB extraction's settings.
    pub features: FeatureOptions,
    /// The descriptor distances matching takes in every stage.
    pub matching: MatchingOptions,
    /// A stereo pair's row matching, and the close depth of stereo and RGB-D.
    pub stereo: StereoOptions,
    /// Starting the map.
    pub initialization: InitializationOptions,
    /// Placing each frame: the motion model, the local map, the PnP fallback.
    pub tracking: TrackingOptions,
    /// Pose optimisation and local BA.
    pub adjustment: AdjustmentOptions,
    /// The keyframe decision.
    pub keyframes: KeyframeOptions,
    /// Local mapping: points from depth, triangulation, fusion and culling.
    pub mapping: MappingOptions,
    /// The covisibility graph.
    pub covisibility: CovisibilityOptions,
    /// Relocalising a failed frame against the whole map.
    pub relocalization: RelocalizationOptions,
    /// Loop closure.
    pub loops: LoopOptions,
    /// When a lost track's map is closed.
    pub loss: LossOptions,
    /// Every RANSAC's seed.
    pub seed: u64,
}

impl Options {
    /// Whether a pipeline with these settings keeps the keyframe database
    /// (`Options.keeps_keyframe_database`): when relocalising or closing loops, either alone, as
    /// ORB-SLAM always keeps its `KeyFrameDatabase`. A vocabulary is then required, and refused
    /// otherwise; the one statement of the rule.
    ///
    /// # Returns
    ///
    /// `relocalization.enabled` or `loops.enabled`.
    #[must_use]
    pub fn keeps_keyframe_database(&self) -> bool {
        self.relocalization.enabled || self.loops.enabled
    }

    /// The ORB extraction settings.
    ///
    /// # Arguments
    ///
    /// * `initializing` - For a monocular frame before the map exists: `features.init_factor`
    ///   times the features.
    ///
    /// # Returns
    ///
    /// The settings.
    #[must_use]
    pub fn extraction(&self, initializing: bool) -> orb::Options {
        let factor = if initializing {
            self.features.init_factor
        } else {
            1
        };
        orb::Options {
            n_features: (self.features.n_features * factor).min(orb::MAX_FEATURES),
            levels: self.features.levels,
            scale_factor: self.features.scale_factor,
            fast_threshold: self.features.fast_threshold,
            min_fast_threshold: self.features.min_fast_threshold,
        }
    }

    /// The stereo pair's row-matching settings.
    ///
    /// # Returns
    ///
    /// The `stereo` group's settings, but the close depth.
    #[must_use]
    pub fn stereo_matching(&self) -> stereo::MatchOptions {
        stereo::MatchOptions {
            max_distance: self.stereo.max_distance,
            band: self.stereo.band,
            window: self.stereo.window,
            search: self.stereo.search,
            outlier_factor: self.stereo.outlier_factor,
        }
    }
}

/// A setting's value, by its kind: what a command line gives, and what a run records.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Setting {
    /// A flag setting's.
    Flag(bool),
    /// An integer setting's.
    Integer(i64),
    /// A real setting's.
    Real(f64),
}

/// Implements [`Options::settings`] and [`Options::set`] over every group's fields, each with
/// its kind, named `group.field` as `Options.settings()` names them, then `seed`. `settings`
/// destructures each struct without `..`, so a field missing from the list fails to build.
macro_rules! named_settings {
    ($($group:ident: $Group:ident { $($field:ident: $kind:ident),+ $(,)? }),+ $(,)?) => {
        impl Options {
            /// Every setting's name, `group.field` (or `seed`), and value, in the groups' order.
            ///
            /// # Returns
            ///
            /// The settings.
            #[must_use]
            pub fn settings(&self) -> Vec<(&'static str, Setting)> {
                let Self { $($group),+, seed } = *self;
                let mut out = Vec::new();
                $(
                    let $Group { $($field),+ } = $group;
                    $(out.push((
                        concat!(stringify!($group), ".", stringify!($field)),
                        named_settings!(@value $kind $field),
                    ));)+
                )+
                out.push(("seed", named_settings!(@value Integer seed)));
                out
            }

            /// Sets one setting by its name, `group.field` (or `seed`).
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
                    $($(concat!(stringify!($group), ".", stringify!($field)) => {
                        named_settings!(@set $kind self.$group.$field, name, value)
                    })+)+
                    "seed" => named_settings!(@set Integer self.seed, name, value),
                    _ => fail(ErrorKind::InvalidInput, format!("options: no setting {name:?}")),
                }
            }
        }
    };
    (@value Flag $value:ident) => {
        Setting::Flag($value)
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
            Setting::Flag(flag) => fail_setting($name, "expected an integer", flag),
        }
    };
    (@set Real $slot:expr, $name:ident, $value:ident) => {
        match $value {
            Setting::Real(real) => {
                $slot = real;
                Ok(())
            }
            Setting::Integer(integer) => fail_setting($name, "expected a real", integer),
            Setting::Flag(flag) => fail_setting($name, "expected a real", flag),
        }
    };
    (@set Flag $slot:expr, $name:ident, $value:ident) => {
        match $value {
            Setting::Flag(flag) => {
                $slot = flag;
                Ok(())
            }
            Setting::Integer(integer) => fail_setting($name, "expected true or false", integer),
            Setting::Real(real) => fail_setting($name, "expected true or false", real),
        }
    };
}

named_settings!(
    features: FeatureOptions { n_features: Integer, levels: Integer, scale_factor: Real, fast_threshold: Integer, min_fast_threshold: Integer, init_factor: Integer },
    matching: MatchingOptions { th_low: Integer, th_high: Integer },
    stereo: StereoOptions { max_distance: Integer, band: Real, window: Integer, search: Integer, outlier_factor: Real, close_depth: Real },
    initialization: InitializationOptions { min_features: Integer, min_features_depth: Integer, window: Real, ratio: Real, min_matches: Integer, parallax_cos: Real, ba_iterations: Integer, ransac_iterations: Integer },
    tracking: TrackingOptions { motion_window: Real, stereo_motion_window: Real, min_motion_matches: Integer, min_motion_inliers: Integer, local_window: Real, rgbd_local_window: Real, recovery_local_window: Real, recovery_window_frames: Integer, aligned_view_cos: Real, aligned_view_radius: Real, oblique_view_radius: Real, local_map_ratio: Real, min_tracked: Integer, recovery_tracked: Integer, max_frames: Integer, view_cos_limit: Real, min_distance_margin: Real, max_distance_margin: Real, pnp_threshold: Real, pnp_min_inliers: Integer },
    adjustment: AdjustmentOptions { chi2_mono: Real, chi2_stereo: Real, pose_rounds: Integer, iterations: Integer, refine_iterations: Integer, min_keyframes: Integer },
    keyframes: KeyframeOptions { ref_ratio: Real, mono_ref_ratio: Real, first_ref_ratio: Real, ref_min_observations: Integer, ref_min_observations_early: Integer, min_inliers: Integer, close_tracked: Integer, close_untracked: Integer, busy_ref_ratio: Real },
    mapping: MappingOptions { min_new_points: Integer, triangulation_neighbours: Integer, mono_triangulation_neighbours: Integer, epipolar_chi2: Real, min_epipole_distance2: Real, min_baseline_depth: Real, max_parallax_cos: Real, scale_ratio: Real, fuse_window: Real, fuse_neighbours: Integer, mono_fuse_neighbours: Integer, fuse_second_neighbours: Integer, found_ratio: Real, recent_point_keyframes: Integer, point_cull_age: Integer, min_point_observations: Integer, min_point_observations_depth: Integer, redundant_fraction: Real, redundant_observers: Integer, max_queued: Integer },
    covisibility: CovisibilityOptions {
        min_points: Integer,
        local_keyframes: Integer,
        local_neighbours: Integer,
        strong_points: Integer,
    },
    relocalization: RelocalizationOptions { enabled: Flag, ratio: Real, common_words: Real, neighbours: Integer, group_score: Real, min_matches: Integer, inlier_ratio: Real, min_inliers: Integer, confidence: Real, ransac_iterations: Integer, window: Real, narrow_inliers: Integer, narrow_window: Real, narrow_distance: Integer },
    loops: LoopOptions { enabled: Flag, min_keyframes: Integer, candidates: Integer, common_words: Real, neighbours: Integer, ratio: Real, min_matches: Integer, min_inliers: Integer, confidence: Real, ransac_iterations: Integer, chi2: Real, coarse_window: Real, coarse_distance: Integer, coarse_matches: Integer, huber_chi2: Real, refine_iterations: Integer, refine_more: Integer, min_kept: Integer, min_refined: Integer, fine_window: Real, fine_distance: Integer, fine_matches: Integer, confirm_window: Real, confirm_distance: Integer, confirm_matches: Integer, confirmations: Integer, misses: Integer, temporal_refined: Integer, temporal_matches: Integer, fuse_window: Real, essential_points: Integer, essential_iterations: Integer, essential_damping: Real, global_ba: Flag, global_iterations: Integer, weld_neighbours: Integer, weld_widen: Integer, weld_rounds: Integer },
    loss: LossOptions { relocalize_seconds: Real, lost_frames: Integer, reset_keyframes: Integer },
);

/// Checks the descriptor distances and the stereo matching's settings, named as `Options`' fields.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
fn check_matching(options: &Options) -> Result<()> {
    if let Err(error) = stereo::check_options(&options.stereo_matching()) {
        return fail(error.kind, format!("stereo.{}", error.message));
    }
    for (name, value) in [
        ("matching.th_low", options.matching.th_low),
        ("matching.th_high", options.matching.th_high),
        (
            "relocalization.narrow_distance",
            options.relocalization.narrow_distance,
        ),
        ("loops.coarse_distance", options.loops.coarse_distance),
        ("loops.fine_distance", options.loops.fine_distance),
        ("loops.confirm_distance", options.loops.confirm_distance),
    ] {
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
    if let Err(error) = orb::check_options(&options.extraction(false)) {
        return fail(error.kind, format!("features.{}", error.message));
    }
    check_matching(options)?;
    for (name, value) in [
        (
            "initialization.min_features",
            options.initialization.min_features,
        ),
        (
            "initialization.min_features_depth",
            options.initialization.min_features_depth,
        ),
        ("loss.reset_keyframes", options.loss.reset_keyframes),
        ("loops.weld_rounds", options.loops.weld_rounds),
    ] {
        check_range(name, value, 0, MAX_SETTING)?;
    }
    check_range("seed", options.seed, 0, MAX_SETTING as u64)?;
    check_counts(options)?;
    check_reals(options)
}

/// Checks the integer settings that must be at least 1.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
#[allow(
    clippy::too_many_lines,
    reason = "one table of the counts, each named as Python names it"
)]
fn check_counts(options: &Options) -> Result<()> {
    for (name, value) in [
        ("features.n_features", options.features.n_features),
        ("features.levels", options.features.levels),
        ("features.init_factor", options.features.init_factor),
        (
            "initialization.min_matches",
            options.initialization.min_matches,
        ),
        (
            "tracking.min_motion_matches",
            options.tracking.min_motion_matches,
        ),
        ("tracking.min_tracked", options.tracking.min_tracked),
        ("adjustment.pose_rounds", options.adjustment.pose_rounds),
        ("tracking.max_frames", options.tracking.max_frames),
        ("keyframes.min_inliers", options.keyframes.min_inliers),
        ("keyframes.close_tracked", options.keyframes.close_tracked),
        (
            "keyframes.close_untracked",
            options.keyframes.close_untracked,
        ),
        ("mapping.min_new_points", options.mapping.min_new_points),
        ("adjustment.iterations", options.adjustment.iterations),
        (
            "adjustment.refine_iterations",
            options.adjustment.refine_iterations,
        ),
        ("adjustment.min_keyframes", options.adjustment.min_keyframes),
        (
            "mapping.min_point_observations",
            options.mapping.min_point_observations,
        ),
        (
            "mapping.min_point_observations_depth",
            options.mapping.min_point_observations_depth,
        ),
        (
            "mapping.redundant_observers",
            options.mapping.redundant_observers,
        ),
        ("mapping.max_queued", options.mapping.max_queued),
        ("tracking.pnp_min_inliers", options.tracking.pnp_min_inliers),
        ("loss.lost_frames", options.loss.lost_frames),
        (
            "tracking.min_motion_inliers",
            options.tracking.min_motion_inliers,
        ),
        (
            "tracking.recovery_window_frames",
            options.tracking.recovery_window_frames,
        ),
        (
            "tracking.recovery_tracked",
            options.tracking.recovery_tracked,
        ),
        (
            "keyframes.ref_min_observations",
            options.keyframes.ref_min_observations,
        ),
        (
            "keyframes.ref_min_observations_early",
            options.keyframes.ref_min_observations_early,
        ),
        (
            "mapping.triangulation_neighbours",
            options.mapping.triangulation_neighbours,
        ),
        (
            "mapping.mono_triangulation_neighbours",
            options.mapping.mono_triangulation_neighbours,
        ),
        (
            "mapping.recent_point_keyframes",
            options.mapping.recent_point_keyframes,
        ),
        ("mapping.fuse_neighbours", options.mapping.fuse_neighbours),
        (
            "mapping.mono_fuse_neighbours",
            options.mapping.mono_fuse_neighbours,
        ),
        (
            "mapping.fuse_second_neighbours",
            options.mapping.fuse_second_neighbours,
        ),
        ("mapping.point_cull_age", options.mapping.point_cull_age),
        (
            "initialization.ba_iterations",
            options.initialization.ba_iterations,
        ),
        (
            "initialization.ransac_iterations",
            options.initialization.ransac_iterations,
        ),
        ("covisibility.min_points", options.covisibility.min_points),
        (
            "covisibility.local_keyframes",
            options.covisibility.local_keyframes,
        ),
        (
            "covisibility.local_neighbours",
            options.covisibility.local_neighbours,
        ),
        (
            "covisibility.strong_points",
            options.covisibility.strong_points,
        ),
        (
            "relocalization.neighbours",
            options.relocalization.neighbours,
        ),
        (
            "relocalization.min_matches",
            options.relocalization.min_matches,
        ),
        (
            "relocalization.min_inliers",
            options.relocalization.min_inliers,
        ),
        (
            "relocalization.ransac_iterations",
            options.relocalization.ransac_iterations,
        ),
        (
            "relocalization.narrow_inliers",
            options.relocalization.narrow_inliers,
        ),
        ("loops.min_keyframes", options.loops.min_keyframes),
        ("loops.candidates", options.loops.candidates),
        ("loops.neighbours", options.loops.neighbours),
        ("loops.min_matches", options.loops.min_matches),
        ("loops.min_inliers", options.loops.min_inliers),
        ("loops.ransac_iterations", options.loops.ransac_iterations),
        ("loops.coarse_matches", options.loops.coarse_matches),
        ("loops.refine_iterations", options.loops.refine_iterations),
        ("loops.refine_more", options.loops.refine_more),
        ("loops.min_kept", options.loops.min_kept),
        ("loops.min_refined", options.loops.min_refined),
        ("loops.fine_matches", options.loops.fine_matches),
        ("loops.confirm_matches", options.loops.confirm_matches),
        ("loops.confirmations", options.loops.confirmations),
        ("loops.misses", options.loops.misses),
        ("loops.temporal_refined", options.loops.temporal_refined),
        ("loops.temporal_matches", options.loops.temporal_matches),
        ("loops.essential_points", options.loops.essential_points),
        (
            "loops.essential_iterations",
            options.loops.essential_iterations,
        ),
        ("loops.global_iterations", options.loops.global_iterations),
        ("loops.weld_neighbours", options.loops.weld_neighbours),
        ("loops.weld_widen", options.loops.weld_widen),
    ] {
        check_range(name, value, 1, MAX_SETTING)?;
    }
    Ok(())
}

/// Checks the real settings that must be finite and positive.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
fn check_positive(options: &Options) -> Result<()> {
    for (name, value) in [
        ("features.scale_factor", options.features.scale_factor),
        ("stereo.close_depth", options.stereo.close_depth),
        ("initialization.window", options.initialization.window),
        ("tracking.motion_window", options.tracking.motion_window),
        (
            "tracking.stereo_motion_window",
            options.tracking.stereo_motion_window,
        ),
        ("tracking.local_window", options.tracking.local_window),
        (
            "tracking.rgbd_local_window",
            options.tracking.rgbd_local_window,
        ),
        (
            "tracking.recovery_local_window",
            options.tracking.recovery_local_window,
        ),
        ("adjustment.chi2_mono", options.adjustment.chi2_mono),
        ("adjustment.chi2_stereo", options.adjustment.chi2_stereo),
        ("mapping.scale_ratio", options.mapping.scale_ratio),
        ("mapping.fuse_window", options.mapping.fuse_window),
        ("tracking.pnp_threshold", options.tracking.pnp_threshold),
        (
            "tracking.aligned_view_radius",
            options.tracking.aligned_view_radius,
        ),
        (
            "tracking.oblique_view_radius",
            options.tracking.oblique_view_radius,
        ),
        (
            "tracking.max_distance_margin",
            options.tracking.max_distance_margin,
        ),
        ("mapping.epipolar_chi2", options.mapping.epipolar_chi2),
        (
            "mapping.min_epipole_distance2",
            options.mapping.min_epipole_distance2,
        ),
        ("relocalization.window", options.relocalization.window),
        (
            "relocalization.narrow_window",
            options.relocalization.narrow_window,
        ),
        ("loss.relocalize_seconds", options.loss.relocalize_seconds),
        ("loops.chi2", options.loops.chi2),
        ("loops.coarse_window", options.loops.coarse_window),
        ("loops.huber_chi2", options.loops.huber_chi2),
        ("loops.fine_window", options.loops.fine_window),
        ("loops.confirm_window", options.loops.confirm_window),
        ("loops.fuse_window", options.loops.fuse_window),
        ("loops.essential_damping", options.loops.essential_damping),
    ] {
        if !(value.is_finite() && value > 0.0) {
            return fail_setting(name, "must be finite and positive", value);
        }
    }
    Ok(())
}

/// Checks the real settings: those that must be finite and positive, the fractions in (0, 1],
/// and the confidences below 1.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
fn check_reals(options: &Options) -> Result<()> {
    check_positive(options)?;
    for (name, value) in [
        ("initialization.ratio", options.initialization.ratio),
        ("keyframes.ref_ratio", options.keyframes.ref_ratio),
        ("keyframes.mono_ref_ratio", options.keyframes.mono_ref_ratio),
        ("mapping.found_ratio", options.mapping.found_ratio),
        (
            "mapping.redundant_fraction",
            options.mapping.redundant_fraction,
        ),
        ("tracking.view_cos_limit", options.tracking.view_cos_limit),
        ("mapping.max_parallax_cos", options.mapping.max_parallax_cos),
        (
            "initialization.parallax_cos",
            options.initialization.parallax_cos,
        ),
        (
            "tracking.aligned_view_cos",
            options.tracking.aligned_view_cos,
        ),
        ("tracking.local_map_ratio", options.tracking.local_map_ratio),
        (
            "tracking.min_distance_margin",
            options.tracking.min_distance_margin,
        ),
        (
            "keyframes.first_ref_ratio",
            options.keyframes.first_ref_ratio,
        ),
        ("keyframes.busy_ref_ratio", options.keyframes.busy_ref_ratio),
        (
            "mapping.min_baseline_depth",
            options.mapping.min_baseline_depth,
        ),
        ("relocalization.ratio", options.relocalization.ratio),
        (
            "relocalization.common_words",
            options.relocalization.common_words,
        ),
        (
            "relocalization.group_score",
            options.relocalization.group_score,
        ),
        (
            "relocalization.inlier_ratio",
            options.relocalization.inlier_ratio,
        ),
        (
            "relocalization.confidence",
            options.relocalization.confidence,
        ),
        ("loops.common_words", options.loops.common_words),
        ("loops.ratio", options.loops.ratio),
        ("loops.confidence", options.loops.confidence),
    ] {
        if !(value.is_finite() && value > 0.0 && value <= 1.0) {
            return fail_setting(name, "must be in (0, 1]", value);
        }
    }
    for (name, value) in [
        (
            "relocalization.confidence",
            options.relocalization.confidence,
        ),
        ("loops.confidence", options.loops.confidence),
    ] {
        if value >= 1.0 {
            return fail_setting(name, "must be in (0, 1)", value);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Native tests of the settings' checks and extraction.

    use super::*;

    /// Settings are read and set by name, `group.field`; an unknown name, a value of the other
    /// kind, or a count out of range is refused.
    #[test]
    fn settings_by_name() {
        let mut options = Options::default();
        let settings = options.settings();
        assert_eq!(settings.len(), 137);
        assert_eq!(settings[0], ("features.n_features", Setting::Integer(1000)));
        assert_eq!(settings[2], ("features.scale_factor", Setting::Real(1.2)));
        options
            .set("covisibility.local_neighbours", Setting::Integer(7))
            .unwrap();
        options
            .set("stereo.close_depth", Setting::Real(35.5))
            .unwrap();
        assert_eq!(options.covisibility.local_neighbours, 7);
        assert!((options.stereo.close_depth - 35.5).abs() < f64::EPSILON);
        for (name, value) in [
            ("windows", Setting::Integer(1)),
            ("covisibility.local_neighbours", Setting::Real(1.0)),
            ("stereo.close_depth", Setting::Integer(1)),
            ("covisibility.local_neighbours", Setting::Integer(-1)),
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
                covisibility: CovisibilityOptions {
                    local_neighbours: 0,
                    ..CovisibilityOptions::default()
                },
                ..Options::default()
            },
            Options {
                keyframes: KeyframeOptions {
                    ref_ratio: 1.5,
                    ..KeyframeOptions::default()
                },
                ..Options::default()
            },
            Options {
                adjustment: AdjustmentOptions {
                    chi2_mono: f64::NAN,
                    ..AdjustmentOptions::default()
                },
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
                    Setting::Flag(_) => return None, // either value is valid
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

    /// Initialisation's extraction has `features.init_factor` times the features.
    #[test]
    fn initializing_doubles_the_features() {
        let options = Options::default();
        assert_eq!(
            options.extraction(true).n_features,
            2 * options.features.n_features
        );
        assert_eq!(Mode::ALL.map(Mode::name), ["monocular", "stereo", "rgbd"]);
    }
}
