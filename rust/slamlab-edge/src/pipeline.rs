//! The edge pipeline, initialising, tracking each frame and mapping on keyframes: the Rust
//! core's mirror of `slamlab.edge.pipeline`, whose docstring states the per-frame loop. It runs
//! in the native program, not across the binding.

use std::time::Instant;

use nalgebra::Matrix4;
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::{orb, se3};

use crate::frames::{Camera, Frame, Tracked, check_camera};
use crate::initialization::{self, Started};
use crate::map::{self, Map};
use crate::mapping;
use crate::options::{Mode, Options, check_options};
use crate::tracking;

/// On a recovery's frame and the next, the local map is searched in this many times the window
/// (`SearchLocalPoints`' `th = 5`).
pub const RECOVERY_WIDEN: f64 = 5.0;

/// The frames, counting the recovery's, searched that wide.
pub const RECOVERY_WIDE_FRAMES: usize = 2;

/// Within `max_frames` of a recovery, a frame needs this many inliers.
pub const RECOVERY_TRACKED: usize = 50;

/// Where the pipeline is, with the codes it is written as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No map yet.
    NotInitialized = 0,
    /// Tracking frames against the map.
    Tracking = 1,
    /// The track was lost: the run ends.
    Lost = 2,
}

impl State {
    /// Every state, in code order.
    pub const ALL: [Self; 3] = [Self::NotInitialized, Self::Tracking, Self::Lost];

    /// The state's name, as `slamlab.edge.pipeline.State` spells it.
    ///
    /// # Returns
    ///
    /// A static string.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::NotInitialized => "not_initialized",
            Self::Tracking => "tracking",
            Self::Lost => "lost",
        }
    }
}

/// What tracking one frame gave (`slamlab.edge.pipeline.FrameResult`).
#[derive(Debug, Clone, PartialEq)]
pub struct FrameResult {
    /// The frame's time.
    pub timestamp: f64,
    /// The pipeline's state after the frame.
    pub state: State,
    /// The frame's `T_cam_world`, if it was tracked.
    pub pose: Option<Matrix4<f64>>,
    /// Its keypoints matched to map points.
    pub tracked: usize,
    /// Whether it became a keyframe (or started the map).
    pub keyframe: bool,
    /// The time tracking (and mapping) it took.
    pub seconds: f64,
}

/// The edge pipeline for one sequence (`slamlab.edge.pipeline.Pipeline`).
#[derive(Debug, Clone)]
pub struct Pipeline {
    /// The sensor mode.
    mode: Mode,
    /// The camera.
    camera: Camera,
    /// The settings.
    options: Options,
    /// Where the pipeline is.
    state: State,
    /// The map, once initialised.
    map: Option<Map>,
    /// The last tracked frame.
    last_frame: Option<Frame>,
    /// Its pose and matches.
    last_tracked: Option<Tracked>,
    /// The last motion `T_last_previous`, if the last two frames were tracked.
    velocity: Option<Matrix4<f64>>,
    /// The monocular initialisation's reference frame.
    reference: Option<Frame>,
    /// Frames in a row not tracked.
    failures: usize,
    /// Frames since the last recovery, if any.
    since_recovery: Option<usize>,
}

impl Pipeline {
    /// Starts a pipeline.
    ///
    /// # Arguments
    ///
    /// * `mode` - The sensor mode.
    /// * `camera` - The camera.
    /// * `options` - The settings.
    ///
    /// # Returns
    ///
    /// The pipeline.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for an invalid camera or settings, or a stereo or RGB-D camera without a
    /// baseline.
    pub fn new(mode: Mode, camera: Camera, options: Options) -> Result<Self> {
        check_camera(&camera)?;
        check_options(&options)?;
        if mode != Mode::Monocular && !(camera.baseline > 0.0) {
            return fail(
                ErrorKind::InvalidInput,
                format!("camera.baseline: {} needs a positive baseline", mode.name()),
            );
        }
        Ok(Self {
            mode,
            camera,
            options,
            state: State::NotInitialized,
            map: None,
            last_frame: None,
            last_tracked: None,
            velocity: None,
            reference: None,
            failures: 0,
            since_recovery: None,
        })
    }

    /// The extraction settings the next frame should be built with.
    ///
    /// # Returns
    ///
    /// `init_feature_factor` times the features for a monocular frame before the map exists,
    /// the usual ones otherwise.
    #[must_use]
    pub fn extraction(&self) -> orb::Options {
        self.options
            .extraction(self.mode == Mode::Monocular && self.state == State::NotInitialized)
    }

    /// Where the pipeline is.
    ///
    /// # Returns
    ///
    /// The state.
    #[must_use]
    pub const fn state(&self) -> State {
        self.state
    }

    /// The map, once initialised.
    ///
    /// # Returns
    ///
    /// The map, if any.
    #[must_use]
    pub const fn map(&self) -> Option<&Map> {
        self.map.as_ref()
    }

    /// The last tracked frame's pose and matches.
    ///
    /// # Returns
    ///
    /// They, if any.
    #[must_use]
    pub const fn last_tracked(&self) -> Option<&Tracked> {
        self.last_tracked.as_ref()
    }

    /// Tracks one frame (and maps, if it becomes a keyframe).
    ///
    /// # Arguments
    ///
    /// * `frame` - The next frame, built with [`Pipeline::extraction`]'s settings.
    ///
    /// # Returns
    ///
    /// Its pose, or why there is none.
    ///
    /// # Errors
    ///
    /// A stage's error.
    pub fn track(&mut self, frame: &Frame) -> Result<FrameResult> {
        let start = Instant::now();
        let (tracked, keyframe) = match self.state {
            State::NotInitialized => {
                let tracked = self.initialize(frame)?;
                let started = tracked.is_some();
                (tracked, started)
            }
            State::Tracking => self.track_frame(frame)?,
            State::Lost => (None, false),
        };
        Ok(FrameResult {
            timestamp: frame.timestamp,
            state: self.state,
            pose: tracked.as_ref().map(|t| t.pose),
            tracked: tracked.as_ref().map_or(0, Tracked::inliers),
            keyframe,
            seconds: start.elapsed().as_secs_f64(),
        })
    }

    /// Tries to start the map with a frame.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The frame tracked if it started the map.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn initialize(&mut self, frame: &Frame) -> Result<Option<Tracked>> {
        let started = if self.mode == Mode::Monocular {
            self.initialize_monocular(frame)?
        } else {
            initialization::initialize_from_depth(frame, &self.options)?
        };
        let Some(Started { map, tracked }) = started else {
            return Ok(None);
        };
        self.map = Some(map);
        self.state = State::Tracking;
        self.accept(frame, &tracked)?;
        Ok(Some(tracked))
    }

    /// Pairs a monocular frame with the reference, or makes it the reference.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The started map, if any.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn initialize_monocular(&mut self, frame: &Frame) -> Result<Option<Started>> {
        if frame.size() < self.options.min_init_features {
            self.reference = None; // too few features: ORB-SLAM drops its initializer
            return Ok(None);
        }
        let Some(reference) = &self.reference else {
            self.reference = Some(frame.clone());
            return Ok(None);
        };
        let matched = initialization::match_for_initialization(reference, frame, &self.options)?;
        if matched.len() < self.options.init_min_matches {
            self.reference = Some(frame.clone()); // the views no longer overlap enough: start again
            return Ok(None);
        }
        // `None` keeps the reference: a later frame may have the parallax.
        initialization::initialize_monocular(&self.camera, reference, frame, &self.options)
    }

    /// Makes a tracked frame the last one; the velocity only spans consecutive frames.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    /// * `tracked` - Its pose and matches.
    ///
    /// # Errors
    ///
    /// An error if the last pose cannot be inverted.
    fn accept(&mut self, frame: &Frame, tracked: &Tracked) -> Result<()> {
        self.velocity = match &self.last_tracked {
            Some(last) if self.failures == 0 => Some(tracked.pose * se3::inverse(&last.pose)?),
            _ => None,
        };
        self.last_frame = Some(frame.clone());
        self.last_tracked = Some(tracked.clone());
        self.failures = 0;
        Ok(())
    }

    /// Counts a frame that was not tracked: it ends the velocity; enough in a row lose the
    /// track.
    fn record_failure(&mut self) {
        self.failures += 1;
        self.velocity = None;
        if self.failures >= self.options.lost_frames {
            self.state = State::Lost;
        }
    }

    /// Tracks the last frame, or recovers by PnP.
    ///
    /// # Arguments
    ///
    /// * `m` - The map.
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The placed frame, if any, and whether it recovered.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn place(&self, m: &Map, frame: &Frame) -> Result<(Option<Tracked>, bool)> {
        let (Some(last_frame), Some(last_tracked)) = (&self.last_frame, &self.last_tracked) else {
            return fail(
                ErrorKind::InvalidInput,
                "track: no last frame to track from",
            );
        };
        // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM
        // tracks its reference keyframe by bag of words here; the lite pipeline has none.
        let predicted = tracking::predict(&last_tracked.pose, self.velocity.as_ref());
        let tracked = tracking::track_last_frame(
            m,
            &self.camera,
            frame,
            last_frame,
            last_tracked,
            &predicted,
            self.mode,
            &self.options,
        )?;
        if tracked.is_some() {
            return Ok((tracked, false));
        }
        Ok((
            tracking::relocalize_locally(m, &self.camera, frame, &self.options)?,
            true,
        ))
    }

    /// Tracks a frame against the map; inserts a keyframe if one is needed.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The tracked frame, if any, and whether it became a keyframe.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn track_frame(&mut self, frame: &Frame) -> Result<(Option<Tracked>, bool)> {
        let Some(mut m) = self.map.take() else {
            return fail(ErrorKind::InvalidInput, "track: no map to track against");
        };
        let placed = self.place(&m, frame);
        let outcome =
            placed.and_then(|(placed, recovered)| self.refine(&mut m, frame, placed, recovered));
        self.map = Some(m);
        let (tracked, keyframe) = outcome?;
        match &tracked {
            Some(t) => self.accept(frame, t)?,
            None => self.record_failure(),
        }
        Ok((tracked, keyframe))
    }

    /// Tracks the local map from a placed frame, counts its points, and maps it if it needs a
    /// keyframe.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, changed in place.
    /// * `frame` - The frame.
    /// * `placed` - The placed frame, if any.
    /// * `recovered` - Whether it was placed by the fallback.
    ///
    /// # Returns
    ///
    /// The tracked frame, if any, and whether it became a keyframe.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn refine(
        &mut self,
        m: &mut Map,
        frame: &Frame,
        placed: Option<Tracked>,
        recovered: bool,
    ) -> Result<(Option<Tracked>, bool)> {
        let options = self.options;
        if recovered && placed.is_some() {
            self.since_recovery = Some(0);
        } else if let Some(since) = &mut self.since_recovery {
            *since += 1;
        }
        let recent = self.since_recovery.is_some_and(|s| s < options.max_frames);
        let wide = self
            .since_recovery
            .is_some_and(|s| s < RECOVERY_WIDE_FRAMES);
        let Some(placed) = placed else {
            return Ok((None, false));
        };
        let widen = if wide { RECOVERY_WIDEN } else { 1.0 };
        let local =
            tracking::track_local_map(m, &self.camera, frame, &placed, self.mode, &options, widen)?;
        map::count(
            m,
            &local.visible,
            &vec![1; local.visible.len()],
            &vec![0; local.visible.len()],
        )?;
        map::count(
            m,
            &local.found,
            &vec![0; local.found.len()],
            &vec![1; local.found.len()],
        )?;
        let needed = if recent {
            RECOVERY_TRACKED
        } else {
            options.min_tracked
        };
        if local.tracked.inliers() < needed {
            return Ok((None, false));
        }
        let tracked = local.tracked;
        let held = recent && m.keyframes() > options.max_frames;
        let keyframe = !held
            && mapping::need_keyframe(
                m,
                &self.camera,
                &frame.depth,
                &tracked,
                self.mode,
                &options,
            )?;
        if keyframe {
            mapping::process(m, &self.camera, frame, &tracked, self.mode, &options)?;
        }
        Ok((Some(tracked), keyframe))
    }
}
