//! The edge pipeline, initialising, tracking each frame and mapping on keyframes: the Rust
//! core's mirror of `slam_it_all.edge.pipeline`, whose docstring states the per-frame loop. It runs
//! in the native program, not across the binding.

use std::time::Instant;

use nalgebra::{Matrix4, Vector3};
use slam_it_all_foundations::error::{ErrorKind, Result, fail};
use slam_it_all_foundations::{orb, se3};

use crate::frames::{Camera, Frame, Tracked, check_setup};
use crate::initialization::{self, InitSummary, Initialization, Outcome, Started};
use crate::map::{self, Map, Removal};
use crate::mapping::{self, MappingSummary};
use crate::options::{Mode, Options};
use crate::tracking::{self, LocalMapSummary, MotionSummary, PnpSummary};

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

    /// The state's name, as `slam_it_all.edge.pipeline.State` spells it.
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

/// How a frame was placed, before the local map refined it, with the codes it is written as
/// (`slam_it_all.edge.pipeline.Placement`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Placement {
    /// It started the map.
    Initialization = 0,
    /// Tracking the last frame, from the constant-velocity prediction.
    Motion = 1,
    /// The fallback: PnP against the local map.
    LocalPnp = 2,
}

impl Placement {
    /// The placement's name, as `slam_it_all.edge.pipeline.Placement` spells it.
    ///
    /// # Returns
    ///
    /// A static string.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Initialization => "initialization",
            Self::Motion => "motion",
            Self::LocalPnp => "local_pnp",
        }
    }
}

/// How the stages went on one frame (`slam_it_all.edge.pipeline.FrameSummary`); each `None`
/// where the stage did not run.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FrameSummary {
    /// How the frame was placed, if it was; a frame placed may still fail the local map.
    pub placed_by: Option<Placement>,
    /// For a frame before the map existed.
    pub init: Option<InitSummary>,
    /// Tracking the last frame.
    pub motion: Option<MotionSummary>,
    /// The PnP fallback, run only when tracking the last frame failed.
    pub pnp: Option<PnpSummary>,
    /// Tracking the local map, run on a placed frame.
    pub local_map: Option<LocalMapSummary>,
    /// Local mapping, on a keyframe after the first.
    pub mapping: Option<MappingSummary>,
}

/// What tracking one frame gave (`slam_it_all.edge.pipeline.FrameResult`).
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
    /// The time tracking it took, in seconds, initialisation included: what ORB-SLAM's
    /// tracking thread does.
    pub track_seconds: f64,
    /// The time mapping it took as a keyframe (`mapping::process`), 0 for any other frame:
    /// what ORB-SLAM's local-mapping thread does.
    pub map_seconds: f64,
    /// How its stages went.
    pub summary: FrameSummary,
}

/// A frame handled, as `Pipeline::initialize` and `Pipeline::track_frame` give it.
#[derive(Debug, Default)]
struct Step {
    /// The tracked frame, if it kept enough inliers.
    tracked: Option<Tracked>,
    /// Whether it became a keyframe.
    keyframe: bool,
    /// The time mapping it took, in seconds, if it did.
    map_seconds: f64,
    /// How its stages went.
    summary: FrameSummary,
}

/// An id as an index (a map's ids are given from 0: `check_map` holds them non-negative).
///
/// # Arguments
///
/// * `id` - The id.
///
/// # Returns
///
/// The index.
fn to_index(id: i64) -> usize {
    usize::try_from(id).expect("map ids are non-negative")
}

/// A life: the frame that made a point or keyframe, the frame that removed it or -1, and why
/// (a [`Removal::code`]) or -1.
pub type Life = [i64; 3];

/// Every point and keyframe a run made, by id, where each last was, and its life
/// (`slam_it_all.edge.pipeline.Lifetimes`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Lifetimes {
    /// Every point, its last position in the world frame.
    pub points: Vec<Vector3<f64>>,
    /// Their lives.
    pub point_lives: Vec<Life>,
    /// Every keyframe's time, ascending.
    pub keyframe_times: Vec<f64>,
    /// Its last `T_cam_world`.
    pub keyframe_poses: Vec<Matrix4<f64>>,
    /// Their lives.
    pub keyframe_lives: Vec<Life>,
}

/// The edge pipeline for one sequence (`slam_it_all.edge.pipeline.Pipeline`).
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
    /// Where each of the reference's keypoints was last matched.
    previous: Vec<[f64; 2]>,
    /// Frames in a row not tracked.
    failures: usize,
    /// Frames since the last recovery, if any.
    since_recovery: Option<usize>,
    /// Frames given so far.
    frames: i64,
    /// Every point and keyframe made so far: those removed where they last were (the others'
    /// entries are placeholders, filled from the map by [`Pipeline::lifetimes`]).
    lives: Lifetimes,
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
        check_setup(&camera, &options)?;
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
            previous: Vec::new(),
            failures: 0,
            since_recovery: None,
            frames: 0,
            lives: Lifetimes::default(),
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
        let step = match self.state {
            State::NotInitialized => self.initialize(frame)?,
            State::Tracking => self.track_frame(frame)?,
            State::Lost => Step::default(),
        };
        self.date();
        let tracked = step.tracked.as_ref();
        Ok(FrameResult {
            timestamp: frame.timestamp,
            state: self.state,
            pose: tracked.map(|t| t.pose),
            tracked: tracked.map_or(0, Tracked::inliers),
            keyframe: step.keyframe,
            track_seconds: start.elapsed().as_secs_f64() - step.map_seconds,
            map_seconds: step.map_seconds,
            summary: step.summary,
        })
    }

    /// Every point and keyframe the run has made, with their lives
    /// (`slam_it_all.edge.pipeline.Pipeline.lifetimes`).
    ///
    /// # Returns
    ///
    /// The points and keyframes by id: those in the map where the map has them, the removed
    /// ones where they last were.
    #[must_use]
    pub fn lifetimes(&self) -> Lifetimes {
        let mut out = self.lives.clone();
        if let Some(m) = &self.map {
            for (&id, xyz) in m.pt_id.iter().zip(&m.pt_xyz) {
                out.points[to_index(id)] = *xyz;
            }
            for ((&id, &time), pose) in m.kf_id.iter().zip(&m.kf_time).zip(&m.kf_pose) {
                out.keyframe_times[to_index(id)] = time;
                out.keyframe_poses[to_index(id)] = *pose;
            }
        }
        out
    }

    /// Dates what this frame made and removed, and empties the map's archive of removals.
    fn date(&mut self) {
        let frame = self.frames;
        self.frames += 1;
        let Some(m) = &mut self.map else {
            return;
        };
        let lives = &mut self.lives;
        let points = to_index(m.next_pt_id);
        lives.point_lives.resize(points, [frame, -1, -1]);
        lives.points.resize(points, Vector3::zeros());
        let keyframes = to_index(m.next_kf_id);
        lives.keyframe_lives.resize(keyframes, [frame, -1, -1]);
        lives.keyframe_times.resize(keyframes, 0.0);
        lives.keyframe_poses.resize(keyframes, Matrix4::identity());
        for ((&id, xyz), &how) in m.gone_pt_id.iter().zip(&m.gone_pt_xyz).zip(&m.gone_pt_how) {
            let id = to_index(id);
            lives.point_lives[id] = [lives.point_lives[id][0], frame, how];
            lives.points[id] = *xyz;
        }
        for ((&id, &time), pose) in m
            .gone_kf_id
            .iter()
            .zip(&m.gone_kf_time)
            .zip(&m.gone_kf_pose)
        {
            let id = to_index(id);
            lives.keyframe_lives[id] = [lives.keyframe_lives[id][0], frame, Removal::Culled.code()];
            lives.keyframe_times[id] = time;
            lives.keyframe_poses[id] = *pose;
        }
        map::clear_gone(m);
    }

    /// Tries to start the map with a frame.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The frame tracked if it started the map, and how the attempt went.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn initialize(&mut self, frame: &Frame) -> Result<Step> {
        let attempt = if self.mode == Mode::Monocular {
            self.initialize_monocular(frame)?
        } else {
            initialization::initialize_from_depth(frame, &self.options)?
        };
        let mut summary = FrameSummary {
            init: Some(attempt.summary),
            ..FrameSummary::default()
        };
        let Some(Started { map, tracked }) = attempt.started else {
            return Ok(Step {
                summary,
                ..Step::default()
            });
        };
        self.map = Some(map);
        self.state = State::Tracking;
        self.accept(frame, &tracked)?;
        summary.placed_by = Some(Placement::Initialization);
        Ok(Step {
            tracked: Some(tracked),
            keyframe: true,
            map_seconds: 0.0,
            summary,
        })
    }

    /// Pairs a monocular frame with the reference, or makes it the reference.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The started map, if any, and how the attempt went.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn initialize_monocular(&mut self, frame: &Frame) -> Result<Initialization> {
        if frame.size() <= self.options.min_init_features {
            self.reference = None; // too few features: ORB-SLAM drops its initializer
            return Ok(Initialization::failed(
                Outcome::TooFewFeatures,
                None,
                Vec::new(),
            ));
        }
        let Some(reference) = &self.reference else {
            self.set_reference(frame);
            return Ok(Initialization::failed(
                Outcome::ReferenceSet,
                None,
                Vec::new(),
            ));
        };
        let matched = initialization::match_for_initialization(
            reference,
            frame,
            &self.previous,
            &self.options,
        )?;
        if matched.len() < self.options.init_min_matches {
            let replaced = Some(matched.len());
            self.set_reference(frame); // the views no longer overlap enough: start again
            return Ok(Initialization::failed(
                Outcome::ReferenceReplaced,
                replaced,
                Vec::new(),
            ));
        }
        // The search follows each matched keypoint (SearchForInitialization's vbPrevMatched).
        for m in &matched {
            self.previous[m.first] = frame.features.keypoints[m.second].xy;
        }
        let pairs: Vec<[usize; 2]> = matched.iter().map(|m| [m.first, m.second]).collect();
        // `None` keeps the reference: a later frame may have the parallax.
        initialization::initialize_monocular(&self.camera, reference, frame, &pairs, &self.options)
    }

    /// Makes a frame the monocular reference, each keypoint searched from its own position.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    fn set_reference(&mut self, frame: &Frame) {
        self.previous = frame.features.keypoints.iter().map(|k| k.xy).collect();
        self.reference = Some(frame.clone());
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
    /// The placed frame, if any, and how it was placed.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn place(&self, m: &Map, frame: &Frame) -> Result<(Option<Tracked>, FrameSummary)> {
        let (Some(last_frame), Some(last_tracked)) = (&self.last_frame, &self.last_tracked) else {
            return fail(
                ErrorKind::InvalidInput,
                "track: no last frame to track from",
            );
        };
        // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM
        // tracks its reference keyframe by bag of words here; the lite pipeline has none.
        let predicted = tracking::predict(&last_tracked.pose, self.velocity.as_ref());
        let motion = tracking::track_last_frame(
            m,
            &self.camera,
            frame,
            last_frame,
            last_tracked,
            &predicted,
            self.mode,
            &self.options,
        )?;
        let mut summary = FrameSummary {
            motion: Some(motion.summary),
            ..FrameSummary::default()
        };
        if motion.tracked.is_some() {
            summary.placed_by = Some(Placement::Motion);
            return Ok((motion.tracked, summary));
        }
        let recovery = tracking::relocalize_locally(m, &self.camera, frame, &self.options)?;
        summary.pnp = Some(recovery.summary);
        if recovery.tracked.is_some() {
            summary.placed_by = Some(Placement::LocalPnp);
        }
        Ok((recovery.tracked, summary))
    }

    /// Tracks a frame against the map; inserts a keyframe if one is needed (and times it).
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The tracked frame, if any, whether it became a keyframe, the time mapping it took, and
    /// how its stages went.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn track_frame(&mut self, frame: &Frame) -> Result<Step> {
        let Some(mut m) = self.map.take() else {
            return fail(ErrorKind::InvalidInput, "track: no map to track against");
        };
        let placed = self.place(&m, frame);
        let outcome =
            placed.and_then(|(placed, summary)| self.refine(&mut m, frame, placed, summary));
        self.map = Some(m);
        let step = outcome?;
        match &step.tracked {
            Some(t) => self.accept(frame, t)?,
            None => self.record_failure(),
        }
        Ok(step)
    }

    /// Tracks the local map from a placed frame, counts its points, and maps it if it needs a
    /// keyframe.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, changed in place.
    /// * `frame` - The frame.
    /// * `placed` - The placed frame, if any.
    /// * `summary` - How it was placed.
    ///
    /// # Returns
    ///
    /// The tracked frame, if any, whether it became a keyframe, the time mapping it took, and
    /// how its stages went.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn refine(
        &mut self,
        m: &mut Map,
        frame: &Frame,
        placed: Option<Tracked>,
        mut summary: FrameSummary,
    ) -> Result<Step> {
        let options = self.options;
        if summary.placed_by == Some(Placement::LocalPnp) {
            self.since_recovery = Some(0);
        } else if let Some(since) = &mut self.since_recovery {
            *since += 1;
        }
        let recent = self.since_recovery.is_some_and(|s| s < options.max_frames);
        let wide = self
            .since_recovery
            .is_some_and(|s| s < options.recovery_window_frames);
        let Some(placed) = placed else {
            return Ok(Step {
                summary,
                ..Step::default()
            });
        };
        let local =
            tracking::track_local_map(m, &self.camera, frame, &placed, self.mode, &options, wide)?;
        summary.local_map = Some(local.summary());
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
            options.recovery_tracked
        } else {
            options.min_tracked
        };
        if local.tracked.inliers() < needed {
            return Ok(Step {
                summary,
                ..Step::default()
            });
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
        let mut map_seconds = 0.0;
        if keyframe {
            let start = Instant::now();
            let mapped = mapping::process(m, &self.camera, frame, &tracked, self.mode, &options)?;
            map_seconds = start.elapsed().as_secs_f64();
            summary.mapping = Some(mapped);
        }
        Ok(Step {
            tracked: Some(tracked),
            keyframe,
            map_seconds,
            summary,
        })
    }
}
