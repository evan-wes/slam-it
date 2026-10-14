//! The edge pipeline, initialising, tracking each frame and mapping on keyframes: the Rust
//! core's mirror of `slam_it.edge.pipeline`, whose docstring states the per-frame loop. It runs
//! in the native program, not across the binding.

use std::time::Instant;

use nalgebra::{Matrix4, Vector3};
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::{orb, se3};

use crate::frames::{Camera, Frame, Tracked, check_setup};
use crate::initialization::{self, InitSummary, Initialization, Outcome, Started};
use crate::map::{self, Map, Removal};
use crate::mapping::{self, MappingSummary};
use crate::options::{Mode, Options};
use crate::tracking::{self, LocalMapSummary, MotionSummary, PnpSummary};

/// Where the pipeline is, with the codes it is written as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// No map: before the first, or after a lost one, until a new one starts.
    NotInitialized = 0,
    /// Tracking frames against the map.
    Tracking = 1,
    /// The track was lost on this frame: its map was kept or reset, and the next frame starts
    /// a new one. Only a frame's result has it.
    Lost = 2,
}

impl State {
    /// Every state, in code order.
    pub const ALL: [Self; 3] = [Self::NotInitialized, Self::Tracking, Self::Lost];

    /// The state's name, as `slam_it.edge.pipeline.State` spells it.
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
/// (`slam_it.edge.pipeline.Placement`).
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
    /// The placement's name, as `slam_it.edge.pipeline.Placement` spells it.
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

/// What became of a keypoint on its frame, with the codes a run records
/// (`slam_it.edge.pipeline.Fate`, whose docstring defines each): the first that applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fate {
    /// The frame's last tracking stage did not match it, and no point was made of it.
    Unmatched = 0,
    /// The frame's last tracking stage matched it, and it was not tracked in the end.
    Rejected = 1,
    /// An inlier of the frame's final pose.
    Tracked = 2,
    /// A point was made of it on its frame.
    Made = 3,
}

/// What became of each of a frame's keypoints, and the point concerned
/// (`slam_it.edge.pipeline.Fates`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Fates {
    /// Each keypoint's fate.
    pub fate: Vec<Fate>,
    /// The run's id of the point concerned (`Lifetimes`' ids): the one made of it, the one it
    /// was tracked to, or the one it was matched to and rejected from; -1 if unmatched.
    pub point: Vec<i64>,
}

impl Fates {
    /// Gives a fate to the keypoints a stage gave a point, over any fate before it
    /// (`slam_it.edge.pipeline._Step.fates`, applied rejected, tracked, then made).
    ///
    /// # Arguments
    ///
    /// * `fate` - The fate.
    /// * `ids` - Each keypoint's point in the map, -1 where none; empty if no stage gave them.
    /// * `base` - The run's id of the map's point 0.
    fn mark(&mut self, fate: Fate, ids: &[i64], base: i64) {
        for ((slot, point), &id) in self.fate.iter_mut().zip(&mut self.point).zip(ids) {
            if id >= 0 {
                *slot = fate;
                *point = base + id;
            }
        }
    }
}

/// The map a lost track closed, as a run records it on the frame it was lost on
/// (`slam_it.edge.pipeline.LostSummary`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LostSummary {
    /// The map's keyframes then.
    pub keyframes: usize,
    /// Its points.
    pub points: usize,
    /// Whether it was reset, having `reset_keyframes` keyframes or fewer, rather than kept.
    pub reset: bool,
}

/// How the stages went on one frame (`slam_it.edge.pipeline.FrameSummary`); each `None`
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
    /// The map closed, on the frame the track was lost on.
    pub lost: Option<LostSummary>,
}

/// What tracking one frame gave (`slam_it.edge.pipeline.FrameResult`).
#[derive(Debug, Clone, PartialEq)]
pub struct FrameResult {
    /// The frame's time.
    pub timestamp: f64,
    /// The frame's: the pipeline's after it, or [`State::Lost`] on the frame the track was lost
    /// on.
    pub state: State,
    /// The map it was tracked in, or lost from; `None` when there was none.
    pub map: Option<i64>,
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
    /// What became of each of its keypoints.
    pub fates: Fates,
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
    /// The last tracking stage's match for each keypoint before refinement; empty if no stage
    /// gave them.
    matched: Vec<i64>,
    /// The point made of each keypoint, -1 where none; empty if none were made.
    made: Vec<i64>,
}

/// A frame placed, as `Pipeline::place` gives it.
struct Placed {
    /// The placed frame, if any.
    tracked: Option<Tracked>,
    /// How it was placed.
    summary: FrameSummary,
    /// The last stage's match for each keypoint before refinement.
    matched: Vec<i64>,
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

/// A count as the run's next id.
///
/// # Arguments
///
/// * `count` - The points or keyframes made so far.
///
/// # Returns
///
/// The id.
fn to_id(count: usize) -> i64 {
    i64::try_from(count).expect("a run's ids fit in i64")
}

/// A life: the frame that made a point or keyframe, the frame that removed it or -1, and why
/// (a [`Removal::code`]) or -1.
pub type Life = [i64; 3];

/// Every point and keyframe a run made, by the run's id (continuing across maps), where each
/// last was, its life and its map (`slam_it.edge.pipeline.Lifetimes`).
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
    /// The map each point was made in.
    pub point_maps: Vec<i64>,
    /// The map each keyframe was made in.
    pub keyframe_maps: Vec<i64>,
}

/// The edge pipeline for one sequence (`slam_it.edge.pipeline.Pipeline`).
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
    /// The current map's id, if there is one.
    map_id: Option<i64>,
    /// The maps started so far: the next one's id.
    maps: i64,
    /// The run's id of the current map's keyframe 0.
    kf_base: i64,
    /// The run's id of the current map's point 0.
    pt_base: i64,
    /// Every point and keyframe made so far, by the run's id: those no longer in the current
    /// map (removed, or in a closed map) where they last were; the others' entries are
    /// placeholders, filled from the map by [`Pipeline::lifetimes`].
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
            map_id: None,
            maps: 0,
            kf_base: 0,
            pt_base: 0,
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

    /// Where the pipeline is: never [`State::Lost`], which a lost frame's result has.
    ///
    /// # Returns
    ///
    /// The state.
    #[must_use]
    pub const fn state(&self) -> State {
        self.state
    }

    /// The current map: none until one starts, and from a lost one to the next.
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
        let (state, map_id) = (self.state, self.map_id);
        let mut fates = Fates {
            fate: vec![Fate::Unmatched; frame.size()],
            point: vec![-1; frame.size()],
        };
        fates.mark(Fate::Rejected, &step.matched, self.pt_base);
        if let Some(t) = &step.tracked {
            fates.mark(Fate::Tracked, &t.points, self.pt_base);
        }
        fates.mark(Fate::Made, &step.made, self.pt_base);
        let mut summary = step.summary;
        if state == State::Lost
            && let Some(lost) = self.map.take()
        {
            summary.lost = Some(self.close(&lost));
        }
        let track_seconds = start.elapsed().as_secs_f64() - step.map_seconds;
        let tracked = step.tracked.as_ref();
        Ok(FrameResult {
            timestamp: frame.timestamp,
            state,
            map: map_id,
            pose: tracked.map(|t| t.pose),
            tracked: tracked.map_or(0, Tracked::inliers),
            keyframe: step.keyframe,
            track_seconds,
            map_seconds: step.map_seconds,
            summary,
            fates,
        })
    }

    /// Every point and keyframe the run has made, with their lives
    /// (`slam_it.edge.pipeline.Pipeline.lifetimes`).
    ///
    /// # Returns
    ///
    /// The points and keyframes by the run's id: those in the current map where it has them,
    /// the others where they last were.
    #[must_use]
    pub fn lifetimes(&self) -> Lifetimes {
        let mut out = self.lives.clone();
        if let Some(m) = &self.map {
            for (&id, xyz) in m.pt_id.iter().zip(&m.pt_xyz) {
                out.points[to_index(self.pt_base + id)] = *xyz;
            }
            for ((&id, &time), pose) in m.kf_id.iter().zip(&m.kf_time).zip(&m.kf_pose) {
                out.keyframe_times[to_index(self.kf_base + id)] = time;
                out.keyframe_poses[to_index(self.kf_base + id)] = *pose;
            }
        }
        out
    }

    /// Dates what this frame made and removed, and empties the map's archive of removals.
    fn date(&mut self) {
        let frame = self.frames;
        self.frames += 1;
        let (Some(m), Some(map_id)) = (&mut self.map, self.map_id) else {
            return;
        };
        let lives = &mut self.lives;
        let points = to_index(self.pt_base + m.next_pt_id);
        lives.point_lives.resize(points, [frame, -1, -1]);
        lives.point_maps.resize(points, map_id);
        lives.points.resize(points, Vector3::zeros());
        let keyframes = to_index(self.kf_base + m.next_kf_id);
        lives.keyframe_lives.resize(keyframes, [frame, -1, -1]);
        lives.keyframe_maps.resize(keyframes, map_id);
        lives.keyframe_times.resize(keyframes, 0.0);
        lives.keyframe_poses.resize(keyframes, Matrix4::identity());
        for ((&id, xyz), &how) in m.gone_pt_id.iter().zip(&m.gone_pt_xyz).zip(&m.gone_pt_how) {
            let id = to_index(self.pt_base + id);
            lives.point_lives[id] = [lives.point_lives[id][0], frame, how];
            lives.points[id] = *xyz;
        }
        for ((&id, &time), pose) in m
            .gone_kf_id
            .iter()
            .zip(&m.gone_kf_time)
            .zip(&m.gone_kf_pose)
        {
            let id = to_index(self.kf_base + id);
            lives.keyframe_lives[id] = [lives.keyframe_lives[id][0], frame, Removal::Culled.code()];
            lives.keyframe_times[id] = time;
            lives.keyframe_poses[id] = *pose;
        }
        map::clear_gone(m);
    }

    /// Closes the map the track was lost in, on this frame: keeps it, or resets it if it has
    /// `reset_keyframes` keyframes or fewer (its lives end, as [`Removal::Reset`]). The
    /// tracking state goes with it, so the next frame starts a new map.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, taken out of the pipeline.
    ///
    /// # Returns
    ///
    /// The closed map's summary.
    fn close(&mut self, m: &Map) -> LostSummary {
        let frame = self.frames - 1; // dated already
        let reset = m.kf_id.len() <= self.options.reset_keyframes;
        let code = Removal::Reset.code();
        let lives = &mut self.lives;
        for (&id, xyz) in m.pt_id.iter().zip(&m.pt_xyz) {
            let id = to_index(self.pt_base + id);
            lives.points[id] = *xyz;
            if reset {
                lives.point_lives[id] = [lives.point_lives[id][0], frame, code];
            }
        }
        for ((&id, &time), pose) in m.kf_id.iter().zip(&m.kf_time).zip(&m.kf_pose) {
            let id = to_index(self.kf_base + id);
            lives.keyframe_times[id] = time;
            lives.keyframe_poses[id] = *pose;
            if reset {
                lives.keyframe_lives[id] = [lives.keyframe_lives[id][0], frame, code];
            }
        }
        self.map_id = None;
        self.state = State::NotInitialized;
        self.last_frame = None;
        self.last_tracked = None;
        self.velocity = None;
        self.reference = None;
        self.previous.clear();
        self.failures = 0;
        self.since_recovery = None;
        LostSummary {
            keyframes: m.kf_id.len(),
            points: m.pt_id.len(),
            reset,
        }
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
        self.map_id = Some(self.maps);
        self.maps += 1;
        self.kf_base = to_id(self.lives.keyframe_lives.len());
        self.pt_base = to_id(self.lives.point_lives.len());
        self.state = State::Tracking;
        self.accept(frame, &tracked)?;
        summary.placed_by = Some(Placement::Initialization);
        Ok(Step {
            made: tracked.points.clone(), // every point the frame observes was made of it
            tracked: Some(tracked),
            keyframe: true,
            map_seconds: 0.0,
            summary,
            matched: Vec::new(),
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
    /// The placed frame, if any, how it was placed, and the last stage's matches.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn place(&self, m: &Map, frame: &Frame) -> Result<Placed> {
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
            return Ok(Placed {
                tracked: motion.tracked,
                summary,
                matched: motion.matched,
            });
        }
        let recovery = tracking::relocalize_locally(m, &self.camera, frame, &self.options)?;
        summary.pnp = Some(recovery.summary);
        if recovery.tracked.is_some() {
            summary.placed_by = Some(Placement::LocalPnp);
        }
        Ok(Placed {
            tracked: recovery.tracked,
            summary,
            matched: recovery.matched,
        })
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
        let outcome = placed.and_then(|placed| self.refine(&mut m, frame, placed));
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
    /// * `placed` - The placed frame, if any, how it was placed, and the last stage's matches.
    ///
    /// # Returns
    ///
    /// The tracked frame, if any, whether it became a keyframe, the time mapping it took, and
    /// how its stages went.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn refine(&mut self, m: &mut Map, frame: &Frame, placed: Placed) -> Result<Step> {
        let Placed {
            tracked: placed,
            mut summary,
            matched,
        } = placed;
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
                matched,
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
                matched: local.matched,
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
        let mut made = Vec::new();
        if keyframe {
            let start = Instant::now();
            let mapped = mapping::process(m, &self.camera, frame, &tracked, self.mode, &options)?;
            map_seconds = start.elapsed().as_secs_f64();
            summary.mapping = Some(mapped.summary);
            made = mapped.made;
        }
        Ok(Step {
            tracked: Some(tracked),
            keyframe,
            map_seconds,
            summary,
            matched: local.matched,
            made,
        })
    }
}
