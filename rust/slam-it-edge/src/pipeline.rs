//! The edge pipeline, initialising, tracking each frame, mapping on keyframes and closing
//! loops: the Rust core's mirror of `slam_it.edge.pipeline`, whose docstring states the
//! per-frame loop, the steps' own state and the map's owner. It runs in the native program,
//! not across the binding.

use std::time::Instant;

use nalgebra::Matrix4;
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::vocabulary::Vocabulary;
use slam_it_foundations::{orb, se3};

use crate::atlas::{Atlas, Lifetimes, Trajectory};
use crate::database::Database;
use crate::frames::{Camera, Frame, Tracked, check_setup};
use crate::initialization::{self, InitSummary, Initialization, Outcome, Started};
use crate::loops::{self, CorrectionSummary, DetectionSummary, Pending};
use crate::map::{self, Map};
use crate::mapping::{self, MappingSummary};
use crate::options::{Mode, Options};
use crate::relocalization::{self, RelocalizationSummary};
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
    /// Relocalised against the whole map, through its keyframe database.
    Relocalization = 3,
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
            Self::Relocalization => "relocalization",
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
    /// Whether it was reset, having `loss.reset_keyframes` keyframes or fewer, rather than kept.
    pub reset: bool,
}

/// How loop closing went on a keyframe (`slam_it.edge.pipeline.LoopSummary`); its time is the
/// frame's `loop_seconds`.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopSummary {
    /// The search.
    pub detection: DetectionSummary,
    /// The correction, when a loop closed.
    pub correction: Option<CorrectionSummary>,
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
    /// Relocalisation, run only when the fallback failed too.
    pub relocalization: Option<RelocalizationSummary>,
    /// Tracking the local map, run on a placed frame.
    pub local_map: Option<LocalMapSummary>,
    /// Local mapping, on a keyframe after the first.
    pub mapping: Option<MappingSummary>,
    /// Loop closing, on a keyframe after the first while it is on (Python's `loop`).
    pub loop_closing: Option<LoopSummary>,
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
    /// The time loop closing took on it as a keyframe, 0 for any other frame: what ORB-SLAM's
    /// loop-closing thread does.
    pub loop_seconds: f64,
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
    /// Whether the frame lost the track.
    lost: bool,
    /// The time loop closing took, in seconds, if it ran.
    loop_seconds: f64,
}

/// A keyframe mapped, as `Pipeline::map_keyframe` gives it.
#[derive(Debug, Default)]
struct KeyframeMapped {
    /// The time mapping it took, in seconds.
    map_seconds: f64,
    /// The time loop closing took, in seconds.
    loop_seconds: f64,
    /// The point made of each keypoint, -1 where none.
    made: Vec<i64>,
    /// Whether a loop was corrected on it.
    corrected: bool,
}

/// A frame placed, as `place` gives it.
struct Placed {
    /// The placed frame, if any.
    tracked: Option<Tracked>,
    /// How it was placed.
    summary: FrameSummary,
    /// The last stage's match for each keypoint before refinement.
    matched: Vec<i64>,
}

/// Tracking's result on a frame, as `track_step` gives it (`slam_it.edge.pipeline._Tracking`).
#[derive(Debug, Default)]
struct Tracking {
    /// The tracked frame, if it kept enough inliers.
    tracked: Option<Tracked>,
    /// How its stages went.
    summary: FrameSummary,
    /// The last stage's match for each keypoint before refinement.
    matched: Vec<i64>,
    /// The frame's reference keyframe's row, when tracked.
    reference: Option<usize>,
    /// Whether it asks for a keyframe.
    keyframe: bool,
    /// Whether the frame lost the track.
    lost: bool,
}

/// Tracking's own state from frame to frame (`slam_it.edge.pipeline.TrackingState`,
/// ORB-SLAM's `Tracking`'s). A map closed takes it with it.
#[derive(Debug, Clone, Default)]
pub struct TrackingState {
    /// The last tracked frame.
    pub last_frame: Option<Frame>,
    /// Its pose and matches.
    pub last_tracked: Option<Tracked>,
    /// The last motion `T_last_previous`, if the last two frames were tracked.
    pub velocity: Option<Matrix4<f64>>,
    /// The monocular initialisation's reference frame.
    pub reference: Option<Frame>,
    /// Where each of the reference's keypoints was last matched.
    pub previous: Vec<[f64; 2]>,
    /// Frames in a row not tracked.
    pub failures: usize,
    /// Frames since the last recovery, if any.
    pub since_recovery: Option<usize>,
    /// The first failed frame's time, while the track is recently lost.
    pub lost_since: Option<f64>,
}

impl TrackingState {
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
    pub fn accept(&mut self, frame: &Frame, tracked: &Tracked) -> Result<()> {
        self.velocity = match &self.last_tracked {
            Some(last) if self.failures == 0 => Some(tracked.pose * se3::inverse(&last.pose)?),
            _ => None,
        };
        self.last_frame = Some(frame.clone());
        self.last_tracked = Some(tracked.clone());
        self.failures = 0;
        self.lost_since = None;
        Ok(())
    }

    /// Counts a frame that was not tracked, which ends the velocity
    /// (`slam_it.edge.pipeline.TrackingState.fail`).
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    /// * `keyframes` - The map's keyframes.
    /// * `options` - The settings.
    ///
    /// # Returns
    ///
    /// Whether it loses the track: past `loss.relocalize_seconds` since the first failure, or
    /// for a small map or with `relocalization.enabled` off, on the `loss.lost_frames`-th in a
    /// row.
    pub fn record_failure(&mut self, frame: &Frame, keyframes: usize, options: &Options) -> bool {
        self.failures += 1;
        self.velocity = None;
        let since = *self.lost_since.get_or_insert(frame.timestamp);
        if !options.relocalization.enabled || keyframes <= options.loss.reset_keyframes {
            return self.failures >= options.loss.lost_frames;
        }
        frame.timestamp - since > options.loss.relocalize_seconds
    }

    /// Makes a frame the monocular reference, each keypoint searched from its own position.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    pub fn set_reference(&mut self, frame: &Frame) {
        self.previous = frame.features.keypoints.iter().map(|k| k.xy).collect();
        self.reference = Some(frame.clone());
    }

    /// Moves the last frame tracked to its keyframe's pose, after a correction moved it.
    ///
    /// # Arguments
    ///
    /// * `pose` - Its new `T_cam_world`.
    pub const fn move_last(&mut self, pose: &Matrix4<f64>) {
        if let Some(last) = &mut self.last_tracked {
            last.pose = *pose;
        }
    }
}

/// Loop closing's own state from keyframe to keyframe (`slam_it.edge.pipeline.LoopState`,
/// ORB-SLAM3's `LoopClosing`'s). A map closed takes it with it.
#[derive(Debug, Clone, Default)]
pub struct LoopState {
    /// The loop being confirmed in the map, if any.
    pub pending: Option<Pending>,
}

impl LoopState {
    /// The keyframes the loop being confirmed holds, which mapping's culling spares.
    ///
    /// # Returns
    ///
    /// Their ids in the map; none when no loop is being confirmed.
    #[must_use]
    pub fn held(&self) -> Vec<i64> {
        self.pending.as_ref().map(Pending::held).unwrap_or_default()
    }
}

/// The map being tracked and its keyframe database (`slam_it.edge.pipeline.MapOwner`): every
/// change to them goes through here. The steps change the map in place, so the owner runs each
/// step that changes it on its map, then keeps the database in step. In one thread it holds the
/// map between the steps; it is the one place milestone 2B's B3b gives the map one writer at a
/// time and tracking a published view of it (the online-SLAM design, section 12).
#[derive(Debug, Clone)]
pub struct MapOwner {
    /// The vocabulary, when relocalising or closing loops.
    vocabulary: Option<Vocabulary>,
    /// The map, once initialised.
    map: Option<Map>,
    /// The map's keyframe database, when relocalising or closing loops and a map exists.
    database: Option<Database>,
}

impl MapOwner {
    /// An owner with no map yet.
    ///
    /// # Arguments
    ///
    /// * `vocabulary` - The database's vocabulary; `None` when neither relocalising nor closing
    ///   loops, and there is no database.
    ///
    /// # Returns
    ///
    /// The owner.
    #[must_use]
    pub const fn new(vocabulary: Option<Vocabulary>) -> Self {
        Self {
            vocabulary,
            map: None,
            database: None,
        }
    }

    /// The map being tracked: none until one starts, and from a lost one to the next.
    ///
    /// # Returns
    ///
    /// The map, if any.
    #[must_use]
    pub const fn map(&self) -> Option<&Map> {
        self.map.as_ref()
    }

    /// Its keyframe database: none when neither relocalising nor closing loops, or with no map.
    ///
    /// # Returns
    ///
    /// The database, if any.
    #[must_use]
    pub const fn database(&self) -> Option<&Database> {
        self.database.as_ref()
    }

    /// The database's vocabulary, when relocalising or closing loops.
    ///
    /// # Returns
    ///
    /// The vocabulary, if any.
    #[must_use]
    pub const fn vocabulary(&self) -> Option<&Vocabulary> {
        self.vocabulary.as_ref()
    }

    /// Takes a map just started, and starts its database with its keyframes.
    ///
    /// # Arguments
    ///
    /// * `m` - The new map.
    ///
    /// # Errors
    ///
    /// A quantisation's error.
    pub fn start(&mut self, m: Map) -> Result<()> {
        self.map = Some(m);
        if self.vocabulary.is_some() {
            self.database = Some(Database::default());
            return self.sync_database();
        }
        Ok(())
    }

    /// Counts a frame tracked against the local map: the points seen, and found, in it.
    ///
    /// # Arguments
    ///
    /// * `visible` - The points the frame's view held, by the map's ids.
    /// * `found` - The points it tracked.
    ///
    /// # Errors
    ///
    /// `InvalidInput` with no map, or for an id the map does not hold.
    pub fn count(&mut self, visible: &[i64], found: &[i64]) -> Result<()> {
        let Some(m) = &mut self.map else {
            return fail(ErrorKind::InvalidInput, "count: no map");
        };
        map::count(m, visible, &vec![1; visible.len()], &vec![0; visible.len()])?;
        map::count(m, found, &vec![0; found.len()], &vec![1; found.len()])
    }

    /// Runs a mapping step on the map, then brings the database to its keyframes.
    ///
    /// # Arguments
    ///
    /// * `step` - The step, given the map to change in place.
    ///
    /// # Returns
    ///
    /// The step's result.
    ///
    /// # Errors
    ///
    /// The step's, the database's, or `InvalidInput` with no map.
    pub fn apply_mapping<T>(&mut self, step: impl FnOnce(&mut Map) -> Result<T>) -> Result<T> {
        let Some(m) = &mut self.map else {
            return fail(ErrorKind::InvalidInput, "mapping: no map");
        };
        let out = step(m)?;
        self.sync_database()?;
        Ok(out)
    }

    /// Runs a loop's correction on the map, held through the handshake ([`MapOwner::stopped`]),
    /// then brings the database to its keyframes.
    ///
    /// # Arguments
    ///
    /// * `step` - The correction, given the map to change in place.
    ///
    /// # Returns
    ///
    /// The correction's result.
    ///
    /// # Errors
    ///
    /// The correction's, the database's, or `InvalidInput` with no map.
    pub fn apply_correction<T>(&mut self, step: impl FnOnce(&mut Map) -> Result<T>) -> Result<T> {
        let out = self.stopped(step)?;
        self.sync_database()?;
        Ok(out)
    }

    /// The handshake: runs a change on the map held, mapping stopped and global BA aborted, then
    /// both released, as ORB-SLAM's loop closing does (`RequestStop`, `mbStopGBA`, `Release`).
    /// In one thread nothing runs beside the change, so there is nothing to stop or abort: this
    /// is the one place milestone 2B's B3b's threads do it, and its merge (B4) enters it too.
    ///
    /// # Arguments
    ///
    /// * `step` - The change, given the map to change in place.
    ///
    /// # Returns
    ///
    /// The change's result.
    ///
    /// # Errors
    ///
    /// The change's, or `InvalidInput` with no map.
    pub fn stopped<T>(&mut self, step: impl FnOnce(&mut Map) -> Result<T>) -> Result<T> {
        let Some(m) = &mut self.map else {
            return fail(ErrorKind::InvalidInput, "stopped: no map");
        };
        step(m)
    }

    /// Empties the map's archive of removals, once the atlas has dated them.
    pub fn clear_gone(&mut self) {
        if let Some(m) = &mut self.map {
            map::clear_gone(m);
        }
    }

    /// Gives up the map, lost: the database goes with it.
    ///
    /// # Returns
    ///
    /// The map, if there was one.
    pub fn close(&mut self) -> Option<Map> {
        self.database = None;
        self.map.take()
    }

    /// Brings the keyframe database to the map's keyframes: adds the new, erases the culled.
    ///
    /// # Errors
    ///
    /// A quantisation's error.
    fn sync_database(&mut self) -> Result<()> {
        let (Some(vocabulary), Some(m), Some(database)) =
            (&self.vocabulary, &self.map, &mut self.database)
        else {
            return Ok(()); // no keyframe database
        };
        let culled: Vec<i64> = database
            .keyframes
            .keys()
            .copied()
            .filter(|&id| m.keyframe_row(id).is_none())
            .collect();
        for id in culled {
            database.erase(id)?;
        }
        for (k, &id) in m.kf_id.iter().enumerate() {
            if !database.keyframes.contains_key(&id) {
                let rows = m.kf_start[k]..m.kf_start[k + 1];
                database.add(vocabulary, id, &m.kp_descriptor[rows])?;
            }
        }
        Ok(())
    }
}

/// Pairs a monocular frame with the reference, or makes it the reference.
///
/// # Arguments
///
/// * `state` - Tracking's state: its reference and where its keypoints were last matched.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `options` - The settings.
///
/// # Returns
///
/// The started map, if any, and how the attempt went.
///
/// # Errors
///
/// A stage's error.
fn initialize_monocular(
    state: &mut TrackingState,
    camera: &Camera,
    frame: &Frame,
    options: &Options,
) -> Result<Initialization> {
    if frame.size() <= options.initialization.min_features {
        state.reference = None; // too few features: ORB-SLAM drops its initializer
        return Ok(Initialization::failed(
            Outcome::TooFewFeatures,
            None,
            Vec::new(),
        ));
    }
    let Some(reference) = &state.reference else {
        state.set_reference(frame);
        return Ok(Initialization::failed(
            Outcome::ReferenceSet,
            None,
            Vec::new(),
        ));
    };
    let matched =
        initialization::match_for_initialization(reference, frame, &state.previous, options)?;
    if matched.len() < options.initialization.min_matches {
        let replaced = Some(matched.len());
        state.set_reference(frame); // the views no longer overlap enough: start again
        return Ok(Initialization::failed(
            Outcome::ReferenceReplaced,
            replaced,
            Vec::new(),
        ));
    }
    // The search follows each matched keypoint (SearchForInitialization's vbPrevMatched).
    for m in &matched {
        state.previous[m.first] = frame.features.keypoints[m.second].xy;
    }
    let pairs: Vec<[usize; 2]> = matched.iter().map(|m| [m.first, m.second]).collect();
    // `None` keeps the reference: a later frame may have the parallax.
    initialization::initialize_monocular(camera, reference, frame, &pairs, options)
}

/// Tracks the last frame, or recovers by PnP, or relocalises.
///
/// # Arguments
///
/// * `state` - Tracking's state.
/// * `owner` - The map's owner, for its database.
/// * `m` - The map.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
///
/// # Returns
///
/// The placed frame, if any, how it was placed, and the last stage's matches.
///
/// # Errors
///
/// A stage's error.
fn place(
    state: &TrackingState,
    owner: &MapOwner,
    m: &Map,
    camera: &Camera,
    frame: &Frame,
    mode: Mode,
    options: &Options,
) -> Result<Placed> {
    let (Some(last_frame), Some(last_tracked)) = (&state.last_frame, &state.last_tracked) else {
        return fail(
            ErrorKind::InvalidInput,
            "track: no last frame to track from",
        );
    };
    // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM tracks
    // its reference keyframe by bag of words here; the lite pipeline's words only relocalise.
    let predicted = tracking::predict(&last_tracked.pose, state.velocity.as_ref());
    let motion = tracking::track_last_frame(
        m,
        camera,
        frame,
        last_frame,
        last_tracked,
        &predicted,
        mode,
        options,
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
    let recovery = tracking::relocalize_locally(m, camera, frame, &last_tracked.points, options)?;
    summary.pnp = Some(recovery.summary);
    // The database relocalises only with relocalization.enabled: loop closing alone keeps it.
    let relocalizing = owner.database().filter(|_| options.relocalization.enabled);
    let (Some(database), Some(vocabulary), None) =
        (relocalizing, owner.vocabulary(), &recovery.tracked)
    else {
        // Placed by the fallback, or not relocalising.
        if recovery.tracked.is_some() {
            summary.placed_by = Some(Placement::LocalPnp);
        }
        return Ok(Placed {
            tracked: recovery.tracked,
            summary,
            matched: recovery.matched,
        });
    };
    let found = relocalization::relocalize(m, database, vocabulary, camera, frame, options)?;
    if found.tracked.is_some() {
        summary.placed_by = Some(Placement::Relocalization);
    }
    summary.relocalization = Some(found.summary);
    Ok(Placed {
        tracked: found.tracked,
        summary,
        matched: found.matched,
    })
}

/// Tracking's step (`slam_it.edge.pipeline._track_frame`): places a frame against the map,
/// tracks the local map, counts its points through the owner, and asks for a keyframe; a frame
/// not tracked is counted as a failure.
///
/// # Arguments
///
/// * `state` - Tracking's state.
/// * `owner` - The map's owner, given the frame's counts.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
/// * `keyframe_base` - The run's id of the map's keyframe 0, for the summary.
///
/// # Returns
///
/// How tracking went.
///
/// # Errors
///
/// A stage's error, or `InvalidInput` with no map.
fn track_step(
    state: &mut TrackingState,
    owner: &mut MapOwner,
    camera: &Camera,
    frame: &Frame,
    mode: Mode,
    options: &Options,
    keyframe_base: i64,
) -> Result<Tracking> {
    let Some(m) = owner.map() else {
        return fail(ErrorKind::InvalidInput, "track: no map to track against");
    };
    let Placed {
        tracked: placed,
        mut summary,
        matched,
    } = place(state, owner, m, camera, frame, mode, options)?;
    if matches!(
        summary.placed_by,
        Some(Placement::LocalPnp | Placement::Relocalization)
    ) {
        state.since_recovery = Some(0);
    } else if let Some(since) = &mut state.since_recovery {
        *since += 1;
    }
    let recent = state
        .since_recovery
        .is_some_and(|s| s < options.tracking.max_frames);
    let wide = state
        .since_recovery
        .is_some_and(|s| s < options.tracking.recovery_window_frames);
    let Some(placed) = placed else {
        let lost = state.record_failure(frame, m.keyframes(), options);
        return Ok(Tracking {
            summary,
            matched,
            lost,
            ..Tracking::default()
        });
    };
    let local = tracking::track_local_map(m, camera, frame, &placed, mode, options, wide)?;
    summary.local_map = Some(local.summary(keyframe_base));
    owner.count(&local.visible, &local.found)?;
    let Some(m) = owner.map() else {
        return fail(ErrorKind::InvalidInput, "track: no map to track against");
    };
    let needed = if recent {
        options.tracking.recovery_tracked
    } else {
        options.tracking.min_tracked
    };
    if local.tracked.inliers() < needed {
        let lost = state.record_failure(frame, m.keyframes(), options);
        return Ok(Tracking {
            summary,
            matched: local.matched,
            lost,
            ..Tracking::default()
        });
    }
    let held = recent && m.keyframes() > options.tracking.max_frames;
    // Placed with inliers, the frame was matched to points: its reference keyframe exists.
    let Some(reference) = local.reference else {
        return fail(
            ErrorKind::InvalidInput,
            "reference: a tracked frame matched no point of the map",
        );
    };
    let keyframe = !held
        && mapping::need_keyframe(
            m,
            camera,
            &frame.depth,
            &local.tracked,
            reference,
            mode,
            options,
        )?;
    Ok(Tracking {
        tracked: Some(local.tracked),
        summary,
        matched: local.matched,
        reference: Some(reference),
        keyframe,
        lost: false,
    })
}

/// Loop closing's step (`slam_it.edge.pipeline._close_loops`): searches the newest keyframe
/// for a loop, and corrects one that closes, through the owner.
///
/// # Arguments
///
/// * `state` - Loop closing's state.
/// * `owner` - The map's owner.
/// * `camera` - The camera.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
///
/// # Returns
///
/// How it went (`None` when loop closing is off, there is no database, or the keyframe is the
/// map's first), and whether a loop was corrected.
///
/// # Errors
///
/// Detection's, the correction's or the database's.
fn close_loops(
    state: &mut LoopState,
    owner: &mut MapOwner,
    camera: &Camera,
    mode: Mode,
    options: &Options,
) -> Result<(Option<LoopSummary>, bool)> {
    let (Some(m), Some(database)) = (owner.map(), owner.database()) else {
        return Ok((None, false));
    };
    let keyframe = m.kf_id[m.keyframes() - 1];
    if !options.loops.enabled || keyframe == 0 {
        return Ok((None, false));
    }
    let found = loops::detect(
        m,
        database,
        camera,
        keyframe,
        state.pending.as_ref(),
        mode,
        options,
    )?;
    state.pending = found.pending;
    let mut correction = None;
    if let Some(closed) = &found.closed {
        correction =
            Some(owner.apply_correction(|m| loops::correct(m, camera, closed, mode, options))?);
    }
    let corrected = correction.is_some();
    Ok((
        Some(LoopSummary {
            detection: found.summary,
            correction,
        }),
        corrected,
    ))
}

/// The edge pipeline for one sequence (`slam_it.edge.pipeline.Pipeline`): tracking, local
/// mapping and loop closing as steps over state of their own, called in turn.
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
    /// Tracking's own state.
    tracking: TrackingState,
    /// Loop closing's own state.
    closing: LoopState,
    /// The map being tracked and its database.
    owner: MapOwner,
    /// The run's records.
    atlas: Atlas,
}

impl Pipeline {
    /// Starts a pipeline.
    ///
    /// # Arguments
    ///
    /// * `mode` - The sensor mode.
    /// * `camera` - The camera.
    /// * `options` - The settings.
    /// * `vocabulary` - The keyframe database's vocabulary: required with `relocalization.enabled`
    ///   or `loops.enabled` on, refused with both off.
    ///
    /// # Returns
    ///
    /// The pipeline.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for an invalid camera or settings, a stereo or RGB-D camera without a
    /// baseline, or a vocabulary missing while relocalising or closing loops, or given while
    /// doing neither.
    pub fn new(
        mode: Mode,
        camera: Camera,
        options: Options,
        vocabulary: Option<Vocabulary>,
    ) -> Result<Self> {
        check_setup(&camera, &options)?;
        if mode != Mode::Monocular && !(camera.baseline > 0.0) {
            return fail(
                ErrorKind::InvalidInput,
                format!("camera.baseline: {} needs a positive baseline", mode.name()),
            );
        }
        let needed = options.keeps_keyframe_database();
        if needed && vocabulary.is_none() {
            return fail(
                ErrorKind::InvalidInput,
                "vocabulary: relocalization.enabled or loops.enabled needs one",
            );
        }
        if !needed && vocabulary.is_some() {
            return fail(
                ErrorKind::InvalidInput,
                "vocabulary: given, but relocalization.enabled and loops.enabled are both off",
            );
        }
        Ok(Self {
            mode,
            camera,
            options,
            state: State::NotInitialized,
            tracking: TrackingState::default(),
            closing: LoopState::default(),
            owner: MapOwner::new(vocabulary),
            atlas: Atlas::new(options),
        })
    }

    /// The extraction settings the next frame should be built with.
    ///
    /// # Returns
    ///
    /// `features.init_factor` times the features for a monocular frame before the map exists,
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
        self.owner.map()
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
        // Between frames the state is never `Lost`: a lost track closes its map on its frame.
        let step = if self.state == State::Tracking {
            self.track_frame(frame)?
        } else {
            self.initialize(frame)?
        };
        self.atlas.date(self.owner.map());
        self.owner.clear_gone();
        let state = if step.lost { State::Lost } else { self.state };
        let map_id = self.atlas.map_id();
        let base = self.atlas.point_base();
        let mut fates = Fates {
            fate: vec![Fate::Unmatched; frame.size()],
            point: vec![-1; frame.size()],
        };
        fates.mark(Fate::Rejected, &step.matched, base);
        if let Some(t) = &step.tracked {
            fates.mark(Fate::Tracked, &t.points, base);
        }
        fates.mark(Fate::Made, &step.made, base);
        let mut summary = step.summary;
        if step.lost
            && let Some(lost) = self.owner.close()
        {
            summary.lost = Some(self.close(&lost));
        }
        let track_seconds = start.elapsed().as_secs_f64() - step.map_seconds - step.loop_seconds;
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
            loop_seconds: step.loop_seconds,
            summary,
            fates,
        })
    }

    /// Every point and keyframe the run has made, with their lives ([`Atlas::lifetimes`], with
    /// the current map).
    ///
    /// # Returns
    ///
    /// The points and keyframes by the run's id: those in the current map where it has them,
    /// the others where they last were.
    #[must_use]
    pub fn lifetimes(&self) -> Lifetimes {
        self.atlas.lifetimes(self.owner.map())
    }

    /// The run's trajectory so far ([`Atlas::trajectory`], with the current map).
    ///
    /// # Returns
    ///
    /// The frames tracked, but a reset map's.
    #[must_use]
    pub fn trajectory(&self) -> Trajectory {
        self.atlas.trajectory(self.owner.map())
    }

    /// Closes the map the track was lost in, on this frame: keeps it, or resets it if it has
    /// `loss.reset_keyframes` keyframes or fewer ([`Atlas::close`]). Tracking's and loop
    /// closing's state go with it, so the next frame starts a new map.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, given up by the owner.
    ///
    /// # Returns
    ///
    /// The closed map's summary.
    fn close(&mut self, m: &Map) -> LostSummary {
        let reset = m.kf_id.len() <= self.options.loss.reset_keyframes;
        self.atlas.close(m, reset);
        self.tracking = TrackingState::default();
        self.closing = LoopState::default();
        self.state = State::NotInitialized;
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
            initialize_monocular(&mut self.tracking, &self.camera, frame, &self.options)?
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
        let reference = map.kf_id[map.keyframes() - 1];
        self.owner.start(map)?;
        self.atlas.start();
        self.state = State::Tracking;
        if let Some(m) = self.owner.map() {
            self.atlas
                .place_frame(m, frame.timestamp, reference, &tracked.pose)?;
        }
        self.tracking.accept(frame, &tracked)?;
        summary.placed_by = Some(Placement::Initialization);
        Ok(Step {
            made: tracked.points.clone(), // every point the frame observes was made of it
            tracked: Some(tracked),
            keyframe: true,
            map_seconds: 0.0,
            summary,
            matched: Vec::new(),
            lost: false,
            loop_seconds: 0.0,
        })
    }

    /// Runs the steps on a frame: tracks it, and maps and closes loops if it becomes a
    /// keyframe.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    ///
    /// # Returns
    ///
    /// The tracked frame, if any, whether it became a keyframe, the times mapping and loop
    /// closing took, and how its stages went.
    ///
    /// # Errors
    ///
    /// A stage's error.
    fn track_frame(&mut self, frame: &Frame) -> Result<Step> {
        let found = track_step(
            &mut self.tracking,
            &mut self.owner,
            &self.camera,
            frame,
            self.mode,
            &self.options,
            self.atlas.keyframe_base(),
        )?;
        let (Some(tracked), Some(reference)) = (found.tracked, found.reference) else {
            return Ok(Step {
                summary: found.summary,
                matched: found.matched,
                lost: found.lost,
                ..Step::default()
            });
        };
        let mut summary = found.summary;
        let mut mapped = KeyframeMapped::default();
        if found.keyframe {
            mapped = self.map_keyframe(frame, &tracked, &mut summary)?;
        } else if let Some(m) = self.owner.map() {
            self.atlas
                .place_frame(m, frame.timestamp, m.kf_id[reference], &tracked.pose)?;
        }
        self.tracking.accept(frame, &tracked)?;
        if mapped.corrected
            && let Some(m) = self.owner.map()
        {
            // The last frame, the new keyframe's own, moved with it.
            self.tracking.move_last(&m.kf_pose[m.keyframes() - 1]);
        }
        Ok(Step {
            tracked: Some(tracked),
            keyframe: found.keyframe,
            map_seconds: mapped.map_seconds,
            summary,
            matched: found.matched,
            made: mapped.made,
            lost: false,
            loop_seconds: mapped.loop_seconds,
        })
    }

    /// Maps a new keyframe and closes loops on it, timing each: its place in the trajectory is
    /// its own keyframe's.
    ///
    /// # Arguments
    ///
    /// * `frame` - The keyframe's frame.
    /// * `tracked` - Its pose and matches.
    /// * `summary` - The frame's summary, mapping's and loop closing's written in.
    ///
    /// # Returns
    ///
    /// The times, the points made and whether a loop was corrected.
    ///
    /// # Errors
    ///
    /// Mapping's, loop closing's or the database's.
    fn map_keyframe(
        &mut self,
        frame: &Frame,
        tracked: &Tracked,
        summary: &mut FrameSummary,
    ) -> Result<KeyframeMapped> {
        let start = Instant::now();
        let holding = self.closing.held();
        let (camera, mode, options) = (&self.camera, self.mode, &self.options);
        // Local mapping's step: insert the frame as a keyframe and map around it.
        let mapped = self.owner.apply_mapping(|m| {
            mapping::process(m, camera, frame, tracked, mode, options, &holding)
        })?;
        let mut out = KeyframeMapped {
            map_seconds: start.elapsed().as_secs_f64(),
            made: mapped.made,
            ..KeyframeMapped::default()
        };
        summary.mapping = Some(mapped.summary);
        // The frame's reference is its own keyframe, placed where the frame was.
        if let Some(m) = self.owner.map() {
            self.atlas.place_keyframe(m, frame.timestamp, &tracked.pose);
        }
        let searching = Instant::now();
        let (looped, corrected) = close_loops(
            &mut self.closing,
            &mut self.owner,
            &self.camera,
            self.mode,
            &self.options,
        )?;
        out.corrected = corrected;
        if let Some(looped) = looped {
            out.loop_seconds = searching.elapsed().as_secs_f64();
            summary.loop_closing = Some(looped);
        }
        Ok(out)
    }
}
