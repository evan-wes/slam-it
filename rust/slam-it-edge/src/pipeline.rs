//! The edge pipeline, initialising, tracking each frame, mapping on keyframes and closing
//! loops: the Rust core's mirror of `slam_it.edge.pipeline`, whose docstring states the
//! per-frame loop, the steps' own state, the map's owner and the steps in threads of their own.
//! It runs in the native program, not across the binding.
//!
//! Threaded, the map is shared with the view a writer publishes (`Arc<Map>`) and copied when a
//! writer next changes it while tracking still reads that view; tracking holds the view it read
//! for its whole frame. Inline, the view is the map itself.

use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use nalgebra::Matrix4;
use slam_it_foundations::error::{Error, ErrorKind, Result, fail};
use slam_it_foundations::nls::Stop;
use slam_it_foundations::vocabulary::Vocabulary;
use slam_it_foundations::{orb, se3};

use crate::atlas::{Atlas, Lifetimes, Trajectory};
use crate::database::Database;
use crate::frames::{Camera, Frame, Tracked, check_setup};
use crate::initialization::{self, InitSummary, Initialization, Outcome, Started};
use crate::loops::{
    self, CorrectionSummary, Detection, DetectionSummary, GlobalSummary, Loop, Pending,
};
use crate::map::{self, Map};
use crate::mapping::{self, Interrupts, MappingSummary, Schedule};
use crate::options::{Mode, Options};
use crate::relocalization::{self, RelocalizationSummary};
use crate::threads::{Generation, Sender, Turn, Worker, Writer, lock};
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
    /// The time tracking waited for the map, part of `track_seconds`: 0 inline; threaded,
    /// relocalisation's wait for mapping to let the map go, and a lost map's for the threads.
    pub wait_seconds: f64,
    /// Whether it wanted a keyframe (`mapping::need_keyframe`).
    pub asked: bool,
    /// Whether it wanted one and mapping could not take it: stopped, or busy and its queue full
    /// (a monocular frame's queue being none). Never inline.
    pub refused: bool,
    /// The keyframes waiting for mapping when it asked; 0 inline, and when it did not ask.
    pub queue: usize,
    /// How mapping scheduled its keyframe's steps, inline; threaded, it comes later
    /// ([`Completion`]).
    pub schedule: Option<Schedule>,
}

/// What a thread of its own finished after its frame's result was given
/// (`slam_it.edge.pipeline.Completion`, threaded runs): a keyframe's mapping and loop closing on
/// the keyframe's frame; a global BA's on the frame being tracked when it ended, or the last
/// frame when the run's end waited for it. Each part is set only by the step that finished.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Completion {
    /// The frame, counted from 0, one per [`Pipeline::track`].
    pub frame: i64,
    /// A keyframe's mapping: its time, in seconds.
    pub map_seconds: Option<f64>,
    /// How it went.
    pub mapping: Option<MappingSummary>,
    /// How its steps were scheduled.
    pub schedule: Option<Schedule>,
    /// The keyframe's keypoints' points made by mapping, by the run's ids, -1 where none: their
    /// fate is [`Fate::Made`]. Empty but for a keyframe's mapping.
    pub made: Vec<i64>,
    /// Loop closing on a keyframe: its time, in seconds.
    pub loop_seconds: Option<f64>,
    /// How it went.
    pub loop_closing: Option<LoopSummary>,
    /// A global BA in a thread of its own, ended.
    pub global_ba: Option<GlobalSummary>,
    /// Its time, in seconds.
    pub global_ba_seconds: f64,
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
    /// How its handshakes with local mapping went.
    handshake: Handshake,
    /// How mapping scheduled its keyframe's steps, inline.
    schedule: Option<Schedule>,
}

/// How tracking's handshakes with local mapping went on a frame ([`FrameResult`]'s).
#[derive(Debug, Clone, Copy, Default)]
struct Handshake {
    /// The time tracking waited for the map.
    wait_seconds: f64,
    /// Whether it wanted a keyframe.
    asked: bool,
    /// Whether mapping could not take the keyframe it wanted.
    refused: bool,
    /// The keyframes waiting for mapping when it asked.
    queue: usize,
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
    /// How mapping scheduled its steps.
    schedule: Option<Schedule>,
}

/// A frame placed, as `place` gives it.
struct Placed {
    /// The placed frame, if any.
    tracked: Option<Tracked>,
    /// How it was placed.
    summary: FrameSummary,
    /// The last stage's match for each keypoint before refinement.
    matched: Vec<i64>,
    /// The time spent waiting for the map, 0 inline.
    waited: f64,
}

/// Tracking's result on a frame, as `track_step` gives it (`slam_it.edge.pipeline._Tracking`).
#[derive(Debug, Default)]
struct Tracking {
    /// The map tracking read, held for the frame; `None` only when there was none.
    view: Option<Arc<Map>>,
    /// The tracked frame, if it kept enough inliers.
    tracked: Option<Tracked>,
    /// How its stages went.
    summary: FrameSummary,
    /// The last stage's match for each keypoint before refinement.
    matched: Vec<i64>,
    /// The frame's reference keyframe's row, when tracked.
    reference: Option<usize>,
    /// Whether it becomes a keyframe.
    keyframe: bool,
    /// Whether the frame lost the track.
    lost: bool,
    /// How its handshakes with local mapping went.
    handshake: Handshake,
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
    /// The frames since the last that became a keyframe (ORB-SLAM's `mnLastKeyFrameId`).
    pub since_keyframe: usize,
    /// Threaded: the keyframe the last frame was tracked against, by the map's id.
    pub anchor: Option<i64>,
    /// Threaded: the last frame's pose relative to it, `T_last_anchor`.
    pub anchor_relative: Option<Matrix4<f64>>,
    /// Threaded: the corrections of the map seen ([`MapOwner::corrections`]).
    pub corrections: u64,
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

    /// Keeps the last frame's pose relative to the keyframe it was tracked against
    /// (`slam_it.edge.pipeline.TrackingState.anchor_to`, threaded).
    ///
    /// # Arguments
    ///
    /// * `view` - The map tracking read.
    /// * `row` - The keyframe's row in it.
    ///
    /// # Errors
    ///
    /// An error if the keyframe's pose cannot be inverted.
    pub fn anchor_to(&mut self, view: &Map, row: usize) -> Result<()> {
        if let Some(last) = &self.last_tracked {
            self.anchor = Some(view.kf_id[row]);
            self.anchor_relative = Some(last.pose * se3::inverse(&view.kf_pose[row])?);
        }
        Ok(())
    }

    /// Moves the last frame with its anchor keyframe, if the map was corrected since
    /// (`slam_it.edge.pipeline.TrackingState.follow`, threaded): ORB-SLAM's `UpdateLastFrame`
    /// places the last frame by its reference keyframe; here only after a correction, as inline
    /// only the keyframe's own frame moves.
    ///
    /// # Arguments
    ///
    /// * `view` - The map's view now.
    /// * `corrections` - The corrections of the map so far.
    pub fn follow(&mut self, view: &Map, corrections: u64) {
        if corrections == self.corrections {
            return;
        }
        self.corrections = corrections;
        if let (Some(anchor), Some(relative)) = (self.anchor, self.anchor_relative)
            && let Some(row) = view.keyframe_row(anchor)
        {
            self.move_last(&(relative * view.kf_pose[row]));
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

/// The map and its database, which only a writer changes ([`MapOwner`]'s).
#[derive(Debug, Default)]
pub struct Held {
    /// The map, once initialised: shared with the view a writer published, copied on the next
    /// write while tracking still reads it.
    map: Option<Arc<Map>>,
    /// The map's keyframe database, when relocalising or closing loops and a map exists.
    database: Option<Database>,
}

impl Held {
    /// The map.
    ///
    /// # Returns
    ///
    /// The map, if any.
    #[must_use]
    pub fn map(&self) -> Option<&Map> {
        self.map.as_deref()
    }

    /// Its keyframe database.
    ///
    /// # Returns
    ///
    /// The database, if any.
    #[must_use]
    pub const fn database(&self) -> Option<&Database> {
        self.database.as_ref()
    }
}

/// The map being tracked and its keyframe database (`slam_it.edge.pipeline.MapOwner`): every
/// change to them goes through here. The steps change the map in place, so the owner runs each
/// step that changes it on its map, then keeps the database in step. Inline, it holds the map
/// between the steps, and the view is the map. Threaded (a writer given), one thread writes the
/// map at a time ([`Writer`]), tracking reads the view the writers publish, and tracking's counts
/// wait for local mapping to apply them ([`MapOwner::apply_counts`]). The map is shared with a
/// view published (`Arc`), and copied when a writer next changes it while tracking still reads
/// that view: a copy per publish at most, made only when needed.
#[derive(Debug)]
pub struct MapOwner {
    /// The vocabulary, when relocalising or closing loops.
    vocabulary: Option<Vocabulary>,
    /// The handshake between the threads; `None` inline.
    writer: Option<Arc<Writer>>,
    /// The map and its database.
    held: Mutex<Held>,
    /// The map as last published, threaded.
    view: Mutex<Option<Arc<Map>>>,
    /// Tracking's counts waiting for local mapping: the points seen, and found, per frame.
    counts: Mutex<Vec<(Vec<i64>, Vec<i64>)>>,
    /// The corrections applied: loops corrected, and global BAs carried onto the map.
    corrections: AtomicU64,
}

impl MapOwner {
    /// An owner with no map yet.
    ///
    /// # Arguments
    ///
    /// * `vocabulary` - The database's vocabulary; `None` when neither relocalising nor closing
    ///   loops, and there is no database.
    /// * `threaded` - Whether the steps run in threads of their own (a writer made).
    ///
    /// # Returns
    ///
    /// The owner.
    #[must_use]
    pub fn new(vocabulary: Option<Vocabulary>, threaded: bool) -> Self {
        Self {
            vocabulary,
            writer: threaded.then(|| Arc::new(Writer::default())),
            held: Mutex::new(Held::default()),
            view: Mutex::new(None),
            counts: Mutex::new(Vec::new()),
            corrections: AtomicU64::new(0),
        }
    }

    /// The handshake between the threads.
    ///
    /// # Returns
    ///
    /// The writer; `None` inline.
    #[must_use]
    pub fn writer(&self) -> Option<&Writer> {
        self.writer.as_deref()
    }

    /// The map as last published, what tracking reads: threaded, a view the writers published;
    /// inline, the map itself.
    ///
    /// # Returns
    ///
    /// The view, or `None` with no map.
    pub fn view(&self) -> Option<Arc<Map>> {
        if self.writer.is_some() {
            lock(&self.view).clone()
        } else {
            lock(&self.held).map.clone()
        }
    }

    /// Whether a map is being tracked.
    ///
    /// # Returns
    ///
    /// Whether one is.
    pub fn has_map(&self) -> bool {
        if self.writer.is_some() {
            lock(&self.view).is_some()
        } else {
            lock(&self.held).map.is_some()
        }
    }

    /// The map being tracked: none until one starts, and from a lost one to the next.
    /// Threaded, read it once the run is finished.
    ///
    /// # Returns
    ///
    /// The map, if any.
    pub fn map(&self) -> Option<Arc<Map>> {
        lock(&self.held).map.clone()
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

    /// The corrections applied: loops corrected, and global BAs carried onto the map.
    ///
    /// # Returns
    ///
    /// The count.
    pub fn corrections(&self) -> u64 {
        self.corrections.load(Ordering::SeqCst)
    }

    /// Takes a map just started, publishes it, and starts its database with its keyframes.
    ///
    /// # Arguments
    ///
    /// * `m` - The new map.
    ///
    /// # Errors
    ///
    /// A quantisation's error.
    pub fn start(&self, m: Map) -> Result<()> {
        let mut held = lock(&self.held);
        held.map = Some(Arc::new(m));
        if self.vocabulary.is_some() {
            held.database = Some(Database::default());
        }
        self.changed(&mut held)
    }

    /// Counts a frame tracked against the local map: the points seen, and found, in it.
    /// Threaded, the counts wait for local mapping ([`MapOwner::apply_counts`]).
    ///
    /// # Arguments
    ///
    /// * `view` - The map tracking read, given up inline so that the map is changed in place.
    /// * `visible` - The points the frame's view held, by the map's ids.
    /// * `found` - The points it tracked.
    ///
    /// # Returns
    ///
    /// The map tracking goes on with: its view threaded, the map as counted inline.
    ///
    /// # Errors
    ///
    /// `InvalidInput` with no map, inline.
    pub fn count(&self, view: Arc<Map>, visible: &[i64], found: &[i64]) -> Result<Arc<Map>> {
        if self.writer.is_some() {
            lock(&self.counts).push((visible.to_vec(), found.to_vec()));
            return Ok(view);
        }
        drop(view);
        let mut held = lock(&self.held);
        let Some(m) = &mut held.map else {
            return fail(ErrorKind::InvalidInput, "count: no map");
        };
        add_counts(Arc::make_mut(m), visible, found)?;
        Ok(Arc::clone(m))
    }

    /// Applies the counts tracking sent since the last (threaded; local mapping's, in its turn).
    /// A point gone since is skipped.
    ///
    /// # Errors
    ///
    /// `InvalidInput` with no map.
    pub fn apply_counts(&self) -> Result<()> {
        let counts = std::mem::take(&mut *lock(&self.counts));
        if counts.is_empty() {
            return Ok(());
        }
        let mut held = lock(&self.held);
        let Some(m) = &mut held.map else {
            return fail(ErrorKind::InvalidInput, "apply_counts: no map");
        };
        let m = Arc::make_mut(m);
        for (visible, found) in &counts {
            add_counts(m, visible, found)?;
        }
        Ok(())
    }

    /// Runs a mapping step on the map, then brings the database to its keyframes and, threaded,
    /// publishes the map.
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
    pub fn apply_mapping<T>(&self, step: impl FnOnce(&mut Map) -> Result<T>) -> Result<T> {
        let mut held = lock(&self.held);
        let Some(m) = &mut held.map else {
            return fail(ErrorKind::InvalidInput, "mapping: no map");
        };
        let out = step(Arc::make_mut(m))?;
        self.changed(&mut held)?;
        Ok(out)
    }

    /// The handshake: runs a change on the map held, mapping stopped and its local BA aborted,
    /// then mapping released, as ORB-SLAM's loop closing does (`RequestStop`, `Release`; a
    /// global BA running is superseded by the caller's new generation). A change that gives a
    /// value corrected the map: its database is brought to it, it is published, and the
    /// correction counted. Inline nothing runs beside the change, so there is nothing to stop.
    /// Loop corrections, global BAs' results and milestone 2B's B4's merges all enter here.
    ///
    /// # Arguments
    ///
    /// * `step` - The change, given the map to change in place: `Some` if it corrected it.
    ///
    /// # Returns
    ///
    /// The change's result.
    ///
    /// # Errors
    ///
    /// The change's, the database's, or `InvalidInput` with no map.
    pub fn stopped<T>(
        &self,
        step: impl FnOnce(&mut Map) -> Result<Option<T>>,
    ) -> Result<Option<T>> {
        let _stop = self.writer.as_deref().map(Writer::stopped);
        let mut held = lock(&self.held);
        let Some(m) = &mut held.map else {
            return fail(ErrorKind::InvalidInput, "stopped: no map");
        };
        let out = step(Arc::make_mut(m))?;
        if out.is_some() {
            self.changed(&mut held)?;
            self.corrections.fetch_add(1, Ordering::SeqCst);
        }
        Ok(out)
    }

    /// Publishes the map so far for tracking to read, while its writer goes on (threaded): a
    /// copy.
    ///
    /// # Arguments
    ///
    /// * `m` - The map so far.
    pub fn publish(&self, m: &Map) {
        let copy = Arc::new(m.clone());
        *lock(&self.view) = Some(copy);
    }

    /// Runs something with the map and its database held whole, local mapping held off (a
    /// hold, threaded; nothing to hold inline).
    ///
    /// # Arguments
    ///
    /// * `read` - What to run, given the map and its database.
    ///
    /// # Returns
    ///
    /// What it gives.
    pub fn reading<T>(&self, read: impl FnOnce(&Held) -> T) -> T {
        let _hold = self.hold();
        self.read(read)
    }

    /// Holds the map whole, local mapping held off, while the hold lives (threaded; nothing
    /// inline): loop closing searches and then corrects under one hold.
    ///
    /// # Returns
    ///
    /// The hold; `None` inline.
    pub fn hold(&self) -> Option<Turn<'_>> {
        self.writer.as_deref().map(Writer::held)
    }

    /// Runs something with the map and its database, under a hold the caller has taken
    /// ([`MapOwner::hold`]).
    ///
    /// # Arguments
    ///
    /// * `read` - What to run, given the map and its database.
    ///
    /// # Returns
    ///
    /// What it gives.
    pub fn read<T>(&self, read: impl FnOnce(&Held) -> T) -> T {
        read(&lock(&self.held))
    }

    /// Empties the map's archive of removals, once the atlas has dated them.
    pub fn clear_gone(&self) {
        if let Some(m) = &mut lock(&self.held).map
            && !(m.gone_pt_id.is_empty() && m.gone_kf_id.is_empty())
        {
            map::clear_gone(Arc::make_mut(m));
        }
    }

    /// Gives up the map, lost: the database, the view and the counts waiting go with it.
    ///
    /// # Returns
    ///
    /// The map, if there was one.
    pub fn close(&self) -> Option<Arc<Map>> {
        *lock(&self.view) = None;
        lock(&self.counts).clear();
        let mut held = lock(&self.held);
        held.database = None;
        held.map.take()
    }

    /// After a step changed the map: brings the database to it and, threaded, publishes it.
    ///
    /// # Arguments
    ///
    /// * `held` - The map and its database, held.
    ///
    /// # Errors
    ///
    /// A quantisation's error.
    fn changed(&self, held: &mut Held) -> Result<()> {
        sync_database(self.vocabulary.as_ref(), held)?;
        if self.writer.is_some() {
            lock(&self.view).clone_from(&held.map);
        }
        Ok(())
    }
}

/// Brings the keyframe database to the map's keyframes: adds the new, erases the culled.
///
/// # Arguments
///
/// * `vocabulary` - The vocabulary, when relocalising or closing loops.
/// * `held` - The map and its database.
///
/// # Errors
///
/// A quantisation's error.
fn sync_database(vocabulary: Option<&Vocabulary>, held: &mut Held) -> Result<()> {
    let (Some(vocabulary), Some(m), Some(database)) =
        (vocabulary, held.map.as_deref(), &mut held.database)
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

/// Adds a frame's counts to the map: the points seen, and found, in it (a point gone is
/// skipped).
///
/// # Arguments
///
/// * `m` - The map.
/// * `visible` - The points the frame's view held, by the map's ids.
/// * `found` - The points it tracked.
///
/// # Errors
///
/// [`map::count`]'s.
fn add_counts(m: &mut Map, visible: &[i64], found: &[i64]) -> Result<()> {
    map::count(m, visible, &vec![1; visible.len()], &vec![0; visible.len()])?;
    map::count(m, found, &vec![0; found.len()], &vec![1; found.len()])
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

/// Tracks the last frame against the view, or recovers by PnP, or relocalises. Threaded,
/// relocalisation reads the database with mapping held off, against the view published last,
/// which the database matches; the frame goes on with that view.
///
/// # Arguments
///
/// * `state` - Tracking's state.
/// * `owner` - The map's owner, for its database.
/// * `view` - The map tracking reads; threaded, a relocalisation's view replaces it.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
///
/// # Returns
///
/// The placed frame, if any, how it was placed, the last stage's matches, and the time spent
/// waiting for the map.
///
/// # Errors
///
/// A stage's error.
fn place(
    state: &TrackingState,
    owner: &MapOwner,
    view: &mut Arc<Map>,
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
    let m = Arc::clone(view);
    // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM tracks
    // its reference keyframe by bag of words here; the lite pipeline's words only relocalise.
    let predicted = tracking::predict(&last_tracked.pose, state.velocity.as_ref());
    let motion = tracking::track_last_frame(
        &m,
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
            waited: 0.0,
        });
    }
    let recovery = tracking::relocalize_locally(&m, camera, frame, &last_tracked.points, options)?;
    summary.pnp = Some(recovery.summary);
    // The database relocalises only with relocalization.enabled: loop closing alone keeps it.
    let relocalizing = owner
        .vocabulary()
        .filter(|_| options.relocalization.enabled);
    let (Some(vocabulary), None) = (relocalizing, &recovery.tracked) else {
        // Placed by the fallback, or not relocalising.
        if recovery.tracked.is_some() {
            summary.placed_by = Some(Placement::LocalPnp);
        }
        return Ok(Placed {
            tracked: recovery.tracked,
            summary,
            matched: recovery.matched,
            waited: 0.0,
        });
    };
    let asked = Instant::now();
    let mut held_at = asked;
    let found = owner.reading(|held| {
        held_at = Instant::now();
        if owner.writer().is_some()
            && let Some(latest) = owner.view()
        {
            *view = latest;
        }
        held.database()
            .map(|database| {
                relocalization::relocalize(view, database, vocabulary, camera, frame, options)
            })
            .transpose()
    })?;
    let Some(found) = found else {
        return Ok(Placed {
            tracked: None,
            summary,
            matched: recovery.matched,
            waited: 0.0,
        });
    };
    if found.tracked.is_some() {
        summary.placed_by = Some(Placement::Relocalization);
    }
    summary.relocalization = Some(found.summary);
    let waited = if owner.writer().is_some() {
        held_at.duration_since(asked).as_secs_f64()
    } else {
        0.0
    };
    Ok(Placed {
        tracked: found.tracked,
        summary,
        matched: found.matched,
        waited,
    })
}

/// Asks local mapping for a keyframe a frame wants, as ORB-SLAM's `NeedNewKeyFrame` does: none
/// while mapping is stopped or being stopped; taken at once with mapping idle (as inline,
/// always); busy, the local BA is interrupted, and a stereo or RGB-D keyframe is queued while
/// fewer than `mapping.max_queued` wait, a monocular one refused.
///
/// # Arguments
///
/// * `out` - Tracking's result, its keyframe, refusal and queue set.
/// * `threads` - The threads; `None` inline.
/// * `idle` - Whether mapping was idle when the frame asked.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
fn ask_mapping(
    out: &mut Tracking,
    threads: Option<&Threads>,
    idle: bool,
    mode: Mode,
    options: &Options,
) {
    let Some(threads) = threads else {
        out.keyframe = true;
        return;
    };
    out.handshake.queue = threads.waiting();
    if threads.stopping() {
        out.handshake.refused = true;
    } else if idle {
        out.keyframe = true;
    } else {
        threads.interrupt();
        out.keyframe = mode != Mode::Monocular && out.handshake.queue < options.mapping.max_queued;
        out.handshake.refused = !out.keyframe;
    }
}

/// Tracking's step (`slam_it.edge.pipeline._track_frame`): places a frame against the map's
/// view, tracks the local map, counts its points through the owner, and asks for a keyframe; a
/// frame not tracked is counted as a failure.
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
/// * `threads` - The threads, asked whether mapping takes a keyframe; `None` inline.
///
/// # Returns
///
/// How tracking went.
///
/// # Errors
///
/// A stage's error, or `InvalidInput` with no map.
#[allow(
    clippy::too_many_arguments,
    reason = "tracking's state, the map's owner, the frame, the settings and the threads"
)]
fn track_step(
    state: &mut TrackingState,
    owner: &MapOwner,
    camera: &Camera,
    frame: &Frame,
    mode: Mode,
    options: &Options,
    keyframe_base: i64,
    threads: Option<&Threads>,
) -> Result<Tracking> {
    state.since_keyframe += 1;
    let Some(mut view) = owner.view() else {
        return fail(ErrorKind::InvalidInput, "track: no map to track against");
    };
    let Placed {
        tracked: placed,
        mut summary,
        matched,
        waited,
    } = place(state, owner, &mut view, camera, frame, mode, options)?;
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
        let lost = state.record_failure(frame, view.keyframes(), options);
        return Ok(Tracking {
            view: Some(view),
            summary,
            matched,
            lost,
            handshake: Handshake {
                wait_seconds: waited,
                ..Handshake::default()
            },
            ..Tracking::default()
        });
    };
    let local = tracking::track_local_map(&view, camera, frame, &placed, mode, options, wide)?;
    summary.local_map = Some(local.summary(keyframe_base));
    let view = owner.count(view, &local.visible, &local.found)?;
    let needed = if recent {
        options.tracking.recovery_tracked
    } else {
        options.tracking.min_tracked
    };
    if local.tracked.inliers() < needed {
        let lost = state.record_failure(frame, view.keyframes(), options);
        return Ok(Tracking {
            view: Some(view),
            summary,
            matched: local.matched,
            lost,
            handshake: Handshake {
                wait_seconds: waited,
                ..Handshake::default()
            },
            ..Tracking::default()
        });
    }
    // Placed with inliers, the frame was matched to points: its reference keyframe exists.
    let Some(reference) = local.reference else {
        return fail(
            ErrorKind::InvalidInput,
            "reference: a tracked frame matched no point of the map",
        );
    };
    let mut out = Tracking {
        summary,
        matched: local.matched,
        reference: Some(reference),
        handshake: Handshake {
            wait_seconds: waited,
            ..Handshake::default()
        },
        ..Tracking::default()
    };
    if !(recent && view.keyframes() > options.tracking.max_frames) {
        // none just after a recovery
        let idle = threads.is_none_or(Threads::idle);
        out.handshake.asked = mapping::need_keyframe(
            &view,
            camera,
            &frame.depth,
            &local.tracked,
            reference,
            mode,
            options,
            idle,
            Some(state.since_keyframe),
        )?;
        if out.handshake.asked {
            ask_mapping(&mut out, threads, idle, mode, options);
        }
        if out.keyframe {
            state.since_keyframe = 0;
        }
    }
    out.tracked = Some(local.tracked);
    out.view = Some(view);
    Ok(out)
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
    owner: &MapOwner,
    camera: &Camera,
    mode: Mode,
    options: &Options,
) -> Result<(Option<LoopSummary>, bool)> {
    let found = owner.read(|held| -> Result<Option<Detection>> {
        let (Some(m), Some(database)) = (held.map(), held.database()) else {
            return Ok(None);
        };
        let keyframe = m.kf_id[m.keyframes() - 1];
        if !options.loops.enabled || keyframe == 0 {
            return Ok(None);
        }
        loops::detect(
            m,
            database,
            camera,
            keyframe,
            state.pending.as_ref(),
            mode,
            options,
        )
        .map(Some)
    })?;
    let Some(found) = found else {
        return Ok((None, false));
    };
    state.pending = found.pending;
    let correction = match &found.closed {
        Some(closed) => {
            owner.stopped(|m| loops::correct(m, camera, closed, mode, options).map(Some))?
        }
        None => None,
    };
    let corrected = correction.is_some();
    Ok((
        Some(LoopSummary {
            detection: found.summary,
            correction,
        }),
        corrected,
    ))
}

/// What the threads share with tracking: the map's owner, the run's records and loop closing's
/// state.
#[derive(Debug)]
struct Shared {
    /// The map being tracked and its database.
    owner: MapOwner,
    /// The run's records.
    atlas: Mutex<Atlas>,
    /// Loop closing's own state.
    closing: Mutex<LoopState>,
}

/// A keyframe tracking asked local mapping for (`slam_it.edge.pipeline._Request`).
struct Request {
    /// Its frame's number.
    index: i64,
    /// The frame.
    frame: Frame,
    /// Its pose and matches.
    tracked: Tracked,
}

/// A keyframe mapped, given to loop closing: its frame's number and its id.
struct Searched {
    /// Its frame's number.
    index: i64,
    /// Its id in the map.
    keyframe: i64,
}

/// What local mapping, loop closing and the global BAs share among themselves.
#[derive(Debug)]
struct Steps {
    /// The sensor mode.
    mode: Mode,
    /// The camera.
    camera: Camera,
    /// The settings.
    options: Options,
    /// The settings, global BA left out of a loop's correction.
    without_global_ba: Options,
    /// The global BAs' generations.
    generation: Generation,
    /// The completions not yet taken.
    completions: Mutex<Vec<Completion>>,
    /// The global BAs' threads started and not yet joined.
    global: Mutex<Vec<JoinHandle<()>>>,
    /// A global BA's error, if one failed.
    error: Mutex<Option<Error>>,
}

impl Steps {
    /// Keeps a completion for the run.
    ///
    /// # Arguments
    ///
    /// * `completion` - The completion.
    fn complete(&self, completion: Completion) {
        lock(&self.completions).push(completion);
    }

    /// Keeps a global BA's error, for tracking's thread.
    ///
    /// # Arguments
    ///
    /// * `error` - The error.
    fn keep(&self, error: Error) {
        lock(&self.error).get_or_insert(error);
    }
}

/// What local mapping's thread is asked between its steps (`slam_it.edge.mapping.Interrupts`).
struct Asked<'a> {
    /// Local mapping's own queue.
    queue: &'a Sender<Request>,
    /// The map's owner, its writer and its view.
    owner: &'a MapOwner,
    /// The writer's abort flag.
    abort: Stop,
}

impl Interrupts for Asked<'_> {
    fn waiting(&self) -> bool {
        self.queue.waiting() > 0
    }

    fn stopping(&self) -> bool {
        self.owner.writer().is_some_and(Writer::stopping)
    }

    fn abort(&self) -> &Stop {
        &self.abort
    }

    fn publish(&self, m: &Map) {
        self.owner.publish(m);
    }
}

/// A frame as tracked, its points no longer in the map dropped (`ProcessNewKeyFrame`;
/// `slam_it.edge.pipeline._still_mapped`): threaded, mapping may cull or fuse a point a keyframe
/// tracked before it inserts the keyframe, and ORB-SLAM skips the points gone bad.
///
/// # Arguments
///
/// * `tracked` - The frame's pose and matches.
/// * `m` - The map.
///
/// # Returns
///
/// Its pose, and its matches still in the map.
fn still_mapped(tracked: &Tracked, m: &Map) -> Tracked {
    let mut out = tracked.clone();
    for id in &mut out.points {
        if *id >= 0 && m.point_row(*id).is_none() {
            *id = -1;
        }
    }
    out
}

/// Local mapping's thread: maps a keyframe, the one writer while it does, interrupted between
/// its steps; then gives it to loop closing.
///
/// # Arguments
///
/// * `shared` - What the threads share with tracking.
/// * `steps` - What they share among themselves.
/// * `request` - The keyframe.
/// * `queue` - Local mapping's own queue.
/// * `searching` - Loop closing's queue.
///
/// # Errors
///
/// A step's.
fn map_keyframe(
    shared: &Shared,
    steps: &Steps,
    request: &Request,
    queue: &Sender<Request>,
    searching: &Sender<Searched>,
) -> Result<()> {
    let owner = &shared.owner;
    let Some(writer) = owner.writer() else {
        return fail(ErrorKind::InvalidInput, "mapping: no writer");
    };
    let mut done = Completion {
        frame: request.index,
        ..Completion::default()
    };
    let keyframe = {
        let _turn = writer.mapping();
        writer.resume();
        owner.apply_counts()?;
        let start = Instant::now();
        let asked = Asked {
            queue,
            owner,
            abort: writer.abort(),
        };
        let holding = lock(&shared.closing).held();
        let (camera, mode, options) = (&steps.camera, steps.mode, &steps.options);
        let (mapped, keyframe, base) = owner.apply_mapping(|m| {
            let tracked = still_mapped(&request.tracked, m);
            let mapped = mapping::process(
                m,
                camera,
                &request.frame,
                &tracked,
                mode,
                options,
                &holding,
                Some(&asked),
            )?;
            let mut atlas = lock(&shared.atlas);
            atlas.resolve(request.index, m);
            atlas.record(Some(m));
            map::clear_gone(m);
            Ok((mapped, m.kf_id[m.keyframes() - 1], atlas.point_base()))
        })?;
        done.map_seconds = Some(start.elapsed().as_secs_f64());
        done.made = mapped
            .made
            .iter()
            .map(|&id| if id >= 0 { base + id } else { -1 })
            .collect();
        done.mapping = Some(mapped.summary);
        done.schedule = Some(mapped.schedule);
        keyframe
    };
    steps.complete(done);
    searching.put(Searched {
        index: request.index,
        keyframe,
    });
    Ok(())
}

/// A global BA started by a loop's correction.
#[derive(Default)]
struct GlobalStart {
    /// The map as the correction left it, which global BA solves (`loops.global_ba`).
    copy: Option<Map>,
    /// Its generation.
    generation: u64,
    /// Its stop flag, set once a newer generation starts.
    superseded: Stop,
}

/// Loop closing's thread: searches a keyframe for a loop with mapping held off, and corrects
/// one that closes with mapping stopped, without global BA, which then solves a copy of the map
/// in a thread of its own.
///
/// # Arguments
///
/// * `shared` - What the threads share with tracking.
/// * `steps` - What they share among themselves, the global BAs' threads given this one's.
/// * `item` - The keyframe.
///
/// # Errors
///
/// Detection's, the correction's or the database's.
fn search_loops(shared: &Arc<Shared>, steps: &Arc<Steps>, item: &Searched) -> Result<()> {
    if !steps.options.loops.enabled || item.keyframe == 0 {
        return Ok(());
    }
    let start = Instant::now();
    let owner = &shared.owner;
    let mut started = GlobalStart::default();
    let summary = {
        let _hold = owner.hold(); // mapping held off through the search and the correction
        let found = owner.read(|held| -> Result<Option<Detection>> {
            let (Some(m), Some(database)) = (held.map(), held.database()) else {
                return Ok(None); // no keyframe database
            };
            if m.keyframe_row(item.keyframe).is_none() {
                return Ok(None); // culled since it was mapped
            }
            let closing = lock(&shared.closing);
            let found = loops::detect(
                m,
                database,
                &steps.camera,
                item.keyframe,
                closing.pending.as_ref(),
                steps.mode,
                &steps.options,
            )?;
            Ok(Some(found))
        })?;
        let Some(found) = found else {
            return Ok(());
        };
        lock(&shared.closing).pending = found.pending;
        let correction = match &found.closed {
            Some(closed) => correct_loop(shared, steps, closed, &mut started)?,
            None => None,
        };
        LoopSummary {
            detection: found.summary,
            correction,
        }
    };
    steps.complete(Completion {
        frame: item.index,
        loop_seconds: Some(start.elapsed().as_secs_f64()),
        loop_closing: Some(summary),
        ..Completion::default()
    });
    if let Some(copy) = started.copy {
        let (shared, theirs) = (Arc::clone(shared), Arc::clone(steps));
        let (generation, superseded) = (started.generation, started.superseded);
        let thread = thread::Builder::new()
            .name("global BA".to_owned())
            .spawn(move || {
                if let Err(error) = global_ba(&shared, &theirs, copy, generation, &superseded) {
                    theirs.keep(error);
                }
            })
            .map_err(|error| Error::new(ErrorKind::InvalidInput, format!("global BA: {error}")))?;
        lock(&steps.global).push(thread);
    }
    Ok(())
}

/// Corrects a loop with mapping stopped, without global BA, and takes the copy global BA
/// solves.
///
/// # Arguments
///
/// * `shared` - What the threads share with tracking.
/// * `steps` - What they share among themselves.
/// * `closed` - The loop.
/// * `started` - The global BA it starts, set.
///
/// # Returns
///
/// The correction.
///
/// # Errors
///
/// The correction's or the database's.
fn correct_loop(
    shared: &Shared,
    steps: &Steps,
    closed: &Loop,
    started: &mut GlobalStart,
) -> Result<Option<CorrectionSummary>> {
    shared.owner.stopped(|m| {
        started.generation = steps.generation.next();
        started.superseded = steps.generation.stop();
        let corrected = loops::correct(
            m,
            &steps.camera,
            closed,
            steps.mode,
            &steps.without_global_ba,
        )?;
        lock(&shared.atlas).record(Some(m));
        map::clear_gone(m);
        if steps.options.loops.global_ba {
            started.copy = Some(m.clone());
        }
        Ok(Some(corrected))
    })
}

/// A global BA's thread: solves a copy, then carries the result onto the map as it has become
/// if no newer loop superseded it, mapping stopped.
///
/// # Arguments
///
/// * `shared` - What the threads share with tracking.
/// * `steps` - What they share among themselves.
/// * `copy` - The map as the loop's correction left it.
/// * `generation` - The global BA's generation.
/// * `superseded` - Its stop flag, set once a newer generation starts.
///
/// # Errors
///
/// The solve's, or the propagation's.
fn global_ba(
    shared: &Shared,
    steps: &Steps,
    mut copy: Map,
    generation: u64,
    superseded: &Stop,
) -> Result<()> {
    let start = Instant::now();
    let solve = loops::global_bundle_adjust(&mut copy, &steps.camera, &steps.options, superseded)?;
    let mut frame = 0;
    let propagation = shared.owner.stopped(|m| {
        let mut atlas = lock(&shared.atlas);
        frame = atlas.frame();
        if !steps.generation.current(generation) {
            return Ok(None);
        }
        let moved = loops::propagate_global_ba(m, &copy)?;
        atlas.record(Some(m));
        map::clear_gone(m);
        Ok(Some(moved))
    })?;
    steps.complete(Completion {
        frame,
        global_ba: Some(GlobalSummary {
            generation,
            applied: propagation.is_some(),
            solve,
            propagation,
        }),
        global_ba_seconds: start.elapsed().as_secs_f64(),
        ..Completion::default()
    });
    Ok(())
}

/// Local mapping, loop closing and the global BAs, each in a thread of its own
/// (`slam_it.edge.pipeline._Threads`): tracking asks it whether mapping takes a keyframe, and
/// gives it the keyframes; the threads give back [`Completion`] records.
struct Threads {
    /// What the threads share with tracking.
    shared: Arc<Shared>,
    /// What they share among themselves.
    steps: Arc<Steps>,
    /// Local mapping's thread, ended first: it gives loop closing its keyframes.
    mapping: Worker<Request>,
    /// Loop closing's thread.
    loop_closing: Worker<Searched>,
}

impl Threads {
    /// Starts the threads.
    ///
    /// # Arguments
    ///
    /// * `shared` - What they share with tracking.
    /// * `mode` - The sensor mode.
    /// * `camera` - The camera.
    /// * `options` - The settings.
    ///
    /// # Returns
    ///
    /// The threads.
    fn start(shared: &Arc<Shared>, mode: Mode, camera: &Camera, options: &Options) -> Self {
        let mut without_global_ba = *options;
        without_global_ba.loops.global_ba = false; // it runs in a thread of its own
        let steps = Arc::new(Steps {
            mode,
            camera: camera.clone(),
            options: *options,
            without_global_ba,
            generation: Generation::default(),
            completions: Mutex::new(Vec::new()),
            global: Mutex::new(Vec::new()),
            error: Mutex::new(None),
        });
        let (theirs, their_steps) = (Arc::clone(shared), Arc::clone(&steps));
        let loop_closing = Worker::new(
            "loop closing",
            move |item: Searched, _: &Sender<Searched>| search_loops(&theirs, &their_steps, &item),
        );
        let searching = loop_closing.sender();
        let (theirs, their_steps) = (Arc::clone(shared), Arc::clone(&steps));
        let mapping = Worker::new(
            "local mapping",
            move |request: Request, queue: &Sender<Request>| {
                map_keyframe(&theirs, &their_steps, &request, queue, &searching)
            },
        );
        Self {
            shared: Arc::clone(shared),
            steps,
            mapping,
            loop_closing,
        }
    }

    /// The handshake between the threads.
    ///
    /// # Returns
    ///
    /// The owner's writer, `None` only inline.
    fn writer(&self) -> Option<&Writer> {
        self.shared.owner.writer()
    }

    /// Whether local mapping takes a keyframe at once (`AcceptKeyFrames`): none in hand, none
    /// waiting.
    ///
    /// # Returns
    ///
    /// Whether it is idle.
    fn idle(&self) -> bool {
        self.mapping.unfinished() == 0
    }

    /// Whether local mapping is stopped or being stopped.
    ///
    /// # Returns
    ///
    /// Whether it is.
    fn stopping(&self) -> bool {
        self.writer().is_some_and(Writer::stopping)
    }

    /// The keyframes waiting for local mapping (`KeyframesInQueue`).
    ///
    /// # Returns
    ///
    /// The count.
    fn waiting(&self) -> usize {
        self.mapping.waiting()
    }

    /// Interrupts local mapping's local BA (`InterruptBA`).
    fn interrupt(&self) {
        if let Some(writer) = self.writer() {
            writer.interrupt();
        }
    }

    /// Queues a keyframe for local mapping, aborting its local BA (`InsertKeyFrame`).
    ///
    /// # Arguments
    ///
    /// * `request` - The keyframe.
    fn request(&self, request: Request) {
        self.mapping.put(request);
        self.interrupt();
    }

    /// The completions since the last call, in the order they came.
    ///
    /// # Returns
    ///
    /// The completions.
    fn completed(&self) -> Vec<Completion> {
        std::mem::take(&mut *lock(&self.steps.completions))
    }

    /// The error a thread failed with, if one did.
    ///
    /// # Errors
    ///
    /// The error.
    fn check(&self) -> Result<()> {
        self.mapping.check()?;
        self.loop_closing.check()?;
        lock(&self.steps.error).clone().map_or(Ok(()), Err)
    }

    /// Waits for local mapping and loop closing to handle every keyframe given, and for the
    /// global BAs.
    ///
    /// # Arguments
    ///
    /// * `apply` - Whether a global BA still running applies its result when it ends (the run's
    ///   end); if not (a map lost), it is superseded.
    ///
    /// # Errors
    ///
    /// The error a thread failed with.
    fn settle(&self, apply: bool) -> Result<()> {
        self.mapping.drain();
        self.loop_closing.drain();
        if !apply {
            self.steps.generation.next();
        }
        self.join_global();
        self.check()
    }

    /// Waits for the global BAs' threads started so far.
    fn join_global(&self) {
        let running = std::mem::take(&mut *lock(&self.steps.global));
        for thread in running {
            // A global BA that panicked has no result to carry; its error is the panic's own.
            let _ = thread.join();
        }
    }

    /// Ends local mapping and loop closing, after the keyframes given ([`Threads::settle`]
    /// first).
    fn stop(&mut self) {
        self.mapping.stop();
        self.loop_closing.stop();
    }
}

impl fmt::Debug for Threads {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Threads").finish_non_exhaustive()
    }
}

impl Drop for Threads {
    fn drop(&mut self) {
        self.steps.generation.next();
        self.stop();
        self.join_global();
    }
}

/// The edge pipeline for one sequence (`slam_it.edge.pipeline.Pipeline`): tracking, local
/// mapping and loop closing as steps over state of their own, called in turn, or each in a
/// thread of its own (threaded).
#[derive(Debug)]
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
    /// The map's owner, the run's records and loop closing's state, shared with the threads.
    shared: Arc<Shared>,
    /// The threads, threaded.
    threads: Option<Threads>,
    /// Whether the run is finished ([`Pipeline::finish`]).
    finished: bool,
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
    /// * `threaded` - Whether the steps run in threads of their own; inline, a run repeats
    ///   exactly.
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
        threaded: bool,
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
        let shared = Arc::new(Shared {
            owner: MapOwner::new(vocabulary, threaded),
            atlas: Mutex::new(Atlas::new(options)),
            closing: Mutex::new(LoopState::default()),
        });
        let threads = threaded.then(|| Threads::start(&shared, mode, &camera, &options));
        Ok(Self {
            mode,
            camera,
            options,
            state: State::NotInitialized,
            tracking: TrackingState::default(),
            shared,
            threads,
            finished: false,
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

    /// The current map: none until one starts, and from a lost one to the next. Threaded, read
    /// it once the run is finished.
    ///
    /// # Returns
    ///
    /// The map, if any.
    #[must_use]
    pub fn map(&self) -> Option<Arc<Map>> {
        self.shared.owner.map()
    }

    /// The handshake between the threads, threaded: a test holds a stop through it.
    ///
    /// # Returns
    ///
    /// The writer, shared; `None` inline.
    #[must_use]
    pub fn writer(&self) -> Option<Arc<Writer>> {
        self.shared.owner.writer.clone()
    }

    /// Tracks one frame (and maps, if it becomes a keyframe; threaded, local mapping is given
    /// it).
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
    /// A stage's error, the error a thread failed with, or `InvalidInput` once the run is
    /// finished.
    pub fn track(&mut self, frame: &Frame) -> Result<FrameResult> {
        if self.finished {
            return fail(ErrorKind::InvalidInput, "track: the run is finished");
        }
        let start = Instant::now();
        // Between frames the state is never `Lost`: a lost track closes its map on its frame.
        let tracking_map = self.state == State::Tracking;
        let step = if tracking_map {
            self.track_frame(frame)?
        } else {
            self.initialize(frame)?
        };
        let mut waited = step.handshake.wait_seconds;
        let owner = &self.shared.owner;
        if let Some(threads) = &self.threads {
            threads.check()?;
            if !tracking_map && owner.has_map() {
                // A map started on it: the threads have none of it.
                let m = owner.map();
                lock(&self.shared.atlas).record(m.as_deref());
                drop(m);
                owner.clear_gone();
            }
            if step.lost && owner.has_map() {
                // The threads end the map's work first.
                let settling = Instant::now();
                threads.settle(false)?;
                waited += settling.elapsed().as_secs_f64();
            }
            lock(&self.shared.atlas).tick();
        } else {
            let m = owner.map();
            lock(&self.shared.atlas).date(m.as_deref());
            drop(m);
            owner.clear_gone();
        }
        let state = if step.lost { State::Lost } else { self.state };
        let (map_id, base) = {
            let atlas = lock(&self.shared.atlas);
            (atlas.map_id(), atlas.point_base())
        };
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
            && let Some(lost) = self.shared.owner.close()
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
            wait_seconds: waited,
            asked: step.handshake.asked,
            refused: step.handshake.refused,
            queue: step.handshake.queue,
            schedule: step.schedule,
        })
    }

    /// Ends the run: threaded, waits for the threads to finish their work, then ends them.
    /// Local mapping and loop closing handle every keyframe given, and each global BA running
    /// ends, the latest carried onto the map. Inline, nothing waits. No frame is tracked after.
    ///
    /// # Errors
    ///
    /// The error a thread failed with.
    pub fn finish(&mut self) -> Result<()> {
        let already = std::mem::replace(&mut self.finished, true);
        let Some(threads) = &mut self.threads else {
            return Ok(());
        };
        if already {
            return Ok(());
        }
        lock(&self.shared.atlas).end();
        let settled = threads.settle(true);
        threads.stop();
        settled
    }

    /// What the threads finished since the last call (threaded; none inline).
    ///
    /// # Returns
    ///
    /// The completions, in the order they came.
    #[must_use]
    pub fn completed(&self) -> Vec<Completion> {
        self.threads
            .as_ref()
            .map_or_else(Vec::new, Threads::completed)
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
        self.shared
            .owner
            .reading(|held| lock(&self.shared.atlas).lifetimes(held.map()))
    }

    /// The run's trajectory so far ([`Atlas::trajectory`], with the current map).
    ///
    /// # Returns
    ///
    /// The frames tracked, but a reset map's.
    #[must_use]
    pub fn trajectory(&self) -> Trajectory {
        self.shared
            .owner
            .reading(|held| lock(&self.shared.atlas).trajectory(held.map()))
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
        lock(&self.shared.atlas).close(m, reset);
        self.tracking = TrackingState::default();
        // Shared with loop closing's thread, settled by now.
        *lock(&self.shared.closing) = LoopState::default();
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
        lock(&self.shared.atlas).start();
        self.state = State::Tracking;
        lock(&self.shared.atlas).place_frame(&map, frame.timestamp, reference, &tracked.pose)?;
        let row = map.keyframes() - 1;
        self.shared.owner.start(map)?;
        self.tracking.accept(frame, &tracked)?;
        if self.threads.is_some()
            && let Some(view) = self.shared.owner.view()
        {
            self.tracking.anchor_to(&view, row)?;
            self.tracking.corrections = self.shared.owner.corrections();
        }
        summary.placed_by = Some(Placement::Initialization);
        Ok(Step {
            made: tracked.points.clone(), // every point the frame observes was made of it
            tracked: Some(tracked),
            keyframe: true,
            summary,
            ..Step::default()
        })
    }

    /// Runs the steps on a frame: tracks it, and maps and closes loops if it becomes a
    /// keyframe. Threaded, a keyframe goes to local mapping's thread instead.
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
        if self.threads.is_some()
            && let Some(view) = self.shared.owner.view()
        {
            self.tracking.follow(&view, self.shared.owner.corrections());
        }
        let keyframe_base = lock(&self.shared.atlas).keyframe_base();
        let mut found = track_step(
            &mut self.tracking,
            &self.shared.owner,
            &self.camera,
            frame,
            self.mode,
            &self.options,
            keyframe_base,
            self.threads.as_ref(),
        )?;
        let mut step = Step {
            summary: std::mem::take(&mut found.summary),
            matched: std::mem::take(&mut found.matched),
            lost: found.lost,
            handshake: found.handshake,
            ..Step::default()
        };
        let (Some(tracked), Some(reference), Some(view)) =
            (found.tracked, found.reference, found.view)
        else {
            return Ok(step);
        };
        step.keyframe = found.keyframe;
        if self.threads.is_some() {
            self.hand_over(frame, &view, &tracked, reference, found.keyframe)?;
            step.tracked = Some(tracked);
            return Ok(step);
        }
        drop(view); // inline, the view is the map itself, which mapping changes in place
        let mut mapped = KeyframeMapped::default();
        if found.keyframe {
            mapped = self.map_keyframe(frame, &tracked, &mut step.summary)?;
        } else if let Some(m) = self.shared.owner.map() {
            lock(&self.shared.atlas).place_frame(
                &m,
                frame.timestamp,
                m.kf_id[reference],
                &tracked.pose,
            )?;
        }
        self.tracking.accept(frame, &tracked)?;
        if mapped.corrected
            && let Some(m) = self.shared.owner.map()
        {
            // The last frame, the new keyframe's own, moved with it.
            self.tracking.move_last(&m.kf_pose[m.keyframes() - 1]);
        }
        step.tracked = Some(tracked);
        step.map_seconds = mapped.map_seconds;
        step.loop_seconds = mapped.loop_seconds;
        step.made = mapped.made;
        step.schedule = mapped.schedule;
        Ok(step)
    }

    /// Threaded: places a tracked frame, and gives local mapping its keyframe if it is one.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    /// * `view` - The map tracking read.
    /// * `tracked` - Its pose and matches.
    /// * `reference` - Its reference keyframe's row in the view.
    /// * `keyframe` - Whether it becomes a keyframe.
    ///
    /// # Errors
    ///
    /// An error if a pose cannot be inverted.
    fn hand_over(
        &mut self,
        frame: &Frame,
        view: &Map,
        tracked: &Tracked,
        reference: usize,
        keyframe: bool,
    ) -> Result<()> {
        {
            let mut atlas = lock(&self.shared.atlas);
            if keyframe {
                let index = atlas.frames();
                atlas.place_pending(index, frame.timestamp, &tracked.pose);
                drop(atlas);
                if let Some(threads) = &self.threads {
                    threads.request(Request {
                        index,
                        frame: frame.clone(),
                        tracked: tracked.clone(),
                    });
                }
            } else {
                atlas.place_frame(view, frame.timestamp, view.kf_id[reference], &tracked.pose)?;
            }
        }
        self.tracking.accept(frame, tracked)?;
        self.tracking.anchor_to(view, reference)
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
    /// The times, the points made, how mapping scheduled its steps, and whether a loop was
    /// corrected.
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
        let holding = lock(&self.shared.closing).held();
        let (camera, mode, options) = (&self.camera, self.mode, &self.options);
        // Local mapping's step: insert the frame as a keyframe and map around it.
        let mapped = self.shared.owner.apply_mapping(|m| {
            mapping::process(m, camera, frame, tracked, mode, options, &holding, None)
        })?;
        let mut out = KeyframeMapped {
            map_seconds: start.elapsed().as_secs_f64(),
            made: mapped.made,
            schedule: Some(mapped.schedule),
            ..KeyframeMapped::default()
        };
        summary.mapping = Some(mapped.summary);
        // The frame's reference is its own keyframe, placed where the frame was.
        if let Some(m) = self.shared.owner.map() {
            lock(&self.shared.atlas).place_keyframe(&m, frame.timestamp, &tracked.pose);
        }
        let searching = Instant::now();
        let (looped, corrected) = close_loops(
            &mut lock(&self.shared.closing),
            &self.shared.owner,
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
