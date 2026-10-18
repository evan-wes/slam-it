//! The edge pipeline, initialising, tracking each frame, mapping on keyframes and closing
//! loops: the Rust core's mirror of `slam_it.edge.pipeline`, whose docstring states the
//! per-frame loop, the steps' own state, the map's owner and the steps in threads of their own.
//! It runs in the native program, not across the binding.
//!
//! Threaded, the view a writer publishes ([`View`]) is a copy of the part of the map tracking's
//! next frame may read, from where its last frame left it
//! ([`crate::covisibility::view_keyframes`]), made each time a writer publishes: tracking holds
//! the view it read for its whole frame, and the writers go on changing the map in place. Inline,
//! the view is the map itself.

use std::fmt;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use nalgebra::Matrix4;
use slam_it_foundations::error::{Error, ErrorKind, Result, fail};
use slam_it_foundations::nls::Stop;
use slam_it_foundations::vocabulary::Vocabulary;
use slam_it_foundations::{orb, se3, sim3};

use crate::atlas::{Atlas, Lifetimes, Trajectory};
use crate::covisibility;
use crate::database::{Database, MapRef};
use crate::frames::{Camera, Frame, Tracked, check_setup};
use crate::initialization::{self, InitSummary, Initialization, Outcome, Started};
use crate::loops::{
    self, CorrectionSummary, Detection, DetectionSummary, GlobalSummary, Loop, Merge,
    MergeDetection, MergeSummary, Merging, Pending,
};
use crate::map::{self, Map};
use crate::mapping::{self, Interrupts, MappingSummary, Schedule};
use crate::options::{Mode, Options};
use crate::relocalization::{self, RelocalizationSummary, Relocalized};
use crate::threads::{Generation, Sender, Turn, Worker, Writer, lock, panicked};
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
    /// * `atlas` - The atlas, for the points' run ids.
    fn mark(&mut self, fate: Fate, ids: &[i64], atlas: &Atlas) {
        for ((slot, point), &id) in self.fate.iter_mut().zip(&mut self.point).zip(ids) {
            if id >= 0 {
                *slot = fate;
                *point = atlas.point_run_id(id);
            }
        }
    }
}

/// A map closed, as a run records it (`slam_it.edge.pipeline.LostSummary`): on the frame a
/// track was lost on, the frame that took up a kept map, or a session's first frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LostSummary {
    /// The map's keyframes then.
    pub keyframes: usize,
    /// Its points.
    pub points: usize,
    /// Whether it was reset rather than kept, having too few keyframes: `loss.reset_keyframes`
    /// or fewer, or fewer than `loss.session_keyframes` as a session ended.
    pub reset: bool,
}

/// How loop closing went on a keyframe (`slam_it.edge.pipeline.LoopSummary`); its time is the
/// frame's `loop_seconds`.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopSummary {
    /// The search for a loop.
    pub detection: DetectionSummary,
    /// The correction, when a loop closed.
    pub correction: Option<CorrectionSummary>,
    /// The search for a merge, when other maps are kept (`loops::detect_merge`).
    pub merging: Option<DetectionSummary>,
    /// The merge, when one was found.
    pub merge: Option<MergeSummary>,
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
    /// The map closed on a frame relocalisation found in another kept map, which tracking goes
    /// on in.
    pub left: Option<LostSummary>,
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

/// What tracking reads of the map on a frame ([`MapOwner::view`], `slam_it.edge.pipeline.View`):
/// a writer publishes the map and the corrections applied to it together, inside its turn, where
/// the count cannot move, so a view read once never has one without the other.
#[derive(Debug, Clone)]
pub struct View {
    /// The map itself (inline), or the part of it a writer published: those keyframes, their
    /// keypoints and points, ids kept ([`map::sub_map`]). Never changed after.
    pub map: Arc<Map>,
    /// The whole map's keyframes, which tracking's rules count.
    pub keyframes: usize,
    /// The corrections applied to it ([`MapOwner::corrections`]).
    pub corrections: u64,
}

/// A published view's size, as a run records it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ViewSize {
    /// Its keyframes.
    pub keyframes: usize,
    /// Its points.
    pub points: usize,
    /// The bytes its arrays hold.
    pub bytes: usize,
}

/// A map's size, as a view's is recorded.
///
/// # Arguments
///
/// * `m` - The map.
///
/// # Returns
///
/// Its keyframes, points, and the bytes its arrays hold.
#[must_use]
pub fn size_of(m: &Map) -> ViewSize {
    const fn bytes<T>(values: &[T]) -> usize {
        std::mem::size_of_val(values)
    }
    let observations: usize = m.pt_observations.iter().map(|rows| bytes(rows)).sum();
    ViewSize {
        keyframes: m.keyframes(),
        points: m.points(),
        bytes: bytes(&m.kf_id)
            + bytes(&m.kf_time)
            + bytes(&m.kf_pose)
            + bytes(&m.kf_parent)
            + bytes(&m.kf_start)
            + bytes(&m.kp_xy)
            + bytes(&m.kp_level)
            + bytes(&m.kp_angle)
            + bytes(&m.kp_descriptor)
            + bytes(&m.kp_bearing)
            + bytes(&m.kp_depth)
            + bytes(&m.kp_right)
            + bytes(&m.kp_point)
            + bytes(&m.pt_id)
            + bytes(&m.pt_xyz)
            + bytes(&m.pt_descriptor)
            + bytes(&m.pt_normal)
            + bytes(&m.pt_min_distance)
            + bytes(&m.pt_max_distance)
            + bytes(&m.pt_visible)
            + bytes(&m.pt_found)
            + bytes(&m.pt_first_kf)
            + bytes(&m.pt_reference_kf)
            + observations,
    }
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
    /// A view a writer published.
    pub view: Option<ViewSize>,
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
    /// The map the frame was tracked in, when a merge took that map into another on it.
    merged_from: Option<i64>,
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
    /// How a merge on it moved the map's point ids on, if one was made.
    point_offset: Option<i64>,
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
    /// The number of the kept map relocalisation found the frame in, if not the one tracked.
    moved: Option<i64>,
}

/// Tracking's result on a frame, as `track_step` gives it (`slam_it.edge.pipeline._Tracking`).
#[derive(Debug, Default)]
struct Tracking {
    /// The map tracking read, held for the frame; `None` only when there was none.
    view: Option<View>,
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
    /// Threaded: the corrections the view the last frame was placed on carried
    /// ([`View::corrections`]).
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
    /// Whether it loses the track: past `loss.relocalize_seconds` since the first failure (on
    /// it if that is 0), or
    /// for a small map or with `relocalization.enabled` off, on the `loss.lost_frames`-th in a
    /// row.
    pub fn record_failure(&mut self, frame: &Frame, keyframes: usize, options: &Options) -> bool {
        self.failures += 1;
        self.velocity = None;
        let since = *self.lost_since.get_or_insert(frame.timestamp);
        if !options.relocalization.enabled || keyframes <= options.loss.reset_keyframes {
            return self.failures >= options.loss.lost_frames;
        }
        let seconds = options.loss.relocalize_seconds;
        seconds == 0.0 || frame.timestamp - since > seconds
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
    /// * `corrections` - The corrections the view carried ([`View::corrections`]): the last
    ///   frame's pose is in that map's world.
    ///
    /// # Errors
    ///
    /// An error if the keyframe's pose cannot be inverted.
    pub fn anchor_to(&mut self, view: &Map, row: usize, corrections: u64) -> Result<()> {
        if let Some(last) = &self.last_tracked {
            self.anchor = Some(view.kf_id[row]);
            self.anchor_relative = Some(last.pose * se3::inverse(&view.kf_pose[row])?);
            self.corrections = corrections;
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
    /// * `corrections` - The corrections that view carries ([`View::corrections`]), read with it.
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

    /// Leaves the last frame's map for another (`slam_it.edge.pipeline.TrackingState
    /// .change_map`): its points and its motion hold there no more; the frame placed in the
    /// other map follows with no velocity, and the loss's time starts again from it.
    pub fn change_map(&mut self) {
        if let Some(last) = &mut self.last_tracked {
            last.points.fill(-1);
        }
        self.velocity = None;
        self.lost_since = None;
        self.failures = self.failures.max(1);
    }
}

/// Loop closing's own state from keyframe to keyframe (`slam_it.edge.pipeline.LoopState`,
/// ORB-SLAM3's `LoopClosing`'s). A map closed takes it with it.
#[derive(Debug, Clone, Default)]
pub struct LoopState {
    /// The loop being confirmed in the map, if any.
    pub pending: Option<Pending>,
    /// The merge being confirmed with another map, if any.
    pub merging: Option<Merging>,
}

impl LoopState {
    /// The map's keyframes a loop or merge being confirmed holds, which mapping's culling
    /// spares.
    ///
    /// # Returns
    ///
    /// Their ids in the map, ascending: the loop's two, and the merge's last; none when neither
    /// is being confirmed.
    #[must_use]
    pub fn held(&self) -> Vec<i64> {
        let mut out = self.pending.as_ref().map(Pending::held).unwrap_or_default();
        if let Some(merging) = &self.merging {
            out.push(merging.pending.last);
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// The map and its database, which only a writer changes ([`MapOwner`]'s).
#[derive(Debug, Default)]
pub struct Held {
    /// The map, once initialised: inline, shared with tracking's view, which tracking gives up
    /// before the map is changed; threaded, never shared, a view being a copy of its own.
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
/// map at a time ([`Writer`]), tracking reads the view the writers publish, a copy of the part its
/// next frame may read from where it says its last frame left it ([`MapOwner::anchor`]), and
/// tracking's counts wait for local mapping to apply them ([`MapOwner::apply_counts`]). The map
/// is never shared with a reader while it is changed, so it is never copied whole.
#[derive(Debug)]
pub struct MapOwner {
    /// The vocabulary, when relocalising or closing loops.
    vocabulary: Option<Vocabulary>,
    /// The handshake between the threads; `None` inline.
    writer: Option<Arc<Writer>>,
    /// The settings: the local map's, which a view's keyframes follow.
    options: Options,
    /// Inline, whether tracking reads each frame a view made then ([`Threading::InlineView`]).
    views: bool,
    /// The map and its database.
    held: Mutex<Held>,
    /// The view as last published, threaded.
    view: Mutex<Option<View>>,
    /// Where tracking's last frame left it: its points, by id, and its reference keyframe's id
    /// or -1.
    anchor: Mutex<(Vec<i64>, i64)>,
    /// The views published and not yet taken, each a completion with its frame.
    published: Mutex<Vec<Completion>>,
    /// The frame being tracked now, which a view published is dated by.
    frame: AtomicI64,
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
    /// * `threading` - How the steps run: threaded, a writer is made.
    /// * `options` - The settings: the local map's, which a view's keyframes follow.
    ///
    /// # Returns
    ///
    /// The owner.
    #[must_use]
    pub fn new(vocabulary: Option<Vocabulary>, threading: Threading, options: Options) -> Self {
        Self {
            vocabulary,
            writer: threading.threaded().then(|| Arc::new(Writer::default())),
            options,
            views: threading == Threading::InlineView,
            held: Mutex::new(Held::default()),
            view: Mutex::new(None),
            anchor: Mutex::new((Vec::new(), -1)),
            published: Mutex::new(Vec::new()),
            frame: AtomicI64::new(0),
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

    /// Whether tracking reads views, not the map itself: threaded, or inline as their test.
    ///
    /// # Returns
    ///
    /// Whether it does.
    #[must_use]
    pub const fn views(&self) -> bool {
        self.writer.is_some() || self.views
    }

    /// What tracking reads, once a frame: threaded, the view last published; inline, the map
    /// itself, or a view made now ([`Threading::InlineView`]). Each carries the corrections
    /// applied to its map.
    ///
    /// # Returns
    ///
    /// The view, or `None` with no map.
    pub fn view(&self) -> Option<View> {
        if self.writer.is_some() {
            return lock(&self.view).clone();
        }
        let held = lock(&self.held);
        let m = held.map.as_ref()?;
        if self.views {
            let (points, reference) = lock(&self.anchor).clone();
            return Some(self.view_of(m, &points, reference));
        }
        Some(View {
            map: Arc::clone(m),
            keyframes: m.keyframes(),
            corrections: self.corrections(),
        })
    }

    /// A view of a map from where a frame left tracking: the map's part those keyframes make
    /// ([`covisibility::view_keyframes`], [`map::sub_map`]).
    ///
    /// # Arguments
    ///
    /// * `m` - The map.
    /// * `points` - The point each of the frame's keypoints tracked, -1 for none.
    /// * `reference` - Its reference keyframe's id, or -1.
    ///
    /// # Returns
    ///
    /// The view.
    fn view_of(&self, m: &Map, points: &[i64], reference: i64) -> View {
        // Tracking's points are ids, -1 where none, which view_keyframes takes: it cannot fail,
        // and a debug build says so if it did, rather than fall back to the whole map silently.
        let rows = covisibility::view_keyframes(m, points, reference, &self.options);
        debug_assert!(
            rows.is_ok(),
            "view_keyframes failed on tracking's points: {rows:?}"
        );
        let rows = rows.unwrap_or_else(|_| (0..m.keyframes()).collect());
        // Made in a writer's turn or under a hold, where no correction runs: the count is the
        // map's.
        View {
            map: Arc::new(map::sub_map(m, &rows)),
            keyframes: m.keyframes(),
            corrections: self.corrections(),
        }
    }

    /// Says where tracking's last frame left it, which the views published follow.
    ///
    /// # Arguments
    ///
    /// * `points` - The point each of its keypoints tracked, -1 for none.
    /// * `reference` - Its reference keyframe's id, or -1.
    pub fn anchor(&self, points: Vec<i64>, reference: i64) {
        *lock(&self.anchor) = (points, reference);
    }

    /// Follows a frame relocalisation placed: the views follow it from now, and, threaded, one
    /// is published at once (under the hold relocalisation took), so that the frames after it
    /// read where it was placed.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, held.
    /// * `points` - The point each of the frame's keypoints was matched to, -1 for none.
    ///
    /// # Returns
    ///
    /// The view of where it was placed.
    pub fn relocalized(&self, m: &Map, points: Vec<i64>) -> View {
        let made = self.view_of(m, &points, -1);
        self.anchor(points, -1);
        if self.writer.is_some() {
            self.keep(made.clone());
        }
        made
    }

    /// Dates the views published from now: by the frame being tracked.
    ///
    /// # Arguments
    ///
    /// * `frame` - Its number ([`Atlas::frame`]).
    pub fn date(&self, frame: i64) {
        self.frame.store(frame, Ordering::SeqCst);
    }

    /// The views published since the last call, each with its frame.
    ///
    /// # Returns
    ///
    /// The completions saying so, in the order published.
    pub fn published(&self) -> Vec<Completion> {
        std::mem::take(&mut *lock(&self.published))
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
    /// * `view` - The map tracking read, given up inline where it is the map itself, so that
    ///   the map is changed in place.
    /// * `visible` - The points the frame's view held, by the map's ids.
    /// * `found` - The points it tracked.
    ///
    /// # Returns
    ///
    /// The view tracking goes on with: the map as counted, where it is the map itself.
    ///
    /// # Errors
    ///
    /// `InvalidInput` with no map, inline.
    pub fn count(&self, view: View, visible: &[i64], found: &[i64]) -> Result<View> {
        if self.writer.is_some() {
            lock(&self.counts).push((visible.to_vec(), found.to_vec()));
            return Ok(view);
        }
        let mut held = lock(&self.held);
        let Some(m) = &mut held.map else {
            return fail(ErrorKind::InvalidInput, "count: no map");
        };
        if self.views {
            add_counts(writable(m)?, visible, found)?;
            return Ok(view);
        }
        let (keyframes, corrections) = (view.keyframes, view.corrections);
        drop(view);
        add_counts(writable(m)?, visible, found)?;
        Ok(View {
            map: Arc::clone(m),
            keyframes,
            corrections,
        })
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
        let m = writable(m)?;
        for (visible, found) in &counts {
            add_counts(m, visible, found)?;
        }
        Ok(())
    }

    /// Runs a mapping step on the map, then brings the database to its keyframes and, threaded,
    /// publishes a view.
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
        let out = step(writable(m)?)?;
        self.changed(&mut held)?;
        Ok(out)
    }

    /// The handshake: runs a change on the map held, mapping stopped and its local BA aborted,
    /// then mapping released, as ORB-SLAM's loop closing does (`RequestStop`, `Release`; a
    /// global BA running is superseded by the caller's new generation). A change that gives a
    /// value corrected the map: the correction is counted, then its database is brought to it
    /// and a view published, carrying the new count. Inline nothing runs beside the change, so
    /// there is nothing to stop.
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
        let out = step(writable(m)?)?;
        if out.is_some() {
            // Counted first: the view published next carries the new count.
            self.corrections.fetch_add(1, Ordering::SeqCst);
            self.changed(&mut held)?;
        }
        Ok(out)
    }

    /// Publishes a view of the map so far for tracking to read, while its writer goes on
    /// (threaded): a copy of the part tracking's next frame may read, from where it last said it
    /// was ([`MapOwner::anchor`]), its size kept for the run ([`MapOwner::published`]).
    ///
    /// # Arguments
    ///
    /// * `m` - The map so far.
    pub fn publish(&self, m: &Map) {
        let (points, reference) = lock(&self.anchor).clone();
        self.keep(self.view_of(m, &points, reference));
    }

    /// Makes a view the one tracking reads, its size kept for the run.
    ///
    /// # Arguments
    ///
    /// * `view` - The view.
    fn keep(&self, view: View) {
        lock(&self.published).push(Completion {
            frame: self.frame.load(Ordering::SeqCst),
            view: Some(size_of(&view.map)),
            ..Completion::default()
        });
        *lock(&self.view) = Some(view);
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
    ///
    /// # Errors
    ///
    /// [`writable`]'s.
    pub fn clear_gone(&self) -> Result<()> {
        if let Some(m) = &mut lock(&self.held).map
            && !(m.gone_pt_id.is_empty() && m.gone_kf_id.is_empty())
        {
            map::clear_gone(writable(m)?);
        }
        Ok(())
    }

    /// Gives up the map, to be kept or reset: the database, the view, where tracking was and
    /// the counts waiting go with it.
    ///
    /// # Returns
    ///
    /// The map, if there was one, and its database, when it keeps one.
    pub fn close(&self) -> (Option<Map>, Option<Database>) {
        *lock(&self.view) = None;
        *lock(&self.anchor) = (Vec::new(), -1);
        lock(&self.counts).clear();
        let mut held = lock(&self.held);
        let database = held.database.take();
        // No reader holds the map once tracking has given its view up: never copied then.
        let m = held
            .map
            .take()
            .map(|m| Arc::try_unwrap(m).unwrap_or_else(|shared| (*shared).clone()));
        (m, database)
    }

    /// Takes a kept map up again, with its database, as the map being tracked; its view is
    /// made once tracking anchors it ([`MapOwner::relocalized`]). Counted as a correction first,
    /// so that view carries a new count and nothing stamped with the map left's is taken as the
    /// new map's (`slam_it.edge.pipeline.MapOwner.resume`).
    ///
    /// # Arguments
    ///
    /// * `m` - The map.
    /// * `database` - Its keyframe database.
    pub fn resume(&self, m: Map, database: Database) {
        let mut held = lock(&self.held);
        self.corrections.fetch_add(1, Ordering::SeqCst);
        held.map = Some(Arc::new(m));
        held.database = self.vocabulary.is_some().then_some(database);
    }

    /// Takes a merge's map, and brings the matched map's database to its keyframes; threaded,
    /// publishes a view. The merge moved the map's ids and world, so it is counted as a
    /// correction first, and the view published carries the new count.
    ///
    /// # Arguments
    ///
    /// * `m` - The merged map.
    /// * `database` - The matched map's database, which becomes the merged map's.
    ///
    /// # Errors
    ///
    /// A quantisation's error.
    pub fn take_merged(&self, m: Map, database: Database) -> Result<()> {
        let mut held = lock(&self.held);
        held.map = Some(Arc::new(m));
        if self.vocabulary.is_some() {
            held.database = Some(database);
        }
        self.corrections.fetch_add(1, Ordering::SeqCst);
        self.changed(&mut held)
    }

    /// After a step changed the map: brings the database to it and, threaded, publishes a view.
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
        if self.writer.is_some()
            && let Some(m) = held.map.as_deref()
        {
            self.publish(m);
        }
        Ok(())
    }
}

/// The map to change in place: never shared while it is changed (threaded, a view is a copy of
/// its own; inline, tracking gives its view up first), so never copied.
///
/// # Arguments
///
/// * `m` - The map, held.
///
/// # Returns
///
/// The map.
///
/// # Errors
///
/// `InvalidInput` if a reader still holds it.
fn writable(m: &mut Arc<Map>) -> Result<&mut Map> {
    match Arc::get_mut(m) {
        Some(m) => Ok(m),
        None => fail(
            ErrorKind::InvalidInput,
            "map: still read while it is changed",
        ),
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

/// Tracks the last frame against the view, or recovers by PnP, or relocalises. Relocalisation
/// reads the database and the whole map (threaded, with mapping held off); where tracking reads
/// views, the frame goes on with the view of where it was placed, published for the frames
/// after it.
///
/// # Arguments
///
/// * `state` - Tracking's state.
/// * `owner` - The map's owner, for its database and the view of a relocalised frame.
/// * `view` - What tracking reads; where it reads views, a relocalisation's replaces it.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
/// * `atlas` - The atlas: the map's number, and the kept maps relocalisation searches too.
///
/// # Returns
///
/// The placed frame, if any, how it was placed, the last stage's matches, the time spent
/// waiting for the map, and the kept map relocalisation found it in.
///
/// # Errors
///
/// A stage's error.
#[allow(
    clippy::too_many_arguments,
    reason = "tracking's state, the maps' holders, the frame, the settings, as Python's _place"
)]
fn place(
    state: &TrackingState,
    owner: &MapOwner,
    view: &mut View,
    camera: &Camera,
    frame: &Frame,
    mode: Mode,
    options: &Options,
    atlas: &Mutex<Atlas>,
) -> Result<Placed> {
    let (Some(last_frame), Some(last_tracked)) = (&state.last_frame, &state.last_tracked) else {
        return fail(
            ErrorKind::InvalidInput,
            "track: no last frame to track from",
        );
    };
    let m = Arc::clone(&view.map);
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
            moved: None,
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
            moved: None,
        });
    };
    let asked = Instant::now();
    let mut held_at = asked;
    let found = owner.reading(|held| -> Result<_> {
        held_at = Instant::now();
        let (Some(whole), Some(database)) = (held.map(), held.database()) else {
            return Ok(None);
        };
        let (found, moved) = relocalize_everywhere(
            whole,
            database,
            vocabulary,
            &lock(atlas),
            camera,
            frame,
            options,
        )?;
        // A view holds the part of the map its frame may read: the frame goes on with the view
        // of where it was placed; one placed in a kept map goes on in that map, once tracking
        // has taken it up.
        if owner.views()
            && moved.is_none()
            && let Some(tracked) = &found.tracked
        {
            *view = owner.relocalized(whole, tracked.points.clone());
        }
        Ok(Some((found, moved)))
    })?;
    let Some((found, moved)) = found else {
        return Ok(Placed {
            tracked: None,
            summary,
            matched: recovery.matched,
            waited: 0.0,
            moved: None,
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
        moved: found.tracked.as_ref().and(moved),
        tracked: found.tracked,
        summary,
        matched: found.matched,
        waited,
    })
}

/// Relocalises a frame in the map tracked, and in every kept map with it (the online-SLAM
/// design's section 14, item 2) when there are any.
///
/// # Arguments
///
/// * `m` - The map tracked.
/// * `database` - Its database.
/// * `vocabulary` - The vocabulary.
/// * `atlas` - The atlas: the map's number, and the kept maps.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `options` - The settings.
///
/// # Returns
///
/// The relocalisation, and the number of the kept map it placed the frame in, if not the one
/// tracked.
///
/// # Errors
///
/// A stage's error.
fn relocalize_everywhere(
    m: &Map,
    database: &Database,
    vocabulary: &Vocabulary,
    atlas: &Atlas,
    camera: &Camera,
    frame: &Frame,
    options: &Options,
) -> Result<(Relocalized, Option<i64>)> {
    if atlas.kept().is_empty() {
        let found = relocalization::relocalize(m, database, vocabulary, camera, frame, options)?;
        return Ok((found, None));
    }
    let number = atlas.map_id().unwrap_or(0);
    let mut searched = vec![MapRef {
        number,
        map: m,
        database,
    }];
    searched.extend(atlas.kept().iter().map(MapRef::from));
    let mut across =
        relocalization::relocalize_across(&searched, vocabulary, camera, frame, options)?;
    if across.map.is_none() && across.maps.last().is_some_and(|&last| last != number) {
        // The last candidate's matches are points of another map, not this one's.
        across.relocalized.matched.fill(-1);
    }
    let moved = across.map.filter(|&found| found != number);
    Ok((across.relocalized, moved))
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

/// A map taken up by tracking (`Pipeline::track_frame`'s switch): how the map left was closed,
/// and the time spent waiting for the threads.
type Switched = (LostSummary, f64);

/// Tracking's step (`slam_it.edge.pipeline._track_frame`): places a frame against its view,
/// tracks the local map, counts its points through the owner, and asks for a keyframe; a frame
/// not tracked is counted as a failure, and one relocalisation found in a kept map goes on in
/// it.
///
/// # Arguments
///
/// * `state` - Tracking's state.
/// * `shared` - The map's owner, given the frame's counts, and the atlas: the kept maps, and
///   the run's ids for the summary.
/// * `view` - The map's view, read once for the frame.
/// * `camera` - The camera.
/// * `frame` - The frame.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
/// * `threads` - The threads, asked whether mapping takes a keyframe; `None` inline.
/// * `switch` - Takes up the kept map relocalisation found the frame in, leaving the map
///   tracked, and says how it left it and how long it waited for the threads.
///
/// # Returns
///
/// How tracking went.
///
/// # Errors
///
/// A stage's error, the switch's, or `InvalidInput` with no map.
#[allow(
    clippy::too_many_arguments,
    reason = "tracking's state, the maps' holders and view, the frame, the settings and the threads"
)]
#[allow(
    clippy::too_many_lines,
    reason = "Python's _track_frame, step for step, with the switch to a kept map"
)]
fn track_step(
    state: &mut TrackingState,
    shared: &Shared,
    mut view: View,
    camera: &Camera,
    frame: &Frame,
    mode: Mode,
    options: &Options,
    threads: Option<&Threads>,
    switch: impl FnOnce(&mut TrackingState, i64) -> Result<Switched>,
) -> Result<Tracking> {
    let owner = &shared.owner;
    state.since_keyframe += 1;
    let Placed {
        tracked: placed,
        mut summary,
        matched,
        mut waited,
        moved,
    } = place(
        state,
        owner,
        &mut view,
        camera,
        frame,
        mode,
        options,
        &shared.atlas,
    )?;
    if let (Some(number), Some(placed)) = (moved, &placed) {
        drop(view); // inline, the map left is given up whole
        let (left, settled) = switch(state, number)?;
        summary.left = Some(left);
        waited += settled;
        // The map taken up, where the frame was placed.
        let taken = if owner.views() {
            owner.read(|held| {
                held.map()
                    .map(|m| owner.relocalized(m, placed.points.clone()))
            })
        } else {
            owner.view()
        };
        let Some(taken) = taken else {
            return fail(ErrorKind::InvalidInput, "track: no map to track against");
        };
        view = taken;
    }
    if threads.is_some() {
        // The view may hold what the threads have not recorded yet.
        lock(&shared.atlas).extend(&view.map);
    }
    // The rules count the whole map's keyframes, which a view holds a part of.
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
        let lost = state.record_failure(frame, view.keyframes, options);
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
    let local = tracking::track_local_map(&view.map, camera, frame, &placed, mode, options, wide)?;
    let mut looked = local.summary(0);
    looked.reference = lock(&shared.atlas).run_id(looked.reference);
    summary.local_map = Some(looked);
    let view = owner.count(view, &local.visible, &local.found)?;
    let needed = if recent {
        options.tracking.recovery_tracked
    } else {
        options.tracking.min_tracked
    };
    if local.tracked.inliers() < needed {
        let lost = state.record_failure(frame, view.keyframes, options);
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
    if !(recent && view.keyframes > options.tracking.max_frames) {
        // none just after a recovery
        let idle = threads.is_none_or(Threads::idle);
        out.handshake.asked = mapping::need_keyframe(
            &view.map,
            camera,
            &frame.depth,
            &local.tracked,
            reference,
            mode,
            options,
            idle,
            Some(state.since_keyframe),
            Some(view.keyframes),
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

/// Loop closing's result on a keyframe: how it went, whether a loop was corrected, and how a
/// merge moved the map's point ids on, if one was made.
#[derive(Debug, Default)]
struct Looped {
    /// How it went; `None` when loop closing did not run.
    summary: Option<LoopSummary>,
    /// Whether a loop was corrected.
    corrected: bool,
    /// The merge's point offset, if one was made.
    point_offset: Option<i64>,
}

/// Ids moved on by an offset, -1 (none) kept.
///
/// # Arguments
///
/// * `ids` - The ids, changed in place.
/// * `offset` - The offset.
fn offset_ids(ids: &mut [i64], offset: i64) {
    for id in ids.iter_mut().filter(|id| **id >= 0) {
        *id += offset;
    }
}

/// Merges the map being tracked into the kept map a merge found
/// (`slam_it.edge.pipeline._merge_maps`): the merged map is tracked on, with the matched map's
/// number and database; the loop being confirmed is carried into it (ids offset, similarity
/// `S G⁻¹`).
///
/// # Arguments
///
/// * `state` - Loop closing's state.
/// * `shared` - The map's owner, and the atlas holding the matched map.
/// * `camera` - The camera.
/// * `found` - The merge.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
///
/// # Returns
///
/// The merge's summary, and its point offset.
///
/// # Errors
///
/// `InvalidInput` for a matched map not kept; the merge's or the database's.
fn merge_maps(
    state: &mut LoopState,
    shared: &Shared,
    camera: &Camera,
    found: &Merge,
    mode: Mode,
    options: &Options,
) -> Result<(MergeSummary, i64)> {
    let mut to_world = Matrix4::identity();
    let mut merged = None;
    // The merge reads the map being tracked with mapping stopped, and makes the merged map,
    // which then replaces it ([`MapOwner::take_merged`]).
    shared.owner.stopped(|m| -> Result<Option<()>> {
        let atlas = lock(&shared.atlas);
        let Some(matched) = atlas.kept().iter().find(|kept| kept.number == found.map) else {
            return fail(
                ErrorKind::InvalidInput,
                "merge: the matched map is not kept",
            );
        };
        to_world = loops::merge_transform(m, found)?;
        merged = Some(loops::merge(m, &matched.map, camera, found, mode, options)?);
        Ok(None)
    })?;
    let Some(merged) = merged else {
        return fail(ErrorKind::InvalidInput, "merge: no map");
    };
    let Some(database) = lock(&shared.atlas).merge(found.map, &to_world) else {
        return fail(
            ErrorKind::InvalidInput,
            "merge: the matched map is not kept",
        );
    };
    let loops::Merged {
        map,
        keyframe_offset,
        point_offset,
        summary,
    } = merged;
    shared.owner.take_merged(map, database)?;
    if let Some(pending) = &mut state.pending {
        pending.matched += keyframe_offset;
        pending.last += keyframe_offset;
        pending.similarity *= sim3::inverse(&to_world);
        offset_ids(&mut pending.points, point_offset);
        offset_ids(&mut pending.matched_points, point_offset);
    }
    Ok((summary, point_offset))
}

/// Loop closing's step (`slam_it.edge.pipeline._close_loops`): searches the newest keyframe
/// for a loop and, with maps kept, for a merge, and makes one, through the owner; a merge goes
/// before a loop closed on the same keyframe (`loops::merge_first`).
///
/// # Arguments
///
/// * `state` - Loop closing's state.
/// * `shared` - The map's owner, and the atlas: the kept maps.
/// * `camera` - The camera.
/// * `mode` - The sensor mode.
/// * `options` - The settings.
///
/// # Returns
///
/// How it went (`None` when loop closing is off, there is no database, or the keyframe is the
/// map's first), whether a loop was corrected, and the merge's point offset if one was made.
///
/// # Errors
///
/// Detection's, the correction's, the merge's or the database's.
fn close_loops(
    state: &mut LoopState,
    shared: &Shared,
    camera: &Camera,
    mode: Mode,
    options: &Options,
) -> Result<Looped> {
    let owner = &shared.owner;
    let found = owner.read(
        |held| -> Result<Option<(Detection, Option<MergeDetection>)>> {
            let (Some(m), Some(database)) = (held.map(), held.database()) else {
                return Ok(None);
            };
            let keyframe = m.kf_id[m.keyframes() - 1];
            if !options.loops.enabled || keyframe == 0 {
                return Ok(None);
            }
            let mut found = loops::detect(
                m,
                database,
                camera,
                keyframe,
                state.pending.as_ref(),
                mode,
                options,
            )?;
            let atlas = lock(&shared.atlas);
            if atlas.kept().is_empty() {
                return Ok(Some((found, None)));
            }
            let searched = loops::detect_merge(
                m,
                database,
                camera,
                keyframe,
                atlas.kept(),
                state.merging.as_ref(),
                mode,
                options,
            )?;
            found = loops::merge_first(found, &searched);
            Ok(Some((found, Some(searched))))
        },
    )?;
    let Some((found, merging)) = found else {
        return Ok(Looped::default());
    };
    if let Some(searched) = &merging {
        state.merging.clone_from(&searched.pending);
    }
    state.pending = found.pending;
    let mut summary = LoopSummary {
        detection: found.summary,
        correction: None,
        merging: merging.as_ref().map(|m| m.summary.clone()),
        merge: None,
    };
    if let Some(made) = merging.and_then(|m| m.merge) {
        let (merged, point_offset) = merge_maps(state, shared, camera, &made, mode, options)?;
        summary.merge = Some(merged);
        return Ok(Looped {
            summary: Some(summary),
            corrected: false,
            point_offset: Some(point_offset),
        });
    }
    if let Some(closed) = &found.closed {
        summary.correction =
            owner.stopped(|m| loops::correct(m, camera, closed, mode, options).map(Some))?;
    }
    Ok(Looped {
        corrected: summary.correction.is_some(),
        summary: Some(summary),
        point_offset: None,
    })
}

/// How a pipeline's steps run (C++'s `Threading`, but its baseline's locks).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Threading {
    /// In turn, each step on the map itself: a run repeats exactly.
    #[default]
    Inline,
    /// Each in a thread of its own, our design: tracking reads the views the writers publish.
    View,
    /// In turn, as [`Threading::Inline`], tracking each frame on the view a writer would
    /// publish from the map as it is then: a test that tracking on a view decides as on the
    /// whole map (each frame copies its view).
    InlineView,
}

impl Threading {
    /// Whether the steps run in threads of their own.
    ///
    /// # Returns
    ///
    /// Whether they do.
    #[must_use]
    pub const fn threaded(self) -> bool {
        matches!(self, Self::View)
    }
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

impl Shared {
    /// Counts a frame in the run's records: what is recorded from now on, and the views
    /// published, are dated by the next.
    fn tick(&self) {
        let mut atlas = lock(&self.atlas);
        atlas.tick();
        self.owner.date(atlas.frame());
    }

    /// Ends the run's frames: what threads record after the last, and the views they publish,
    /// are dated by it.
    fn end(&self) {
        let mut atlas = lock(&self.atlas);
        atlas.end();
        self.owner.date(atlas.frame());
    }
}

/// A keyframe tracking asked local mapping for (`slam_it.edge.pipeline._Request`): mapping
/// places it by its reference keyframe if the map was corrected since.
struct Request {
    /// Its frame's number.
    index: i64,
    /// The frame.
    frame: Frame,
    /// Its pose and matches.
    tracked: Tracked,
    /// The frame's reference keyframe, by the map's id.
    reference: i64,
    /// Its pose relative to it, `T_frame_reference`.
    relative: Matrix4<f64>,
    /// The corrections the view it was tracked on carried ([`View::corrections`]).
    corrections: u64,
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

/// A keyframe as tracked, placed by its reference keyframe if the map moved since
/// (`slam_it.edge.pipeline._Threads._placed`): a correction (a loop's, or a global BA's carried
/// onto the map) since the frame was tracked moved the map from under its pose, so it is placed
/// where its reference keyframe now is, `T_frame_reference T_reference_world` (ORB-SLAM's
/// `UpdateLastFrame` rule; B3b's review panel, A2 and L1), through the reference's culled
/// ancestors. If that chain ends at no keyframe of the map, its pose as tracked is kept, as the
/// trajectory keeps such a frame's. Mapping's turn held.
///
/// # Arguments
///
/// * `shared` - What the threads share with tracking: the owner's count and the atlas.
/// * `request` - The keyframe.
/// * `m` - The map.
///
/// # Returns
///
/// Its pose and matches.
fn placed(shared: &Shared, request: &Request, m: &Map) -> Tracked {
    let mut out = request.tracked.clone();
    if request.corrections != shared.owner.corrections()
        && let Some(pose) = lock(&shared.atlas).pose_by(m, request.reference, &request.relative)
    {
        out.pose = pose;
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
        let (mapped, keyframe, run_ids) = owner.apply_mapping(|m| {
            let tracked = still_mapped(&placed(shared, request, m), m);
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
            // The points made, by the run's ids.
            let run_ids: Vec<i64> = mapped
                .made
                .iter()
                .map(|&id| atlas.point_run_id(id))
                .collect();
            Ok((mapped, m.kf_id[m.keyframes() - 1], run_ids))
        })?;
        done.map_seconds = Some(start.elapsed().as_secs_f64());
        done.made = run_ids;
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
            merging: None,
            merge: None,
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
            // A global BA that panicked has no result to carry: its panic is kept as an error,
            // raised as a failed thread's is (B3b's review panel, A4).
            if let Err(payload) = thread.join() {
                self.steps.keep(panicked("global BA", payload.as_ref()));
            }
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
    /// * `threading` - How the steps run: inline, a run repeats exactly.
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
        threading: Threading,
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
            owner: MapOwner::new(vocabulary, threading, options),
            atlas: Mutex::new(Atlas::new(options)),
            closing: Mutex::new(LoopState::default()),
        });
        let threads = threading
            .threaded()
            .then(|| Threads::start(&shared, mode, &camera, &options));
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

    /// The map's owner: a test corrects the map through it, as loop closing does
    /// ([`MapOwner::stopped`]).
    ///
    /// # Returns
    ///
    /// The owner.
    #[must_use]
    pub fn owner(&self) -> &MapOwner {
        &self.shared.owner
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
                owner.clear_gone()?;
            }
            if step.lost && owner.has_map() {
                // The threads end the map's work first.
                let settling = Instant::now();
                threads.settle(false)?;
                waited += settling.elapsed().as_secs_f64();
            }
            self.shared.tick();
        } else {
            let m = owner.map();
            lock(&self.shared.atlas).date(m.as_deref());
            drop(m);
            owner.clear_gone()?;
        }
        let state = if step.lost { State::Lost } else { self.state };
        let mut fates = Fates {
            fate: vec![Fate::Unmatched; frame.size()],
            point: vec![-1; frame.size()],
        };
        let map_id = {
            let atlas = lock(&self.shared.atlas);
            fates.mark(Fate::Rejected, &step.matched, &atlas);
            if let Some(t) = &step.tracked {
                fates.mark(Fate::Tracked, &t.points, &atlas);
            }
            fates.mark(Fate::Made, &step.made, &atlas);
            step.merged_from.or_else(|| atlas.map_id())
        };
        let mut summary = step.summary;
        if step.lost && self.shared.owner.map().is_some() {
            // A lost frame was tracking a map.
            let small = self.options.loss.reset_keyframes + 1;
            summary.lost = Some(leave(&self.shared, small, true));
            self.tracking = TrackingState::default();
            self.state = State::NotInitialized;
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
        self.shared.end();
        let settled = threads.settle(true);
        threads.stop();
        settled
    }

    /// What the threads finished since the last call, and the views the writers published
    /// (threaded; none inline).
    ///
    /// # Returns
    ///
    /// The completions, in the order they came, then the views in the order published.
    #[must_use]
    pub fn completed(&self) -> Vec<Completion> {
        let Some(threads) = &self.threads else {
            return Vec::new();
        };
        let mut out = threads.completed();
        out.extend(self.shared.owner.published());
        out
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

    /// Starts the next of a run's sequences (`slam_it.edge.pipeline.Pipeline.start_session`,
    /// ORB-SLAM3's `ChangeDataset`): the map being tracked is reset if it holds fewer than
    /// `loss.session_keyframes` keyframes and kept whole otherwise, and tracking and loop
    /// closing start again, so the sequence's first frames start a new map.
    ///
    /// Threaded, the map waits for the threads first, as a lost map does.
    ///
    /// # Returns
    ///
    /// How the map was closed; `None` if none was being tracked.
    ///
    /// # Errors
    ///
    /// Threaded, the error a thread failed with.
    pub fn start_session(&mut self) -> Result<Option<LostSummary>> {
        let mut closed = None;
        if self.shared.owner.map().is_some() {
            if let Some(threads) = &self.threads {
                threads.settle(false)?;
            }
            closed = Some(leave(
                &self.shared,
                self.options.loss.session_keyframes,
                false,
            ));
        }
        self.tracking = TrackingState::default();
        self.state = State::NotInitialized;
        Ok(closed)
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
        self.shared.owner.anchor(tracked.points.clone(), reference); // what the first view follows
        self.shared.owner.start(map)?;
        self.tracking.accept(frame, &tracked)?;
        if self.threads.is_some()
            && let Some(view) = self.shared.owner.view()
            && let Some(row) = view.map.keyframe_row(reference)
        {
            // The map's first view, of the keyframe it started on.
            self.tracking.anchor_to(&view.map, row, view.corrections)?;
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
        // Read once for the frame: the map and the corrections it carries, published together.
        let Some(view) = self.shared.owner.view() else {
            return fail(ErrorKind::InvalidInput, "track: no map to track against");
        };
        if self.threads.is_some() {
            self.tracking.follow(&view.map, view.corrections);
        }
        let (shared, threads, options) = (&self.shared, self.threads.as_ref(), &self.options);
        let mut found = track_step(
            &mut self.tracking,
            shared,
            view,
            &self.camera,
            frame,
            self.mode,
            options,
            threads,
            |tracking, number| {
                // Threaded, the threads end the map's work first, as for a map lost.
                let settling = Instant::now();
                if let Some(threads) = threads {
                    threads.settle(false)?;
                }
                let waited = if threads.is_some() {
                    settling.elapsed().as_secs_f64()
                } else {
                    0.0
                };
                let left = leave(shared, options.loss.reset_keyframes + 1, false);
                let kept = lock(&shared.atlas).resume(number);
                if let Some(kept) = kept {
                    shared.owner.resume(kept.map, kept.database);
                }
                tracking.change_map();
                Ok((left, waited))
            },
        )?;
        let mut step = Step {
            summary: std::mem::take(&mut found.summary),
            matched: std::mem::take(&mut found.matched),
            lost: found.lost,
            handshake: found.handshake,
            ..Step::default()
        };
        let (Some(mut tracked), Some(reference), Some(view)) =
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
        // Inline, the frame's reference by its id in the view it was tracked on (the map itself,
        // or the test's view of it); then the view given up, the map being changed in place.
        let reference = view.map.kf_id[reference];
        drop(view);
        if self.shared.owner.views() {
            self.shared.owner.anchor(tracked.points.clone(), reference); // the next view follows
        }
        let mut mapped = KeyframeMapped::default();
        if found.keyframe {
            let before = lock(&self.shared.atlas).map_id();
            mapped = self.map_keyframe(frame, &tracked, &mut step.summary)?;
            if let Some(offset) = mapped.point_offset {
                // The frame's ids are the merged map's now.
                step.merged_from = before;
                offset_ids(&mut tracked.points, offset);
                offset_ids(&mut step.matched, offset);
                offset_ids(&mut mapped.made, offset);
                if self.shared.owner.views()
                    && let Some(m) = self.shared.owner.map()
                {
                    // The next frame's view follows the merged map's ids.
                    let newest = m.kf_id[m.keyframes() - 1];
                    self.shared.owner.anchor(tracked.points.clone(), newest);
                }
            }
        } else if let Some(m) = self.shared.owner.map() {
            lock(&self.shared.atlas).place_frame(&m, frame.timestamp, reference, &tracked.pose)?;
        }
        self.tracking.accept(frame, &tracked)?;
        if (mapped.corrected || mapped.point_offset.is_some())
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
    /// * `seen` - The view tracking read, with the corrections it carries.
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
        seen: &View,
        tracked: &Tracked,
        reference: usize,
        keyframe: bool,
    ) -> Result<()> {
        let view = &*seen.map;
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
                        reference: view.kf_id[reference],
                        relative: tracked.pose * se3::inverse(&view.kf_pose[reference])?,
                        corrections: seen.corrections,
                    });
                }
            } else {
                atlas.place_frame(view, frame.timestamp, view.kf_id[reference], &tracked.pose)?;
            }
        }
        self.tracking.accept(frame, tracked)?;
        self.tracking.anchor_to(view, reference, seen.corrections)?;
        // What the next views follow.
        self.shared
            .owner
            .anchor(tracked.points.clone(), view.kf_id[reference]);
        Ok(())
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
        let looped = close_loops(
            &mut lock(&self.shared.closing),
            &self.shared,
            &self.camera,
            self.mode,
            &self.options,
        )?;
        out.corrected = looped.corrected;
        out.point_offset = looped.point_offset;
        if let Some(looped) = looped.summary {
            out.loop_seconds = searching.elapsed().as_secs_f64();
            summary.loop_closing = Some(looped);
        }
        Ok(out)
    }
}

/// Closes the map being tracked (`slam_it.edge.pipeline.Pipeline._leave`): keeps it whole, or
/// resets it if it has fewer than `small` keyframes ([`Atlas::close`]). Loop closing's state
/// goes with it; threaded, the threads have settled first.
///
/// # Arguments
///
/// * `shared` - The map's owner, giving the map up; the atlas, keeping or resetting it; and
///   loop closing's state, cleared.
/// * `small` - The keyframes a map must hold to be kept.
/// * `dated` - Whether this frame is dated already: a map lost on a frame is closed after it,
///   one left for another kept map while tracking it.
///
/// # Returns
///
/// The closed map's summary.
fn leave(shared: &Shared, small: usize, dated: bool) -> LostSummary {
    // Shared with loop closing's thread, settled by now.
    *lock(&shared.closing) = LoopState::default();
    let (Some(m), database) = shared.owner.close() else {
        return LostSummary::default();
    };
    let out = LostSummary {
        keyframes: m.keyframes(),
        points: m.points(),
        reset: m.keyframes() < small,
    };
    lock(&shared.atlas).close(m, database, out.reset, dated);
    out
}
