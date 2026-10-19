//! The edge pipeline, initialising, tracking each frame, mapping on keyframes and closing
//! loops: the Rust core's mirror of `slam_it.edge.pipeline`, whose docstring states the
//! per-frame loop and the final trajectory. It runs in the native program, not across the
//! binding.

use std::collections::BTreeMap;
use std::time::Instant;

use nalgebra::{Matrix4, Vector3};
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::vocabulary::Vocabulary;
use slam_it_foundations::{orb, se3};

use crate::covisibility;
use crate::database::Database;
use crate::frames::{Camera, Frame, Tracked, check_setup};
use crate::initialization::{self, InitSummary, Initialization, Outcome, Started};
use crate::loops::{self, CorrectionSummary, DetectionSummary, Pending};
use crate::map::{self, Map, Removal};
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
    /// Whether a loop was corrected on it.
    corrected: bool,
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
    /// Each one's last parent in its map's spanning tree, by the run's id, or -1.
    pub keyframe_parents: Vec<i64>,
    /// Each map's strong links when it closed, and the current map's now, by the run's ids,
    /// ascending.
    pub keyframe_links: Vec<[i64; 3]>,
    /// Each map's loop edges, the older keyframe first, by the run's ids, in the order closed.
    pub keyframe_loops: Vec<[i64; 2]>,
    /// The map each point was made in.
    pub point_maps: Vec<i64>,
    /// The map each keyframe was made in.
    pub keyframe_maps: Vec<i64>,
}

/// The run's trajectory: each tracked frame's final pose, and its pose as tracked
/// (`slam_it.edge.pipeline.Trajectory`). A reset map's frames are left out.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Trajectory {
    /// Each tracked frame's time, in the order tracked.
    pub timestamps: Vec<f64>,
    /// Its final `T_cam_world`: its pose relative to its reference keyframe, carried by the
    /// keyframe's last pose.
    pub poses: Vec<Matrix4<f64>>,
    /// Its `T_cam_world` as tracked.
    pub tracked: Vec<Matrix4<f64>>,
    /// The map it was tracked in.
    pub maps: Vec<i64>,
}

/// A tracked frame's place, for the final trajectory (`slam_it.edge.pipeline._Placement`).
#[derive(Debug, Clone, PartialEq)]
struct FramePlace {
    /// The frame's time.
    timestamp: f64,
    /// The map it was tracked in.
    map: i64,
    /// The run's id of its reference keyframe.
    reference: i64,
    /// Its pose relative to it, `T_cam_ref`.
    relative: Matrix4<f64>,
    /// Its `T_cam_world` as tracked.
    tracked: Matrix4<f64>,
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
    /// The vocabulary, when relocalising or closing loops.
    vocabulary: Option<Vocabulary>,
    /// The map's keyframe database, when relocalising or closing loops and a map exists.
    database: Option<Database>,
    /// The first failed frame's time, while the track is recently lost.
    lost_since: Option<f64>,
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
    /// The loop being confirmed in the current map, if any.
    pending: Option<Pending>,
    /// Each tracked frame's place, for the final trajectory ([`Pipeline::trajectory`]).
    placements: Vec<FramePlace>,
    /// Each culled keyframe's parent and pose relative to it, by the run's ids.
    culled: BTreeMap<i64, (i64, Matrix4<f64>)>,
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
            map: None,
            last_frame: None,
            last_tracked: None,
            velocity: None,
            reference: None,
            previous: Vec::new(),
            failures: 0,
            vocabulary,
            database: None,
            lost_since: None,
            since_recovery: None,
            frames: 0,
            map_id: None,
            maps: 0,
            kf_base: 0,
            pt_base: 0,
            lives: Lifetimes::default(),
            pending: None,
            placements: Vec::new(),
            culled: BTreeMap::new(),
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
        // Between frames the state is never `Lost`: a lost track closes its map on its frame.
        let step = if self.state == State::Tracking {
            self.track_frame(frame)?
        } else {
            self.initialize(frame)?
        };
        self.date();
        let state = if step.lost { State::Lost } else { self.state };
        let map_id = self.map_id;
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
        if step.lost
            && let Some(lost) = self.map.take()
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
            for (k, &id) in m.kf_id.iter().enumerate() {
                let row = to_index(self.kf_base + id);
                out.keyframe_times[row] = m.kf_time[k];
                out.keyframe_poses[row] = m.kf_pose[k];
                out.keyframe_parents[row] = self.run_id(m.kf_parent[k]);
            }
            out.keyframe_links.extend(self.run_links(m));
            out.keyframe_loops.extend(self.run_loops(m));
        }
        out
    }

    /// The run's trajectory so far: each tracked frame's final pose, and as tracked
    /// (`slam_it.edge.pipeline.Pipeline.trajectory`).
    ///
    /// # Returns
    ///
    /// The frames tracked, but a reset map's, each placed by its reference keyframe's last
    /// pose, through the keyframe's culled ancestors where it was culled.
    #[must_use]
    pub fn trajectory(&self) -> Trajectory {
        let last = self.lifetimes().keyframe_poses;
        let mut out = Trajectory::default();
        for placed in &self.placements {
            let (mut pose, mut keyframe) = (placed.relative, placed.reference);
            while let Some(&(parent, relative)) = self.culled.get(&keyframe) {
                pose *= relative;
                keyframe = parent;
            }
            let anchor = usize::try_from(keyframe).ok().and_then(|k| last.get(k));
            out.timestamps.push(placed.timestamp);
            out.poses
                .push(anchor.map_or(placed.tracked, |anchor| pose * anchor));
            out.tracked.push(placed.tracked);
            out.maps.push(placed.map);
        }
        out
    }

    /// A map's loop edges, by the run's ids.
    fn run_loops(&self, m: &Map) -> Vec<[i64; 2]> {
        m.loop_edges
            .iter()
            .map(|&[a, b]| [self.kf_base + a, self.kf_base + b])
            .collect()
    }

    /// A parent of the current map, by the run's id: -1 kept.
    fn run_id(&self, parent: i64) -> i64 {
        if parent >= 0 {
            self.kf_base + parent
        } else {
            -1
        }
    }

    /// A map's strong links, by the run's ids.
    fn run_links(&self, m: &Map) -> Vec<[i64; 3]> {
        covisibility::strong_links(m, &self.options)
            .into_iter()
            .map(|[a, b, weight]| [self.kf_base + a, self.kf_base + b, weight])
            .collect()
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
        lives.keyframe_parents.resize(keyframes, -1);
        for ((&id, xyz), &how) in m.gone_pt_id.iter().zip(&m.gone_pt_xyz).zip(&m.gone_pt_how) {
            let id = to_index(self.pt_base + id);
            lives.point_lives[id] = [lives.point_lives[id][0], frame, how];
            lives.points[id] = *xyz;
        }
        for (g, &id) in m.gone_kf_id.iter().enumerate() {
            let id = to_index(self.kf_base + id);
            lives.keyframe_lives[id] = [lives.keyframe_lives[id][0], frame, Removal::Culled.code()];
            lives.keyframe_times[id] = m.gone_kf_time[g];
            lives.keyframe_poses[id] = m.gone_kf_pose[g];
            let parent = m.gone_kf_parent[g];
            lives.keyframe_parents[id] = if parent >= 0 {
                self.kf_base + parent
            } else {
                -1
            };
            if parent >= 0 {
                self.culled.insert(
                    self.kf_base + m.gone_kf_id[g],
                    (self.kf_base + parent, m.gone_kf_relative[g]),
                );
            }
        }
        map::clear_gone(m);
    }

    /// Closes the map the track was lost in, on this frame: keeps it, or resets it if it has
    /// `loss.reset_keyframes` keyframes or fewer (its lives end, as [`Removal::Reset`]). The
    /// tracking state goes with it, so the next frame starts a new map. A stand-in until
    /// milestone 2B's B4: the map's strong links are counted here, for the run's record, inside
    /// the frame's timed step (58 to 360 ms in the C++ program on the test set's largest maps);
    /// B4's atlas keeps maps whole, and the record moves with it (the online-SLAM design,
    /// section 4).
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
        let reset = m.kf_id.len() <= self.options.loss.reset_keyframes;
        let code = Removal::Reset.code();
        let parents: Vec<i64> = m.kf_parent.iter().map(|&p| self.run_id(p)).collect();
        let links = self.run_links(m);
        let loop_edges = self.run_loops(m);
        let lives = &mut self.lives;
        for (&id, xyz) in m.pt_id.iter().zip(&m.pt_xyz) {
            let id = to_index(self.pt_base + id);
            lives.points[id] = *xyz;
            if reset {
                lives.point_lives[id] = [lives.point_lives[id][0], frame, code];
            }
        }
        for (k, &id) in m.kf_id.iter().enumerate() {
            let id = to_index(self.kf_base + id);
            lives.keyframe_times[id] = m.kf_time[k];
            lives.keyframe_poses[id] = m.kf_pose[k];
            lives.keyframe_parents[id] = parents[k];
            if reset {
                lives.keyframe_lives[id] = [lives.keyframe_lives[id][0], frame, code];
            }
        }
        lives.keyframe_links.extend(links);
        lives.keyframe_loops.extend(loop_edges);
        if reset {
            // Its frames' poses go with it.
            let closing = self.map_id;
            self.placements.retain(|p| Some(p.map) != closing);
        }
        self.pending = None;
        self.map_id = None;
        self.state = State::NotInitialized;
        self.last_frame = None;
        self.last_tracked = None;
        self.velocity = None;
        self.reference = None;
        self.previous.clear();
        self.failures = 0;
        self.since_recovery = None;
        self.lost_since = None;
        self.database = None;
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
        if self.vocabulary.is_some() {
            let mut database = Database::default();
            self.sync_database(&mut database, &map)?;
            self.database = Some(database);
        }
        self.map_id = Some(self.maps);
        self.maps += 1;
        self.kf_base = to_id(self.lives.keyframe_lives.len());
        self.pt_base = to_id(self.lives.point_lives.len());
        self.state = State::Tracking;
        self.place_frame(&map, frame, &tracked, map.kf_id[map.keyframes() - 1])?;
        self.map = Some(map);
        self.accept(frame, &tracked)?;
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
            corrected: false,
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
        if frame.size() <= self.options.initialization.min_features {
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
        if matched.len() < self.options.initialization.min_matches {
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
        self.lost_since = None;
        Ok(())
    }

    /// Keeps a tracked frame's place: its reference keyframe and its pose relative to it
    /// (`slam_it.edge.pipeline.Pipeline._place_frame`).
    ///
    /// # Arguments
    ///
    /// * `m` - The map.
    /// * `frame` - The frame.
    /// * `tracked` - Its pose.
    /// * `reference` - The reference keyframe's id in the map.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for a keyframe the map does not hold.
    fn place_frame(
        &mut self,
        m: &Map,
        frame: &Frame,
        tracked: &Tracked,
        reference: i64,
    ) -> Result<()> {
        let k = map::keyframe_row(m, reference)?;
        let relative = tracked.pose * se3::inverse(&m.kf_pose[k])?;
        self.placements.push(FramePlace {
            timestamp: frame.timestamp,
            map: self.map_id.unwrap_or(0),
            reference: self.kf_base + reference,
            relative,
            tracked: tracked.pose,
        });
        Ok(())
    }

    /// Searches the newest keyframe for a loop, and corrects one that closes
    /// (`slam_it.edge.pipeline.Pipeline._close_loops`).
    ///
    /// # Arguments
    ///
    /// * `m` - The map, changed in place by a correction.
    ///
    /// # Returns
    ///
    /// How it went (`None` when loop closing is off, there is no database, or the keyframe is
    /// the map's first), and whether a loop was corrected.
    ///
    /// # Errors
    ///
    /// Detection's, the correction's or the database's.
    fn close_loops(&mut self, m: &mut Map) -> Result<(Option<LoopSummary>, bool)> {
        let keyframe = m.kf_id[m.keyframes() - 1];
        let Some(database) = &self.database else {
            return Ok((None, false));
        };
        if !self.options.loops.enabled || keyframe == 0 {
            return Ok((None, false));
        }
        let found = loops::detect(
            m,
            database,
            &self.camera,
            keyframe,
            self.pending.as_ref(),
            self.mode,
            &self.options,
        )?;
        self.pending = found.pending;
        let mut correction = None;
        if let Some(closed) = &found.closed {
            correction = Some(loops::correct(
                m,
                &self.camera,
                closed,
                self.mode,
                &self.options,
            )?);
            if let Some(mut database) = self.database.take() {
                let synced = self.sync_database(&mut database, m);
                self.database = Some(database);
                synced?;
            }
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

    /// Counts a frame that was not tracked, which ends the velocity
    /// (`slam_it.edge.pipeline.Pipeline._record_failure`).
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame.
    /// * `keyframes` - The map's keyframes.
    ///
    /// # Returns
    ///
    /// Whether it loses the track: past `loss.relocalize_seconds` since the first failure, or for a
    /// small map or with `relocalization.enabled` off, on the `loss.lost_frames`-th in a row.
    fn record_failure(&mut self, frame: &Frame, keyframes: usize) -> bool {
        self.failures += 1;
        self.velocity = None;
        let since = *self.lost_since.get_or_insert(frame.timestamp);
        if !self.options.relocalization.enabled || keyframes <= self.options.loss.reset_keyframes {
            return self.failures >= self.options.loss.lost_frames;
        }
        frame.timestamp - since > self.options.loss.relocalize_seconds
    }

    /// Brings a keyframe database to a map's keyframes: adds the new, erases the culled.
    ///
    /// # Arguments
    ///
    /// * `database` - The database.
    /// * `m` - The map.
    ///
    /// # Errors
    ///
    /// A quantisation's error.
    fn sync_database(&self, database: &mut Database, m: &Map) -> Result<()> {
        let Some(vocabulary) = &self.vocabulary else {
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
        // tracks its reference keyframe by bag of words here; the lite pipeline's words only
        // relocalise.
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
        let recovery = tracking::relocalize_locally(
            m,
            &self.camera,
            frame,
            &last_tracked.points,
            &self.options,
        )?;
        summary.pnp = Some(recovery.summary);
        // The database relocalises only with relocalization.enabled: loop closing alone keeps it.
        let relocalizing = self
            .database
            .as_ref()
            .filter(|_| self.options.relocalization.enabled);
        let (Some(database), Some(vocabulary), None) =
            (relocalizing, &self.vocabulary, &recovery.tracked)
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
        let found = relocalization::relocalize(
            m,
            database,
            vocabulary,
            &self.camera,
            frame,
            &self.options,
        )?;
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
        let mut step = outcome?;
        if let Some(t) = &step.tracked {
            self.accept(frame, t)?;
            if step.corrected
                && let (Some(m), Some(last)) = (&self.map, &mut self.last_tracked)
            {
                // The last frame, the new keyframe's own, moved with it.
                last.pose = m.kf_pose[m.keyframes() - 1];
            }
        } else {
            let keyframes = self.map.as_ref().map_or(0, Map::keyframes);
            step.lost = self.record_failure(frame, keyframes);
        }
        Ok(step)
    }

    /// Maps a new keyframe and closes loops on it, timing each: its place in the trajectory is
    /// its own keyframe's.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, changed in place.
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
        m: &mut Map,
        frame: &Frame,
        tracked: &Tracked,
        summary: &mut FrameSummary,
    ) -> Result<KeyframeMapped> {
        let start = Instant::now();
        let holding = self.pending.as_ref().map(Pending::held).unwrap_or_default();
        let mapped = mapping::process(
            m,
            &self.camera,
            frame,
            tracked,
            self.mode,
            &self.options,
            &holding,
        )?;
        if let Some(mut database) = self.database.take() {
            let synced = self.sync_database(&mut database, m);
            self.database = Some(database);
            synced?;
        }
        let mut out = KeyframeMapped {
            map_seconds: start.elapsed().as_secs_f64(),
            made: mapped.made,
            ..KeyframeMapped::default()
        };
        summary.mapping = Some(mapped.summary);
        // The frame's reference is its own keyframe, placed where the frame was.
        self.placements.push(FramePlace {
            timestamp: frame.timestamp,
            map: self.map_id.unwrap_or(0),
            reference: self.kf_base + m.kf_id[m.keyframes() - 1],
            relative: Matrix4::identity(),
            tracked: tracked.pose,
        });
        let searching = Instant::now();
        let (looped, corrected) = self.close_loops(m)?;
        out.corrected = corrected;
        if let Some(looped) = looped {
            out.loop_seconds = searching.elapsed().as_secs_f64();
            summary.loop_closing = Some(looped);
        }
        Ok(out)
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
        if matches!(
            summary.placed_by,
            Some(Placement::LocalPnp | Placement::Relocalization)
        ) {
            self.since_recovery = Some(0);
        } else if let Some(since) = &mut self.since_recovery {
            *since += 1;
        }
        let recent = self
            .since_recovery
            .is_some_and(|s| s < options.tracking.max_frames);
        let wide = self
            .since_recovery
            .is_some_and(|s| s < options.tracking.recovery_window_frames);
        let Some(placed) = placed else {
            return Ok(Step {
                summary,
                matched,
                ..Step::default()
            });
        };
        let local =
            tracking::track_local_map(m, &self.camera, frame, &placed, self.mode, &options, wide)?;
        summary.local_map = Some(local.summary(self.kf_base));
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
            options.tracking.recovery_tracked
        } else {
            options.tracking.min_tracked
        };
        if local.tracked.inliers() < needed {
            return Ok(Step {
                summary,
                matched: local.matched,
                ..Step::default()
            });
        }
        let tracked = local.tracked;
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
                &self.camera,
                &frame.depth,
                &tracked,
                reference,
                self.mode,
                &options,
            )?;
        let mut mapped = KeyframeMapped::default();
        if keyframe {
            mapped = self.map_keyframe(m, frame, &tracked, &mut summary)?;
        } else {
            self.place_frame(m, frame, &tracked, m.kf_id[reference])?;
        }
        Ok(Step {
            tracked: Some(tracked),
            keyframe,
            map_seconds: mapped.map_seconds,
            summary,
            matched: local.matched,
            made: mapped.made,
            lost: false,
            loop_seconds: mapped.loop_seconds,
            corrected: mapped.corrected,
        })
    }
}
