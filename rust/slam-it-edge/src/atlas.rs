//! The run's records across its maps, each frame's place and every point's and keyframe's life:
//! the Rust core's mirror of `slam_it.edge.atlas`, whose docstring states them. It only reads a
//! map.
//!
//! With the steps in threads of their own (milestone 2B's B3b), the threads that change the map
//! record what they made and removed as they change it ([`Atlas::record`]), dated by the frame
//! being tracked then, and tracking counts the frames ([`Atlas::tick`]); a frame that became a
//! keyframe is placed before local mapping makes its keyframe ([`Atlas::place_pending`]), its
//! reference set once it has ([`Atlas::resolve`]). The pipeline shares the atlas behind a lock.

use std::collections::BTreeMap;

use nalgebra::{Matrix4, Vector3};
use slam_it_foundations::error::Result;
use slam_it_foundations::se3;

use crate::covisibility;
use crate::map::{self, Map, Removal};
use crate::options::Options;

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
/// last was, its life and its map (`slam_it.edge.atlas.Lifetimes`).
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
/// (`slam_it.edge.atlas.Trajectory`). A reset map's frames are left out.
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

/// A tracked frame's place, for the final trajectory (`slam_it.edge.atlas._Placement`).
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

/// The run's records across its maps (`slam_it.edge.atlas.Atlas`): the maps' numbers and the
/// run's id bases, every point's and keyframe's life, the closed maps' last places, and each
/// tracked frame's place for the final trajectory.
#[derive(Debug, Clone)]
pub struct Atlas {
    /// The settings: the strong links' threshold.
    options: Options,
    /// Frames dated so far.
    frames: i64,
    /// The current map's number, if there is one.
    map_id: Option<i64>,
    /// The maps started so far: the next one's number.
    maps: i64,
    /// The run's id of the current map's keyframe 0.
    kf_base: i64,
    /// The run's id of the current map's point 0.
    pt_base: i64,
    /// Every point and keyframe made so far, by the run's id: those no longer in the current
    /// map (removed, or in a closed map) where they last were; the others' entries are
    /// placeholders, filled from the map by [`Atlas::lifetimes`].
    lives: Lifetimes,
    /// Each tracked frame's place, for the final trajectory ([`Atlas::trajectory`]).
    placements: Vec<FramePlace>,
    /// Each culled keyframe's parent and pose relative to it, by the run's ids.
    culled: BTreeMap<i64, (i64, Matrix4<f64>)>,
    /// Each frame placed before its keyframe was made: its place's index, by the frame.
    pending: BTreeMap<i64, usize>,
    /// Whether the run's frames have ended.
    ended: bool,
}

impl Atlas {
    /// An atlas before the run's first frame.
    ///
    /// # Arguments
    ///
    /// * `options` - The settings: the strong links' threshold.
    ///
    /// # Returns
    ///
    /// The atlas.
    #[must_use]
    pub fn new(options: Options) -> Self {
        Self {
            options,
            frames: 0,
            map_id: None,
            maps: 0,
            kf_base: 0,
            pt_base: 0,
            lives: Lifetimes::default(),
            placements: Vec::new(),
            culled: BTreeMap::new(),
            pending: BTreeMap::new(),
            ended: false,
        }
    }

    /// The frames counted so far: the number of the frame being tracked.
    ///
    /// # Returns
    ///
    /// The count.
    #[must_use]
    pub const fn frames(&self) -> i64 {
        self.frames
    }

    /// The frame a record is dated by now: the one being tracked, or the last after
    /// [`Atlas::end`].
    ///
    /// # Returns
    ///
    /// The frame's number.
    #[must_use]
    pub const fn frame(&self) -> i64 {
        if self.ended {
            self.frames - 1
        } else {
            self.frames
        }
    }

    /// Ends the run's frames: what threads record after the last is dated by it.
    pub const fn end(&mut self) {
        self.ended = true;
    }

    /// Counts a frame: what is recorded from now on is dated by the next.
    pub const fn tick(&mut self) {
        self.frames += 1;
    }

    /// The current map's number, if a map is being tracked.
    ///
    /// # Returns
    ///
    /// The number.
    #[must_use]
    pub const fn map_id(&self) -> Option<i64> {
        self.map_id
    }

    /// The run's id of the current map's keyframe 0 (the next map's, between maps).
    ///
    /// # Returns
    ///
    /// The id.
    #[must_use]
    pub const fn keyframe_base(&self) -> i64 {
        self.kf_base
    }

    /// The run's id of the current map's point 0 (the next map's, between maps).
    ///
    /// # Returns
    ///
    /// The id.
    #[must_use]
    pub const fn point_base(&self) -> i64 {
        self.pt_base
    }

    /// Gives the map starting on this frame its number, and its ids past every map before it.
    pub fn start(&mut self) {
        self.map_id = Some(self.maps);
        self.maps += 1;
        self.kf_base = to_id(self.lives.keyframe_lives.len());
        self.pt_base = to_id(self.lives.point_lives.len());
    }

    /// Dates what this frame made in the current map and what it removed from it, read from its
    /// archive of removals, which the map's owner then empties. Every frame is dated, one a
    /// frame, whether or not a map is being tracked: [`Atlas::record`], then [`Atlas::tick`].
    ///
    /// # Arguments
    ///
    /// * `m` - The current map after the frame, if there is one.
    pub fn date(&mut self, m: Option<&Map>) {
        self.record(m);
        self.tick();
    }

    /// Records what the current map made and removed since the last record, dated by the frame
    /// being tracked ([`Atlas::frame`]); what it removed is read from its archive, which the
    /// map's owner then empties.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, if there is one.
    pub fn record(&mut self, m: Option<&Map>) {
        let frame = self.frame();
        let (Some(m), Some(map_id)) = (m, self.map_id) else {
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
    }

    /// Closes the current map on the frame just dated: keeps its record, or resets it (its
    /// lives end, as [`Removal::Reset`], and its frames' places are dropped). A stand-in until
    /// milestone 2B's B4: the map's strong links are counted here, for the run's record, inside
    /// the frame's timed step (58 to 360 ms in the C++ program on the test set's largest maps);
    /// B4's atlas keeps maps whole, and the record moves with it (the online-SLAM design,
    /// section 4).
    ///
    /// # Arguments
    ///
    /// * `m` - The map the track was lost in.
    /// * `reset` - Whether it is reset rather than kept.
    pub fn close(&mut self, m: &Map, reset: bool) {
        let frame = self.frames - 1; // dated already
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
        self.map_id = None;
        self.pending.clear();
    }

    /// Keeps a tracked frame's place: its reference keyframe and its pose relative to it.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map.
    /// * `timestamp` - The frame's time.
    /// * `reference` - Its reference keyframe's id in the map.
    /// * `pose` - Its `T_cam_world` as tracked.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for a keyframe the map does not hold.
    pub fn place_frame(
        &mut self,
        m: &Map,
        timestamp: f64,
        reference: i64,
        pose: &Matrix4<f64>,
    ) -> Result<()> {
        let k = map::keyframe_row(m, reference)?;
        let relative = pose * se3::inverse(&m.kf_pose[k])?;
        self.placements.push(FramePlace {
            timestamp,
            map: self.map_id.unwrap_or(0),
            reference: self.kf_base + reference,
            relative,
            tracked: *pose,
        });
        Ok(())
    }

    /// Places a frame that will be the current map's next keyframe, once local mapping makes it:
    /// its reference its own keyframe, its pose relative to it the identity, set by
    /// [`Atlas::resolve`]; until then the trajectory gives its pose as tracked.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame's number ([`Atlas::frames`] when it was tracked).
    /// * `timestamp` - Its time.
    /// * `pose` - Its `T_cam_world` as tracked.
    pub fn place_pending(&mut self, frame: i64, timestamp: f64, pose: &Matrix4<f64>) {
        self.pending.insert(frame, self.placements.len());
        self.placements.push(FramePlace {
            timestamp,
            map: self.map_id.unwrap_or(0),
            reference: -1,
            relative: Matrix4::identity(),
            tracked: *pose,
        });
    }

    /// Sets a pending frame's reference to the map's newest keyframe, the one made of it.
    ///
    /// # Arguments
    ///
    /// * `frame` - The frame's number, as [`Atlas::place_pending`] was given it.
    /// * `m` - The current map, the frame's keyframe its last.
    pub fn resolve(&mut self, frame: i64, m: &Map) {
        if let Some(at) = self.pending.remove(&frame) {
            self.placements[at].reference = self.kf_base + m.kf_id[m.keyframes() - 1];
        }
    }

    /// Keeps the place of a frame that became the map's newest keyframe: its own, where it was.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, the frame's keyframe its last.
    /// * `timestamp` - The frame's time.
    /// * `pose` - Its `T_cam_world` as tracked.
    pub fn place_keyframe(&mut self, m: &Map, timestamp: f64, pose: &Matrix4<f64>) {
        self.placements.push(FramePlace {
            timestamp,
            map: self.map_id.unwrap_or(0),
            reference: self.kf_base + m.kf_id[m.keyframes() - 1],
            relative: Matrix4::identity(),
            tracked: *pose,
        });
    }

    /// Every point and keyframe the run has made, with their lives and maps.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, if there is one.
    ///
    /// # Returns
    ///
    /// The points and keyframes by the run's id: those in the current map where it has them,
    /// the others where they last were.
    #[must_use]
    pub fn lifetimes(&self, m: Option<&Map>) -> Lifetimes {
        let mut out = self.lives.clone();
        if let Some(m) = m {
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

    /// The run's trajectory so far: each tracked frame's final pose, and as tracked.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, if there is one.
    ///
    /// # Returns
    ///
    /// The frames tracked, but a reset map's, each placed by its reference keyframe's last
    /// pose, through the keyframe's culled ancestors where it was culled.
    #[must_use]
    pub fn trajectory(&self, m: Option<&Map>) -> Trajectory {
        let last = self.lifetimes(m).keyframe_poses;
        let mut out = Trajectory::default();
        for placed in &self.placements {
            let (keyframe, pose) = self.through_culled(placed.reference, placed.relative);
            let anchor = usize::try_from(keyframe).ok().and_then(|k| last.get(k));
            out.timestamps.push(placed.timestamp);
            out.poses
                .push(anchor.map_or(placed.tracked, |anchor| pose * anchor));
            out.tracked.push(placed.tracked);
            out.maps.push(placed.map);
        }
        out
    }

    /// Places a pose given relative to one of the current map's keyframes by where that
    /// keyframe is now (`slam_it.edge.atlas.Atlas.pose_by`): a keyframe culled since is followed
    /// through its culled ancestors, as the trajectory places a frame. Local mapping places a
    /// keyframe tracked before a correction so (the online-SLAM design, section 12): ORB-SLAM's
    /// `UpdateLastFrame` rule.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map.
    /// * `keyframe` - The keyframe, by the map's id.
    /// * `relative` - The pose relative to it, `T_pose_keyframe`.
    ///
    /// # Returns
    ///
    /// The pose's `T_cam_world` in the map now; `None` if the keyframe's chain of culled
    /// ancestors ends at no keyframe of the map.
    #[must_use]
    pub fn pose_by(&self, m: &Map, keyframe: i64, relative: &Matrix4<f64>) -> Option<Matrix4<f64>> {
        let (found, pose) = self.through_culled(self.kf_base + keyframe, *relative);
        m.keyframe_row(found - self.kf_base)
            .map(|row| pose * m.kf_pose[row])
    }

    /// Follows a keyframe through its culled ancestors.
    ///
    /// # Arguments
    ///
    /// * `keyframe` - The keyframe, by the run's id.
    /// * `relative` - A pose relative to it.
    ///
    /// # Returns
    ///
    /// The first ancestor not culled, by the run's id, and the pose relative to it.
    fn through_culled(&self, mut keyframe: i64, mut relative: Matrix4<f64>) -> (i64, Matrix4<f64>) {
        while let Some(&(parent, step)) = self.culled.get(&keyframe) {
            relative *= step;
            keyframe = parent;
        }
        (keyframe, relative)
    }

    /// A keyframe of the current map, by the run's id: -1 kept.
    ///
    /// # Arguments
    ///
    /// * `id` - Its id in the map, or -1.
    ///
    /// # Returns
    ///
    /// The run's id, or -1.
    #[must_use]
    pub const fn run_id(&self, id: i64) -> i64 {
        if id >= 0 { self.kf_base + id } else { -1 }
    }

    /// A map's loop edges, by the run's ids.
    fn run_loops(&self, m: &Map) -> Vec<[i64; 2]> {
        m.loop_edges
            .iter()
            .map(|&[a, b]| [self.kf_base + a, self.kf_base + b])
            .collect()
    }

    /// A map's strong links, by the run's ids.
    fn run_links(&self, m: &Map) -> Vec<[i64; 3]> {
        covisibility::strong_links(m, &self.options)
            .into_iter()
            .map(|[a, b, weight]| [self.kf_base + a, self.kf_base + b, weight])
            .collect()
    }
}
