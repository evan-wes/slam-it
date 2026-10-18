//! The run's maps and records: every map kept whole, each frame's place, every point's and
//! keyframe's life: the Rust core's mirror of `slam_it.edge.atlas`, whose docstring states them.
//! It only reads the map being tracked.
//!
//! With the steps in threads of their own (milestone 2B's B3b), the threads that change the map
//! record what they made and removed as they change it ([`Atlas::record`]), dated by the frame
//! being tracked then, and tracking counts the frames ([`Atlas::tick`]); a frame that became a
//! keyframe is placed before local mapping makes its keyframe ([`Atlas::place_pending`]), its
//! reference set once it has ([`Atlas::resolve`]). The pipeline shares the atlas behind a lock.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{Matrix4, Vector3};
use slam_it_foundations::error::Result;
use slam_it_foundations::{se3, sim3};

use crate::covisibility;
use crate::database::{Database, Indexed};
use crate::loops::{rigid_of, scale_of};
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
/// * `count` - The count.
///
/// # Returns
///
/// The id.
fn to_id(count: usize) -> i64 {
    i64::try_from(count).expect("a run's ids fit in i64")
}

/// A life: the frame that made a point or keyframe, the frame that removed it or -1, and why
/// (a `Removal` code) or -1.
pub type Life = [i64; 3];

/// Every point and keyframe a run made, by the run's id (continuing across maps), where each
/// last was, its life and the map it ends in (`slam_it.edge.atlas.Lifetimes`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Lifetimes {
    /// Every point, its last position in its map's world frame.
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
    /// Each kept map's strong links at the end, and each reset map's when it was reset, by the
    /// run's ids, the rows ascending.
    pub keyframe_links: Vec<[i64; 3]>,
    /// Each map's loop edges, by the run's ids, the lower first, in the order closed.
    pub keyframe_loops: Vec<[i64; 2]>,
    /// Each map's merge edges, by the run's ids, the lower first, in the order made.
    pub keyframe_merges: Vec<[i64; 2]>,
    /// The map each point ends in.
    pub point_maps: Vec<i64>,
    /// The map each keyframe ends in.
    pub keyframe_maps: Vec<i64>,
}

/// The run's trajectory: each tracked frame's final pose, and its pose as tracked
/// (`slam_it.edge.atlas.Trajectory`); a reset map's frames are left out.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Trajectory {
    /// Each tracked frame's time, in the order tracked.
    pub timestamps: Vec<f64>,
    /// Its final `T_cam_world`: its pose relative to its reference keyframe, carried by the
    /// keyframe's last pose.
    pub poses: Vec<Matrix4<f64>>,
    /// Its `T_cam_world` as tracked.
    pub tracked: Vec<Matrix4<f64>>,
    /// The map its final pose is in: the map it was tracked in, or the one that map was merged
    /// into.
    pub maps: Vec<i64>,
    /// The map it was tracked in.
    pub tracked_maps: Vec<i64>,
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
    /// Its pose relative to the reference, `T_cam_ref`.
    relative: Matrix4<f64>,
    /// Its `T_cam_world` as tracked.
    tracked: Matrix4<f64>,
}

/// A map's ids' run ids, by its ids.
#[derive(Debug, Clone, Default)]
struct Runs {
    /// Each keyframe id's.
    keyframes: Vec<i64>,
    /// Each point id's.
    points: Vec<i64>,
}

/// An id of a map as the run's, by its table of run ids; -1 kept.
///
/// # Arguments
///
/// * `runs` - The map's ids' run ids.
/// * `id` - The id, or -1.
///
/// # Returns
///
/// The run's id, or -1.
fn run_of(runs: &[i64], id: i64) -> i64 {
    if id >= 0 { runs[to_index(id)] } else { -1 }
}

/// Two keyframes' run ids, the lower first.
///
/// # Arguments
///
/// * `runs` - The map's keyframe ids' run ids.
/// * `edge` - The two keyframes' ids in the map.
///
/// # Returns
///
/// The run ids, ascending.
fn pair_of(runs: &[i64], [first, second]: [i64; 2]) -> [i64; 2] {
    let (a, b) = (run_of(runs, first), run_of(runs, second));
    [a.min(b), a.max(b)]
}

/// The run's maps and records (`slam_it.edge.atlas.Atlas`): every map kept whole with its
/// database, the maps' numbers and each one's ids' run ids, every point's and keyframe's life,
/// and each tracked frame's place for the final trajectory.
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
    /// The current map's ids' run ids.
    runs: Runs,
    /// The maps kept, by number, each with its database.
    kept: Vec<Indexed>,
    /// Each kept map's ids' run ids, in `kept`'s order.
    kept_runs: Vec<Runs>,
    /// Each merged map's number to the number of the map it was merged into.
    merged_into: BTreeMap<i64, i64>,
    /// Lives, the map each point and keyframe was made in, and the removed ones' last places
    /// (the others' unset), by the run's id; the reset maps' strong links and loops.
    lives: Lifetimes,
    /// Each tracked frame's place, for the final trajectory.
    placements: Vec<FramePlace>,
    /// Each culled keyframe's parent and its pose relative to it, by the run's ids.
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
            runs: Runs::default(),
            kept: Vec::new(),
            kept_runs: Vec::new(),
            merged_into: BTreeMap::new(),
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

    /// The maps kept, not being tracked, by number, each with its database (empty when the run
    /// does not relocalise).
    ///
    /// # Returns
    ///
    /// The maps.
    #[must_use]
    pub fn kept(&self) -> &[Indexed] {
        &self.kept
    }

    /// Gives the map starting on this frame its number; its ids are the run's next.
    pub fn start(&mut self) {
        self.map_id = Some(self.maps);
        self.maps += 1;
        self.runs = Runs::default();
    }

    /// Gives each of the current map's ids without one a run id, its life starting on this
    /// frame: threaded, tracking reads a view that may hold points and keyframes the threads
    /// have not recorded yet, and gives them their run ids before it reads them by the run's
    /// ids.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, or a view of it ([`map::sub_map`] keeps its next ids).
    pub fn extend(&mut self, m: &Map) {
        let (frame, number) = (self.frame(), self.map_id.unwrap_or(0));
        let lives = &mut self.lives;
        while to_id(self.runs.points.len()) < m.next_pt_id {
            self.runs.points.push(to_id(lives.point_lives.len()));
            lives.point_lives.push([frame, -1, -1]);
            lives.point_maps.push(number);
            lives.points.push(Vector3::zeros());
        }
        while to_id(self.runs.keyframes.len()) < m.next_kf_id {
            self.runs.keyframes.push(to_id(lives.keyframe_lives.len()));
            lives.keyframe_lives.push([frame, -1, -1]);
            lives.keyframe_maps.push(number);
            lives.keyframe_times.push(0.0);
            lives.keyframe_poses.push(Matrix4::identity());
            lives.keyframe_parents.push(-1);
        }
    }

    /// Dates what this frame made in the current map and what it removed from it, read from
    /// its archive of removals, which the map's owner then empties. Every frame is dated, one a
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
        if let (Some(m), Some(_)) = (m, self.map_id) {
            self.extend(m);
            for ((&id, xyz), &how) in m.gone_pt_id.iter().zip(&m.gone_pt_xyz).zip(&m.gone_pt_how) {
                let id = to_index(run_of(&self.runs.points, id));
                let lives = &mut self.lives;
                lives.point_lives[id] = [lives.point_lives[id][0], frame, how];
                lives.points[id] = *xyz;
            }
            for (g, &id) in m.gone_kf_id.iter().enumerate() {
                let run = run_of(&self.runs.keyframes, id);
                let parent = run_of(&self.runs.keyframes, m.gone_kf_parent[g]);
                let lives = &mut self.lives;
                let row = to_index(run);
                lives.keyframe_lives[row] =
                    [lives.keyframe_lives[row][0], frame, Removal::Culled.code()];
                lives.keyframe_times[row] = m.gone_kf_time[g];
                lives.keyframe_poses[row] = m.gone_kf_pose[g];
                lives.keyframe_parents[row] = parent;
                if parent >= 0 {
                    self.culled.insert(run, (parent, m.gone_kf_relative[g]));
                }
            }
        }
    }

    /// Closes the current map on this frame: keeps it whole with its database, or resets it
    /// (its lives end as `Removal::Reset`, its strong links and loops kept, its frames' places
    /// dropped).
    ///
    /// # Arguments
    ///
    /// * `m` - The map the track left.
    /// * `database` - Its keyframe database, if the run relocalises.
    /// * `reset` - Whether it is reset rather than kept.
    /// * `dated` - Whether this frame is dated already: a map lost on a frame is closed after it,
    ///   one left for another kept map while tracking it.
    pub fn close(&mut self, m: Map, database: Option<Database>, reset: bool, dated: bool) {
        let number = self.map_id.take().unwrap_or(0);
        self.pending.clear();
        let runs = std::mem::take(&mut self.runs);
        if !reset {
            let at = self.kept.partition_point(|kept| kept.number < number);
            self.kept.insert(
                at,
                Indexed {
                    number,
                    map: m,
                    database: database.unwrap_or_default(),
                },
            );
            self.kept_runs.insert(at, runs);
            return;
        }
        let frame = if dated { self.frames - 1 } else { self.frames };
        let code = Removal::Reset.code();
        let links = self.run_links(&m, &runs);
        let lives = &mut self.lives;
        for (&id, xyz) in m.pt_id.iter().zip(&m.pt_xyz) {
            let row = to_index(run_of(&runs.points, id));
            lives.points[row] = *xyz;
            lives.point_lives[row] = [lives.point_lives[row][0], frame, code];
        }
        for (k, &id) in m.kf_id.iter().enumerate() {
            let row = to_index(run_of(&runs.keyframes, id));
            lives.keyframe_times[row] = m.kf_time[k];
            lives.keyframe_poses[row] = m.kf_pose[k];
            lives.keyframe_parents[row] = run_of(&runs.keyframes, m.kf_parent[k]);
            lives.keyframe_lives[row] = [lives.keyframe_lives[row][0], frame, code];
        }
        lives.keyframe_links.extend(links);
        lives.keyframe_loops.extend(
            m.loop_edges
                .iter()
                .map(|&edge| pair_of(&runs.keyframes, edge)),
        );
        self.placements.retain(|p| p.map != number);
    }

    /// Takes a kept map up again as the current one: relocalisation found the track in it.
    ///
    /// # Arguments
    ///
    /// * `number` - The kept map's number.
    ///
    /// # Returns
    ///
    /// The map and its database, for the map's owner; `None` if no kept map has the number.
    pub fn resume(&mut self, number: i64) -> Option<Indexed> {
        let at = self.kept.iter().position(|kept| kept.number == number)?;
        self.map_id = Some(number);
        self.runs = self.kept_runs.remove(at);
        Some(self.kept.remove(at))
    }

    /// Takes the current map into the kept map it merged with (`loops::merge`): the merged map
    /// is the current one after, with the matched map's number, its run ids the matched map's
    /// then the current map's, as the merge's offsets (the matched map's next ids) moved them;
    /// what the current map removed before is carried into the matched map's world by `G`.
    ///
    /// # Arguments
    ///
    /// * `matched` - The kept map's number.
    /// * `to_world` - `G`, the similarity carrying the current map's world into the matched
    ///   map's (`loops::merge_transform`).
    ///
    /// # Returns
    ///
    /// The matched map's database, which becomes the merged map's; `None` if no kept map has
    /// the number.
    pub fn merge(&mut self, matched: i64, to_world: &Matrix4<f64>) -> Option<Database> {
        let at = self.kept.iter().position(|kept| kept.number == matched)?;
        let current = self.map_id.unwrap_or(0);
        let kept = self.kept.remove(at);
        let mut runs = self.kept_runs.remove(at);
        let back = sim3::inverse(to_world);
        let scale = scale_of(&back);
        let lives = &mut self.lives;
        for &id in &self.runs.points {
            let row = to_index(id);
            if lives.point_lives[row][1] >= 0 {
                let p = lives.points[row];
                lives.points[row] =
                    to_world.fixed_view::<3, 3>(0, 0) * p + to_world.fixed_view::<3, 1>(0, 3);
            }
        }
        for &id in &self.runs.keyframes {
            let row = to_index(id);
            if lives.keyframe_lives[row][1] >= 0 {
                lives.keyframe_poses[row] = rigid_of(&(lives.keyframe_poses[row] * back));
            }
            if let Some((_, relative)) = self.culled.get_mut(&id) {
                for r in 0..3 {
                    relative[(r, 3)] /= scale;
                }
            }
        }
        runs.keyframes.append(&mut self.runs.keyframes);
        runs.points.append(&mut self.runs.points);
        self.runs = runs;
        self.merged_into.insert(current, matched);
        self.map_id = Some(matched);
        Some(kept.database)
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
    /// `InvalidInput` for a reference the map does not hold; the inversion's.
    pub fn place_frame(
        &mut self,
        m: &Map,
        timestamp: f64,
        reference: i64,
        pose: &Matrix4<f64>,
    ) -> Result<()> {
        self.extend(m);
        let k = map::keyframe_row(m, reference)?;
        let relative = pose * se3::inverse(&m.kf_pose[k])?;
        self.placements.push(FramePlace {
            timestamp,
            map: self.map_id.unwrap_or(0),
            reference: self.run_id(reference),
            relative,
            tracked: *pose,
        });
        Ok(())
    }

    /// Keeps the place of a frame that became the map's newest keyframe: its own, where it was.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, the frame's keyframe its last.
    /// * `timestamp` - The frame's time.
    /// * `pose` - Its `T_cam_world` as tracked.
    pub fn place_keyframe(&mut self, m: &Map, timestamp: f64, pose: &Matrix4<f64>) {
        self.extend(m);
        let reference = self.run_id(m.kf_id[m.keyframes() - 1]);
        self.placements.push(FramePlace {
            timestamp,
            map: self.map_id.unwrap_or(0),
            reference,
            relative: Matrix4::identity(),
            tracked: *pose,
        });
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
            self.extend(m);
            self.placements[at].reference = self.run_id(m.kf_id[m.keyframes() - 1]);
        }
    }

    /// Every map held whole, the kept ones and the current one, each with its ids' run ids.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, if there is one.
    ///
    /// # Returns
    ///
    /// The maps.
    fn whole<'a>(&'a self, m: Option<&'a Map>) -> Vec<(&'a Map, &'a Runs)> {
        let mut out: Vec<(&Map, &Runs)> = self
            .kept
            .iter()
            .map(|kept| &kept.map)
            .zip(&self.kept_runs)
            .collect();
        if let (Some(m), Some(_)) = (m, self.map_id) {
            out.push((m, &self.runs));
        }
        out
    }

    /// Every point and keyframe the run has made, with their lives and maps.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, if there is one.
    ///
    /// # Returns
    ///
    /// The points and keyframes by the run's id: those in a map where it has them, the others
    /// where they last were.
    #[must_use]
    pub fn lifetimes(&self, m: Option<&Map>) -> Lifetimes {
        let mut out = self.lives.clone();
        for (whole, runs) in self.whole(m) {
            for (&id, xyz) in whole.pt_id.iter().zip(&whole.pt_xyz) {
                out.points[to_index(run_of(&runs.points, id))] = *xyz;
            }
            for (k, &id) in whole.kf_id.iter().enumerate() {
                let row = to_index(run_of(&runs.keyframes, id));
                out.keyframe_times[row] = whole.kf_time[k];
                out.keyframe_poses[row] = whole.kf_pose[k];
                out.keyframe_parents[row] = run_of(&runs.keyframes, whole.kf_parent[k]);
            }
            out.keyframe_links.extend(self.run_links(whole, runs));
            out.keyframe_loops.extend(
                whole
                    .loop_edges
                    .iter()
                    .map(|&edge| pair_of(&runs.keyframes, edge)),
            );
            out.keyframe_merges.extend(
                whole
                    .merge_edges
                    .iter()
                    .map(|&edge| pair_of(&runs.keyframes, edge)),
            );
        }
        out.keyframe_links.sort_unstable();
        for map in out.point_maps.iter_mut().chain(&mut out.keyframe_maps) {
            *map = self.ends_in(*map);
        }
        out
    }

    /// The run's trajectory so far: each tracked frame placed by its reference keyframe's last
    /// pose, through the keyframe's culled ancestors where it was culled; a frame whose chain
    /// ends at none keeps its pose as tracked.
    ///
    /// # Arguments
    ///
    /// * `m` - The current map, if there is one.
    ///
    /// # Returns
    ///
    /// The frames tracked, but a reset map's.
    #[must_use]
    pub fn trajectory(&self, m: Option<&Map>) -> Trajectory {
        let mut last: BTreeMap<i64, Matrix4<f64>> = BTreeMap::new();
        for (row, life) in self.lives.keyframe_lives.iter().enumerate() {
            if life[1] >= 0 {
                last.insert(to_id(row), self.lives.keyframe_poses[row]);
            }
        }
        for (whole, runs) in self.whole(m) {
            for (k, &id) in whole.kf_id.iter().enumerate() {
                last.insert(run_of(&runs.keyframes, id), whole.kf_pose[k]);
            }
        }
        let mut out = Trajectory::default();
        for placed in &self.placements {
            let (keyframe, pose) = self.through_culled(placed.reference, placed.relative);
            out.timestamps.push(placed.timestamp);
            out.poses.push(
                last.get(&keyframe)
                    .map_or(placed.tracked, |anchor| pose * anchor),
            );
            out.tracked.push(placed.tracked);
            out.maps.push(self.ends_in(placed.map));
            out.tracked_maps.push(placed.map);
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
    /// ancestors ends at no keyframe of the map. The map's ids without a run id are given one
    /// first ([`Atlas::extend`]).
    #[must_use]
    pub fn pose_by(
        &mut self,
        m: &Map,
        keyframe: i64,
        relative: &Matrix4<f64>,
    ) -> Option<Matrix4<f64>> {
        self.extend(m);
        let (found, pose) = self.through_culled(run_of(&self.runs.keyframes, keyframe), *relative);
        m.kf_id
            .iter()
            .position(|&id| run_of(&self.runs.keyframes, id) == found)
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

    /// A keyframe of the current map, by the run's id.
    ///
    /// # Arguments
    ///
    /// * `id` - Its id in the map, given a run id, or -1.
    ///
    /// # Returns
    ///
    /// The run's id, or -1.
    #[must_use]
    pub fn run_id(&self, id: i64) -> i64 {
        run_of(&self.runs.keyframes, id)
    }

    /// A point of the current map, by the run's id.
    ///
    /// # Arguments
    ///
    /// * `id` - Its id in the map, given a run id, or -1.
    ///
    /// # Returns
    ///
    /// The run's id, or -1.
    #[must_use]
    pub fn point_run_id(&self, id: i64) -> i64 {
        run_of(&self.runs.points, id)
    }

    /// The map a map's points and keyframes end in: itself, or the one it merged into.
    ///
    /// # Arguments
    ///
    /// * `number` - The map's number.
    ///
    /// # Returns
    ///
    /// The number.
    fn ends_in(&self, mut number: i64) -> i64 {
        let mut seen = BTreeSet::new();
        while let Some(&into) = self.merged_into.get(&number) {
            if !seen.insert(number) {
                break;
            }
            number = into;
        }
        number
    }

    /// A map's strong links, by the run's ids, each the lower first.
    ///
    /// # Arguments
    ///
    /// * `m` - The map.
    /// * `runs` - Its ids' run ids.
    ///
    /// # Returns
    ///
    /// The links.
    fn run_links(&self, m: &Map, runs: &Runs) -> Vec<[i64; 3]> {
        covisibility::strong_links(m, &self.options)
            .into_iter()
            .map(|[a, b, weight]| {
                let [first, second] = pair_of(&runs.keyframes, [a, b]);
                [first, second, weight]
            })
            .collect()
    }
}
