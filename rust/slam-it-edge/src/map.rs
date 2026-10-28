//! The map: the Rust core's mirror of `slam_it.edge.map`.
//!
//! Keyframes, their keypoints and the map points, as one struct of arrays; points and
//! keyframes referred to by id, the id columns ascending (the Python module's docstring states
//! the tables and the rules a point's descriptor, normal and distances follow). Operations
//! change the map in place and leave it as Python's return it, array for array.
//!
//! One column has no Python twin: `pt_observations`, each point's observing keypoint rows,
//! which the operations keep in step with `kp_point` (as ORB-SLAM's `MapPoint` keeps its
//! observations). With it, an operation on a point costs what the point's observations cost,
//! however many keyframes the map holds; Python's reference scans `kp_point` instead.
//!
//! What was removed is kept aside, in the `gone_*` columns, until the pipeline takes it
//! ([`clear_gone`]), as in Python.

use std::collections::BTreeSet;

use nalgebra::{Matrix4, Vector3};
use slam_it_foundations::error::{ErrorKind, Result, check_range, fail, fail_setting};
use slam_it_foundations::orb::{self, Descriptor, MAX_LEVELS};

use crate::frames::Frame;
use crate::options::Options;

/// Why a point or keyframe left the map, with the codes a run records
/// (`slam_it.edge.map.Removal`, whose members document each).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Removal {
    /// Culled: a recent point tracking rarely found, or a redundant keyframe.
    Culled = 0,
    /// Fused into another point.
    Fused = 1,
    /// Erased with its views: left seen too little.
    Weak = 2,
    /// Its map was reset: the track was lost while the map was too small to keep.
    Reset = 3,
}

impl Removal {
    /// Every removal, in code order.
    pub const ALL: [Self; 4] = [Self::Culled, Self::Fused, Self::Weak, Self::Reset];

    /// The removal's code, as a run records it.
    ///
    /// # Returns
    ///
    /// Its position in [`Removal::ALL`].
    #[must_use]
    pub const fn code(self) -> i64 {
        self as i64
    }
}

/// Keyframes, their keypoints and the map points (`slam_it.edge.map.Map`).
#[derive(Debug, Clone, PartialEq)]
pub struct Map {
    /// The pyramid's levels.
    pub levels: usize,
    /// Its scale factor.
    pub scale_factor: f64,
    /// Each keyframe's id, ascending.
    pub kf_id: Vec<i64>,
    /// Its time, in seconds.
    pub kf_time: Vec<f64>,
    /// Its pose `T_cam_world`.
    pub kf_pose: Vec<Matrix4<f64>>,
    /// Its parent in the spanning tree, another keyframe's id, or -1 (the map's first, and one
    /// that has shared no point yet; [`crate::covisibility`]).
    pub kf_parent: Vec<i64>,
    /// Where each keyframe's keypoints start; the last is the keypoint count.
    pub kf_start: Vec<usize>,
    /// Each keypoint's pixel.
    pub kp_xy: Vec<[f64; 2]>,
    /// Its pyramid level.
    pub kp_level: Vec<usize>,
    /// Its orientation bin.
    pub kp_angle: Vec<usize>,
    /// Its descriptor.
    pub kp_descriptor: Vec<Descriptor>,
    /// Its descriptor's node at the vocabulary's direct level, at least 0, given with its
    /// keyframe (`slam_it.edge.map`'s docstring: ORB-SLAM's `mFeatVec`).
    pub kp_node: Vec<i64>,
    /// Its unit bearing (NaN where it has none).
    pub kp_bearing: Vec<Vector3<f64>>,
    /// Its depth, in metres (NaN for none).
    pub kp_depth: Vec<f64>,
    /// Its right-image column (NaN for none).
    pub kp_right: Vec<f64>,
    /// The id of the map point it observes, or -1.
    pub kp_point: Vec<i64>,
    /// Each point's id, ascending.
    pub pt_id: Vec<i64>,
    /// Its position in the world.
    pub pt_xyz: Vec<Vector3<f64>>,
    /// Its distinctive descriptor.
    pub pt_descriptor: Vec<Descriptor>,
    /// Its mean viewing direction, a unit vector.
    pub pt_normal: Vec<Vector3<f64>>,
    /// The nearest distance it is matchable from.
    pub pt_min_distance: Vec<f64>,
    /// The farthest.
    pub pt_max_distance: Vec<f64>,
    /// How many frames had it in view.
    pub pt_visible: Vec<i64>,
    /// How many matched it.
    pub pt_found: Vec<i64>,
    /// The id of the keyframe that created it.
    pub pt_first_kf: Vec<i64>,
    /// The id of its reference keyframe.
    pub pt_reference_kf: Vec<i64>,
    /// The next keyframe id to give.
    pub next_kf_id: i64,
    /// The next point id to give.
    pub next_pt_id: i64,
    /// The ids of the points erased since the archive was last cleared, in erasing order.
    pub gone_pt_id: Vec<i64>,
    /// Their last positions.
    pub gone_pt_xyz: Vec<Vector3<f64>>,
    /// Why each went ([`Removal::code`]).
    pub gone_pt_how: Vec<i64>,
    /// The ids of the keyframes erased since then (culled).
    pub gone_kf_id: Vec<i64>,
    /// Their times.
    pub gone_kf_time: Vec<f64>,
    /// Their last poses `T_cam_world`.
    pub gone_kf_pose: Vec<Matrix4<f64>>,
    /// Their last parents.
    pub gone_kf_parent: Vec<i64>,
    /// Each one's pose relative to its parent when it went, `T_cam_parent` (ORB-SLAM's
    /// `mTcp`): the identity for one without a parent.
    pub gone_kf_relative: Vec<Matrix4<f64>>,
    /// The loops closed in the map, each the two keyframes' ids, the older first, in the order
    /// they were closed.
    pub loop_edges: Vec<[i64; 2]>,
    /// The merges that made the map, each the two keyframes' ids, the older (the matched map's)
    /// first, in the order they were made.
    pub merge_edges: Vec<[i64; 2]>,
    /// Each point's observing keypoint rows, ascending: the index of `kp_point` the operations
    /// here keep. It does not cross to Python; `index_observations` builds it for a map
    /// assembled column by column.
    pub pt_observations: Vec<Vec<usize>>,
}

impl Map {
    /// A map with no keyframes and no points.
    ///
    /// # Arguments
    ///
    /// * `levels` - The pyramid's levels.
    /// * `scale_factor` - Its scale factor.
    ///
    /// # Returns
    ///
    /// The empty map.
    #[must_use]
    pub fn empty(levels: usize, scale_factor: f64) -> Self {
        Self {
            levels,
            scale_factor,
            kf_id: Vec::new(),
            kf_time: Vec::new(),
            kf_pose: Vec::new(),
            kf_parent: Vec::new(),
            kf_start: vec![0],
            kp_xy: Vec::new(),
            kp_level: Vec::new(),
            kp_angle: Vec::new(),
            kp_descriptor: Vec::new(),
            kp_node: Vec::new(),
            kp_bearing: Vec::new(),
            kp_depth: Vec::new(),
            kp_right: Vec::new(),
            kp_point: Vec::new(),
            pt_id: Vec::new(),
            pt_xyz: Vec::new(),
            pt_descriptor: Vec::new(),
            pt_normal: Vec::new(),
            pt_min_distance: Vec::new(),
            pt_max_distance: Vec::new(),
            pt_visible: Vec::new(),
            pt_found: Vec::new(),
            pt_first_kf: Vec::new(),
            pt_reference_kf: Vec::new(),
            next_kf_id: 0,
            next_pt_id: 0,
            gone_pt_id: Vec::new(),
            gone_pt_xyz: Vec::new(),
            gone_pt_how: Vec::new(),
            gone_kf_id: Vec::new(),
            gone_kf_time: Vec::new(),
            gone_kf_pose: Vec::new(),
            gone_kf_parent: Vec::new(),
            gone_kf_relative: Vec::new(),
            loop_edges: Vec::new(),
            merge_edges: Vec::new(),
            pt_observations: Vec::new(),
        }
    }

    /// The number of keyframes.
    ///
    /// # Returns
    ///
    /// It.
    #[must_use]
    pub fn keyframes(&self) -> usize {
        self.kf_id.len()
    }

    /// The number of keypoints.
    ///
    /// # Returns
    ///
    /// It.
    #[must_use]
    pub fn keypoints(&self) -> usize {
        self.kp_point.len()
    }

    /// The number of points.
    ///
    /// # Returns
    ///
    /// It.
    #[must_use]
    pub fn points(&self) -> usize {
        self.pt_id.len()
    }

    /// Finds a point by id.
    ///
    /// # Arguments
    ///
    /// * `id` - The id (-1 allowed).
    ///
    /// # Returns
    ///
    /// Its row, or `None`.
    #[must_use]
    pub fn point_row(&self, id: i64) -> Option<usize> {
        row_of(&self.pt_id, id)
    }

    /// Finds a keyframe by id.
    ///
    /// # Arguments
    ///
    /// * `id` - The id.
    ///
    /// # Returns
    ///
    /// Its row, or `None`.
    #[must_use]
    pub fn keyframe_row(&self, id: i64) -> Option<usize> {
        row_of(&self.kf_id, id)
    }

    /// A keypoint's keyframe row.
    ///
    /// # Arguments
    ///
    /// * `row` - The keypoint's row, in range.
    ///
    /// # Returns
    ///
    /// The row of the keyframe whose keypoints hold it.
    #[must_use]
    pub fn keyframe_of(&self, row: usize) -> usize {
        self.kf_start.partition_point(|&start| start <= row) - 1
    }

    /// A keyframe's camera centre in the world, `-Rᵀ t`.
    ///
    /// # Arguments
    ///
    /// * `k` - The keyframe's row.
    ///
    /// # Returns
    ///
    /// The centre.
    #[must_use]
    pub fn centre(&self, k: usize) -> Vector3<f64> {
        camera_centre(&self.kf_pose[k])
    }

    /// The points the given keyframes observe.
    ///
    /// # Arguments
    ///
    /// * `keyframes` - Keyframe rows.
    ///
    /// # Returns
    ///
    /// The points' rows, ascending, each once.
    #[must_use]
    pub fn observed_by(&self, keyframes: &[usize]) -> Vec<usize> {
        let ids: BTreeSet<i64> = keyframes
            .iter()
            .flat_map(|&k| {
                self.kp_point[self.kf_start[k]..self.kf_start[k + 1]]
                    .iter()
                    .copied()
            })
            .filter(|&id| id >= 0)
            .collect();
        let mut out: Vec<usize> = ids
            .into_iter()
            .filter_map(|id| self.point_row(id))
            .collect();
        out.sort_unstable();
        out
    }
}

/// Moves a world point into a camera's frame: `R X + t` (`slam_it.edge.map.to_camera`).
///
/// # Arguments
///
/// * `pose` - The camera's `T_cam_world`.
/// * `x` - The point.
///
/// # Returns
///
/// The point in the camera's frame.
#[must_use]
pub fn to_camera(pose: &Matrix4<f64>, x: &Vector3<f64>) -> Vector3<f64> {
    pose.fixed_view::<3, 3>(0, 0) * x + pose.fixed_view::<3, 1>(0, 3)
}

/// A camera's centre in the world, `-Rᵀ t` (`slam_it.edge.map.camera_centre`).
///
/// # Arguments
///
/// * `pose` - The camera's `T_cam_world`.
///
/// # Returns
///
/// The centre.
#[must_use]
pub fn camera_centre(pose: &Matrix4<f64>) -> Vector3<f64> {
    -(pose.fixed_view::<3, 3>(0, 0).transpose() * pose.fixed_view::<3, 1>(0, 3))
}

/// Finds an id in an ascending id column.
///
/// # Arguments
///
/// * `ids` - The column.
/// * `id` - The id.
///
/// # Returns
///
/// Its row, or `None`.
fn row_of(ids: &[i64], id: i64) -> Option<usize> {
    if id < 0 {
        return None;
    }
    ids.binary_search(&id).ok()
}

/// Checks a map's structure (`Map.__post_init__`).
///
/// # Arguments
///
/// * `m` - The map.
///
/// # Errors
///
/// `InvalidInput` for arrays of the wrong length, offsets that do not run from 0 to the
/// keypoint count, id columns that are not strictly ascending from 0 or reach their next id,
/// a keypoint observing a point not in the map, or observation lists that are not `kp_point`'s.
pub fn check_map(m: &Map) -> Result<()> {
    check_range("map.levels", m.levels, 1, MAX_LEVELS)?;
    if !(m.scale_factor.is_finite() && m.scale_factor > 1.0) {
        return fail_setting(
            "map.scale_factor",
            "must be finite and above 1",
            m.scale_factor,
        );
    }
    if m.kf_start.first() != Some(&0)
        || m.kf_start.windows(2).any(|w| w[1] < w[0])
        || m.kf_start.last() != Some(&m.keypoints())
    {
        return fail(
            ErrorKind::InvalidInput,
            "map.kf_start: offsets must run from 0 to the keypoint count, not decreasing",
        );
    }
    let (k, t, p) = (m.kf_id.len(), m.kp_point.len(), m.pt_id.len());
    let lengths = [
        m.kf_time.len() == k,
        m.kf_pose.len() == k,
        m.kf_parent.len() == k,
        m.kf_start.len() == k + 1,
        m.kp_xy.len() == t,
        m.kp_level.len() == t,
        m.kp_angle.len() == t,
        m.kp_descriptor.len() == t,
        m.kp_node.len() == t,
        m.kp_bearing.len() == t,
        m.kp_depth.len() == t,
        m.kp_right.len() == t,
        m.pt_xyz.len() == p,
        m.pt_descriptor.len() == p,
        m.pt_normal.len() == p,
        m.pt_min_distance.len() == p,
        m.pt_max_distance.len() == p,
        m.pt_visible.len() == p,
        m.pt_found.len() == p,
        m.pt_first_kf.len() == p,
        m.pt_reference_kf.len() == p,
        m.gone_pt_xyz.len() == m.gone_pt_id.len(),
        m.gone_pt_how.len() == m.gone_pt_id.len(),
        m.gone_kf_time.len() == m.gone_kf_id.len(),
        m.gone_kf_pose.len() == m.gone_kf_id.len(),
        m.gone_kf_parent.len() == m.gone_kf_id.len(),
        m.gone_kf_relative.len() == m.gone_kf_id.len(),
    ];
    if lengths.contains(&false) {
        return fail(
            ErrorKind::InvalidInput,
            "map: every table's arrays must have its row count",
        );
    }
    for (name, ids, next) in [
        ("kf_id", &m.kf_id, m.next_kf_id),
        ("pt_id", &m.pt_id, m.next_pt_id),
    ] {
        let ascending = ids.windows(2).all(|w| w[0] < w[1]);
        if !ascending
            || ids.first().is_some_and(|&f| f < 0)
            || ids.last().is_some_and(|&l| l >= next)
        {
            return fail(
                ErrorKind::InvalidInput,
                format!("map.{name}: ids must be ascending, from 0, below the next id"),
            );
        }
    }
    if m.kp_node.iter().any(|&node| node < 0) {
        return fail(
            ErrorKind::InvalidInput,
            "map.kp_node: every node must be at least 0",
        );
    }
    if m.kp_point
        .iter()
        .any(|&id| id < -1 || (id >= 0 && m.point_row(id).is_none()))
    {
        return fail(
            ErrorKind::InvalidInput,
            "map.kp_point: every observed point must be in the map",
        );
    }
    check_links(m)?;
    if m.pt_observations != indexed(m) {
        return fail(
            ErrorKind::InvalidInput,
            "map.pt_observations: expected each point's observing keypoint rows, ascending",
        );
    }
    check_gone(m)
}

/// Checks the keyframes' links: each parent, and each loop edge.
///
/// # Arguments
///
/// * `m` - The map.
///
/// # Errors
///
/// `InvalidInput` for a parent that is neither -1 nor another of its keyframes, or a loop edge
/// not joining two of them, the older first.
fn check_links(m: &Map) -> Result<()> {
    if m.kf_parent
        .iter()
        .zip(&m.kf_id)
        .any(|(&parent, &id)| parent == id || (parent != -1 && m.keyframe_row(parent).is_none()))
    {
        return fail(
            ErrorKind::InvalidInput,
            "map.kf_parent: a parent must be -1 or another of its keyframes",
        );
    }
    for (name, edges) in [
        ("loop_edges", &m.loop_edges),
        ("merge_edges", &m.merge_edges),
    ] {
        if edges.iter().any(|&[older, newer]| {
            older >= newer || m.keyframe_row(older).is_none() || m.keyframe_row(newer).is_none()
        }) {
            return fail(
                ErrorKind::InvalidInput,
                format!("map.{name}: each joins two of its keyframes, the older (lower id) first"),
            );
        }
    }
    Ok(())
}

/// Checks a map's archive of erased points and keyframes: each id one the map gave, once, and
/// no longer holds; each point's removal a [`Removal`] code.
///
/// # Arguments
///
/// * `m` - The map.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check_gone(m: &Map) -> Result<()> {
    for (gone, live, next) in [
        (&m.gone_pt_id, &m.pt_id, m.next_pt_id),
        (&m.gone_kf_id, &m.kf_id, m.next_kf_id),
    ] {
        let mut sorted = gone.clone();
        sorted.sort_unstable();
        let once = sorted.windows(2).all(|w| w[0] < w[1]);
        if !once
            || sorted
                .iter()
                .any(|&id| id < 0 || id >= next || live.binary_search(&id).is_ok())
        {
            return fail(
                ErrorKind::InvalidInput,
                "map.gone_*: an erased id must be one the map gave, once, and no longer holds",
            );
        }
    }
    if m.gone_pt_how
        .iter()
        .any(|&how| !Removal::ALL.iter().any(|r| r.code() == how))
    {
        return fail(
            ErrorKind::InvalidInput,
            "map.gone_pt_how: expected Removal codes",
        );
    }
    Ok(())
}

/// Empties the map's archive of removed points and keyframes (`slam_it.edge.map
/// .clear_gone`); the pipeline does after dating each frame's removals.
///
/// # Arguments
///
/// * `m` - The map.
pub fn clear_gone(m: &mut Map) {
    m.gone_pt_id.clear();
    m.gone_pt_xyz.clear();
    m.gone_pt_how.clear();
    m.gone_kf_id.clear();
    m.gone_kf_time.clear();
    m.gone_kf_pose.clear();
    m.gone_kf_parent.clear();
    m.gone_kf_relative.clear();
}

/// The part of a map some of its keyframes make (`slam_it.edge.map.sub_map`): those keyframes,
/// every keypoint of theirs, and the points they observe, ids kept and in the same order, each
/// point's observations those keyframes' (a writer's view,
/// [`crate::covisibility::view_keyframes`]).
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The keyframes' rows, ascending, each a row of `m` once.
///
/// # Returns
///
/// The map of them: a parent not among them -1, as for a keyframe without one; the archive of
/// removals empty, the loops and next ids as `m`'s.
#[must_use]
pub fn sub_map(m: &Map, rows: &[usize]) -> Map {
    let mut out = Map::empty(m.levels, m.scale_factor);
    out.next_kf_id = m.next_kf_id;
    out.next_pt_id = m.next_pt_id;
    out.loop_edges.clone_from(&m.loop_edges);
    let mut seen = vec![false; m.points()];
    for &k in rows {
        out.kf_id.push(m.kf_id[k]);
        out.kf_time.push(m.kf_time[k]);
        out.kf_pose.push(m.kf_pose[k]);
        out.kf_parent.push(m.kf_parent[k]);
        for row in m.kf_start[k]..m.kf_start[k + 1] {
            out.kp_xy.push(m.kp_xy[row]);
            out.kp_level.push(m.kp_level[row]);
            out.kp_angle.push(m.kp_angle[row]);
            out.kp_descriptor.push(m.kp_descriptor[row]);
            out.kp_node.push(m.kp_node[row]);
            out.kp_bearing.push(m.kp_bearing[row]);
            out.kp_depth.push(m.kp_depth[row]);
            out.kp_right.push(m.kp_right[row]);
            out.kp_point.push(m.kp_point[row]);
            if let Some(p) = m.point_row(m.kp_point[row]) {
                seen[p] = true;
            }
        }
        out.kf_start.push(out.kp_point.len());
    }
    for p in (0..m.points()).filter(|&p| seen[p]) {
        out.pt_id.push(m.pt_id[p]);
        out.pt_xyz.push(m.pt_xyz[p]);
        out.pt_descriptor.push(m.pt_descriptor[p]);
        out.pt_normal.push(m.pt_normal[p]);
        out.pt_min_distance.push(m.pt_min_distance[p]);
        out.pt_max_distance.push(m.pt_max_distance[p]);
        out.pt_visible.push(m.pt_visible[p]);
        out.pt_found.push(m.pt_found[p]);
        out.pt_first_kf.push(m.pt_first_kf[p]);
        out.pt_reference_kf.push(m.pt_reference_kf[p]);
    }
    out.kf_parent = out
        .kf_parent
        .iter()
        .map(|&parent| out.keyframe_row(parent).map_or(-1, |_| parent))
        .collect();
    index_observations(&mut out);
    out
}

/// Builds `pt_observations` from `kp_point`, for a map assembled column by column.
///
/// # Arguments
///
/// * `m` - The map (ids in `kp_point` not in the map are left out; `check_map` refuses them).
pub fn index_observations(m: &mut Map) {
    m.pt_observations = indexed(m);
}

/// Each point's observing keypoint rows, ascending, found by scanning `kp_point`.
///
/// # Arguments
///
/// * `m` - The map.
///
/// # Returns
///
/// One list per point.
fn indexed(m: &Map) -> Vec<Vec<usize>> {
    let mut out = vec![Vec::new(); m.points()];
    for (row, &id) in m.kp_point.iter().enumerate() {
        if let Some(p) = m.point_row(id) {
            out[p].push(row);
        }
    }
    out
}

/// Points a keypoint at nothing, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `row` - The keypoint's row.
fn detach(m: &mut Map, row: usize) {
    if let Some(p) = m.point_row(m.kp_point[row]) {
        let seen = &mut m.pt_observations[p];
        if let Ok(at) = seen.binary_search(&row) {
            seen.remove(at);
        }
    }
    m.kp_point[row] = -1;
}

/// Points a keypoint observing nothing at a point, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `row` - The keypoint's row.
/// * `p` - The point's row.
fn attach(m: &mut Map, row: usize, p: usize) {
    let seen = &mut m.pt_observations[p];
    seen.insert(seen.partition_point(|&r| r < row), row);
    m.kp_point[row] = m.pt_id[p];
}

/// Checks rows against a bound.
///
/// # Arguments
///
/// * `rows` - The rows.
/// * `bound` - One past the largest valid row.
/// * `name` - Their name, for the message.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
fn check_rows(rows: &[usize], bound: usize, name: &str) -> Result<()> {
    if rows.iter().any(|&r| r >= bound) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: rows must lie in [0, {bound})"),
        );
    }
    Ok(())
}

/// Keeps the elements of a vector whose flag is set, in order.
///
/// # Arguments
///
/// * `values` - The vector.
/// * `keep` - One flag per element.
fn keep_where<T>(values: &mut Vec<T>, keep: &[bool]) {
    let mut flags = keep.iter();
    values.retain(|_| *flags.next().unwrap_or(&false));
}

/// The cosine between a point's mean viewing direction and a view of it
/// (`slam_it.edge.map.viewing_cos`, `isInFrustum`'s `viewCos`), unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `p` - The point's row.
/// * `offset` - The point minus the viewing camera's centre.
/// * `distance` - The offset's length, positive.
///
/// # Returns
///
/// The cosine.
#[must_use]
pub fn viewing_cos(m: &Map, p: usize, offset: &Vector3<f64>, distance: f64) -> f64 {
    offset.dot(&m.pt_normal[p]) / distance
}

/// Whether a point lies within the distances it is matchable from
/// (`slam_it.edge.map.in_distance_range`): from `tracking.min_distance_margin` of its minimum
/// distance to `tracking.max_distance_margin` of its maximum (`isInFrustum`, `Fuse`), unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `p` - The point's row.
/// * `distance` - Its distance from the camera centre.
/// * `options` - The margins.
///
/// # Returns
///
/// Whether it is in range.
#[must_use]
pub fn in_distance_range(m: &Map, p: usize, distance: f64, options: &Options) -> bool {
    distance >= options.tracking.min_distance_margin * m.pt_min_distance[p]
        && distance <= options.tracking.max_distance_margin * m.pt_max_distance[p]
}

/// Predicts the pyramid level points appear at from a distance (`PredictScale`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The points' rows.
/// * `distance` - Their distances, positive.
///
/// # Returns
///
/// `ceil(log(d_max / d) / log s)`, clamped to the pyramid.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or a distance that is not positive, or not one per
/// row.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    reason = "the level is clamped to the pyramid before the cast"
)]
pub fn predict_level(m: &Map, rows: &[usize], distance: &[f64]) -> Result<Vec<usize>> {
    check_rows(rows, m.points(), "rows")?;
    if distance.len() != rows.len() || distance.iter().any(|&d| !(d.is_finite() && d > 0.0)) {
        return fail(
            ErrorKind::InvalidInput,
            "distance: expected one positive distance per row",
        );
    }
    let log_scale = m.scale_factor.ln();
    Ok(rows
        .iter()
        .zip(distance)
        .map(|(&p, &d)| {
            let level = ((m.pt_max_distance[p] / d).ln() / log_scale).ceil();
            level.clamp(0.0, (m.levels - 1) as f64) as usize
        })
        .collect())
}

/// Picks the descriptor with the least median distance to the others (ties: the earliest).
///
/// # Arguments
///
/// * `descriptors` - At least one descriptor.
///
/// # Returns
///
/// Its index.
///
/// # Errors
///
/// `InvalidInput` for none.
pub fn distinctive_descriptor(descriptors: &[Descriptor]) -> Result<usize> {
    if descriptors.is_empty() {
        return fail(
            ErrorKind::InvalidInput,
            "descriptors: expected at least one",
        );
    }
    let n = descriptors.len();
    let mut best = (0, i64::MAX);
    for (i, a) in descriptors.iter().enumerate() {
        let mut row: Vec<i64> = descriptors.iter().map(|b| orb::distance(a, b)).collect();
        row.sort_unstable();
        let median = row[(n - 1) / 2];
        if median < best.1 {
            best = (i, median);
        }
    }
    Ok(best.0)
}

/// Refreshes points, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The points' rows, each observed.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    reason = "observation counts and levels are small, exact in f64 and i32"
)]
fn refresh(m: &mut Map, rows: &[usize]) {
    for &p in rows {
        let seen = &m.pt_observations[p];
        let keyframes: Vec<usize> = seen.iter().map(|&r| m.keyframe_of(r)).collect();
        let descriptors: Vec<Descriptor> = seen.iter().map(|&r| m.kp_descriptor[r]).collect();
        let chosen = distinctive_descriptor(&descriptors).unwrap_or(0);
        let mut sum = Vector3::zeros();
        for &k in &keyframes {
            let direction = m.pt_xyz[p] - m.centre(k);
            sum += direction / direction.norm();
        }
        let mean = sum / seen.len() as f64;
        let observers: Vec<i64> = keyframes.iter().map(|&k| m.kf_id[k]).collect();
        let reference = if observers.contains(&m.pt_reference_kf[p]) {
            m.pt_reference_kf[p]
        } else {
            observers.iter().copied().min().unwrap_or(-1)
        };
        let at = observers
            .iter()
            .position(|&id| id == reference)
            .unwrap_or(0);
        let distance = (m.pt_xyz[p] - m.centre(keyframes[at])).norm();
        let level = m.kp_level[seen[at]];
        m.pt_descriptor[p] = descriptors[chosen];
        m.pt_normal[p] = mean / mean.norm();
        m.pt_reference_kf[p] = reference;
        m.pt_max_distance[p] = distance * m.scale_factor.powf(level as f64);
        m.pt_min_distance[p] = m.pt_max_distance[p] / m.scale_factor.powf((m.levels - 1) as f64);
    }
}

/// Removes the flagged points, unchecked, keeping them aside in the archive.
///
/// # Arguments
///
/// * `m` - The map.
/// * `erase` - One flag per point.
/// * `how` - Why they go.
fn erase(m: &mut Map, erase: &[bool], how: Removal) {
    if !erase.contains(&true) {
        return;
    }
    for (p, _) in erase.iter().enumerate().filter(|&(_, &e)| e) {
        for &r in &m.pt_observations[p] {
            m.kp_point[r] = -1;
        }
        m.gone_pt_id.push(m.pt_id[p]);
        m.gone_pt_xyz.push(m.pt_xyz[p]);
        m.gone_pt_how.push(how.code());
    }
    let keep: Vec<bool> = erase.iter().map(|&e| !e).collect();
    keep_where(&mut m.pt_id, &keep);
    keep_where(&mut m.pt_xyz, &keep);
    keep_where(&mut m.pt_descriptor, &keep);
    keep_where(&mut m.pt_normal, &keep);
    keep_where(&mut m.pt_min_distance, &keep);
    keep_where(&mut m.pt_max_distance, &keep);
    keep_where(&mut m.pt_visible, &keep);
    keep_where(&mut m.pt_found, &keep);
    keep_where(&mut m.pt_first_kf, &keep);
    keep_where(&mut m.pt_reference_kf, &keep);
    keep_where(&mut m.pt_observations, &keep);
}

/// Points' observation weights, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The points' rows.
///
/// # Returns
///
/// The weights.
fn weights(m: &Map, rows: &[usize]) -> Vec<i64> {
    rows.iter()
        .map(|&p| {
            m.pt_observations[p]
                .iter()
                .map(|&r| if m.kp_right[r].is_nan() { 1 } else { 2 })
                .sum()
        })
        .collect()
}

/// Forgets observations, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - Keypoint rows.
fn forget(m: &mut Map, rows: &[usize]) {
    let touched: BTreeSet<i64> = rows
        .iter()
        .map(|&r| m.kp_point[r])
        .filter(|&id| id >= 0)
        .collect();
    if touched.is_empty() {
        return;
    }
    for &r in rows {
        detach(m, r);
    }
    let points: Vec<usize> = touched
        .into_iter()
        .filter_map(|id| m.point_row(id))
        .collect();
    let weight = weights(m, &points);
    let mut weak = vec![false; m.points()];
    let mut remaining = Vec::new();
    for (&p, &w) in points.iter().zip(&weight) {
        if w <= 2 {
            weak[p] = true;
        } else {
            remaining.push(p);
        }
    }
    refresh(m, &remaining);
    erase(m, &weak, Removal::Weak);
}

/// Adds a keyframe and refreshes the points its keypoints observe.
///
/// # Arguments
///
/// * `m` - The map.
/// * `frame` - The frame (its pyramid the map's).
/// * `pose` - Its `T_cam_world`.
/// * `points` - The id each keypoint observes (each in the map, at most once), or -1.
/// * `nodes` - Each keypoint's vocabulary node at the direct level, at least 0
///   (`slam_it_foundations::vocabulary::quantize`'s `nodes`).
///
/// # Errors
///
/// `InvalidInput` for another pyramid, a pose that is not finite, ids that are not one per
/// keypoint, in the map, and distinct, or nodes that are not one per keypoint, each at least 0.
#[allow(
    clippy::float_cmp,
    reason = "the frame's scale factor must be the map's exactly, as in Python"
)]
pub fn insert_keyframe(
    m: &mut Map,
    frame: &Frame,
    pose: &Matrix4<f64>,
    points: &[i64],
    nodes: &[i64],
) -> Result<()> {
    if frame.features.levels != m.levels || frame.features.scale_factor != m.scale_factor {
        return fail(
            ErrorKind::InvalidInput,
            "frame: its pyramid must be the map's",
        );
    }
    if !pose.iter().all(|v| v.is_finite()) {
        return fail(ErrorKind::InvalidInput, "pose: expected finite values");
    }
    let n = frame.size();
    if points.len() != n
        || frame.features.descriptors.len() != n
        || frame.bearings.len() != n
        || frame.depth.len() != n
        || frame.right.len() != n
    {
        return fail(
            ErrorKind::InvalidInput,
            "points: expected one id per keypoint",
        );
    }
    if nodes.len() != n || nodes.iter().any(|&node| node < 0) {
        return fail(
            ErrorKind::InvalidInput,
            format!("nodes: expected {n} vocabulary nodes, one per keypoint, each at least 0"),
        );
    }
    let mut observed = BTreeSet::new();
    for &id in points {
        if id < -1 || (id >= 0 && (m.point_row(id).is_none() || !observed.insert(id))) {
            return fail(
                ErrorKind::InvalidInput,
                "points: each must be in the map, and observed once",
            );
        }
    }
    let first = m.keypoints();
    m.kf_id.push(m.next_kf_id);
    m.next_kf_id += 1;
    m.kf_time.push(frame.timestamp);
    m.kf_pose.push(*pose);
    m.kf_parent.push(-1);
    m.kf_start.push(first + n);
    for (i, k) in frame.features.keypoints.iter().enumerate() {
        m.kp_xy.push(k.xy);
        m.kp_level.push(k.level);
        m.kp_angle.push(k.angle);
        m.kp_descriptor.push(frame.features.descriptors[i]);
        m.kp_node.push(nodes[i]);
        m.kp_bearing.push(frame.bearings[i]);
        m.kp_depth.push(frame.depth[i]);
        m.kp_right.push(frame.right[i]);
        m.kp_point.push(points[i]);
        if let Some(p) = m.point_row(points[i]) {
            m.pt_observations[p].push(first + i);
        }
    }
    let rows: Vec<usize> = observed
        .into_iter()
        .filter_map(|id| m.point_row(id))
        .collect();
    refresh(m, &rows);
    Ok(())
}

/// Creates points, each observed by keyframe keypoints; their creator and reference is the last
/// observer's keyframe, visible and found once.
///
/// # Arguments
///
/// * `m` - The map.
/// * `xyz` - The points in the world, finite.
/// * `observers` - Each point's observing keypoint rows, at least one: free, used once, and
///   each point's from different keyframes.
///
/// # Errors
///
/// `InvalidInput` for points that are not finite or observers breaking a rule.
pub fn add_points(m: &mut Map, xyz: &[Vector3<f64>], observers: &[Vec<usize>]) -> Result<()> {
    if observers.len() != xyz.len() || xyz.iter().any(|x| !x.iter().all(|v| v.is_finite())) {
        return fail(
            ErrorKind::InvalidInput,
            "xyz: expected finite points, and one row of observers per point",
        );
    }
    let mut used = BTreeSet::new();
    for rows in observers {
        let mut keyframes = BTreeSet::new();
        let ok = !rows.is_empty()
            && rows.iter().all(|&r| {
                r < m.keypoints()
                    && m.kp_point[r] < 0
                    && used.insert(r)
                    && keyframes.insert(m.keyframe_of(r))
            });
        if !ok {
            return fail(
                ErrorKind::InvalidInput,
                "observers: each point needs free keypoint rows, used once, from different keyframes",
            );
        }
    }
    let first = m.points();
    for (point, rows) in xyz.iter().zip(observers) {
        let id = m.next_pt_id;
        m.next_pt_id += 1;
        let mut seen = rows.clone();
        seen.sort_unstable();
        for &r in &seen {
            m.kp_point[r] = id;
        }
        let creator = m.kf_id[m.keyframe_of(seen.last().copied().unwrap_or(0))];
        m.pt_id.push(id);
        m.pt_xyz.push(*point);
        m.pt_descriptor.push([0; 32]);
        m.pt_normal.push(Vector3::zeros());
        m.pt_min_distance.push(0.0);
        m.pt_max_distance.push(0.0);
        m.pt_visible.push(1);
        m.pt_found.push(1);
        m.pt_first_kf.push(creator);
        m.pt_reference_kf.push(creator);
        m.pt_observations.push(seen);
    }
    let rows: Vec<usize> = (first..m.points()).collect();
    refresh(m, &rows);
    Ok(())
}

/// Recomputes points' descriptors, normals and distances from their observations.
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The points' rows, each observed.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or a point with no observation.
pub fn refresh_points(m: &mut Map, rows: &[usize]) -> Result<()> {
    check_rows(rows, m.points(), "rows")?;
    if rows.iter().any(|&p| m.pt_observations[p].is_empty()) {
        return fail(
            ErrorKind::InvalidInput,
            "rows: every point must have an observation",
        );
    }
    refresh(m, rows);
    Ok(())
}

/// Removes points; the keypoints that observed them observe nothing.
///
/// # Arguments
///
/// * `m` - The map.
/// * `erase_flags` - One flag per point.
/// * `how` - Why they go.
///
/// # Errors
///
/// `InvalidInput` for flags that are not one per point.
pub fn erase_points(m: &mut Map, erase_flags: &[bool], how: Removal) -> Result<()> {
    if erase_flags.len() != m.points() {
        return fail(
            ErrorKind::InvalidInput,
            "erase: expected one flag per point",
        );
    }
    erase(m, erase_flags, how);
    Ok(())
}

/// Each point's observation weight: 1 per keypoint, 2 for one with a right column.
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - The points' rows.
///
/// # Returns
///
/// The weights.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
pub fn observation_weight(m: &Map, rows: &[usize]) -> Result<Vec<i64>> {
    check_rows(rows, m.points(), "rows")?;
    Ok(weights(m, rows))
}

/// Removes keypoints' observations; points left with weight 2 or less are erased, the others
/// refreshed (`EraseObservation`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `rows` - Keypoint rows.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
pub fn forget_observations(m: &mut Map, rows: &[usize]) -> Result<()> {
    check_rows(rows, m.keypoints(), "rows")?;
    forget(m, rows);
    Ok(())
}

/// Removes a keyframe and its keypoints; its points lose an observation each.
///
/// A keyframe that is still another's parent is refused, so every parent stays a keyframe of
/// the map: [`crate::covisibility::cull`] gives its children new parents first. It guards the
/// parents, not the root: a childless first keyframe is erased; callers above the map go through
/// `cull`, which refuses it. A keyframe with a loop edge is refused too (`cull` spares it). Its
/// pose relative to its parent is archived, for the final trajectory.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or a keyframe that is still a parent or has a loop
/// edge.
pub fn erase_keyframe(m: &mut Map, k: usize) -> Result<()> {
    if k >= m.keyframes() {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "k: must be from 0 to {}, got {k}",
                m.keyframes().saturating_sub(1)
            ),
        );
    }
    if let Some(child) = m.kf_parent.iter().position(|&p| p == m.kf_id[k]) {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "k: keyframe {} is still keyframe {}'s parent (covisibility's cull gives its \
                 children new parents first)",
                m.kf_id[k], m.kf_id[child]
            ),
        );
    }
    if has_loop(m, k) {
        return fail(
            ErrorKind::InvalidInput,
            format!("k: keyframe {} has a loop edge, and is kept", m.kf_id[k]),
        );
    }
    if has_merge(m, k) {
        return fail(
            ErrorKind::InvalidInput,
            format!("k: keyframe {} has a merge edge, and is kept", m.kf_id[k]),
        );
    }
    let relative = relative_to_parent(m, k);
    let (start, stop) = (m.kf_start[k], m.kf_start[k + 1]);
    let rows: Vec<usize> = (start..stop).collect();
    forget(m, &rows);
    let keep: Vec<bool> = (0..m.keypoints()).map(|r| r < start || r >= stop).collect();
    keep_where(&mut m.kp_xy, &keep);
    keep_where(&mut m.kp_level, &keep);
    keep_where(&mut m.kp_angle, &keep);
    keep_where(&mut m.kp_descriptor, &keep);
    keep_where(&mut m.kp_node, &keep);
    keep_where(&mut m.kp_bearing, &keep);
    keep_where(&mut m.kp_depth, &keep);
    keep_where(&mut m.kp_right, &keep);
    keep_where(&mut m.kp_point, &keep);
    m.gone_kf_id.push(m.kf_id[k]);
    m.gone_kf_time.push(m.kf_time[k]);
    m.gone_kf_pose.push(m.kf_pose[k]);
    m.gone_kf_parent.push(m.kf_parent[k]);
    m.gone_kf_relative.push(relative);
    m.kf_id.remove(k);
    m.kf_time.remove(k);
    m.kf_pose.remove(k);
    m.kf_parent.remove(k);
    m.kf_start.remove(k + 1);
    for start_of in &mut m.kf_start[k + 1..] {
        *start_of -= stop - start;
    }
    for seen in &mut m.pt_observations {
        for r in seen.iter_mut().filter(|r| **r >= stop) {
            *r -= stop - start;
        }
    }
    Ok(())
}

/// A keyframe's pose relative to its parent, `T_cam_parent`, or the identity.
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// The relative pose.
fn relative_to_parent(m: &Map, k: usize) -> Matrix4<f64> {
    let Some(parent) = (m.kf_parent[k] >= 0)
        .then(|| m.keyframe_row(m.kf_parent[k]))
        .flatten()
    else {
        return Matrix4::identity();
    };
    let (pose, of_parent) = (&m.kf_pose[k], &m.kf_pose[parent]);
    let rotation = pose.fixed_view::<3, 3>(0, 0) * of_parent.fixed_view::<3, 3>(0, 0).transpose();
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
    out.fixed_view_mut::<3, 1>(0, 3).copy_from(
        &(pose.fixed_view::<3, 1>(0, 3) - rotation * of_parent.fixed_view::<3, 1>(0, 3)),
    );
    out
}

/// Whether a keyframe has a loop edge (`slam_it.edge.map.has_loop`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row, in range.
///
/// # Returns
///
/// Whether a loop closed in the map joins it.
#[must_use]
pub fn has_loop(m: &Map, k: usize) -> bool {
    m.loop_edges.iter().any(|edge| edge.contains(&m.kf_id[k]))
}

/// Whether a keyframe has a merge edge (`slam_it.edge.map.has_merge`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row, in range.
///
/// # Returns
///
/// Whether a merge made in the map joins it.
#[must_use]
pub fn has_merge(m: &Map, k: usize) -> bool {
    m.merge_edges.iter().any(|edge| edge.contains(&m.kf_id[k]))
}

/// Adds a loop edge joining two of the map's keyframes (`AddLoopEdge`, both ways).
///
/// # Arguments
///
/// * `m` - The map.
/// * `first`, `second` - The two keyframes' ids, in either order.
///
/// # Errors
///
/// `InvalidInput` if either is not the map's, or they are the same.
pub fn add_loop_edge(m: &mut Map, first: i64, second: i64) -> Result<()> {
    let edge = joining(m, first, second, "loop")?;
    m.loop_edges.push(edge);
    Ok(())
}

/// Adds a merge edge joining two of the map's keyframes (`AddMergeEdge`, both ways).
///
/// # Arguments
///
/// * `m` - The map.
/// * `first`, `second` - The two keyframes' ids, in either order.
///
/// # Errors
///
/// `InvalidInput` if either is not the map's, or they are the same.
pub fn add_merge_edge(m: &mut Map, first: i64, second: i64) -> Result<()> {
    let edge = joining(m, first, second, "merge")?;
    m.merge_edges.push(edge);
    Ok(())
}

/// An edge joining two of the map's keyframes, the older first.
///
/// # Arguments
///
/// * `m` - The map.
/// * `first`, `second` - The two keyframes' ids, in either order.
/// * `what` - The edge's kind, for the message.
///
/// # Errors
///
/// `InvalidInput` if either is not the map's, or they are the same.
fn joining(m: &Map, first: i64, second: i64, what: &str) -> Result<[i64; 2]> {
    if first == second || m.keyframe_row(first).is_none() || m.keyframe_row(second).is_none() {
        return fail(
            ErrorKind::InvalidInput,
            format!("{what} edge {first}-{second}: expected two of the map's keyframes"),
        );
    }
    Ok([first.min(second), first.max(second)])
}

/// Replaces one point by another (`slam_it.edge.map.replace_point`, `MapPoint::Replace`).
///
/// The lost point's observations pass to the kept one, except in keyframes that already see
/// the kept one, where they are dropped; its visible and found counts are added to the kept
/// one's; it is erased ([`Removal::Fused`]), and the kept one refreshed. Local mapping's fusion
/// and loop closing replace points through it.
///
/// # Arguments
///
/// * `m` - The map.
/// * `lose` - The lost point's id.
/// * `keep` - The kept point's id.
///
/// # Errors
///
/// `InvalidInput` if either point is not in the map, or they are the same.
pub fn replace_point(m: &mut Map, lose: i64, keep: i64) -> Result<()> {
    let (Some(p_lose), Some(p_keep)) = (m.point_row(lose), m.point_row(keep)) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("replace {lose} by {keep}: expected two of the map's points"),
        );
    };
    if lose == keep {
        return fail(
            ErrorKind::InvalidInput,
            format!("replace {lose} by {keep}: expected two of the map's points"),
        );
    }
    let kept_in: BTreeSet<usize> = m.pt_observations[p_keep]
        .iter()
        .map(|&r| m.keyframe_of(r))
        .collect();
    for r in m.pt_observations[p_lose].clone() {
        let to = if kept_in.contains(&m.keyframe_of(r)) {
            -1
        } else {
            keep
        };
        observe(m, r, to)?;
    }
    m.pt_visible[p_keep] += m.pt_visible[p_lose];
    m.pt_found[p_keep] += m.pt_found[p_lose];
    let mut flags = vec![false; m.points()];
    flags[p_lose] = true;
    erase(m, &flags, Removal::Fused);
    let kept: Vec<usize> = m.point_row(keep).into_iter().collect();
    refresh_points(m, &kept)
}

/// The row of the keyframe with an id, refusing one the map does not hold
/// (`slam_it.edge.map.keyframe_row`).
///
/// # Arguments
///
/// * `m` - The map.
/// * `keyframe` - The keyframe's id.
///
/// # Returns
///
/// Its row.
///
/// # Errors
///
/// `InvalidInput` if the map holds no keyframe with that id.
pub fn keyframe_row(m: &Map, keyframe: i64) -> Result<usize> {
    match m.keyframe_row(keyframe) {
        Some(k) => Ok(k),
        None => fail(
            ErrorKind::InvalidInput,
            format!("keyframe {keyframe} is not in the map"),
        ),
    }
}

/// Points a keypoint at a point, or at none, keeping the observation lists.
///
/// # Arguments
///
/// * `m` - The map.
/// * `row` - The keypoint's row.
/// * `id` - The point's id, or -1.
///
/// # Errors
///
/// `InvalidInput` for a row out of range or an id not in the map.
pub fn observe(m: &mut Map, row: usize, id: i64) -> Result<()> {
    check_rows(&[row], m.keypoints(), "row")?;
    let p = m.point_row(id);
    if id != -1 && p.is_none() {
        return fail(
            ErrorKind::InvalidInput,
            "id: expected -1 or a point in the map",
        );
    }
    detach(m, row);
    if let Some(p) = p {
        attach(m, row, p);
    }
    Ok(())
}

/// Adds to points' visible and found counts (ids no longer in the map are skipped).
///
/// # Arguments
///
/// * `m` - The map.
/// * `ids` - Point ids.
/// * `visible` - Each id's visible increment, non-negative.
/// * `found` - Each id's found increment, non-negative.
///
/// # Errors
///
/// `InvalidInput` for increments that are negative or not one per id.
pub fn count(m: &mut Map, ids: &[i64], visible: &[i64], found: &[i64]) -> Result<()> {
    if visible.len() != ids.len()
        || found.len() != ids.len()
        || visible.iter().chain(found).any(|&v| v < 0)
    {
        return fail(
            ErrorKind::InvalidInput,
            "visible, found: expected one non-negative increment per id",
        );
    }
    for ((&id, &v), &f) in ids.iter().zip(visible).zip(found) {
        if let Some(p) = m.point_row(id) {
            m.pt_visible[p] += v;
            m.pt_found[p] += f;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Native tests on a small hand-built map; parity with the Python reference is checked
    //! through the bindings by tests/edge.

    use super::*;
    use slam_it_foundations::orb::{Features, Keypoint};

    /// A frame of `n` keypoints along a row, each a distinct descriptor, with depth if asked.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        reason = "small n"
    )]
    fn frame_of(n: usize, with_depth: bool) -> Frame {
        let mut keypoints = Vec::new();
        let mut descriptors = Vec::new();
        let mut bearings = Vec::new();
        for i in 0..n {
            let x = 100.0 + 10.0 * i as f64;
            keypoints.push(Keypoint {
                xy: [x, 120.0],
                level: 0,
                level_xy: [0, 0],
                response: 0,
                angle: 0,
            });
            let mut d = [0_u8; 32];
            d[i % 32] = 1 + i as u8;
            descriptors.push(d);
            bearings.push(Vector3::new((x - 160.0) / 250.0, 0.0, 1.0).normalize());
        }
        let features = Features {
            keypoints,
            descriptors,
            scale_factor: 1.2,
            levels: 8,
        };
        let depth = vec![if with_depth { 2.0 } else { f64::NAN }; n];
        let right = (0..n)
            .map(|i| {
                if with_depth {
                    95.0 + 10.0 * i as f64
                } else {
                    f64::NAN
                }
            })
            .collect();
        Frame::new(0.0, features, bearings, depth, right).unwrap()
    }

    /// The first keyframe's vocabulary nodes, each keypoint its own.
    const FIRST_NODES: [i64; 4] = [0, 1, 2, 3];

    /// The second's.
    const SECOND_NODES: [i64; 4] = [10, 11, 12, 13];

    /// A map of two keyframes: the first with depth creating two points, the second seeing them.
    fn two_keyframes() -> Map {
        let mut m = Map::empty(8, 1.2);
        let none = [-1; 4];
        insert_keyframe(
            &mut m,
            &frame_of(4, true),
            &Matrix4::identity(),
            &none,
            &FIRST_NODES,
        )
        .unwrap();
        let xyz = [Vector3::new(0.0, 0.0, 2.0), Vector3::new(0.1, 0.0, 2.0)];
        add_points(&mut m, &xyz, &[vec![0], vec![1]]).unwrap();
        let mut moved = Matrix4::identity();
        moved[(0, 3)] = -0.2;
        insert_keyframe(
            &mut m,
            &frame_of(4, false),
            &moved,
            &[0, 1, -1, -1],
            &SECOND_NODES,
        )
        .unwrap();
        m
    }

    /// Each keypoint's vocabulary node goes where its keypoint goes: inserted as given, refused
    /// when not one per keypoint or negative, kept by a sub-map and by the keyframes a culled one
    /// leaves (a merge's are held to Python's by `tests/edge/test_merges.py`).
    #[test]
    fn a_keypoint_keeps_its_node() {
        let mut m = two_keyframes();
        assert_eq!(m.kp_node, [FIRST_NODES, SECOND_NODES].concat());
        let none = [-1; 4];
        for nodes in [&[0_i64, 1, 2][..], &[0, 1, -1, 3]] {
            let refused = insert_keyframe(
                &mut m,
                &frame_of(4, false),
                &Matrix4::identity(),
                &none,
                nodes,
            );
            assert_eq!(refused.unwrap_err().kind, ErrorKind::InvalidInput);
        }
        assert_eq!(m.keyframes(), 2);
        let second = sub_map(&m, &[1]);
        assert_eq!(second.kp_node, SECOND_NODES);
        check_map(&second).unwrap();
        let mut negative = m.clone();
        negative.kp_node[5] = -1;
        assert_eq!(
            check_map(&negative).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let mut shorter = m.clone();
        shorter.kp_node.pop();
        assert_eq!(
            check_map(&shorter).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        erase_keyframe(&mut m, 0).unwrap();
        assert_eq!(m.kp_node, SECOND_NODES);
    }

    /// Building a map gives ids, links, counts, normals and distances.
    #[test]
    fn building_links_points_and_keyframes() {
        let mut m = two_keyframes();
        assert_eq!(m.pt_id, vec![0, 1]);
        assert_eq!(m.kf_start, vec![0, 4, 8]);
        assert_eq!((m.kp_point[0], m.kp_point[4]), (0, 0));
        assert!((m.pt_normal[0].norm() - 1.0).abs() < 1e-12);
        assert_eq!(observation_weight(&m, &[0, 1]).unwrap(), vec![3, 3]);
        count(&mut m, &[0, 0, 7], &[1, 1, 1], &[0, 1, 1]).unwrap();
        assert_eq!((m.pt_visible[0], m.pt_found[0]), (3, 2));
        check_map(&m).unwrap();
    }

    /// Forgetting erases points left with weight 2 or less; erasing a keyframe moves the next up.
    #[test]
    fn removing_drops_weak_points_and_rows() {
        let mut m = two_keyframes();
        let first = m.pt_xyz[0];
        forget_observations(&mut m, &[4]).unwrap();
        assert_eq!(m.pt_id, vec![1]);
        assert_eq!(m.kp_point[0], -1);
        // The erased point is kept aside, where it was, as too weakly seen.
        assert_eq!(
            (
                m.gone_pt_id.clone(),
                m.gone_pt_xyz.clone(),
                m.gone_pt_how.clone()
            ),
            (vec![0], vec![first], vec![Removal::Weak.code()])
        );
        let time = m.kf_time[0];
        erase_keyframe(&mut m, 0).unwrap();
        assert_eq!(m.kf_start, vec![0, 4]);
        assert_eq!(m.points(), 0);
        // The erased keyframe too, with the point it left too weakly seen.
        assert_eq!(
            (m.gone_kf_id.clone(), m.gone_kf_time.clone()),
            (vec![0], vec![time])
        );
        assert_eq!(m.gone_pt_id, vec![0, 1]);
        check_map(&m).unwrap();
        clear_gone(&mut m);
        assert!(m.gone_pt_id.is_empty() && m.gone_kf_id.is_empty() && m.gone_kf_pose.is_empty());
        assert_eq!(
            erase_keyframe(&mut m, 1).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }

    /// The observation lists follow the operations; `check_map` refuses a stale one.
    #[test]
    fn the_index_follows_the_operations() {
        let mut m = two_keyframes();
        assert_eq!(m.pt_observations, vec![vec![0, 4], vec![1, 5]]);
        let mut further = Matrix4::identity();
        further[(0, 3)] = -0.4;
        insert_keyframe(
            &mut m,
            &frame_of(4, false),
            &further,
            &[0, 1, -1, -1],
            &SECOND_NODES,
        )
        .unwrap();
        erase_keyframe(&mut m, 1).unwrap();
        assert_eq!(m.pt_observations, vec![vec![0, 4], vec![1, 5]]);
        check_map(&m).unwrap();
        observe(&mut m, 4, -1).unwrap();
        assert_eq!((m.kp_point[4], m.pt_observations[0].clone()), (-1, vec![0]));
        observe(&mut m, 4, 0).unwrap();
        assert_eq!(m.pt_observations[0], vec![0, 4]);
        check_map(&m).unwrap();
        for (row, id) in [(8, 0), (2, 7)] {
            assert_eq!(
                observe(&mut m, row, id).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
        let mut erased = m.clone();
        erase_points(&mut erased, &[false, true], Removal::Fused).unwrap();
        assert_eq!(erased.pt_observations, vec![vec![0, 4]]);
        check_map(&erased).unwrap();
        let mut stale = m.clone();
        stale.pt_observations[0].pop();
        assert_eq!(check_map(&stale).unwrap_err().kind, ErrorKind::InvalidInput);
        index_observations(&mut stale);
        assert_eq!(stale.pt_observations, m.pt_observations);
    }

    /// Malformed maps and arguments are `InvalidInput`.
    /// Loop and merge edges are kept older first, apart, refused between a keyframe and itself or
    /// a stranger, and their keyframes never erased; an erased keyframe keeps its pose relative to
    /// its parent.
    #[test]
    fn loop_edges_and_relative_poses() {
        let mut m = two_keyframes();
        let mut further = Matrix4::identity();
        further[(0, 3)] = -0.4;
        further[(1, 3)] = 0.1;
        insert_keyframe(
            &mut m,
            &frame_of(4, false),
            &further,
            &[0, 1, -1, -1],
            &SECOND_NODES,
        )
        .unwrap();
        m.kf_parent = vec![-1, 0, 0];
        let mut looped = m.clone();
        add_loop_edge(&mut looped, 1, 0).unwrap();
        assert_eq!(looped.loop_edges, vec![[0, 1]]);
        assert!(has_loop(&looped, 0) && has_loop(&looped, 1) && !has_loop(&looped, 2));
        check_map(&looped).unwrap();
        assert!(add_loop_edge(&mut looped, 1, 1).is_err());
        assert!(add_loop_edge(&mut looped, 1, 9).is_err());
        assert!(erase_keyframe(&mut looped, 1).is_err());
        let mut backwards = m.clone();
        backwards.loop_edges = vec![[1, 0]];
        assert!(check_map(&backwards).is_err());
        let mut merged = m.clone();
        add_merge_edge(&mut merged, 1, 0).unwrap();
        assert_eq!(merged.merge_edges, vec![[0, 1]]);
        assert!(merged.loop_edges.is_empty() && !has_loop(&merged, 1));
        assert!(has_merge(&merged, 0) && has_merge(&merged, 1) && !has_merge(&merged, 2));
        check_map(&merged).unwrap();
        assert!(add_merge_edge(&mut merged, 1, 1).is_err());
        assert!(add_merge_edge(&mut merged, 1, 9).is_err());
        assert!(erase_keyframe(&mut merged, 1).is_err());
        let mut backwards = m.clone();
        backwards.merge_edges = vec![[1, 0]];
        assert!(check_map(&backwards).is_err());
        let expected = m.kf_pose[2] * m.kf_pose[0].try_inverse().unwrap();
        erase_keyframe(&mut m, 2).unwrap();
        assert!((m.gone_kf_relative[0] - expected).amax() < 1e-12);
    }

    #[test]
    fn rejects_invalid_input() {
        let m = two_keyframes();
        let mut dangling = m.clone();
        dangling.kp_point[2] = 9;
        assert_eq!(
            check_map(&dangling).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        for parent in [m.kf_id[1], m.next_kf_id + 5, -2] {
            let mut bad = m.clone(); // a parent that is itself, not in the map, or below -1
            bad.kf_parent[1] = parent;
            assert_eq!(check_map(&bad).unwrap_err().kind, ErrorKind::InvalidInput);
        }
        let mut again = m.clone();
        let result = add_points(&mut again, &[Vector3::new(0.0, 0.0, 1.0)], &[vec![2, 3]]);
        assert_eq!(result.unwrap_err().kind, ErrorKind::InvalidInput);
        let result = insert_keyframe(
            &mut again,
            &frame_of(4, true),
            &Matrix4::identity(),
            &[5, -1, -1, -1],
            &FIRST_NODES,
        );
        assert_eq!(result.unwrap_err().kind, ErrorKind::InvalidInput);
        assert_eq!(
            distinctive_descriptor(&[]).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }
}
