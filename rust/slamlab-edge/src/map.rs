//! The map: the Rust core's mirror of `slamlab.edge.map`.
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

use std::collections::BTreeSet;

use nalgebra::{Matrix4, Vector3};
use slamlab_foundations::error::{ErrorKind, Result, check_range, fail, fail_setting};
use slamlab_foundations::matching;
use slamlab_foundations::orb::{Descriptor, MAX_LEVELS};

use crate::frames::Frame;
use crate::options::Options;

/// Keyframes, their keypoints and the map points (`slamlab.edge.map.Map`).
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
            kf_start: vec![0],
            kp_xy: Vec::new(),
            kp_level: Vec::new(),
            kp_angle: Vec::new(),
            kp_descriptor: Vec::new(),
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
        let pose = &self.kf_pose[k];
        -(pose.fixed_view::<3, 3>(0, 0).transpose() * pose.fixed_view::<3, 1>(0, 3))
    }

    /// The sliding window: the rows of the last `size` keyframes, oldest first.
    ///
    /// # Arguments
    ///
    /// * `size` - The window's size.
    ///
    /// # Returns
    ///
    /// The rows.
    #[must_use]
    pub fn window(&self, size: usize) -> Vec<usize> {
        (self.keyframes().saturating_sub(size)..self.keyframes()).collect()
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
        m.kf_start.len() == k + 1,
        m.kp_xy.len() == t,
        m.kp_level.len() == t,
        m.kp_angle.len() == t,
        m.kp_descriptor.len() == t,
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
    if m.kp_point
        .iter()
        .any(|&id| id < -1 || (id >= 0 && m.point_row(id).is_none()))
    {
        return fail(
            ErrorKind::InvalidInput,
            "map.kp_point: every observed point must be in the map",
        );
    }
    if m.pt_observations != indexed(m) {
        return fail(
            ErrorKind::InvalidInput,
            "map.pt_observations: expected each point's observing keypoint rows, ascending",
        );
    }
    Ok(())
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

/// Whether a point lies within the distances it is matchable from
/// (`slamlab.edge.map.in_distance_range`): from `min_distance_margin` of its minimum distance
/// to `max_distance_margin` of its maximum (`isInFrustum`, `Fuse`), unchecked.
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
    distance >= options.min_distance_margin * m.pt_min_distance[p]
        && distance <= options.max_distance_margin * m.pt_max_distance[p]
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
        let mut row: Vec<i64> = descriptors
            .iter()
            .map(|b| matching::distance(a, b))
            .collect();
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

/// Removes the flagged points, unchecked.
///
/// # Arguments
///
/// * `m` - The map.
/// * `erase` - One flag per point.
fn erase(m: &mut Map, erase: &[bool]) {
    if !erase.contains(&true) {
        return;
    }
    for (p, _) in erase.iter().enumerate().filter(|&(_, &e)| e) {
        for &r in &m.pt_observations[p] {
            m.kp_point[r] = -1;
        }
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
    erase(m, &weak);
}

/// Adds a keyframe and refreshes the points its keypoints observe.
///
/// # Arguments
///
/// * `m` - The map.
/// * `frame` - The frame (its pyramid the map's).
/// * `pose` - Its `T_cam_world`.
/// * `points` - The id each keypoint observes (each in the map, at most once), or -1.
///
/// # Errors
///
/// `InvalidInput` for another pyramid, a pose that is not finite, or ids that are not one per
/// keypoint, in the map, and distinct.
#[allow(
    clippy::float_cmp,
    reason = "the frame's scale factor must be the map's exactly, as in Python"
)]
pub fn insert_keyframe(
    m: &mut Map,
    frame: &Frame,
    pose: &Matrix4<f64>,
    points: &[i64],
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
    m.kf_start.push(first + n);
    for (i, k) in frame.features.keypoints.iter().enumerate() {
        m.kp_xy.push(k.xy);
        m.kp_level.push(k.level);
        m.kp_angle.push(k.angle);
        m.kp_descriptor.push(frame.features.descriptors[i]);
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
///
/// # Errors
///
/// `InvalidInput` for flags that are not one per point.
pub fn erase_points(m: &mut Map, erase_flags: &[bool]) -> Result<()> {
    if erase_flags.len() != m.points() {
        return fail(
            ErrorKind::InvalidInput,
            "erase: expected one flag per point",
        );
    }
    erase(m, erase_flags);
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
/// # Arguments
///
/// * `m` - The map.
/// * `k` - The keyframe's row.
///
/// # Errors
///
/// `InvalidInput` for a row out of range.
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
    let (start, stop) = (m.kf_start[k], m.kf_start[k + 1]);
    let rows: Vec<usize> = (start..stop).collect();
    forget(m, &rows);
    let keep: Vec<bool> = (0..m.keypoints()).map(|r| r < start || r >= stop).collect();
    keep_where(&mut m.kp_xy, &keep);
    keep_where(&mut m.kp_level, &keep);
    keep_where(&mut m.kp_angle, &keep);
    keep_where(&mut m.kp_descriptor, &keep);
    keep_where(&mut m.kp_bearing, &keep);
    keep_where(&mut m.kp_depth, &keep);
    keep_where(&mut m.kp_right, &keep);
    keep_where(&mut m.kp_point, &keep);
    m.kf_id.remove(k);
    m.kf_time.remove(k);
    m.kf_pose.remove(k);
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
    use slamlab_foundations::orb::{Features, Keypoint};

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
        Frame {
            timestamp: 0.0,
            features: Features {
                keypoints,
                descriptors,
                scale_factor: 1.2,
                levels: 8,
            },
            bearings,
            depth: vec![if with_depth { 2.0 } else { f64::NAN }; n],
            right: (0..n)
                .map(|i| {
                    if with_depth {
                        95.0 + 10.0 * i as f64
                    } else {
                        f64::NAN
                    }
                })
                .collect(),
        }
    }

    /// A map of two keyframes: the first with depth creating two points, the second seeing them.
    fn two_keyframes() -> Map {
        let mut m = Map::empty(8, 1.2);
        insert_keyframe(&mut m, &frame_of(4, true), &Matrix4::identity(), &[-1; 4]).unwrap();
        let xyz = [Vector3::new(0.0, 0.0, 2.0), Vector3::new(0.1, 0.0, 2.0)];
        add_points(&mut m, &xyz, &[vec![0], vec![1]]).unwrap();
        let mut moved = Matrix4::identity();
        moved[(0, 3)] = -0.2;
        insert_keyframe(&mut m, &frame_of(4, false), &moved, &[0, 1, -1, -1]).unwrap();
        m
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
        forget_observations(&mut m, &[4]).unwrap();
        assert_eq!(m.pt_id, vec![1]);
        assert_eq!(m.kp_point[0], -1);
        erase_keyframe(&mut m, 0).unwrap();
        assert_eq!(m.kf_start, vec![0, 4]);
        assert_eq!(m.points(), 0);
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
        insert_keyframe(&mut m, &frame_of(4, false), &further, &[0, 1, -1, -1]).unwrap();
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
        erase_points(&mut erased, &[false, true]).unwrap();
        assert_eq!(erased.pt_observations, vec![vec![0, 4]]);
        check_map(&erased).unwrap();
        let mut stale = m.clone();
        stale.pt_observations[0].pop();
        assert_eq!(check_map(&stale).unwrap_err().kind, ErrorKind::InvalidInput);
        index_observations(&mut stale);
        assert_eq!(stale.pt_observations, m.pt_observations);
    }

    /// Malformed maps and arguments are `InvalidInput`.
    #[test]
    fn rejects_invalid_input() {
        let m = two_keyframes();
        let mut dangling = m.clone();
        dangling.kp_point[2] = 9;
        assert_eq!(
            check_map(&dangling).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let mut again = m.clone();
        let result = add_points(&mut again, &[Vector3::new(0.0, 0.0, 1.0)], &[vec![2, 3]]);
        assert_eq!(result.unwrap_err().kind, ErrorKind::InvalidInput);
        let result = insert_keyframe(
            &mut again,
            &frame_of(4, true),
            &Matrix4::identity(),
            &[5, -1, -1, -1],
        );
        assert_eq!(result.unwrap_err().kind, ErrorKind::InvalidInput);
        assert_eq!(
            distinctive_descriptor(&[]).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }
}
