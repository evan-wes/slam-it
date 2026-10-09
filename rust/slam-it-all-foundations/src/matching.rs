//! Matching binary descriptors: the Rust core's mirror of `slam_it_all.foundations.matching`.
//!
//! Hamming distances, and ORB-SLAM's filtered nearest neighbours: a distance cap, Lowe's ratio
//! test, one match per second-set descriptor, and the rotation-consistency histogram; and
//! guided matching, a search by projection over a feature grid (the Python module's docstring
//! defines each filter).

use crate::error::{ErrorKind, Result, check_range, fail, fail_setting};
use crate::orb::{ANGLE_BINS, DESCRIPTOR_BYTES, Descriptor, Features, Keypoint, MAX_LEVELS};

/// The most descriptors a set may hold.
pub const MAX_DESCRIPTORS: usize = 1 << 20;
/// The largest Hamming distance: every bit differs.
pub const MAX_DISTANCE: i64 = 8 * 32;
const _: () = assert!(DESCRIPTOR_BYTES == 32, "update MAX_DISTANCE");
/// The rotation histogram's second and third bins are kept only if they hold at least this
/// fraction of the highest (ORB-SLAM's `ComputeThreeMaxima`).
pub const SECONDARY_FRACTION: f64 = 0.1;

/// The feature grid's cells across the keypoints' bounding box (ORB-SLAM's `FRAME_GRID_COLS`).
pub const GRID_COLS: usize = 64;
/// Its cells down the box (ORB-SLAM's `FRAME_GRID_ROWS`).
pub const GRID_ROWS: usize = 48;

/// Matching filters (`slam_it_all.foundations.matching.Options`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// The largest Hamming distance a match may have, from 0 to [`MAX_DISTANCE`].
    pub max_distance: i64,
    /// The ratio test's bound, in (0, 1].
    pub ratio: f64,
    /// Keep only matches in the rotation histogram's three highest bins.
    pub check_orientation: bool,
}

impl Default for Options {
    /// ORB-SLAM's distance cap, a ratio of 0.8, and the rotation check.
    fn default() -> Self {
        Self {
            max_distance: 50,
            ratio: 0.8,
            check_orientation: true,
        }
    }
}

/// The filters of a search by projection (`slam_it_all.foundations.matching.SearchOptions`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SearchOptions {
    /// The largest Hamming distance a match may have, from 0 to [`MAX_DISTANCE`].
    pub max_distance: i64,
    /// The one-level ratio test's bound, in (0, 1]; 1 disables it.
    pub ratio: f64,
    /// Keep only matches in the rotation histogram's three highest bins.
    pub check_orientation: bool,
}

impl Default for SearchOptions {
    /// ORB-SLAM's search from the last frame: `TH_HIGH`, no ratio test, the rotation check.
    fn default() -> Self {
        Self {
            max_distance: 100,
            ratio: 1.0,
            check_orientation: true,
        }
    }
}

/// One map point projected into a frame (a row of `slam_it_all.foundations.matching.Projections`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Projection {
    /// The projection, in the full image's pixels.
    pub xy: [f64; 2],
    /// The square search window's half-width, in pixels, positive.
    pub radius: f64,
    /// The lowest pyramid level a match may lie on.
    pub min_level: i64,
    /// The highest (an empty range finds nothing).
    pub max_level: i64,
    /// The map point's descriptor.
    pub descriptor: Descriptor,
    /// The orientation bin of the observation the descriptor came from.
    pub angle: usize,
    /// The predicted right-image column (stereo, RGB-D); NaN for none.
    pub right: f64,
}

/// One match.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Match {
    /// The first-set index.
    pub first: usize,
    /// The second-set index.
    pub second: usize,
    /// The Hamming distance.
    pub distance: i64,
}

/// Checks matching filters.
///
/// # Arguments
///
/// * `options` - The filters.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
pub fn check_options(options: &Options) -> Result<()> {
    check_range("max_distance", options.max_distance, 0, MAX_DISTANCE)?;
    if !(options.ratio.is_finite() && options.ratio > 0.0 && options.ratio <= 1.0) {
        return fail_setting("ratio", "must be in (0, 1]", options.ratio);
    }
    Ok(())
}

/// The Hamming distance between two descriptors: the bits in which they differ.
///
/// # Arguments
///
/// * `a` - A descriptor.
/// * `b` - Another.
///
/// # Returns
///
/// From 0 to 256.
#[must_use]
pub fn distance(a: &Descriptor, b: &Descriptor) -> i64 {
    a.as_chunks::<8>()
        .0
        .iter()
        .zip(b.as_chunks::<8>().0)
        .map(|(x, y)| i64::from((u64::from_le_bytes(*x) ^ u64::from_le_bytes(*y)).count_ones()))
        .sum()
}

/// Every pair's Hamming distance.
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
///
/// # Returns
///
/// The distances, row-major (first set by second set).
///
/// # Errors
///
/// `InvalidInput` for a set of more than [`MAX_DESCRIPTORS`].
pub fn distances(descriptors1: &[Descriptor], descriptors2: &[Descriptor]) -> Result<Vec<i64>> {
    if descriptors1.len() > MAX_DESCRIPTORS || descriptors2.len() > MAX_DESCRIPTORS {
        return fail(ErrorKind::InvalidInput, "descriptors: too many descriptors");
    }
    Ok(descriptors1
        .iter()
        .flat_map(|a| descriptors2.iter().map(move |b| distance(a, b)))
        .collect())
}

/// Checks a descriptor set's size and its angles.
///
/// # Arguments
///
/// * `count` - The number of descriptors.
/// * `angles` - Their orientation bins.
/// * `name` - The angles' name, for the message.
///
/// # Errors
///
/// `InvalidInput` for too many descriptors, or angles that are not one bin per descriptor.
fn check_set(count: usize, angles: &[usize], name: &str) -> Result<()> {
    if count > MAX_DESCRIPTORS {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: too many descriptors"),
        );
    }
    if angles.len() != count {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected {count} integers, got {}", angles.len()),
        );
    }
    if angles.iter().any(|&a| a >= ANGLE_BINS) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: bins must lie in [0, {ANGLE_BINS})"),
        );
    }
    Ok(())
}

/// A descriptor's nearest index and distance in a set, and the second-nearest distance (the
/// best again on a tie); ties go to the lowest index.
///
/// # Arguments
///
/// * `query` - The descriptor.
/// * `set` - The set, non-empty.
///
/// # Returns
///
/// `(index, best, second)`.
fn nearest(query: &Descriptor, set: &[Descriptor]) -> (usize, i64, i64) {
    let mut out = (0, i64::MAX, i64::MAX);
    for (j, candidate) in set.iter().enumerate() {
        let d = distance(query, candidate);
        if d < out.1 {
            out = (j, d, out.1);
        } else if d < out.2 {
            out.2 = d;
        }
    }
    out
}

/// The rotation histogram's kept bins: its highest three, the lower two only if they hold at
/// least a tenth of the highest (ORB-SLAM's `ComputeThreeMaxima`).
///
/// # Arguments
///
/// * `counts` - The histogram.
///
/// # Returns
///
/// The kept bins, highest first.
#[allow(
    clippy::cast_precision_loss,
    reason = "match counts are at most 2^20, exact in f64"
)]
fn three_maxima(counts: &[i64; ANGLE_BINS]) -> Vec<usize> {
    let mut top: Vec<(i64, usize)> = Vec::with_capacity(4);
    for (k, &count) in counts.iter().enumerate() {
        let place = top
            .iter()
            .position(|&(c, _)| count > c)
            .unwrap_or(top.len());
        top.insert(place, (count, k));
        top.truncate(3);
    }
    let highest = top[0].0;
    let mut kept = vec![top[0].1];
    for &(count, k) in &top[1..] {
        if (count as f64) < SECONDARY_FRACTION * highest as f64 {
            break;
        }
        kept.push(k);
    }
    kept
}

/// Matches each descriptor of the first set to its nearest in the second, filtered (Python's
/// and C++'s `match`; `match` is a Rust keyword).
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
/// * `angles1` - The first set's orientation bins.
/// * `angles2` - The second set's.
/// * `options` - The filters.
///
/// # Returns
///
/// The kept matches, by first-set index.
///
/// # Errors
///
/// `InvalidInput` for a bad filter, a set too large, or angles that are not one bin in
/// `[0, ANGLE_BINS)` per descriptor.
#[allow(
    clippy::cast_precision_loss,
    reason = "Hamming distances are at most 256, exact in f64"
)]
pub fn match_descriptors(
    descriptors1: &[Descriptor],
    descriptors2: &[Descriptor],
    angles1: &[usize],
    angles2: &[usize],
    options: &Options,
) -> Result<Vec<Match>> {
    check_options(options)?;
    check_set(descriptors1.len(), angles1, "angles1")?;
    check_set(descriptors2.len(), angles2, "angles2")?;
    if descriptors1.is_empty() || descriptors2.is_empty() {
        return Ok(Vec::new());
    }
    // Each second-set descriptor's claimant: the nearest first-set match passing the filters
    // (ties: the lowest first-set index, which is offered first).
    let mut claims = Claims::new(descriptors2.len());
    for (i, query) in descriptors1.iter().enumerate() {
        let (j, best, second) = nearest(query, descriptors2);
        let passes = descriptors2.len() == 1 || (best as f64) < options.ratio * second as f64;
        if passes && best <= options.max_distance {
            claims.offer(Match {
                first: i,
                second: j,
                distance: best,
            });
        }
    }
    let mut out = claims.by_first();
    if options.check_orientation {
        keep_consistent_rotations(&mut out, |i| angles1[i], |j| angles2[j]);
    }
    Ok(out)
}

/// Each second-side item's claimant: the nearest match offered for it, the first offered on a
/// tie (so the lowest first-side index, when offered in order).
struct Claims(Vec<Option<Match>>);

impl Claims {
    /// No claims yet.
    ///
    /// # Arguments
    ///
    /// * `count` - The second side's items.
    ///
    /// # Returns
    ///
    /// The empty claims.
    fn new(count: usize) -> Self {
        Self(vec![None; count])
    }

    /// Offers a match; it holds its item unless a nearer one already does.
    ///
    /// # Arguments
    ///
    /// * `m` - The match.
    fn offer(&mut self, m: Match) {
        let held = &mut self.0[m.second];
        if held.is_none_or(|h| m.distance < h.distance) {
            *held = Some(m);
        }
    }

    /// The holding matches.
    ///
    /// # Returns
    ///
    /// Them, by first-side index.
    fn by_first(self) -> Vec<Match> {
        let mut out: Vec<Match> = self.0.into_iter().flatten().collect();
        out.sort_by_key(|m| m.first);
        out
    }
}

/// Keeps only matches whose rotation bin is among the histogram's three highest.
///
/// # Arguments
///
/// * `matches` - The matches.
/// * `angles1` - The first side's orientation bin, by [`Match::first`].
/// * `angles2` - The second side's, by [`Match::second`].
fn keep_consistent_rotations(
    matches: &mut Vec<Match>,
    angles1: impl Fn(usize) -> usize,
    angles2: impl Fn(usize) -> usize,
) {
    if matches.is_empty() {
        return;
    }
    let rotation = |m: &Match| (angles1(m.first) + ANGLE_BINS - angles2(m.second)) % ANGLE_BINS;
    let mut counts = [0_i64; ANGLE_BINS];
    for m in matches.iter() {
        counts[rotation(m)] += 1;
    }
    let kept = three_maxima(&counts);
    matches.retain(|m| kept.contains(&rotation(m)));
}

/// A frame's keypoints bucketed into [`GRID_COLS`] x [`GRID_ROWS`] cells over their bounding
/// box (ORB-SLAM's grid), so that a window reads only the cells it covers.
///
/// A position's cell is `floor((x - x_min) * scale)` along each axis, clamped to the grid,
/// with `scale` the cells over the box's width (0 for a box of no width, or one too narrow for
/// a finite scale; see [`Axis`]). The map is monotone in `x`, and IEEE rounding is monotone too, so the cells
/// of a window's two floating-point ends `u - r` and `u + r` bound the cells of every keypoint
/// that passes the window's test `|x - u| < r`: the grid finds exactly the keypoints a brute
/// force finds.
#[derive(Debug, Clone, PartialEq)]
pub struct FeatureGrid {
    /// The keypoints' columns.
    x: Vec<f64>,
    /// Their rows.
    y: Vec<f64>,
    /// Their levels.
    level: Vec<i64>,
    /// The grid across.
    x_axis: Axis,
    /// The grid down.
    y_axis: Axis,
    /// Where each cell's keypoints start in `items`, cell by cell, row-major, with one past the
    /// end last.
    start: Vec<u32>,
    /// The keypoints, cell by cell, ascending within each.
    items: Vec<u32>,
}

impl FeatureGrid {
    /// Buckets keypoints.
    ///
    /// # Arguments
    ///
    /// * `keypoints` - The keypoints (positions and levels are read).
    ///
    /// # Returns
    ///
    /// The grid.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for a position that is not finite, or more than [`MAX_DESCRIPTORS`]
    /// keypoints.
    pub fn build(keypoints: &[Keypoint]) -> Result<Self> {
        if keypoints.len() > MAX_DESCRIPTORS {
            return fail(ErrorKind::InvalidInput, "xy: too many keypoints");
        }
        if keypoints
            .iter()
            .any(|k| !(k.xy[0].is_finite() && k.xy[1].is_finite()))
        {
            return fail(ErrorKind::InvalidInput, "xy: expected finite values");
        }
        let x: Vec<f64> = keypoints.iter().map(|k| k.xy[0]).collect();
        let y: Vec<f64> = keypoints.iter().map(|k| k.xy[1]).collect();
        let (x_axis, y_axis) = (Axis::new(&x, GRID_COLS), Axis::new(&y, GRID_ROWS));
        let cells: Vec<usize> = x
            .iter()
            .zip(&y)
            .map(|(&u, &v)| y_axis.cell(v) * GRID_COLS + x_axis.cell(u))
            .collect();
        let mut start = vec![0_u32; GRID_COLS * GRID_ROWS + 1];
        for &c in &cells {
            start[c + 1] += 1;
        }
        for c in 0..GRID_COLS * GRID_ROWS {
            start[c + 1] += start[c];
        }
        let mut items = vec![0_u32; keypoints.len()];
        let mut next = start[..GRID_COLS * GRID_ROWS].to_vec();
        for (i, &c) in (0_u32..).zip(&cells) {
            items[next[c] as usize] = i;
            next[c] += 1;
        }
        Ok(Self {
            x,
            y,
            level: keypoints.iter().map(|k| level_of(k.level)).collect(),
            x_axis,
            y_axis,
            start,
            items,
        })
    }

    /// The number of keypoints.
    ///
    /// # Returns
    ///
    /// It.
    #[must_use]
    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// Whether the grid holds no keypoints.
    ///
    /// # Returns
    ///
    /// True if empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    /// The keypoints in a window, checked.
    ///
    /// # Arguments
    ///
    /// * `center` - The window's centre `(u, v)`.
    /// * `radius` - Its half-width `r`: a keypoint is in it when `|x - u| < r` and `|y - v| < r`.
    /// * `min_level` - The lowest level it covers.
    /// * `max_level` - The highest.
    ///
    /// # Returns
    ///
    /// The keypoints' indices, ascending.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for a centre or radius that is not finite, or a radius that is not
    /// positive.
    pub fn in_area(
        &self,
        center: [f64; 2],
        radius: f64,
        min_level: i64,
        max_level: i64,
    ) -> Result<Vec<usize>> {
        check_window(center, radius, "window")?;
        let mut found = Vec::new();
        self.collect(center, radius, min_level, max_level, &mut found);
        found.sort_unstable();
        Ok(found.into_iter().map(|i| i as usize).collect())
    }

    /// Appends the keypoints in a window, unchecked and in cell order (not ascending).
    ///
    /// # Arguments
    ///
    /// * `center` - The window's centre, finite.
    /// * `radius` - Its half-width, finite and positive.
    /// * `min_level` - The lowest level it covers.
    /// * `max_level` - The highest.
    /// * `out` - Where the indices go.
    pub fn collect(
        &self,
        center: [f64; 2],
        radius: f64,
        min_level: i64,
        max_level: i64,
        out: &mut Vec<u32>,
    ) {
        if self.x.is_empty() || min_level > max_level {
            return;
        }
        let [u, v] = center;
        let (x0, x1) = (self.x_axis.cell(u - radius), self.x_axis.cell(u + radius));
        let (y0, y1) = (self.y_axis.cell(v - radius), self.y_axis.cell(v + radius));
        for cy in y0..=y1 {
            for cx in x0..=x1 {
                let c = cy * GRID_COLS + cx;
                for &i in &self.items[self.start[c] as usize..self.start[c + 1] as usize] {
                    let k = i as usize;
                    if (self.x[k] - u).abs() < radius
                        && (self.y[k] - v).abs() < radius
                        && (min_level..=max_level).contains(&self.level[k])
                    {
                        out.push(i);
                    }
                }
            }
        }
    }
}

/// One axis of the feature grid: a position `t` is in cell `floor((t - origin) * scale)`,
/// clamped to `[0, cells)`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Axis {
    /// The box's lower edge.
    origin: f64,
    /// Cells per pixel (0: every position in the first cell).
    scale: f64,
    /// The cells along the axis.
    cells: usize,
}

impl Axis {
    /// One axis of the box around some positions.
    ///
    /// # Arguments
    ///
    /// * `values` - The keypoints' coordinates along the axis.
    /// * `cells` - The cells along it.
    ///
    /// # Returns
    ///
    /// The axis: its origin the values' minimum, its scale the cells over their range (0 for
    /// no range, or one too narrow or too wide for a finite scale).
    #[allow(
        clippy::cast_precision_loss,
        reason = "the grid has at most 64 cells along an axis"
    )]
    fn new(values: &[f64], cells: usize) -> Self {
        let Some(low) = values.iter().copied().reduce(f64::min) else {
            return Self {
                origin: 0.0,
                scale: 0.0,
                cells,
            };
        };
        let high = values.iter().copied().fold(low, f64::max);
        let scale = cells as f64 / (high - low);
        Self {
            origin: low,
            scale: if scale.is_finite() { scale } else { 0.0 },
            cells,
        }
    }

    /// A position's cell.
    ///
    /// # Arguments
    ///
    /// * `t` - The position.
    ///
    /// # Returns
    ///
    /// The cell, in `[0, cells)`.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "the cell is clamped to [0, 64) before the cast"
    )]
    fn cell(&self, t: f64) -> usize {
        let at = (t - self.origin) * self.scale;
        // Below the box, or NaN (a scale of 0 times an infinite offset): the first cell.
        if !(at >= 0.0) {
            return 0;
        }
        let last = (self.cells - 1) as f64;
        if at >= last {
            self.cells - 1
        } else {
            at.floor() as usize
        }
    }
}

/// A keypoint's level as a level range compares it.
///
/// # Arguments
///
/// * `level` - The level.
///
/// # Returns
///
/// It as an `i64` (levels are at most [`MAX_LEVELS`] in a checked pyramid).
fn level_of(level: usize) -> i64 {
    i64::try_from(level).unwrap_or(i64::MAX)
}

/// Checks a window's centre and radius.
///
/// # Arguments
///
/// * `center` - The centre.
/// * `radius` - The half-width.
/// * `name` - The window's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for a centre or radius that is not finite, or a radius that is not positive.
fn check_window(center: [f64; 2], radius: f64, name: &str) -> Result<()> {
    if !(center[0].is_finite() && center[1].is_finite()) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected a finite centre"),
        );
    }
    if !(radius.is_finite() && radius > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: the radius must be finite and positive"),
        );
    }
    Ok(())
}

/// Checks a search's filters.
///
/// # Arguments
///
/// * `options` - The filters.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
pub fn check_search_options(options: &SearchOptions) -> Result<()> {
    check_options(&Options {
        max_distance: options.max_distance,
        ratio: options.ratio,
        check_orientation: options.check_orientation,
    })
}

/// The keypoints in a square window, on a range of levels
/// (`slam_it_all.foundations.matching.features_in_area`, ORB-SLAM's `GetFeaturesInArea`).
///
/// # Arguments
///
/// * `keypoints` - The keypoints (positions and levels are read).
/// * `center` - The window's centre `(u, v)`.
/// * `radius` - Its half-width `r`, positive.
/// * `min_level` - The lowest level it covers.
/// * `max_level` - The highest.
///
/// # Returns
///
/// The keypoints' indices, ascending.
///
/// # Errors
///
/// `InvalidInput` for a position that is not finite, too many keypoints, or a bad window.
pub fn features_in_area(
    keypoints: &[Keypoint],
    center: [f64; 2],
    radius: f64,
    min_level: i64,
    max_level: i64,
) -> Result<Vec<usize>> {
    FeatureGrid::build(keypoints)?.in_area(center, radius, min_level, max_level)
}

/// Checks a search's projections.
///
/// # Arguments
///
/// * `projections` - The projections.
///
/// # Errors
///
/// `InvalidInput` for too many, a window that is not finite or positive, an angle out of
/// range, or an infinite right column.
fn check_projections(projections: &[Projection]) -> Result<()> {
    if projections.len() > MAX_DESCRIPTORS {
        return fail(ErrorKind::InvalidInput, "projections: too many projections");
    }
    for p in projections {
        check_window(p.xy, p.radius, "projections")?;
        if p.angle >= ANGLE_BINS {
            return fail(
                ErrorKind::InvalidInput,
                format!("projections.angle: bins must lie in [0, {ANGLE_BINS})"),
            );
        }
        if p.right.is_infinite() {
            return fail(
                ErrorKind::InvalidInput,
                "projections.right: expected finite values",
            );
        }
    }
    Ok(())
}

/// Checks a frame for a search: its features, grid and per-keypoint extras agree.
///
/// # Arguments
///
/// * `features` - The features.
/// * `grid` - Their grid.
/// * `right` - The right columns.
/// * `taken` - The taken flags.
///
/// # Errors
///
/// `InvalidInput` for sizes that disagree, a level outside the pyramid, an angle out of range,
/// or an infinite right column.
fn check_frame(
    features: &Features,
    grid: &FeatureGrid,
    right: &[f64],
    taken: &[bool],
) -> Result<()> {
    let n = features.keypoints.len();
    if features.descriptors.len() != n || grid.len() != n || right.len() != n || taken.len() != n {
        return fail(
            ErrorKind::InvalidInput,
            "features: the descriptors, grid, right columns and taken flags must have one entry \
             per keypoint",
        );
    }
    check_range("features.levels", features.levels, 1, MAX_LEVELS)?;
    for k in &features.keypoints {
        if k.level >= features.levels {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "features.keypoints.level: levels must lie in [0, {})",
                    features.levels
                ),
            );
        }
        if k.angle >= ANGLE_BINS {
            return fail(
                ErrorKind::InvalidInput,
                format!("features.keypoints.angle: bins must lie in [0, {ANGLE_BINS})"),
            );
        }
    }
    if right.iter().any(|v| v.is_infinite()) {
        return fail(ErrorKind::InvalidInput, "right: expected finite values");
    }
    Ok(())
}

/// A candidate's rank: its distance, then its index (the lower ranks first).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Ranked {
    /// The Hamming distance.
    distance: i64,
    /// The keypoint's index.
    index: usize,
}

/// Ranks a projection's candidates: free, within its right column, by distance.
///
/// # Arguments
///
/// * `p` - The projection.
/// * `candidates` - The keypoints in its window.
/// * `descriptors` - The frame's descriptors.
/// * `right` - The frame's right columns (NaN for none).
/// * `taken` - The frame's taken flags.
///
/// # Returns
///
/// The nearest and the second nearest, either `None` if there are too few.
fn nearest_two(
    p: &Projection,
    candidates: &[u32],
    descriptors: &[Descriptor],
    right: &[f64],
    taken: &[bool],
) -> (Option<Ranked>, Option<Ranked>) {
    let mut best: Option<Ranked> = None;
    let mut second: Option<Ranked> = None;
    for &i in candidates {
        let k = i as usize;
        // A NaN column on either side compares false: no constraint.
        if taken[k] || (right[k] - p.right).abs() > p.radius {
            continue;
        }
        let here = Ranked {
            distance: distance(&p.descriptor, &descriptors[k]),
            index: k,
        };
        if best.is_none_or(|b| here < b) {
            second = best;
            best = Some(here);
        } else if second.is_none_or(|s| here < s) {
            second = Some(here);
        }
    }
    (best, second)
}

/// Matches map points projected into a frame to its keypoints
/// (`slam_it_all.foundations.matching.search_by_projection`, ORB-SLAM's `SearchByProjection`).
///
/// # Arguments
///
/// * `projections` - The projections.
/// * `features` - The frame's features (positions, levels, angles and descriptors are read).
/// * `grid` - The frame's grid, built from `features.keypoints`.
/// * `right` - Each keypoint's right-image column; NaN for none.
/// * `taken` - Each keypoint's flag: true if already matched, never a candidate.
/// * `options` - The filters.
///
/// # Returns
///
/// `(projection, keypoint)` matches, by projection.
///
/// # Errors
///
/// `InvalidInput` for a bad filter, a malformed projection, features whose arrays, grid,
/// `right` or `taken` disagree in size, a level outside the pyramid, an angle out of range, or
/// an infinite right column.
#[allow(
    clippy::cast_precision_loss,
    reason = "Hamming distances are at most 256, exact in f64"
)]
pub fn search_by_projection(
    projections: &[Projection],
    features: &Features,
    grid: &FeatureGrid,
    right: &[f64],
    taken: &[bool],
    options: &SearchOptions,
) -> Result<Vec<Match>> {
    check_search_options(options)?;
    check_projections(projections)?;
    check_frame(features, grid, right, taken)?;
    // Each keypoint's claimant: the nearest projection choosing it (ties: the lowest index).
    let mut claims = Claims::new(features.keypoints.len());
    let mut candidates = Vec::new();
    for (i, p) in projections.iter().enumerate() {
        candidates.clear();
        grid.collect(p.xy, p.radius, p.min_level, p.max_level, &mut candidates);
        let (Some(best), second) = nearest_two(p, &candidates, &features.descriptors, right, taken)
        else {
            continue;
        };
        if best.distance > options.max_distance {
            continue;
        }
        if let Some(second) = second
            && features.keypoints[second.index].level == features.keypoints[best.index].level
            && best.distance as f64 > options.ratio * second.distance as f64
        {
            continue;
        }
        claims.offer(Match {
            first: i,
            second: best.index,
            distance: best.distance,
        });
    }
    let mut out = claims.by_first();
    if options.check_orientation {
        keep_consistent_rotations(
            &mut out,
            |i| projections[i].angle,
            |k| features.keypoints[k].angle,
        );
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/matching.json`,
    //! which the Python and C++ tests share, and the core's own input checks.

    use super::*;
    use crate::rng::Pcg32;
    use serde_json::Value;

    /// Reads the shared fixture's functions.
    fn functions() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/matching.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        let fixture: Value = serde_json::from_str(&text).expect("the fixture is valid JSON");
        fixture["functions"].clone()
    }

    /// Reads a non-negative integer.
    fn count(value: &Value) -> usize {
        usize::try_from(value.as_u64().expect("a non-negative integer")).expect("fits a usize")
    }

    /// Reads a list with `read`.
    fn list<T>(value: &Value, read: impl Fn(&Value) -> T) -> Vec<T> {
        value.as_array().expect("a list").iter().map(read).collect()
    }

    /// Reads a descriptor set.
    fn descriptors(value: &Value) -> Vec<Descriptor> {
        list(value, |row| {
            let bytes = list(row, |v| u8::try_from(count(v)).expect("a byte"));
            bytes.try_into().expect("32 bytes")
        })
    }

    /// `distances` matches the fixture.
    #[test]
    fn distances_match_the_fixture() {
        for case in list(&functions()["distances"]["cases"], Clone::clone) {
            let got = distances(
                &descriptors(&case["input"][0]),
                &descriptors(&case["input"][1]),
            )
            .unwrap();
            let expected = list(&case["expected"], |row| {
                list(row, |v| v.as_i64().expect("an integer"))
            });
            assert_eq!(got, expected.concat(), "{}", case["name"]);
        }
    }

    /// `match_descriptors` matches the fixture.
    #[test]
    fn match_matches_the_fixture() {
        for case in list(&functions()["match"]["cases"], Clone::clone) {
            let input = &case["input"];
            let settings = &input[4];
            let (max_distance, ratio, check_orientation) = filters(settings);
            let options = Options {
                max_distance,
                ratio,
                check_orientation,
            };
            let got = match_descriptors(
                &descriptors(&input[0]),
                &descriptors(&input[1]),
                &list(&input[2], count),
                &list(&input[3], count),
                &options,
            )
            .unwrap();
            assert_eq!(got, matches_of(&case["expected"]), "{}", case["name"]);
        }
    }

    /// Reads a fixture's filters: `(max_distance, ratio, check_orientation)`.
    fn filters(settings: &Value) -> (i64, f64, bool) {
        (
            settings["max_distance"].as_i64().expect("an integer"),
            settings["ratio"].as_f64().expect("a float"),
            settings["check_orientation"].as_bool().expect("a bool"),
        )
    }

    /// Reads a float, `null` as NaN (a right column of none).
    fn column(value: &Value) -> f64 {
        value.as_f64().unwrap_or(f64::NAN)
    }

    /// Reads a fixture's keypoints: positions, levels and, if given, angles.
    fn keypoints_of(xy: &Value, level: &Value, angle: Option<&Value>) -> Vec<Keypoint> {
        let xy = list(xy, |p| {
            [p[0].as_f64().expect("x"), p[1].as_f64().expect("y")]
        });
        let level = list(level, count);
        (0..xy.len())
            .map(|i| Keypoint {
                xy: xy[i],
                level: level[i],
                level_xy: [0, 0],
                response: 0,
                angle: angle.map_or(0, |a| count(&a[i])),
            })
            .collect()
    }

    /// Reads a fixture's matches.
    fn matches_of(e: &Value) -> Vec<Match> {
        (0..e["distances"].as_array().expect("a list").len())
            .map(|k| Match {
                first: count(&e["pairs"][k][0]),
                second: count(&e["pairs"][k][1]),
                distance: e["distances"][k].as_i64().expect("an integer"),
            })
            .collect()
    }

    /// `features_in_area` matches the fixture.
    #[test]
    fn features_in_area_matches_the_fixture() {
        for case in list(&functions()["features_in_area"]["cases"], Clone::clone) {
            let input = &case["input"];
            let got = features_in_area(
                &keypoints_of(&input[0], &input[1], None),
                [column(&input[2][0]), column(&input[2][1])],
                column(&input[3]),
                input[4].as_i64().expect("an integer"),
                input[5].as_i64().expect("an integer"),
            )
            .unwrap();
            assert_eq!(got, list(&case["expected"], count), "{}", case["name"]);
        }
    }

    /// `search_by_projection` matches the fixture.
    #[test]
    fn search_by_projection_matches_the_fixture() {
        for case in list(&functions()["search_by_projection"]["cases"], Clone::clone) {
            let (raw, keys, settings) = (&case["input"][0], &case["input"][1], &case["input"][2]);
            let descriptors_p = descriptors(&raw["descriptors"]);
            let projections: Vec<Projection> = (0..descriptors_p.len())
                .map(|i| Projection {
                    xy: [column(&raw["xy"][i][0]), column(&raw["xy"][i][1])],
                    radius: column(&raw["radius"][i]),
                    min_level: raw["min_level"][i].as_i64().expect("an integer"),
                    max_level: raw["max_level"][i].as_i64().expect("an integer"),
                    descriptor: descriptors_p[i],
                    angle: count(&raw["angle"][i]),
                    right: column(&raw["right"][i]),
                })
                .collect();
            let features = Features {
                keypoints: keypoints_of(&keys["xy"], &keys["level"], Some(&keys["angle"])),
                descriptors: descriptors(&keys["descriptors"]),
                scale_factor: 1.2,
                levels: count(&keys["levels"]),
            };
            let right = list(&keys["right"], column);
            let taken = list(&keys["taken"], |v| v.as_bool().expect("a bool"));
            let (max_distance, ratio, check_orientation) = filters(settings);
            let options = SearchOptions {
                max_distance,
                ratio,
                check_orientation,
            };
            let grid = FeatureGrid::build(&features.keypoints).unwrap();
            let got =
                search_by_projection(&projections, &features, &grid, &right, &taken, &options)
                    .unwrap();
            assert_eq!(got, matches_of(&case["expected"]), "{}", case["name"]);
        }
    }

    /// The grid finds what a brute force finds, on keypoints of several spreads.
    #[test]
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "levels drawn in [0, 8)"
    )]
    fn the_grid_finds_what_a_brute_force_finds() {
        let mut rng = Pcg32::new(7, 0);
        let mut unit = move || f64::from(rng.next_u32()) / 4_294_967_296.0;
        for (width, height) in [(640.0, 480.0), (0.0, 480.0), (1e-9, 1e-9), (5000.0, 3.0)] {
            let keypoints: Vec<Keypoint> = (0..800)
                .map(|_| Keypoint {
                    xy: [unit() * width, unit() * height],
                    level: (unit() * 8.0) as usize,
                    level_xy: [0, 0],
                    response: 0,
                    angle: 0,
                })
                .collect();
            let grid = FeatureGrid::build(&keypoints).unwrap();
            for trial in 0..300 {
                let near = keypoints[trial];
                let r = if trial % 3 == 0 {
                    1e-12
                } else {
                    unit() * 0.2 * (width + height + 1.0)
                };
                let center = [near.xy[0] + (unit() - 0.5) * r, near.xy[1]];
                let low = (unit() * 8.0) as i64;
                let brute: Vec<usize> = (0..keypoints.len())
                    .filter(|&i| {
                        let k = &keypoints[i];
                        (k.xy[0] - center[0]).abs() < r
                            && (k.xy[1] - center[1]).abs() < r
                            && (low..=low + 2).contains(&level_of(k.level))
                    })
                    .collect();
                assert_eq!(grid.in_area(center, r, low, low + 2).unwrap(), brute);
            }
        }
    }

    /// Malformed searches and windows are `InvalidInput`.
    #[test]
    fn search_rejects_invalid_input() {
        let keypoints = vec![
            Keypoint {
                xy: [1.0, 1.0],
                level: 0,
                level_xy: [0, 0],
                response: 0,
                angle: 0,
            },
            Keypoint {
                xy: [2.0, 2.0],
                level: 1,
                level_xy: [0, 0],
                response: 0,
                angle: 0,
            },
        ];
        let features = Features {
            keypoints: keypoints.clone(),
            descriptors: vec![[0_u8; DESCRIPTOR_BYTES]; 2],
            scale_factor: 1.2,
            levels: 8,
        };
        let grid = FeatureGrid::build(&keypoints).unwrap();
        let (right, taken) = (vec![f64::NAN; 2], vec![false; 2]);
        let good = Projection {
            xy: [1.0, 1.0],
            radius: 1.0,
            min_level: 0,
            max_level: 0,
            descriptor: [0_u8; DESCRIPTOR_BYTES],
            angle: 0,
            right: f64::NAN,
        };
        let search = |p: Projection, f: &Features| {
            search_by_projection(&[p], f, &grid, &right, &taken, &SearchOptions::default())
        };
        assert!(search(good, &features).is_ok());
        let mut deep = features.clone();
        deep.keypoints[1].level = 8;
        let mut short = features.clone();
        short.descriptors.pop();
        for result in [
            search(
                Projection {
                    radius: 0.0,
                    ..good
                },
                &features,
            ),
            search(
                Projection {
                    xy: [f64::NAN, 1.0],
                    ..good
                },
                &features,
            ),
            search(Projection { angle: 30, ..good }, &features),
            search(
                Projection {
                    right: f64::INFINITY,
                    ..good
                },
                &features,
            ),
            search(good, &deep),
            search(good, &short),
            search_by_projection(
                &[],
                &features,
                &grid,
                &right,
                &taken,
                &SearchOptions {
                    ratio: 0.0,
                    ..SearchOptions::default()
                },
            ),
        ] {
            assert_eq!(result.unwrap_err().kind, ErrorKind::InvalidInput);
        }
        let infinite = Keypoint {
            xy: [f64::INFINITY, 0.0],
            ..keypoints[0]
        };
        assert_eq!(
            FeatureGrid::build(&[infinite]).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        assert_eq!(
            grid.in_area([0.0, 0.0], -1.0, 0, 7).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }

    /// Bad filters and angles are `InvalidInput`.
    #[test]
    fn rejects_invalid_input() {
        let defaults = Options::default();
        for bad in [
            Options {
                max_distance: 257,
                ..defaults
            },
            Options {
                ratio: 0.0,
                ..defaults
            },
        ] {
            assert_eq!(
                check_options(&bad).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
        let set = vec![[0_u8; DESCRIPTOR_BYTES]; 3];
        for (angles1, angles2) in [(&[0, 1][..], &[0, 1, 2][..]), (&[0, 1, 2], &[0, 1, 30])] {
            assert_eq!(
                match_descriptors(&set, &set, angles1, angles2, &defaults)
                    .unwrap_err()
                    .kind,
                ErrorKind::InvalidInput
            );
        }
    }
}
