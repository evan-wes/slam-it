//! ORB keypoints as ORB-SLAM detects them: the Rust core's mirror of
//! `slamlab.foundations.orb`.
//!
//! FAST-9 corners in 30-pixel cells, spread by a quadtree and oriented by their intensity
//! centroid, on every level of a pyramid; all in integer arithmetic, so the three languages
//! return identical keypoints (the Python module's docstring defines each step).

use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::image::{self, Image};

/// Keypoints lie at least this many pixels inside their level (ORB-SLAM's).
pub const EDGE_THRESHOLD: usize = 19;
/// The target side of a detection cell, in pixels (ORB-SLAM's).
pub const CELL_SIZE: usize = 30;
/// The 16 `(dx, dy)` offsets of FAST's radius-3 circle, in order around it.
pub const FAST_CIRCLE: [[i64; 2]; 16] = [
    [0, 3],
    [1, 3],
    [2, 2],
    [3, 1],
    [3, 0],
    [3, -1],
    [2, -2],
    [1, -3],
    [0, -3],
    [-1, -3],
    [-2, -2],
    [-3, -1],
    [-3, 0],
    [-3, 1],
    [-2, 2],
    [-1, 3],
];
/// The contiguous circle pixels a FAST-9 corner needs.
pub const FAST_ARC: usize = 9;
/// The radius of the orientation patch.
pub const HALF_PATCH_SIZE: i64 = 15;
/// The orientation disc's half-width at each row offset `|v|` (OpenCV's `umax`).
pub const UMAX: [i64; 16] = [15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3];
/// Orientation bins, 12 degrees each.
pub const ANGLE_BINS: usize = 30;
/// `round(2^14 (cos, sin))` of `12 k` degrees for each bin `k`.
pub const ROTATIONS: [[i64; 2]; ANGLE_BINS] = [
    [16384, 0],
    [16026, 3406],
    [14968, 6664],
    [13255, 9630],
    [10963, 12176],
    [8192, 14189],
    [5063, 15582],
    [1713, 16294],
    [-1713, 16294],
    [-5063, 15582],
    [-8192, 14189],
    [-10963, 12176],
    [-13255, 9630],
    [-14968, 6664],
    [-16026, 3406],
    [-16384, 0],
    [-16026, -3406],
    [-14968, -6664],
    [-13255, -9630],
    [-10963, -12176],
    [-8192, -14189],
    [-5063, -15582],
    [-1713, -16294],
    [1713, -16294],
    [5063, -15582],
    [8192, -14189],
    [10963, -12176],
    [13255, -9630],
    [14968, -6664],
    [16026, -3406],
];
/// The most pyramid levels [`Options`] accepts.
pub const MAX_LEVELS: usize = 32;

/// ORB extraction settings (`slamlab.foundations.orb.Options`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// The feature budget over all levels, from 1 to [`image::MAX_SIDE`].
    pub n_features: usize,
    /// Pyramid levels, from 1 to [`MAX_LEVELS`].
    pub levels: usize,
    /// Each level's downscale from the previous one, above 1.
    pub scale_factor: f64,
    /// The FAST threshold a cell tries first, from 0 to 254.
    pub fast_threshold: i64,
    /// The threshold for cells with no corner at `fast_threshold`.
    pub min_fast_threshold: i64,
}

impl Default for Options {
    /// ORB-SLAM's settings for 640x480 images.
    fn default() -> Self {
        Self {
            n_features: 1000,
            levels: 8,
            scale_factor: 1.2,
            fast_threshold: 20,
            min_fast_threshold: 7,
        }
    }
}

/// One oriented keypoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Keypoint {
    /// Its pixel in the full image: the level's pixel times `scale_factor^level`.
    pub xy: [f64; 2],
    /// The pyramid level it was detected on.
    pub level: usize,
    /// Its integer pixel in that level.
    pub level_xy: [usize; 2],
    /// Its FAST score.
    pub response: i64,
    /// Its orientation bin, in `[0, ANGLE_BINS)`: `12 * angle` degrees.
    pub angle: usize,
}

/// A half-open box `[x_min, x_max) x [y_min, y_max)`, in pixels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bounds {
    /// The first column inside.
    pub x_min: usize,
    /// One past the last column inside.
    pub x_max: usize,
    /// The first row inside.
    pub y_min: usize,
    /// One past the last row inside.
    pub y_max: usize,
}

/// Checks extraction settings.
///
/// # Arguments
///
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` naming the first setting out of range.
pub fn check_options(options: &Options) -> Result<()> {
    if !(1..=image::MAX_SIDE).contains(&options.n_features) {
        return fail_setting(
            "n_features",
            "must be from 1 to 1048576",
            options.n_features,
        );
    }
    if !(1..=MAX_LEVELS).contains(&options.levels) {
        return fail_setting("levels", "must be from 1 to 32", options.levels);
    }
    if !options.scale_factor.is_finite() || options.scale_factor <= 1.0 {
        return fail_setting(
            "scale_factor",
            "must be finite and above 1",
            options.scale_factor,
        );
    }
    if !(0..=254).contains(&options.fast_threshold) {
        return fail_setting(
            "fast_threshold",
            "must be from 0 to 254",
            options.fast_threshold,
        );
    }
    if !(0..=options.fast_threshold).contains(&options.min_fast_threshold) {
        return fail_setting(
            "min_fast_threshold",
            "must be from 0 to fast_threshold",
            options.min_fast_threshold,
        );
    }
    Ok(())
}

/// Splits the feature budget over the levels, falling geometrically (ORB-SLAM's split).
///
/// # Arguments
///
/// * `n_features` - The total budget.
/// * `levels` - The number of levels.
/// * `scale_factor` - The pyramid's scale factor, above 1.
///
/// # Returns
///
/// One budget per level, summing to `n_features`.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "budgets are at most 2^20, exact in f64; the rounded share is non-negative"
)]
pub fn level_budgets(n_features: usize, levels: usize, scale_factor: f64) -> Vec<usize> {
    let factor = 1.0 / scale_factor;
    let mut power = 1.0;
    for _ in 0..levels {
        power *= factor;
    }
    let mut desired = n_features as f64 * (1.0 - factor) / (1.0 - power);
    let mut budgets = Vec::with_capacity(levels);
    for _ in 1..levels {
        budgets.push((desired + 0.5).floor() as usize);
        desired *= factor;
    }
    let total: usize = budgets.iter().sum();
    budgets.push(n_features.saturating_sub(total));
    budgets
}

/// Reads one pixel at a signed offset from another.
///
/// # Arguments
///
/// * `image` - The image.
/// * `row` - The base row.
/// * `col` - The base column.
/// * `offset` - `(dx, dy)`, keeping the pixel inside.
///
/// # Returns
///
/// The intensity.
fn offset_pixel(image: &Image, row: usize, col: usize, offset: [i64; 2]) -> i64 {
    let r = row.checked_add_signed(to_isize(offset[1])).expect("inside");
    let c = col.checked_add_signed(to_isize(offset[0])).expect("inside");
    i64::from(image.at(r, c))
}

/// Converts a small offset to `isize`.
///
/// # Arguments
///
/// * `value` - The offset.
///
/// # Returns
///
/// It as an `isize`.
fn to_isize(value: i64) -> isize {
    isize::try_from(value).expect("offsets are small")
}

/// A pixel's FAST-9 score, when it reaches `floor`.
///
/// # Arguments
///
/// * `image` - The image.
/// * `row` - The pixel's row, at least 3 from the border.
/// * `col` - The pixel's column, at least 3 from the border.
/// * `floor` - The smallest score of interest, non-negative.
///
/// # Returns
///
/// The score if it is at least `floor`, else -1.
fn fast_score(image: &Image, row: usize, col: usize, floor: i64) -> i64 {
    let centre = i64::from(image.at(row, col));
    // Each circle pixel minus the centre, then the first eight again so every arc is
    // contiguous.
    let mut d = [0_i64; FAST_CIRCLE.len() + FAST_ARC - 1];
    for (k, &offset) in FAST_CIRCLE.iter().enumerate() {
        d[k] = offset_pixel(image, row, col, offset) - centre;
    }
    // Every arc of nine holds one pixel of each opposite pair, (0, 8) and (4, 12): if both of
    // either pair are within `floor` of the centre, no arc clears it.
    let clears = |v: i64| v > floor || v < -floor;
    if (!clears(d[0]) && !clears(d[8])) || (!clears(d[4]) && !clears(d[12])) {
        return -1;
    }
    for k in 0..FAST_ARC - 1 {
        d[FAST_CIRCLE.len() + k] = d[k];
    }
    let best = d
        .windows(FAST_ARC)
        .map(|arc| {
            let low = arc.iter().min().expect("an arc is not empty");
            let high = arc.iter().max().expect("an arc is not empty");
            (*low).max(-*high)
        })
        .max()
        .expect("16 arcs");
    // A corner needs every margin strictly above the threshold: the largest it passes is one
    // below the best arc's smallest margin.
    let score = best - 1;
    if score >= floor { score } else { -1 }
}

/// Every pixel's FAST-9 score: the largest threshold at which it is a corner, or -1.
///
/// # Arguments
///
/// * `image` - The image.
///
/// # Returns
///
/// The scores, row-major.
///
/// # Errors
///
/// `InvalidInput` for a malformed image.
pub fn fast_scores(image: &Image) -> Result<Vec<i64>> {
    image::check_image(image, "image")?;
    let mut out = vec![-1; image.pixels.len()];
    for r in 3..image.rows.saturating_sub(3) {
        for c in 3..image.cols.saturating_sub(3) {
            out[r * image.cols + c] = fast_score(image, r, c, 0);
        }
    }
    Ok(out)
}

/// A quadtree node: a box `[x0, x1) x [y0, y1)` and the corners inside it.
#[derive(Debug, Clone)]
struct Node {
    /// The box's first column.
    x0: usize,
    /// One past the box's last column.
    x1: usize,
    /// The box's first row.
    y0: usize,
    /// One past the box's last row.
    y1: usize,
    /// The corners inside, ascending.
    members: Vec<usize>,
}

impl Node {
    /// Quarters the box at its rounded-up midpoint.
    ///
    /// # Arguments
    ///
    /// * `xy` - The corners' pixels.
    ///
    /// # Returns
    ///
    /// The non-empty quarters: upper left, upper right, lower left, lower right.
    fn split(&self, xy: &[[usize; 2]]) -> Vec<Node> {
        let mx = self.x0 + (self.x1 - self.x0).div_ceil(2);
        let my = self.y0 + (self.y1 - self.y0).div_ceil(2);
        let quarter = |x0, x1, y0, y1| Node {
            x0,
            x1,
            y0,
            y1,
            members: Vec::new(),
        };
        let mut quarters = [
            quarter(self.x0, mx, self.y0, my),
            quarter(mx, self.x1, self.y0, my),
            quarter(self.x0, mx, my, self.y1),
            quarter(mx, self.x1, my, self.y1),
        ];
        for &i in &self.members {
            let k = usize::from(xy[i][0] >= mx) + 2 * usize::from(xy[i][1] >= my);
            quarters[k].members.push(i);
        }
        quarters
            .into_iter()
            .filter(|q| !q.members.is_empty())
            .collect()
    }
}

/// Spreads checked corners with the quadtree (see [`distribute`]).
///
/// # Arguments
///
/// * `xy` - The corners' pixels.
/// * `response` - Their strengths.
/// * `bounds` - Their box.
/// * `target` - How many to keep.
///
/// # Returns
///
/// The kept indices, ascending.
fn distribute_checked(
    xy: &[[usize; 2]],
    response: &[i64],
    bounds: Bounds,
    target: usize,
) -> Vec<usize> {
    if xy.is_empty() || target == 0 {
        return Vec::new();
    }
    let width = bounds.x_max - bounds.x_min;
    let height = bounds.y_max - bounds.y_min;
    let roots = ((2 * width + height) / (2 * height)).max(1);
    let edges: Vec<usize> = (0..=roots)
        .map(|k| bounds.x_min + k * width / roots)
        .collect();
    let mut nodes: Vec<Node> = (0..roots)
        .map(|k| Node {
            x0: edges[k],
            x1: edges[k + 1],
            y0: bounds.y_min,
            y1: bounds.y_max,
            members: Vec::new(),
        })
        .collect();
    for (i, p) in xy.iter().enumerate() {
        let root = edges.partition_point(|&edge| edge <= p[0]) - 1;
        nodes[root].members.push(i);
    }
    nodes.retain(|node| !node.members.is_empty());
    while nodes.len() < target {
        // A box of one pixel cannot be split further: its corners coincide.
        let mut crowded: Vec<usize> = (0..nodes.len())
            .filter(|&k| {
                let node = &nodes[k];
                node.members.len() > 1 && (node.x1 - node.x0) * (node.y1 - node.y0) > 1
            })
            .collect();
        if crowded.is_empty() {
            break;
        }
        if nodes.len() + 3 * crowded.len() > target {
            // A full round would overshoot: the most crowded first, ties in node order.
            crowded.sort_by_key(|&k| std::cmp::Reverse(nodes[k].members.len()));
        }
        let mut quartered: Vec<Option<Vec<Node>>> = vec![None; nodes.len()];
        let mut count = nodes.len();
        for k in crowded {
            if count >= target {
                break;
            }
            let quarters = nodes[k].split(xy);
            count += quarters.len() - 1;
            quartered[k] = Some(quarters);
        }
        nodes = nodes
            .into_iter()
            .zip(quartered)
            .flat_map(|(node, quarters)| quarters.unwrap_or_else(|| vec![node]))
            .collect();
    }
    let mut kept: Vec<usize> = nodes
        .iter()
        .map(|node| {
            // The highest response; ties go to the earliest corner.
            let mut best = node.members[0];
            for &i in &node.members {
                if response[i] > response[best] {
                    best = i;
                }
            }
            best
        })
        .collect();
    kept.sort_unstable();
    kept
}

/// Spreads corners with ORB-SLAM's quadtree (`slamlab.foundations.orb.distribute` defines it).
///
/// # Arguments
///
/// * `xy` - Integer pixel coordinates, each inside `bounds`.
/// * `response` - Each corner's strength.
/// * `bounds` - The box the corners lie in, non-empty.
/// * `target` - How many corners to keep, from 0 to [`image::MAX_SIDE`].
///
/// # Returns
///
/// The kept corners' indices, ascending.
///
/// # Errors
///
/// `InvalidInput` for mismatched sizes, an empty box, a corner outside it, or a target out
/// of range.
pub fn distribute(
    xy: &[[usize; 2]],
    response: &[i64],
    bounds: Bounds,
    target: usize,
) -> Result<Vec<usize>> {
    if response.len() != xy.len() {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "response: expected {} integers, got {}",
                xy.len(),
                response.len()
            ),
        );
    }
    let in_range = |low: usize, high: usize| low < high && high <= image::MAX_SIDE;
    if !in_range(bounds.x_min, bounds.x_max) || !in_range(bounds.y_min, bounds.y_max) {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "bounds: expected a non-empty box inside [0, {}]^2",
                image::MAX_SIDE
            ),
        );
    }
    let inside = |p: &[usize; 2]| {
        (bounds.x_min..bounds.x_max).contains(&p[0]) && (bounds.y_min..bounds.y_max).contains(&p[1])
    };
    if !xy.iter().all(inside) {
        return fail(
            ErrorKind::InvalidInput,
            "xy: every corner must lie inside bounds",
        );
    }
    if target > image::MAX_SIDE {
        return fail_setting("target", "must be from 0 to 1048576", target);
    }
    Ok(distribute_checked(xy, response, bounds, target))
}

/// A keypoint's orientation bin, from its disc's intensity centroid.
///
/// # Arguments
///
/// * `image` - The level.
/// * `x` - The keypoint's column, at least [`HALF_PATCH_SIZE`] inside.
/// * `y` - The keypoint's row, at least [`HALF_PATCH_SIZE`] inside.
///
/// # Returns
///
/// The bin whose direction has the largest dot product with `(m10, m01)` (ties: the lowest).
fn orientation(image: &Image, x: usize, y: usize) -> usize {
    let (mut m10, mut m01) = (0_i64, 0_i64);
    for v in -HALF_PATCH_SIZE..=HALF_PATCH_SIZE {
        let half_width = UMAX[usize::try_from(v.abs()).expect("small")];
        for u in -half_width..=half_width {
            let value = offset_pixel(image, y, x, [u, v]);
            m10 += u * value;
            m01 += v * value;
        }
    }
    let mut best = (0, i64::MIN);
    for (k, &[c, s]) in ROTATIONS.iter().enumerate() {
        let dot = m10 * c + m01 * s;
        if dot > best.1 {
            best = (k, dot);
        }
    }
    best.0
}

/// One level's corners before distribution: their pixels and scores, in raster order.
///
/// FAST maxima over 3x3, at the threshold each cell uses.
///
/// # Arguments
///
/// * `image` - The level.
/// * `region` - The box keypoints may lie in.
/// * `options` - The settings.
///
/// # Returns
///
/// The corners' pixels and their scores.
fn level_corners(image: &Image, region: Bounds, options: &Options) -> (Vec<[usize; 2]>, Vec<i64>) {
    // Scores over the region and a one-pixel ring around it (its maxima's neighbours); a
    // score below the lower threshold reads as -1, which leaves every comparison that decides
    // a maximum at or above that threshold unchanged.
    let (top, left) = (region.y_min - 1, region.x_min - 1);
    let rows = region.y_max - region.y_min + 2;
    let cols = region.x_max - region.x_min + 2;
    let mut scores = vec![-1_i64; rows * cols];
    for r in 0..rows {
        for c in 0..cols {
            scores[r * cols + c] = fast_score(image, top + r, left + c, options.min_fast_threshold);
        }
    }
    let score = |y: usize, x: usize| scores[(y - top) * cols + (x - left)];
    let is_maximum = |y: usize, x: usize| {
        let s = score(y, x);
        (y - 1..=y + 1).all(|ny| (x - 1..=x + 1).all(|nx| (ny, nx) == (y, x) || s > score(ny, nx)))
    };
    let width = region.x_max - region.x_min;
    let height = region.y_max - region.y_min;
    let cell_w = width.div_ceil((width / CELL_SIZE).max(1));
    let cell_h = height.div_ceil((height / CELL_SIZE).max(1));
    let mut threshold = vec![0_i64; width * height];
    for y0 in (region.y_min..region.y_max).step_by(cell_h) {
        for x0 in (region.x_min..region.x_max).step_by(cell_w) {
            let y1 = (y0 + cell_h).min(region.y_max);
            let x1 = (x0 + cell_w).min(region.x_max);
            let strong = (y0..y1).any(|y| {
                (x0..x1).any(|x| score(y, x) >= options.fast_threshold && is_maximum(y, x))
            });
            let cell_threshold = if strong {
                options.fast_threshold
            } else {
                options.min_fast_threshold
            };
            for y in y0..y1 {
                for x in x0..x1 {
                    threshold[(y - region.y_min) * width + (x - region.x_min)] = cell_threshold;
                }
            }
        }
    }
    let mut xy = Vec::new();
    let mut response = Vec::new();
    for y in region.y_min..region.y_max {
        for x in region.x_min..region.x_max {
            let t = threshold[(y - region.y_min) * width + (x - region.x_min)];
            if score(y, x) >= t && is_maximum(y, x) {
                xy.push([x, y]);
                response.push(score(y, x));
            }
        }
    }
    (xy, response)
}

/// Detects oriented keypoints on every pyramid level.
///
/// # Arguments
///
/// * `image` - The image.
/// * `options` - The settings.
///
/// # Returns
///
/// The keypoints by level, then row, then column.
///
/// # Errors
///
/// `InvalidInput` for a malformed image or setting, or a pyramid level that would be empty.
#[allow(
    clippy::cast_precision_loss,
    reason = "pixel coordinates are at most 2^20, exact in f64"
)]
pub fn detect(image: &Image, options: &Options) -> Result<Vec<Keypoint>> {
    check_options(options)?;
    let levels = image::pyramid(image, options.levels, options.scale_factor)?;
    let budgets = level_budgets(options.n_features, options.levels, options.scale_factor);
    let mut out = Vec::new();
    let mut scale = 1.0;
    for (level, (pixels, &budget)) in levels.iter().zip(&budgets).enumerate() {
        if level > 0 {
            scale *= options.scale_factor;
        }
        if pixels.cols <= 2 * EDGE_THRESHOLD || pixels.rows <= 2 * EDGE_THRESHOLD || budget == 0 {
            continue;
        }
        let region = Bounds {
            x_min: EDGE_THRESHOLD,
            x_max: pixels.cols - EDGE_THRESHOLD,
            y_min: EDGE_THRESHOLD,
            y_max: pixels.rows - EDGE_THRESHOLD,
        };
        let (xy, response) = level_corners(pixels, region, options);
        for i in distribute_checked(&xy, &response, region, budget) {
            let [x, y] = xy[i];
            out.push(Keypoint {
                xy: [x as f64 * scale, y as f64 * scale],
                level,
                level_xy: [x, y],
                response: response[i],
                angle: orientation(pixels, x, y),
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/orb.json`, which the
    //! Python and C++ tests share, and the core's own input checks.

    use super::*;
    use serde_json::Value;

    /// Reads the shared fixture's functions.
    fn functions() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/orb.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        let fixture: Value = serde_json::from_str(&text).expect("the fixture is valid JSON");
        fixture["functions"].clone()
    }

    /// Reads a list of values with `read`.
    fn list<T>(value: &Value, read: impl Fn(&Value) -> T) -> Vec<T> {
        value.as_array().expect("a list").iter().map(read).collect()
    }

    /// Reads a non-negative integer.
    fn count(value: &Value) -> usize {
        usize::try_from(value.as_u64().expect("a non-negative integer")).expect("fits a usize")
    }

    /// Reads a signed integer.
    fn int(value: &Value) -> i64 {
        value.as_i64().expect("an integer")
    }

    /// Reads a nested list of intensities as an image.
    fn image(value: &Value) -> Image {
        let rows = list(value, |row| {
            list(row, |v| u8::try_from(count(v)).expect("u8"))
        });
        Image {
            rows: rows.len(),
            cols: rows[0].len(),
            pixels: rows.concat(),
        }
    }

    /// `fast_scores` matches the brute-force fixture.
    #[test]
    fn fast_scores_match_the_fixture() {
        for case in list(&functions()["fast_scores"]["cases"], Clone::clone) {
            let expected = list(&case["expected"], |row| list(row, int)).concat();
            assert_eq!(fast_scores(&image(&case["input"][0])).unwrap(), expected);
        }
    }

    /// `distribute` matches the fixture.
    #[test]
    fn distribute_matches_the_fixture() {
        for case in list(&functions()["distribute"]["cases"], Clone::clone) {
            let input = &case["input"];
            let xy = list(&input[0], |p| [count(&p[0]), count(&p[1])]);
            let response = list(&input[1], int);
            let b = list(&input[2], count);
            let bounds = Bounds {
                x_min: b[0],
                x_max: b[1],
                y_min: b[2],
                y_max: b[3],
            };
            let got = distribute(&xy, &response, bounds, count(&input[3])).unwrap();
            assert_eq!(got, list(&case["expected"], count), "{}", case["name"]);
        }
    }

    /// `detect` matches the fixture, every field exactly.
    #[test]
    fn detect_matches_the_fixture() {
        for case in list(&functions()["detect"]["cases"], Clone::clone) {
            let settings = &case["input"][1];
            let options = Options {
                n_features: count(&settings["n_features"]),
                levels: count(&settings["levels"]),
                scale_factor: settings["scale_factor"].as_f64().expect("a float"),
                fast_threshold: int(&settings["fast_threshold"]),
                min_fast_threshold: int(&settings["min_fast_threshold"]),
            };
            let got = detect(&image(&case["input"][0]), &options).unwrap();
            let e = &case["expected"];
            let expected: Vec<Keypoint> = (0..e["level"].as_array().expect("levels").len())
                .map(|i| Keypoint {
                    xy: [
                        e["xy"][i][0].as_f64().unwrap(),
                        e["xy"][i][1].as_f64().unwrap(),
                    ],
                    level: count(&e["level"][i]),
                    level_xy: [count(&e["level_xy"][i][0]), count(&e["level_xy"][i][1])],
                    response: int(&e["response"][i]),
                    angle: count(&e["angle"][i]),
                })
                .collect();
            assert_eq!(got, expected, "{}", case["name"]);
        }
    }

    /// Bad settings, corners and boxes are `InvalidInput`.
    #[test]
    fn rejects_invalid_input() {
        let defaults = Options::default();
        for bad in [
            Options {
                n_features: 0,
                ..defaults
            },
            Options {
                levels: 33,
                ..defaults
            },
            Options {
                scale_factor: 1.0,
                ..defaults
            },
            Options {
                fast_threshold: 255,
                ..defaults
            },
            Options {
                min_fast_threshold: 21,
                ..defaults
            },
        ] {
            assert_eq!(
                check_options(&bad).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
        let tiny = Image {
            rows: 10,
            cols: 10,
            pixels: vec![0; 100],
        };
        let deep = Options {
            scale_factor: 2.0,
            ..defaults
        };
        assert!(detect(&tiny, &deep).is_err());
        let xy = [[1, 2], [3, 4]];
        let bounds = Bounds {
            x_min: 0,
            x_max: 5,
            y_min: 0,
            y_max: 5,
        };
        assert!(distribute(&xy, &[1], bounds, 1).is_err());
        let empty = Bounds { x_max: 0, ..bounds };
        assert!(distribute(&xy, &[1, 2], empty, 1).is_err());
        let narrow = Bounds { x_max: 3, ..bounds };
        assert!(distribute(&xy, &[1, 2], narrow, 1).is_err());
        assert!(distribute(&xy, &[1, 2], bounds, image::MAX_SIDE + 1).is_err());
        assert_eq!(
            level_budgets(1000, 8, 1.2),
            [217, 181, 151, 126, 105, 87, 73, 60]
        );
    }
}
