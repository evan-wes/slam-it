//! Rectified stereo: the Rust core's mirror of `slamlab.foundations.stereo`.
//!
//! Bouguet's rectification of a calibrated pair, the maps that resample each original image
//! into the rectified view, and ORB-SLAM's matching along rectified rows (the Python module's
//! docstring defines each).

use nalgebra::{Matrix3, Matrix4, Vector2, Vector3, Vector4};

use crate::camera::{self, Model};
use crate::error::{ErrorKind, Result, check_range, fail, fail_setting};
use crate::image::{self, Image, MAX_SIDE};
use crate::orb::{self, Features, Keypoint};
use crate::{matching, se3, so3};

/// The largest half-range [`MatchOptions::search`] accepts, in level pixels: a bound on a
/// match's work, `2 search + 1` patch sums, far above ORB-SLAM's 5.
pub const MAX_SEARCH: usize = 64;
/// Camera centres closer than this count as coinciding.
pub const MIN_BASELINE: f64 = 1e-9;
/// Where a map sends a pixel whose ray points behind the camera: outside every image.
pub const OUTSIDE: f64 = -2.0;

/// A rectified stereo pair.
#[derive(Debug, Clone, PartialEq)]
pub struct Rectification {
    /// `R_rect_left`: turns left-camera coordinates into the rectified frame.
    pub rotation_left: Matrix3<f64>,
    /// `R_rect_right`: turns right-camera coordinates into the rectified right frame.
    pub rotation_right: Matrix3<f64>,
    /// Both rectified cameras' pinhole intrinsics `[f, f, cx, cy]`.
    pub intrinsics: Vector4<f64>,
    /// The distance between the camera centres.
    pub baseline: f64,
}

/// One calibrated camera: its model and intrinsics.
#[derive(Debug, Clone, Copy)]
pub struct Camera<'a> {
    /// The model.
    pub model: Model,
    /// Its intrinsics.
    pub intrinsics: &'a [f64],
}

/// A camera's smallest focal length.
///
/// # Arguments
///
/// * `camera` - The camera, its intrinsics checked.
///
/// # Returns
///
/// The smallest of its focal-length entries ([`Model::focal_indices`]).
fn smallest_focal(camera: Camera<'_>) -> f64 {
    camera
        .model
        .focal_indices()
        .iter()
        .map(|&i| camera.intrinsics[i])
        .fold(f64::INFINITY, f64::min)
}

/// Checks an image shape: each side from 1 to [`MAX_SIDE`].
///
/// # Arguments
///
/// * `rows` - The rows.
/// * `cols` - The columns.
///
/// # Errors
///
/// `InvalidInput` naming the side out of range.
fn check_shape(rows: usize, cols: usize) -> Result<()> {
    check_range("rows", rows, 1, MAX_SIDE)?;
    check_range("cols", cols, 1, MAX_SIDE)
}

/// Rectifies a calibrated pair (Bouguet's algorithm).
///
/// # Arguments
///
/// * `left` - The left camera.
/// * `right` - The right camera.
/// * `t_right_left` - The rigid transform from left-camera to right-camera coordinates.
/// * `rows` - The images' rows.
/// * `cols` - The images' columns.
///
/// # Returns
///
/// The rectification.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics, a non-rigid transform, a bad shape, a baseline closer
/// to vertical, or a right camera to the left of the left one; `Degenerate` if the centres
/// coincide.
#[allow(
    clippy::cast_precision_loss,
    reason = "image sides are at most 2^20, exact in f64"
)]
pub fn rectify(
    left: Camera<'_>,
    right: Camera<'_>,
    t_right_left: &Matrix4<f64>,
    rows: usize,
    cols: usize,
) -> Result<Rectification> {
    camera::check_intrinsics(left.model, left.intrinsics)?;
    camera::check_intrinsics(right.model, right.intrinsics)?;
    se3::check_transform(t_right_left)?;
    check_shape(rows, cols)?;
    let rotation: Matrix3<f64> = t_right_left.fixed_view::<3, 3>(0, 0).into();
    let translation: Vector3<f64> = t_right_left.fixed_view::<3, 1>(0, 3).into();
    let baseline = translation.norm();
    if baseline <= MIN_BASELINE {
        return fail(
            ErrorKind::Degenerate,
            "T_right_left: the camera centres coincide",
        );
    }
    // Each camera turns half the way, then both turn the baseline onto the x axis.
    let half = so3::exp(&(-0.5 * so3::log(&rotation)?));
    let t = half * translation;
    if t.x.abs() <= t.y.abs() {
        return fail(
            ErrorKind::InvalidInput,
            "T_right_left: the baseline is closer to vertical; only side-by-side pairs rectify",
        );
    }
    if t.x > 0.0 {
        return fail(
            ErrorKind::InvalidInput,
            "T_right_left: the right camera lies left of the left one; swap the cameras",
        );
    }
    let target = Vector3::new(-1.0, 0.0, 0.0); // the left camera's centre, seen from the right
    let mut axis = t.cross(&target);
    let norm = axis.norm();
    if norm > 0.0 {
        axis *= (t.x.abs() / t.norm()).min(1.0).acos() / norm;
    }
    let onto_x = so3::exp(&axis);
    let focal = smallest_focal(left).min(smallest_focal(right));
    Ok(Rectification {
        rotation_left: onto_x * half.transpose(),
        rotation_right: onto_x * half,
        intrinsics: Vector4::new(
            focal,
            focal,
            (cols - 1) as f64 / 2.0,
            (rows - 1) as f64 / 2.0,
        ),
        baseline,
    })
}

/// Where each rectified pixel samples one original image.
///
/// # Arguments
///
/// * `camera` - The original camera.
/// * `rotation` - That side's rectifying rotation.
/// * `rectified` - The rectified pinhole intrinsics `[f, f, cx, cy]`.
/// * `rows` - The images' rows.
/// * `cols` - The images' columns.
///
/// # Returns
///
/// `(x, y)` per rectified pixel, row by row, for `image::remap`; [`OUTSIDE`] where the ray
/// points behind the camera.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics, a rotation that is not one, or a bad shape.
#[allow(
    clippy::cast_precision_loss,
    reason = "pixel indices are at most 2^20, exact in f64"
)]
pub fn rectification_map(
    camera: Camera<'_>,
    rotation: &Matrix3<f64>,
    rectified: &Vector4<f64>,
    rows: usize,
    cols: usize,
) -> Result<Vec<Vector2<f64>>> {
    camera::check_intrinsics(camera.model, camera.intrinsics)?;
    camera::check_intrinsics(Model::Pinhole, rectified.as_slice())?;
    so3::check_rotation(rotation)?;
    check_shape(rows, cols)?;
    let (f, cx, cy) = (rectified[0], rectified[2], rectified[3]);
    let r = rotation;
    let mut out = Vec::with_capacity(rows * cols);
    for v in 0..rows {
        for u in 0..cols {
            let x = (u as f64 - cx) / f;
            let y = (v as f64 - cy) / f;
            // R^T (x, y, 1), each product and sum in the Python reference's order.
            let ray = Vector3::new(
                r[(0, 0)] * x + r[(1, 0)] * y + r[(2, 0)],
                r[(0, 1)] * x + r[(1, 1)] * y + r[(2, 1)],
                r[(0, 2)] * x + r[(1, 2)] * y + r[(2, 2)],
            );
            out.push(if ray.z > camera::MIN_DEPTH {
                camera::project(camera.model, camera.intrinsics, &ray)?
            } else {
                Vector2::new(OUTSIDE, OUTSIDE)
            });
        }
    }
    Ok(out)
}

/// Stereo-matching settings (`slamlab.foundations.stereo.Options`, ORB-SLAM's).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchOptions {
    /// A match's Hamming distance must be below this, from 1 to [`matching::MAX_DISTANCE`] + 1.
    pub max_distance: i64,
    /// The row band, in multiples of the right keypoint's level scale, positive.
    pub band: f64,
    /// The SAD patch's half-width, from 1 to [`orb::EDGE_THRESHOLD`].
    pub window: usize,
    /// The SAD search's half-range, from 1 (the V fit needs the best offset to have a
    /// neighbour on each side) to [`MAX_SEARCH`].
    pub search: usize,
    /// Matches whose best sum exceeds this times the median are dropped; at least 1.
    pub outlier_factor: f64,
}

impl Default for MatchOptions {
    /// ORB-SLAM's settings.
    fn default() -> Self {
        Self {
            max_distance: 75,
            band: 2.0,
            window: 5,
            search: 5,
            outlier_factor: 2.1,
        }
    }
}

/// One left keypoint matched along its row.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StereoMatch {
    /// The left keypoint's index.
    pub left: usize,
    /// The right keypoint's index.
    pub right: usize,
    /// The refined right column, in full-image pixels.
    pub right_x: f64,
    /// `x_left - right_x`, positive.
    pub disparity: f64,
    /// `focal baseline / disparity`.
    pub depth: f64,
}

/// Checks stereo-matching settings.
///
/// # Arguments
///
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` naming the first one out of range.
pub fn check_options(options: &MatchOptions) -> Result<()> {
    check_range(
        "max_distance",
        options.max_distance,
        1,
        matching::MAX_DISTANCE + 1,
    )?;
    if !(options.band.is_finite() && options.band > 0.0) {
        return fail_setting("band", "must be positive and finite", options.band);
    }
    check_range("window", options.window, 1, orb::EDGE_THRESHOLD)?;
    check_range("search", options.search, 1, MAX_SEARCH)?;
    if !(options.outlier_factor.is_finite() && options.outlier_factor >= 1.0) {
        return fail_setting(
            "outlier_factor",
            "must be at least 1",
            options.outlier_factor,
        );
    }
    Ok(())
}

/// Checks the two sides' pyramids: a valid one, the same for both.
///
/// # Arguments
///
/// * `left` - The left features.
/// * `right` - The right features.
///
/// # Errors
///
/// `InvalidInput` for an invalid pyramid or two different ones.
#[allow(clippy::float_cmp, reason = "the same pyramid: an exact equality")]
fn check_pyramids(left: &Features, right: &Features) -> Result<()> {
    check_range("left.levels", left.levels, 1, orb::MAX_LEVELS)?;
    image::check_scale_factor(left.scale_factor, "left.scale_factor")?;
    if right.levels != left.levels || right.scale_factor != left.scale_factor {
        return fail(
            ErrorKind::InvalidInput,
            "right: extracted on another pyramid than left (levels and scale_factor differ)",
        );
    }
    Ok(())
}

/// Checks one camera's features against their pyramid: one descriptor per keypoint, every
/// keypoint inside its level, and its `xy` exactly its level pixel's full-image coordinate.
///
/// # Arguments
///
/// * `features` - The features.
/// * `levels` - The pyramid.
/// * `name` - The side's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for the first keypoint that fails.
#[allow(
    clippy::float_cmp,
    reason = "xy must be the exact map, computed identically everywhere"
)]
#[allow(
    clippy::cast_precision_loss,
    reason = "level pixels are at most MAX_SIDE, exact in f64"
)]
fn check_side(features: &Features, levels: &[Image], name: &str) -> Result<()> {
    if features.descriptors.len() != features.keypoints.len() {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected one keypoint and descriptor per row"),
        );
    }
    let full = &levels[0];
    for k in &features.keypoints {
        let Some(level) = levels.get(k.level) else {
            return fail(
                ErrorKind::InvalidInput,
                format!("{name}: levels must lie in [0, {})", levels.len()),
            );
        };
        let [x, y] = k.level_xy;
        if x >= level.cols || y >= level.rows {
            return fail(
                ErrorKind::InvalidInput,
                format!("{name}: level_xy must lie inside its level"),
            );
        }
        if k.xy[0] != image::full_image_coordinate(x as f64, level.cols, full.cols)
            || k.xy[1] != image::full_image_coordinate(y as f64, level.rows, full.rows)
        {
            return fail(
                ErrorKind::InvalidInput,
                format!("{name}: xy must be level_xy mapped to the full image (orb.Keypoints.xy)"),
            );
        }
    }
    Ok(())
}

/// Checks every left keypoint lies at least `margin` inside its level.
///
/// # Arguments
///
/// * `features` - The left features, checked against the pyramid.
/// * `levels` - The pyramid.
/// * `margin` - The SAD window's half-width.
///
/// # Errors
///
/// `InvalidInput` for a keypoint too close to its level's border.
fn check_inside(features: &Features, levels: &[Image], margin: usize) -> Result<()> {
    for k in &features.keypoints {
        let level = &levels[k.level];
        let [x, y] = k.level_xy;
        if x < margin || x + margin >= level.cols || y < margin || y + margin >= level.rows {
            return fail(
                ErrorKind::InvalidInput,
                format!("left: keypoints must lie at least {margin} pixels inside their level"),
            );
        }
    }
    Ok(())
}

/// The column on one level nearest a pixel's centre on another.
///
/// # Arguments
///
/// * `x` - The pixel's column, inside its level.
/// * `from_cols` - Its level's columns.
/// * `to_cols` - The other level's columns.
///
/// # Returns
///
/// `floor((2 x + 1) to_cols / (2 from_cols))`, inside the other level.
fn nearest_column(x: usize, from_cols: usize, to_cols: usize) -> usize {
    (2 * x + 1) * to_cols / (2 * from_cols)
}

/// The zero-mean sum of absolute differences between two patches, times their size.
///
/// # Arguments
///
/// * `left` - The left level.
/// * `right` - The right level.
/// * `pixel` - The left patch's centre `(x, y)`.
/// * `column` - The right patch's centre column, on row `y`.
/// * `w` - The patches' half-width.
///
/// # Returns
///
/// `sum |n (L - R) - (sum L - sum R)|`, `n = (2 w + 1)^2`.
#[allow(
    clippy::cast_possible_wrap,
    reason = "a patch holds at most (2 EDGE_THRESHOLD + 1)^2 pixels"
)]
fn zero_mean_sad(left: &Image, right: &Image, pixel: [usize; 2], column: usize, w: usize) -> i64 {
    let [x, y] = pixel;
    let n = ((2 * w + 1) * (2 * w + 1)) as i64;
    let difference = |dy: usize, dx: usize| {
        i64::from(left.at(y + dy - w, x + dx - w))
            - i64::from(right.at(y + dy - w, column + dx - w))
    };
    let window = || (0..=2 * w).flat_map(|dy| (0..=2 * w).map(move |dx| (dy, dx)));
    let offset: i64 = window().map(|(dy, dx)| difference(dy, dx)).sum(); // sum L - sum R
    window()
        .map(|(dy, dx)| (n * difference(dy, dx) - offset).abs())
        .sum()
}

/// Slides the left patch along the right row by zero-mean SAD and V-fits the best sum.
///
/// # Arguments
///
/// * `left` - The left level.
/// * `right` - The right level.
/// * `pixel` - The left keypoint's level pixel `(x, y)`.
/// * `column` - The right keypoint's column on that level, inside it.
/// * `options` - The settings.
///
/// # Returns
///
/// The sub-pixel offset from `column` and the best sum, or `None` if the search leaves the
/// image or the best offset is an end.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    reason = "sums and offsets are far below 2^52"
)]
fn refine(
    left: &Image,
    right: &Image,
    pixel: [usize; 2],
    column: usize,
    options: &MatchOptions,
) -> Option<(f64, i64)> {
    let (w, reach) = (options.window, options.search);
    // Every value is below 2^21 (a column inside its level, reach and w small): no overflow.
    if column < reach + w || column + reach + w >= right.cols {
        return None;
    }
    let sums: Vec<i64> = (0..=2 * reach)
        .map(|step| zero_mean_sad(left, right, pixel, column - reach + step, w))
        .collect();
    // The first smallest sum, so d1 > d2 <= d3 below: the V fit's denominator is positive and
    // its offset within half a pixel.
    let best = (0..sums.len()).fold(0, |b, i| if sums[i] < sums[b] { i } else { b });
    if best == 0 || best == 2 * reach {
        return None;
    }
    let (d1, d2, d3) = (sums[best - 1], sums[best], sums[best + 1]);
    let delta = (d1 - d3) as f64 / (2 * (d1 - d2).max(d3 - d2)) as f64;
    Some(((best as i64 - reach as i64) as f64 + delta, d2))
}

/// A left keypoint's nearest right candidate: within a level, within the row band, at most
/// `focal` columns to its left, under the distance cap.
///
/// # Arguments
///
/// * `k` - The left keypoint.
/// * `descriptor` - Its descriptor.
/// * `right` - The right features.
/// * `scales` - Each level's scale.
/// * `focal` - The rectified focal length.
/// * `options` - The settings.
///
/// # Returns
///
/// The nearest candidate's index (ties: the lowest), or `None`.
fn nearest(
    k: &Keypoint,
    descriptor: &orb::Descriptor,
    right: &Features,
    scales: &[f64],
    focal: f64,
    options: &MatchOptions,
) -> Option<usize> {
    let mut best = None;
    let mut best_distance = options.max_distance;
    for (j, (r, other)) in right.keypoints.iter().zip(&right.descriptors).enumerate() {
        if r.level.abs_diff(k.level) > 1
            || (r.xy[1] - k.xy[1]).abs() > options.band * scales[r.level]
            || r.xy[0] < k.xy[0] - focal
            || r.xy[0] > k.xy[0]
        {
            continue;
        }
        let d = matching::distance(descriptor, other);
        if d < best_distance {
            best_distance = d;
            best = Some(j);
        }
    }
    best
}

/// Matches ORB features along the rows of a rectified pair (ORB-SLAM's stereo matching, with
/// `slamlab.foundations.stereo`'s departures; Python's and C++'s `match`, a Rust keyword).
///
/// # Arguments
///
/// * `images` - The rectified left and right images, of one shape.
/// * `left` - The left features.
/// * `right` - The right features, extracted on the same pyramid.
/// * `focal` - The rectified focal length, in pixels.
/// * `baseline` - The distance between the camera centres.
/// * `options` - The matching settings.
///
/// # Returns
///
/// The matches, by left index.
///
/// # Errors
///
/// `InvalidInput` for malformed images, features or settings, features from different
/// pyramids or one too deep for the image, a keypoint outside its level or whose `xy` is not
/// its level pixel's full-image coordinate, or a left keypoint closer than `options.window` to
/// its level's border.
#[allow(
    clippy::cast_precision_loss,
    reason = "columns are at most 2^20, exact in f64"
)]
pub fn match_rows(
    images: (&Image, &Image),
    left: &Features,
    right: &Features,
    focal: f64,
    baseline: f64,
    options: &MatchOptions,
) -> Result<Vec<StereoMatch>> {
    let (left_image, right_image) = images;
    if (right_image.rows, right_image.cols) != (left_image.rows, left_image.cols) {
        return fail(
            ErrorKind::InvalidInput,
            "right_image: expected the left image's shape",
        );
    }
    if !(focal.is_finite() && focal > 0.0) {
        return fail_setting("focal", "must be positive and finite", focal);
    }
    if !(baseline.is_finite() && baseline > 0.0) {
        return fail_setting("baseline", "must be positive and finite", baseline);
    }
    check_options(options)?;
    check_pyramids(left, right)?;
    let left_levels = image::pyramid(left_image, left.levels, left.scale_factor)?;
    let right_levels = image::pyramid(right_image, left.levels, left.scale_factor)?;
    check_side(left, &left_levels, "left")?;
    check_side(right, &left_levels, "right")?;
    check_inside(left, &left_levels, options.window)?;
    let mut scales = vec![1.0];
    while scales.len() < left_levels.len() {
        scales.push(scales[scales.len() - 1] * left.scale_factor);
    }
    let cols = left_image.cols;
    let mut found: Vec<(StereoMatch, i64)> = Vec::new();
    for (i, (k, descriptor)) in left.keypoints.iter().zip(&left.descriptors).enumerate() {
        let Some(j) = nearest(k, descriptor, right, &scales, focal, options) else {
            continue;
        };
        let r = &right.keypoints[j];
        let level_cols = left_levels[k.level].cols;
        let column = nearest_column(r.level_xy[0], left_levels[r.level].cols, level_cols);
        let Some((offset, cost)) = refine(
            &left_levels[k.level],
            &right_levels[k.level],
            k.level_xy,
            column,
            options,
        ) else {
            continue;
        };
        let right_x = image::full_image_coordinate(column as f64 + offset, level_cols, cols);
        let disparity = k.xy[0] - right_x;
        if disparity <= 0.0 || disparity >= focal {
            continue;
        }
        found.push((
            StereoMatch {
                left: i,
                right: j,
                right_x,
                disparity,
                depth: focal * baseline / disparity,
            },
            cost,
        ));
    }
    Ok(drop_outliers(found, options.outlier_factor))
}

/// Drops matches whose best sum exceeds `factor` times the median.
///
/// # Arguments
///
/// * `found` - The matches and their best sums, by left index.
/// * `factor` - The factor.
///
/// # Returns
///
/// The kept matches, by left index.
#[allow(clippy::cast_precision_loss, reason = "sums are far below 2^52")]
fn drop_outliers(found: Vec<(StereoMatch, i64)>, factor: f64) -> Vec<StereoMatch> {
    if found.is_empty() {
        return Vec::new();
    }
    let mut costs: Vec<i64> = found.iter().map(|&(_, cost)| cost).collect();
    costs.sort_unstable();
    let limit = factor * costs[costs.len() / 2] as f64;
    found
        .into_iter()
        .filter(|&(_, cost)| cost as f64 <= limit)
        .map(|(m, _)| m)
        .collect()
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/stereo.json`, which
    //! the Python and C++ tests share, and the core's own input checks.

    use super::*;
    use nalgebra::DMatrix;
    use serde_json::Value;

    /// Reads the shared fixture's functions.
    fn functions() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/stereo.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        let fixture: Value = serde_json::from_str(&text).expect("the fixture is valid JSON");
        fixture["functions"].clone()
    }

    /// Reads a list of floats.
    fn floats(value: &Value) -> Vec<f64> {
        value
            .as_array()
            .expect("a list")
            .iter()
            .map(|v| v.as_f64().expect("a number"))
            .collect()
    }

    /// Reads a row-major matrix from a list of rows.
    fn matrix(value: &Value) -> DMatrix<f64> {
        let rows: Vec<Vec<f64>> = value.as_array().expect("rows").iter().map(floats).collect();
        DMatrix::from_fn(rows.len(), rows[0].len(), |r, c| rows[r][c])
    }

    /// Reads a model from its name.
    fn model(value: &Value) -> Model {
        match value.as_str().expect("a name") {
            "radtan" => Model::RadTan,
            "radial" => Model::Radial,
            _ => Model::Pinhole,
        }
    }

    /// Reads a non-negative integer.
    fn count(value: &Value) -> usize {
        usize::try_from(value.as_u64().expect("a count")).expect("fits a usize")
    }

    /// The largest absolute difference between two equal-sized slices.
    fn worst(a: &[f64], b: &[f64]) -> f64 {
        assert_eq!(a.len(), b.len());
        a.iter()
            .zip(b)
            .map(|(x, y)| (x - y).abs())
            .fold(0.0, f64::max)
    }

    /// `rectify` matches the fixture within its tolerance.
    #[test]
    fn rectify_matches_the_fixture() {
        let spec = &functions()["rectify"];
        let tolerance = spec["tolerance"]["atol"].as_f64().expect("a tolerance");
        for case in spec["cases"].as_array().expect("cases") {
            let input = &case["input"];
            let (left_k, right_k) = (floats(&input[1]), floats(&input[3]));
            let t = matrix(&input[4]);
            let got = rectify(
                Camera {
                    model: model(&input[0]),
                    intrinsics: &left_k,
                },
                Camera {
                    model: model(&input[2]),
                    intrinsics: &right_k,
                },
                &Matrix4::from_fn(|r, c| t[(r, c)]),
                count(&input[5][0]),
                count(&input[5][1]),
            )
            .unwrap();
            let e = &case["expected"];
            assert!(
                worst(
                    got.rotation_left.transpose().as_slice(),
                    matrix(&e["rotation_left"]).transpose().as_slice()
                ) <= tolerance
            );
            assert!(
                worst(
                    got.rotation_right.transpose().as_slice(),
                    matrix(&e["rotation_right"]).transpose().as_slice()
                ) <= tolerance
            );
            assert!(worst(got.intrinsics.as_slice(), &floats(&e["intrinsics"])) <= tolerance);
            assert!((got.baseline - e["baseline"].as_f64().unwrap()).abs() <= tolerance);
        }
    }

    /// `rectification_map` matches the fixture within its tolerance.
    #[test]
    fn rectification_map_matches_the_fixture() {
        let maps = &functions()["rectification_map"];
        let tolerance = maps["tolerance"]["atol"].as_f64().expect("a tolerance");
        for case in maps["cases"].as_array().expect("cases") {
            let fields = &case["input"][2];
            let (rows, cols) = (count(&fields["shape"][0]), count(&fields["shape"][1]));
            let pinhole = floats(&fields["intrinsics"]);
            for (side, key) in [(0, "rotation_left"), (1, "rotation_right")] {
                let camera = &case["input"][side];
                let k = floats(&camera[1]);
                let r = matrix(&fields[key]);
                let got = rectification_map(
                    Camera {
                        model: model(&camera[0]),
                        intrinsics: &k,
                    },
                    &Matrix3::from_fn(|i, j| r[(i, j)]),
                    &Vector4::from_row_slice(&pinhole),
                    rows,
                    cols,
                )
                .unwrap();
                let expected: Vec<f64> = case["expected"][side]
                    .as_array()
                    .expect("rows")
                    .iter()
                    .flat_map(|row| row.as_array().expect("a row").iter().flat_map(floats))
                    .collect();
                let flat: Vec<f64> = got.iter().flat_map(|p| [p.x, p.y]).collect();
                assert!(worst(&flat, &expected) <= tolerance);
            }
        }
    }

    /// Reads a list of non-negative integers.
    fn counts(value: &Value) -> Vec<usize> {
        value
            .as_array()
            .expect("a list")
            .iter()
            .map(count)
            .collect()
    }

    /// Reads a nested list of intensities as an image.
    fn picture(value: &Value) -> Image {
        let rows: Vec<Vec<u8>> = value
            .as_array()
            .expect("rows")
            .iter()
            .map(|row| {
                counts(row)
                    .into_iter()
                    .map(|v| u8::try_from(v).expect("a byte"))
                    .collect()
            })
            .collect();
        Image {
            rows: rows.len(),
            cols: rows[0].len(),
            pixels: rows.concat(),
        }
    }

    /// Reads a match case's features (responses and angles, unused, are zero).
    fn features(fields: &Value) -> Features {
        let levels = counts(&fields["level"]);
        let keypoints = (0..levels.len())
            .map(|i| Keypoint {
                xy: [
                    fields["xy"][i][0].as_f64().unwrap(),
                    fields["xy"][i][1].as_f64().unwrap(),
                ],
                level: levels[i],
                level_xy: [
                    count(&fields["level_xy"][i][0]),
                    count(&fields["level_xy"][i][1]),
                ],
                response: 0,
                angle: 0,
            })
            .collect();
        let descriptors = fields["descriptors"]
            .as_array()
            .expect("rows")
            .iter()
            .map(|row| {
                let bytes: Vec<u8> = counts(row)
                    .into_iter()
                    .map(|v| u8::try_from(v).expect("a byte"))
                    .collect();
                bytes.try_into().expect("32 bytes")
            })
            .collect();
        Features {
            keypoints,
            descriptors,
            scale_factor: fields["scale_factor"].as_f64().unwrap(),
            levels: count(&fields["levels"]),
        }
    }

    /// `match_rows` matches the fixture exactly.
    #[test]
    fn match_matches_the_fixture() {
        for case in functions()["match"]["cases"].as_array().expect("cases") {
            let input = &case["input"];
            let s = &input[6];
            let settings = MatchOptions {
                max_distance: s["max_distance"].as_i64().unwrap(),
                band: s["band"].as_f64().unwrap(),
                window: count(&s["window"]),
                search: count(&s["search"]),
                outlier_factor: s["outlier_factor"].as_f64().unwrap(),
            };
            let got = match_rows(
                (&picture(&input[0]), &picture(&input[1])),
                &features(&input[2]),
                &features(&input[3]),
                input[4].as_f64().unwrap(),
                input[5].as_f64().unwrap(),
                &settings,
            )
            .unwrap();
            let e = &case["expected"];
            let expected: Vec<StereoMatch> = (0..counts(&e["left"]).len())
                .map(|k| StereoMatch {
                    left: count(&e["left"][k]),
                    right: count(&e["right"][k]),
                    right_x: e["right_x"][k].as_f64().unwrap(),
                    disparity: e["disparity"][k].as_f64().unwrap(),
                    depth: e["depth"][k].as_f64().unwrap(),
                })
                .collect();
            assert_eq!(got, expected, "{}", case["name"]);
        }
    }

    /// Vertical, coincident and malformed pairs are refused.
    #[test]
    fn rejects_bad_pairs() {
        let k = [500.0, 500.0, 320.0, 240.0];
        let camera = Camera {
            model: Model::Pinhole,
            intrinsics: &k,
        };
        let mut vertical = Matrix4::identity();
        vertical[(1, 3)] = 0.1;
        assert_eq!(
            rectify(camera, camera, &vertical, 480, 640)
                .unwrap_err()
                .kind,
            ErrorKind::InvalidInput
        );
        assert_eq!(
            rectify(camera, camera, &Matrix4::identity(), 480, 640)
                .unwrap_err()
                .kind,
            ErrorKind::Degenerate
        );
        let mut side = Matrix4::identity();
        side[(0, 3)] = -0.1;
        assert_eq!(
            rectify(camera, camera, &side, 0, 640).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        assert!(rectify(camera, camera, &side, 480, 640).is_ok());
        let mut swapped = Matrix4::identity();
        swapped[(0, 3)] = 0.1;
        assert_eq!(
            rectify(camera, camera, &swapped, 480, 640)
                .unwrap_err()
                .kind,
            ErrorKind::InvalidInput
        );
    }

    /// One-level features at one level-0 pixel, with a zero descriptor.
    #[allow(clippy::cast_precision_loss, reason = "small pixels")]
    fn one_keypoint(x: usize, y: usize) -> Features {
        Features {
            keypoints: vec![Keypoint {
                xy: [x as f64, y as f64],
                level: 0,
                level_xy: [x, y],
                response: 0,
                angle: 0,
            }],
            descriptors: vec![[0; orb::DESCRIPTOR_BYTES]],
            scale_factor: 1.2,
            levels: 1,
        }
    }

    /// Features off their pyramid are refused, not read out of bounds.
    #[test]
    fn rejects_features_off_their_pyramid() {
        let frame = Image {
            rows: 60,
            cols: 80,
            pixels: vec![0; 60 * 80],
        };
        let settings = MatchOptions::default();
        let refused = |left: &Features, right: &Features| {
            match_rows((&frame, &frame), left, right, 100.0, 0.1, &settings)
                .is_err_and(|e| e.kind == ErrorKind::InvalidInput)
        };
        assert!(!refused(&one_keypoint(30, 30), &one_keypoint(25, 30)));
        for x in [1e300, -1e19, 25.5, f64::NAN] {
            let mut right = one_keypoint(25, 30);
            right.keypoints[0].xy[0] = x;
            assert!(refused(&one_keypoint(30, 30), &right), "xy {x}");
        }
        assert!(refused(&one_keypoint(30, 30), &one_keypoint(80, 30)));
        let coarser = Features {
            scale_factor: 1.3,
            ..one_keypoint(25, 30)
        };
        assert!(refused(&one_keypoint(30, 30), &coarser));
        let deep = Features {
            levels: 5,
            scale_factor: 4.0,
            ..one_keypoint(30, 30)
        };
        assert!(refused(&deep, &deep));
        let shallow = Features {
            levels: 0,
            ..one_keypoint(30, 30)
        };
        assert!(refused(&shallow, &shallow));
        let mut off = one_keypoint(30, 30);
        off.keypoints[0].level = 1_000_000;
        assert!(refused(&off, &one_keypoint(25, 30)));
    }
}
