//! Frames: the Rust core's mirror of `slamlab.edge.frames`.
//!
//! One input image's ORB features, each keypoint's bearing, depth and right column, however
//! the mode produced them (the Python module's docstring states the three modes). Keypoints
//! stay where ORB found them; projections go through the full camera model.

use nalgebra::{Matrix4, Vector2, Vector3};
use slamlab_foundations::camera::{self, Model};
use slamlab_foundations::error::{ErrorKind, Result, check_range, fail, fail_setting};
use slamlab_foundations::image::{self, Image};
use slamlab_foundations::matching::{self, FeatureGrid, Match, Projection, SearchOptions};
use slamlab_foundations::orb::{self, Features};
use slamlab_foundations::stereo::{self, MatchOptions};

/// The camera a sequence was taken with (`slamlab.edge.frames.Camera`).
#[derive(Debug, Clone, PartialEq)]
pub struct Camera {
    /// The model.
    pub model: Model,
    /// Its intrinsics.
    pub intrinsics: Vec<f64>,
    /// The image's width, in pixels.
    pub width: usize,
    /// Its height.
    pub height: usize,
    /// The stereo baseline in metres, or RGB-D's virtual one; 0 for monocular.
    pub baseline: f64,
}

impl Camera {
    /// The horizontal focal length, in pixels.
    ///
    /// # Returns
    ///
    /// It.
    #[must_use]
    pub fn focal(&self) -> f64 {
        self.intrinsics[self.model.focal_indices()[0]]
    }

    /// Projects a camera-frame point, through the full model.
    ///
    /// # Arguments
    ///
    /// * `point` - The point, in front of the camera.
    ///
    /// # Returns
    ///
    /// Its pixel, or NaNs if it cannot project.
    #[must_use]
    pub fn project(&self, point: &Vector3<f64>) -> Vector2<f64> {
        camera::project(self.model, &self.intrinsics, point)
            .unwrap_or_else(|_| Vector2::repeat(f64::NAN))
    }

    /// A pixel's unit bearing.
    ///
    /// # Arguments
    ///
    /// * `pixel` - The pixel.
    ///
    /// # Returns
    ///
    /// The bearing, or NaNs where the model cannot unproject it.
    #[must_use]
    pub fn bearing(&self, pixel: &Vector2<f64>) -> Vector3<f64> {
        camera::unproject(self.model, &self.intrinsics, pixel)
            .unwrap_or_else(|_| Vector3::repeat(f64::NAN))
    }

    /// Whether a pixel lies in the image, `[0, width) x [0, height)`.
    ///
    /// # Arguments
    ///
    /// * `pixel` - The pixel.
    ///
    /// # Returns
    ///
    /// True if inside.
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        reason = "image sides are at most MAX_SIDE, exact in f64"
    )]
    pub fn inside(&self, pixel: &Vector2<f64>) -> bool {
        pixel.x >= 0.0
            && pixel.x < self.width as f64
            && pixel.y >= 0.0
            && pixel.y < self.height as f64
    }
}

/// Checks a camera.
///
/// # Arguments
///
/// * `camera` - The camera.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics, a side out of range, or a negative or non-finite
/// baseline.
pub fn check_camera(camera: &Camera) -> Result<()> {
    camera::check_intrinsics(camera.model, &camera.intrinsics)?;
    check_range("width", camera.width, 1, image::MAX_SIDE)?;
    check_range("height", camera.height, 1, image::MAX_SIDE)?;
    if !(camera.baseline.is_finite() && camera.baseline >= 0.0) {
        return fail_setting(
            "baseline",
            "must be finite and non-negative",
            camera.baseline,
        );
    }
    Ok(())
}

/// One frame's features, as tracking and mapping read them (`slamlab.edge.frames.Frame`).
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Its time, in seconds.
    pub timestamp: f64,
    /// Its ORB features.
    pub features: Features,
    /// Each keypoint's unit bearing (NaN where it cannot be unprojected).
    pub bearings: Vec<Vector3<f64>>,
    /// Each keypoint's depth, in metres (NaN where unknown).
    pub depth: Vec<f64>,
    /// Each keypoint's right-image column (NaN where its depth is unknown).
    pub right: Vec<f64>,
}

impl Frame {
    /// The number of keypoints.
    ///
    /// # Returns
    ///
    /// It.
    #[must_use]
    pub fn size(&self) -> usize {
        self.features.keypoints.len()
    }
}

/// A frame placed against the map (`slamlab.edge.frames.Tracked`).
#[derive(Debug, Clone, PartialEq)]
pub struct Tracked {
    /// Its `T_cam_world`.
    pub pose: Matrix4<f64>,
    /// The id of the map point each keypoint matched, or -1.
    pub points: Vec<i64>,
}

impl Tracked {
    /// How many keypoints matched a map point.
    ///
    /// # Returns
    ///
    /// The count.
    #[must_use]
    pub fn inliers(&self) -> usize {
        self.points.iter().filter(|&&p| p >= 0).count()
    }
}

/// A depth map in metres (0 or NaN where the sensor measured nothing).
#[derive(Debug, Clone, PartialEq)]
pub struct DepthMap {
    /// Rows.
    pub rows: usize,
    /// Columns.
    pub cols: usize,
    /// The depths, row by row.
    pub values: Vec<f64>,
}

/// Reads a depth map at each keypoint's nearest pixel, half up (`slamlab.edge.frames.depth_at`).
///
/// # Arguments
///
/// * `xy` - The keypoints' pixels.
/// * `depth_map` - The depth map.
///
/// # Returns
///
/// Each depth, positive, or NaN.
///
/// # Errors
///
/// `InvalidInput` for a map whose values do not fill its shape, or a pixel that is not finite.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    reason = "the pixel is clamped to the map before the cast"
)]
pub fn depth_at(xy: &[[f64; 2]], depth_map: &DepthMap) -> Result<Vec<f64>> {
    if depth_map.rows == 0
        || depth_map.cols == 0
        || depth_map.values.len() != depth_map.rows * depth_map.cols
    {
        return fail(
            ErrorKind::InvalidInput,
            "depth_map: expected rows x cols values",
        );
    }
    let nearest = |t: f64, size: usize| (t + 0.5).floor().clamp(0.0, (size - 1) as f64) as usize;
    xy.iter()
        .map(|&[x, y]| {
            if !(x.is_finite() && y.is_finite()) {
                return fail(ErrorKind::InvalidInput, "xy: expected finite values");
            }
            let value = depth_map.values
                [nearest(y, depth_map.rows) * depth_map.cols + nearest(x, depth_map.cols)];
            Ok(if value.is_finite() && value > 0.0 {
                value
            } else {
                f64::NAN
            })
        })
        .collect()
}

/// Assembles a frame from its features: their bearings, and the depths and columns.
///
/// # Arguments
///
/// * `timestamp` - The frame's time.
/// * `features` - Its features.
/// * `camera` - The camera.
/// * `depth` - Each keypoint's depth (NaN for none).
/// * `right` - Each keypoint's right column (NaN for none).
///
/// # Returns
///
/// The frame.
fn assemble(
    timestamp: f64,
    features: Features,
    camera: &Camera,
    depth: Vec<f64>,
    right: Vec<f64>,
) -> Frame {
    let bearings = features
        .keypoints
        .iter()
        .map(|k| camera.bearing(&Vector2::new(k.xy[0], k.xy[1])))
        .collect();
    Frame {
        timestamp,
        features,
        bearings,
        depth,
        right,
    }
}

/// Builds a monocular frame: features and bearings, no depth.
///
/// # Arguments
///
/// * `image` - The gray image.
/// * `timestamp` - Its time.
/// * `camera` - The camera.
/// * `options` - The extraction settings.
///
/// # Returns
///
/// The frame.
///
/// # Errors
///
/// The extraction's.
pub fn monocular(
    image: &Image,
    timestamp: f64,
    camera: &Camera,
    options: &orb::Options,
) -> Result<Frame> {
    let features = orb::extract(image, options)?;
    let n = features.keypoints.len();
    Ok(assemble(
        timestamp,
        features,
        camera,
        vec![f64::NAN; n],
        vec![f64::NAN; n],
    ))
}

/// Builds an RGB-D frame: each keypoint's depth and virtual right column.
///
/// # Arguments
///
/// * `image` - The gray image.
/// * `depth_map` - The registered depth.
/// * `timestamp` - Its time.
/// * `camera` - The camera, with its virtual baseline.
/// * `options` - The extraction settings.
///
/// # Returns
///
/// The frame.
///
/// # Errors
///
/// `InvalidInput` for a depth map of another shape, or no baseline; the extraction's.
pub fn rgbd(
    image: &Image,
    depth_map: &DepthMap,
    timestamp: f64,
    camera: &Camera,
    options: &orb::Options,
) -> Result<Frame> {
    if (depth_map.rows, depth_map.cols) != (image.rows, image.cols) {
        return fail(
            ErrorKind::InvalidInput,
            "depth_map: expected the image's shape",
        );
    }
    if camera.baseline <= 0.0 {
        return fail(
            ErrorKind::InvalidInput,
            "camera.baseline: an RGB-D camera needs a virtual baseline",
        );
    }
    let features = orb::extract(image, options)?;
    let xy: Vec<[f64; 2]> = features.keypoints.iter().map(|k| k.xy).collect();
    let depth = depth_at(&xy, depth_map)?;
    let bf = camera.focal() * camera.baseline;
    let right = xy.iter().zip(&depth).map(|(p, &z)| p[0] - bf / z).collect();
    Ok(assemble(timestamp, features, camera, depth, right))
}

/// Builds a stereo frame from a rectified pair: left features, matched along rows.
///
/// # Arguments
///
/// * `images` - The rectified left and right images.
/// * `timestamp` - Their time.
/// * `camera` - The rectified left camera (pinhole) and the pair's baseline.
/// * `options` - The extraction settings.
/// * `stereo_options` - The row matching's settings.
///
/// # Returns
///
/// The frame.
///
/// # Errors
///
/// `InvalidInput` for a camera that is not pinhole or has no baseline; the extraction's and
/// the matching's.
pub fn stereo_pair(
    images: (&Image, &Image),
    timestamp: f64,
    camera: &Camera,
    options: &orb::Options,
    stereo_options: &MatchOptions,
) -> Result<Frame> {
    if camera.model != Model::Pinhole || camera.baseline <= 0.0 {
        return fail(
            ErrorKind::InvalidInput,
            "camera: a stereo pair needs a rectified pinhole camera and baseline",
        );
    }
    let features = orb::extract(images.0, options)?;
    let other = orb::extract(images.1, options)?;
    let found = stereo::match_rows(
        images,
        &features,
        &other,
        camera.focal(),
        camera.baseline,
        stereo_options,
    )?;
    let n = features.keypoints.len();
    let (mut depth, mut columns) = (vec![f64::NAN; n], vec![f64::NAN; n]);
    for m in &found {
        depth[m.left] = m.depth;
        columns[m.left] = m.right_x;
    }
    Ok(assemble(timestamp, features, camera, depth, columns))
}

/// Matches projected map points to a frame's keypoints, its right columns checked
/// (`slamlab.edge.frames.search`): `matching::search_by_projection` over the frame's grid and
/// right columns (NaN for a monocular frame: no check).
///
/// # Arguments
///
/// * `frame` - The frame.
/// * `projections` - The projected points.
/// * `options` - The filters.
/// * `taken` - One flag per keypoint, true if already matched; empty for none.
///
/// # Returns
///
/// `(projection, keypoint)` matches.
///
/// # Errors
///
/// The search's `InvalidInput`.
pub fn search(
    frame: &Frame,
    projections: &[Projection],
    options: &SearchOptions,
    taken: &[bool],
) -> Result<Vec<Match>> {
    let grid = FeatureGrid::build(&frame.features.keypoints)?;
    let none = vec![false; if taken.is_empty() { frame.size() } else { 0 }];
    matching::search_by_projection(
        projections,
        &frame.features,
        &grid,
        &frame.right,
        if taken.is_empty() { &none } else { taken },
        options,
    )
}

#[cfg(test)]
mod tests {
    //! Native tests of depth lookup and the camera check.

    use super::*;

    /// Depth is read half up, clamped, and NaN where the map holds none.
    #[test]
    fn depth_at_rounds_clamps_and_marks_holes() {
        let map = DepthMap {
            rows: 2,
            cols: 2,
            values: vec![0.0, 1.5, 2.5, f64::NAN],
        };
        let read = depth_at(&[[0.4, 0.4], [0.5, -3.0], [-9.0, 0.6], [1.2, 1.2]], &map).unwrap();
        assert!(read[0].is_nan() && read[3].is_nan());
        assert!((read[1] - 1.5).abs() == 0.0 && (read[2] - 2.5).abs() == 0.0);
        assert_eq!(
            depth_at(&[[f64::NAN, 0.0]], &map).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }

    /// A negative baseline is `InvalidInput`.
    #[test]
    fn check_camera_rejects_a_negative_baseline() {
        let camera = Camera {
            model: Model::Pinhole,
            intrinsics: vec![250.0, 250.0, 160.0, 120.0],
            width: 320,
            height: 240,
            baseline: -1.0,
        };
        assert_eq!(
            check_camera(&camera).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }
}
