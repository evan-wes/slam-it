//! Bindings for `slamlab.foundations.stereo` (`slamlab_rs.foundations.stereo`).
//!
//! Camera models cross as their integer codes; a rectification comes back as a dict of its
//! rotations, rectified intrinsics and baseline. Features cross as dicts of `xy`, `level`,
//! `level_xy` and `descriptors`; settings as dicts of their dataclasses' fields, read strictly
//! by serde.

use nalgebra::{Matrix3, Matrix4, Vector4};
use numpy::ndarray::Array1;
use numpy::{IntoPyArray, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slamlab_foundations::camera::Model;
use slamlab_foundations::orb::{self, DESCRIPTOR_BYTES, Features, Keypoint};
use slamlab_foundations::stereo::{self as core, Camera};

use super::image_crossing::image_in;

use crate::common::{
    ArrayIn, ArrayOut, Item, add_functions, contiguous, decode, item_array, new_array, one, unwrap,
};

/// Reads a camera from its model code and intrinsics.
///
/// # Arguments
///
/// * `model` - The model's code.
/// * `intrinsics` - The intrinsics.
///
/// # Returns
///
/// The camera.
///
/// # Errors
///
/// `ValueError` for an unknown code or a non-contiguous array.
fn read_camera<'a>(model: i64, intrinsics: &'a PyReadonlyArray1<'_, f64>) -> PyResult<Camera<'a>> {
    Ok(Camera {
        model: decode(&Model::ALL, model, "model")?,
        intrinsics: contiguous(intrinsics)?,
    })
}

/// Rectifies a pair; returns the rotations, the rectified intrinsics and the baseline.
///
/// # Arguments
///
/// * `model_left` - The left camera's model code.
/// * `intrinsics_left` - Its intrinsics.
/// * `model_right` - The right camera's model code.
/// * `intrinsics_right` - Its intrinsics.
/// * `T_right_left` - The (4, 4) transform from left to right camera coordinates.
/// * `rows` - The images' rows.
/// * `cols` - The images' columns.
///
/// # Returns
///
/// A dict: `rotation_left`, `rotation_right` (3, 3), `intrinsics` (4,), `baseline`.
///
/// # Errors
///
/// `ValueError` for a malformed array or code; `InvalidInputError` or `DegenerateError` from
/// the core.
#[pyfunction]
#[allow(
    non_snake_case,
    reason = "the frame convention's name, as Python spells it"
)]
#[allow(
    clippy::too_many_arguments,
    reason = "two cameras, their pose and the image shape"
)]
fn rectify<'py>(
    py: Python<'py>,
    model_left: i64,
    intrinsics_left: PyReadonlyArray1<'py, f64>,
    model_right: i64,
    intrinsics_right: PyReadonlyArray1<'py, f64>,
    T_right_left: ArrayIn<'py>,
    rows: usize,
    cols: usize,
) -> PyResult<Bound<'py, PyDict>> {
    let transform: Matrix4<f64> = one(&T_right_left)?;
    let r = unwrap(
        py,
        core::rectify(
            read_camera(model_left, &intrinsics_left)?,
            read_camera(model_right, &intrinsics_right)?,
            &transform,
            rows,
            cols,
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("rotation_left", item_array(py, &r.rotation_left)?)?;
    out.set_item("rotation_right", item_array(py, &r.rotation_right)?)?;
    out.set_item("intrinsics", item_array(py, &r.intrinsics)?)?;
    out.set_item("baseline", r.baseline)?;
    Ok(out)
}

/// Returns the (rows, cols, 2) map from rectified pixels to the original image.
///
/// # Arguments
///
/// * `model` - The camera's model code.
/// * `intrinsics` - Its intrinsics.
/// * `rotation` - Its rectifying rotation.
/// * `rectified` - The rectified pinhole intrinsics.
/// * `rows` - The images' rows.
/// * `cols` - The images' columns.
///
/// # Returns
///
/// A new (rows, cols, 2) float64 array.
///
/// # Errors
///
/// `ValueError` for a malformed array or code; `InvalidInputError` from the core.
#[pyfunction]
fn rectification_map<'py>(
    py: Python<'py>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    rotation: ArrayIn<'py>,
    rectified: PyReadonlyArray1<'py, f64>,
    rows: usize,
    cols: usize,
) -> PyResult<ArrayOut<'py>> {
    let turn: Matrix3<f64> = one(&rotation)?;
    let pinhole = contiguous(&rectified)?;
    if pinhole.len() != 4 {
        return Err(PyValueError::new_err("rectified: expected 4 values"));
    }
    let map = unwrap(
        py,
        core::rectification_map(
            read_camera(model, &intrinsics)?,
            &turn,
            &Vector4::from_row_slice(pinhole),
            rows,
            cols,
        ),
    )?;
    let mut values = Vec::with_capacity(2 * map.len());
    for point in &map {
        point.write(&mut values);
    }
    new_array(py, &[rows, cols, 2], values)
}

/// `stereo.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MatchOptionsIn {
    /// The distance cap.
    max_distance: i64,
    /// The row band.
    band: f64,
    /// The SAD half-width.
    window: usize,
    /// The SAD search half-range.
    search: usize,
    /// The outlier factor.
    outlier_factor: f64,
}

/// `orb.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OrbOptionsIn {
    /// The feature budget.
    n_features: usize,
    /// Pyramid levels.
    levels: usize,
    /// Each level's downscale.
    scale_factor: f64,
    /// The first FAST threshold.
    fast_threshold: i64,
    /// The fallback FAST threshold.
    min_fast_threshold: i64,
}

/// Reads one item of a features dict.
///
/// # Arguments
///
/// * `features` - The dict.
/// * `key` - The item's key.
///
/// # Returns
///
/// The item.
///
/// # Errors
///
/// `ValueError` if the key is missing.
fn item<'py>(features: &Bound<'py, PyDict>, key: &str) -> PyResult<Bound<'py, PyAny>> {
    features
        .get_item(key)?
        .ok_or_else(|| PyValueError::new_err(format!("features: missing `{key}`")))
}

/// Converts a non-negative int64 to an index.
///
/// # Arguments
///
/// * `value` - The value.
///
/// # Returns
///
/// It as a `usize`.
///
/// # Errors
///
/// `ValueError` if it is negative.
fn index(value: i64) -> PyResult<usize> {
    usize::try_from(value).map_err(|e| PyValueError::new_err(format!("features: {e}")))
}

/// Reads one camera's features from a dict of arrays.
///
/// # Arguments
///
/// * `features` - The dict: `xy` (N, 2) float64, `level` (N,) and `level_xy` (N, 2) int64,
///   `descriptors` (N, 32) uint8.
///
/// # Returns
///
/// The features (responses and angles, unused by matching, are zero).
///
/// # Errors
///
/// `ValueError` for a missing key or a malformed array.
fn features_in(features: &Bound<'_, PyDict>) -> PyResult<Features> {
    let xy: PyReadonlyArray2<'_, f64> = item(features, "xy")?.extract()?;
    let level: PyReadonlyArray1<'_, i64> = item(features, "level")?.extract()?;
    let level_xy: PyReadonlyArray2<'_, i64> = item(features, "level_xy")?.extract()?;
    let descriptors: PyReadonlyArray2<'_, u8> = item(features, "descriptors")?.extract()?;
    let n = xy.shape()[0];
    if xy.shape()[1] != 2
        || level.len() != n
        || level_xy.shape() != [n, 2]
        || descriptors.shape() != [n, DESCRIPTOR_BYTES]
    {
        return Err(PyValueError::new_err(
            "features: expected one row per keypoint in every array",
        ));
    }
    let (xy, level) = (contiguous(&xy)?, contiguous(&level)?);
    let (level_xy, bytes) = (contiguous(&level_xy)?, contiguous(&descriptors)?);
    let keypoints = (0..n)
        .map(|i| {
            Ok(Keypoint {
                xy: [xy[2 * i], xy[2 * i + 1]],
                level: index(level[i])?,
                level_xy: [index(level_xy[2 * i])?, index(level_xy[2 * i + 1])?],
                response: 0,
                angle: 0,
            })
        })
        .collect::<PyResult<Vec<Keypoint>>>()?;
    Ok(Features {
        keypoints,
        descriptors: bytes.as_chunks::<DESCRIPTOR_BYTES>().0.to_vec(),
    })
}

/// Matches features along rows; returns a dict of `StereoMatches`' fields.
///
/// # Arguments
///
/// * `left_image` - The rectified left image.
/// * `right_image` - The rectified right image.
/// * `left` - The left features, as a dict.
/// * `right` - The right features, as a dict.
/// * `focal` - The rectified focal length.
/// * `baseline` - The baseline.
/// * `orb_options` - The extraction settings, as a dict.
/// * `options` - The matching settings, as a dict.
///
/// # Returns
///
/// A dict with `left`, `right` (int64), `right_x`, `disparity` and `depth` (float64).
///
/// # Errors
///
/// `ValueError` for a malformed array or dict; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(name = "match")]
#[allow(
    clippy::too_many_arguments,
    reason = "two images, their features, the camera, the settings"
)]
fn match_rows<'py>(
    py: Python<'py>,
    left_image: PyReadonlyArray2<'py, u8>,
    right_image: PyReadonlyArray2<'py, u8>,
    left: &Bound<'py, PyDict>,
    right: &Bound<'py, PyDict>,
    focal: f64,
    baseline: f64,
    orb_options: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let raw: OrbOptionsIn = pythonize::depythonize(orb_options.as_any())
        .map_err(|e| PyValueError::new_err(format!("orb_options: {e}")))?;
    let extraction = orb::Options {
        n_features: raw.n_features,
        levels: raw.levels,
        scale_factor: raw.scale_factor,
        fast_threshold: raw.fast_threshold,
        min_fast_threshold: raw.min_fast_threshold,
    };
    let raw: MatchOptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    let settings = core::MatchOptions {
        max_distance: raw.max_distance,
        band: raw.band,
        window: raw.window,
        search: raw.search,
        outlier_factor: raw.outlier_factor,
    };
    let matches = unwrap(
        py,
        core::match_rows(
            (&image_in(&left_image)?, &image_in(&right_image)?),
            &features_in(left)?,
            &features_in(right)?,
            focal,
            baseline,
            &extraction,
            &settings,
        ),
    )?;
    let to_i64 = |i: usize| i64::try_from(i).expect("indices fit an i64");
    let out = PyDict::new(py);
    let lefts: Vec<i64> = matches.iter().map(|m| to_i64(m.left)).collect();
    out.set_item("left", Array1::from_vec(lefts).into_pyarray(py))?;
    let rights: Vec<i64> = matches.iter().map(|m| to_i64(m.right)).collect();
    out.set_item("right", Array1::from_vec(rights).into_pyarray(py))?;
    for (key, value) in [
        (
            "right_x",
            matches.iter().map(|m| m.right_x).collect::<Vec<f64>>(),
        ),
        ("disparity", matches.iter().map(|m| m.disparity).collect()),
        ("depth", matches.iter().map(|m| m.depth).collect()),
    ] {
        out.set_item(key, Array1::from_vec(value).into_pyarray(py))?;
    }
    Ok(out)
}

/// Registers the `stereo` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn stereo(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; rectify, rectification_map, match_rows);
    Ok(())
}
