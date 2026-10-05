//! Shared by every binding that returns or takes ORB features (`orb`, `stereo`): they cross as
//! a dict of `slamlab.foundations.orb.Keypoints`' fields, one array each, with `descriptors`,
//! `scale_factor` and `levels` for features.

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slamlab_foundations::orb::{DESCRIPTOR_BYTES, Features, Keypoint};

use crate::common::{contiguous, int64_array, to_i64};

/// Wraps row-major pairs as a new (N, 2) array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `pairs` - The rows.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the shape does not match (a binding bug).
fn pairs_array<T: numpy::Element>(
    py: Python<'_>,
    pairs: Vec<[T; 2]>,
) -> PyResult<Bound<'_, PyArray2<T>>> {
    let n = pairs.len();
    let flat: Vec<T> = pairs.into_iter().flatten().collect();
    let array =
        Array2::from_shape_vec((n, 2), flat).map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}

/// Builds the dict of `Keypoints`' fields, one array each.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `keypoints` - The keypoints.
///
/// # Returns
///
/// A dict with `xy`, `level`, `level_xy`, `response` and `angle`.
///
/// # Errors
///
/// A Python error if an item cannot be set.
pub fn keypoints_dict<'py>(
    py: Python<'py>,
    keypoints: &[Keypoint],
) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item(
        "xy",
        pairs_array(py, keypoints.iter().map(|k| k.xy).collect())?,
    )?;
    let levels = keypoints.iter().map(|k| to_i64(k.level)).collect();
    out.set_item("level", int64_array(py, levels))?;
    let level_xy = keypoints.iter().map(|k| k.level_xy.map(to_i64)).collect();
    out.set_item("level_xy", pairs_array(py, level_xy)?)?;
    let responses = keypoints.iter().map(|k| k.response).collect();
    out.set_item("response", int64_array(py, responses))?;
    let angles = keypoints.iter().map(|k| to_i64(k.angle)).collect();
    out.set_item("angle", int64_array(py, angles))?;
    Ok(out)
}

/// Builds the dict of `Features`' fields: the keypoints' fields, `descriptors` (N, 32) uint8,
/// `scale_factor` and `levels`.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `features` - The features.
///
/// # Returns
///
/// The dict.
///
/// # Errors
///
/// A Python error if an item cannot be set.
pub fn features_dict<'py>(py: Python<'py>, features: &Features) -> PyResult<Bound<'py, PyDict>> {
    let out = keypoints_dict(py, &features.keypoints)?;
    let n = features.descriptors.len();
    let bytes = features.descriptors.concat();
    let descriptors = Array2::from_shape_vec((n, DESCRIPTOR_BYTES), bytes)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    out.set_item("descriptors", descriptors.into_pyarray(py))?;
    out.set_item("scale_factor", features.scale_factor)?;
    out.set_item("levels", features.levels)?;
    Ok(out)
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
///   `descriptors` (N, 32) uint8, `scale_factor` and `levels`.
///
/// # Returns
///
/// The features (responses and angles, unused by matching, are zero).
///
/// # Errors
///
/// `ValueError` for a missing or unknown key or a malformed array.
pub fn features_in(features: &Bound<'_, PyDict>) -> PyResult<Features> {
    let keys = [
        "xy",
        "level",
        "level_xy",
        "descriptors",
        "scale_factor",
        "levels",
    ];
    if features.len() != keys.len() {
        return Err(PyValueError::new_err(format!(
            "features: expected exactly the keys {keys:?}"
        )));
    }
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
        scale_factor: item(features, "scale_factor")?.extract()?,
        levels: item(features, "levels")?.extract()?,
    })
}
