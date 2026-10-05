//! Bindings for `slamlab.foundations.orb` (`slamlab_rs.foundations.orb`).
//!
//! Options cross as one dict of `slamlab.foundations.orb.Options`' fields, read strictly by
//! serde; keypoints come back as a dict of `Keypoints`' fields, one array each, with
//! `descriptors` for features.

use numpy::ndarray::{Array1, Array2};
use numpy::{
    IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slamlab_foundations::orb::{self as core, Bounds, Keypoint};

use super::image_crossing::image_in;
use crate::common::{add_functions, contiguous, unwrap};

/// `orb.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OptionsIn {
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

/// Converts a non-negative index to `i64` for numpy.
///
/// # Arguments
///
/// * `value` - The index.
///
/// # Returns
///
/// It as an `i64`.
fn to_i64(value: usize) -> i64 {
    i64::try_from(value).expect("indices fit an i64")
}

/// Wraps values as a new 1D int64 array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `values` - The values.
///
/// # Returns
///
/// The array.
fn int64_array(py: Python<'_>, values: Vec<i64>) -> Bound<'_, PyArray1<i64>> {
    Array1::from_vec(values).into_pyarray(py)
}

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

/// Returns every pixel's FAST-9 score.
///
/// # Arguments
///
/// * `image` - The image.
///
/// # Returns
///
/// A new (H, W) int64 array.
///
/// # Errors
///
/// `ValueError` for a non-contiguous array; `InvalidInputError` from the core.
#[pyfunction]
fn fast_scores<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
) -> PyResult<Bound<'py, PyArray2<i64>>> {
    let pixels = image_in(&image)?;
    let (rows, cols) = (pixels.rows, pixels.cols);
    let scores = unwrap(py, core::fast_scores(&pixels))?;
    let array = Array2::from_shape_vec((rows, cols), scores)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}

/// Returns the indices of the corners a quadtree keeps.
///
/// # Arguments
///
/// * `xy` - The corners' pixels, (N, 2) int64.
/// * `response` - Their strengths, (N,) int64.
/// * `bounds` - `(x_min, x_max, y_min, y_max)`.
/// * `target` - How many to keep.
///
/// # Returns
///
/// The kept indices, a new int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed array or a negative coordinate; `InvalidInputError` from the
/// core.
#[pyfunction]
fn distribute<'py>(
    py: Python<'py>,
    xy: PyReadonlyArray2<'py, i64>,
    response: PyReadonlyArray1<'py, i64>,
    bounds: [usize; 4],
    target: usize,
) -> PyResult<Bound<'py, PyArray1<i64>>> {
    if xy.shape()[1] != 2 {
        return Err(PyValueError::new_err(format!(
            "expected shape (N, 2), got {:?}",
            xy.shape()
        )));
    }
    let pixels = contiguous(&xy)?
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&[x, y]| Ok([usize::try_from(x)?, usize::try_from(y)?]))
        .collect::<Result<Vec<[usize; 2]>, std::num::TryFromIntError>>()
        .map_err(|e| PyValueError::new_err(format!("xy: {e}")))?;
    let [x_min, x_max, y_min, y_max] = bounds;
    let box_ = Bounds {
        x_min,
        x_max,
        y_min,
        y_max,
    };
    let kept = unwrap(
        py,
        core::distribute(&pixels, contiguous(&response)?, box_, target),
    )?;
    Ok(int64_array(py, kept.into_iter().map(to_i64).collect()))
}

/// Reads the options dict.
///
/// # Arguments
///
/// * `options` - The dict, with exactly `orb.Options`' fields.
///
/// # Returns
///
/// The settings (checked by the core when it runs).
///
/// # Errors
///
/// `ValueError` for a missing or unknown field, or a value of the wrong type.
fn read_options(options: &Bound<'_, PyDict>) -> PyResult<core::Options> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    Ok(core::Options {
        n_features: raw.n_features,
        levels: raw.levels,
        scale_factor: raw.scale_factor,
        fast_threshold: raw.fast_threshold,
        min_fast_threshold: raw.min_fast_threshold,
    })
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
fn keypoints_dict<'py>(py: Python<'py>, keypoints: &[Keypoint]) -> PyResult<Bound<'py, PyDict>> {
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

/// Detects oriented keypoints; returns a dict of `Keypoints`' fields.
///
/// # Arguments
///
/// * `image` - The image.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// A dict with `xy`, `level`, `level_xy`, `response` and `angle`.
///
/// # Errors
///
/// `ValueError` for a malformed array or dict; `InvalidInputError` from the core.
#[pyfunction]
fn detect<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let keypoints = unwrap(
        py,
        core::detect(&image_in(&image)?, &read_options(options)?),
    )?;
    keypoints_dict(py, &keypoints)
}

/// Extracts features: `Keypoints`' fields, and `descriptors`.
///
/// # Arguments
///
/// * `image` - The image.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The keypoints' dict with `descriptors`, an (N, 32) uint8 array.
///
/// # Errors
///
/// `ValueError` for a malformed array or dict; `InvalidInputError` from the core.
#[pyfunction]
fn extract<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let features = unwrap(
        py,
        core::extract(&image_in(&image)?, &read_options(options)?),
    )?;
    let out = keypoints_dict(py, &features.keypoints)?;
    let n = features.descriptors.len();
    let bytes = features.descriptors.concat();
    let descriptors = Array2::from_shape_vec((n, core::DESCRIPTOR_BYTES), bytes)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    out.set_item("descriptors", descriptors.into_pyarray(py))?;
    Ok(out)
}

/// Registers the `orb` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn orb(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; fast_scores, distribute, detect, extract);
    Ok(())
}
