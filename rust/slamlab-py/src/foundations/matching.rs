//! Bindings for `slamlab.foundations.matching` (`slamlab_rs.foundations.matching`).
//!
//! Descriptor sets cross as C-contiguous (N, 32) uint8 arrays, orientation bins as int64
//! arrays, and the filters as one dict of `slamlab.foundations.matching.Options`' fields, read
//! strictly by serde; matches come back as a dict of `Matches`' fields.

use numpy::ndarray::{Array1, Array2};
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slamlab_foundations::matching as core;
use slamlab_foundations::orb::{DESCRIPTOR_BYTES, Descriptor};

use crate::common::{add_functions, contiguous, unwrap};

/// `matching.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OptionsIn {
    /// The distance cap.
    max_distance: i64,
    /// The ratio test's bound.
    ratio: f64,
    /// Whether to check rotation consistency.
    check_orientation: bool,
}

/// Copies a descriptor set in.
///
/// # Arguments
///
/// * `descriptors` - An (N, 32) uint8 array.
///
/// # Returns
///
/// The descriptors.
///
/// # Errors
///
/// `ValueError` for another shape or a non-contiguous array.
fn descriptors_in(descriptors: &PyReadonlyArray2<'_, u8>) -> PyResult<Vec<Descriptor>> {
    if descriptors.shape()[1] != DESCRIPTOR_BYTES {
        return Err(PyValueError::new_err(format!(
            "expected shape (N, {DESCRIPTOR_BYTES}), got {:?}",
            descriptors.shape()
        )));
    }
    Ok(contiguous(descriptors)?
        .as_chunks::<DESCRIPTOR_BYTES>()
        .0
        .to_vec())
}

/// Copies orientation bins in.
///
/// # Arguments
///
/// * `angles` - An int64 array.
///
/// # Returns
///
/// The bins.
///
/// # Errors
///
/// `ValueError` for a negative bin or a non-contiguous array.
fn angles_in(angles: &PyReadonlyArray1<'_, i64>) -> PyResult<Vec<usize>> {
    contiguous(angles)?
        .iter()
        .map(|&a| usize::try_from(a).map_err(|e| PyValueError::new_err(format!("angles: {e}"))))
        .collect()
}

/// Wraps row-major values as a new int64 array of the given shape.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `shape` - `(rows, cols)`.
/// * `values` - The values.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the count does not match the shape (a binding bug).
fn int64_matrix(
    py: Python<'_>,
    shape: (usize, usize),
    values: Vec<i64>,
) -> PyResult<Bound<'_, PyArray2<i64>>> {
    let array =
        Array2::from_shape_vec(shape, values).map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}

/// Converts an index to `i64` for numpy.
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

/// Returns every pair's Hamming distance.
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
///
/// # Returns
///
/// A new (N, M) int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` from the core.
#[pyfunction]
fn distances<'py>(
    py: Python<'py>,
    descriptors1: PyReadonlyArray2<'py, u8>,
    descriptors2: PyReadonlyArray2<'py, u8>,
) -> PyResult<Bound<'py, PyArray2<i64>>> {
    let (first, second) = (
        descriptors_in(&descriptors1)?,
        descriptors_in(&descriptors2)?,
    );
    let values = unwrap(py, core::distances(&first, &second))?;
    int64_matrix(py, (first.len(), second.len()), values)
}

/// Matches two descriptor sets; returns a dict of `Matches`' fields.
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
/// * `angles1` - The first set's orientation bins.
/// * `angles2` - The second set's.
/// * `options` - The filters, as a dict.
///
/// # Returns
///
/// A dict with `pairs` (K, 2) and `distances` (K,), both int64.
///
/// # Errors
///
/// `ValueError` for a malformed array or dict; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(name = "match")]
fn match_descriptors<'py>(
    py: Python<'py>,
    descriptors1: PyReadonlyArray2<'py, u8>,
    descriptors2: PyReadonlyArray2<'py, u8>,
    angles1: PyReadonlyArray1<'py, i64>,
    angles2: PyReadonlyArray1<'py, i64>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    let settings = core::Options {
        max_distance: raw.max_distance,
        ratio: raw.ratio,
        check_orientation: raw.check_orientation,
    };
    let matches = unwrap(
        py,
        core::match_descriptors(
            &descriptors_in(&descriptors1)?,
            &descriptors_in(&descriptors2)?,
            &angles_in(&angles1)?,
            &angles_in(&angles2)?,
            &settings,
        ),
    )?;
    let pairs = matches
        .iter()
        .flat_map(|m| [to_i64(m.first), to_i64(m.second)])
        .collect();
    let out = PyDict::new(py);
    out.set_item("pairs", int64_matrix(py, (matches.len(), 2), pairs)?)?;
    let distances: Vec<i64> = matches.iter().map(|m| m.distance).collect();
    out.set_item("distances", Array1::from_vec(distances).into_pyarray(py))?;
    Ok(out)
}

/// Registers the `matching` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn matching(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; distances, match_descriptors);
    Ok(())
}
