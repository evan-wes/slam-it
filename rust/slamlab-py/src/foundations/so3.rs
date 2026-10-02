//! Bindings for `slamlab.foundations.so3` (`slamlab_rs.foundations.so3`).

use numpy::{PyArray1, PyArray2, PyArrayMethods, PyReadonlyArray1};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use slamlab_foundations::so3 as core;

/// Maps a 3-vector to its skew-symmetric matrix.
///
/// # Arguments
///
/// * `v` - A contiguous float64 array of shape (3,). The Python entry point validates first;
///   a direct caller passing anything else gets a `TypeError` (wrong dtype or not an array)
///   or `ValueError` (wrong length or non-contiguous).
///
/// # Returns
///
/// A new float64 array of shape (3, 3): the skew-symmetric matrix `[v]x`.
///
/// # Errors
///
/// Returns `ValueError` if `v` does not hold exactly three contiguous values.
#[pyfunction]
#[allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]
fn hat<'py>(py: Python<'py>, v: PyReadonlyArray1<'py, f64>) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let values = v
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let vec: core::Vec3 = values.try_into().map_err(|_| {
        PyValueError::new_err(format!("expected a 3-vector, got {} values", values.len()))
    })?;
    let flat: Vec<f64> = core::hat(vec).iter().flatten().copied().collect();
    PyArray1::from_vec(py, flat).reshape([3, 3])
}

/// Registers the `so3` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn so3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hat, m)?)?;
    Ok(())
}
