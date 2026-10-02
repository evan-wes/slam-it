//! Bindings for `slamlab.foundations.so3` (`slamlab_rs.foundations.so3`).
//!
//! Rust implements a curated subset of each module (design doc, "Languages"); the Protocol
//! test checks that every function present here matches the Python contract. Functions are
//! batched: float64 arrays of shape (N, ...) in, new arrays out.

use numpy::ndarray::Array3;
use numpy::{IntoPyArray, PyArray3, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use slamlab_foundations::so3 as core;

/// Maps (N, 3) vectors to (N, 3, 3) skew-symmetric matrices.
///
/// # Arguments
///
/// * `omega` - A contiguous float64 array of shape (N, 3). The Python entry point validates
///   first; a direct caller passing anything else gets a `TypeError` (wrong dtype or not an
///   array) or `ValueError` (wrong shape, non-contiguous or misaligned).
///
/// # Returns
///
/// A new float64 array of shape (N, 3, 3).
///
/// # Errors
///
/// Returns `ValueError` if `omega` is not a contiguous (N, 3) array.
#[pyfunction]
#[allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]
fn hat<'py>(
    py: Python<'py>,
    omega: PyReadonlyArray2<'py, f64>,
) -> PyResult<Bound<'py, PyArray3<f64>>> {
    let values = omega
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let (n, cols) = omega.as_array().dim();
    if cols != 3 {
        return Err(PyValueError::new_err(format!(
            "expected shape (N, 3), got ({n}, {cols})"
        )));
    }
    let mut out = Array3::<f64>::zeros((n, 3, 3));
    for (row, mut item) in values.as_chunks::<3>().0.iter().zip(out.outer_iter_mut()) {
        let h = core::hat(*row);
        for (i, h_row) in h.iter().enumerate() {
            for (j, value) in h_row.iter().enumerate() {
                item[[i, j]] = *value;
            }
        }
    }
    Ok(out.into_pyarray(py))
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
