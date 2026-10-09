//! Bindings for `slam_it_all.foundations.alignment` (`slam_it_all_rs.foundations.alignment`).

use nalgebra::{Matrix4, Vector3};
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use slam_it_all_foundations::alignment as core;

use crate::common::{ArrayIn, ArrayOut, Item, add_functions, new_array, rows, unwrap};

/// Aligns (N, 3) point sets; returns the (4, 4) matrix `[[s R, t], [0, 1]]`.
///
/// # Arguments
///
/// * `source` - Source points, (N, 3).
/// * `target` - Target points, (N, 3).
/// * `weights` - Non-negative weights, (N,).
/// * `with_scale` - Estimate a scale too.
///
/// # Returns
///
/// A new (4, 4) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn umeyama<'py>(
    py: Python<'py>,
    source: ArrayIn<'py>,
    target: ArrayIn<'py>,
    weights: PyReadonlyArray1<'py, f64>,
    with_scale: bool,
) -> PyResult<ArrayOut<'py>> {
    let weights = weights
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let result: Matrix4<f64> = unwrap(
        py,
        core::umeyama(
            &rows::<Vector3<f64>>(&source)?,
            &rows::<Vector3<f64>>(&target)?,
            weights,
            with_scale,
        ),
    )?;
    let mut out = Vec::with_capacity(16);
    result.write(&mut out);
    new_array(py, &[4, 4], out)
}

/// Registers the `alignment` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn alignment(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; umeyama);
    Ok(())
}
