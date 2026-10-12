//! Bindings for `slam_it.edge.residuals` (`slam_it_rs.edge.residuals`).

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_edge::residuals as core;

use super::edge_crossing::{camera_in, rows, vector, vector_array, vectors};
use crate::common::{add_functions, contiguous, unwrap};

/// Each observation's chi-square error.
///
/// # Arguments
///
/// * `camera` - The camera, as a dict.
/// * `local` - The points in the camera's frame, (N, 3).
/// * `xy` - Their observed pixels, (N, 2).
/// * `right` - Their right columns (NaN for none), (N,).
/// * `variance` - Their levels' variances, (N,).
///
/// # Returns
///
/// A new (N,) float64 array.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (camera, local, xy, right, variance))]
fn chi2<'py>(
    py: Python<'py>,
    camera: &Bound<'py, PyDict>,
    local: Bound<'py, PyAny>,
    xy: Bound<'py, PyAny>,
    right: PyReadonlyArray1<'py, f64>,
    variance: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let errors = unwrap(
        py,
        core::chi2(
            &camera_in(camera)?,
            &vectors(&local)?,
            &rows::<f64, 2>(&xy)?,
            contiguous(&right)?,
            &vector::<f64>(&variance)?,
        ),
    )?;
    Ok(vector_array(py, &errors))
}

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn residuals(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; chi2);
    Ok(())
}
