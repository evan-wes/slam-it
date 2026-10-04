//! Bindings for `slamlab.foundations.camera` (`slamlab_rs.foundations.camera`).
//!
//! The model crosses as its integer code, the intrinsics as one flat float64 array, and the
//! points or pixels as a batch; each item is computed by the core in
//! `slamlab_foundations::camera`.

use nalgebra::{Vector2, Vector3};
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use slamlab_foundations::camera::{self as core, Model};

use crate::common::{
    ArrayIn, ArrayOut, add_functions, batch, decode, map_batch, new_array, unwrap,
};

/// Reads a model code and its intrinsics.
///
/// # Arguments
///
/// * `model` - The model's code.
/// * `intrinsics` - A contiguous float64 vector.
///
/// # Returns
///
/// The model and the intrinsics as a slice.
///
/// # Errors
///
/// `ValueError` for an unknown code or a non-contiguous array.
fn read_camera<'a>(
    model: i64,
    intrinsics: &'a PyReadonlyArray1<'_, f64>,
) -> PyResult<(Model, &'a [f64])> {
    let values = intrinsics
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok((decode(&Model::ALL, model, "model")?, values))
}

/// Projects (N, 3) camera-frame points to (N, 2) pixels.
///
/// # Arguments
///
/// * `model` - The model's code.
/// * `intrinsics` - The model's intrinsics.
/// * `points` - The points, (N, 3).
///
/// # Returns
///
/// A new (N, 2) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays or an unknown code; `InvalidInputError` from the core.
#[pyfunction]
#[allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]
fn project<'py>(
    py: Python<'py>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    points: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let (model, k) = read_camera(model, &intrinsics)?;
    map_batch::<Vector3<f64>, Vector2<f64>>(py, &points, |p| core::project(model, k, p))
}

/// Returns (N, 2, 3 + K): d(pixel)/d(point) then d(pixel)/d(intrinsics).
///
/// # Arguments
///
/// * `model` - The model's code.
/// * `intrinsics` - The model's intrinsics, K of them.
/// * `points` - The points, (N, 3).
///
/// # Returns
///
/// A new (N, 2, 3 + K) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays or an unknown code; `InvalidInputError` from the core.
#[pyfunction]
#[allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]
fn project_jacobians<'py>(
    py: Python<'py>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    points: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let (model, k) = read_camera(model, &intrinsics)?;
    let (values, n) = batch(&points, &[3])?;
    let cols = 3 + k.len();
    let mut out = Vec::with_capacity(n * 2 * cols);
    for item in values.as_chunks::<3>().0 {
        let jacobian = unwrap(py, core::project_jacobian(model, k, &Vector3::from(*item)))?;
        for row in jacobian.row_iter() {
            out.extend(row.iter());
        }
    }
    new_array(py, &[n, 2, cols], out)
}

/// Maps (N, 2) pixels to (N, 3) unit bearing vectors.
///
/// # Arguments
///
/// * `model` - The model's code.
/// * `intrinsics` - The model's intrinsics.
/// * `pixels` - The pixels, (N, 2).
///
/// # Returns
///
/// A new (N, 3) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays or an unknown code; `InvalidInputError` or
/// `NotConvergedError` from the core.
#[pyfunction]
#[allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]
fn unproject<'py>(
    py: Python<'py>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    pixels: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let (model, k) = read_camera(model, &intrinsics)?;
    map_batch::<Vector2<f64>, Vector3<f64>>(py, &pixels, |p| core::unproject(model, k, p))
}

/// Registers the `camera` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn camera(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; project, project_jacobians, unproject);
    Ok(())
}
