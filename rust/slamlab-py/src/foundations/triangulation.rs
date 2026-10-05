//! Bindings for `slamlab.foundations.triangulation` (`slamlab_rs.foundations.triangulation`).
//!
//! Each function returns a (P, 5) array: the point, `in_front` as 0 or 1, and the parallax.

use nalgebra::{Matrix4, Vector3};
use numpy::{PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::prelude::*;
use slamlab_foundations::triangulation::{self as core, Triangulation};

use crate::common::{
    ArrayIn, ArrayOut, add_functions, batch, contiguous, new_array, one, rows, unwrap,
};

/// Packs a triangulation as a new (P, 5) array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `result` - The triangulation.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// Never in practice (the shape always matches).
fn pack<'py>(py: Python<'py>, result: &Triangulation) -> PyResult<ArrayOut<'py>> {
    let mut out = Vec::with_capacity(5 * result.points.len());
    for ((point, &front), &parallax) in result
        .points
        .iter()
        .zip(&result.in_front)
        .zip(&result.parallax)
    {
        out.extend([
            point.x,
            point.y,
            point.z,
            if front { 1.0 } else { 0.0 },
            parallax,
        ]);
    }
    new_array(py, &[result.points.len(), 5], out)
}

/// Triangulates (P, 3) bearing pairs through the (4, 4) pose `T_c2_c1`.
///
/// # Arguments
///
/// * `relative_pose` - The (4, 4) relative pose.
/// * `bearings1` - (P, 3) unit bearings in camera 1.
/// * `bearings2` - (P, 3) unit bearings in camera 2.
///
/// # Returns
///
/// A new (P, 5) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` from the core.
#[pyfunction]
fn two_view<'py>(
    py: Python<'py>,
    relative_pose: ArrayIn<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let pose: Matrix4<f64> = one(&relative_pose)?;
    let result = core::two_view(
        &pose,
        &rows::<Vector3<f64>>(&bearings1)?,
        &rows::<Vector3<f64>>(&bearings2)?,
    );
    pack(py, &unwrap(py, result)?)
}

/// Triangulates (P, V, 3) bearings from (V, 4, 4) poses where observed.
///
/// # Arguments
///
/// * `poses` - (V, 4, 4) `T_cam_world`.
/// * `bearings` - (P, V, 3) unit bearings.
/// * `observed` - (P, V) observation mask.
///
/// # Returns
///
/// A new (P, 5) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` from the core.
#[pyfunction]
fn n_view<'py>(
    py: Python<'py>,
    poses: ArrayIn<'py>,
    bearings: ArrayIn<'py>,
    observed: PyReadonlyArray2<'py, bool>,
) -> PyResult<ArrayOut<'py>> {
    let views = observed.shape()[1];
    let (rays, _) = batch(&bearings, &[views, 3])?;
    let rays: Vec<Vector3<f64>> = rays
        .as_chunks::<3>()
        .0
        .iter()
        .map(|&r| Vector3::from(r))
        .collect();
    let result = core::n_view(
        &rows::<Matrix4<f64>>(&poses)?,
        &rays,
        contiguous(&observed)?,
    );
    pack(py, &unwrap(py, result)?)
}

/// Registers the `triangulation` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn triangulation(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; two_view, n_view);
    Ok(())
}
