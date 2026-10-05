//! Bindings for `slamlab.foundations.pnp` (`slamlab_rs.foundations.pnp`).

use nalgebra::Matrix4;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slamlab_foundations::p3p::Solver;
use slamlab_foundations::pnp as core;

use super::{nls_crossing, ransac_crossing};
use crate::common::{
    ArrayIn, ArrayOut, add_functions, decode, enum_names, item_array, one, rows, unwrap,
};

/// EPnP's pose.
///
/// # Arguments
///
/// * `points` - (N, 3) world points, in the world frame's units.
/// * `bearings` - (N, 3) unit bearings.
///
/// # Returns
///
/// A new (4, 4) array, `T_cam_world`: it maps world points into the camera.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn epnp<'py>(
    py: Python<'py>,
    points: ArrayIn<'py>,
    bearings: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let found = core::epnp(&rows(&points)?, &rows(&bearings)?);
    item_array(py, &unwrap(py, found)?)
}

/// Refines a pose on its inliers' reprojection errors.
///
/// # Arguments
///
/// * `points` - The inliers' (M, 3) world points, in the world frame's units.
/// * `bearings` - Their (M, 3) bearings.
/// * `pose` - The starting (4, 4) `T_cam_world`.
/// * `scale` - The Cauchy loss's scale.
///
/// # Returns
///
/// `{"pose": (4, 4), "summary": the solve's summary}`.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` from the core.
#[pyfunction]
fn refine_pose<'py>(
    py: Python<'py>,
    points: ArrayIn<'py>,
    bearings: ArrayIn<'py>,
    pose: ArrayIn<'py>,
    scale: f64,
) -> PyResult<Bound<'py, PyDict>> {
    let solved = core::refine_pose(
        &one::<Matrix4<f64>>(&pose)?,
        &rows(&points)?,
        &rows(&bearings)?,
        scale,
    );
    nls_crossing::refined(py, "pose", &unwrap(py, solved)?)
}

/// LO-MSAC with a P3P solver and EPnP refits, then (optionally) the refinement.
///
/// # Arguments
///
/// * `points` - (N, 3) world points, in the world frame's units.
/// * `bearings` - Their (N, 3) bearings.
/// * `options` - `ransac.Options`' fields.
/// * `minimal_solver` - The P3P solver's code (an index into `p3p.SOLVERS`).
/// * `refine` - Whether to refine the consensus pose.
///
/// # Returns
///
/// The solution as a dict.
///
/// # Errors
///
/// `ValueError` for malformed arguments; `InvalidInputError` or `DegenerateError` from the
/// core.
#[pyfunction]
fn pnp_ransac<'py>(
    py: Python<'py>,
    points: ArrayIn<'py>,
    bearings: ArrayIn<'py>,
    options: Bound<'py, PyDict>,
    minimal_solver: i64,
    refine: bool,
) -> PyResult<Bound<'py, PyDict>> {
    let found = core::pnp_ransac(
        &rows(&points)?,
        &rows(&bearings)?,
        &ransac_crossing::read_options(&options)?,
        decode(&Solver::ALL, minimal_solver, "minimal_solver")?,
        refine,
    );
    ransac_crossing::refined_result(py, &unwrap(py, found)?)
}

/// The enum tables: the P3P solvers, RANSAC's and NLS' terminations.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"minimal_solver": [...], "ransac_termination": [...], "nls_termination": [...]}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
#[pyfunction]
fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = ransac_crossing::refined_codes(py)?;
    out.set_item("minimal_solver", enum_names(&Solver::ALL, Solver::name))?;
    Ok(out)
}

/// Registers the `pnp` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn pnp(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, epnp, refine_pose, pnp_ransac);
    Ok(())
}
