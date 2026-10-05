//! Bindings for `slamlab.foundations.epipolar` (`slamlab_rs.foundations.epipolar`).

use nalgebra::{Matrix3, Vector2, Vector3};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slamlab_foundations::epipolar::{self as core, MinimalSolver};

use super::{nls_crossing, ransac_crossing};
use crate::common::{
    ArrayIn, ArrayOut, add_functions, decode, enum_names, item_array, items_array, one, rows,
    unwrap, vector_array,
};

/// E from (N, 3) bearing pairs: `x2ᵀ E x1 = 0`, camera 1 then camera 2.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - (N, 3) bearings in cameras 1 and 2.
///
/// # Returns
///
/// A new (3, 3) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn essential_8pt<'py>(
    py: Python<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let found = core::essential_8pt(&rows(&bearings1)?, &rows(&bearings2)?);
    item_array(py, &unwrap(py, found)?)
}

/// F from (N, 2) pixel pairs: `x2ᵀ F x1 = 0`, image 1 then image 2.
///
/// # Arguments
///
/// * `pixels1`, `pixels2` - (N, 2) pixels in images 1 and 2.
///
/// # Returns
///
/// A new (3, 3) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn fundamental_8pt<'py>(
    py: Python<'py>,
    pixels1: ArrayIn<'py>,
    pixels2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let found = core::fundamental_8pt(&rows(&pixels1)?, &rows(&pixels2)?);
    item_array(py, &unwrap(py, found)?)
}

/// Squared Sampson errors under E.
///
/// # Arguments
///
/// * `essential` - (3, 3) E.
/// * `bearings1`, `bearings2` - (N, 3) bearings in cameras 1 and 2.
///
/// # Returns
///
/// A new (N,) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` from the core.
#[pyfunction]
fn sampson_essential<'py>(
    py: Python<'py>,
    essential: ArrayIn<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let errors = core::sampson_essential(
        &one::<Matrix3<f64>>(&essential)?,
        &rows::<Vector3<f64>>(&bearings1)?,
        &rows::<Vector3<f64>>(&bearings2)?,
    );
    vector_array(py, &unwrap(py, errors)?)
}

/// Squared Sampson errors under F.
///
/// # Arguments
///
/// * `fundamental` - (3, 3) F.
/// * `pixels1`, `pixels2` - (N, 2) pixels in images 1 and 2.
///
/// # Returns
///
/// A new (N,) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` from the core.
#[pyfunction]
fn sampson_fundamental<'py>(
    py: Python<'py>,
    fundamental: ArrayIn<'py>,
    pixels1: ArrayIn<'py>,
    pixels2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let errors = core::sampson_fundamental(
        &one::<Matrix3<f64>>(&fundamental)?,
        &rows::<Vector2<f64>>(&pixels1)?,
        &rows::<Vector2<f64>>(&pixels2)?,
    );
    vector_array(py, &unwrap(py, errors)?)
}

/// E's four candidate poses.
///
/// # Arguments
///
/// * `essential` - (3, 3) E.
///
/// # Returns
///
/// A new (4, 4, 4) array: the candidates `T_c2_c1`, each with a unit translation.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn decompose_essential<'py>(py: Python<'py>, essential: ArrayIn<'py>) -> PyResult<ArrayOut<'py>> {
    let poses = core::decompose_essential(&one::<Matrix3<f64>>(&essential)?);
    items_array(py, &unwrap(py, poses)?)
}

/// Refines E on its inliers' Sampson distances.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - The inliers' (M, 3) bearings in cameras 1 and 2.
/// * `essential` - The starting (3, 3) E.
/// * `scale` - The Cauchy loss's scale.
///
/// # Returns
///
/// `{"essential": (3, 3), "summary": the solve's summary}`.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn refine_essential<'py>(
    py: Python<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
    essential: ArrayIn<'py>,
    scale: f64,
) -> PyResult<Bound<'py, PyDict>> {
    let solved = core::refine_essential(
        &one::<Matrix3<f64>>(&essential)?,
        &rows(&bearings1)?,
        &rows(&bearings2)?,
        scale,
    );
    nls_crossing::refined(py, "essential", &unwrap(py, solved)?)
}

/// LO-MSAC with a minimal E solver, then (optionally) the Sampson refinement.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - (N, 3) bearings in cameras 1 and 2.
/// * `options` - `ransac.Options`' fields.
/// * `minimal_solver` - The minimal solver's code.
/// * `refine` - Whether to refine the consensus model.
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
fn essential_ransac<'py>(
    py: Python<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
    options: Bound<'py, PyDict>,
    minimal_solver: i64,
    refine: bool,
) -> PyResult<Bound<'py, PyDict>> {
    let found = core::essential_ransac(
        &rows(&bearings1)?,
        &rows(&bearings2)?,
        &ransac_crossing::read_options(&options)?,
        decode(&MinimalSolver::ALL, minimal_solver, "minimal_solver")?,
        refine,
    );
    ransac_crossing::refined_result(py, &unwrap(py, found)?)
}

/// LO-MSAC with the normalised 8-point F solver.
///
/// # Arguments
///
/// * `pixels1`, `pixels2` - (N, 2) pixels in images 1 and 2.
/// * `options` - `ransac.Options`' fields.
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
fn fundamental_ransac<'py>(
    py: Python<'py>,
    pixels1: ArrayIn<'py>,
    pixels2: ArrayIn<'py>,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = core::fundamental_ransac(
        &rows(&pixels1)?,
        &rows(&pixels2)?,
        &ransac_crossing::read_options(&options)?,
    );
    ransac_crossing::ransac_result(py, &unwrap(py, found)?)
}

/// The enums' name tables: RANSAC's terminations, the minimal solvers and the refinement's
/// terminations.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"ransac_termination": [...], "nls_termination": [...], "minimal_solver": [...]}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
#[pyfunction]
fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = ransac_crossing::refined_codes(py)?;
    out.set_item(
        "minimal_solver",
        enum_names(&MinimalSolver::ALL, MinimalSolver::name),
    )?;
    Ok(out)
}

/// Registers the `epipolar` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn epipolar(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(
        m;
        codes,
        essential_8pt,
        fundamental_8pt,
        sampson_essential,
        sampson_fundamental,
        decompose_essential,
        refine_essential,
        essential_ransac,
        fundamental_ransac
    );
    Ok(())
}
