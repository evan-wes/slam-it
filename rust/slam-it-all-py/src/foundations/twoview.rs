//! Bindings for `slam_it_all.foundations.twoview` (`slam_it_all_rs.foundations.twoview`).

use numpy::IntoPyArray;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slam_it_all_foundations::epipolar::MinimalSolver;
use slam_it_all_foundations::twoview::{self as core, Model, Status};

use super::ransac_crossing;
use crate::common::{
    ArrayIn, add_functions, by_name, encode, enum_names, item_array, items_array, rows, unwrap,
};

/// `twoview.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OptionsIn {
    /// The focal length in pixels.
    focal: f64,
    /// E-RANSAC's inlier threshold, in pixels.
    threshold_px: f64,
    /// The image noise for GRIC, in pixels.
    sigma_px: f64,
    /// RANSAC's target confidence.
    confidence: f64,
    /// RANSAC's sample limit, per model.
    max_iterations: i64,
    /// The random seed.
    seed: u64,
    /// The parallax a point needs, in degrees.
    min_parallax_deg: f64,
    /// The well-triangulated points initialisation needs.
    min_points: i64,
    /// The runner-up's fraction of the best's points in front.
    ambiguity_ratio: f64,
    /// E-RANSAC's minimal solver, by name.
    essential_solver: String,
    /// Whether E-RANSAC's model is refined.
    refine: bool,
}

/// Reads the initialiser's options from a dict of scalars.
///
/// # Arguments
///
/// * `options` - The dict, with exactly `twoview.Options`' fields.
///
/// # Returns
///
/// The options (checked by the core).
///
/// # Errors
///
/// `ValueError` for a missing or unknown field, or an unknown solver name.
fn read_options(options: &Bound<'_, PyDict>) -> PyResult<core::Options> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    Ok(core::Options {
        focal: raw.focal,
        threshold_px: raw.threshold_px,
        sigma_px: raw.sigma_px,
        confidence: raw.confidence,
        max_iterations: raw.max_iterations,
        seed: raw.seed,
        min_parallax_deg: raw.min_parallax_deg,
        min_points: raw.min_points,
        ambiguity_ratio: raw.ambiguity_ratio,
        essential_solver: by_name(
            &MinimalSolver::ALL,
            MinimalSolver::name,
            &raw.essential_solver,
            "essential_solver",
        )?,
        refine: raw.refine,
    })
}

/// Initialises two views.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - (N, 3) bearings in cameras 1 and 2.
/// * `options` - `twoview.Options`' fields.
///
/// # Returns
///
/// The result as a dict.
///
/// # Errors
///
/// `ValueError` for malformed arguments; `InvalidInputError`, or `DegenerateError` if neither
/// RANSAC found a model.
#[pyfunction]
fn initialize<'py>(
    py: Python<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = core::initialize(
        &rows(&bearings1)?,
        &rows(&bearings2)?,
        &read_options(&options)?,
    );
    let result = unwrap(py, found)?;
    let out = PyDict::new(py);
    out.set_item("status", encode(&Status::ALL, result.status)?)?;
    out.set_item("model", encode(&Model::ALL, result.model)?)?;
    out.set_item("fallback", result.fallback)?;
    out.set_item("relative_pose", item_array(py, &result.relative_pose)?)?;
    out.set_item("inliers", result.inliers.into_pyarray(py))?;
    out.set_item("points", items_array(py, &result.points)?)?;
    out.set_item("triangulated", result.triangulated.into_pyarray(py))?;
    out.set_item("gric_essential", result.gric_essential)?;
    out.set_item("gric_homography", result.gric_homography)?;
    let in_front: Vec<i64> = result
        .in_front
        .iter()
        .map(|&count| i64::try_from(count).unwrap_or(i64::MAX))
        .collect();
    out.set_item("in_front", in_front.into_pyarray(py))?;
    let essential = result
        .essential
        .as_ref()
        .map(|summary| ransac_crossing::refined_summary(py, summary))
        .transpose()?;
    out.set_item("essential", essential)?;
    let homography = result
        .homography
        .as_ref()
        .map(|summary| ransac_crossing::summary(py, summary))
        .transpose()?;
    out.set_item("homography", homography)?;
    Ok(out)
}

/// The enum tables: status, model, and RANSAC and NLS termination names in code order.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// The tables.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
#[pyfunction]
fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = ransac_crossing::refined_codes(py)?;
    out.set_item("status", enum_names(&Status::ALL, Status::name))?;
    out.set_item("model", enum_names(&Model::ALL, Model::name))?;
    Ok(out)
}

/// Registers the `twoview` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn twoview(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, initialize);
    Ok(())
}
