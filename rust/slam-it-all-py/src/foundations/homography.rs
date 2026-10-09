//! Bindings for `slam_it_all.foundations.homography` (`slam_it_all_rs.foundations.homography`).

use nalgebra::{Matrix3, Vector2};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_all_foundations::homography as core;

use super::ransac_crossing;
use crate::common::{
    ArrayIn, ArrayOut, add_functions, item_array, items_array, one, rows, unwrap, vector_array,
};

/// H from (N, 2) point pairs.
///
/// # Arguments
///
/// * `points1`, `points2` - (N, 2) points in images 1 and 2.
///
/// # Returns
///
/// A new (3, 3) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn homography_dlt<'py>(
    py: Python<'py>,
    points1: ArrayIn<'py>,
    points2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let found = core::homography_dlt(&rows(&points1)?, &rows(&points2)?);
    item_array(py, &unwrap(py, found)?)
}

/// Squared Sampson errors under H.
///
/// # Arguments
///
/// * `homography` - (3, 3) H.
/// * `points1`, `points2` - (N, 2) points in images 1 and 2.
///
/// # Returns
///
/// A new (N,) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays; `InvalidInputError` from the core.
#[pyfunction]
fn sampson_homography<'py>(
    py: Python<'py>,
    homography: ArrayIn<'py>,
    points1: ArrayIn<'py>,
    points2: ArrayIn<'py>,
) -> PyResult<ArrayOut<'py>> {
    let errors = core::sampson_homography(
        &one::<Matrix3<f64>>(&homography)?,
        &rows::<Vector2<f64>>(&points1)?,
        &rows::<Vector2<f64>>(&points2)?,
    );
    vector_array(py, &unwrap(py, errors)?)
}

/// The motions a calibrated H allows.
///
/// # Arguments
///
/// * `homography` - (3, 3) H.
///
/// # Returns
///
/// `{"poses": (K, 4, 4), "normals": (K, 3)}`, K = 4, or 1 for a pure rotation.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` or `DegenerateError` from the core.
#[pyfunction]
fn decompose_homography<'py>(
    py: Python<'py>,
    homography: ArrayIn<'py>,
) -> PyResult<Bound<'py, PyDict>> {
    let motions = core::decompose_homography(&one::<Matrix3<f64>>(&homography)?);
    let motions = unwrap(py, motions)?;
    let out = PyDict::new(py);
    out.set_item("poses", items_array(py, &motions.poses)?)?;
    out.set_item("normals", items_array(py, &motions.normals)?)?;
    Ok(out)
}

/// LO-MSAC with the normalised DLT.
///
/// # Arguments
///
/// * `points1`, `points2` - (N, 2) points in images 1 and 2.
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
fn homography_ransac<'py>(
    py: Python<'py>,
    points1: ArrayIn<'py>,
    points2: ArrayIn<'py>,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = core::homography_ransac(
        &rows(&points1)?,
        &rows(&points2)?,
        &ransac_crossing::read_options(&options)?,
    );
    ransac_crossing::ransac_result(py, &unwrap(py, found)?)
}

/// The enum table: RANSAC's terminations.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"ransac_termination": [...]}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
#[pyfunction]
fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    ransac_crossing::codes(py)
}

/// Registers the `homography` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn homography(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(
        m;
        codes,
        homography_dlt,
        sampson_homography,
        decompose_homography,
        homography_ransac
    );
    Ok(())
}
