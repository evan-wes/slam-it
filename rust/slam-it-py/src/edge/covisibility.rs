//! Bindings for `slam_it.edge.covisibility` (`slam_it_rs.edge.covisibility`).
//!
//! A map crosses as `Map.as_dict()`'s dict, the settings as `Options.as_dict()`'s. Links return
//! as a pair of int64 arrays (rows, weights); the local map as a dict of `keyframes` and
//! `reference`; the strong links as an (L, 3) int64 array; a changed map as its dict.

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use slam_it_edge::covisibility as core;

use super::edge_crossing::{indices_array, map_dict, map_in, options_in, rows_array};
use crate::common::{add_functions, contiguous, to_i64, unwrap};

/// A keyframe's links.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The linked rows and their weights.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn links<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    k: usize,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyTuple>> {
    let found = unwrap(py, core::links(&map_in(py, m)?, k, &options_in(options)?))?;
    let rows: Vec<usize> = found.iter().map(|link| link.row).collect();
    let weights: Vec<usize> = found.iter().map(|link| link.weight).collect();
    PyTuple::new(py, [indices_array(py, &rows), indices_array(py, &weights)])
}

/// Tracking's local map.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `points` - The point each keypoint was matched to, -1 for none.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `keyframes` (int64) and `reference`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn local_map<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    points: PyReadonlyArray1<'py, i64>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = unwrap(
        py,
        core::local_map(&map_in(py, m)?, contiguous(&points)?, &options_in(options)?),
    )?;
    let out = PyDict::new(py);
    out.set_item("keyframes", indices_array(py, &found.keyframes))?;
    out.set_item("reference", found.reference.map_or(-1, to_i64))?;
    Ok(out)
}

/// Gives a keyframe its parent.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// The map, as a dict.
///
/// # Errors
///
/// `InvalidInputError` from the core.
#[pyfunction]
fn adopt<'py>(py: Python<'py>, m: &Bound<'py, PyDict>, k: usize) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, core::adopt(&mut out, k))?;
    map_dict(py, &out)
}

/// Gives a keyframe's children new parents.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The map, as a dict.
///
/// # Errors
///
/// `InvalidInputError` from the core.
#[pyfunction]
fn reparent<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    k: usize,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, core::reparent(&mut out, k, &options_in(options)?))?;
    map_dict(py, &out)
}

/// The strong links.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// An (L, 3) int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed argument.
#[pyfunction]
fn strong_links<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    rows_array(
        py,
        &core::strong_links(&map_in(py, m)?, &options_in(options)?),
    )
}

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn covisibility(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; links, local_map, adopt, reparent, strong_links);
    Ok(())
}
