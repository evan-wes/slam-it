//! Bindings for `slam_it.edge.covisibility` (`slam_it_rs.edge.covisibility`).
//!
//! A map crosses as `Map.as_dict()`'s dict, the settings as `Options.as_dict()`'s. Links and the
//! connected keyframes return as a pair of int64 arrays (rows, weights); the local map as a dict
//! of `keyframes` and `reference`; the strong links as an (L, 3) int64 array; a changed map as
//! its dict.

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};
use slam_it_edge::covisibility as core;
use slam_it_edge::map::Map;
use slam_it_edge::options::Options;
use slam_it_foundations::error::Result;

use super::edge_crossing::{changed, indices_array, map_in, options_in, rows_array};
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
    links_out(
        py,
        &unwrap(py, core::links(&map_in(py, m)?, k, &options_in(options)?))?,
    )
}

/// A keyframe's connected keyframes.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// The rows of the keyframes sharing a point with it, and their weights.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn connected<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    k: usize,
) -> PyResult<Bound<'py, PyTuple>> {
    links_out(py, &unwrap(py, core::connected(&map_in(py, m)?, k))?)
}

/// Links as Python takes them.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `found` - The links.
///
/// # Returns
///
/// Their rows and their weights, as int64 arrays.
///
/// # Errors
///
/// Returns a Python error if the tuple cannot be made.
fn links_out<'py>(py: Python<'py>, found: &[core::Link]) -> PyResult<Bound<'py, PyTuple>> {
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

/// The keyframes tracking's next frame may read.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `last` - The point each of the last frame's keypoints tracked, -1 for none.
/// * `reference` - Its reference keyframe's id, or -1.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The keyframes' rows (int64), ascending.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn view_keyframes<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    last: PyReadonlyArray1<'py, i64>,
    reference: i64,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let rows = unwrap(
        py,
        core::view_keyframes(
            &map_in(py, m)?,
            contiguous(&last)?,
            reference,
            &options_in(options)?,
        ),
    )?;
    Ok(indices_array(py, &rows))
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
    changed(py, m, |out| core::adopt(out, k))
}

/// Runs a step on a keyframe, with the settings, on a copy of the map.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
/// * `options` - The settings, as a dict.
/// * `step` - The core's step.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for malformed settings; `InvalidInputError` from the core.
fn with_settings<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    k: usize,
    options: &Bound<'py, PyDict>,
    step: impl Fn(&mut Map, usize, &Options) -> Result<()>,
) -> PyResult<Bound<'py, PyDict>> {
    let options = options_in(options)?;
    changed(py, m, |out| step(out, k, &options))
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
    with_settings(py, m, k, options, core::reparent)
}

/// Culls a keyframe: its children given new parents, then it erased; or spares it.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
/// * `options` - The settings, as a dict.
/// * `held` - The ids of the keyframes loop detection holds.
///
/// # Returns
///
/// The map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn cull<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    k: usize,
    options: &Bound<'py, PyDict>,
    held: PyReadonlyArray1<'py, i64>,
) -> PyResult<Bound<'py, PyDict>> {
    let held = contiguous(&held)?;
    with_settings(py, m, k, options, |m, k, options| {
        core::cull(m, k, options, held)
    })
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
    add_functions!(m; links, connected, local_map, view_keyframes, adopt, reparent, cull,
        strong_links);
    Ok(())
}
