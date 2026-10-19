//! Bindings for `slam_it.edge.initialization` (`slam_it_rs.edge.initialization`).
//!
//! A started map crosses back as a dict of `map`, `pose` and `points`, or None.

use numpy::IntoPyArray;
use numpy::ndarray::Array2;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_edge::initialization as core;
use slam_it_foundations::nls;

use super::edge_crossing::{
    camera_in, frame_in, initialization_dict, options_in, rows, vector_array,
};
use crate::common::{add_functions, enum_codes, to_i64, unwrap};

/// Starts the map from a stereo or RGB-D frame.
///
/// # Arguments
///
/// * `frame` - The frame, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The attempt, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (frame, options))]
fn initialize_from_depth<'py>(
    py: Python<'py>,
    frame: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let attempt = unwrap(
        py,
        core::initialize_from_depth(&frame_in(frame)?, &options_in(options)?),
    )?;
    initialization_dict(py, attempt)
}

/// Matches a monocular reference frame to a later one.
///
/// # Arguments
///
/// * `reference` - The reference frame, as a dict.
/// * `frame` - The later frame, as a dict.
/// * `previous` - Where each reference keypoint was last matched, (N, 2).
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `pairs` (M, 2) and `distances` (M,), int64.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (reference, frame, previous, options))]
fn match_for_initialization<'py>(
    py: Python<'py>,
    reference: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    previous: &Bound<'py, PyAny>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = unwrap(
        py,
        core::match_for_initialization(
            &frame_in(reference)?,
            &frame_in(frame)?,
            &rows::<f64, 2>(previous)?,
            &options_in(options)?,
        ),
    )?;
    let pairs: Vec<i64> = found
        .iter()
        .flat_map(|m| [to_i64(m.first), to_i64(m.second)])
        .collect();
    let distances: Vec<i64> = found.iter().map(|m| m.distance).collect();
    let pairs = Array2::from_shape_vec((found.len(), 2), pairs)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let out = PyDict::new(py);
    out.set_item("pairs", pairs.into_pyarray(py))?;
    out.set_item("distances", vector_array(py, &distances))?;
    Ok(out)
}

/// Starts the map from two monocular frames.
///
/// # Arguments
///
/// * `camera` - The camera, as a dict.
/// * `reference` - The reference frame, as a dict.
/// * `frame` - The later frame, as a dict.
/// * `pairs` - The matches, (M, 2) int64.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The attempt, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument or a negative keypoint; `InvalidInputError` from the
/// core.
#[pyfunction]
#[pyo3(signature = (camera, reference, frame, pairs, options))]
fn initialize_monocular<'py>(
    py: Python<'py>,
    camera: &Bound<'py, PyDict>,
    reference: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    pairs: &Bound<'py, PyAny>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let matches = rows::<i64, 2>(pairs)?
        .into_iter()
        .map(|[a, b]| {
            let index = |v: i64| {
                usize::try_from(v).map_err(|e| PyValueError::new_err(format!("pairs: {e}")))
            };
            Ok([index(a)?, index(b)?])
        })
        .collect::<PyResult<Vec<[usize; 2]>>>()?;
    let attempt = unwrap(
        py,
        core::initialize_monocular(
            &camera_in(camera)?,
            &frame_in(reference)?,
            &frame_in(frame)?,
            &matches,
            &options_in(options)?,
        ),
    )?;
    initialization_dict(py, attempt)
}

enum_codes!("outcome" => core::Outcome, "nls_termination" => nls::Termination);

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn initialization(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(
        m;
        codes,
        initialize_from_depth,
        match_for_initialization,
        initialize_monocular
    );
    Ok(())
}
