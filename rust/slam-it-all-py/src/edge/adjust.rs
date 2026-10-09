//! Bindings for `slam_it_all.edge.adjust` (`slam_it_all_rs.edge.adjust`).

use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_all_edge::adjust as core;

use super::edge_crossing::{camera_in, indices, map_dict, map_in, options_in, vector};
use crate::common::{add_functions, unwrap};

/// Bundle-adjusts keyframes and the points they observe.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `keyframes` - The keyframes' rows.
/// * `fixed` - One flag per keyframe.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, keyframes, fixed, options))]
fn bundle_adjust<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    keyframes: Bound<'py, PyAny>,
    fixed: Bound<'py, PyAny>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(
        py,
        core::bundle_adjust(
            &mut out,
            &camera_in(camera)?,
            &indices(&keyframes)?,
            &vector::<bool>(&fixed)?,
            &options_in(options)?,
        ),
    )?;
    map_dict(py, &out)
}

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn adjust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; bundle_adjust);
    Ok(())
}
