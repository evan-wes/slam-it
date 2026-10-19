//! Bindings for `slam_it.edge.adjust` (`slam_it_rs.edge.adjust`).
//!
//! An adjustment crosses back as a dict of `map`, `solves` and `outliers`.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_edge::adjust as core;

use slam_it_foundations::nls::{self, Stop};

use super::edge_crossing::{camera_in, indices, map_dict, map_in, options_in, solves_list, vector};
use crate::common::{add_functions, enum_codes, to_i64, unwrap};

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
/// The new map, as a dict, the solves and the outliers dropped.
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
    let mut adjusted = map_in(py, m)?;
    let adjustment = unwrap(
        py,
        core::bundle_adjust(
            &mut adjusted,
            &camera_in(camera)?,
            &indices(&keyframes)?,
            &vector::<bool>(&fixed)?,
            &options_in(options)?,
            &Stop::never(),
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &adjusted)?)?;
    out.set_item("solves", solves_list(py, &adjustment.solves)?)?;
    out.set_item("outliers", to_i64(adjustment.outliers))?;
    out.set_item("free", to_i64(adjustment.free))?;
    out.set_item("fixed", to_i64(adjustment.fixed))?;
    out.set_item("aborted", adjustment.aborted)?;
    Ok(out)
}

enum_codes!("nls_termination" => nls::Termination);

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn adjust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, bundle_adjust);
    Ok(())
}
