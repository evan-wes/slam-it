//! The `slam_it_rs.edge` submodule: one module per bound edge module.
#![allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]

use pyo3::prelude::*;

mod adjust;
mod edge_crossing;
mod frames;
mod initialization;
mod map;
mod mapping;
mod residuals;
mod tracking;

/// Registers the edge submodules.
///
/// # Errors
///
/// Returns a Python error if a submodule cannot be added.
#[pymodule]
pub fn edge(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_wrapped(pyo3::wrap_pymodule!(frames::frames))?;
    m.add_wrapped(pyo3::wrap_pymodule!(map::map))?;
    m.add_wrapped(pyo3::wrap_pymodule!(residuals::residuals))?;
    m.add_wrapped(pyo3::wrap_pymodule!(adjust::adjust))?;
    m.add_wrapped(pyo3::wrap_pymodule!(initialization::initialization))?;
    m.add_wrapped(pyo3::wrap_pymodule!(tracking::tracking))?;
    m.add_wrapped(pyo3::wrap_pymodule!(mapping::mapping))?;
    Ok(())
}
