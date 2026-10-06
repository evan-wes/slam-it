//! The `slamlab_rs.edge` submodule: one module per bound edge module.
#![allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]

use pyo3::prelude::*;

mod edge_crossing;
mod frames;
mod map;

/// Registers the edge submodules.
///
/// # Errors
///
/// Returns a Python error if a submodule cannot be added.
#[pymodule]
pub fn edge(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_wrapped(pyo3::wrap_pymodule!(frames::frames))?;
    m.add_wrapped(pyo3::wrap_pymodule!(map::map))?;
    Ok(())
}
