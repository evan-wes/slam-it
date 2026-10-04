//! Bindings for rung 0, foundations (`slamlab_rs.foundations`).

use pyo3::prelude::*;

mod alignment;
mod camera;
mod se3;
mod so3;

/// Registers the foundations submodules.
///
/// # Errors
///
/// Returns a Python error if a submodule cannot be added.
#[pymodule]
pub fn foundations(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_wrapped(pyo3::wrap_pymodule!(alignment::alignment))?;
    m.add_wrapped(pyo3::wrap_pymodule!(camera::camera))?;
    m.add_wrapped(pyo3::wrap_pymodule!(se3::se3))?;
    m.add_wrapped(pyo3::wrap_pymodule!(so3::so3))?;
    Ok(())
}
