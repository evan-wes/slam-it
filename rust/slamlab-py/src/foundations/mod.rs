//! Bindings for rung 0, foundations (`slamlab_rs.foundations`).

use pyo3::prelude::*;

mod alignment;
mod ba;
mod camera;
mod epipolar;
mod fivepoint;
mod homography;
mod nls_crossing;
mod ransac_crossing;
mod se3;
mod so3;
mod triangulation;
mod twoview;

/// Registers the foundations submodules.
///
/// # Errors
///
/// Returns a Python error if a submodule cannot be added.
#[pymodule]
pub fn foundations(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_wrapped(pyo3::wrap_pymodule!(alignment::alignment))?;
    m.add_wrapped(pyo3::wrap_pymodule!(ba::ba))?;
    m.add_wrapped(pyo3::wrap_pymodule!(camera::camera))?;
    m.add_wrapped(pyo3::wrap_pymodule!(epipolar::epipolar))?;
    m.add_wrapped(pyo3::wrap_pymodule!(fivepoint::fivepoint))?;
    m.add_wrapped(pyo3::wrap_pymodule!(homography::homography))?;
    m.add_wrapped(pyo3::wrap_pymodule!(se3::se3))?;
    m.add_wrapped(pyo3::wrap_pymodule!(so3::so3))?;
    m.add_wrapped(pyo3::wrap_pymodule!(triangulation::triangulation))?;
    m.add_wrapped(pyo3::wrap_pymodule!(twoview::twoview))?;
    Ok(())
}
