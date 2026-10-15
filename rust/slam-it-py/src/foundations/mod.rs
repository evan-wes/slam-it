//! Bindings for rung 0, foundations (`slam_it_rs.foundations`).
//!
//! PyO3 extracts every `#[pyfunction]` argument by value, and an array's read-only borrow
//! guard *is* the argument, so clippy's `needless_pass_by_value` is allowed once here for all
//! the binding modules below, rather than on each function.

#![allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]

use pyo3::prelude::*;

mod alignment;
mod ba;
mod camera;
mod epipolar;
mod fivepoint;
mod homography;
mod image;
mod image_crossing;
mod matching;
mod nls_crossing;
mod orb;
mod orb_crossing;
mod p3p;
mod pnp;
mod ransac_crossing;
mod se3;
mod so3;
mod stereo;
mod triangulation;
mod twoview;
mod vocabulary;
pub(crate) mod vocabulary_crossing;

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
    m.add_wrapped(pyo3::wrap_pymodule!(image::image))?;
    m.add_wrapped(pyo3::wrap_pymodule!(matching::matching))?;
    m.add_wrapped(pyo3::wrap_pymodule!(orb::orb))?;
    m.add_wrapped(pyo3::wrap_pymodule!(p3p::p3p))?;
    m.add_wrapped(pyo3::wrap_pymodule!(pnp::pnp))?;
    m.add_wrapped(pyo3::wrap_pymodule!(se3::se3))?;
    m.add_wrapped(pyo3::wrap_pymodule!(so3::so3))?;
    m.add_wrapped(pyo3::wrap_pymodule!(stereo::stereo))?;
    m.add_wrapped(pyo3::wrap_pymodule!(triangulation::triangulation))?;
    m.add_wrapped(pyo3::wrap_pymodule!(twoview::twoview))?;
    m.add_wrapped(pyo3::wrap_pymodule!(vocabulary::vocabulary))?;
    Ok(())
}
