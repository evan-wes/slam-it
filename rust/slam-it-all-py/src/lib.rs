//! The `slam_it_all_rs` Python module: thin `PyO3` wrappers over the per-rung Rust crates.
//!
//! One file per bound module, mirroring the Python package tree (`src/foundations/so3.rs`
//! binds `slam_it_all.foundations.so3` as `slam_it_all_rs.foundations.so3`), so no file grows with
//! every rung. This root only wires the rung modules. Bindings only convert types: float64
//! numpy arrays (uint8 for images) in and out, plain slices and arrays into the cores. The
//! Python entry points validate input before calling.

use pyo3::prelude::*;

mod common;
mod edge;
mod foundations;

/// Registers the rung submodules.
///
/// # Errors
///
/// Returns a Python error if a submodule cannot be added.
#[pymodule]
fn slam_it_all_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_wrapped(pyo3::wrap_pymodule!(foundations::foundations))?;
    m.add_wrapped(pyo3::wrap_pymodule!(edge::edge))?;
    Ok(())
}
