//! The `slamlab_rs` Python module: thin `PyO3` wrappers over the per-rung Rust crates.
//!
//! One file per bound module, mirroring the Python package tree (`src/foundations/so3.rs`
//! binds `slamlab.foundations.so3` as `slamlab_rs.foundations.so3`), so no file grows with
//! every rung. This root only wires the rung modules. Bindings only convert types: float64
//! numpy arrays in and out, plain slices and arrays into the cores. The Python entry points
//! validate input before calling.

use pyo3::prelude::*;

mod common;
mod foundations;

/// Registers the rung submodules.
///
/// # Errors
///
/// Returns a Python error if a submodule cannot be added.
#[pymodule]
fn slamlab_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_wrapped(pyo3::wrap_pymodule!(foundations::foundations))?;
    Ok(())
}
