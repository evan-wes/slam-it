//! The `slamlab_rs` Python module: thin `PyO3` wrappers over `slamlab-core`.
//!
//! Each wrapper only converts types; the algorithms live in `slamlab-core`, which builds
//! without Python.

use pyo3::prelude::*;

/// Maps a 3-vector to its skew-symmetric matrix, returned to Python as nested lists.
///
/// # Arguments
///
/// * `v` - The vector to map. `PyO3` rejects anything that is not a sequence of exactly
///   three floats with a Python `TypeError`.
///
/// # Returns
///
/// The skew-symmetric matrix `[v]x`, row-major.
#[pyfunction]
fn hat(v: [f64; 3]) -> [[f64; 3]; 3] {
    slamlab_core::so3::hat(v)
}

/// Registers the module's functions with Python.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added to the module.
#[pymodule]
fn slamlab_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hat, m)?)?;
    Ok(())
}
