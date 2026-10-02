//! The `slamlab_rs` Python module: thin PyO3 wrappers over slamlab-core.

use pyo3::prelude::*;

/// Skew-symmetric matrix [v]x of a 3-vector, as nested lists.
#[pyfunction]
fn hat(v: [f64; 3]) -> [[f64; 3]; 3] {
    slamlab_core::so3::hat(v)
}

#[pymodule]
fn slamlab_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hat, m)?)?;
    Ok(())
}
