//! Bindings for `slamlab.foundations.fivepoint` (`slamlab_rs.foundations.fivepoint`).

use nalgebra::Vector3;
use pyo3::prelude::*;
use slamlab_foundations::fivepoint::{self as core, Solver};

use crate::common::{
    ArrayIn, ArrayOut, add_functions, decode, enum_codes, items_array, rows, unwrap,
};

/// Every essential matrix five bearing pairs allow.
///
/// # Arguments
///
/// * `bearings1` - (5, 3) bearings in camera 1.
/// * `bearings2` - (5, 3) bearings in camera 2.
/// * `solver` - The solver's code (index into `fivepoint.SOLVERS`).
///
/// # Returns
///
/// A new (K, 3, 3) array.
///
/// # Errors
///
/// `ValueError` for malformed arrays or an unknown code; `InvalidInputError` from the core.
#[pyfunction]
#[allow(
    clippy::needless_pass_by_value,
    reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
)]
fn essential_5pt<'py>(
    py: Python<'py>,
    bearings1: ArrayIn<'py>,
    bearings2: ArrayIn<'py>,
    solver: i64,
) -> PyResult<ArrayOut<'py>> {
    let solver = decode(&Solver::ALL, solver, "solver")?;
    let found = core::essential_5pt(
        &rows::<Vector3<f64>>(&bearings1)?,
        &rows::<Vector3<f64>>(&bearings2)?,
        solver,
    );
    items_array(py, &unwrap(py, found)?)
}

enum_codes!("solver" => Solver);

/// Registers the `fivepoint` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn fivepoint(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, essential_5pt);
    Ok(())
}
