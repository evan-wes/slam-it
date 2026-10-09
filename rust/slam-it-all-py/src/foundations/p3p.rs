//! Bindings for `slam_it_all.foundations.p3p` (`slam_it_all_rs.foundations.p3p`).

use pyo3::prelude::*;
use slam_it_all_foundations::p3p::{self as core, Solver};

use crate::common::{
    ArrayIn, ArrayOut, add_functions, decode, enum_codes, items_array, rows, unwrap,
};

/// Every camera pose three world points and their bearings allow.
///
/// # Arguments
///
/// * `points` - (3, 3) world points.
/// * `bearings` - (3, 3) unit bearings.
/// * `solver` - The solver's code (index into `p3p.SOLVERS`).
///
/// # Returns
///
/// A new (K, 4, 4) array of poses `T_cam_world`.
///
/// # Errors
///
/// `ValueError` for malformed arrays or an unknown code; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(name = "p3p")]
fn solve_p3p<'py>(
    py: Python<'py>,
    points: ArrayIn<'py>,
    bearings: ArrayIn<'py>,
    solver: i64,
) -> PyResult<ArrayOut<'py>> {
    let solver = decode(&Solver::ALL, solver, "solver")?;
    let found = core::p3p(&rows(&points)?, &rows(&bearings)?, solver);
    items_array(py, &unwrap(py, found)?)
}

enum_codes!("solver" => Solver);

/// Registers the `p3p` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn p3p(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, solve_p3p);
    Ok(())
}
