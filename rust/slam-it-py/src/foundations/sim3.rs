//! Bindings for `slam_it.foundations.sim3` (`slam_it_rs.foundations.sim3`).

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_foundations::camera::Model;
use slam_it_foundations::sim3::{self as core, Lens};

use super::ransac_crossing::{self, codes};
use crate::common::{ArrayIn, add_functions, contiguous, decode, rows, unwrap};

/// Estimates `S_12` from two cameras' matched points robustly (`sim3.sim3_ransac`).
///
/// # Arguments
///
/// * `points1`, `points2` - (N, 3) points in camera 1's frame and in camera 2's.
/// * `sigma2_1`, `sigma2_2` - Each match's variance in image 1 and in image 2.
/// * `model` - The camera model's code.
/// * `intrinsics` - Its intrinsics.
/// * `fixed_scale` - Hold the scale at 1.
/// * `options` - `ransac.Options`' fields.
///
/// # Returns
///
/// The solution as a dict.
///
/// # Errors
///
/// `ValueError` for malformed arguments or an unknown model code; `InvalidInputError` or
/// `DegenerateError` from the core.
#[pyfunction]
#[pyo3(signature = (
    *, points1, points2, sigma2_1, sigma2_2, model, intrinsics, fixed_scale, options
))]
#[allow(
    clippy::too_many_arguments,
    reason = "every argument crosses by keyword, by name"
)]
fn sim3_ransac<'py>(
    py: Python<'py>,
    points1: ArrayIn<'py>,
    points2: ArrayIn<'py>,
    sigma2_1: PyReadonlyArray1<'py, f64>,
    sigma2_2: PyReadonlyArray1<'py, f64>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    fixed_scale: bool,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let lens = Lens {
        model: decode(&Model::ALL, model, "model")?,
        intrinsics: contiguous(&intrinsics)?.to_vec(),
    };
    let found = core::sim3_ransac(
        &rows(&points1)?,
        &rows(&points2)?,
        (contiguous(&sigma2_1)?, contiguous(&sigma2_2)?),
        &lens,
        fixed_scale,
        &ransac_crossing::read_options(&options)?,
    );
    ransac_crossing::ransac_result(py, &unwrap(py, found)?)
}

/// Registers the `sim3` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn sim3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, sim3_ransac);
    Ok(())
}
