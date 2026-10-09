//! Bindings for `slam_it_all.edge.frames` (`slam_it_all_rs.edge.frames`).
//!
//! Keypoints cross as an (N, 2) float64 array and the depth map as an (H, W) float64 one.

use numpy::{PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::prelude::*;
use slam_it_all_edge::frames as core;

use super::edge_crossing::{rows, vector_array};
use crate::common::{add_functions, contiguous, unwrap};

/// Reads a depth map at each keypoint's nearest pixel; NaN where it holds none.
///
/// # Arguments
///
/// * `xy` - The keypoints' pixels, (N, 2).
/// * `depth_map` - The depths in metres, (H, W).
///
/// # Returns
///
/// A new (N,) float64 array.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` from the core.
#[pyfunction]
fn depth_at<'py>(
    py: Python<'py>,
    xy: Bound<'py, PyAny>,
    depth_map: PyReadonlyArray2<'py, f64>,
) -> PyResult<Bound<'py, PyAny>> {
    let pixels = rows::<f64, 2>(&xy)?;
    let map = core::DepthMap {
        rows: depth_map.shape()[0],
        cols: depth_map.shape()[1],
        values: contiguous(&depth_map)?.to_vec(),
    };
    Ok(vector_array(
        py,
        &unwrap(py, core::depth_at(&pixels, &map))?,
    ))
}

/// Registers the `frames` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn frames(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; depth_at);
    Ok(())
}
