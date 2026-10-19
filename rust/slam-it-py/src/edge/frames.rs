//! Bindings for `slam_it.edge.frames` (`slam_it_rs.edge.frames`).
//!
//! Keypoints cross as an (N, 2) float64 array, the depth map as an (H, W) float64 one and a
//! mask as an (H, W) uint8 one.

use numpy::{PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::prelude::*;
use slam_it_edge::frames as core;

use super::edge_crossing::{indices_array, rows, vector_array};
use crate::common::{add_functions, contiguous, unwrap};
use crate::foundations::image_crossing::image_in;

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

/// Returns the keypoints a mask keeps: those whose nearest pixel is not 0.
///
/// # Arguments
///
/// * `xy` - The keypoints' pixels, (N, 2).
/// * `mask` - The mask, (H, W) uint8: 0 where features are dropped.
///
/// # Returns
///
/// A new int64 array: the kept keypoints' indices, ascending.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` from the core.
#[pyfunction]
fn unmasked<'py>(
    py: Python<'py>,
    xy: Bound<'py, PyAny>,
    mask: PyReadonlyArray2<'py, u8>,
) -> PyResult<Bound<'py, PyAny>> {
    let pixels = rows::<f64, 2>(&xy)?;
    let kept = unwrap(py, core::unmasked(&pixels, &image_in(&mask)?))?;
    Ok(indices_array(py, &kept))
}

/// Registers the `frames` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn frames(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; depth_at, unmasked);
    Ok(())
}
