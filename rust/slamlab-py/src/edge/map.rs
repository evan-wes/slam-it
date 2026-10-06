//! Bindings for `slamlab.edge.map` (`slamlab_rs.edge.map`).
//!
//! The map crosses as a dict of `Map`'s fields both ways (`edge_crossing`); an operation
//! copies it in, changes the copy, and returns it.

use nalgebra::Matrix4;
use numpy::{PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slamlab_edge::map as core;

use super::edge_crossing::{
    frame_in, indices, indices_array, map_dict, map_in, rows, vector, vector_array, vectors,
};
use crate::common::{add_functions, contiguous, to_i64, unwrap};

/// Reads a 4x4 pose.
///
/// # Arguments
///
/// * `pose` - The (4, 4) array.
///
/// # Returns
///
/// The pose.
///
/// # Errors
///
/// `ValueError` for another shape.
fn pose_in(pose: &PyReadonlyArray2<'_, f64>) -> PyResult<Matrix4<f64>> {
    if pose.shape() != [4, 4] {
        return Err(PyValueError::new_err("pose: expected shape (4, 4)"));
    }
    Ok(Matrix4::from_row_slice(contiguous(pose)?))
}

/// Predicts points' levels from their distances.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `rows_in` - The points' rows.
/// * `distance` - Their distances.
///
/// # Returns
///
/// A new int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, rows, distance))]
fn predict_level<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    rows: Bound<'py, PyAny>,
    distance: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let levels = unwrap(
        py,
        core::predict_level(
            &map_in(py, m)?,
            &indices(&rows)?,
            &vector::<f64>(&distance)?,
        ),
    )?;
    Ok(indices_array(py, &levels))
}

/// Adds a keyframe.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `frame` - The frame, as a dict.
/// * `pose` - Its `T_cam_world`.
/// * `points` - The point id each keypoint observes, or -1.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn insert_keyframe<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    pose: PyReadonlyArray2<'py, f64>,
    points: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(
        py,
        core::insert_keyframe(
            &mut out,
            &frame_in(frame)?,
            &pose_in(&pose)?,
            &vector::<i64>(&points)?,
        ),
    )?;
    map_dict(py, &out)
}

/// Creates map points.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `xyz` - The points, (Q, 3).
/// * `observers` - Each point's observing keypoint rows, (Q, V), -1 padded.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn add_points<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    xyz: Bound<'py, PyAny>,
    observers: PyReadonlyArray2<'py, i64>,
) -> PyResult<Bound<'py, PyDict>> {
    let points = vectors(&xyz)?;
    let width = observers.shape()[1];
    if observers.shape()[0] != points.len() {
        return Err(PyValueError::new_err(
            "observers: expected one row per point",
        ));
    }
    let flat = contiguous(&observers)?;
    let rows_of: Vec<Vec<usize>> = (0..points.len())
        .map(|i| {
            flat[i * width..(i + 1) * width]
                .iter()
                .filter(|&&r| r >= 0)
                .map(|&r| usize::try_from(r).map_err(|e| PyValueError::new_err(e.to_string())))
                .collect::<PyResult<Vec<usize>>>()
        })
        .collect::<PyResult<_>>()?;
    let mut out = map_in(py, m)?;
    unwrap(py, core::add_points(&mut out, &points, &rows_of))?;
    map_dict(py, &out)
}

/// Picks the most central descriptor.
///
/// # Arguments
///
/// * `descriptors` - The descriptors, (n, 32) uint8.
///
/// # Returns
///
/// Its index.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` for none.
#[pyfunction]
fn distinctive_descriptor(py: Python<'_>, descriptors: Bound<'_, PyAny>) -> PyResult<i64> {
    let rows_in = rows::<u8, 32>(&descriptors)?;
    Ok(to_i64(unwrap(py, core::distinctive_descriptor(&rows_in))?))
}

/// Recomputes points' descriptors, normals and distances.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `rows_in` - The points' rows.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, rows))]
fn refresh_points<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    rows: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, core::refresh_points(&mut out, &indices(&rows)?))?;
    map_dict(py, &out)
}

/// Removes points.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `erase` - One boolean per point.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn erase_points<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    erase: PyReadonlyArray1<'py, bool>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, core::erase_points(&mut out, contiguous(&erase)?))?;
    map_dict(py, &out)
}

/// Weighs points' observations.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `rows_in` - The points' rows.
///
/// # Returns
///
/// A new int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, rows))]
fn observation_weight<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    rows: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let weights = unwrap(
        py,
        core::observation_weight(&map_in(py, m)?, &indices(&rows)?),
    )?;
    Ok(vector_array(py, &weights))
}

/// Removes keypoints' observations.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `rows_in` - Keypoint rows.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, rows))]
fn forget_observations<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    rows: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, core::forget_observations(&mut out, &indices(&rows)?))?;
    map_dict(py, &out)
}

/// Removes a keyframe.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `k` - The keyframe's row.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `InvalidInputError` from the core.
#[pyfunction]
fn erase_keyframe<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    k: usize,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, core::erase_keyframe(&mut out, k))?;
    map_dict(py, &out)
}

/// Adds to points' visible and found counts.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `ids` - Point ids.
/// * `visible` - Each id's visible increment.
/// * `found` - Each id's found increment.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn count<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    ids: Bound<'py, PyAny>,
    visible: Bound<'py, PyAny>,
    found: Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(
        py,
        core::count(
            &mut out,
            &vector::<i64>(&ids)?,
            &vector::<i64>(&visible)?,
            &vector::<i64>(&found)?,
        ),
    )?;
    map_dict(py, &out)
}

/// Registers the `map` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn map(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; predict_level, insert_keyframe, add_points, distinctive_descriptor,
        refresh_points, erase_points, observation_weight, forget_observations, erase_keyframe,
        count);
    Ok(())
}
