//! Bindings for `slamlab.edge.tracking` (`slamlab_rs.edge.tracking`).
//!
//! A tracked frame crosses as a dict of `pose` and `points` (or None where the stage may
//! fail); `InView` and `LocalTracking` as dicts of their fields, the latter's tracked frame
//! flattened into `pose` and `points`. The mode crosses by name.

use numpy::PyReadonlyArray2;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slamlab_edge::tracking as core;

use super::edge_crossing::{
    camera_in, frame_in, indices, indices_array, map_in, maybe_tracked, mode_in, options_in,
    pose_in, rows_array, tracked_dict, tracked_in, vector_array,
};
use crate::common::{add_functions, unwrap};

/// Refines a frame's pose against its matched points.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `frame` - The frame, as a dict.
/// * `tracked` - The starting pose and the matches, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The tracked frame, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, frame, tracked, options))]
fn optimize_pose<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    tracked: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let tracked = unwrap(
        py,
        core::optimize_pose(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &frame_in(frame)?,
            &tracked_in(tracked)?,
            &options_in(options)?,
        ),
    )?;
    tracked_dict(py, &tracked)
}

/// Selects the points a camera sees.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `pose` - Its pose.
/// * `rows` - The candidates' rows.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `rows`, `xy`, `right`, `level` and `view_cos`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, pose, rows, options))]
fn in_view<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    pose: PyReadonlyArray2<'py, f64>,
    rows: Bound<'py, PyAny>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let view = unwrap(
        py,
        core::in_view(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &pose_in(&pose)?,
            &indices(&rows)?,
            &options_in(options)?,
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("rows", indices_array(py, &view.rows))?;
    out.set_item("xy", rows_array(py, &view.xy)?)?;
    out.set_item("right", vector_array(py, &view.right))?;
    out.set_item("level", indices_array(py, &view.level))?;
    out.set_item("view_cos", vector_array(py, &view.view_cos))?;
    Ok(out)
}

/// Tracks the last frame's points into this one.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `frame` - The frame, as a dict.
/// * `last` - The last frame, as a dict.
/// * `last_tracked` - Its pose and matches, as a dict.
/// * `predicted` - The predicted pose.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The tracked frame as a dict, or None.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, frame, last, last_tracked, predicted, mode, options))]
#[allow(
    clippy::too_many_arguments,
    reason = "the stage's state, as the reference takes it"
)]
fn track_last_frame<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    last: &Bound<'py, PyDict>,
    last_tracked: &Bound<'py, PyDict>,
    predicted: PyReadonlyArray2<'py, f64>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let tracked = unwrap(
        py,
        core::track_last_frame(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &frame_in(frame)?,
            &frame_in(last)?,
            &tracked_in(last_tracked)?,
            &pose_in(&predicted)?,
            mode_in(mode)?,
            &options_in(options)?,
        ),
    )?;
    maybe_tracked(py, tracked)
}

/// Matches the local map's points in view and refines the pose.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `frame` - The frame, as a dict.
/// * `tracked` - Its pose and matches so far, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
/// * `recovering` - Whether the frame is a recovery's or the next.
///
/// # Returns
///
/// `pose`, `points`, `visible` and `found`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, frame, tracked, mode, options, recovering))]
#[allow(
    clippy::too_many_arguments,
    reason = "the stage's state, as the reference takes it"
)]
fn track_local_map<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    tracked: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
    recovering: bool,
) -> PyResult<Bound<'py, PyDict>> {
    let local = unwrap(
        py,
        core::track_local_map(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &frame_in(frame)?,
            &tracked_in(tracked)?,
            mode_in(mode)?,
            &options_in(options)?,
            recovering,
        ),
    )?;
    let out = tracked_dict(py, &local.tracked)?;
    out.set_item("visible", vector_array(py, &local.visible))?;
    out.set_item("found", vector_array(py, &local.found))?;
    Ok(out)
}

/// Places a frame against the local map by descriptors and PnP RANSAC.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `frame` - The frame, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The tracked frame as a dict, or None.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, frame, options))]
fn relocalize_locally<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let tracked = unwrap(
        py,
        core::relocalize_locally(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &frame_in(frame)?,
            &options_in(options)?,
        ),
    )?;
    maybe_tracked(py, tracked)
}

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn tracking(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; optimize_pose, in_view, track_last_frame, track_local_map, relocalize_locally);
    Ok(())
}
