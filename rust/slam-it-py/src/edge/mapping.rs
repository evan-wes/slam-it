//! Bindings for `slam_it.edge.mapping` (`slam_it_rs.edge.mapping`).
//!
//! Each step copies the map in, changes the copy, and returns it as a dict; the mode crosses
//! by name, a tracked frame as `pose` and `points`. `process` also returns how each step went.

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_edge::mapping as core;

use slam_it_foundations::nls;

use super::edge_crossing::{
    camera_in, changed, frame_in, indices_array, map_dict, map_in, mode_in, options_in, rows_array,
    solves_list, tracked_in, vector, vector_array,
};
use crate::common::{add_functions, contiguous, enum_codes, to_i64, unwrap};

/// Decides whether a tracked frame becomes a keyframe.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `depth` - The frame's keypoints' depths.
/// * `tracked` - Its pose and matches, as a dict.
/// * `reference` - Its reference keyframe's row.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
/// * `idle` - Whether local mapping is idle.
/// * `since_keyframe` - The frames since the last keyframe, or `None`.
/// * `keyframes` - The whole map's keyframes when `m` is a view of it, or `None`.
///
/// # Returns
///
/// Whether to insert a keyframe.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, depth, tracked, reference, mode, options, idle, since_keyframe, keyframes))]
#[allow(
    clippy::too_many_arguments,
    reason = "the stage's state, as the reference takes it"
)]
fn need_keyframe<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    depth: Bound<'py, PyAny>,
    tracked: &Bound<'py, PyDict>,
    reference: usize,
    mode: &str,
    options: &Bound<'py, PyDict>,
    idle: bool,
    since_keyframe: Option<usize>,
    keyframes: Option<usize>,
) -> PyResult<bool> {
    unwrap(
        py,
        core::need_keyframe(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &vector::<f64>(&depth)?,
            &tracked_in(tracked)?,
            reference,
            mode_in(mode)?,
            &options_in(options)?,
            idle,
            since_keyframe,
            keyframes,
        ),
    )
}

/// Binds map steps that take the camera and the settings: each runs on a copy of the map
/// (`changed`) and returns it.
macro_rules! camera_steps {
    ($($(#[$meta:meta])* $name:ident;)+) => {$(
        $(#[$meta])*
        #[pyfunction]
        #[pyo3(signature = (m, camera, options))]
        fn $name<'py>(
            py: Python<'py>,
            m: &Bound<'py, PyDict>,
            camera: &Bound<'py, PyDict>,
            options: &Bound<'py, PyDict>,
        ) -> PyResult<Bound<'py, PyDict>> {
            let (camera, options) = (camera_in(camera)?, options_in(options)?);
            changed(py, m, |out| core::$name(out, &camera, &options))
        }
    )+};
}

/// Binds map steps that take the camera, the mode and the settings, likewise.
macro_rules! mode_steps {
    ($($(#[$meta:meta])* $name:ident;)+) => {$(
        $(#[$meta])*
        #[pyfunction]
        #[pyo3(signature = (m, camera, mode, options))]
        fn $name<'py>(
            py: Python<'py>,
            m: &Bound<'py, PyDict>,
            camera: &Bound<'py, PyDict>,
            mode: &str,
            options: &Bound<'py, PyDict>,
        ) -> PyResult<Bound<'py, PyDict>> {
            let (camera, mode) = (camera_in(camera)?, mode_in(mode)?);
            let options = options_in(options)?;
            changed(py, m, |out| core::$name(out, &camera, mode, &options))
        }
    )+};
}

camera_steps! {
    /// Creates points from the newest keyframe's close depth.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, as a dict.
    /// * `camera` - The camera, as a dict.
    /// * `options` - The settings, as a dict.
    ///
    /// # Returns
    ///
    /// The new map, as a dict.
    ///
    /// # Errors
    ///
    /// `ValueError` for a malformed argument; `InvalidInputError` from the core.
    create_points_from_depth;
}

mode_steps! {
    /// Fuses the newest keyframe's points with its neighbours'.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, as a dict.
    /// * `camera` - The camera, as a dict.
    /// * `mode` - The mode's name.
    /// * `options` - The settings, as a dict.
    ///
    /// # Returns
    ///
    /// The new map, as a dict.
    ///
    /// # Errors
    ///
    /// `ValueError` for a malformed argument; `InvalidInputError` from the core.
    fuse_points;
    /// Triangulates the newest keyframe's free keypoints against its best links'.
    ///
    /// # Arguments
    ///
    /// * `m` - The map, as a dict.
    /// * `camera` - The camera, as a dict.
    /// * `mode` - The mode's name.
    /// * `options` - The settings, as a dict.
    ///
    /// # Returns
    ///
    /// The new map, as a dict.
    ///
    /// # Errors
    ///
    /// `ValueError` for a malformed argument; `InvalidInputError` from the core.
    triangulate_new_points;
}

/// Culls the newest keyframe's redundant links, sparing those loop detection holds.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
/// * `held` - The ids of the keyframes loop detection holds.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, mode, options, held))]
fn cull_keyframes<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
    held: PyReadonlyArray1<'py, i64>,
) -> PyResult<Bound<'py, PyDict>> {
    let (camera, mode) = (camera_in(camera)?, mode_in(mode)?);
    let options = options_in(options)?;
    let held = contiguous(&held)?;
    changed(py, m, |out| {
        core::cull_keyframes(out, &camera, mode, &options, held)
    })
}

/// Culls recently created points tracking rarely finds.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, mode, options))]
fn cull_points<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let (mode, options) = (mode_in(mode)?, options_in(options)?);
    changed(py, m, |out| core::cull_points(out, mode, &options))
}

/// The keyframes the newest keyframe's points are fused into.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// Their ids (int64).
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn fuse_targets<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let (mode, options) = (mode_in(mode)?, options_in(options)?);
    let ids = unwrap(py, core::fuse_targets(&map_in(py, m)?, mode, &options))?;
    Ok(vector_array(py, &ids))
}

/// The keyframes local BA adjusts.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// Their rows (int64).
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
fn local_ba_keyframes<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let options = options_in(options)?;
    let rows = unwrap(py, core::local_ba_keyframes(&map_in(py, m)?, &options))?;
    Ok(indices_array(py, &rows))
}

/// Matches two keyframes' free keypoints for triangulation.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `k1` - The first keyframe's row.
/// * `k2` - The second's.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// A new (Q, 2) int64 array of keypoint rows.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, k1, k2, options))]
fn match_for_triangulation<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    k1: usize,
    k2: usize,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let pairs = unwrap(
        py,
        core::match_for_triangulation(
            &map_in(py, m)?,
            &camera_in(camera)?,
            k1,
            k2,
            &options_in(options)?,
        ),
    )?;
    let rows: Vec<[i64; 2]> = pairs
        .iter()
        .map(|&[a, b]| [crate::common::to_i64(a), crate::common::to_i64(b)])
        .collect();
    rows_array(py, &rows)
}

/// Inserts a keyframe and runs local mapping on it.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `frame` - The keyframe's frame, as a dict.
/// * `tracked` - Its pose and matches, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
/// * `held` - The ids of the keyframes loop detection holds.
///
/// # Returns
///
/// The new map, as a dict, how each step went (the summary's counts and `solves`), `made`, and
/// how the steps were scheduled (`schedule`, by name).
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, frame, tracked, mode, options, held))]
#[allow(
    clippy::too_many_arguments,
    reason = "the stage's state, as the reference takes it"
)]
fn process<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    tracked: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
    held: PyReadonlyArray1<'py, i64>,
) -> PyResult<Bound<'py, PyDict>> {
    let (camera, frame, tracked) = (camera_in(camera)?, frame_in(frame)?, tracked_in(tracked)?);
    let (mode, options) = (mode_in(mode)?, options_in(options)?);
    let mut mapped = map_in(py, m)?;
    let done = unwrap(
        py,
        core::process(
            &mut mapped,
            &camera,
            &frame,
            &tracked,
            mode,
            &options,
            contiguous(&held)?,
            None,
        ),
    )?;
    let summary = &done.summary;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &mapped)?)?;
    out.set_item("from_depth", to_i64(summary.from_depth))?;
    out.set_item("culled_points", to_i64(summary.culled_points))?;
    out.set_item("triangulated", to_i64(summary.triangulated))?;
    out.set_item("fused", to_i64(summary.fused))?;
    out.set_item("solves", solves_list(py, &summary.solves)?)?;
    out.set_item("free", to_i64(summary.free))?;
    out.set_item("fixed", to_i64(summary.fixed))?;
    out.set_item("outliers", to_i64(summary.outliers))?;
    out.set_item("weak_points", to_i64(summary.weak_points))?;
    out.set_item("culled_keyframes", to_i64(summary.culled_keyframes))?;
    out.set_item("made", vector_array(py, &done.made))?;
    let schedule = PyDict::new(py);
    schedule.set_item("fusion", done.schedule.fusion.name())?;
    schedule.set_item("adjustment", done.schedule.adjustment.name())?;
    schedule.set_item("culling", done.schedule.culling.name())?;
    out.set_item("schedule", schedule)?;
    Ok(out)
}

enum_codes!("nls_termination" => nls::Termination);

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn mapping(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(
        m;
        codes,
        need_keyframe,
        create_points_from_depth,
        cull_points,
        match_for_triangulation,
        triangulate_new_points,
        fuse_targets,
        fuse_points,
        local_ba_keyframes,
        cull_keyframes,
        process
    );
    Ok(())
}
