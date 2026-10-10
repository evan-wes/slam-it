//! Bindings for `slam_it_all.edge.mapping` (`slam_it_all_rs.edge.mapping`).
//!
//! Each step copies the map in, changes the copy, and returns it as a dict; the mode crosses
//! by name, a tracked frame as `pose` and `points`. `process` also returns how each step went.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_all_edge::mapping as core;

use slam_it_all_edge::map::Map;
use slam_it_all_foundations::nls;

use super::edge_crossing::{
    camera_in, frame_in, map_dict, map_in, mode_in, options_in, rows_array, solves_list,
    tracked_in, vector, vector_array,
};
use crate::common::{add_functions, enum_codes, to_i64, unwrap};

/// Runs a step on a copy of the map.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `step` - The step, changing the copy.
///
/// # Returns
///
/// The new map, as a dict.
///
/// # Errors
///
/// `ValueError` for a malformed map; `InvalidInputError` from the core.
fn changed<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    step: impl FnOnce(&mut Map) -> slam_it_all_foundations::error::Result<()>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut out = map_in(py, m)?;
    unwrap(py, step(&mut out))?;
    map_dict(py, &out)
}

/// Decides whether a tracked frame becomes a keyframe.
///
/// # Arguments
///
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `depth` - The frame's keypoints' depths.
/// * `tracked` - Its pose and matches, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// Whether to insert a keyframe.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, depth, tracked, mode, options))]
fn need_keyframe<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    depth: Bound<'py, PyAny>,
    tracked: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<bool> {
    unwrap(
        py,
        core::need_keyframe(
            &map_in(py, m)?,
            &camera_in(camera)?,
            &vector::<f64>(&depth)?,
            &tracked_in(tracked)?,
            mode_in(mode)?,
            &options_in(options)?,
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
    /// Fuses the newest keyframe's points with the window's.
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
    fuse_points;
}

mode_steps! {
    /// Triangulates the newest keyframe's free keypoints against the window's.
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
    /// Culls the window's redundant keyframes.
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
    cull_keyframes;
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
///
/// # Returns
///
/// The new map, as a dict, how each step went (the summary's counts and `solves`), and `made`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, frame, tracked, mode, options))]
fn process<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    tracked: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let (camera, frame, tracked) = (camera_in(camera)?, frame_in(frame)?, tracked_in(tracked)?);
    let (mode, options) = (mode_in(mode)?, options_in(options)?);
    let mut mapped = map_in(py, m)?;
    let done = unwrap(
        py,
        core::process(&mut mapped, &camera, &frame, &tracked, mode, &options),
    )?;
    let summary = &done.summary;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &mapped)?)?;
    out.set_item("from_depth", to_i64(summary.from_depth))?;
    out.set_item("culled_points", to_i64(summary.culled_points))?;
    out.set_item("triangulated", to_i64(summary.triangulated))?;
    out.set_item("fused", to_i64(summary.fused))?;
    out.set_item("solves", solves_list(py, &summary.solves)?)?;
    out.set_item("outliers", to_i64(summary.outliers))?;
    out.set_item("weak_points", to_i64(summary.weak_points))?;
    out.set_item("culled_keyframes", to_i64(summary.culled_keyframes))?;
    out.set_item("made", vector_array(py, &done.made))?;
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
        fuse_points,
        cull_keyframes,
        process
    );
    Ok(())
}
