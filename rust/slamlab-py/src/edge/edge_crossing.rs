//! Shared by the edge rung's bindings: the map and a frame cross as dicts of arrays.
//!
//! A map is a dict of `slamlab.edge.map.Map`'s fields; a frame is the dict
//! `slamlab.edge.map.frame_dict` builds; the camera and the settings are the dicts of scalars
//! `Camera.as_dict` and `Options.as_dict` give. Arrays are C-contiguous, of the dtype and shape the
//! Python dataclasses hold; a malformed dict raises `ValueError`, and the map is then checked
//! by the core (`check_map`, an `InvalidInputError`).

use nalgebra::{Matrix4, Vector3};
use numpy::ndarray::{Array1, Array2, Array3};
use numpy::{
    Element, IntoPyArray, PyReadonlyArray1, PyReadonlyArray2, PyReadonlyArray3,
    PyUntypedArrayMethods,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slamlab_edge::frames::{Camera, Frame, Tracked};
use slamlab_edge::initialization::Started;
use slamlab_edge::map::{Map, check_map};
use slamlab_edge::options::{Mode, Options};
use slamlab_foundations::camera::Model;
use slamlab_foundations::orb::{DESCRIPTOR_BYTES, Descriptor, Features, Keypoint};

use crate::common::{by_name, contiguous, item_array, to_i64, unwrap};

/// The map's fields, as the dict's keys.
const MAP_KEYS: [&str; 26] = [
    "levels",
    "scale_factor",
    "kf_id",
    "kf_time",
    "kf_pose",
    "kf_start",
    "kp_xy",
    "kp_level",
    "kp_angle",
    "kp_descriptor",
    "kp_bearing",
    "kp_depth",
    "kp_right",
    "kp_point",
    "pt_id",
    "pt_xyz",
    "pt_descriptor",
    "pt_normal",
    "pt_min_distance",
    "pt_max_distance",
    "pt_visible",
    "pt_found",
    "pt_first_kf",
    "pt_reference_kf",
    "next_kf_id",
    "next_pt_id",
];

/// The frame's fields, as the dict's keys.
const FRAME_KEYS: [&str; 12] = [
    "timestamp",
    "xy",
    "level",
    "level_xy",
    "response",
    "angle",
    "descriptors",
    "scale_factor",
    "levels",
    "bearings",
    "depth",
    "right",
];

/// Reads one item of a dict, checking the dict holds exactly the expected keys.
///
/// # Arguments
///
/// * `values` - The dict.
/// * `keys` - Its expected keys.
/// * `key` - The item's key.
///
/// # Returns
///
/// The item.
///
/// # Errors
///
/// `ValueError` for a missing key, or a dict with other keys.
fn item<'py>(values: &Bound<'py, PyDict>, keys: &[&str], key: &str) -> PyResult<Bound<'py, PyAny>> {
    if values.len() != keys.len() {
        return Err(PyValueError::new_err(format!(
            "expected exactly the keys {keys:?}"
        )));
    }
    values
        .get_item(key)?
        .ok_or_else(|| PyValueError::new_err(format!("missing field `{key}`")))
}

/// Reads a 1D array item.
///
/// # Arguments
///
/// * `value` - The item.
///
/// # Returns
///
/// Its values.
///
/// # Errors
///
/// `ValueError` for another dtype or a non-contiguous array.
pub fn vector<T: Element + Copy>(value: &Bound<'_, PyAny>) -> PyResult<Vec<T>> {
    let array: PyReadonlyArray1<'_, T> = value.extract()?;
    Ok(contiguous(&array)?.to_vec())
}

/// Reads a 1D int64 array item as indices.
///
/// # Arguments
///
/// * `value` - The item.
///
/// # Returns
///
/// The indices.
///
/// # Errors
///
/// `ValueError` for a negative value or another dtype.
pub fn indices(value: &Bound<'_, PyAny>) -> PyResult<Vec<usize>> {
    vector::<i64>(value)?
        .into_iter()
        .map(|v| usize::try_from(v).map_err(|e| PyValueError::new_err(format!("index: {e}"))))
        .collect()
}

/// Reads an (N, C) array item as rows.
///
/// # Arguments
///
/// * `value` - The item.
///
/// # Returns
///
/// Its rows.
///
/// # Errors
///
/// `ValueError` for another dtype, column count or a non-contiguous array.
pub fn rows<T: Element + Copy + Default, const C: usize>(
    value: &Bound<'_, PyAny>,
) -> PyResult<Vec<[T; C]>> {
    let array: PyReadonlyArray2<'_, T> = value.extract()?;
    if array.shape()[1] != C {
        return Err(PyValueError::new_err(format!(
            "expected {C} columns, got {}",
            array.shape()[1]
        )));
    }
    Ok(contiguous(&array)?.as_chunks::<C>().0.to_vec())
}

/// Reads an (N, 3) float64 item as vectors.
///
/// # Arguments
///
/// * `value` - The item.
///
/// # Returns
///
/// The vectors.
///
/// # Errors
///
/// `ValueError` for another shape or dtype.
pub fn vectors(value: &Bound<'_, PyAny>) -> PyResult<Vec<Vector3<f64>>> {
    Ok(rows::<f64, 3>(value)?
        .into_iter()
        .map(Vector3::from)
        .collect())
}

/// Reads a (K, 4, 4) float64 item as poses.
///
/// # Arguments
///
/// * `value` - The item.
///
/// # Returns
///
/// The poses.
///
/// # Errors
///
/// `ValueError` for another shape or dtype.
fn poses(value: &Bound<'_, PyAny>) -> PyResult<Vec<Matrix4<f64>>> {
    let array: PyReadonlyArray3<'_, f64> = value.extract()?;
    if array.shape()[1..] != [4, 4] {
        return Err(PyValueError::new_err("expected (K, 4, 4) poses"));
    }
    Ok(contiguous(&array)?
        .as_chunks::<16>()
        .0
        .iter()
        .map(|p| Matrix4::from_row_slice(p))
        .collect())
}

/// Reads a map from a dict of its fields, checked.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `values` - The dict.
///
/// # Returns
///
/// The map.
///
/// # Errors
///
/// `ValueError` for a missing or unknown key or a malformed array; `InvalidInputError` for a
/// map that fails `check_map`.
pub fn map_in(py: Python<'_>, values: &Bound<'_, PyDict>) -> PyResult<Map> {
    let get = |key| item(values, &MAP_KEYS, key);
    let m = Map {
        levels: get("levels")?.extract()?,
        scale_factor: get("scale_factor")?.extract()?,
        kf_id: vector(&get("kf_id")?)?,
        kf_time: vector(&get("kf_time")?)?,
        kf_pose: poses(&get("kf_pose")?)?,
        kf_start: indices(&get("kf_start")?)?,
        kp_xy: rows::<f64, 2>(&get("kp_xy")?)?,
        kp_level: indices(&get("kp_level")?)?,
        kp_angle: indices(&get("kp_angle")?)?,
        kp_descriptor: rows::<u8, DESCRIPTOR_BYTES>(&get("kp_descriptor")?)?,
        kp_bearing: vectors(&get("kp_bearing")?)?,
        kp_depth: vector(&get("kp_depth")?)?,
        kp_right: vector(&get("kp_right")?)?,
        kp_point: vector(&get("kp_point")?)?,
        pt_id: vector(&get("pt_id")?)?,
        pt_xyz: vectors(&get("pt_xyz")?)?,
        pt_descriptor: rows::<u8, DESCRIPTOR_BYTES>(&get("pt_descriptor")?)?,
        pt_normal: vectors(&get("pt_normal")?)?,
        pt_min_distance: vector(&get("pt_min_distance")?)?,
        pt_max_distance: vector(&get("pt_max_distance")?)?,
        pt_visible: vector(&get("pt_visible")?)?,
        pt_found: vector(&get("pt_found")?)?,
        pt_first_kf: vector(&get("pt_first_kf")?)?,
        pt_reference_kf: vector(&get("pt_reference_kf")?)?,
        next_kf_id: get("next_kf_id")?.extract()?,
        next_pt_id: get("next_pt_id")?.extract()?,
    };
    unwrap(py, check_map(&m))?;
    Ok(m)
}

/// Wraps rows as a new (N, C) array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `values` - The rows.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the shape does not match (a binding bug).
pub fn rows_array<'py, T: Element + Copy, const C: usize>(
    py: Python<'py>,
    values: &[[T; C]],
) -> PyResult<Bound<'py, PyAny>> {
    let flat: Vec<T> = values.iter().flatten().copied().collect();
    let array = Array2::from_shape_vec((values.len(), C), flat)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py).into_any())
}

/// Wraps vectors as a new (N, 3) array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `values` - The vectors.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the shape does not match (a binding bug).
pub fn vectors_array<'py>(py: Python<'py>, values: &[Vector3<f64>]) -> PyResult<Bound<'py, PyAny>> {
    let rows: Vec<[f64; 3]> = values.iter().map(|v| [v.x, v.y, v.z]).collect();
    rows_array(py, &rows)
}

/// Wraps a vector as a new 1D array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `values` - The values.
///
/// # Returns
///
/// The array.
pub fn vector_array<'py, T: Element + Copy>(py: Python<'py>, values: &[T]) -> Bound<'py, PyAny> {
    Array1::from_vec(values.to_vec())
        .into_pyarray(py)
        .into_any()
}

/// Wraps indices as a new int64 array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `values` - The indices.
///
/// # Returns
///
/// The array.
pub fn indices_array<'py>(py: Python<'py>, values: &[usize]) -> Bound<'py, PyAny> {
    let converted: Vec<i64> = values.iter().map(|&v| to_i64(v)).collect();
    vector_array(py, &converted)
}

/// Builds the dict of a map's fields.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map.
///
/// # Returns
///
/// The dict.
///
/// # Errors
///
/// A Python error if an item cannot be set.
pub fn map_dict<'py>(py: Python<'py>, m: &Map) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("levels", m.levels)?;
    out.set_item("scale_factor", m.scale_factor)?;
    out.set_item("kf_id", vector_array(py, &m.kf_id))?;
    out.set_item("kf_time", vector_array(py, &m.kf_time))?;
    let poses: Vec<f64> = m
        .kf_pose
        .iter()
        .flat_map(|p| p.transpose().iter().copied().collect::<Vec<_>>())
        .collect();
    let poses = Array3::from_shape_vec((m.kf_pose.len(), 4, 4), poses)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    out.set_item("kf_pose", poses.into_pyarray(py))?;
    out.set_item("kf_start", indices_array(py, &m.kf_start))?;
    out.set_item("kp_xy", rows_array(py, &m.kp_xy)?)?;
    out.set_item("kp_level", indices_array(py, &m.kp_level))?;
    out.set_item("kp_angle", indices_array(py, &m.kp_angle))?;
    out.set_item("kp_descriptor", rows_array(py, &m.kp_descriptor)?)?;
    out.set_item("kp_bearing", vectors_array(py, &m.kp_bearing)?)?;
    out.set_item("kp_depth", vector_array(py, &m.kp_depth))?;
    out.set_item("kp_right", vector_array(py, &m.kp_right))?;
    out.set_item("kp_point", vector_array(py, &m.kp_point))?;
    out.set_item("pt_id", vector_array(py, &m.pt_id))?;
    out.set_item("pt_xyz", vectors_array(py, &m.pt_xyz)?)?;
    out.set_item("pt_descriptor", rows_array(py, &m.pt_descriptor)?)?;
    out.set_item("pt_normal", vectors_array(py, &m.pt_normal)?)?;
    out.set_item("pt_min_distance", vector_array(py, &m.pt_min_distance))?;
    out.set_item("pt_max_distance", vector_array(py, &m.pt_max_distance))?;
    out.set_item("pt_visible", vector_array(py, &m.pt_visible))?;
    out.set_item("pt_found", vector_array(py, &m.pt_found))?;
    out.set_item("pt_first_kf", vector_array(py, &m.pt_first_kf))?;
    out.set_item("pt_reference_kf", vector_array(py, &m.pt_reference_kf))?;
    out.set_item("next_kf_id", m.next_kf_id)?;
    out.set_item("next_pt_id", m.next_pt_id)?;
    Ok(out)
}

/// Reads a frame from its dict.
///
/// # Arguments
///
/// * `values` - The dict.
///
/// # Returns
///
/// The frame.
///
/// # Errors
///
/// `ValueError` for a missing or unknown key, or arrays that are malformed or not one row per
/// keypoint.
pub fn frame_in(values: &Bound<'_, PyDict>) -> PyResult<Frame> {
    let get = |key| item(values, &FRAME_KEYS, key);
    let xy = rows::<f64, 2>(&get("xy")?)?;
    let level = indices(&get("level")?)?;
    let level_xy = rows::<i64, 2>(&get("level_xy")?)?;
    let response = vector::<i64>(&get("response")?)?;
    let angle = indices(&get("angle")?)?;
    let descriptors: Vec<Descriptor> = rows::<u8, DESCRIPTOR_BYTES>(&get("descriptors")?)?;
    let bearings = vectors(&get("bearings")?)?;
    let depth = vector::<f64>(&get("depth")?)?;
    let right = vector::<f64>(&get("right")?)?;
    let n = xy.len();
    if [
        level.len(),
        level_xy.len(),
        response.len(),
        angle.len(),
        descriptors.len(),
        bearings.len(),
        depth.len(),
        right.len(),
    ]
    .iter()
    .any(|&len| len != n)
    {
        return Err(PyValueError::new_err(
            "frame: expected one row per keypoint in every array",
        ));
    }
    let keypoints = (0..n)
        .map(|i| {
            let [x, y] = level_xy[i];
            Ok(Keypoint {
                xy: xy[i],
                level: level[i],
                level_xy: [
                    usize::try_from(x)
                        .map_err(|e| PyValueError::new_err(format!("level_xy: {e}")))?,
                    usize::try_from(y)
                        .map_err(|e| PyValueError::new_err(format!("level_xy: {e}")))?,
                ],
                response: response[i],
                angle: angle[i],
            })
        })
        .collect::<PyResult<Vec<Keypoint>>>()?;
    Ok(Frame {
        timestamp: get("timestamp")?.extract()?,
        features: Features {
            keypoints,
            descriptors,
            scale_factor: get("scale_factor")?.extract()?,
            levels: get("levels")?.extract()?,
        },
        bearings,
        depth,
        right,
    })
}

/// The camera's fields, as the dict's keys.
const CAMERA_KEYS: [&str; 5] = ["model", "intrinsics", "width", "height", "baseline"];

/// Reads the camera from its dict.
///
/// # Arguments
///
/// * `values` - The dict, with exactly `Camera`'s fields.
///
/// # Returns
///
/// The camera (checked by the core).
///
/// # Errors
///
/// `ValueError` for a missing or unknown key, a malformed array, or an unknown model.
pub fn camera_in(values: &Bound<'_, PyDict>) -> PyResult<Camera> {
    let get = |key| item(values, &CAMERA_KEYS, key);
    let model: String = get("model")?.extract()?;
    Ok(Camera {
        model: by_name(&Model::ALL, Model::name, &model, "model")?,
        intrinsics: vector(&get("intrinsics")?)?,
        width: get("width")?.extract()?,
        height: get("height")?.extract()?,
        baseline: get("baseline")?.extract()?,
    })
}

/// Reads `edge.Options` from a dict holding exactly the named fields: the struct literal
/// lists every field, so one missing here fails to build.
macro_rules! read_options {
    ($values:ident; $($field:ident),+ $(,)?) => {{
        const KEYS: &[&str] = &[$(stringify!($field)),+];
        Options { $($field: item($values, KEYS, stringify!($field))?.extract()?,)+ }
    }};
}

/// Reads the settings from a dict of scalars.
///
/// # Arguments
///
/// * `values` - The dict, with exactly `Options`' fields.
///
/// # Returns
///
/// The settings (checked by the core).
///
/// # Errors
///
/// `ValueError` for a missing or unknown key; a Python error for a value of the wrong type.
pub fn options_in(values: &Bound<'_, PyDict>) -> PyResult<Options> {
    Ok(read_options!(values;
        n_features, levels, scale_factor, fast_threshold, min_fast_threshold, close_depth,
        init_feature_factor, min_init_features, min_depth_features, init_window, init_ratio,
        init_min_matches, motion_window, stereo_motion_window, min_motion_matches, local_window,
        rgbd_local_window, min_tracked, chi2_mono, chi2_stereo, pose_rounds, view_cos_limit,
        max_frames, ref_ratio, mono_ref_ratio, min_keyframe_inliers, close_tracked,
        close_untracked, window, min_new_points, triangulation_ratio, max_parallax_cos,
        scale_ratio, fuse_window, ba_iterations, ba_refine_iterations, found_ratio,
        min_point_observations, min_point_observations_depth, redundant_fraction,
        redundant_observers, pnp_threshold, pnp_min_inliers, lost_frames, seed,
    ))
}

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
pub fn pose_in(pose: &PyReadonlyArray2<'_, f64>) -> PyResult<Matrix4<f64>> {
    if pose.shape() != [4, 4] {
        return Err(PyValueError::new_err("pose: expected shape (4, 4)"));
    }
    Ok(Matrix4::from_row_slice(contiguous(pose)?))
}

/// A tracked frame's fields, as the dict's keys.
const TRACKED_KEYS: [&str; 2] = ["pose", "points"];

/// Reads a tracked frame from its dict.
///
/// # Arguments
///
/// * `values` - The dict: `pose` (4, 4) and `points` (N,).
///
/// # Returns
///
/// The tracked frame.
///
/// # Errors
///
/// `ValueError` for a missing or unknown key, or a malformed array.
pub fn tracked_in(values: &Bound<'_, PyDict>) -> PyResult<Tracked> {
    let get = |key| item(values, &TRACKED_KEYS, key);
    Ok(Tracked {
        pose: pose_in(&get("pose")?.extract()?)?,
        points: vector(&get("points")?)?,
    })
}

/// Builds a tracked frame's dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `tracked` - The tracked frame.
///
/// # Returns
///
/// `pose` (4, 4) and `points` (N,).
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn tracked_dict<'py>(py: Python<'py>, tracked: &Tracked) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("pose", item_array(py, &tracked.pose)?)?;
    out.set_item("points", vector_array(py, &tracked.points))?;
    Ok(out)
}

/// Builds a tracked frame's dict, or None.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `tracked` - The tracked frame, if any.
///
/// # Returns
///
/// The dict (see [`tracked_dict`]), or None.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn maybe_tracked(py: Python<'_>, tracked: Option<Tracked>) -> PyResult<Bound<'_, PyAny>> {
    match tracked {
        Some(tracked) => Ok(tracked_dict(py, &tracked)?.into_any()),
        None => Ok(py.None().into_bound(py)),
    }
}

/// Reads a mode by its name (`slamlab.edge.options.MODES`).
///
/// # Arguments
///
/// * `name` - The name.
///
/// # Returns
///
/// The mode.
///
/// # Errors
///
/// `ValueError` for an unknown name.
pub fn mode_in(name: &str) -> PyResult<Mode> {
    by_name(&Mode::ALL, Mode::name, name, "mode")
}

/// Builds a started map's dict, or None.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `started` - The map and its tracked frame, if any.
///
/// # Returns
///
/// `map` (as [`map_dict`]), `pose` (4, 4) and `points` (N,); or None.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn started_dict(py: Python<'_>, started: Option<Started>) -> PyResult<Bound<'_, PyAny>> {
    let Some(started) = started else {
        return Ok(py.None().into_bound(py));
    };
    let out = tracked_dict(py, &started.tracked)?;
    out.set_item("map", map_dict(py, &started.map)?)?;
    Ok(out.into_any())
}
