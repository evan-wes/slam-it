//! Bindings for `slamlab.foundations.matching` (`slamlab_rs.foundations.matching`).
//!
//! Descriptor sets cross as C-contiguous (N, 32) uint8 arrays, orientation bins as int64
//! arrays, and the filters as one dict of `slamlab.foundations.matching.Options`' (or
//! `SearchOptions`') fields, read strictly by serde; matches come back as a dict of `Matches`'
//! fields. A search by projection takes its projections as a dict of `Projections`' fields and
//! the frame's keypoints as a dict of arrays (see `slamlab.foundations.matching.Implementation`);
//! the grid is built per call.

use numpy::{
    IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods,
    ndarray::{Array1, Array2},
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slamlab_foundations::matching as core;
use slamlab_foundations::orb::{DESCRIPTOR_BYTES, Descriptor, Features, Keypoint};

use crate::common::{add_functions, contiguous, int64_array, to_i64, unwrap};

/// `matching.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OptionsIn {
    /// The distance cap.
    max_distance: i64,
    /// The ratio test's bound.
    ratio: f64,
    /// Whether to check rotation consistency.
    check_orientation: bool,
}

/// `Projections`' fields, as the dict's keys.
const PROJECTION_KEYS: [&str; 7] = [
    "xy",
    "radius",
    "min_level",
    "max_level",
    "descriptors",
    "angle",
    "right",
];

/// The frame's fields a search reads, as the dict's keys.
const FRAME_KEYS: [&str; 7] = [
    "xy",
    "level",
    "angle",
    "descriptors",
    "right",
    "taken",
    "levels",
];

/// Copies a descriptor set in.
///
/// # Arguments
///
/// * `descriptors` - An (N, 32) uint8 array.
///
/// # Returns
///
/// The descriptors.
///
/// # Errors
///
/// `ValueError` for another shape or a non-contiguous array.
fn descriptors_in(descriptors: &PyReadonlyArray2<'_, u8>) -> PyResult<Vec<Descriptor>> {
    if descriptors.shape()[1] != DESCRIPTOR_BYTES {
        return Err(PyValueError::new_err(format!(
            "expected shape (N, {DESCRIPTOR_BYTES}), got {:?}",
            descriptors.shape()
        )));
    }
    Ok(contiguous(descriptors)?
        .as_chunks::<DESCRIPTOR_BYTES>()
        .0
        .to_vec())
}

/// Copies orientation bins in.
///
/// # Arguments
///
/// * `angles` - An int64 array.
///
/// # Returns
///
/// The bins.
///
/// # Errors
///
/// `ValueError` for a negative bin or a non-contiguous array.
fn angles_in(angles: &PyReadonlyArray1<'_, i64>) -> PyResult<Vec<usize>> {
    contiguous(angles)?
        .iter()
        .map(|&a| usize::try_from(a).map_err(|e| PyValueError::new_err(format!("angles: {e}"))))
        .collect()
}

/// Wraps row-major values as a new int64 array of the given shape.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `shape` - `(rows, cols)`.
/// * `values` - The values.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the count does not match the shape (a binding bug).
fn int64_matrix(
    py: Python<'_>,
    shape: (usize, usize),
    values: Vec<i64>,
) -> PyResult<Bound<'_, PyArray2<i64>>> {
    let array =
        Array2::from_shape_vec(shape, values).map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}

/// Returns every pair's Hamming distance.
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
///
/// # Returns
///
/// A new (N, M) int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` from the core.
#[pyfunction]
fn distances<'py>(
    py: Python<'py>,
    descriptors1: PyReadonlyArray2<'py, u8>,
    descriptors2: PyReadonlyArray2<'py, u8>,
) -> PyResult<Bound<'py, PyArray2<i64>>> {
    let (first, second) = (
        descriptors_in(&descriptors1)?,
        descriptors_in(&descriptors2)?,
    );
    let values = unwrap(py, core::distances(&first, &second))?;
    int64_matrix(py, (first.len(), second.len()), values)
}

/// Matches two descriptor sets; returns a dict of `Matches`' fields.
///
/// # Arguments
///
/// * `descriptors1` - The first set.
/// * `descriptors2` - The second set.
/// * `angles1` - The first set's orientation bins.
/// * `angles2` - The second set's.
/// * `options` - The filters, as a dict.
///
/// # Returns
///
/// A dict with `pairs` (K, 2) and `distances` (K,), both int64.
///
/// # Errors
///
/// `ValueError` for a malformed array or dict; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(name = "match")]
fn match_descriptors<'py>(
    py: Python<'py>,
    descriptors1: PyReadonlyArray2<'py, u8>,
    descriptors2: PyReadonlyArray2<'py, u8>,
    angles1: PyReadonlyArray1<'py, i64>,
    angles2: PyReadonlyArray1<'py, i64>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    let settings = core::Options {
        max_distance: raw.max_distance,
        ratio: raw.ratio,
        check_orientation: raw.check_orientation,
    };
    let matches = unwrap(
        py,
        core::match_descriptors(
            &descriptors_in(&descriptors1)?,
            &descriptors_in(&descriptors2)?,
            &angles_in(&angles1)?,
            &angles_in(&angles2)?,
            &settings,
        ),
    )?;
    matches_dict(py, &matches)
}

/// Wraps matches as a dict of `Matches`' fields.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `matches` - The matches.
///
/// # Returns
///
/// A dict with `pairs` (K, 2) and `distances` (K,), both int64.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn matches_dict<'py>(py: Python<'py>, matches: &[core::Match]) -> PyResult<Bound<'py, PyDict>> {
    let pairs = matches
        .iter()
        .flat_map(|m| [to_i64(m.first), to_i64(m.second)])
        .collect();
    let out = PyDict::new(py);
    out.set_item("pairs", int64_matrix(py, (matches.len(), 2), pairs)?)?;
    let distances: Vec<i64> = matches.iter().map(|m| m.distance).collect();
    out.set_item("distances", Array1::from_vec(distances).into_pyarray(py))?;
    Ok(out)
}

/// Converts levels to the core's.
///
/// # Arguments
///
/// * `levels` - The int64 levels.
///
/// # Returns
///
/// The levels.
///
/// # Errors
///
/// `ValueError` for a negative level.
fn levels_in(levels: &[i64]) -> PyResult<Vec<usize>> {
    levels
        .iter()
        .map(|&l| usize::try_from(l).map_err(|e| PyValueError::new_err(format!("level: {e}"))))
        .collect()
}

/// Reads keypoints' positions and levels (the rest zero).
///
/// # Arguments
///
/// * `xy` - The (N, 2) positions.
/// * `level` - The (N,) levels.
///
/// # Returns
///
/// The keypoints.
///
/// # Errors
///
/// `ValueError` for mismatched or malformed arrays, or a negative level.
fn keypoints_in(
    xy: &PyReadonlyArray2<'_, f64>,
    level: &PyReadonlyArray1<'_, i64>,
) -> PyResult<Vec<Keypoint>> {
    let n = xy.shape()[0];
    if xy.shape()[1] != 2 || level.len() != n {
        return Err(PyValueError::new_err(
            "keypoints: expected (N, 2) positions and one level each",
        ));
    }
    let (positions, levels) = (contiguous(xy)?, levels_in(contiguous(level)?)?);
    Ok((0..n)
        .map(|i| Keypoint {
            xy: [positions[2 * i], positions[2 * i + 1]],
            level: levels[i],
            level_xy: [0, 0],
            response: 0,
            angle: 0,
        })
        .collect())
}

/// Returns the indices of the keypoints in a window, ascending.
///
/// # Arguments
///
/// * `xy` - The keypoints' positions.
/// * `level` - Their levels.
/// * `center` - The window's centre.
/// * `radius` - Its half-width.
/// * `min_level` - The lowest level it covers.
/// * `max_level` - The highest.
///
/// # Returns
///
/// A new (K,) int64 array.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` from the core.
#[pyfunction]
fn features_in_area<'py>(
    py: Python<'py>,
    xy: PyReadonlyArray2<'py, f64>,
    level: PyReadonlyArray1<'py, i64>,
    center: [f64; 2],
    radius: f64,
    min_level: i64,
    max_level: i64,
) -> PyResult<Bound<'py, PyArray1<i64>>> {
    let keypoints = keypoints_in(&xy, &level)?;
    let found = unwrap(
        py,
        core::features_in_area(&keypoints, center, radius, min_level, max_level),
    )?;
    Ok(int64_array(py, found.into_iter().map(to_i64).collect()))
}

/// Reads one item of a dict, checking the dict holds exactly the expected keys.
///
/// # Arguments
///
/// * `values` - The dict.
/// * `keys` - Its expected keys.
/// * `key` - The item's key.
/// * `what` - The dict's name, for the message.
///
/// # Returns
///
/// The item.
///
/// # Errors
///
/// `ValueError` for a missing key, or a dict with other keys.
fn item<'py>(
    values: &Bound<'py, PyDict>,
    keys: &[&str],
    key: &str,
    what: &str,
) -> PyResult<Bound<'py, PyAny>> {
    if values.len() != keys.len() {
        return Err(PyValueError::new_err(format!(
            "{what}: expected exactly the keys {keys:?}"
        )));
    }
    values
        .get_item(key)?
        .ok_or_else(|| PyValueError::new_err(format!("{what}: missing `{key}`")))
}

/// Reads projections from a dict of `Projections`' fields.
///
/// # Arguments
///
/// * `values` - The dict.
///
/// # Returns
///
/// The projections.
///
/// # Errors
///
/// `ValueError` for a missing or unknown key, or a malformed or mismatched array.
fn projections_in(values: &Bound<'_, PyDict>) -> PyResult<Vec<core::Projection>> {
    let get = |key| item(values, &PROJECTION_KEYS, key, "projections");
    let xy: PyReadonlyArray2<'_, f64> = get("xy")?.extract()?;
    let radius: PyReadonlyArray1<'_, f64> = get("radius")?.extract()?;
    let min_level: PyReadonlyArray1<'_, i64> = get("min_level")?.extract()?;
    let max_level: PyReadonlyArray1<'_, i64> = get("max_level")?.extract()?;
    let descriptors = descriptors_in(&get("descriptors")?.extract()?)?;
    let angle = angles_in(&get("angle")?.extract()?)?;
    let right: PyReadonlyArray1<'_, f64> = get("right")?.extract()?;
    let m = descriptors.len();
    if xy.shape() != [m, 2]
        || radius.len() != m
        || min_level.len() != m
        || max_level.len() != m
        || angle.len() != m
        || right.len() != m
    {
        return Err(PyValueError::new_err(
            "projections: expected one row per projection in every array",
        ));
    }
    let (xy, radius, right) = (contiguous(&xy)?, contiguous(&radius)?, contiguous(&right)?);
    let (min_level, max_level) = (contiguous(&min_level)?, contiguous(&max_level)?);
    Ok((0..m)
        .map(|i| core::Projection {
            xy: [xy[2 * i], xy[2 * i + 1]],
            radius: radius[i],
            min_level: min_level[i],
            max_level: max_level[i],
            descriptor: descriptors[i],
            angle: angle[i],
            right: right[i],
        })
        .collect())
}

/// A frame as a search reads it: its features, right columns and taken flags.
type FrameIn = (Features, Vec<f64>, Vec<bool>);

/// Reads a frame from a dict.
///
/// # Arguments
///
/// * `values` - The dict: `xy`, `level`, `angle`, `descriptors`, `right`, `taken`, `levels`.
///
/// # Returns
///
/// The frame (responses and level pixels zero; the scale factor unused).
///
/// # Errors
///
/// `ValueError` for a missing or unknown key, or a malformed or mismatched array.
fn frame_in(values: &Bound<'_, PyDict>) -> PyResult<FrameIn> {
    let get = |key| item(values, &FRAME_KEYS, key, "keypoints");
    let xy: PyReadonlyArray2<'_, f64> = get("xy")?.extract()?;
    let level: PyReadonlyArray1<'_, i64> = get("level")?.extract()?;
    let mut keypoints = keypoints_in(&xy, &level)?;
    let angle = angles_in(&get("angle")?.extract()?)?;
    let right: PyReadonlyArray1<'_, f64> = get("right")?.extract()?;
    let taken: PyReadonlyArray1<'_, bool> = get("taken")?.extract()?;
    let n = keypoints.len();
    if angle.len() != n || right.len() != n || taken.len() != n {
        return Err(PyValueError::new_err(
            "keypoints: expected one row per keypoint in every array",
        ));
    }
    for (k, &a) in keypoints.iter_mut().zip(&angle) {
        k.angle = a;
    }
    let features = Features {
        keypoints,
        descriptors: descriptors_in(&get("descriptors")?.extract()?)?,
        scale_factor: 0.0,
        levels: get("levels")?.extract()?,
    };
    Ok((
        features,
        contiguous(&right)?.to_vec(),
        contiguous(&taken)?.to_vec(),
    ))
}

/// Matches projected map points to a frame's keypoints; returns a dict of `Matches`' fields.
///
/// # Arguments
///
/// * `projections` - A dict of `Projections`' fields.
/// * `keypoints` - The frame's keypoints, as a dict.
/// * `options` - The filters, as a dict of `SearchOptions`' fields.
///
/// # Returns
///
/// A dict with `pairs` (K, 2) and `distances` (K,), both int64.
///
/// # Errors
///
/// `ValueError` for a malformed array or dict; `InvalidInputError` from the core.
#[pyfunction]
fn search_by_projection<'py>(
    py: Python<'py>,
    projections: &Bound<'py, PyDict>,
    keypoints: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    let settings = core::SearchOptions {
        max_distance: raw.max_distance,
        ratio: raw.ratio,
        check_orientation: raw.check_orientation,
    };
    let projected = projections_in(projections)?;
    let (features, right, taken) = frame_in(keypoints)?;
    let grid = unwrap(py, core::FeatureGrid::build(&features.keypoints))?;
    let matches = unwrap(
        py,
        core::search_by_projection(&projected, &features, &grid, &right, &taken, &settings),
    )?;
    matches_dict(py, &matches)
}

/// Registers the `matching` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn matching(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; distances, match_descriptors, features_in_area, search_by_projection);
    Ok(())
}
