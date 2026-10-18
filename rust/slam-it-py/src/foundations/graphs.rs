//! Bindings for `slam_it.foundations.graphs` (`slam_it_rs.foundations.graphs`).
//!
//! Each solve takes its arrays and options by keyword, as the Python `Implementation`
//! Protocol names them, and returns a dict: its arrays, and the summary's fields
//! (`initial_error`, `final_error`, `iterations`, `termination` as its code in `codes()`).
//! The options cross as `graphs.Options.as_dict()`, read strictly by serde.

use nalgebra::{Matrix4, Vector2, Vector3};
use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slam_it_foundations::camera::Model;
use slam_it_foundations::graphs::{self as core, Lens, Matches, Observations, Termination};
use slam_it_foundations::nls::Stop;

use crate::common::{
    ArrayIn, add_functions, contiguous, decode, encode, enum_names, item_array, items_array,
    new_array, one, rows, unwrap,
};

/// `graphs.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OptionsIn {
    /// Iterations at most.
    max_iterations: usize,
    /// Levenberg-Marquardt's first damping.
    initial_damping: f64,
    /// The relative tolerance.
    relative_tolerance: f64,
    /// The absolute tolerance.
    absolute_tolerance: f64,
}

/// Reads a solve's options from `graphs.Options.as_dict()`.
///
/// # Arguments
///
/// * `options` - The dict.
///
/// # Returns
///
/// The options (checked by the core).
///
/// # Errors
///
/// `ValueError` for a missing or unknown field.
fn read_options(options: &Bound<'_, PyDict>) -> PyResult<core::Options> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    Ok(core::Options {
        max_iterations: raw.max_iterations,
        initial_damping: raw.initial_damping,
        relative_tolerance: raw.relative_tolerance,
        absolute_tolerance: raw.absolute_tolerance,
        stop: Stop::never(), // a compiled core's stop is its own threads'
    })
}

/// Indices crossing as int64, as `usize`.
///
/// # Arguments
///
/// * `values` - The int64 values.
/// * `what` - What they are, for the message.
///
/// # Returns
///
/// The indices.
///
/// # Errors
///
/// `ValueError` for a negative value.
fn indices(values: &[i64], what: &str) -> PyResult<Vec<usize>> {
    values
        .iter()
        .map(|&v| {
            usize::try_from(v)
                .map_err(|_| PyValueError::new_err(format!("{what}: expected indices >= 0")))
        })
        .collect()
}

/// Writes a summary's fields into a result dict.
///
/// # Arguments
///
/// * `summary` - The summary.
/// * `out` - The dict.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn write_summary(summary: &core::Summary, out: &Bound<'_, PyDict>) -> PyResult<()> {
    out.set_item("initial_error", summary.initial_error)?;
    out.set_item("final_error", summary.final_error)?;
    out.set_item("iterations", summary.iterations)?;
    out.set_item(
        "termination",
        encode(&Termination::ALL, summary.termination)?,
    )?;
    Ok(())
}

/// The enums' name tables, by enum, in code order.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"graphs_termination": [...], "model": [...]}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
#[pyfunction]
fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = PyDict::new(py);
    out.set_item(
        "graphs_termination",
        enum_names(&Termination::ALL, Termination::name),
    )?;
    out.set_item("model", enum_names(&Model::ALL, Model::name))?;
    Ok(out)
}

/// Optimises a pose graph (`graphs.optimize_pose_graph`).
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `poses` - (N, 4, 4) camera-to-world starts.
/// * `edges` - (E, 2) int64 pose indices.
/// * `relatives` - (E, 4, 4) measured `T_i_j`.
/// * `fixed` - int64, the poses held fixed.
/// * `similarity` - Optimise Sim(3) rather than SE(3).
/// * `options` - `graphs.Options.as_dict()`.
///
/// # Returns
///
/// `{"poses", "initial_error", "final_error", "iterations", "termination"}`.
///
/// # Errors
///
/// `ValueError` for a malformed array or options; `InvalidInputError` for an invalid graph;
/// `NotConvergedError` if GTSAM fails.
#[pyfunction]
#[pyo3(signature = (*, poses, edges, relatives, fixed, similarity, options))]
fn optimize_pose_graph<'py>(
    py: Python<'py>,
    poses: ArrayIn<'py>,
    edges: PyReadonlyArray2<'py, i64>,
    relatives: ArrayIn<'py>,
    fixed: PyReadonlyArray1<'py, i64>,
    similarity: bool,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let starts = rows::<Matrix4<f64>>(&poses)?;
    let measured = rows::<Matrix4<f64>>(&relatives)?;
    let ends = indices(
        edges
            .as_slice()
            .map_err(|e| PyValueError::new_err(e.to_string()))?,
        "edges",
    )?;
    let pairs: Vec<[usize; 2]> = ends.as_chunks::<2>().0.to_vec();
    let held = indices(contiguous(&fixed)?, "fixed")?;
    let options = read_options(&options)?;
    let result = unwrap(
        py,
        py.detach(|| {
            core::optimize_pose_graph(&starts, &pairs, &measured, &held, similarity, &options)
        }),
    )?;
    let out = PyDict::new(py);
    out.set_item("poses", items_array(py, &result.poses)?)?;
    write_summary(&result.summary, &out)?;
    Ok(out)
}

/// Refines a similarity against its matches (`graphs.refine_similarity`).
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `similarity` - The starting `S_12`, (4, 4).
/// * `points1`, `points2` - (M, 3) points in each camera's frame.
/// * `pixels1`, `pixels2` - (M, 2) where each image observes them.
/// * `information` - (M, 2) informations, image 1 then image 2.
/// * `model` - The camera model's code; `intrinsics` its intrinsics.
/// * `fixed_scale` - Hold the scale at 1.
/// * `huber` - The Huber loss's threshold, or None for no robust loss.
/// * `options` - `graphs.Options.as_dict()`.
///
/// # Returns
///
/// `{"similarity", "chi2", "initial_error", "final_error", "iterations", "termination"}`.
///
/// # Errors
///
/// `ValueError` for a malformed array, an unknown model or bad options; `InvalidInputError`
/// for invalid matches; `NotConvergedError` if GTSAM fails.
#[pyfunction]
#[pyo3(signature = (
    *, similarity, points1, points2, pixels1, pixels2, information, model, intrinsics,
    fixed_scale, huber, options
))]
#[allow(
    clippy::too_many_arguments,
    reason = "every argument crosses by keyword, by name"
)]
fn refine_similarity<'py>(
    py: Python<'py>,
    similarity: ArrayIn<'py>,
    points1: ArrayIn<'py>,
    points2: ArrayIn<'py>,
    pixels1: ArrayIn<'py>,
    pixels2: ArrayIn<'py>,
    information: ArrayIn<'py>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    fixed_scale: bool,
    huber: Option<f64>,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let start = one::<Matrix4<f64>>(&similarity)?;
    let one_side = rows::<Vector3<f64>>(&points1)?;
    let other_side = rows::<Vector3<f64>>(&points2)?;
    let seen1 = rows::<Vector2<f64>>(&pixels1)?;
    let seen2 = rows::<Vector2<f64>>(&pixels2)?;
    let weights: Vec<[f64; 2]> = rows::<Vector2<f64>>(&information)?
        .iter()
        .map(|w| [w.x, w.y])
        .collect();
    let model = decode(&Model::ALL, model, "model")?;
    let camera = contiguous(&intrinsics)?.to_vec();
    let options = read_options(&options)?;
    let result = unwrap(
        py,
        py.detach(|| {
            core::refine_similarity(
                &start,
                &Matches {
                    points1: &one_side,
                    points2: &other_side,
                    pixels1: &seen1,
                    pixels2: &seen2,
                    information: &weights,
                },
                model,
                &camera,
                fixed_scale,
                huber,
                &options,
            )
        }),
    )?;
    let chi2: Vec<f64> = result.chi2.iter().flatten().copied().collect();
    let out = PyDict::new(py);
    out.set_item("similarity", item_array(py, &result.similarity)?)?;
    out.set_item("chi2", new_array(py, &[result.chi2.len(), 2], chi2)?)?;
    write_summary(&result.summary, &out)?;
    Ok(out)
}

/// Adjusts a bundle (`graphs.bundle_adjust`).
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `poses` - (C, 4, 4) camera-to-world starts; `points` (P, 3) world points.
/// * `cameras`, `point_ids` - int64, each observation's camera and point.
/// * `pixels` - (M, 2) observed pixels; `right` each right column, NaN for none.
/// * `information` - Each observation's information.
/// * `model` - The camera model's code; `intrinsics` its intrinsics; `bf` the baseline times
///   the focal length.
/// * `fixed` - int64, the cameras held fixed.
/// * `huber` - Each observation's Huber threshold, or None for no robust loss.
/// * `options` - `graphs.Options.as_dict()`.
///
/// # Returns
///
/// `{"poses", "points", "initial_error", "final_error", "iterations", "termination"}`.
///
/// # Errors
///
/// `ValueError` for a malformed array, an unknown model or bad options; `InvalidInputError`
/// for an invalid bundle; `NotConvergedError` if GTSAM fails.
#[pyfunction]
#[pyo3(signature = (
    *, poses, points, cameras, point_ids, pixels, right, information, model, intrinsics, bf,
    fixed, huber, options
))]
#[allow(
    clippy::too_many_arguments,
    reason = "every argument crosses by keyword, by name"
)]
fn bundle_adjust<'py>(
    py: Python<'py>,
    poses: ArrayIn<'py>,
    points: ArrayIn<'py>,
    cameras: PyReadonlyArray1<'py, i64>,
    point_ids: PyReadonlyArray1<'py, i64>,
    pixels: ArrayIn<'py>,
    right: PyReadonlyArray1<'py, f64>,
    information: PyReadonlyArray1<'py, f64>,
    model: i64,
    intrinsics: PyReadonlyArray1<'py, f64>,
    bf: f64,
    fixed: PyReadonlyArray1<'py, i64>,
    huber: Option<PyReadonlyArray1<'py, f64>>,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let thresholds = match &huber {
        Some(given) => contiguous(given)?.to_vec(),
        None => Vec::new(),
    };
    let starts = rows::<Matrix4<f64>>(&poses)?;
    let world = rows::<Vector3<f64>>(&points)?;
    let which = indices(contiguous(&cameras)?, "cameras")?;
    let whose = indices(contiguous(&point_ids)?, "point_ids")?;
    let seen = rows::<Vector2<f64>>(&pixels)?;
    let columns = contiguous(&right)?.to_vec();
    let weights = contiguous(&information)?.to_vec();
    let lens = Lens {
        model: decode(&Model::ALL, model, "model")?,
        intrinsics: contiguous(&intrinsics)?.to_vec(),
        bf,
    };
    let held = indices(contiguous(&fixed)?, "fixed")?;
    let options = read_options(&options)?;
    let result = unwrap(
        py,
        py.detach(|| {
            core::bundle_adjust(
                &starts,
                &world,
                &Observations {
                    cameras: &which,
                    point_ids: &whose,
                    pixels: &seen,
                    right: &columns,
                    information: &weights,
                    huber: &thresholds,
                },
                &lens,
                &held,
                &options,
            )
        }),
    )?;
    let out = PyDict::new(py);
    out.set_item("poses", items_array(py, &result.poses)?)?;
    out.set_item("points", items_array(py, &result.points)?)?;
    write_summary(&result.summary, &out)?;
    Ok(out)
}

/// Registers the `graphs` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn graphs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, optimize_pose_graph, refine_similarity, bundle_adjust);
    Ok(())
}
