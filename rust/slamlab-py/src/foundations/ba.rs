//! Bindings for `slamlab.foundations.ba` (`slamlab_rs.foundations.ba`).
//!
//! Everything crosses by name: the problem's arrays are keyword arguments, and the options
//! one dict of `BAOptions`' fields, nested where `BAOptions` nests (`solver`, the losses),
//! with each choice by its name, read strictly by serde (a missing or unknown field is an
//! error at every depth). The camera model, a standalone argument, crosses as a code; so do
//! the enums in the result. Every enum's names come from its core's `name()`, exported by
//! `codes()` so the Python side's tables are tested against them.

use nalgebra::{Matrix4, Vector2, Vector3};
use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slamlab_foundations::ba::{self as core, Gauge, Intrinsics};
use slamlab_foundations::camera::Model;
use slamlab_foundations::nls::{LossKind, Method};

use super::nls_crossing::{self, LossIn, OptionsIn};
use crate::common::{
    ArrayIn, add_functions, by_name, decode, encode, enum_names, items_array, new_array, rows,
    slice, unwrap,
};

/// `ba.BAOptions`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BaOptionsIn {
    /// The least-squares settings.
    solver: OptionsIn,
    /// The depth priors' loss.
    depth_loss: LossIn,
    /// The gauge, by name.
    gauge: String,
    /// The parallax below which a free point is dropped (degrees).
    min_parallax_deg: f64,
    /// The relative focal deviation above which a focal length is unobservable.
    focal_std_ratio: f64,
    /// Whether the deviations are scaled by the variance factor.
    scale_deviations: bool,
}

/// Reads bundle adjustment's options from a dict of `BAOptions`' fields.
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
/// `ValueError` for a missing or unknown field, an unknown name, or a `block_loss` set by
/// the caller.
fn read_options(options: &Bound<'_, PyDict>) -> PyResult<core::Options> {
    let raw: BaOptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    Ok(core::Options {
        solver: raw.solver.read()?,
        depth_loss: raw.depth_loss.read("depth_loss")?,
        gauge: by_name(&Gauge::ALL, Gauge::name, &raw.gauge, "gauge")?,
        min_parallax_deg: raw.min_parallax_deg,
        focal_std_ratio: raw.focal_std_ratio,
        scale_deviations: raw.scale_deviations,
    })
}

/// The enums' name tables, by enum, in code order.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"model", "gauge", "method", "loss", "nls_termination"}`, each a list of names.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
#[pyfunction]
fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("model", enum_names(&Model::ALL, Model::name))?;
    out.set_item("gauge", enum_names(&Gauge::ALL, Gauge::name))?;
    out.set_item("method", enum_names(&Method::ALL, Method::name))?;
    out.set_item("loss", enum_names(&LossKind::ALL, LossKind::name))?;
    out.set_item("nls_termination", nls_crossing::terminations())?;
    Ok(out)
}

/// A pose index, or -1 for none, as it crosses.
///
/// A pose index always fits in `i64` (there are far fewer than 2⁶³ poses), so -1 means
/// "none" and nothing else.
///
/// # Arguments
///
/// * `index` - The index.
///
/// # Returns
///
/// The index, or -1.
fn code_of(index: Option<usize>) -> i64 {
    index.and_then(|i| i64::try_from(i).ok()).unwrap_or(-1)
}

/// Runs bundle adjustment on a problem given by named arrays, and its options.
///
/// # Arguments
///
/// Every field of `BAProblem`, by keyword, as the Python `Implementation` Protocol in
/// `src/slamlab/foundations/ba/solve.py` passes them:
///
/// * `intrinsics` - float64 (K, P), one group per row; `intrinsics_idx` int64 (C).
/// * `poses` - float64 (C, 4, 4), camera-to-world `T_world_cam`; `points` float64 (N, 3).
/// * `observations` - float64 (M, 2), pixels; `cam_idx`, `pt_idx` int64 (M).
/// * `weights`, `depth_prior`, `depth_prior_weights` - float64 (M).
/// * `fixed_poses` bool (C), `fixed_intrinsics` bool (K P, row-major), `fixed_points` bool
///   (N), `fixed_depth_affine` bool (C).
/// * `depth_affine` - float64 (C, 2); `pose_prior` float64 (C, 4, 4);
///   `pose_prior_weights` float64 (C, 2); `point_anchor` int64 (N).
/// * `model` - the camera model's code; `options` - a dict of `BAOptions`' fields.
///
/// # Returns
///
/// A dict of named results; see the Python `Implementation` Protocol.
///
/// # Errors
///
/// `ValueError` for a malformed array, an unknown code or name, or a malformed options
/// dict; `InvalidInputError` for an invalid problem; `NotConvergedError` in strict mode.
#[pyfunction]
#[pyo3(signature = (
    *, intrinsics, intrinsics_idx, poses, points, observations, cam_idx, pt_idx, weights,
    fixed_poses, fixed_intrinsics, fixed_points, depth_prior, depth_prior_weights,
    depth_affine, fixed_depth_affine, pose_prior, pose_prior_weights, point_anchor, model,
    options
))]
#[allow(
    clippy::needless_pass_by_value,
    clippy::too_many_arguments,
    reason = "every argument crosses by keyword, by name; PyO3 extracts them by value"
)]
fn solve<'py>(
    py: Python<'py>,
    intrinsics: PyReadonlyArray2<'py, f64>,
    intrinsics_idx: PyReadonlyArray1<'py, i64>,
    poses: ArrayIn<'py>,
    points: ArrayIn<'py>,
    observations: ArrayIn<'py>,
    cam_idx: PyReadonlyArray1<'py, i64>,
    pt_idx: PyReadonlyArray1<'py, i64>,
    weights: PyReadonlyArray1<'py, f64>,
    fixed_poses: PyReadonlyArray1<'py, bool>,
    fixed_intrinsics: PyReadonlyArray1<'py, bool>,
    fixed_points: PyReadonlyArray1<'py, bool>,
    depth_prior: PyReadonlyArray1<'py, f64>,
    depth_prior_weights: PyReadonlyArray1<'py, f64>,
    depth_affine: ArrayIn<'py>,
    fixed_depth_affine: PyReadonlyArray1<'py, bool>,
    pose_prior: ArrayIn<'py>,
    pose_prior_weights: ArrayIn<'py>,
    point_anchor: PyReadonlyArray1<'py, i64>,
    model: i64,
    options: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let params = intrinsics.as_array().ncols();
    let problem = core::Problem {
        model: decode(&Model::ALL, model, "model")?,
        intrinsics: Intrinsics {
            values: intrinsics
                .as_slice()
                .map_err(|e| PyValueError::new_err(e.to_string()))?
                .to_vec(),
            params,
        },
        poses: rows::<Matrix4<f64>>(&poses)?,
        points: rows::<Vector3<f64>>(&points)?,
        observations: rows::<Vector2<f64>>(&observations)?,
        cam_idx: slice(&cam_idx)?.to_vec(),
        pt_idx: slice(&pt_idx)?.to_vec(),
        intrinsics_idx: slice(&intrinsics_idx)?.to_vec(),
        weights: slice(&weights)?.to_vec(),
        fixed_poses: slice(&fixed_poses)?.to_vec(),
        fixed_intrinsics: slice(&fixed_intrinsics)?.to_vec(),
        fixed_points: slice(&fixed_points)?.to_vec(),
        depth_prior: slice(&depth_prior)?.to_vec(),
        depth_prior_weights: slice(&depth_prior_weights)?.to_vec(),
        depth_affine: rows::<Vector2<f64>>(&depth_affine)?,
        fixed_depth_affine: slice(&fixed_depth_affine)?.to_vec(),
        pose_prior: rows::<Matrix4<f64>>(&pose_prior)?,
        pose_prior_weights: rows::<Vector2<f64>>(&pose_prior_weights)?,
        point_anchor: slice(&point_anchor)?.to_vec(),
    };
    let options = read_options(&options)?;
    let solution = unwrap(py, py.detach(|| core::solve(&problem, &options)))?;
    solution_dict(py, &solution)
}

/// A solution as the dict the Python side reads (see the `Implementation` Protocol).
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `solution` - The solution.
///
/// # Returns
///
/// The refined arrays, the least-squares summary, the gauge code and the counts.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn solution_dict<'py>(py: Python<'py>, solution: &core::Solution) -> PyResult<Bound<'py, PyDict>> {
    let summary = &solution.summary;
    let (groups, params) = (solution.intrinsics.groups(), solution.intrinsics.params);
    let out = PyDict::new(py);
    out.set_item("poses", items_array(py, &solution.poses)?)?;
    out.set_item(
        "intrinsics",
        new_array(py, &[groups, params], solution.intrinsics.values.clone())?,
    )?;
    out.set_item("points", items_array(py, &solution.points)?)?;
    out.set_item("depth_affine", items_array(py, &solution.depth_affine)?)?;
    out.set_item(
        "inverse_depth",
        new_array(
            py,
            &[solution.inverse_depth.len()],
            solution.inverse_depth.clone(),
        )?,
    )?;
    out.set_item(
        "intrinsics_std",
        new_array(
            py,
            &[groups, params],
            solution.intrinsics_std.values.clone(),
        )?,
    )?;
    out.set_item("solve", nls_crossing::summary(py, &summary.solve)?)?;
    out.set_item("gauge", encode(&Gauge::ALL, summary.gauge))?;
    out.set_item("gauge_pose", code_of(summary.gauge_pose))?;
    out.set_item("confined_pose", code_of(summary.confined_pose))?;
    for (name, count) in [
        ("free_gauge", summary.free_gauge),
        ("dropped_points", summary.dropped_points),
        ("dropped_observations", summary.dropped_observations),
        ("unobserved_poses", summary.unobserved_poses),
        ("unobserved_intrinsics", summary.unobserved_intrinsics),
        ("unobserved_depth_affines", summary.unobserved_depth_affines),
        ("unobservable_focal", summary.unobservable_focal),
    ] {
        out.set_item(name, count)?;
    }
    out.set_item("variance_factor", summary.variance_factor)?;
    Ok(out)
}

/// Registers the `ba` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn ba(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, solve);
    Ok(())
}
