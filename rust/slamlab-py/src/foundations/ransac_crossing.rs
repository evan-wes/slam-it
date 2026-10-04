//! Shared by every binding that runs RANSAC: its options in, its solution out.
//!
//! The options cross as one dict of scalars keyed by `slamlab.foundations.ransac.Options`'
//! field names, read strictly by serde: a missing or unknown key is an error. A solution
//! crosses as a dict: `model`, `inliers` (bool) and `summary`, a dict of the summary's fields
//! with `termination` as its index in `codes()["ransac_termination"]`. A refined run's
//! summary crosses as a dict: `run`, `refinement` (an NLS summary, or None if it did not
//! run), `refined` and `rounds`; its module's `codes()` carries both `ransac_termination` and
//! `nls_termination`.

use numpy::IntoPyArray;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use slamlab_foundations::ransac::{self, Consensus, Termination};
use slamlab_foundations::solve;

use super::nls_crossing;
use crate::common::{Item, encode, enum_names, item_array};

/// `ransac.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OptionsIn {
    /// The inlier threshold on a residual.
    threshold: f64,
    /// The confidence that stops the run.
    confidence: f64,
    /// The iteration cap.
    max_iterations: i64,
    /// PCG32's seed.
    seed: u64,
    /// PCG32's stream.
    stream: u64,
    /// Whether to run local optimisation.
    local_optimization: bool,
    /// Local optimisation's threshold steps.
    lo_steps: i64,
    /// Its threshold's starting multiple.
    lo_threshold_multiplier: f64,
    /// Its inner RANSAC's iterations.
    lo_inner_iterations: i64,
    /// Its inner sample size, as a multiple of the minimal one.
    lo_sample_factor: i64,
}

/// Reads RANSAC's options from a dict of scalars.
///
/// # Arguments
///
/// * `options` - The dict, with exactly `ransac.Options`' fields.
///
/// # Returns
///
/// The options (checked by the core when it runs).
///
/// # Errors
///
/// `ValueError` for a missing or unknown field, or a value of the wrong type.
pub fn options(options: &Bound<'_, PyDict>) -> PyResult<ransac::Options> {
    let raw: OptionsIn = pythonize::depythonize(options.as_any())
        .map_err(|e| PyValueError::new_err(format!("options: {e}")))?;
    Ok(ransac::Options {
        threshold: raw.threshold,
        confidence: raw.confidence,
        max_iterations: raw.max_iterations,
        seed: raw.seed,
        stream: raw.stream,
        local_optimization: raw.local_optimization,
        lo_steps: raw.lo_steps,
        lo_threshold_multiplier: raw.lo_threshold_multiplier,
        lo_inner_iterations: raw.lo_inner_iterations,
        lo_sample_factor: raw.lo_sample_factor,
    })
}

/// A RANSAC summary as a dict of scalars, `termination` as its code.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `summary` - The summary.
///
/// # Returns
///
/// The summary's fields (`converged` follows from `termination`).
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn summary<'py>(py: Python<'py>, summary: &ransac::Summary) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("iterations", summary.iterations)?;
    out.set_item(
        "termination",
        encode(&Termination::ALL, summary.termination)?,
    )?;
    out.set_item("degenerate_samples", summary.degenerate_samples)?;
    out.set_item("local_optimizations", summary.local_optimizations)?;
    out.set_item("inlier_count", summary.inlier_count)?;
    out.set_item("score", summary.score)?;
    Ok(out)
}

/// A refined run's summary as a dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `summary` - The summary.
///
/// # Returns
///
/// `{"run", "refinement", "refined", "rounds"}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn refined_summary<'py>(
    py: Python<'py>,
    summary: &ransac::RefinedSummary,
) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("run", self::summary(py, &summary.run)?)?;
    let refinement = match &summary.refinement {
        Some(refinement) => Some(nls_crossing::summary(py, refinement)?),
        None => None,
    };
    out.set_item("refinement", refinement)?;
    out.set_item("refined", summary.refined)?;
    out.set_item("rounds", summary.rounds)?;
    Ok(out)
}

/// A solution's consensus and summary as a dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `consensus` - The model and its inliers.
/// * `summary` - The summary's dict.
///
/// # Returns
///
/// `{"model", "inliers", "summary"}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn result<'py, M: Item>(
    py: Python<'py>,
    consensus: &Consensus<M>,
    summary: Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("model", item_array(py, &consensus.model)?)?;
    out.set_item("inliers", consensus.inliers.clone().into_pyarray(py))?;
    out.set_item("summary", summary)?;
    Ok(out)
}

/// A RANSAC solution as a dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `solution` - The solution.
///
/// # Returns
///
/// `{"model", "inliers" (bool), "summary"}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn ransac_result<'py, M: Item>(
    py: Python<'py>,
    solution: &ransac::Solution<M>,
) -> PyResult<Bound<'py, PyDict>> {
    result(py, &solution.state, summary(py, &solution.summary)?)
}

/// A refined run's solution as a dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `solution` - The solution.
///
/// # Returns
///
/// `{"model", "inliers" (bool), "summary"}`, the summary as [`refined_summary`] gives it.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn refined_result<'py, M: Item>(
    py: Python<'py>,
    solution: &solve::Solution<Consensus<M>, ransac::RefinedSummary>,
) -> PyResult<Bound<'py, PyDict>> {
    result(py, &solution.state, refined_summary(py, &solution.summary)?)
}

/// The enum tables of a module running a refined RANSAC: RANSAC's and NLS' terminations.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"ransac_termination": [...], "nls_termination": [...]}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn refined_codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = codes(py)?;
    out.set_item("nls_termination", nls_crossing::terminations())?;
    Ok(out)
}

/// The enum table a RANSAC module's `codes()` returns.
///
/// # Arguments
///
/// * `py` - The Python token.
///
/// # Returns
///
/// `{"ransac_termination": [names in code order]}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn codes(py: Python<'_>) -> PyResult<Bound<'_, PyDict>> {
    let out = PyDict::new(py);
    out.set_item(
        "ransac_termination",
        enum_names(&Termination::ALL, Termination::name),
    )?;
    Ok(out)
}
