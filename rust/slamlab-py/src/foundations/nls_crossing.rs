//! Shared by every binding that runs `nls::solve`: its options in, its summary out.
//!
//! A summary crosses as a dict: `initial_cost`, `final_cost`, `termination` (its index in
//! `nls_termination`'s names), `trace` (one row per step) and `trace_columns` (the
//! `nls::Iteration` field each column holds, which the Python side reads by name). Options
//! cross as `nls.Options`' fields, read strictly by serde: a missing or unknown field is an
//! error, at every depth.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Deserialize;
use serde::de::IgnoredAny;
use slamlab_foundations::nls::{self, Iteration, LossKind, Method, Termination};

use crate::common::{by_name, enum_names, new_array};

/// The trace's column names (`nls::Iteration`'s fields), in row order.
const TRACE_COLUMNS: [&str; 7] = [
    "iteration",
    "cost",
    "gradient_norm",
    "step_norm",
    "damping",
    "relative_decrease",
    "accepted",
];

/// `nls.Loss`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LossIn {
    /// The loss, by name.
    kind: String,
    /// Its scale.
    scale: f64,
}

impl LossIn {
    /// The core's loss.
    ///
    /// # Arguments
    ///
    /// * `what` - The field's name, for the message.
    ///
    /// # Returns
    ///
    /// The loss (checked by the core).
    ///
    /// # Errors
    ///
    /// `ValueError` for an unknown kind.
    pub fn read(&self, what: &str) -> PyResult<nls::Loss> {
        Ok(nls::Loss {
            kind: by_name(&LossKind::ALL, LossKind::name, &self.kind, what)?,
            scale: self.scale,
        })
    }
}

/// `nls.Options`' fields, as they cross.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OptionsIn {
    /// The step rule, by name.
    method: String,
    /// The loss.
    loss: LossIn,
    /// A per-block loss: set by the solver that owns the blocks, so it must be `None`.
    block_loss: Option<IgnoredAny>,
    /// Steps tried before stopping.
    max_iterations: i64,
    /// The relative cost decrease that converges.
    function_tolerance: f64,
    /// The gradient size that converges.
    gradient_tolerance: f64,
    /// The step size that converges.
    step_tolerance: f64,
    /// The first damping.
    initial_damping: f64,
    /// The damping that stops the solve.
    max_damping: f64,
    /// The lower clamp on the damped diagonal.
    min_diagonal: f64,
    /// The upper clamp on the damped diagonal.
    max_diagonal: f64,
    /// The gain ratio a step must exceed.
    min_relative_decrease: f64,
    /// Whether an unconverged solve is an error.
    strict: bool,
}

impl OptionsIn {
    /// The core's options, with no per-block loss.
    ///
    /// # Returns
    ///
    /// The options (checked by the core).
    ///
    /// # Errors
    ///
    /// `ValueError` for an unknown name, or a `block_loss` set by the caller.
    pub fn read(&self) -> PyResult<nls::Options> {
        if self.block_loss.is_some() {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "solver.block_loss: set by the solver that owns the blocks, not the caller",
            ));
        }
        Ok(nls::Options {
            method: by_name(&Method::ALL, Method::name, &self.method, "method")?,
            loss: self.loss.read("loss")?,
            block_loss: nls::BlockLoss::default(),
            max_iterations: self.max_iterations,
            function_tolerance: self.function_tolerance,
            gradient_tolerance: self.gradient_tolerance,
            step_tolerance: self.step_tolerance,
            initial_damping: self.initial_damping,
            max_damping: self.max_damping,
            min_diagonal: self.min_diagonal,
            max_diagonal: self.max_diagonal,
            min_relative_decrease: self.min_relative_decrease,
            strict: self.strict,
        })
    }
}

/// One trace record as a row of the trace array.
///
/// # Arguments
///
/// * `it` - The record.
///
/// # Returns
///
/// The row, in [`TRACE_COLUMNS`] order.
#[allow(
    clippy::cast_precision_loss,
    reason = "an iteration index is far below 2^52"
)]
fn trace_row(it: &Iteration) -> [f64; 7] {
    [
        it.iteration as f64,
        it.cost,
        it.gradient_norm,
        it.step_norm,
        it.damping,
        it.relative_decrease,
        if it.accepted { 1.0 } else { 0.0 },
    ]
}

/// A solve's summary as a dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `summary` - The summary.
///
/// # Returns
///
/// `{"initial_cost", "final_cost", "termination", "trace", "trace_columns"}`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
pub fn summary<'py>(py: Python<'py>, summary: &nls::Summary) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("initial_cost", summary.initial_cost)?;
    out.set_item("final_cost", summary.final_cost)?;
    out.set_item("termination", summary.termination as i64)?;
    let rows: Vec<f64> = summary.trace.iter().flat_map(trace_row).collect();
    out.set_item(
        "trace",
        new_array(py, &[summary.trace.len(), TRACE_COLUMNS.len()], rows)?,
    )?;
    out.set_item("trace_columns", TRACE_COLUMNS.to_vec())?;
    Ok(out)
}

/// `nls::Termination`'s names in code order.
///
/// # Returns
///
/// The names.
pub fn terminations() -> Vec<&'static str> {
    enum_names(&Termination::ALL, Termination::name)
}
