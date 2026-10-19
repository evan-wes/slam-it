//! Non-linear least squares on manifolds: the Rust core's Gauss-Newton and
//! Levenberg-Marquardt loop.
//!
//! Mirrors `slam_it.foundations.nls`, whose docstring states the maths: robust losses by
//! iteratively reweighted least squares, Marquardt's clamped diagonal damping with Nielsen's
//! update, and Ceres' termination tests. The loop is generic over the [`Problem`] trait, so
//! the same code drives a dense test problem and bundle adjustment's Schur system, where the
//! C++ core uses a template over a concept. It is not bound to Python itself: the backends
//! meet at the concrete solvers built on it.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use nalgebra::{DMatrix, DVector};

use crate::error::{ErrorKind, Result, fail};
use crate::solve;

/// A flag another thread sets to stop a solve before its next step or iteration (g2o's
/// `setForceStopFlag`, as ORB-SLAM's threads use it). A clone is the same flag.
#[derive(Debug, Clone, Default)]
pub struct Stop(Option<Arc<AtomicBool>>);

impl Stop {
    /// A stop that never fires.
    ///
    /// # Returns
    ///
    /// The stop.
    #[must_use]
    pub const fn never() -> Self {
        Self(None)
    }

    /// A stop that fires when `flag` is set.
    ///
    /// # Arguments
    ///
    /// * `flag` - The flag, shared with the thread that sets it.
    ///
    /// # Returns
    ///
    /// The stop.
    #[must_use]
    pub const fn on(flag: Arc<AtomicBool>) -> Self {
        Self(Some(flag))
    }

    /// Whether it fires now.
    ///
    /// # Returns
    ///
    /// Whether its flag is set; never for [`Stop::never`].
    #[must_use]
    pub fn is_set(&self) -> bool {
        self.0
            .as_ref()
            .is_some_and(|flag| flag.load(Ordering::Relaxed))
    }
}

impl PartialEq for Stop {
    /// Two stops are equal when they are the same flag, or both never fire.
    fn eq(&self, other: &Self) -> bool {
        match (&self.0, &other.0) {
            (None, None) => true,
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }
}

/// The step rule, with the integer codes it crosses the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    /// Damped steps, accepted on a sufficient ratio of actual to predicted decrease.
    LevenbergMarquardt = 0,
    /// Undamped steps, stopping at the first that fails to decrease the cost.
    GaussNewton = 1,
}

impl Method {
    /// Every method, in code order.
    pub const ALL: [Self; 2] = [Self::LevenbergMarquardt, Self::GaussNewton];

    /// The method's name, as `slam_it.foundations.nls.Options` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::LevenbergMarquardt => "levenberg_marquardt",
            Self::GaussNewton => "gauss_newton",
        }
    }
}

/// The robust losses, acting on a block's squared norm `s`, with the integer codes they
/// cross the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LossKind {
    /// `rho(s) = s`.
    Trivial = 0,
    /// `rho(s) = s` for `s <= c²`, else `2 c sqrt(s) - c²`.
    Huber = 1,
    /// `rho(s) = c² log(1 + s / c²)`.
    Cauchy = 2,
}

impl LossKind {
    /// Every loss, in code order.
    pub const ALL: [Self; 3] = [Self::Trivial, Self::Huber, Self::Cauchy];

    /// The loss's name, as `slam_it.foundations.nls.Loss` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Trivial => "trivial",
            Self::Huber => "huber",
            Self::Cauchy => "cauchy",
        }
    }
}

/// A robust loss and its scale `c`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Loss {
    /// The loss.
    pub kind: LossKind,
    /// The residual norm at which the loss starts to discount a block (positive).
    pub scale: f64,
}

impl Default for Loss {
    /// The trivial loss, scale 1.
    fn default() -> Self {
        Self {
            kind: LossKind::Trivial,
            scale: 1.0,
        }
    }
}

/// A loss evaluated at one squared norm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LossValue {
    /// `rho(s)`.
    pub rho: f64,
    /// `rho'(s)`, the block's IRLS weight, in `(0, 1]`.
    pub weight: f64,
}

/// Evaluates a loss and its derivative.
///
/// # Arguments
///
/// * `loss` - The loss (checked by [`check_options`]).
/// * `squared_norm` - The block's squared norm, `s >= 0`.
///
/// # Returns
///
/// `rho(s)` and `rho'(s)`.
#[must_use]
pub fn evaluate(loss: &Loss, squared_norm: f64) -> LossValue {
    let c2 = loss.scale * loss.scale;
    match loss.kind {
        LossKind::Huber if squared_norm > c2 => {
            let norm = squared_norm.sqrt();
            LossValue {
                rho: 2.0 * loss.scale * norm - c2,
                weight: loss.scale / norm,
            }
        }
        LossKind::Cauchy => LossValue {
            rho: c2 * (squared_norm / c2).ln_1p(),
            weight: 1.0 / (1.0 + squared_norm / c2),
        },
        LossKind::Trivial | LossKind::Huber => LossValue {
            rho: squared_norm,
            weight: 1.0,
        },
    }
}

/// A loss per block: block i uses `losses[index[i]]`. Empty means "use `Options::loss`".
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BlockLoss {
    /// The losses.
    pub losses: Vec<Loss>,
    /// Each block's loss, an index into `losses`.
    pub index: Vec<usize>,
}

/// Settings for [`solve`]; the defaults follow Ceres where it has an equivalent.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    /// The step rule.
    pub method: Method,
    /// The robust loss applied to every block.
    pub loss: Loss,
    /// A loss per block, replacing `loss` when not empty.
    pub block_loss: BlockLoss,
    /// Steps tried, accepted or not, before stopping (at least 1).
    pub max_iterations: i64,
    /// Stop when an accepted step lowers the cost by less than this fraction of it.
    pub function_tolerance: f64,
    /// Stop when the gradient's largest component is below this.
    pub gradient_tolerance: f64,
    /// Stop when the step's norm (in the tangent space) is below this.
    pub step_tolerance: f64,
    /// The first damping `mu`.
    pub initial_damping: f64,
    /// Stop, unconverged, when `mu` exceeds this.
    pub max_damping: f64,
    /// Lower clamp on `diag(JᵀWJ)` before it scales the damping.
    pub min_diagonal: f64,
    /// Upper clamp on `diag(JᵀWJ)` before it scales the damping.
    pub max_diagonal: f64,
    /// The smallest ratio of actual to predicted decrease for a step to be accepted.
    pub min_relative_decrease: f64,
    /// Fail with `NotConverged` instead of returning an unconverged estimate.
    pub strict: bool,
    /// Stops the solve before its next step when another thread sets it.
    pub stop: Stop,
}

impl Default for Options {
    /// Ceres' defaults where it has an equivalent.
    fn default() -> Self {
        Self {
            method: Method::LevenbergMarquardt,
            loss: Loss::default(),
            block_loss: BlockLoss::default(),
            max_iterations: 50,
            function_tolerance: 1e-6,
            gradient_tolerance: 1e-10,
            step_tolerance: 1e-10,
            initial_damping: 1e-4,
            max_damping: 1e32,
            min_diagonal: 1e-6,
            max_diagonal: 1e32,
            min_relative_decrease: 1e-3,
            strict: false,
            stop: Stop::never(),
        }
    }
}

/// Checks that a setting is finite and positive (or, if `allow_zero`, non-negative).
///
/// # Arguments
///
/// * `name` - The setting's name, for the message.
/// * `value` - The value.
/// * `allow_zero` - Whether zero is allowed.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming the setting.
fn check_setting(name: &str, value: f64, allow_zero: bool) -> Result<()> {
    if !value.is_finite() || value < 0.0 || (!allow_zero && value == 0.0) {
        let rule = if allow_zero {
            "must be non-negative and finite"
        } else {
            "must be positive and finite"
        };
        return fail(ErrorKind::InvalidInput, format!("{name}: {rule}"));
    }
    Ok(())
}

/// Checks a loss and the solver settings.
///
/// # Arguments
///
/// * `options` - The settings.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming the setting out of range.
pub fn check_options(options: &Options) -> Result<()> {
    check_setting("loss scale", options.loss.scale, false)?;
    for loss in &options.block_loss.losses {
        check_setting("block loss scale", loss.scale, false)?;
    }
    if options
        .block_loss
        .index
        .iter()
        .any(|&k| k >= options.block_loss.losses.len())
    {
        return fail(
            ErrorKind::InvalidInput,
            "block loss: index outside the losses",
        );
    }
    if options.max_iterations < 1 {
        return fail(
            ErrorKind::InvalidInput,
            "max_iterations: must be at least 1",
        );
    }
    for (name, value) in [
        ("initial_damping", options.initial_damping),
        ("max_damping", options.max_damping),
        ("min_diagonal", options.min_diagonal),
        ("max_diagonal", options.max_diagonal),
    ] {
        check_setting(name, value, false)?;
    }
    for (name, value) in [
        ("function_tolerance", options.function_tolerance),
        ("gradient_tolerance", options.gradient_tolerance),
        ("step_tolerance", options.step_tolerance),
        ("min_relative_decrease", options.min_relative_decrease),
    ] {
        check_setting(name, value, true)?;
    }
    if options.min_diagonal > options.max_diagonal {
        return fail(
            ErrorKind::InvalidInput,
            "min_diagonal: must not exceed max_diagonal",
        );
    }
    Ok(())
}

/// Why [`solve`] stopped, with the integer codes it crosses the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Termination {
    /// Converged: the gradient is below `gradient_tolerance`.
    Gradient = 0,
    /// Converged: the last accepted step barely lowered the cost.
    Function = 1,
    /// Converged: the step is below `step_tolerance`.
    Step = 2,
    /// Not converged: `max_iterations` steps were tried.
    MaxIterations = 3,
    /// Not converged: the damping exceeded `max_damping`.
    DampingLimit = 4,
    /// Not converged: a Gauss-Newton step failed to lower the cost.
    NoDecrease = 5,
    /// Not converged: the undamped Gauss-Newton system is not positive definite.
    Singular = 6,
    /// Not converged: stopped from outside, between steps (`Options::stop`).
    Stopped = 7,
}

impl Termination {
    /// Every termination, in code order.
    pub const ALL: [Self; 8] = [
        Self::Gradient,
        Self::Function,
        Self::Step,
        Self::MaxIterations,
        Self::DampingLimit,
        Self::NoDecrease,
        Self::Singular,
        Self::Stopped,
    ];

    /// The termination's name, as `slam_it.foundations.nls.Termination` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Gradient => "gradient",
            Self::Function => "function",
            Self::Step => "step",
            Self::MaxIterations => "max_iterations",
            Self::DampingLimit => "damping_limit",
            Self::NoDecrease => "no_decrease",
            Self::Singular => "singular",
            Self::Stopped => "stopped",
        }
    }

    /// Whether the termination counts as converged.
    ///
    /// # Returns
    ///
    /// True for the gradient, function and step tests.
    #[must_use]
    pub const fn is_converged(self) -> bool {
        matches!(self, Self::Gradient | Self::Function | Self::Step)
    }
}

/// One step tried by [`solve`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Iteration {
    /// The step's index, from 1.
    pub iteration: usize,
    /// The cost before the step.
    pub cost: f64,
    /// The gradient's largest component, before the step.
    pub gradient_norm: f64,
    /// The step's norm (NaN when the system could not be factored).
    pub step_norm: f64,
    /// The damping for this step (0 for Gauss-Newton).
    pub damping: f64,
    /// Actual over predicted cost decrease (NaN when not computed).
    pub relative_decrease: f64,
    /// Whether the step was taken.
    pub accepted: bool,
}

/// How a solve went.
#[derive(Debug, Clone, PartialEq)]
pub struct Summary {
    /// The cost at the start.
    pub initial_cost: f64,
    /// The cost at the returned state.
    pub final_cost: f64,
    /// Why the solve stopped.
    pub termination: Termination,
    /// Every step tried, in order.
    pub trace: Vec<Iteration>,
}

impl solve::Summary for Summary {
    type Termination = Termination;

    fn iterations(&self) -> usize {
        self.trace.len()
    }

    fn converged(&self) -> bool {
        self.termination.is_converged()
    }

    fn termination(&self) -> Termination {
        self.termination
    }
}

/// The lowest-cost state visited, and how the solve went.
pub type Solution<State> = solve::Solution<State, Summary>;

/// A linearisation's normal equations, `(H + diag(d)) δ = -g`.
pub trait LinearSystem {
    /// The weighted gradient `g = JᵀWr`.
    ///
    /// # Returns
    ///
    /// The gradient.
    fn gradient(&self) -> &DVector<f64>;

    /// The diagonal of `H = JᵀWJ`.
    ///
    /// # Returns
    ///
    /// The diagonal.
    fn hessian_diagonal(&self) -> DVector<f64>;

    /// Solves `(H + diag(damping)) δ = -g`.
    ///
    /// # Arguments
    ///
    /// * `damping` - Added to H's diagonal.
    ///
    /// # Returns
    ///
    /// The step, or `None` if the damped system is not positive definite.
    fn solve(&self, damping: &DVector<f64>) -> Option<DVector<f64>>;
}

/// A least-squares problem: block squared norms, a linearisation, a retraction.
pub trait Problem {
    /// The state being estimated.
    type State;
    /// The linearisation's normal equations.
    type System: LinearSystem;

    /// The blocks' squared norms at a state.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    ///
    /// # Returns
    ///
    /// One squared norm per residual block (infinite where a block cannot be evaluated).
    fn squared_norms(&self, state: &Self::State) -> DVector<f64>;

    /// The weighted normal equations at a state.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `weights` - The blocks' IRLS weights.
    ///
    /// # Returns
    ///
    /// The system.
    fn linearize(&self, state: &Self::State, weights: &DVector<f64>) -> Self::System;

    /// Applies a tangent step to a state.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `delta` - The step.
    ///
    /// # Returns
    ///
    /// `state ⊕ δ`.
    fn retract(&self, state: &Self::State, delta: &DVector<f64>) -> Self::State;
}

/// Normal equations formed densely from a full Jacobian, factored by Cholesky.
#[derive(Debug, Clone)]
pub struct DenseSystem {
    /// `H = JᵀWJ`.
    hessian: DMatrix<f64>,
    /// `g = JᵀWr`.
    gradient: DVector<f64>,
}

impl DenseSystem {
    /// Forms the weighted normal equations of M blocks of d rows over n parameters.
    ///
    /// # Arguments
    ///
    /// * `jacobian` - The (M d) x n Jacobian, block by block.
    /// * `residuals` - The M d residuals, block by block.
    /// * `weights` - The M block weights.
    ///
    /// # Returns
    ///
    /// The system.
    ///
    /// # Errors
    ///
    /// `InvalidInput` if the sizes disagree.
    pub fn build(
        jacobian: &DMatrix<f64>,
        residuals: &DVector<f64>,
        weights: &DVector<f64>,
    ) -> Result<Self> {
        let blocks = weights.len();
        if jacobian.nrows() != residuals.len()
            || blocks == 0
            || !residuals.len().is_multiple_of(blocks)
        {
            return fail(
                ErrorKind::InvalidInput,
                "jacobian, residuals, weights: expected (M d) x n, M d and M entries",
            );
        }
        let block_size = residuals.len() / blocks;
        let mut weighted = jacobian.clone();
        for (r, mut row) in weighted.row_iter_mut().enumerate() {
            row *= weights[r / block_size];
        }
        Ok(Self {
            hessian: weighted.tr_mul(jacobian),
            gradient: weighted.tr_mul(residuals),
        })
    }
}

impl LinearSystem for DenseSystem {
    fn gradient(&self) -> &DVector<f64> {
        &self.gradient
    }

    fn hessian_diagonal(&self) -> DVector<f64> {
        self.hessian.diagonal()
    }

    fn solve(&self, damping: &DVector<f64>) -> Option<DVector<f64>> {
        let mut system = self.hessian.clone();
        system.set_diagonal(&(system.diagonal() + damping));
        let factor = system.cholesky()?;
        Some(-factor.solve(&self.gradient))
    }
}

/// The cost `½ Σ rho(s_i)` and the IRLS weights, or an infinite cost if any block is not
/// finite.
#[derive(Debug, Clone)]
pub(crate) struct Evaluation {
    /// The cost.
    pub(crate) cost: f64,
    /// The blocks' weights `rho'(s_i)`.
    pub(crate) weights: DVector<f64>,
}

/// Evaluates the cost and weights of a set of squared norms.
///
/// # Arguments
///
/// * `options` - The settings: their per-block loss if given, else their uniform loss.
/// * `squared_norms` - The blocks' squared norms.
///
/// # Returns
///
/// The evaluation.
pub(crate) fn evaluate_blocks(options: &Options, squared_norms: &DVector<f64>) -> Evaluation {
    let per_block = !options.block_loss.losses.is_empty();
    let mut weights = DVector::zeros(squared_norms.len());
    let mut cost = 0.0;
    for (i, &s) in squared_norms.iter().enumerate() {
        if !s.is_finite() {
            return Evaluation {
                cost: f64::INFINITY,
                weights,
            };
        }
        let loss = if per_block {
            &options.block_loss.losses[options.block_loss.index[i]]
        } else {
            &options.loss
        };
        let value = evaluate(loss, s);
        cost += value.rho;
        weights[i] = value.weight;
    }
    Evaluation {
        cost: 0.5 * cost,
        weights,
    }
}

/// Nielsen's nu: the factor mu first grows by on a rejected step, itself doubled per
/// rejection.
const DAMPING_GROWTH: f64 = 2.0;
/// The most an accepted step divides mu by (Nielsen 1999; Madsen, Nielsen & Tingleff 2004).
const DAMPING_MAX_SHRINK: f64 = 3.0;

/// The Levenberg-Marquardt damping mu and Nielsen's growth factor nu.
#[derive(Debug, Clone, Copy)]
struct Damping {
    /// The damping mu.
    mu: f64,
    /// The factor mu grows by on the next rejection.
    growth: f64,
}

impl Damping {
    /// Starts at damping `mu`.
    ///
    /// # Arguments
    ///
    /// * `mu` - The first damping (0 for Gauss-Newton).
    ///
    /// # Returns
    ///
    /// The damping.
    const fn new(mu: f64) -> Self {
        Self {
            mu,
            growth: DAMPING_GROWTH,
        }
    }

    /// Shrinks mu by the gain ratio of an accepted step; resets nu.
    ///
    /// # Arguments
    ///
    /// * `ratio` - The step's gain ratio.
    fn accept(&mut self, ratio: f64) {
        let centred = 2.0 * ratio - 1.0;
        self.mu *= (1.0 - centred * centred * centred).max(1.0 / DAMPING_MAX_SHRINK);
        self.growth = DAMPING_GROWTH;
    }

    /// Grows mu by nu after a rejected step, and doubles nu.
    fn reject(&mut self) {
        self.mu *= self.growth;
        self.growth *= DAMPING_GROWTH;
    }
}

/// A step tried: its record, and the candidate state with its evaluation (if any).
struct Trial<State> {
    /// The step's trace record.
    record: Iteration,
    /// The state the step leads to, with its evaluation; `None` when the system could not
    /// be factored.
    candidate: Option<(State, Evaluation)>,
}

/// What one step attempt produced.
enum Attempt<State> {
    /// A step was computed and judged.
    Tried(Trial<State>),
    /// A convergence test stopped the solve before a step was needed.
    Stop(Termination),
}

/// The largest absolute entry, NaN if any entry is NaN, 0 for an empty vector.
///
/// `f64::max` drops NaN, so a plain fold would let a corrupted linearisation pass as
/// converged; numpy's `max`, the reference, keeps it.
///
/// # Arguments
///
/// * `vector` - The vector.
///
/// # Returns
///
/// The norm.
fn max_abs(vector: &DVector<f64>) -> f64 {
    vector.iter().fold(0.0_f64, |m, g| {
        if m.is_nan() || g.is_nan() {
            f64::NAN
        } else {
            m.max(g.abs())
        }
    })
}

/// Computes, applies and judges one step; or returns the convergence test it stops on.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `state` - The current state.
/// * `current` - The current state's evaluation.
/// * `system` - The current linearisation.
/// * `damping` - The damping `mu` (0 for Gauss-Newton).
/// * `number` - The step's index, from 1.
/// * `options` - The settings.
///
/// # Returns
///
/// The trial, or `Gradient`/`Step` if a convergence test stops the solve first.
fn try_step<P: Problem>(
    problem: &P,
    state: &P::State,
    current: &Evaluation,
    system: &P::System,
    damping: f64,
    number: usize,
    options: &Options,
) -> Attempt<P::State> {
    let gradient = system.gradient();
    let gradient_norm = max_abs(gradient);
    if gradient_norm <= options.gradient_tolerance {
        return Attempt::Stop(Termination::Gradient);
    }
    let diagonal = system
        .hessian_diagonal()
        .map(|d| d.clamp(options.min_diagonal, options.max_diagonal))
        * damping;
    let record = |step_norm, relative_decrease, accepted| Iteration {
        iteration: number,
        cost: current.cost,
        gradient_norm,
        step_norm,
        damping,
        relative_decrease,
        accepted,
    };
    let Some(delta) = system.solve(&diagonal) else {
        return Attempt::Tried(Trial {
            record: record(f64::NAN, f64::NAN, false),
            candidate: None,
        });
    };
    let step_norm = delta.norm();
    if step_norm <= options.step_tolerance {
        return Attempt::Stop(Termination::Step);
    }
    let candidate = problem.retract(state, &delta);
    let evaluation = evaluate_blocks(options, &problem.squared_norms(&candidate));
    let predicted = 0.5 * (delta.dot(&diagonal.component_mul(&delta)) - gradient.dot(&delta));
    let ratio = if predicted > 0.0 {
        (current.cost - evaluation.cost) / predicted
    } else {
        f64::NAN
    };
    let accepted = evaluation.cost.is_finite()
        && evaluation.cost < current.cost
        && (options.method == Method::GaussNewton || ratio > options.min_relative_decrease);
    Attempt::Tried(Trial {
        record: record(step_norm, ratio, accepted),
        candidate: Some((candidate, evaluation)),
    })
}

/// Minimises `½ Σ rho(|r_i|²)` from `initial` by Levenberg-Marquardt or Gauss-Newton.
///
/// # Arguments
///
/// * `problem` - The residual blocks, their linearisation and the state's retraction.
/// * `initial` - The starting state.
/// * `options` - Method, loss, tolerances and limits.
///
/// # Returns
///
/// The lowest-cost state visited, with the summary.
///
/// # Errors
///
/// `InvalidInput` for bad options, a per-block loss of the wrong length, or a non-finite
/// initial cost; `NotConverged` if `options.strict` and the solve stopped without
/// converging.
pub fn solve<P: Problem>(
    problem: &P,
    initial: P::State,
    options: &Options,
) -> Result<Solution<P::State>> {
    check_options(options)?;
    let levenberg = options.method == Method::LevenbergMarquardt;
    let mut state = initial;
    let initial_norms = problem.squared_norms(&state);
    if !options.block_loss.losses.is_empty()
        && options.block_loss.index.len() != initial_norms.len()
    {
        return fail(
            ErrorKind::InvalidInput,
            "block loss: index length differs from the blocks",
        );
    }
    let mut current = evaluate_blocks(options, &initial_norms);
    if !current.cost.is_finite() {
        return fail(
            ErrorKind::InvalidInput,
            "solve: the initial cost is not finite",
        );
    }
    let initial_cost = current.cost;
    let mut damping = Damping::new(if levenberg {
        options.initial_damping
    } else {
        0.0
    });
    let mut system = problem.linearize(&state, &current.weights);
    let mut trace: Vec<Iteration> = Vec::new();
    let mut termination = Termination::MaxIterations;
    let max_iterations = usize::try_from(options.max_iterations).unwrap_or(usize::MAX);
    while trace.len() < max_iterations {
        if options.stop.is_set() {
            termination = Termination::Stopped;
            break;
        }
        let trial = match try_step(
            problem,
            &state,
            &current,
            &system,
            damping.mu,
            trace.len() + 1,
            options,
        ) {
            Attempt::Stop(stop) => {
                termination = stop;
                break;
            }
            Attempt::Tried(trial) => trial,
        };
        trace.push(trial.record);
        match trial.candidate {
            Some((candidate, evaluation)) if trial.record.accepted => {
                let previous = current.cost;
                state = candidate;
                current = evaluation;
                if levenberg {
                    damping.accept(trial.record.relative_decrease);
                }
                if previous - current.cost <= options.function_tolerance * previous {
                    termination = Termination::Function;
                    break;
                }
                system = problem.linearize(&state, &current.weights);
            }
            candidate => {
                if !levenberg {
                    termination = if candidate.is_some() {
                        Termination::NoDecrease
                    } else {
                        Termination::Singular
                    };
                    break;
                }
                damping.reject();
                if damping.mu > options.max_damping {
                    termination = Termination::DampingLimit;
                    break;
                }
            }
        }
    }
    if options.strict && !termination.is_converged() {
        return fail(
            ErrorKind::NotConverged,
            format!("solve: stopped without converging ({})", termination.name()),
        );
    }
    Ok(Solution {
        state,
        summary: Summary {
            initial_cost,
            final_cost: current.cost,
            termination,
            trace,
        },
    })
}

/// A robust refinement: [`solve`] with the default settings, a Cauchy loss at `scale` and an
/// iteration cap, not strict (an unconverged solve returns its best state).
///
/// # Arguments
///
/// * `problem` - The residual blocks, their linearisation and the state's retraction.
/// * `initial` - The starting state.
/// * `scale` - The Cauchy loss's scale (positive).
/// * `max_iterations` - The iteration cap.
///
/// # Returns
///
/// The lowest-cost state visited, with the summary.
///
/// # Errors
///
/// `InvalidInput` for a bad scale or a non-finite initial cost.
pub fn solve_cauchy<P: Problem>(
    problem: &P,
    initial: P::State,
    scale: f64,
    max_iterations: i64,
) -> Result<Solution<P::State>> {
    let options = Options {
        loss: Loss {
            kind: LossKind::Cauchy,
            scale,
        },
        max_iterations,
        ..Options::default()
    };
    solve(problem, initial, &options)
}

#[cfg(test)]
mod tests {
    //! Native tests, runnable without Python, mirroring the C++ core's: the losses, both
    //! methods on Rosenbrock (including an overshoot and a singular system), robust line
    //! fitting, robust rotation averaging on SO(3), strict mode, and the settings checks.
    #![allow(
        clippy::float_cmp,
        reason = "the losses' values and an untouched parameter are exact"
    )]

    use std::f64::consts::LN_2;

    use nalgebra::{Matrix3, Vector2, Vector3};

    use super::*;
    use crate::so3;

    /// Rosenbrock's function as two residual blocks, plus parameters it ignores.
    struct Rosenbrock {
        /// How many parameters the residuals ignore (making the undamped system singular).
        unused: usize,
    }

    impl Rosenbrock {
        /// The residuals `(10 (y - x²), 1 - x)`.
        fn residuals(state: &DVector<f64>) -> DVector<f64> {
            DVector::from_vec(vec![
                10.0 * (state[1] - state[0] * state[0]),
                1.0 - state[0],
            ])
        }
    }

    impl Problem for Rosenbrock {
        type State = DVector<f64>;
        type System = DenseSystem;

        fn squared_norms(&self, state: &DVector<f64>) -> DVector<f64> {
            Self::residuals(state).map(|r| r * r)
        }

        fn linearize(&self, state: &DVector<f64>, weights: &DVector<f64>) -> DenseSystem {
            let mut jacobian = DMatrix::zeros(2, 2 + self.unused);
            jacobian[(0, 0)] = -20.0 * state[0];
            jacobian[(0, 1)] = 10.0;
            jacobian[(1, 0)] = -1.0;
            DenseSystem::build(&jacobian, &Self::residuals(state), weights).unwrap()
        }

        fn retract(&self, state: &DVector<f64>, delta: &DVector<f64>) -> DVector<f64> {
            state + delta
        }
    }

    /// Fits `y = a x + b` to points, one block per point.
    struct LineFit {
        /// The abscissae.
        xs: DVector<f64>,
        /// The ordinates.
        ys: DVector<f64>,
    }

    impl LineFit {
        /// The residuals `a x_i + b - y_i`.
        fn residuals(&self, state: &Vector2<f64>) -> DVector<f64> {
            self.xs.map(|x| state[0] * x + state[1]) - &self.ys
        }
    }

    impl Problem for LineFit {
        type State = Vector2<f64>;
        type System = DenseSystem;

        fn squared_norms(&self, state: &Vector2<f64>) -> DVector<f64> {
            self.residuals(state).map(|r| r * r)
        }

        fn linearize(&self, state: &Vector2<f64>, weights: &DVector<f64>) -> DenseSystem {
            let mut jacobian = DMatrix::from_element(self.xs.len(), 2, 1.0);
            jacobian.set_column(0, &self.xs);
            DenseSystem::build(&jacobian, &self.residuals(state), weights).unwrap()
        }

        fn retract(&self, state: &Vector2<f64>, delta: &DVector<f64>) -> Vector2<f64> {
            state + Vector2::new(delta[0], delta[1])
        }
    }

    /// Averages rotations on SO(3): blocks `log(Rᵀ R_i)`, with `R ⊕ δ = R exp(δ)`.
    struct RotationAverage {
        /// The measured rotations.
        measured: Vec<Matrix3<f64>>,
    }

    impl RotationAverage {
        /// One block's residual, `log(Rᵀ R_i)`.
        fn residual(&self, state: &Matrix3<f64>, i: usize) -> Vector3<f64> {
            so3::log(&(state.transpose() * self.measured[i])).unwrap()
        }
    }

    impl Problem for RotationAverage {
        type State = Matrix3<f64>;
        type System = DenseSystem;

        fn squared_norms(&self, state: &Matrix3<f64>) -> DVector<f64> {
            DVector::from_iterator(
                self.measured.len(),
                (0..self.measured.len()).map(|i| self.residual(state, i).norm_squared()),
            )
        }

        fn linearize(&self, state: &Matrix3<f64>, weights: &DVector<f64>) -> DenseSystem {
            let rows = 3 * self.measured.len();
            let mut jacobian = DMatrix::zeros(rows, 3);
            let mut residuals = DVector::zeros(rows);
            for i in 0..self.measured.len() {
                let r = self.residual(state, i);
                residuals.fixed_rows_mut::<3>(3 * i).copy_from(&r);
                let block = -so3::left_jacobian_inverse(&r).unwrap();
                jacobian.fixed_view_mut::<3, 3>(3 * i, 0).copy_from(&block);
            }
            DenseSystem::build(&jacobian, &residuals, weights).unwrap()
        }

        fn retract(&self, state: &Matrix3<f64>, delta: &DVector<f64>) -> Matrix3<f64> {
            state * so3::exp(&Vector3::new(delta[0], delta[1], delta[2]))
        }
    }

    /// A line `y = 2 x + 1` with small deterministic noise and three gross outliers.
    fn line_with_outliers() -> LineFit {
        let xs = DVector::from_iterator(30, (0..30).map(|i| 0.1 * f64::from(i)));
        let mut ys = DVector::from_iterator(
            30,
            (0..30).map(|i| 2.0 * 0.1 * f64::from(i) + 1.0 + 0.01 * (1.7 * f64::from(i)).sin()),
        );
        ys[4] += 8.0;
        ys[17] -= 6.0;
        ys[25] += 9.0;
        LineFit { xs, ys }
    }

    /// Tight tolerances, so a solve stops on the gradient or the step, not the cost.
    fn tight(method: Method, loss: Loss) -> Options {
        Options {
            method,
            loss,
            max_iterations: 200,
            function_tolerance: 0.0,
            gradient_tolerance: 1e-12,
            step_tolerance: 1e-14,
            ..Options::default()
        }
    }

    /// Cauchy at scale `c`.
    const fn cauchy(scale: f64) -> Loss {
        Loss {
            kind: LossKind::Cauchy,
            scale,
        }
    }

    /// The losses and their weights, at an inlier and an outlier.
    #[test]
    fn losses() {
        let huber = Loss {
            kind: LossKind::Huber,
            scale: 1.0,
        };
        assert_eq!(
            evaluate(&huber, 0.25),
            LossValue {
                rho: 0.25,
                weight: 1.0
            }
        );
        assert_eq!(
            evaluate(&huber, 4.0),
            LossValue {
                rho: 3.0,
                weight: 0.5
            }
        );
        let value = evaluate(&cauchy(2.0), 4.0);
        assert!((value.rho - 4.0 * LN_2).abs() < 1e-15);
        assert_eq!(value.weight, 0.5);
    }

    /// From the classic start, Levenberg-Marquardt reaches Rosenbrock's minimum at (1, 1).
    /// The undamped Gauss-Newton step overshoots, raising the cost, so Gauss-Newton stops
    /// there; from nearer, it converges.
    #[test]
    fn rosenbrock() {
        let start = DVector::from_vec(vec![-1.2, 1.0]);
        let at_minimum = |solved: Solution<DVector<f64>>| {
            solve::Summary::converged(&solved.summary)
                && (solved.state - DVector::from_vec(vec![1.0, 1.0])).norm() < 1e-10
                && solved.summary.final_cost < 1e-20
        };
        let problem = Rosenbrock { unused: 0 };
        let lm = tight(Method::LevenbergMarquardt, Loss::default());
        let gn = tight(Method::GaussNewton, Loss::default());
        assert!(at_minimum(solve(&problem, start.clone(), &lm).unwrap()));
        let overshoot = solve(&problem, start.clone(), &gn).unwrap();
        assert_eq!(overshoot.summary.termination, Termination::NoDecrease);
        assert_eq!(overshoot.state, start);
        assert_eq!(overshoot.summary.trace.len(), 1);
        let near = DVector::from_vec(vec![0.5, 0.5]);
        assert!(at_minimum(solve(&problem, near, &gn).unwrap()));
    }

    /// A stop set ends the solve before its next step: none taken, not converged.
    #[test]
    fn a_stop_ends_the_solve_between_steps() {
        let start = DVector::from_vec(vec![-1.2, 1.0]);
        let problem = Rosenbrock { unused: 0 };
        let stopping = Options {
            stop: Stop::on(Arc::new(AtomicBool::new(true))),
            ..tight(Method::LevenbergMarquardt, Loss::default())
        };
        let stopped = solve(&problem, start.clone(), &stopping).unwrap();
        assert_eq!(
            (stopped.summary.trace.len(), stopped.summary.termination),
            (0, Termination::Stopped)
        );
        assert_eq!(stopped.state, start);
        assert_eq!(Termination::Stopped.name(), "stopped");
    }

    /// A parameter the residuals ignore leaves the undamped system singular; damping copes.
    #[test]
    fn unobserved_parameter() {
        let padded = DVector::from_vec(vec![-1.2, 1.0, 0.5]);
        let problem = Rosenbrock { unused: 1 };
        let gn = solve(
            &problem,
            padded.clone(),
            &tight(Method::GaussNewton, Loss::default()),
        );
        assert_eq!(gn.unwrap().summary.termination, Termination::Singular);
        let lm = tight(Method::LevenbergMarquardt, Loss::default());
        let damped = solve(&problem, padded, &lm).unwrap();
        assert!(damped.summary.termination.is_converged());
        assert!((damped.state.rows(0, 2) - Vector2::new(1.0, 1.0)).norm() < 1e-10);
        assert_eq!(damped.state[2], 0.5);
    }

    /// Robust losses resist the outliers that drag the plain least-squares line; a per-block
    /// loss naming one loss everywhere reproduces the uniform solve exactly.
    #[test]
    fn robust_line() {
        let fit = line_with_outliers();
        let truth = Vector2::new(2.0, 1.0);
        let lm = |loss| tight(Method::LevenbergMarquardt, loss);
        let plain = solve(&fit, Vector2::zeros(), &lm(Loss::default())).unwrap();
        assert!((plain.state - truth).norm() > 0.3);
        let robust = solve(&fit, Vector2::zeros(), &lm(cauchy(0.05))).unwrap();
        assert!(robust.summary.termination.is_converged());
        assert!((robust.state - truth).norm() < 0.02);
        let mut per_block = lm(Loss::default());
        per_block.block_loss = BlockLoss {
            losses: vec![Loss::default(), cauchy(0.05)],
            index: vec![1; 30],
        };
        let blockwise = solve(&fit, Vector2::zeros(), &per_block).unwrap();
        assert_eq!(blockwise.state, robust.state);
        per_block.block_loss.index[3] = 2;
        let outside = solve(&fit, Vector2::zeros(), &per_block).unwrap_err();
        assert_eq!(outside.kind, ErrorKind::InvalidInput);
    }

    /// The robust mean of noisy rotations with two outliers, on SO(3).
    #[test]
    fn robust_rotation_average() {
        let rotation = so3::exp(&Vector3::new(0.3, -0.2, 0.5));
        let mut measured: Vec<_> = (0..12)
            .map(|i| {
                let i = f64::from(i);
                let noise = Vector3::new((i).sin(), (2.0 * i).cos(), (3.0 * i).sin()) * 0.01;
                rotation * so3::exp(&noise)
            })
            .collect();
        measured.push(rotation * so3::exp(&Vector3::new(0.9, 0.0, 0.0)));
        measured.push(rotation * so3::exp(&Vector3::new(0.0, -1.1, 0.4)));
        let problem = RotationAverage { measured };
        let options = tight(Method::LevenbergMarquardt, cauchy(0.05));
        let mean = solve(&problem, Matrix3::identity(), &options).unwrap();
        assert!(mean.summary.termination.is_converged());
        let error = so3::log(&(rotation.transpose() * mean.state)).unwrap();
        assert!(error.norm() < 0.01);
        assert!((mean.state.transpose() * mean.state - Matrix3::identity()).norm() < 1e-12);
    }

    /// A Rosenbrock whose Jacobian carries a NaN in a parameter the residuals ignore.
    struct NanJacobian;

    impl Problem for NanJacobian {
        type State = DVector<f64>;
        type System = DenseSystem;

        fn squared_norms(&self, state: &DVector<f64>) -> DVector<f64> {
            Rosenbrock::residuals(state).map(|r| r * r)
        }

        fn linearize(&self, state: &DVector<f64>, weights: &DVector<f64>) -> DenseSystem {
            let mut jacobian = DMatrix::zeros(2, 3);
            jacobian[(0, 0)] = -20.0 * state[0];
            jacobian[(0, 1)] = 10.0;
            jacobian[(1, 0)] = -1.0;
            jacobian[(0, 2)] = f64::NAN;
            DenseSystem::build(&jacobian, &Rosenbrock::residuals(state), weights).unwrap()
        }

        fn retract(&self, state: &DVector<f64>, delta: &DVector<f64>) -> DVector<f64> {
            state + delta
        }
    }

    /// A NaN gradient entry is never read as convergence, even at the minimum where every
    /// other entry is zero.
    #[test]
    fn nan_gradient_does_not_converge() {
        assert!(max_abs(&DVector::from_vec(vec![f64::NAN, 0.0, 1e-12])).is_nan());
        assert!(max_abs(&DVector::zeros(0)).abs() < f64::MIN_POSITIVE);
        let at_minimum = DVector::from_vec(vec![1.0, 1.0, 0.0]);
        for method in [Method::LevenbergMarquardt, Method::GaussNewton] {
            let solved = solve(
                &NanJacobian,
                at_minimum.clone(),
                &tight(method, Loss::default()),
            );
            assert!(
                !solved.unwrap().summary.termination.is_converged(),
                "{method:?}"
            );
        }
    }

    /// Strict mode turns an unconverged stop into an error; bad settings are rejected.
    #[test]
    fn strict_mode_and_settings() {
        let start = DVector::from_vec(vec![-1.2, 1.0]);
        let problem = Rosenbrock { unused: 0 };
        let one_step = Options {
            max_iterations: 1,
            strict: true,
            ..tight(Method::LevenbergMarquardt, Loss::default())
        };
        let strict = solve(&problem, start.clone(), &one_step).unwrap_err();
        assert_eq!(strict.kind, ErrorKind::NotConverged);
        let bad = Options {
            initial_damping: -1.0,
            ..Options::default()
        };
        assert_eq!(
            solve(&problem, start, &bad).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let mismatched = DenseSystem::build(
            &DMatrix::zeros(3, 2),
            &DVector::zeros(3),
            &DVector::from_element(2, 1.0),
        );
        assert_eq!(mismatched.unwrap_err().kind, ErrorKind::InvalidInput);
    }

    /// Each crossing enum's discriminant is its position in `ALL`, the code it crosses as.
    #[test]
    fn codes_are_positions() {
        for (i, method) in Method::ALL.iter().enumerate() {
            assert_eq!(*method as usize, i, "{}", method.name());
        }
        for (i, kind) in LossKind::ALL.iter().enumerate() {
            assert_eq!(*kind as usize, i, "{}", kind.name());
        }
        for (i, termination) in Termination::ALL.iter().enumerate() {
            assert_eq!(*termination as usize, i, "{}", termination.name());
        }
    }
}
