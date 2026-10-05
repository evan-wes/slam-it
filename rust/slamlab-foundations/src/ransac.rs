//! RANSAC: the Rust core's LO-MSAC loop.
//!
//! Mirrors `slamlab.foundations.ransac`, whose docstring states the method: MSAC scoring,
//! the adaptive stopping bound, and Lebeda et al.'s local optimisation (iterative least
//! squares, then an inner RANSAC of non-minimal samples), followed by the optional
//! refine-and-reselect rounds. The loop is generic over the [`Estimator`] trait, where the C++
//! core uses a template over a concept. It is not bound to Python itself: the backends meet at
//! the concrete estimators built on it. Samples come from [`Pcg32`], so a seed draws the same
//! samples as the Python and C++ cores.

use nalgebra::DVector;

use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::nls;
use crate::rng::Pcg32;
use crate::solve;

/// A model RANSAC can fit: its minimal and least-squares solvers and its residuals.
pub trait Estimator {
    /// The model fitted.
    type Model: Clone;

    /// The minimal sample size.
    ///
    /// # Returns
    ///
    /// The size.
    fn sample_size(&self) -> usize;

    /// The number of data.
    ///
    /// # Returns
    ///
    /// The count.
    fn size(&self) -> usize;

    /// Every model a minimal sample admits. By default the non-minimal fit, for a solver
    /// whose minimal sample is just the smallest that determines it (the 8-point algorithm,
    /// the DLT).
    ///
    /// # Arguments
    ///
    /// * `indices` - The sample.
    ///
    /// # Returns
    ///
    /// The models (none for a degenerate sample).
    fn fit_minimal(&self, indices: &[usize]) -> Vec<Self::Model> {
        self.fit_nonminimal(indices)
    }

    /// The least-squares models of a larger subset.
    ///
    /// # Arguments
    ///
    /// * `indices` - The subset.
    ///
    /// # Returns
    ///
    /// The models (none for a degenerate subset).
    fn fit_nonminimal(&self, indices: &[usize]) -> Vec<Self::Model>;

    /// Every datum's squared residual under a model.
    ///
    /// # Arguments
    ///
    /// * `model` - The model.
    ///
    /// # Returns
    ///
    /// The squared residuals.
    fn residuals(&self, model: &Self::Model) -> DVector<f64>;
}

/// At most this many refine-and-reselect rounds polish a consensus ([`refine_consensus`]).
pub const REFINE_ROUNDS: usize = 4;

/// Why [`estimate`] stopped, with the integer codes it crosses the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Termination {
    /// Converged: enough samples were drawn to reach the target confidence.
    Confidence = 0,
    /// Not converged: `max_iterations` samples were drawn first.
    MaxIterations = 1,
}

impl Termination {
    /// Every termination, in code order.
    pub const ALL: [Self; 2] = [Self::Confidence, Self::MaxIterations];

    /// The termination's name, as `slamlab.foundations.ransac.Termination` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Confidence => "confidence",
            Self::MaxIterations => "max_iterations",
        }
    }
}

/// RANSAC's settings (see `slamlab.foundations.ransac.Options`).
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    /// The inlier threshold, in the estimator's residual units (not squared).
    pub threshold: f64,
    /// The target probability of having drawn one all-inlier sample, in (0, 1).
    pub confidence: f64,
    /// The most samples to draw.
    pub max_iterations: i64,
    /// The random generator's seed.
    pub seed: u64,
    /// The random generator's stream.
    pub stream: u64,
    /// Refine each new best model (LO-RANSAC).
    pub local_optimization: bool,
    /// Iterative least-squares refits per refinement.
    pub lo_steps: i64,
    /// The first refit's threshold, as a multiple of `threshold` (at least 1).
    pub lo_threshold_multiplier: f64,
    /// Non-minimal inner samples per refinement.
    pub lo_inner_iterations: i64,
    /// The inner sample size, as a multiple of the minimal size.
    pub lo_sample_factor: i64,
}

impl Default for Options {
    /// The defaults of `slamlab.foundations.ransac.Options`.
    fn default() -> Self {
        Self {
            threshold: 1.0,
            confidence: 0.999,
            max_iterations: 10_000,
            seed: 0,
            stream: 0,
            local_optimization: true,
            lo_steps: 4,
            lo_threshold_multiplier: 3.0,
            lo_inner_iterations: 10,
            lo_sample_factor: 3,
        }
    }
}

/// Checks the settings.
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
    if !options.threshold.is_finite() || options.threshold <= 0.0 {
        return fail_setting(
            "threshold",
            "must be positive and finite",
            options.threshold,
        );
    }
    if options.confidence.is_nan() || options.confidence <= 0.0 || options.confidence >= 1.0 {
        return fail_setting("confidence", "must be in (0, 1)", options.confidence);
    }
    for (name, value) in [
        ("max_iterations", options.max_iterations),
        ("lo_steps", options.lo_steps),
        ("lo_sample_factor", options.lo_sample_factor),
    ] {
        if value < 1 {
            return fail_setting(name, "must be at least 1", value);
        }
    }
    if options.lo_inner_iterations < 0 {
        return fail_setting(
            "lo_inner_iterations",
            "must be non-negative",
            options.lo_inner_iterations,
        );
    }
    if !options.lo_threshold_multiplier.is_finite() || options.lo_threshold_multiplier < 1.0 {
        return fail_setting(
            "lo_threshold_multiplier",
            "must be at least 1",
            options.lo_threshold_multiplier,
        );
    }
    Ok(())
}

/// How a run went.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Summary {
    /// Minimal samples drawn.
    pub iterations: usize,
    /// Why sampling stopped.
    pub termination: Termination,
    /// Samples that gave no model.
    pub degenerate_samples: usize,
    /// Refinements run (one per new best model).
    pub local_optimizations: usize,
    /// The best model's inliers.
    pub inlier_count: usize,
    /// The best model's MSAC cost.
    pub score: f64,
}

impl solve::Summary for Summary {
    type Termination = Termination;

    fn iterations(&self) -> usize {
        self.iterations
    }

    fn converged(&self) -> bool {
        self.termination == Termination::Confidence
    }

    fn termination(&self) -> Termination {
        self.termination
    }
}

/// The best model and the data it explains.
#[derive(Debug, Clone, PartialEq)]
pub struct Consensus<Model> {
    /// The model with the lowest MSAC cost.
    pub model: Model,
    /// The data whose residual is below the threshold.
    pub inliers: Vec<bool>,
}

/// The best consensus, and how the run went.
pub type Solution<Model> = solve::Solution<Consensus<Model>, Summary>;

/// The samples needed to draw one all-inlier sample with `confidence`:
/// `log(1 - p) / log(1 - w^s)`, by `log1p`, as the Python reference evaluates it.
///
/// # Arguments
///
/// * `inlier_ratio` - The inlier fraction `w`.
/// * `sample_size` - The minimal sample size `s`.
/// * `confidence` - The target probability `p`.
///
/// # Returns
///
/// The bound: 0 when every datum is an inlier, infinity when none is.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "a sample size is far below 2^52"
)]
pub fn required_iterations(inlier_ratio: f64, sample_size: usize, confidence: f64) -> f64 {
    let all_inliers = inlier_ratio.powf(sample_size as f64);
    if all_inliers >= 1.0 {
        return 0.0;
    }
    if all_inliers <= 0.0 {
        return f64::INFINITY;
    }
    (-confidence).ln_1p() / (-all_inliers).ln_1p()
}

/// The MSAC cost `Σ min(r², θ²)`.
///
/// # Arguments
///
/// * `residuals` - Squared residuals.
/// * `threshold2` - The squared threshold.
///
/// # Returns
///
/// The cost.
fn score(residuals: &DVector<f64>, threshold2: f64) -> f64 {
    residuals.iter().map(|r| r.min(threshold2)).sum()
}

/// The indices whose squared residual is below a squared threshold.
///
/// # Arguments
///
/// * `residuals` - Squared residuals.
/// * `threshold2` - The squared threshold.
///
/// # Returns
///
/// The indices, ascending.
fn below(residuals: &DVector<f64>, threshold2: f64) -> Vec<usize> {
    residuals
        .iter()
        .enumerate()
        .filter(|(_, r)| **r < threshold2)
        .map(|(i, _)| i)
        .collect()
}

/// The inlier mask of squared residuals under a squared threshold.
///
/// # Arguments
///
/// * `residuals` - Squared residuals.
/// * `threshold2` - The squared threshold.
///
/// # Returns
///
/// The mask.
fn mask(residuals: &DVector<f64>, threshold2: f64) -> Vec<bool> {
    residuals.iter().map(|r| *r < threshold2).collect()
}

/// A scored model: its MSAC cost and its squared residuals.
#[derive(Clone)]
struct Scored<Model> {
    /// The model.
    model: Model,
    /// Its MSAC cost.
    score: f64,
    /// Every datum's squared residual.
    residuals: DVector<f64>,
}

/// Scores each model; returns the lowest-cost one (the first on a tie).
///
/// # Arguments
///
/// * `estimator` - The estimator.
/// * `models` - The candidate models.
/// * `threshold2` - The squared threshold.
///
/// # Returns
///
/// The best, or `None` if there were no models.
fn best_of<E: Estimator>(
    estimator: &E,
    models: Vec<E::Model>,
    threshold2: f64,
) -> Option<Scored<E::Model>> {
    let mut best: Option<Scored<E::Model>> = None;
    for model in models {
        let residuals = estimator.residuals(&model);
        let cost = score(&residuals, threshold2);
        if best.as_ref().is_none_or(|b| cost < b.score) {
            best = Some(Scored {
                model,
                score: cost,
                residuals,
            });
        }
    }
    best
}

/// A count setting as a `usize`. [`check_options`] has checked it non-negative, so the clamp
/// to 0 never applies.
///
/// # Arguments
///
/// * `value` - The setting.
///
/// # Returns
///
/// The count.
fn count(value: i64) -> usize {
    usize::try_from(value).unwrap_or(0)
}

/// Refits to the inliers `lo_steps` times, the threshold shrinking to `threshold`.
///
/// # Arguments
///
/// * `estimator` - The estimator.
/// * `start` - The model to refine.
/// * `options` - The settings.
///
/// # Returns
///
/// The lowest-cost model seen.
#[allow(clippy::cast_precision_loss, reason = "a step count is far below 2^52")]
fn least_squares<E: Estimator>(
    estimator: &E,
    start: Scored<E::Model>,
    options: &Options,
) -> Scored<E::Model> {
    let threshold2 = options.threshold * options.threshold;
    let mut best = start.clone();
    let mut current = start;
    let span = options.lo_threshold_multiplier - 1.0;
    let steps = count(options.lo_steps);
    for k in 0..steps {
        let shrink = if steps > 1 {
            span * k as f64 / (steps - 1) as f64
        } else {
            span
        };
        let wide = (options.lo_threshold_multiplier - shrink) * options.threshold;
        let subset = below(&current.residuals, wide * wide);
        if subset.len() < estimator.sample_size() {
            break;
        }
        let Some(fitted) = best_of(estimator, estimator.fit_nonminimal(&subset), threshold2) else {
            break;
        };
        current = fitted;
        if current.score < best.score {
            best = current.clone();
        }
    }
    best
}

/// Refines a new best model: iterative least squares, then an inner RANSAC.
///
/// # Arguments
///
/// * `estimator` - The estimator.
/// * `start` - The new best model.
/// * `rng` - The run's random generator.
/// * `options` - The settings.
///
/// # Returns
///
/// The refined model.
///
/// # Errors
///
/// `InvalidInput` if a draw fails, which [`estimate`]'s checks rule out (the inliers are at
/// most the data, fewer than 2^32, and the draw is at most half of them): one policy with
/// `estimate`'s own draws.
fn local_optimization<E: Estimator>(
    estimator: &E,
    start: Scored<E::Model>,
    rng: &mut Pcg32,
    options: &Options,
) -> Result<Scored<E::Model>> {
    let threshold2 = options.threshold * options.threshold;
    let mut best = least_squares(estimator, start, options);
    for _ in 0..count(options.lo_inner_iterations) {
        let inliers = below(&best.residuals, threshold2);
        let wanted = count(options.lo_sample_factor) * estimator.sample_size();
        let draw = wanted.min(inliers.len() / 2);
        if draw < estimator.sample_size() {
            break;
        }
        let (Ok(population), Ok(size)) = (u32::try_from(inliers.len()), u32::try_from(draw)) else {
            return fail(ErrorKind::InvalidInput, "ransac: more data than 2^32");
        };
        let picks = rng.sample(population, size)?;
        let subset: Vec<usize> = picks.iter().map(|&p| inliers[p as usize]).collect();
        let Some(fitted) = best_of(estimator, estimator.fit_nonminimal(&subset), threshold2) else {
            continue;
        };
        let refined = least_squares(estimator, fitted, options);
        if refined.score < best.score {
            best = refined;
        }
    }
    Ok(best)
}

/// Fits a model robustly by LO-MSAC.
///
/// # Arguments
///
/// * `estimator` - The model's solvers and residuals, over N data.
/// * `options` - The settings.
///
/// # Returns
///
/// The best model, its inliers and the run's summary.
///
/// # Errors
///
/// `InvalidInput` for bad settings or fewer data than a minimal sample; `Degenerate` if no
/// sample gave a model.
#[allow(
    clippy::cast_precision_loss,
    reason = "iteration and data counts are far below 2^52"
)]
pub fn estimate<E: Estimator>(estimator: &E, options: &Options) -> Result<Solution<E::Model>> {
    check_options(options)?;
    let n = estimator.size();
    let s = estimator.sample_size();
    if n < s {
        return fail(
            ErrorKind::InvalidInput,
            format!("ransac: {n} data, fewer than a minimal sample of {s}"),
        );
    }
    let (Ok(population), Ok(sample_size)) = (u32::try_from(n), u32::try_from(s)) else {
        return fail(ErrorKind::InvalidInput, "ransac: more data than 2^32");
    };
    let threshold2 = options.threshold * options.threshold;
    let mut rng = Pcg32::new(options.seed, options.stream);
    let mut best: Option<Scored<E::Model>> = None;
    let mut bound = f64::INFINITY;
    let (mut iterations, mut degenerate, mut local) = (0_usize, 0_usize, 0_usize);
    let max_iterations = count(options.max_iterations);
    while iterations < max_iterations && (iterations as f64) < bound {
        iterations += 1;
        // s <= n, so the draw cannot fail.
        let sample: Vec<usize> = rng
            .sample(population, sample_size)?
            .iter()
            .map(|&i| i as usize)
            .collect();
        let Some(mut fitted) = best_of(estimator, estimator.fit_minimal(&sample), threshold2)
        else {
            degenerate += 1;
            continue;
        };
        if best.as_ref().is_some_and(|b| !(fitted.score < b.score)) {
            continue;
        }
        if options.local_optimization {
            fitted = local_optimization(estimator, fitted, &mut rng, options)?;
            local += 1;
        }
        let inliers = below(&fitted.residuals, threshold2).len();
        bound = required_iterations(inliers as f64 / n as f64, s, options.confidence);
        best = Some(fitted);
    }
    let Some(best) = best else {
        return fail(
            ErrorKind::Degenerate,
            format!("ransac: none of {iterations} samples gave a model"),
        );
    };
    let inliers = mask(&best.residuals, threshold2);
    let termination = if iterations as f64 >= bound {
        Termination::Confidence
    } else {
        Termination::MaxIterations
    };
    let inlier_count = inliers.iter().filter(|&&i| i).count();
    Ok(Solution {
        state: Consensus {
            model: best.model,
            inliers,
        },
        summary: Summary {
            iterations,
            termination,
            degenerate_samples: degenerate,
            local_optimizations: local,
            inlier_count,
            score: best.score,
        },
    })
}

/// How a RANSAC run and the least-squares refinement of its model went (see
/// `slamlab.foundations.ransac.RefinedSummary`).
///
/// It nests the run's summary, and reports the run's iterations, termination and convergence
/// as its own by delegation, as the Python reference's properties do; so, unlike in C++, a
/// refined run is a plain [`solve::Solution`].
#[derive(Debug, Clone, PartialEq)]
pub struct RefinedSummary {
    /// The RANSAC run; its inlier count and score describe the returned model.
    pub run: Summary,
    /// The last refinement's solve, if one ran (refinement on, and enough inliers).
    pub refinement: Option<nls::Summary>,
    /// Whether a refined model was kept (it lowered the MSAC cost).
    pub refined: bool,
    /// Refine-and-reselect rounds run (0 if the refinement did not run).
    pub rounds: usize,
}

impl RefinedSummary {
    /// A run whose refinement did not run.
    ///
    /// # Arguments
    ///
    /// * `run` - The run's summary.
    ///
    /// # Returns
    ///
    /// The summary, with no refinement.
    #[must_use]
    pub const fn unrefined(run: Summary) -> Self {
        Self {
            run,
            refinement: None,
            refined: false,
            rounds: 0,
        }
    }
}

impl solve::Summary for RefinedSummary {
    type Termination = Termination;

    fn iterations(&self) -> usize {
        self.run.iterations
    }

    fn converged(&self) -> bool {
        solve::Summary::converged(&self.run)
    }

    fn termination(&self) -> Termination {
        self.run.termination
    }
}

/// A refined run's estimate and how it went.
pub type RefinedSolution<Model> = solve::Solution<Consensus<Model>, RefinedSummary>;

/// Refines RANSAC's model on its inliers, re-collects them, and repeats while the MSAC cost
/// drops (see `slamlab.foundations.ransac.refine_consensus`): at most [`REFINE_ROUNDS`]
/// rounds, stopping when a round does not lower the cost, the inliers stop changing, or the
/// refinement fails (that round counts, and is not kept).
///
/// # Arguments
///
/// * `estimator` - The estimator RANSAC ran with; its residuals rescore the refined model.
/// * `found` - RANSAC's result.
/// * `refine` - Refines a model on the data the inlier indices select, or fails.
/// * `threshold` - RANSAC's inlier threshold.
/// * `minimum` - The fewest inliers the refinement needs; with fewer it does not run.
///
/// # Returns
///
/// The kept model, its inliers and the summary, whose run's inlier count and score describe
/// the kept model.
pub fn refine_consensus<E: Estimator>(
    estimator: &E,
    found: Solution<E::Model>,
    refine: impl Fn(&E::Model, &[usize]) -> Result<nls::Solution<E::Model>>,
    threshold: f64,
    minimum: usize,
) -> RefinedSolution<E::Model> {
    let threshold2 = threshold * threshold;
    let mut state = found.state;
    let mut summary = RefinedSummary::unrefined(found.summary);
    while summary.rounds < REFINE_ROUNDS {
        let inliers: Vec<usize> = (0..state.inliers.len())
            .filter(|&i| state.inliers[i])
            .collect();
        if inliers.len() < minimum {
            break;
        }
        summary.rounds += 1;
        let Ok(solved) = refine(&state.model, &inliers) else {
            break;
        };
        summary.refinement = Some(solved.summary);
        let residuals = estimator.residuals(&solved.state);
        let candidate = score(&residuals, threshold2);
        if !(candidate < summary.run.score) {
            break;
        }
        let kept = mask(&residuals, threshold2);
        let unchanged = kept == state.inliers;
        state = Consensus {
            model: solved.state,
            inliers: kept,
        };
        summary.run.inlier_count = state.inliers.iter().filter(|&&i| i).count();
        summary.run.score = candidate;
        summary.refined = true;
        if unchanged {
            break;
        }
    }
    solve::Solution { state, summary }
}

/// Refines a model on the data the inlier indices select (A6: indices in every language), or
/// fails; a failed round is not kept (A2).
pub type Refiner<'a, Model> = &'a dyn Fn(&Model, &[usize]) -> Result<nls::Solution<Model>>;

/// Runs LO-MSAC, then, if `refine` is given, refines the consensus (see
/// `slamlab.foundations.ransac.estimate_refined`).
///
/// # Arguments
///
/// * `estimator` - The model's solvers and residuals.
/// * `options` - The settings; the threshold is also the MSAC cap the refined model is
///   rescored with.
/// * `refine` - Refines a model on the data the inlier indices select; `None` skips it.
/// * `minimum` - The fewest inliers the refinement needs.
///
/// # Returns
///
/// The kept model, its inliers and the summary.
///
/// # Errors
///
/// [`estimate`]'s errors.
pub fn estimate_refined<E: Estimator>(
    estimator: &E,
    options: &Options,
    refine: Option<Refiner<'_, E::Model>>,
    minimum: usize,
) -> Result<RefinedSolution<E::Model>> {
    let found = estimate(estimator, options)?;
    Ok(match refine {
        None => solve::Solution {
            state: found.state,
            summary: RefinedSummary::unrefined(found.summary),
        },
        Some(refiner) => refine_consensus(estimator, found, refiner, options.threshold, minimum),
    })
}

#[cfg(test)]
mod tests {
    //! Native tests: a line fitted through outliers, the stopping bound, the settings checks,
    //! and the refine-and-reselect rounds' exits.

    use nalgebra::Vector2;

    use super::*;

    /// Fits `y = a x + b` through 2D points; the residual is the vertical distance.
    struct Line {
        /// The points.
        points: Vec<Vector2<f64>>,
    }

    impl Estimator for Line {
        type Model = (f64, f64);

        fn sample_size(&self) -> usize {
            2
        }

        fn size(&self) -> usize {
            self.points.len()
        }

        fn fit_minimal(&self, indices: &[usize]) -> Vec<(f64, f64)> {
            let (p, q) = (self.points[indices[0]], self.points[indices[1]]);
            if (q.x - p.x).abs() < 1e-12 {
                return Vec::new();
            }
            let a = (q.y - p.y) / (q.x - p.x);
            vec![(a, p.y - a * p.x)]
        }

        #[allow(clippy::cast_precision_loss, reason = "test subsets are small")]
        fn fit_nonminimal(&self, indices: &[usize]) -> Vec<(f64, f64)> {
            let n = indices.len() as f64;
            let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
            for &i in indices {
                let p = self.points[i];
                sx += p.x;
                sy += p.y;
                sxx += p.x * p.x;
                sxy += p.x * p.y;
            }
            let d = n * sxx - sx * sx;
            if d.abs() < 1e-12 {
                return Vec::new();
            }
            let a = (n * sxy - sx * sy) / d;
            vec![(a, (sy - a * sx) / n)]
        }

        fn residuals(&self, model: &(f64, f64)) -> DVector<f64> {
            DVector::from_iterator(
                self.points.len(),
                self.points
                    .iter()
                    .map(|p| (p.y - (model.0 * p.x + model.1)).powi(2)),
            )
        }
    }

    /// A line `y = 2 x + 1` with 30% gross outliers.
    fn line() -> Line {
        let points = (0..50)
            .map(|i| {
                let x = f64::from(i) * 0.1;
                let y = if i % 10 < 3 {
                    x.mul_add(-3.0, 9.0)
                } else {
                    x.mul_add(2.0, 1.0)
                };
                Vector2::new(x, y)
            })
            .collect();
        Line { points }
    }

    /// The line is recovered with every inlier, at the target confidence.
    #[test]
    fn line_through_outliers() {
        let options = Options {
            threshold: 0.1,
            seed: 7,
            ..Options::default()
        };
        let found = estimate(&line(), &options).unwrap();
        let (a, b) = found.state.model;
        assert!((a - 2.0).abs() < 1e-12 && (b - 1.0).abs() < 1e-12);
        assert_eq!(found.summary.inlier_count, 35);
        assert_eq!(found.summary.termination, Termination::Confidence);
    }

    /// The bound: none needed when every datum is an inlier, infinite when none is.
    #[test]
    fn required_iterations_at_the_ends() {
        assert!(required_iterations(1.0, 5, 0.99).abs() < f64::MIN_POSITIVE);
        assert!(required_iterations(0.0, 5, 0.99).is_infinite());
        let mid = required_iterations(0.5, 2, 0.99);
        assert!((mid - (0.01_f64.ln() / 0.75_f64.ln())).abs() < 1e-12);
    }

    /// Bad settings and too few data are `InvalidInput`.
    #[test]
    fn bad_settings_are_rejected() {
        let bad = Options {
            confidence: 1.0,
            ..Options::default()
        };
        assert_eq!(
            estimate(&line(), &bad).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let tiny = Line {
            points: vec![Vector2::zeros()],
        };
        let err = estimate(&tiny, &Options::default()).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
    }

    /// A refinement that improves the model is kept, and its summary reports the run's
    /// convergence by delegation; one that does not is discarded.
    #[test]
    fn refinement_kept_only_when_it_lowers_the_cost() {
        let estimator = line();
        let options = Options {
            threshold: 0.1,
            seed: 7,
            local_optimization: false,
            ..Options::default()
        };
        let found = estimate(&estimator, &options).unwrap();
        let unsolved = nls::Summary {
            initial_cost: 0.0,
            final_cost: 0.0,
            termination: nls::Termination::Function,
            trace: Vec::new(),
        };
        let worse = refine_consensus(
            &estimator,
            found.clone(),
            |m, _| {
                Ok(nls::Solution {
                    state: (m.0 + 1.0, m.1),
                    summary: unsolved.clone(),
                })
            },
            0.1,
            2,
        );
        assert!(!worse.summary.refined);
        assert_eq!(worse.summary.rounds, 1);
        assert_eq!(worse.state.model, found.state.model);
        assert!(solve::Summary::converged(&worse.summary));
        let too_few = refine_consensus(
            &estimator,
            found.clone(),
            |m, _| {
                Ok(nls::Solution {
                    state: *m,
                    summary: unsolved.clone(),
                })
            },
            0.1,
            100,
        );
        assert_eq!(too_few.summary.rounds, 0);
        // A failed refinement counts as a round and is not kept (A2).
        let failed = refine_consensus(
            &estimator,
            found.clone(),
            |_, _| {
                fail(
                    ErrorKind::InvalidInput,
                    "solve: the initial cost is not finite",
                )
            },
            0.1,
            2,
        );
        assert_eq!(failed.summary.rounds, 1);
        assert!(!failed.summary.refined && failed.summary.refinement.is_none());
        assert_eq!(failed.state.model, found.state.model);
    }

    /// Points that all coincide give no model from any sample: `Degenerate`, not a result.
    #[test]
    fn no_model_from_any_sample_is_degenerate() {
        let coincident = Line {
            points: vec![Vector2::new(1.0, 2.0); 10],
        };
        let kind = estimate(&coincident, &Options::default()).unwrap_err().kind;
        assert_eq!(kind, ErrorKind::Degenerate);
    }
}
