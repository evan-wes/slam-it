/// @file
/// @brief Non-linear least squares on manifolds: the C++ core's Gauss-Newton and
///        Levenberg-Marquardt loop.
///
/// Mirrors `slamlab.foundations.nls`, whose docstring states the maths: robust losses by
/// iteratively reweighted least squares, Marquardt's clamped diagonal damping with Nielsen's
/// update, and Ceres' termination tests. The loop is a template over the `Problem` concept,
/// so the same code drives a dense test problem and bundle adjustment's Schur system. It is
/// not bound to Python itself: the backends meet at the concrete solvers built on it.
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "slamlab/foundations/error.hpp"

/// @brief Non-linear least squares.
namespace slamlab::foundations::nls {

/// @brief The step rule.
enum class Method : std::uint8_t {
  /// @brief Damped steps, accepted on a sufficient ratio of actual to predicted decrease.
  kLevenbergMarquardt,
  /// @brief Undamped steps, stopping at the first that fails to decrease the cost.
  kGaussNewton,
};

/// @brief The robust losses, acting on a block's squared norm `s`.
enum class LossKind : std::uint8_t {
  /// @brief `rho(s) = s`.
  kTrivial,
  /// @brief `rho(s) = s` for `s <= c^2`, else `2 c sqrt(s) - c^2`.
  kHuber,
  /// @brief `rho(s) = c^2 log(1 + s / c^2)`.
  kCauchy,
};

/// @brief A robust loss and its scale `c`.
struct Loss {
  /// @brief The loss.
  LossKind kind = LossKind::kTrivial;
  /// @brief The residual norm at which the loss starts to discount a block (positive).
  double scale = 1.0;
};

/// @brief A loss evaluated at one squared norm.
struct LossValue {
  /// @brief `rho(s)`.
  double rho;
  /// @brief `rho'(s)`, the block's IRLS weight, in `(0, 1]`.
  double weight;
};

/// @brief Evaluate a loss and its derivative.
/// @param loss The loss (checked by `check_options`).
/// @param squared_norm The block's squared norm, `s >= 0`.
/// @return `rho(s)` and `rho'(s)`.
[[nodiscard]] LossValue evaluate(const Loss& loss, double squared_norm);

/// @brief Settings for `solve`; the defaults follow Ceres where it has an equivalent.
struct Options {
  /// @brief The step rule.
  Method method = Method::kLevenbergMarquardt;
  /// @brief The robust loss applied to every block.
  Loss loss;
  /// @brief Steps tried, accepted or not, before stopping.
  int max_iterations = 50;
  /// @brief Stop when an accepted step lowers the cost by less than this fraction of it.
  double function_tolerance = 1e-6;
  /// @brief Stop when the gradient's largest component is below this.
  double gradient_tolerance = 1e-10;
  /// @brief Stop when the step's norm (in the tangent space) is below this.
  double step_tolerance = 1e-10;
  /// @brief The first damping `mu`.
  double initial_damping = 1e-4;
  /// @brief Stop, unconverged, when `mu` exceeds this.
  double max_damping = 1e32;
  /// @brief Lower clamp on `diag(J^T W J)` before it scales the damping.
  double min_diagonal = 1e-6;
  /// @brief Upper clamp on `diag(J^T W J)` before it scales the damping.
  double max_diagonal = 1e32;
  /// @brief The smallest ratio of actual to predicted decrease for a step to be accepted.
  double min_relative_decrease = 1e-3;
  /// @brief Fail with `NotConverged` instead of returning an unconverged estimate.
  bool strict = false;
};

/// @brief Check a loss and the solver settings.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the setting out of range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief Why `solve` stopped.
enum class Termination : std::uint8_t {
  /// @brief Converged: the gradient is below `gradient_tolerance`.
  kGradient,
  /// @brief Converged: the last accepted step barely lowered the cost.
  kFunction,
  /// @brief Converged: the step is below `step_tolerance`.
  kStep,
  /// @brief Not converged: `max_iterations` steps were tried.
  kMaxIterations,
  /// @brief Not converged: the damping exceeded `max_damping`.
  kDampingLimit,
  /// @brief Not converged: a Gauss-Newton step failed to lower the cost.
  kNoDecrease,
  /// @brief Not converged: the undamped Gauss-Newton system is not positive definite.
  kSingular,
};

/// @brief Whether a termination counts as converged.
/// @param termination The termination.
/// @return True for the gradient, function and step tests.
[[nodiscard]] bool is_converged(Termination termination);

/// @brief The termination's name, as `slamlab.foundations.nls.Termination` spells it.
/// @param termination The termination.
/// @return A static string.
[[nodiscard]] const char* name(Termination termination);

/// @brief One step tried by `solve`.
struct Iteration {
  /// @brief The step's index, from 1.
  int iteration;
  /// @brief The cost before the step.
  double cost;
  /// @brief The gradient's largest component, before the step.
  double gradient_norm;
  /// @brief The step's norm (NaN when the system could not be factored).
  double step_norm;
  /// @brief The damping for this step (0 for Gauss-Newton).
  double damping;
  /// @brief Actual over predicted cost decrease (NaN when not computed).
  double relative_decrease;
  /// @brief Whether the step was taken.
  bool accepted;
};

/// @brief How a solve went.
struct SolveSummary {
  /// @brief The cost at the start.
  double initial_cost = 0.0;
  /// @brief The cost at the returned state.
  double final_cost = 0.0;
  /// @brief Steps tried, accepted or not.
  int iterations = 0;
  /// @brief Why the solve stopped.
  Termination termination = Termination::kMaxIterations;
  /// @brief Whether `termination` counts as converged.
  bool converged = false;
  /// @brief Every step tried, in order.
  std::vector<Iteration> trace;
};

/// @brief The best state found, and how the solve went.
/// @tparam State The problem's state type.
template <typename State>
struct Solution {
  /// @brief The lowest-cost state visited.
  State state;
  /// @brief The solve's summary.
  SolveSummary summary;
};

/// @brief A linearisation's normal equations, `(H + diag(d)) delta = -g`.
template <typename S>
concept LinearSystem = requires(const S& system, const Eigen::VectorXd& damping) {
  { system.gradient() } -> std::convertible_to<Eigen::VectorXd>;
  { system.hessian_diagonal() } -> std::convertible_to<Eigen::VectorXd>;
  { system.solve(damping) } -> std::same_as<std::optional<Eigen::VectorXd>>;
};

/// @brief A least-squares problem: block squared norms, a linearisation, a retraction.
template <typename P>
concept Problem = requires(const P& problem, const P::State& state, const Eigen::VectorXd& vector) {
  { problem.squared_norms(state) } -> std::convertible_to<Eigen::VectorXd>;
  { problem.linearize(state, vector) } -> LinearSystem;
  { problem.retract(state, vector) } -> std::convertible_to<typename P::State>;
};

/// @brief Normal equations formed densely from a full Jacobian, factored by Cholesky.
class DenseSystem {
 public:
  /// @brief Form the weighted normal equations of M blocks of d rows over n parameters.
  /// @param jacobian The (M d) x n Jacobian, block by block.
  /// @param residuals The M d residuals, block by block.
  /// @param weights The M block weights.
  /// @return The system, or `InvalidInput` if the sizes disagree.
  [[nodiscard]] static Result<DenseSystem> build(const Eigen::MatrixXd& jacobian,
                                                 const Eigen::VectorXd& residuals,
                                                 const Eigen::VectorXd& weights);

  /// @brief The weighted gradient `g = J^T W r`.
  /// @return The gradient.
  [[nodiscard]] const Eigen::VectorXd& gradient() const { return gradient_; }

  /// @brief The diagonal of `H = J^T W J`.
  /// @return The diagonal.
  [[nodiscard]] Eigen::VectorXd hessian_diagonal() const { return hessian_.diagonal(); }

  /// @brief Solve `(H + diag(damping)) delta = -g` by Cholesky.
  /// @param damping Added to H's diagonal.
  /// @return The step, or nothing if the damped system is not positive definite.
  [[nodiscard]] std::optional<Eigen::VectorXd> solve(const Eigen::VectorXd& damping) const;

 private:
  DenseSystem(Eigen::MatrixXd hessian, Eigen::VectorXd gradient)
      : hessian_(std::move(hessian)), gradient_(std::move(gradient)) {}

  Eigen::MatrixXd hessian_;
  Eigen::VectorXd gradient_;
};

/// @brief Implementation details of `solve`.
namespace detail {

/// @brief The cost `1/2 sum rho(s_i)` and the IRLS weights, or an infinite cost if any
///        block is not finite.
struct Evaluation {
  /// @brief The cost.
  double cost;
  /// @brief The blocks' squared norms.
  Eigen::VectorXd squared_norms;
  /// @brief The blocks' weights `rho'(s_i)`.
  Eigen::VectorXd weights;
};

/// @brief Evaluate the cost and weights of a set of squared norms.
/// @param loss The loss.
/// @param squared_norms The blocks' squared norms.
/// @return The evaluation.
[[nodiscard]] Evaluation evaluate_blocks(const Loss& loss, Eigen::VectorXd squared_norms);

/// @brief A step tried: its record, and the candidate state with its evaluation (if any).
/// @tparam State The problem's state type.
template <typename State>
struct Trial {
  /// @brief The step's trace record.
  Iteration record{};
  /// @brief The state the step leads to; empty when the system could not be factored.
  std::optional<State> candidate;
  /// @brief The candidate's evaluation (meaningful only with a candidate).
  Evaluation evaluation;
};

/// @brief Compute, apply and judge one step; or return the convergence test it stops on.
/// @tparam P The problem type.
/// @tparam S The linear system type.
/// @param problem The problem.
/// @param state The current state.
/// @param current The current state's evaluation.
/// @param system The current linearisation.
/// @param damping The damping `mu` (0 for Gauss-Newton).
/// @param number The step's index, from 1.
/// @param options The settings.
/// @return The trial, or `kGradient`/`kStep` if a convergence test stops the solve first.
template <Problem P, LinearSystem S>
[[nodiscard]] std::variant<Trial<typename P::State>, Termination> try_step(
    const P& problem, const typename P::State& state, const Evaluation& current, const S& system,
    double damping, int number, const Options& options) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  const auto& gradient = system.gradient();
  const double gradient_norm = gradient.size() == 0 ? 0.0 : gradient.cwiseAbs().maxCoeff();
  if (gradient_norm <= options.gradient_tolerance) {
    return Termination::kGradient;
  }
  const Eigen::VectorXd diagonal =
      damping *
      system.hessian_diagonal().cwiseMax(options.min_diagonal).cwiseMin(options.max_diagonal);
  const std::optional<Eigen::VectorXd> delta = system.solve(diagonal);
  if (!delta) {
    return Trial<typename P::State>{
        .record = {number, current.cost, gradient_norm, kNan, damping, kNan, false},
        .candidate = std::nullopt,
        .evaluation = {}};
  }
  const double step_norm = delta->norm();
  if (step_norm <= options.step_tolerance) {
    return Termination::kStep;
  }
  typename P::State candidate = problem.retract(state, *delta);
  Evaluation evaluation = evaluate_blocks(options.loss, problem.squared_norms(candidate));
  const double predicted = 0.5 * (delta->dot(diagonal.cwiseProduct(*delta)) - gradient.dot(*delta));
  const double ratio = predicted > 0.0 ? (current.cost - evaluation.cost) / predicted : kNan;
  const bool accepted =
      std::isfinite(evaluation.cost) && evaluation.cost < current.cost &&
      (options.method == Method::kGaussNewton || ratio > options.min_relative_decrease);
  return Trial<typename P::State>{
      .record = {number, current.cost, gradient_norm, step_norm, damping, ratio, accepted},
      .candidate = std::move(candidate),
      .evaluation = std::move(evaluation)};
}

}  // namespace detail

/// @brief Minimise `1/2 sum rho(|r_i|^2)` from `initial` by Levenberg-Marquardt or
///        Gauss-Newton.
/// @tparam P The problem type.
/// @param problem The residual blocks, their linearisation and the state's retraction.
/// @param initial The starting state.
/// @param options Method, loss, tolerances and limits.
/// @return The lowest-cost state visited with the summary; `InvalidInput` for bad options or
///         a non-finite initial cost; `NotConverged` if `options.strict` and the solve
///         stopped without converging.
template <Problem P>
[[nodiscard]] Result<Solution<typename P::State>> solve(const P& problem, typename P::State initial,
                                                        const Options& options) {
  if (auto checked = check_options(options); !checked) {
    return std::unexpected(checked.error());
  }
  const bool levenberg = options.method == Method::kLevenbergMarquardt;
  typename P::State state = std::move(initial);
  detail::Evaluation current = detail::evaluate_blocks(options.loss, problem.squared_norms(state));
  if (!std::isfinite(current.cost)) {
    return fail(ErrorKind::kInvalidInput, "solve: the initial cost is not finite");
  }
  const double initial_cost = current.cost;
  double damping = levenberg ? options.initial_damping : 0.0;
  double growth = 2.0;
  auto system = problem.linearize(state, current.weights);
  std::vector<Iteration> trace;
  Termination termination = Termination::kMaxIterations;
  while (std::cmp_less(trace.size(), options.max_iterations)) {
    auto tried = detail::try_step(problem, state, current, system, damping,
                                  static_cast<int>(trace.size()) + 1, options);
    if (const auto* stop = std::get_if<Termination>(&tried)) {
      termination = *stop;
      break;
    }
    auto& trial = *std::get_if<detail::Trial<typename P::State>>(&tried);
    trace.push_back(trial.record);
    if (trial.record.accepted && trial.candidate) {
      const double previous = current.cost;
      state = std::move(*trial.candidate);
      current = std::move(trial.evaluation);
      if (levenberg) {
        const double centred = (2.0 * trial.record.relative_decrease) - 1.0;
        damping *= std::max(1.0 / 3.0, 1.0 - (centred * centred * centred));
        growth = 2.0;
      }
      if (previous - current.cost <= options.function_tolerance * previous) {
        termination = Termination::kFunction;
        break;
      }
      system = problem.linearize(state, current.weights);
      continue;
    }
    if (!levenberg) {
      termination = trial.candidate ? Termination::kNoDecrease : Termination::kSingular;
      break;
    }
    damping *= growth;
    growth *= 2.0;
    if (damping > options.max_damping) {
      termination = Termination::kDampingLimit;
      break;
    }
  }
  const bool converged = is_converged(termination);
  if (options.strict && !converged) {
    return fail(ErrorKind::kNotConverged,
                std::string("solve: stopped without converging (") + name(termination) + ")");
  }
  const int iterations = static_cast<int>(trace.size());
  return Solution<typename P::State>{
      .state = std::move(state),
      .summary = SolveSummary{.initial_cost = initial_cost,
                              .final_cost = current.cost,
                              .iterations = iterations,
                              .termination = termination,
                              .converged = converged,
                              .trace = std::move(trace)},
  };
}

}  // namespace slamlab::foundations::nls
