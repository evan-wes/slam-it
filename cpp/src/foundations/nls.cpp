/// @file
/// @brief Non-linear least squares: the losses, the option checks and the dense system.
#include "slamlab/foundations/nls.hpp"

#include <Eigen/Cholesky>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace slamlab::foundations::nls {

namespace {

/// @brief Check that a setting is finite and positive (or, if `allow_zero`, non-negative).
/// @param name The setting's name, for the message.
/// @param value The value.
/// @param allow_zero Whether zero is allowed.
/// @return Nothing, or `InvalidInput`.
Result<void> check_setting(const char* name, double value, bool allow_zero) {
  if (!std::isfinite(value) || value < 0.0 || (!allow_zero && value == 0.0)) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + (allow_zero ? ": must be non-negative and finite"
                                                : ": must be positive and finite"));
  }
  return {};
}

}  // namespace

LossValue evaluate(const Loss& loss, double squared_norm) {
  const double c2 = loss.scale * loss.scale;
  switch (loss.kind) {
    case LossKind::kHuber: {
      if (squared_norm <= c2) {
        return {.rho = squared_norm, .weight = 1.0};
      }
      const double norm = std::sqrt(squared_norm);
      return {.rho = (2.0 * loss.scale * norm) - c2, .weight = loss.scale / norm};
    }
    case LossKind::kCauchy:
      return {.rho = c2 * std::log1p(squared_norm / c2),
              .weight = 1.0 / (1.0 + (squared_norm / c2))};
    case LossKind::kTrivial:
      break;
  }
  return {.rho = squared_norm, .weight = 1.0};
}

Result<void> check_options(const Options& options) {
  if (auto checked = check_setting("loss scale", options.loss.scale, false); !checked) {
    return checked;
  }
  if (options.max_iterations < 1) {
    return fail(ErrorKind::kInvalidInput, "max_iterations: must be at least 1");
  }
  const std::array<std::pair<const char*, double>, 4> positive = {{
      {"initial_damping", options.initial_damping},
      {"max_damping", options.max_damping},
      {"min_diagonal", options.min_diagonal},
      {"max_diagonal", options.max_diagonal},
  }};
  const std::array<std::pair<const char*, double>, 4> non_negative = {{
      {"function_tolerance", options.function_tolerance},
      {"gradient_tolerance", options.gradient_tolerance},
      {"step_tolerance", options.step_tolerance},
      {"min_relative_decrease", options.min_relative_decrease},
  }};
  for (const auto& [setting, value] : positive) {
    if (auto checked = check_setting(setting, value, false); !checked) {
      return checked;
    }
  }
  for (const auto& [setting, value] : non_negative) {
    if (auto checked = check_setting(setting, value, true); !checked) {
      return checked;
    }
  }
  if (options.min_diagonal > options.max_diagonal) {
    return fail(ErrorKind::kInvalidInput, "min_diagonal: must not exceed max_diagonal");
  }
  return {};
}

bool is_converged(Termination termination) {
  return termination == Termination::kGradient || termination == Termination::kFunction ||
         termination == Termination::kStep;
}

const char* name(Termination termination) {
  switch (termination) {
    case Termination::kGradient:
      return "gradient";
    case Termination::kFunction:
      return "function";
    case Termination::kStep:
      return "step";
    case Termination::kMaxIterations:
      return "max_iterations";
    case Termination::kDampingLimit:
      return "damping_limit";
    case Termination::kNoDecrease:
      return "no_decrease";
    case Termination::kSingular:
      return "singular";
  }
  return "unknown";
}

Result<DenseSystem> DenseSystem::build(const Eigen::MatrixXd& jacobian,
                                       const Eigen::VectorXd& residuals,
                                       const Eigen::VectorXd& weights) {
  const Eigen::Index blocks = weights.size();
  if (jacobian.rows() != residuals.size() || blocks == 0 || residuals.size() % blocks != 0) {
    return fail(ErrorKind::kInvalidInput,
                "jacobian, residuals, weights: expected (M d) x n, M d and M entries");
  }
  const Eigen::Index block_size = residuals.size() / blocks;
  const Eigen::VectorXd row_weights = weights.transpose().replicate(block_size, 1).reshaped();
  const Eigen::MatrixXd weighted = row_weights.asDiagonal() * jacobian;
  return DenseSystem(weighted.transpose() * jacobian, weighted.transpose() * residuals);
}

std::optional<Eigen::VectorXd> DenseSystem::solve(const Eigen::VectorXd& damping) const {
  Eigen::MatrixXd system = hessian_;
  system.diagonal() += damping;
  const Eigen::LLT<Eigen::MatrixXd> factor(system);
  if (factor.info() != Eigen::Success) {
    return std::nullopt;
  }
  return Eigen::VectorXd(-factor.solve(gradient_));
}

namespace detail {

Evaluation evaluate_blocks(const Loss& loss, Eigen::VectorXd squared_norms) {
  Evaluation out{.cost = 0.0, .squared_norms = std::move(squared_norms), .weights = {}};
  out.weights.resize(out.squared_norms.size());
  for (Eigen::Index i = 0; i < out.squared_norms.size(); ++i) {
    const double s = out.squared_norms(i);
    if (!std::isfinite(s)) {
      out.cost = std::numeric_limits<double>::infinity();
      return out;
    }
    const LossValue value = evaluate(loss, s);
    out.cost += value.rho;
    out.weights(i) = value.weight;
  }
  out.cost *= 0.5;
  return out;
}

}  // namespace detail

}  // namespace slamlab::foundations::nls
