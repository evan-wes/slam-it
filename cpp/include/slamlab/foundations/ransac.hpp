/// @file
/// @brief RANSAC: the C++ core's LO-MSAC loop.
///
/// Mirrors `slamlab.foundations.ransac`, whose docstring states the method: MSAC scoring,
/// the adaptive stopping bound, and Lebeda et al.'s local optimisation (iterative least
/// squares, then an inner RANSAC of non-minimal samples). The loop is a template over the
/// `Estimator` concept, so the same code drives every two-view model. It is not bound to
/// Python itself: the backends meet at the concrete estimators built on it. Samples come
/// from `rng::Pcg32`, so a seed draws the same samples as the Python reference.
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/rng.hpp"
#include "slamlab/foundations/solve.hpp"

/// @brief Robust model fitting by random sampling.
namespace slamlab::foundations::ransac {

/// @brief Indices of data, as drawn by `rng::Pcg32::sample`.
using Indices = std::vector<std::uint32_t>;

/// @brief A model RANSAC can fit: its minimal and least-squares solvers and its residuals.
/// @tparam E The estimator type, with a `Model` type member.
template <typename E>
concept Estimator =
    requires(const E& estimator, std::span<const std::uint32_t> indices, const E::Model& model) {
      typename E::Model;
      { estimator.sample_size() } -> std::convertible_to<std::uint32_t>;
      { estimator.size() } -> std::convertible_to<std::uint32_t>;
      { estimator.fit_minimal(indices) } -> std::convertible_to<std::vector<typename E::Model>>;
      { estimator.fit_nonminimal(indices) } -> std::convertible_to<std::vector<typename E::Model>>;
      { estimator.residuals(model) } -> std::convertible_to<Eigen::VectorXd>;
    };

/// @brief Why `estimate` stopped, with the integer codes it crosses the bindings as.
enum class Termination : std::uint8_t {
  /// @brief Converged: enough samples were drawn to reach the target confidence.
  kConfidence = 0,
  /// @brief Not converged: `max_iterations` samples were drawn first.
  kMaxIterations = 1,
  /// @brief The number of terminations (a sentinel, not a termination).
  kCount = 2,
};

/// @brief The termination's name, as `slamlab.foundations.ransac.Termination` spells it.
/// @param termination The termination.
/// @return A static string.
[[nodiscard]] const char* name(Termination termination);

/// @brief RANSAC's settings (see `slamlab.foundations.ransac.Options`).
struct Options {
  /// @brief The inlier threshold, in the estimator's residual units (not squared).
  double threshold = 1.0;
  /// @brief The target probability of having drawn one all-inlier sample, in (0, 1).
  double confidence = 0.999;
  /// @brief The most samples to draw.
  int max_iterations = 10000;
  /// @brief The random generator's seed.
  std::uint64_t seed = 0;
  /// @brief The random generator's stream.
  std::uint64_t stream = 0;
  /// @brief Refine each new best model (LO-RANSAC).
  bool local_optimization = true;
  /// @brief Iterative least-squares refits per refinement.
  int lo_steps = 4;
  /// @brief The first refit's threshold, as a multiple of `threshold` (at least 1).
  double lo_threshold_multiplier = 3.0;
  /// @brief Non-minimal inner samples per refinement.
  int lo_inner_iterations = 10;
  /// @brief The inner sample size, as a multiple of the minimal size.
  int lo_sample_factor = 3;
};

/// @brief Check the settings.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the setting out of range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief How a run went. Satisfies `solve::Summary`.
struct Summary {
  /// @brief Minimal samples drawn.
  int iterations = 0;
  /// @brief Why sampling stopped.
  Termination termination = Termination::kMaxIterations;
  /// @brief Whether `termination` counts as converged.
  bool converged = false;
  /// @brief Samples that gave no model.
  int degenerate_samples = 0;
  /// @brief Refinements run (one per new best model).
  int local_optimizations = 0;
  /// @brief The best model's inliers.
  int inlier_count = 0;
  /// @brief The best model's MSAC cost.
  double score = 0.0;
};

/// @brief The best model and the data it explains.
/// @tparam Model The estimator's model type.
template <typename Model>
struct Consensus {
  /// @brief The model with the lowest MSAC cost.
  Model model;
  /// @brief The data whose residual is below the threshold.
  Eigen::Array<bool, Eigen::Dynamic, 1> inliers;
};

/// @brief The best consensus, and how the run went.
/// @tparam Model The estimator's model type.
template <typename Model>
using Solution = solve::Solution<Consensus<Model>, Summary>;

/// @brief The samples needed to draw one all-inlier sample with `confidence`:
///        `log(1 - p) / log(1 - w^s)`, by `log1p`, as the Python reference evaluates it.
/// @param inlier_ratio The inlier fraction `w`.
/// @param sample_size The minimal sample size `s`.
/// @param confidence The target probability `p`.
/// @return The bound: 0 when every datum is an inlier, infinity when none is.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (w, s, p), as in the formula
[[nodiscard]] double required_iterations(double inlier_ratio, std::uint32_t sample_size,
                                         double confidence);

namespace detail {

/// @brief A scored model: its MSAC cost and its squared residuals.
/// @tparam Model The model type.
template <typename Model>
struct Scored {
  /// @brief The model.
  Model model;
  /// @brief Its MSAC cost.
  double score = 0.0;
  /// @brief Every datum's squared residual.
  Eigen::VectorXd residuals;
};

/// @brief The MSAC cost `sum min(r^2, theta^2)`.
/// @param residuals Squared residuals.
/// @param threshold2 The squared threshold.
/// @return The cost.
[[nodiscard]] double score(const Eigen::VectorXd& residuals, double threshold2);

/// @brief The indices whose squared residual is below a squared threshold.
/// @param residuals Squared residuals.
/// @param threshold2 The squared threshold.
/// @return The indices, ascending.
[[nodiscard]] Indices below(const Eigen::VectorXd& residuals, double threshold2);

/// @brief Score each model; return the lowest-cost one (the first on a tie).
/// @tparam E The estimator type.
/// @param estimator The estimator.
/// @param models The candidate models.
/// @param threshold2 The squared threshold.
/// @return The best, or nothing if there were no models.
template <Estimator E>
std::optional<Scored<typename E::Model>> best_of(const E& estimator,
                                                 std::vector<typename E::Model> models,
                                                 double threshold2) {
  std::optional<Scored<typename E::Model>> best;
  for (auto& model : models) {
    Eigen::VectorXd residuals = estimator.residuals(model);
    const double cost = detail::score(residuals, threshold2);
    if (!best || cost < best->score) {
      best = Scored<typename E::Model>{std::move(model), cost, std::move(residuals)};
    }
  }
  return best;
}

/// @brief Refit to the inliers `lo_steps` times, the threshold shrinking to `threshold`.
/// @tparam E The estimator type.
/// @param estimator The estimator.
/// @param start The model to refine.
/// @param options The settings.
/// @return The lowest-cost model seen.
template <Estimator E>
Scored<typename E::Model> least_squares(const E& estimator, Scored<typename E::Model> start,
                                        const Options& options) {
  const double threshold2 = options.threshold * options.threshold;
  Scored<typename E::Model> best = start;
  Scored<typename E::Model> current = std::move(start);
  const double span = options.lo_threshold_multiplier - 1.0;
  for (int k = 0; k < options.lo_steps; ++k) {
    const double shrink =
        options.lo_steps > 1 ? span * k / static_cast<double>(options.lo_steps - 1) : span;
    const double multiplier = options.lo_threshold_multiplier - shrink;
    const double wide = multiplier * options.threshold;
    const Indices subset = below(current.residuals, wide * wide);
    if (subset.size() < estimator.sample_size()) {
      break;
    }
    auto fitted = best_of(estimator, estimator.fit_nonminimal(subset), threshold2);
    if (!fitted) {
      break;
    }
    current = std::move(*fitted);
    if (current.score < best.score) {
      best = current;
    }
  }
  return best;
}

/// @brief Refine a new best model: iterative least squares, then an inner RANSAC.
/// @tparam E The estimator type.
/// @param estimator The estimator.
/// @param start The new best model.
/// @param rng The run's random generator.
/// @param options The settings.
/// @return The refined model.
template <Estimator E>
Scored<typename E::Model> local_optimization(const E& estimator, Scored<typename E::Model> start,
                                             rng::Pcg32& rng, const Options& options) {
  const double threshold2 = options.threshold * options.threshold;
  Scored<typename E::Model> best = least_squares(estimator, std::move(start), options);
  for (int i = 0; i < options.lo_inner_iterations; ++i) {
    const Indices inliers = below(best.residuals, threshold2);
    const auto population = static_cast<std::uint32_t>(inliers.size());
    const std::uint32_t count =
        std::min(static_cast<std::uint32_t>(options.lo_sample_factor) * estimator.sample_size(),
                 population / 2);
    if (count < estimator.sample_size()) {
      break;
    }
    // The count is at most half the inliers, so the draw cannot fail.
    const Indices picks = *rng.sample(population, count);
    Indices subset;
    subset.reserve(count);
    for (const std::uint32_t pick : picks) {
      subset.push_back(inliers[pick]);
    }
    auto fitted = best_of(estimator, estimator.fit_nonminimal(subset), threshold2);
    if (!fitted) {
      continue;
    }
    Scored<typename E::Model> refined = least_squares(estimator, std::move(*fitted), options);
    if (refined.score < best.score) {
      best = std::move(refined);
    }
  }
  return best;
}

}  // namespace detail

/// @brief Fit a model robustly by LO-MSAC.
/// @tparam E The estimator type.
/// @param estimator The model's solvers and residuals, over N data.
/// @param options The settings.
/// @return The best model, its inliers and the run's summary; `InvalidInput` for bad settings
///         or fewer data than a minimal sample; `Degenerate` if no sample gave a model.
template <Estimator E>
[[nodiscard]] Result<Solution<typename E::Model>> estimate(const E& estimator,
                                                           const Options& options) {
  if (auto checked = check_options(options); !checked) {
    return std::unexpected(checked.error());
  }
  const std::uint32_t n = estimator.size();
  const std::uint32_t s = estimator.sample_size();
  if (n < s) {
    return fail(ErrorKind::kInvalidInput, "ransac: " + std::to_string(n) +
                                              " data, fewer than a minimal sample of " +
                                              std::to_string(s));
  }
  const double threshold2 = options.threshold * options.threshold;
  rng::Pcg32 rng(options.seed, options.stream);
  std::optional<detail::Scored<typename E::Model>> best;
  double bound = std::numeric_limits<double>::infinity();
  int iterations = 0;
  int degenerate = 0;
  int local = 0;
  while (iterations < options.max_iterations && iterations < bound) {
    ++iterations;
    // s <= n, so the draw cannot fail.
    const Indices sample = *rng.sample(n, s);
    auto fitted = detail::best_of(estimator, estimator.fit_minimal(sample), threshold2);
    if (!fitted) {
      ++degenerate;
      continue;
    }
    if (best && !(fitted->score < best->score)) {
      continue;
    }
    if (options.local_optimization) {
      fitted = detail::local_optimization(estimator, std::move(*fitted), rng, options);
      ++local;
    }
    best = std::move(fitted);
    const double ratio = static_cast<double>((best->residuals.array() < threshold2).count()) /
                         static_cast<double>(n);
    bound = required_iterations(ratio, s, options.confidence);
  }
  if (!best) {
    return fail(ErrorKind::kDegenerate,
                "ransac: none of " + std::to_string(iterations) + " samples gave a model");
  }
  Eigen::Array<bool, Eigen::Dynamic, 1> inliers = best->residuals.array() < threshold2;
  const Termination termination =
      iterations >= bound ? Termination::kConfidence : Termination::kMaxIterations;
  const auto inlier_count = static_cast<int>(inliers.count());
  return Solution<typename E::Model>{
      .state = Consensus<typename E::Model>{.model = std::move(best->model),
                                            .inliers = std::move(inliers)},
      .summary = Summary{.iterations = iterations,
                         .termination = termination,
                         .converged = termination == Termination::kConfidence,
                         .degenerate_samples = degenerate,
                         .local_optimizations = local,
                         .inlier_count = inlier_count,
                         .score = best->score},
  };
}

}  // namespace slamlab::foundations::ransac
