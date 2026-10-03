/// @file
/// @brief The non-template parts of RANSAC declared in slamlab/foundations/ransac.hpp.
#include "slamlab/foundations/ransac.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace slamlab::foundations::ransac {

const char* name(Termination termination) {
  switch (termination) {
    case Termination::kConfidence:
      return "confidence";
    case Termination::kMaxIterations:
      return "max_iterations";
    case Termination::kCount:
      break;
  }
  return "unknown";
}

Result<void> check_options(const Options& options) {
  if (!std::isfinite(options.threshold) || options.threshold <= 0.0) {
    return fail_setting("threshold", "must be positive and finite", options.threshold);
  }
  if (std::isnan(options.confidence) || options.confidence <= 0.0 || options.confidence >= 1.0) {
    return fail_setting("confidence", "must be in (0, 1)", options.confidence);
  }
  for (const auto& [name, value] : {std::pair{"max_iterations", options.max_iterations},
                                    std::pair{"lo_steps", options.lo_steps},
                                    std::pair{"lo_sample_factor", options.lo_sample_factor}}) {
    if (value < 1) {
      return fail_setting(name, "must be at least 1", value);
    }
  }
  if (options.lo_inner_iterations < 0) {
    return fail_setting("lo_inner_iterations", "must be non-negative", options.lo_inner_iterations);
  }
  if (!std::isfinite(options.lo_threshold_multiplier) || options.lo_threshold_multiplier < 1.0) {
    return fail_setting("lo_threshold_multiplier", "must be at least 1",
                        options.lo_threshold_multiplier);
  }
  return {};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (w, s, p), as in the formula
double required_iterations(double inlier_ratio, std::uint32_t sample_size, double confidence) {
  const double all_inliers = std::pow(inlier_ratio, static_cast<double>(sample_size));
  if (all_inliers >= 1.0) {
    return 0.0;
  }
  if (all_inliers <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::log1p(-confidence) / std::log1p(-all_inliers);
}

namespace detail {

double score(const Eigen::VectorXd& residuals, double threshold2) {
  return residuals.cwiseMin(threshold2).sum();
}

Indices below(const Eigen::VectorXd& residuals, double threshold2) {
  Indices out;
  for (Eigen::Index i = 0; i < residuals.size(); ++i) {
    if (residuals(i) < threshold2) {
      out.push_back(static_cast<std::uint32_t>(i));
    }
  }
  return out;
}

}  // namespace detail

}  // namespace slamlab::foundations::ransac
