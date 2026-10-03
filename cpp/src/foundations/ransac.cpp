/// @file
/// @brief The non-template parts of RANSAC declared in slamlab/foundations/ransac.hpp.
#include "slamlab/foundations/ransac.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

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
    return fail(ErrorKind::kInvalidInput, "threshold: must be positive and finite");
  }
  if (std::isnan(options.confidence) || options.confidence <= 0.0 || options.confidence >= 1.0) {
    return fail(ErrorKind::kInvalidInput, "confidence: must be in (0, 1)");
  }
  if (options.max_iterations < 1) {
    return fail(ErrorKind::kInvalidInput, "max_iterations: must be at least 1");
  }
  if (options.lo_steps < 1 || options.lo_inner_iterations < 0 || options.lo_sample_factor < 1) {
    return fail(ErrorKind::kInvalidInput,
                "lo_steps, lo_inner_iterations, lo_sample_factor: out of range");
  }
  if (!std::isfinite(options.lo_threshold_multiplier) || options.lo_threshold_multiplier < 1.0) {
    return fail(ErrorKind::kInvalidInput, "lo_threshold_multiplier: must be at least 1");
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
