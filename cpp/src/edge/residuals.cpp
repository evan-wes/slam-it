/// @file
/// @brief Implementation of the observation errors declared in slamlab/edge/residuals.hpp.
#include "slamlab/edge/residuals.hpp"

#include <cmath>
#include <limits>

namespace slamlab::edge {

double sigma2(const Map& m, std::int64_t level) {
  return std::pow(m.scale_factor, 2.0 * static_cast<double>(level));
}

double threshold(double right, const Options& options) {
  return std::isnan(right) ? options.chi2_mono : options.chi2_stereo;
}

foundations::Result<std::vector<double>> chi2(const Camera& camera,
                                              std::span<const Eigen::Vector3d> local,
                                              std::span<const std::array<double, 2>> xy,
                                              std::span<const double> right,
                                              std::span<const double> variance) {
  const std::size_t n = local.size();
  if (xy.size() != n || right.size() != n || variance.size() != n) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "chi2: expected one row per observation in every array");
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!local[i].allFinite() || !std::isfinite(xy[i][0]) || !std::isfinite(xy[i][1]) ||
        std::isinf(right[i]) || !std::isfinite(variance[i])) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput,
                               "chi2: expected finite values (NaN allowed in right)");
    }
    if (variance[i] <= 0.0) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "variance: must be positive");
    }
  }
  const double bf = focal(camera) * camera.baseline;
  std::vector<double> out(n, std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < n; ++i) {
    if (!(local[i].z() > 0.0)) {
      continue;
    }
    const Eigen::Vector2d pixel = project(camera, local[i]);
    const double dx = pixel.x() - xy[i][0];
    const double dy = pixel.y() - xy[i][1];
    double squared = (dx * dx) + (dy * dy);
    if (!std::isnan(right[i])) {
      const double dr = pixel.x() - (bf / local[i].z()) - right[i];
      squared += dr * dr;
    }
    out[i] = squared / variance[i];
  }
  return out;
}

}  // namespace slamlab::edge
