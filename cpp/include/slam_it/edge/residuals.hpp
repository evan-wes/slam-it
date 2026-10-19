/// @file
/// @brief Observation errors: the C++ core's mirror of `slam_it.edge.residuals`.
///
/// A keypoint on pyramid level `l` has the variance `s^(2 l)`; an observation is an outlier
/// when its squared reprojection error over that variance passes the 95% chi-square threshold,
/// with the predicted right column `u - f b / z` compared too where it has one.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"

namespace slam_it::edge {

/// @brief A level's variance, `s^(2 l)` (`slam_it.edge.residuals.sigma2`).
/// @param m The map (its scale factor).
/// @param level The level.
/// @return The variance.
[[nodiscard]] double sigma2(const Map& m, std::int64_t level);

/// @brief An observation's chi-square threshold (`slam_it.edge.residuals.thresholds`).
/// @param right Its right column, or NaN.
/// @param options The thresholds.
/// @return `adjustment.chi2_stereo` with a right column, else `adjustment.chi2_mono`.
[[nodiscard]] double threshold(double right, const Options& options);

/// @brief Each observation's chi-square error (`slam_it.edge.residuals.chi2`).
/// @param camera The camera.
/// @param local The points, in the camera's frame.
/// @param xy Their observed pixels.
/// @param right Their observed right columns (NaN for none).
/// @param variance Their levels' variances.
/// @return `(|pi(X) - x|^2 + (pi_u(X) - f b / Z - u_r)^2) / sigma^2`, infinite for a point
///         not in front; `InvalidInput` for values that are not finite (NaN allowed in
///         `right`), spans of different lengths, or a variance that is not positive.
[[nodiscard]] foundations::Result<std::vector<double>> chi2(
    const Camera& camera, std::span<const Eigen::Vector3d> local,
    std::span<const std::array<double, 2>> xy, std::span<const double> right,
    std::span<const double> variance);

}  // namespace slam_it::edge
