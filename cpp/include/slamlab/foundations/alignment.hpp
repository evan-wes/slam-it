/// @file
/// @brief Point-set alignment (Umeyama): the C++ core's implementation.
///
/// Mirrors `slamlab.foundations.alignment`: the rotation, translation and optional scale
/// minimising `Σ w_i |y_i - (s R x_i + t)|²` (Umeyama, TPAMI 1991).
#pragma once

#include <Eigen/Core>

#include "slamlab/foundations/error.hpp"

/// @brief Point-set alignment.
namespace slamlab::foundations::alignment {

/// @brief The cross-covariance's second singular value must exceed this fraction of its
///        first; below it the points are collinear and the rotation is undetermined.
inline constexpr double kDegeneracyRatio = 1e-10;

/// @brief Align corresponding point sets.
/// @param source Source points, one per row (N x 3), N >= 3.
/// @param target Target points, one per row (N x 3).
/// @param weights Non-negative weights (N), at least three positive.
/// @param with_scale Estimate a scale too (Sim(3)).
/// @return The 4x4 matrix `[[s R, t], [0, 1]]`; `InvalidInput` for mismatched sizes or
///         a non-finite value or negative weights; `Degenerate` for fewer than three weighted
///         points or collinear points.
[[nodiscard]] Result<Eigen::Matrix4d> umeyama(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>& source,
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>& target,
    const Eigen::Ref<const Eigen::VectorXd>& weights, bool with_scale);

}  // namespace slamlab::foundations::alignment
