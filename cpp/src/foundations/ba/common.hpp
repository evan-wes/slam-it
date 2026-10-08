/// @file
/// @brief Bundle adjustment internals: index types shared by the layout, Schur system and
///        problem.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

/// @brief Bundle adjustment's internals (not part of the public API).
namespace slamlab::foundations::ba::detail {

using Eigen::Index;
/// @brief A list of indices into a vector or matrix.
using IndexList = std::vector<Index>;
/// @brief The most rows a residual block has: a pose prior's six.
inline constexpr Index kMaxBlockRows = 6;
/// @brief The most intrinsics a camera model has: radtan's nine.
inline constexpr Index kMaxIntrinsics = 9;
/// @brief The most camera-side columns a residual block touches: its camera's pose and
///        intrinsics, and an anchored point's host's.
inline constexpr Index kMaxBlockColumns = 2 * (6 + kMaxIntrinsics);
/// @brief A residual block's camera-side Jacobian: bounded, so it lives on the stack.
using Block = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, kMaxBlockRows,
                            kMaxBlockColumns>;
/// @brief A residual block's residual.
using BlockResidual = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, kMaxBlockRows, 1>;
/// @brief A residual block's point Jacobian.
using BlockPoint = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::ColMajor, kMaxBlockRows, 3>;

/// @brief Convert a stored index to an `Eigen::Index`.
/// @param value The index.
/// @return The same value.
inline Index at(std::int64_t value) { return static_cast<Index>(value); }

}  // namespace slamlab::foundations::ba::detail
