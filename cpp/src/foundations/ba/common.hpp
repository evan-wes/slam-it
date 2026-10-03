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
/// @brief A camera-side Jacobian or coupling block: 2 or c rows by c or 3 columns.
using Block = Eigen::MatrixXd;

/// @brief Convert a stored index to an `Eigen::Index`.
/// @param value The index.
/// @return The same value.
inline Index at(std::int64_t value) { return static_cast<Index>(value); }

}  // namespace slamlab::foundations::ba::detail
