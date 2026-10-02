/// @file
/// @brief SO(3) helpers for the C++ core.
///
/// A placeholder that proves the three-language pipeline. Rung 0's real SO(3) work arrives
/// with milestone 1.
#pragma once

#include <array>

/// @brief slamlab's C++ core.
namespace slamlab {

/// @brief A 3-vector.
using Vec3 = std::array<double, 3>;

/// @brief A 3x3 matrix, stored row-major as an array of rows.
using Mat3 = std::array<std::array<double, 3>, 3>;

/// @brief Map a 3-vector to its skew-symmetric matrix.
///
/// The result `[v]x` satisfies `hat(v) * u == cross(v, u)` for every 3-vector `u`. It is the
/// map from R^3 into the Lie algebra so(3).
///
/// @param v The vector to map.
/// @return The skew-symmetric matrix `[v]x`.
[[nodiscard]] Mat3 hat(const Vec3& v) noexcept;

}  // namespace slamlab
