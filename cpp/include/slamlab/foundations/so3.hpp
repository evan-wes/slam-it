/// @file
/// @brief SO(3), the group of 3D rotations: the C++ core's implementation.
///
/// A placeholder that proves the three-language pipeline: only `hat` exists so far. Rung 0's
/// SO(3) work (`vee`, `exp`, `log`, Jacobians) arrives with milestone 1. The Python reference
/// is `slamlab.foundations.so3`; parity tests hold this implementation to it.
#pragma once

#include <array>

/// @brief Rung 0 of slamlab's C++ core, foundations: the geometry every other rung builds on.
namespace slamlab::foundations {

/// @brief A 3-vector.
using Vec3 = std::array<double, 3>;

/// @brief A 3x3 matrix, stored row-major as an array of rows.
using Mat3 = std::array<std::array<double, 3>, 3>;

/// @brief Map a 3-vector to its skew-symmetric matrix.
///
/// The result `[v]x` satisfies `hat(v) * u == cross(v, u)` for every 3-vector `u`. It is the
/// map from R^3 into the Lie algebra so(3). Exact: it only copies and negates entries.
///
/// @param v The vector to map. Input validation (finiteness) is the caller's job; the Python
///          entry point validates before calling.
/// @return The skew-symmetric matrix `[v]x`.
[[nodiscard]] Mat3 hat(const Vec3& v) noexcept;

}  // namespace slamlab::foundations
