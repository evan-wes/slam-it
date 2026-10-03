/// @file
/// @brief Deterministic synthetic scenes for the native two-view tests.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

/// @brief Helpers for slamlab's native tests.
namespace slamlab::testing {

/// @brief A deterministic point 4-8 m ahead of camera 1, or on the plane z = 5.
/// @param i The point's index.
/// @param planar Put it on the plane z = 5.
/// @return The point.
inline Eigen::Vector3d scene_point(int i, bool planar) {
  return {2.0 * std::sin(1.3 * i), 2.0 * std::cos(0.7 * i),
          planar ? 5.0 : 6.0 + (2.0 * std::sin(2.9 * i))};
}

/// @brief The scenes' relative rotation: 0.1 rad about a tilted axis.
/// @return R of `T_c2_c1`.
inline Eigen::Matrix3d scene_rotation() {
  return Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.2, -1.0, 0.3).normalized()).toRotationMatrix();
}

/// @brief The scenes' relative translation: half a metre sideways.
/// @return t of `T_c2_c1`.
inline Eigen::Vector3d scene_translation() { return {0.5, -0.1, 0.05}; }

}  // namespace slamlab::testing
