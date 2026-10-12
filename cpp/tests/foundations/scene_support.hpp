/// @file
/// @brief Deterministic synthetic scenes for the native two-view tests.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <utility>

#include "slam_it/foundations/so3.hpp"

/// @brief Helpers for SLAM It!'s native tests.
namespace slam_it::testing {

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

/// @brief The scenes' essential matrix, `[t]x R` with unit t.
/// @return E.
inline Eigen::Matrix3d scene_essential() {
  return foundations::so3::hat(scene_translation().normalized()) * scene_rotation();
}

/// @brief Rows of 3-vectors, row-major as the cores take them.
using Rows3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

/// @brief Exact bearing pairs of the deterministic scene seen through `[R | t]`.
/// @param n How many points.
/// @param planar Put the points on the plane z = 5.
/// @param pose The motion, `(R, t)`.
/// @param b1 Bearings in camera 1 (filled).
/// @param b2 Bearings in camera 2 (filled).
inline void scene_bearings(int n, bool planar,
                           const std::pair<Eigen::Matrix3d, Eigen::Vector3d>& pose, Rows3& b1,
                           Rows3& b2) {
  b1.resize(n, 3);
  b2.resize(n, 3);
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d x = scene_point(i, planar);
    b1.row(i) = x.normalized().transpose();
    b2.row(i) = ((pose.first * x) + pose.second).normalized().transpose();
  }
}

/// @brief The smaller of `|a - b|` and `|a + b|` (entrywise max): equality up to sign.
/// @param a A matrix.
/// @param b A matrix.
/// @return The distance.
inline double up_to_sign(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return std::min((a - b).cwiseAbs().maxCoeff(), (a + b).cwiseAbs().maxCoeff());
}

}  // namespace slam_it::testing
