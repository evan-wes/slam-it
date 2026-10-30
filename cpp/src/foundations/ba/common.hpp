/// @file
/// @brief Bundle adjustment internals: index types shared by the layout, Schur system and
///        problem.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "slam_it/foundations/ba.hpp"
#include "slam_it/foundations/camera.hpp"

/// @brief Bundle adjustment's internals (not part of the public API).
namespace slam_it::foundations::ba::detail {

using Eigen::Index;
/// @brief A list of indices into a vector or matrix.
using IndexList = std::vector<Index>;
/// @brief The most rows a residual block has: a pose prior's six.
inline constexpr Index kMaxBlockRows = kPoseDim;
/// @brief The most intrinsics a camera model has (radtan's nine, today).
inline constexpr Index kMaxIntrinsics = camera::kMaxIntrinsicsSize;
/// @brief The most camera-side columns a residual block touches: its camera's pose and
///        intrinsics, and an anchored point's host's.
inline constexpr Index kMaxBlockColumns = 2 * (kPoseDim + kMaxIntrinsics);
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

/// @brief Invert a rigid pose, `T_world_cam` to `T_cam_world` and back: `R^T`, `-R^T t`.
/// @param pose The pose.
/// @return Its inverse, as the bundle's state holds it.
inline Eigen::Matrix4d inverse_pose(const Eigen::Matrix4d& pose) {
  Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
  inv.topLeftCorner<3, 3>() = pose.topLeftCorner<3, 3>().transpose();
  inv.topRightCorner<3, 1>() = -inv.topLeftCorner<3, 3>() * pose.topRightCorner<3, 1>();
  return inv;
}

/// @brief A world point in a camera's frame, `R p + t`: the one computation both the layout's
///        in-front test and the bundle's residuals take a point's depth from, so that an
///        observation the projection would refuse (`Z <= camera::kMinDepth`) is never kept.
/// @param t_cam_world The camera's world-to-camera pose.
/// @param point The point in the world frame.
/// @return The point in the camera frame.
inline Eigen::Vector3d to_camera(const Eigen::Matrix4d& t_cam_world, const Eigen::Vector3d& point) {
  return (t_cam_world.topLeftCorner<3, 3>() * point) + t_cam_world.topRightCorner<3, 1>();
}

}  // namespace slam_it::foundations::ba::detail
