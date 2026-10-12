/// @file
/// @brief Camera models: the C++ core's projection, Jacobians and unprojection.
///
/// Mirrors `slam_it.foundations.camera`. Points are in the camera frame (z forward); pixels
/// follow OpenCV (top-left pixel centre at (0, 0)); intrinsics are flat vectors:
/// pinhole `[fx, fy, cx, cy]`, radtan `[fx, fy, cx, cy, k1, k2, p1, p2, k3]`, radial
/// `[f, cx, cy, k1, k2]`.
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <span>

#include "slam_it/foundations/error.hpp"

/// @brief Camera models.
namespace slam_it::foundations::camera {

/// @brief The camera models, with the integer codes they cross the bindings as.
enum class Model : std::uint8_t {
  /// @brief `[fx, fy, cx, cy]`.
  kPinhole = 0,
  /// @brief `[fx, fy, cx, cy, k1, k2, p1, p2, k3]`, Brown-Conrady radial-tangential.
  kRadTan = 1,
  /// @brief `[f, cx, cy, k1, k2]`, COLMAP's RADIAL and BAL's model: radtan with `fx = fy = f`
  ///        and no tangential or k3 terms, computed through radtan.
  kRadial = 2,
  /// @brief The number of models (a sentinel, not a model): the bindings' name tables are
  ///        checked against it, so an enumerator added without its name fails to build.
  kCount = 3,
};

/// @brief The length of a model's intrinsics (Rust's `Model::intrinsics_len`).
/// @param model The model.
/// @return 4 (pinhole), 9 (radtan) or 5 (radial); 0 for `kCount`. Every model is a case, so a
///         model added without its length fails to build (`-Wswitch`), as Rust's `match` does.
[[nodiscard]] constexpr Eigen::Index intrinsics_size(Model model) {
  switch (model) {
    case Model::kPinhole:
      return 4;
    case Model::kRadTan:
      return 9;
    case Model::kRadial:
      return 5;
    case Model::kCount:
      break;
  }
  return 0;
}

/// @brief The most intrinsics any model has: what a fixed-size buffer of a model's
///        intrinsics, or of a Jacobian over them, must hold.
inline constexpr Eigen::Index kMaxIntrinsicsSize = [] {
  Eigen::Index most = 0;
  for (int m = 0; m < static_cast<int>(Model::kCount); ++m) {
    most = std::max(most, intrinsics_size(static_cast<Model>(m)));
  }
  return most;
}();

/// @brief Points with `Z` at or below this are behind the camera and cannot project.
inline constexpr double kMinDepth = 1e-9;
/// @brief Newton iterations allowed when inverting the radtan distortion.
inline constexpr int kUndistortMaxIterations = 30;
/// @brief Newton stops when the distortion residual is below this (normalised units).
inline constexpr double kUndistortTolerance = 1e-15;

/// @brief A 2 x (3 + K) Jacobian: d(pixel)/d(point) then d(pixel)/d(intrinsics).
using ProjectionJacobian = Eigen::Matrix<double, 2, Eigen::Dynamic>;

/// @brief The model's name, as `slam_it.foundations.camera.MODELS` spells it.
/// @param model The model.
/// @return A static string.
[[nodiscard]] const char* name(Model model);

/// @brief The model's focal-length entries in its intrinsics vector
///        (`slam_it.foundations.camera.FOCAL_INDICES`).
/// @param model The model.
/// @return Their indices: `{0, 1}`, or `{0}` for the radial model.
[[nodiscard]] std::span<const Eigen::Index> focal_indices(Model model);

/// @brief Check a model code and its intrinsics vector.
/// @param model The model.
/// @param intrinsics The intrinsics.
/// @return Nothing on success, or `InvalidInput` (wrong length, non-positive focal length).
[[nodiscard]] Result<void> check_intrinsics(Model model,
                                            const Eigen::Ref<const Eigen::VectorXd>& intrinsics);

/// @brief Project a camera-frame point to a pixel.
/// @param model The model.
/// @param intrinsics The model's intrinsics (checked by `check_intrinsics`).
/// @param point The point, with `Z > kMinDepth`.
/// @return The pixel `(u, v)`, or `InvalidInput` if the point is not in front of the camera.
[[nodiscard]] Result<Eigen::Vector2d> project(Model model,
                                              const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                              const Eigen::Vector3d& point);

/// @brief Differentiate the projection with respect to the point and the intrinsics.
/// @param model The model.
/// @param intrinsics The model's intrinsics.
/// @param point The point, with `Z > kMinDepth`.
/// @return The 2 x (3 + K) Jacobian, or `InvalidInput` if the point is not in front.
[[nodiscard]] Result<ProjectionJacobian> project_jacobian(
    Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics, const Eigen::Vector3d& point);

/// @brief Map a pixel to a unit bearing vector.
/// @param model The model.
/// @param intrinsics The model's intrinsics.
/// @param pixel The pixel `(u, v)`.
/// @return The bearing, or `NotConverged` if the distortion cannot be inverted there.
[[nodiscard]] Result<Eigen::Vector3d> unproject(Model model,
                                                const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                                const Eigen::Vector2d& pixel);

}  // namespace slam_it::foundations::camera
