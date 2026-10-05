/// @file
/// @brief Rectified stereo: the C++ core's mirror of `slamlab.foundations.stereo`.
///
/// Bouguet's rectification of a calibrated pair, and the maps that resample each original
/// image into the rectified view (the Python module's docstring defines both).
#pragma once

#include <Eigen/Core>
#include <cstdint>

#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"

/// @brief Rectified stereo.
namespace slamlab::foundations::stereo {

/// @brief Camera centres closer than this count as coinciding.
inline constexpr double kMinBaseline = 1e-9;
/// @brief Where a map sends a pixel whose ray points behind the camera: outside every image.
inline constexpr double kOutside = -2.0;

/// @brief A rectified stereo pair.
struct Rectification {
  /// @brief `R_rect_left`: turns left-camera coordinates into the rectified frame.
  Eigen::Matrix3d rotation_left;
  /// @brief `R_rect_right`: turns right-camera coordinates into the rectified right frame.
  Eigen::Matrix3d rotation_right;
  /// @brief Both rectified cameras' pinhole intrinsics `[f, f, cx, cy]`.
  Eigen::Vector4d intrinsics;
  /// @brief The distance between the camera centres.
  double baseline{0.0};
};

/// @brief Rectify a calibrated pair (Bouguet's algorithm).
/// @param model_left The left camera's model.
/// @param intrinsics_left Its intrinsics.
/// @param model_right The right camera's model.
/// @param intrinsics_right Its intrinsics.
/// @param T_right_left The rigid transform from left-camera to right-camera coordinates.
/// @param rows The images' rows.
/// @param cols The images' columns.
/// @return The rectification; `InvalidInput` for bad intrinsics, a non-rigid transform, a bad
///         shape or a baseline closer to vertical; `Degenerate` if the centres coincide.
[[nodiscard]] Result<Rectification> rectify(
    camera::Model model_left, const Eigen::Ref<const Eigen::VectorXd>& intrinsics_left,
    camera::Model model_right, const Eigen::Ref<const Eigen::VectorXd>& intrinsics_right,
    const Eigen::Matrix4d& T_right_left, std::int64_t rows, std::int64_t cols);

/// @brief Where each rectified pixel samples one original image.
/// @param model The original camera's model.
/// @param intrinsics Its intrinsics.
/// @param rotation That side's rectifying rotation.
/// @param rectified The rectified pinhole intrinsics `[f, f, cx, cy]`.
/// @param rows The images' rows.
/// @param cols The images' columns.
/// @return `(x, y)` per rectified pixel, row by row, for `image::remap`; `kOutside` where the
///         ray points behind the camera. `InvalidInput` for bad intrinsics, a rotation that is
///         not one, or a bad shape.
[[nodiscard]] Result<image::Coordinates> rectification_map(
    camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
    const Eigen::Matrix3d& rotation, const Eigen::Vector4d& rectified, std::int64_t rows,
    std::int64_t cols);

}  // namespace slamlab::foundations::stereo
