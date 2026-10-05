/// @file
/// @brief Rectified stereo: the C++ core's mirror of `slamlab.foundations.stereo`.
///
/// Bouguet's rectification of a calibrated pair, the maps that resample each original image
/// into the rectified view, and ORB-SLAM's matching along rectified rows (the Python module's
/// docstring defines each).
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"
#include "slamlab/foundations/orb.hpp"

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

/// @brief Stereo-matching settings (`slamlab.foundations.stereo.Options`, ORB-SLAM's).
struct MatchOptions {
  /// @brief A match's Hamming distance must be below this, from 1 to 257.
  std::int64_t max_distance{75};
  /// @brief The row band, in multiples of the right keypoint's level scale, positive.
  double band{2.0};
  /// @brief The SAD patch's half-width, from 1 to `orb::kEdgeThreshold`.
  std::int64_t window{5};
  /// @brief The SAD search's half-range, from 2 to 64.
  std::int64_t search{5};
  /// @brief Matches whose best sum exceeds this times the median are dropped; at least 1.
  double outlier_factor{2.1};
};

/// @brief One left keypoint matched along its row.
struct StereoMatch {
  /// @brief The left keypoint's index.
  std::int64_t left{0};
  /// @brief The right keypoint's index.
  std::int64_t right{0};
  /// @brief The refined right column, in full-image pixels.
  double right_x{0.0};
  /// @brief `x_left - right_x`, positive.
  double disparity{0.0};
  /// @brief `focal baseline / disparity`.
  double depth{0.0};
};

/// @brief Check stereo-matching settings.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] Result<void> check_options(const MatchOptions& options);

/// @brief Match ORB features along the rows of a rectified pair (ORB-SLAM's stereo matching).
/// @param left_image The rectified left image.
/// @param right_image The rectified right image, of the same shape.
/// @param left The left features, extracted with `orb_options`.
/// @param right The right features.
/// @param focal The rectified focal length, in pixels.
/// @param baseline The distance between the camera centres.
/// @param orb_options The settings the features were extracted with.
/// @param options The matching settings.
/// @return The matches, by left index; `InvalidInput` for malformed images, features or
///         settings, or a left keypoint closer than `options.window` to its level's border.
[[nodiscard]] Result<std::vector<StereoMatch>> match(
    const image::Image& left_image, const image::Image& right_image, const orb::Features& left,
    const orb::Features& right, double focal, double baseline, const orb::Options& orb_options,
    const MatchOptions& options);

}  // namespace slamlab::foundations::stereo
