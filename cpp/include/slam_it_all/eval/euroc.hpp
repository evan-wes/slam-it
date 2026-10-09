/// @file
/// @brief EuRoC sequences, as a run reads them: the C++ mirror of
///        `slam_it_all.eval.euroc.read_frames` and `read_camera` (the module's docstring states
///        the format). Ground truth stays Python's: a run never reads it.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "slam_it_all/foundations/error.hpp"

/// @brief The EuRoC MAV datasets.
namespace slam_it_all::eval::euroc {

/// @brief The data files' timestamps per second.
inline constexpr double kNanoseconds = 1e9;

/// @brief One camera (`slam_it_all.eval.euroc.Camera`).
struct Camera {
  /// @brief `radtan` intrinsics `[fx, fy, cx, cy, k1, k2, p1, p2, 0]`.
  Eigen::VectorXd intrinsics;
  /// @brief The camera's pose in the body frame.
  Eigen::Matrix4d T_body_cam{Eigen::Matrix4d::Identity()};  // NOLINT(readability-identifier-naming)
  /// @brief Its images' rows.
  std::int64_t rows{0};
  /// @brief Their columns.
  std::int64_t cols{0};
};

/// @brief A sequence's stereo frames, as a run reads them (`slam_it_all.eval.euroc.Frames`).
struct Frames {
  /// @brief The sequence folder (the one holding `mav0`).
  std::string root;
  /// @brief Each frame's timestamp, in seconds, ascending.
  std::vector<double> timestamps;
  /// @brief Each frame's left (`cam0`) image.
  std::vector<std::string> left;
  /// @brief Its right (`cam1`) image.
  std::vector<std::string> right;
  /// @brief The left camera.
  Camera left_camera;
  /// @brief The right camera.
  Camera right_camera;
};

/// @brief Read a camera's `sensor.yaml` (`slam_it_all.eval.euroc.read_camera`).
/// @param path The file.
/// @return The camera; `InvalidInput` for a missing file, a missing or malformed field, a
///         camera that is not a radial-tangential pinhole, or a `T_BS` that is not rigid.
[[nodiscard]] foundations::Result<Camera> read_camera(const std::string& path);

/// @brief Read a sequence's stereo frames (`slam_it_all.eval.euroc.read_frames`): every frame
///        with both images, in time order.
/// @param root The sequence folder.
/// @return The frames; `InvalidInput` for a missing or malformed file, or no frame with both
///         images.
[[nodiscard]] foundations::Result<Frames> read_frames(const std::string& root);

/// @brief The rigid transform from left-camera to right-camera coordinates
///        (`slam_it_all.eval.euroc.right_from_left`).
/// @param left The left camera.
/// @param right The right camera.
/// @return `T_right_left`, or the inversion's error.
[[nodiscard]] foundations::Result<Eigen::Matrix4d> right_from_left(const Camera& left,
                                                                   const Camera& right);

}  // namespace slam_it_all::eval::euroc
