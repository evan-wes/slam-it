/// @file
/// @brief TUM-format sequence folders, as a run reads them: the C++ mirror of
///        `slamlab.eval.tum.read_frames` and `read_camera` (the module's docstring states the
///        format, `camera.txt` included). Ground truth stays Python's: a run never reads it.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/error.hpp"

/// @brief The TUM RGB-D benchmark's folders, and others in its layout.
namespace slamlab::eval::tum {

/// @brief A depth map's value per metre of z, in the benchmark.
inline constexpr double kDepthScale = 5000.0;

/// @brief The largest frame-to-depth-map time difference accepted, in seconds.
inline constexpr double kMaxDifference = 0.02;

/// @brief The benchmark's images' width.
inline constexpr std::int64_t kWidth = 640;

/// @brief Their height.
inline constexpr std::int64_t kHeight = 480;

/// @brief The file a TUM-format folder describes its own camera in.
inline constexpr const char* kCameraFile = "camera.txt";

/// @brief Every freiburg sensor's virtual stereo baseline times focal length, in pixel-metres
///        (`slamlab.eval.tum.SENSOR_BF`: ORB-SLAM2's TUM RGB-D settings' `Camera.bf`).
inline constexpr double kSensorBf = 40.0;

/// @brief A folder's colour camera (`slamlab.eval.tum.Camera`).
struct Camera {
  /// @brief The camera model.
  foundations::camera::Model model{foundations::camera::Model::kPinhole};
  /// @brief Its intrinsics.
  Eigen::VectorXd intrinsics;
  /// @brief The images' width, in pixels.
  std::int64_t width{0};
  /// @brief Their height.
  std::int64_t height{0};
  /// @brief A depth map's value per metre of z.
  double depth_scale{kDepthScale};
  /// @brief The depth sensor's virtual stereo baseline times the focal length, in pixel-metres
  ///        (ORB-SLAM's `Camera.bf`); none if `camera.txt` gives none (no RGB-D run).
  std::optional<double> bf;
};

/// @brief A folder's colour frames, as a run reads them (`slamlab.eval.tum.Frames`).
struct Frames {
  /// @brief The folder.
  std::string root;
  /// @brief Each frame's timestamp, in seconds, in `rgb.txt`'s order (non-decreasing).
  std::vector<double> timestamps;
  /// @brief Each frame's colour image.
  std::vector<std::string> images;
  /// @brief Each frame's nearest depth map within the tolerance, if any.
  std::vector<std::optional<std::string>> depths;
  /// @brief The camera.
  Camera camera;
};

/// @brief A folder's depth map in metres (`slamlab.eval.tum.read_depth`).
struct Depth {
  /// @brief Its rows.
  std::int64_t rows{0};
  /// @brief Its columns.
  std::int64_t cols{0};
  /// @brief Row-major z in metres; NaN where nothing was measured.
  std::vector<double> values;
};

/// @brief Read a folder's camera: its `camera.txt`, or the sensor its name says
///        (`slamlab.eval.tum.read_camera`).
/// @param root The folder.
/// @return The camera; `InvalidInput` for a malformed `camera.txt`, or none and no sensor in
///         the name.
[[nodiscard]] foundations::Result<Camera> read_camera(const std::string& root);

/// @brief Read a folder's colour frames, each with its nearest depth map
///        (`slamlab.eval.tum.read_frames`).
/// @param root The folder.
/// @param max_difference The largest frame-to-depth-map time difference, in seconds.
/// @return The frames; `InvalidInput` for a missing `rgb.txt`, a malformed file (named with
///         its line), a negative `max_difference`, or an unreadable camera.
[[nodiscard]] foundations::Result<Frames> read_frames(const std::string& root,
                                                      double max_difference = kMaxDifference);

/// @brief Read a depth map in metres (`slamlab.eval.tum.read_depth`).
/// @param path A 16-bit depth PNG.
/// @param depth_scale Its value per metre.
/// @return The depths, NaN for a stored 0; `InvalidInput` for an unreadable file or a
///         `depth_scale` that is not finite and positive.
[[nodiscard]] foundations::Result<Depth> read_depth(const std::string& path,
                                                    double depth_scale = kDepthScale);

}  // namespace slamlab::eval::tum
