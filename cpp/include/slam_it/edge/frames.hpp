/// @file
/// @brief Frames: the C++ core's mirror of `slam_it.edge.frames`.
///
/// One input image's ORB features, each keypoint's bearing, depth and right column, however
/// the mode produced them (the Python module's docstring states the three modes). Keypoints
/// stay where ORB found them; projections go through the full camera model.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/image.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/stereo.hpp"

namespace slam_it::edge {

/// @brief The camera a sequence was taken with (`slam_it.edge.frames.Camera`).
struct Camera {
  /// @brief The model.
  foundations::camera::Model model{foundations::camera::Model::kPinhole};
  /// @brief Its intrinsics.
  Eigen::VectorXd intrinsics;
  /// @brief The image's width, in pixels.
  std::int64_t width{0};
  /// @brief Its height.
  std::int64_t height{0};
  /// @brief The stereo baseline in metres, or RGB-D's virtual one; 0 for monocular.
  double baseline{0.0};
};

/// @brief The horizontal focal length, in pixels.
/// @param camera The camera.
/// @return It.
[[nodiscard]] double focal(const Camera& camera);

/// @brief Project a camera-frame point, through the full model.
/// @param camera The camera.
/// @param point The point, in front of the camera.
/// @return Its pixel, or NaNs if it cannot project.
[[nodiscard]] Eigen::Vector2d project(const Camera& camera, const Eigen::Vector3d& point);

/// @brief A pixel's unit bearing.
/// @param camera The camera.
/// @param pixel The pixel.
/// @return The bearing, or NaNs where the model cannot unproject it.
[[nodiscard]] Eigen::Vector3d bearing(const Camera& camera, const Eigen::Vector2d& pixel);

/// @brief Whether a pixel lies in the image, `[0, width) x [0, height)`.
/// @param camera The camera.
/// @param pixel The pixel.
/// @return True if inside.
[[nodiscard]] bool inside(const Camera& camera, const Eigen::Vector2d& pixel);

/// @brief Check a camera.
/// @param camera The camera.
/// @return Nothing, or `InvalidInput` for bad intrinsics, a side out of range, or a negative
///         or non-finite baseline.
[[nodiscard]] foundations::Result<void> check_camera(const Camera& camera);

/// @brief Check a camera and the settings, as every stage does first.
/// @param camera The camera.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` from `check_camera` or `check_options`.
[[nodiscard]] foundations::Result<void> check_setup(const Camera& camera, const Options& options);

/// @brief One frame's features, as tracking and mapping read them
/// (`slam_it.edge.frames.Frame`).
struct Frame {
  /// @brief Its time, in seconds.
  double timestamp{0.0};
  /// @brief Its ORB features.
  foundations::orb::Features features;
  /// @brief Each keypoint's unit bearing (NaN where it cannot be unprojected).
  std::vector<Eigen::Vector3d> bearings;
  /// @brief Each keypoint's depth, in metres (NaN where unknown).
  std::vector<double> depth;
  /// @brief Each keypoint's right-image column (NaN where its depth is unknown).
  std::vector<double> right;
  /// @brief Its keypoints bucketed for search by projection, built with the frame as ORB-SLAM's
  ///        `Frame` assigns its features to a grid (the compiled cores' search structure; Python's
  ///        search needs none). `make_frame` builds it; a frame's keypoints are not changed
  ///        after.
  foundations::matching::FeatureGrid grid;
};

/// @brief A frame from its parts, its feature grid built from its keypoints: how every frame is
///        made (the builders, the bindings, the tests), so none lacks its grid. It does not
///        check that the parts hold one entry per keypoint; a search's `check_frame` does.
/// @param timestamp Its time, in seconds.
/// @param features Its ORB features.
/// @param bearings Each keypoint's unit bearing.
/// @param depth Each keypoint's depth (NaN where unknown).
/// @param right Each keypoint's right-image column (NaN where unknown).
/// @return The frame; `InvalidInput` from the grid for a keypoint position that is not finite
///         (never one extracted).
[[nodiscard]] foundations::Result<Frame> make_frame(double timestamp,
                                                    foundations::orb::Features features,
                                                    std::vector<Eigen::Vector3d> bearings,
                                                    std::vector<double> depth,
                                                    std::vector<double> right);

/// @brief A frame's number of keypoints.
/// @param frame The frame.
/// @return It.
[[nodiscard]] inline std::size_t keypoint_count(const Frame& frame) {
  return frame.features.keypoints.size();
}

/// @brief A frame placed against the map (`slam_it.edge.frames.Tracked`).
struct Tracked {
  /// @brief Its `T_cam_world`.
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  /// @brief The id of the map point each keypoint matched, or -1.
  std::vector<std::int64_t> points;
};

/// @brief How many of a tracked frame's keypoints matched a map point.
/// @param tracked The tracked frame.
/// @return The count.
[[nodiscard]] std::int64_t inliers(const Tracked& tracked);

/// @brief A depth map in metres (0 or NaN where the sensor measured nothing).
struct DepthMap {
  /// @brief Rows.
  std::int64_t rows{0};
  /// @brief Columns.
  std::int64_t cols{0};
  /// @brief The depths, row by row.
  std::vector<double> values;
};

/// @brief Read a depth map at each keypoint's nearest pixel, half up
///        (`slam_it.edge.frames.depth_at`).
/// @param xy The keypoints' pixels.
/// @param depth_map The depth map.
/// @return Each depth, positive, or NaN; `InvalidInput` for a map whose values do not fill
///         its shape, or a pixel that is not finite.
[[nodiscard]] foundations::Result<std::vector<double>> depth_at(
    std::span<const std::array<double, 2>> xy, const DepthMap& depth_map);

/// @brief Build a monocular frame: features and bearings, no depth.
/// @param image The gray image.
/// @param timestamp Its time.
/// @param camera The camera.
/// @param options The extraction settings.
/// @return The frame, or the extraction's error.
[[nodiscard]] foundations::Result<Frame> monocular(const foundations::image::Image& image,
                                                   double timestamp, const Camera& camera,
                                                   const foundations::orb::Options& options);

/// @brief Build an RGB-D frame: each keypoint's depth and virtual right column.
/// @param image The gray image.
/// @param depth_map The registered depth.
/// @param timestamp Its time.
/// @param camera The camera, with its virtual baseline.
/// @param options The extraction settings.
/// @return The frame; `InvalidInput` for a depth map of another shape, or no baseline.
[[nodiscard]] foundations::Result<Frame> rgbd(const foundations::image::Image& image,
                                              const DepthMap& depth_map, double timestamp,
                                              const Camera& camera,
                                              const foundations::orb::Options& options);

/// @brief Build a stereo frame from a rectified pair: left features, matched along rows.
/// @param left The rectified left image.
/// @param right The rectified right image.
/// @param timestamp Their time.
/// @param camera The rectified left camera (pinhole) and the pair's baseline.
/// @param options The extraction settings.
/// @param stereo_options The row matching's settings.
/// @return The frame; `InvalidInput` for a camera that is not pinhole or has no baseline.
[[nodiscard]] foundations::Result<Frame> stereo_pair(
    const foundations::image::Image& left, const foundations::image::Image& right, double timestamp,
    const Camera& camera, const foundations::orb::Options& options,
    const foundations::stereo::MatchOptions& stereo_options);

/// @brief Match projected map points to a frame's keypoints, its right columns checked
///        (`slam_it.edge.frames.search`): `foundations::matching::search_by_projection` over
///        the frame's grid and right columns (NaN for a monocular frame: no check).
/// @param frame The frame.
/// @param projections The projected points.
/// @param options The filters.
/// @param taken One flag per keypoint, nonzero if already matched; empty for none.
/// @return `(projection, keypoint)` matches; the search's `InvalidInput`.
[[nodiscard]] foundations::Result<std::vector<foundations::matching::Match>> search(
    const Frame& frame, std::span<const foundations::matching::Projection> projections,
    const foundations::matching::SearchOptions& options, std::span<const std::uint8_t> taken = {});

}  // namespace slam_it::edge
