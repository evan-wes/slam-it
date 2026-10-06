/// @file
/// @brief Implementation of the frames declared in slamlab/edge/frames.hpp.
#include "slamlab/edge/frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace slamlab::edge {

namespace {

/// @brief NaN, for an unknown depth, column or bearing.
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief Assemble a frame from its features: their bearings, and the depths and columns.
/// @param timestamp The frame's time.
/// @param features Its features.
/// @param camera The camera.
/// @param depth Each keypoint's depth (NaN for none).
/// @param right Each keypoint's right column (NaN for none).
/// @return The frame.
Frame assemble(double timestamp, foundations::orb::Features features, const Camera& camera,
               std::vector<double> depth, std::vector<double> right) {
  std::vector<Eigen::Vector3d> bearings;
  bearings.reserve(features.keypoints.size());
  for (const foundations::orb::Keypoint& k : features.keypoints) {
    bearings.push_back(bearing(camera, {k.xy[0], k.xy[1]}));
  }
  return {.timestamp = timestamp,
          .features = std::move(features),
          .bearings = std::move(bearings),
          .depth = std::move(depth),
          .right = std::move(right)};
}

}  // namespace

double focal(const Camera& camera) {
  return camera.intrinsics(foundations::camera::focal_indices(camera.model)[0]);
}

Eigen::Vector2d project(const Camera& camera, const Eigen::Vector3d& point) {
  const auto pixel = foundations::camera::project(camera.model, camera.intrinsics, point);
  return pixel ? *pixel : Eigen::Vector2d(kNan, kNan);
}

Eigen::Vector3d bearing(const Camera& camera, const Eigen::Vector2d& pixel) {
  const auto ray = foundations::camera::unproject(camera.model, camera.intrinsics, pixel);
  return ray ? *ray : Eigen::Vector3d(kNan, kNan, kNan);
}

bool inside(const Camera& camera, const Eigen::Vector2d& pixel) {
  return pixel.x() >= 0.0 && pixel.x() < static_cast<double>(camera.width) && pixel.y() >= 0.0 &&
         pixel.y() < static_cast<double>(camera.height);
}

foundations::Result<void> check_camera(const Camera& camera) {
  if (auto ok = foundations::camera::check_intrinsics(camera.model, camera.intrinsics); !ok) {
    return ok;
  }
  if (auto ok = foundations::check_range("width", camera.width, 1, foundations::image::kMaxSide);
      !ok) {
    return ok;
  }
  if (auto ok = foundations::check_range("height", camera.height, 1, foundations::image::kMaxSide);
      !ok) {
    return ok;
  }
  if (!std::isfinite(camera.baseline) || camera.baseline < 0.0) {
    return foundations::fail_setting("baseline", "must be finite and non-negative",
                                     camera.baseline);
  }
  return {};
}

std::int64_t inliers(const Tracked& tracked) {
  return std::ranges::count_if(tracked.points, [](std::int64_t p) { return p >= 0; });
}

foundations::Result<std::vector<double>> depth_at(std::span<const std::array<double, 2>> xy,
                                                  const DepthMap& depth_map) {
  if (depth_map.rows < 1 || depth_map.cols < 1 ||
      std::cmp_not_equal(depth_map.values.size(), depth_map.rows * depth_map.cols)) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "depth_map: expected rows x cols values");
  }
  std::vector<double> out;
  out.reserve(xy.size());
  for (const auto& [x, y] : xy) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "xy: expected finite values");
    }
    const auto col = std::clamp(static_cast<std::int64_t>(std::floor(x + 0.5)), std::int64_t{0},
                                depth_map.cols - 1);
    const auto row = std::clamp(static_cast<std::int64_t>(std::floor(y + 0.5)), std::int64_t{0},
                                depth_map.rows - 1);
    const double value = depth_map.values[static_cast<std::size_t>((row * depth_map.cols) + col)];
    out.push_back(std::isfinite(value) && value > 0.0 ? value : kNan);
  }
  return out;
}

foundations::Result<Frame> monocular(const foundations::image::Image& image, double timestamp,
                                     const Camera& camera,
                                     const foundations::orb::Options& options) {
  auto features = foundations::orb::extract(image, options);
  if (!features) {
    return std::unexpected(std::move(features.error()));
  }
  const std::size_t n = features->keypoints.size();
  return assemble(timestamp, std::move(*features), camera, std::vector<double>(n, kNan),
                  std::vector<double>(n, kNan));
}

foundations::Result<Frame> rgbd(const foundations::image::Image& image, const DepthMap& depth_map,
                                double timestamp, const Camera& camera,
                                const foundations::orb::Options& options) {
  if (depth_map.rows != image.rows || depth_map.cols != image.cols) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "depth_map: expected the image's shape");
  }
  if (camera.baseline <= 0.0) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "camera.baseline: an RGB-D camera needs a virtual baseline");
  }
  auto features = foundations::orb::extract(image, options);
  if (!features) {
    return std::unexpected(std::move(features.error()));
  }
  std::vector<std::array<double, 2>> xy;
  xy.reserve(features->keypoints.size());
  for (const foundations::orb::Keypoint& k : features->keypoints) {
    xy.push_back(k.xy);
  }
  auto depth = depth_at(xy, depth_map);
  if (!depth) {
    return std::unexpected(std::move(depth.error()));
  }
  std::vector<double> right(xy.size());
  const double bf = focal(camera) * camera.baseline;
  for (std::size_t i = 0; i < xy.size(); ++i) {
    right[i] = xy[i][0] - (bf / (*depth)[i]);
  }
  return assemble(timestamp, std::move(*features), camera, std::move(*depth), std::move(right));
}

foundations::Result<Frame> stereo_pair(const foundations::image::Image& left,
                                       const foundations::image::Image& right, double timestamp,
                                       const Camera& camera,
                                       const foundations::orb::Options& options,
                                       const foundations::stereo::MatchOptions& stereo_options) {
  if (camera.model != foundations::camera::Model::kPinhole || camera.baseline <= 0.0) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "camera: a stereo pair needs a rectified pinhole camera and baseline");
  }
  auto features = foundations::orb::extract(left, options);
  if (!features) {
    return std::unexpected(std::move(features.error()));
  }
  const auto other = foundations::orb::extract(right, options);
  if (!other) {
    return std::unexpected(other.error());
  }
  const auto found = foundations::stereo::match(left, right, *features, *other, focal(camera),
                                                camera.baseline, stereo_options);
  if (!found) {
    return std::unexpected(found.error());
  }
  const std::size_t n = features->keypoints.size();
  std::vector<double> depth(n, kNan);
  std::vector<double> columns(n, kNan);
  for (const foundations::stereo::StereoMatch& m : *found) {
    depth[static_cast<std::size_t>(m.left)] = m.depth;
    columns[static_cast<std::size_t>(m.left)] = m.right_x;
  }
  return assemble(timestamp, std::move(*features), camera, std::move(depth), std::move(columns));
}

foundations::Result<std::vector<foundations::matching::Match>> search(
    const Frame& frame, std::span<const foundations::matching::Projection> projections,
    const foundations::matching::SearchOptions& options, std::span<const std::uint8_t> taken) {
  auto grid = foundations::matching::FeatureGrid::build(frame.features.keypoints);
  if (!grid) {
    return std::unexpected(std::move(grid.error()));
  }
  const std::vector<std::uint8_t> none(taken.empty() ? keypoint_count(frame) : 0, 0);
  return foundations::matching::search_by_projection(
      projections, frame.features, *grid, frame.right,
      taken.empty() ? std::span<const std::uint8_t>(none) : taken, options);
}

}  // namespace slamlab::edge
