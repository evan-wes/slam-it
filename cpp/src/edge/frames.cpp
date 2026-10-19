/// @file
/// @brief Implementation of the frames declared in slam_it/edge/frames.hpp.
#include "slam_it/edge/frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace slam_it::edge {

namespace {

/// @brief NaN, for an unknown depth, column or bearing.
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief Assemble a frame from its features: their bearings and grid, and the depths and
///        columns.
/// @param timestamp The frame's time.
/// @param features Its features.
/// @param camera The camera.
/// @param depth Each keypoint's depth (NaN for none).
/// @param right Each keypoint's right column (NaN for none).
/// @return The frame; `InvalidInput` from the grid (unreachable for extracted features).
foundations::Result<Frame> assemble(double timestamp, foundations::orb::Features features,
                                    const Camera& camera, std::vector<double> depth,
                                    std::vector<double> right) {
  std::vector<Eigen::Vector3d> bearings;
  bearings.reserve(features.keypoints.size());
  for (const foundations::orb::Keypoint& k : features.keypoints) {
    bearings.push_back(bearing(camera, {k.xy[0], k.xy[1]}));
  }
  return make_frame(timestamp, std::move(features), std::move(bearings), std::move(depth),
                    std::move(right));
}

/// @brief The features' keypoints' pixels, in order.
/// @param features The features.
/// @return Each keypoint's `xy`.
std::vector<std::array<double, 2>> pixels_of(const foundations::orb::Features& features) {
  std::vector<std::array<double, 2>> xy;
  xy.reserve(features.keypoints.size());
  for (const foundations::orb::Keypoint& k : features.keypoints) {
    xy.push_back(k.xy);
  }
  return xy;
}

/// @brief Drop the features a mask covers (`unmasked`), keeping each feature's keypoint and
///        descriptor together; every feature without a mask.
/// @param features The features, extracted from an image of `rows` by `cols`.
/// @param mask The mask, or null for none.
/// @param rows The image's rows.
/// @param cols The image's columns.
/// @return The kept features; `InvalidInput` for a mask of another shape than the image's.
foundations::Result<foundations::orb::Features> masked(foundations::orb::Features features,
                                                       const foundations::image::Image* mask,
                                                       std::int64_t rows, std::int64_t cols) {
  if (mask == nullptr) {
    return features;
  }
  if (mask->rows != rows || mask->cols != cols) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "mask: expected the image's shape");
  }
  const auto kept = unmasked(pixels_of(features), *mask);
  if (!kept) {
    return std::unexpected(kept.error());
  }
  foundations::orb::Features out{.keypoints = {},
                                 .descriptors = {},
                                 .scale_factor = features.scale_factor,
                                 .levels = features.levels};
  out.keypoints.reserve(kept->size());
  out.descriptors.reserve(kept->size());
  for (const std::int64_t i : *kept) {
    out.keypoints.push_back(features.keypoints[static_cast<std::size_t>(i)]);
    out.descriptors.push_back(features.descriptors[static_cast<std::size_t>(i)]);
  }
  return out;
}

/// @brief Extract an image's features, then drop those a mask covers (`masked`).
/// @param image The image.
/// @param options The extraction settings.
/// @param mask The mask, or null for none.
/// @return The features; the extraction's error, or `InvalidInput` for a mask of another shape.
foundations::Result<foundations::orb::Features> extract_masked(
    const foundations::image::Image& image, const foundations::orb::Options& options,
    const foundations::image::Image* mask) {
  return foundations::orb::extract(image, options).and_then([&](foundations::orb::Features f) {
    return masked(std::move(f), mask, image.rows, image.cols);
  });
}

}  // namespace

foundations::Result<Frame> make_frame(double timestamp, foundations::orb::Features features,
                                      std::vector<Eigen::Vector3d> bearings,
                                      std::vector<double> depth, std::vector<double> right) {
  auto grid = foundations::matching::FeatureGrid::build(features.keypoints);
  if (!grid) {
    return std::unexpected(std::move(grid.error()));
  }
  return Frame{.timestamp = timestamp,
               .features = std::move(features),
               .bearings = std::move(bearings),
               .depth = std::move(depth),
               .right = std::move(right),
               .grid = std::move(*grid)};
}

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

foundations::Result<void> check_setup(const Camera& camera, const Options& options) {
  if (auto ok = check_camera(camera); !ok) {
    return ok;
  }
  return check_options(options);
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

foundations::Result<std::vector<std::int64_t>> unmasked(std::span<const std::array<double, 2>> xy,
                                                        const foundations::image::Image& mask) {
  if (mask.rows < 1 || mask.cols < 1 ||
      std::cmp_not_equal(mask.pixels.size(), mask.rows * mask.cols)) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "mask: expected rows x cols pixels");
  }
  std::vector<std::int64_t> out;
  for (std::size_t i = 0; i < xy.size(); ++i) {
    const auto [x, y] = xy[i];
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return foundations::fail(foundations::ErrorKind::kInvalidInput, "xy: expected finite values");
    }
    const auto col =
        std::clamp(static_cast<std::int64_t>(std::floor(x + 0.5)), std::int64_t{0}, mask.cols - 1);
    const auto row =
        std::clamp(static_cast<std::int64_t>(std::floor(y + 0.5)), std::int64_t{0}, mask.rows - 1);
    if (foundations::image::pixel(mask, row, col) != 0) {
      out.push_back(static_cast<std::int64_t>(i));
    }
  }
  return out;
}

foundations::Result<Frame> monocular(const foundations::image::Image& image, double timestamp,
                                     const Camera& camera, const foundations::orb::Options& options,
                                     const foundations::image::Image* mask) {
  auto features = extract_masked(image, options, mask);
  if (!features) {
    return std::unexpected(std::move(features.error()));
  }
  const std::size_t n = features->keypoints.size();
  return assemble(timestamp, std::move(*features), camera, std::vector<double>(n, kNan),
                  std::vector<double>(n, kNan));
}

foundations::Result<Frame> rgbd(const foundations::image::Image& image, const DepthMap& depth_map,
                                double timestamp, const Camera& camera,
                                const foundations::orb::Options& options,
                                const foundations::image::Image* mask) {
  if (depth_map.rows != image.rows || depth_map.cols != image.cols) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "depth_map: expected the image's shape");
  }
  if (camera.baseline <= 0.0) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "camera.baseline: an RGB-D camera needs a virtual baseline");
  }
  auto features = extract_masked(image, options, mask);
  if (!features) {
    return std::unexpected(std::move(features.error()));
  }
  const std::vector<std::array<double, 2>> xy = pixels_of(*features);
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
                                       const foundations::stereo::MatchOptions& stereo_options,
                                       const foundations::image::Image* mask) {
  if (camera.model != foundations::camera::Model::kPinhole || camera.baseline <= 0.0) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             "camera: a stereo pair needs a rectified pinhole camera and baseline");
  }
  // Each image's pyramid, built once: extraction and the row matching both read it.
  const auto left_levels = foundations::image::pyramid(left, options.levels, options.scale_factor);
  const auto right_levels =
      foundations::image::pyramid(right, options.levels, options.scale_factor);
  if (!left_levels || !right_levels) {
    return std::unexpected(left_levels ? right_levels.error() : left_levels.error());
  }
  auto features = foundations::orb::extract_levels(*left_levels, options)
                      .and_then([&](foundations::orb::Features f) {
                        return masked(std::move(f), mask, left.rows, left.cols);
                      });
  if (!features) {
    return std::unexpected(std::move(features.error()));
  }
  const auto other = foundations::orb::extract_levels(*right_levels, options);
  if (!other) {
    return std::unexpected(other.error());
  }
  const auto found = foundations::stereo::match(*left_levels, *right_levels, *features, *other,
                                                focal(camera), camera.baseline, stereo_options);
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
  const std::vector<std::uint8_t> none(taken.empty() ? keypoint_count(frame) : 0, 0);
  return foundations::matching::search_by_projection(
      projections, frame.features, frame.grid, frame.right,
      taken.empty() ? std::span<const std::uint8_t>(none) : taken, options);
}

}  // namespace slam_it::edge
