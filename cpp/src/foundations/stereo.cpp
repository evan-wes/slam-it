/// @file
/// @brief Implementation of the stereo rectification declared in slamlab/foundations/stereo.hpp.
#include "slamlab/foundations/stereo.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "slamlab/foundations/matching.hpp"
#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::stereo {

namespace {

/// @brief A camera's smallest focal length.
/// @param model The model.
/// @param intrinsics Its intrinsics, checked.
/// @return `min(fx, fy)`, or `f` for the radial model.
double smallest_focal(camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics) {
  return model == camera::Model::kRadial ? intrinsics(0) : std::min(intrinsics(0), intrinsics(1));
}

/// @brief Check an image shape: each side from 1 to `image::kMaxSide`.
/// @param rows The rows.
/// @param cols The columns.
/// @return Nothing, or `InvalidInput`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (rows, cols), as numpy's shape
Result<void> check_shape(std::int64_t rows, std::int64_t cols) {
  if (rows < 1 || cols < 1 || rows > image::kMaxSide || cols > image::kMaxSide) {
    return fail(ErrorKind::kInvalidInput,
                "shape: expected (rows, cols), each from 1 to " + std::to_string(image::kMaxSide));
  }
  return {};
}

/// @brief A vector's index from a signed count.
/// @param index The index, non-negative.
/// @return It as a `std::size_t`.
std::size_t at(std::int64_t index) { return static_cast<std::size_t>(index); }

/// @brief Check one camera's features: one descriptor per keypoint, levels in range.
/// @param features The features.
/// @param levels The pyramid's levels.
/// @param name The side's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_side(const orb::Features& features, std::size_t levels, const char* name) {
  if (features.descriptors.size() != features.keypoints.size()) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": expected one keypoint and descriptor per row");
  }
  for (const orb::Keypoint& k : features.keypoints) {
    if (k.level < 0 || std::cmp_greater_equal(k.level, levels)) {
      return fail(ErrorKind::kInvalidInput,
                  std::string(name) + ": levels must lie in [0, " + std::to_string(levels) + ")");
    }
  }
  return {};
}

/// @brief Check every left keypoint lies at least `margin` inside its level.
/// @param features The left features, levels checked.
/// @param levels The pyramid.
/// @param margin The SAD window's half-width.
/// @return Nothing, or `InvalidInput`.
Result<void> check_inside(const orb::Features& features, const std::vector<image::Image>& levels,
                          std::int64_t margin) {
  for (const orb::Keypoint& k : features.keypoints) {
    const image::Image& level = levels[at(k.level)];
    const auto [x, y] = k.level_xy;
    if (x < margin || x >= level.cols - margin || y < margin || y >= level.rows - margin) {
      return fail(ErrorKind::kInvalidInput, "left: keypoints must lie at least " +
                                                std::to_string(margin) +
                                                " pixels inside their level");
    }
  }
  return {};
}

/// @brief Slide the left patch along the right row and fit a parabola to the best sum.
/// @param left The left level.
/// @param right The right level.
/// @param pixel The left keypoint's level pixel `(x, y)`.
/// @param column The right keypoint's column on that level.
/// @param options The settings.
/// @return The sub-pixel offset from `column` and the best sum, or nothing if the search
///         leaves the image, the best offset is an end, the valley is flat, or the parabola's
///         vertex is more than a pixel away.
std::optional<std::pair<double, std::int64_t>> refine(const image::Image& left,
                                                      const image::Image& right,
                                                      std::array<std::int64_t, 2> pixel,
                                                      std::int64_t column,
                                                      const MatchOptions& options) {
  const std::int64_t w = options.window;
  const std::int64_t reach = options.search;
  const auto [x, y] = pixel;
  if (column - reach - w < 0 || column + reach + w >= right.cols) {
    return std::nullopt;
  }
  const std::int64_t left_centre = image::pixel(left, y, x);
  std::vector<std::int64_t> sums;
  for (std::int64_t step = -reach; step <= reach; ++step) {
    const std::int64_t c = column + step;
    const std::int64_t right_centre = image::pixel(right, y, c);
    std::int64_t sum = 0;
    for (std::int64_t dy = -w; dy <= w; ++dy) {
      for (std::int64_t dx = -w; dx <= w; ++dx) {
        sum += std::abs((image::pixel(left, y + dy, x + dx) - left_centre) -
                        (image::pixel(right, y + dy, c + dx) - right_centre));
      }
    }
    sums.push_back(sum);
  }
  const auto best = static_cast<std::int64_t>(
      std::distance(sums.begin(), std::ranges::min_element(sums)));  // ties: the leftmost
  if (best == 0 || best == 2 * reach) {
    return std::nullopt;
  }
  const std::int64_t d1 = sums[at(best - 1)];
  const std::int64_t d2 = sums[at(best)];
  const std::int64_t d3 = sums[at(best + 1)];
  const std::int64_t curvature = 2 * (d1 + d3 - (2 * d2));
  if (curvature == 0) {  // a flat valley: no sub-pixel position
    return std::nullopt;
  }
  const double delta = static_cast<double>(d1 - d3) / static_cast<double>(curvature);
  if (delta < -1.0 || delta > 1.0) {
    return std::nullopt;
  }
  return std::pair{static_cast<double>(best - reach) + delta, d2};
}

/// @brief A left keypoint's nearest right candidate: within a level, within the row band,
///        at most `focal` columns to its left, under the distance cap.
/// @param k The left keypoint.
/// @param descriptor Its descriptor.
/// @param right The right features.
/// @param scales Each level's scale.
/// @param focal The rectified focal length.
/// @param options The settings.
/// @return The nearest candidate's index (ties: the lowest), or nothing.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (focal), then the settings
std::optional<std::size_t> nearest(const orb::Keypoint& k, const orb::Descriptor& descriptor,
                                   const orb::Features& right, const std::vector<double>& scales,
                                   double focal, const MatchOptions& options) {
  std::optional<std::size_t> best;
  std::int64_t best_distance = options.max_distance;
  for (std::size_t j = 0; j < right.keypoints.size(); ++j) {
    const orb::Keypoint& r = right.keypoints[j];
    if (std::abs(r.level - k.level) > 1 ||
        std::abs(r.xy[1] - k.xy[1]) > options.band * scales[at(r.level)] ||
        r.xy[0] < k.xy[0] - focal || r.xy[0] > k.xy[0]) {
      continue;
    }
    const std::int64_t d = matching::distance(descriptor, right.descriptors[j]);
    if (d < best_distance) {
      best_distance = d;
      best = j;
    }
  }
  return best;
}

/// @brief Check the pair's images and camera: one shape, a positive focal length and baseline.
/// @param left_image The left image.
/// @param right_image The right image.
/// @param focal The rectified focal length.
/// @param baseline The baseline.
/// @return Nothing, or `InvalidInput`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (focal, baseline), as in Python
Result<void> check_camera(const image::Image& left_image, const image::Image& right_image,
                          double focal, double baseline) {
  if (right_image.rows != left_image.rows || right_image.cols != left_image.cols) {
    return fail(ErrorKind::kInvalidInput, "right_image: expected the left image's shape");
  }
  if (!std::isfinite(focal) || focal <= 0.0) {
    return fail_setting("focal", "must be positive and finite", focal);
  }
  if (!std::isfinite(baseline) || baseline <= 0.0) {
    return fail_setting("baseline", "must be positive and finite", baseline);
  }
  return {};
}

/// @brief A found match and its best sum, before the outlier filter.
struct Candidate {
  /// @brief The match.
  StereoMatch match;
  /// @brief Its best sum of absolute differences.
  std::int64_t cost;
};

/// @brief Drop matches whose best sum exceeds `factor` times the median.
/// @param found The matches, by left index.
/// @param factor The factor.
/// @return The kept matches, by left index.
std::vector<StereoMatch> drop_outliers(const std::vector<Candidate>& found, double factor) {
  if (found.empty()) {
    return {};
  }
  std::vector<std::int64_t> costs;
  costs.reserve(found.size());
  for (const Candidate& c : found) {
    costs.push_back(c.cost);
  }
  std::ranges::sort(costs);
  const double limit = factor * static_cast<double>(costs[costs.size() / 2]);
  std::vector<StereoMatch> out;
  for (const Candidate& c : found) {
    if (static_cast<double>(c.cost) <= limit) {
      out.push_back(c.match);
    }
  }
  return out;
}

}  // namespace

Result<Rectification> rectify(camera::Model model_left,
                              const Eigen::Ref<const Eigen::VectorXd>& intrinsics_left,
                              camera::Model model_right,
                              const Eigen::Ref<const Eigen::VectorXd>& intrinsics_right,
                              const Eigen::Matrix4d& T_right_left, std::int64_t rows,
                              std::int64_t cols) {
  for (const auto& ok : {camera::check_intrinsics(model_left, intrinsics_left),
                         camera::check_intrinsics(model_right, intrinsics_right),
                         se3::check_transform(T_right_left), check_shape(rows, cols)}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  const Eigen::Matrix3d rotation = T_right_left.topLeftCorner<3, 3>();
  const Eigen::Vector3d translation = T_right_left.topRightCorner<3, 1>();
  const double baseline = translation.norm();
  if (baseline <= kMinBaseline) {
    return fail(ErrorKind::kDegenerate, "T_right_left: the camera centres coincide");
  }
  const auto omega = so3::log(rotation);
  if (!omega) {
    return std::unexpected(omega.error());
  }
  // Each camera turns half the way, then both turn the baseline onto the x axis.
  const Eigen::Matrix3d half = so3::exp(-0.5 * *omega);
  const Eigen::Vector3d t = half * translation;
  if (std::abs(t.x()) <= std::abs(t.y())) {
    return fail(ErrorKind::kInvalidInput,
                "T_right_left: the baseline is closer to vertical; only side-by-side pairs "
                "rectify");
  }
  const Eigen::Vector3d target(std::copysign(1.0, t.x()), 0.0, 0.0);
  Eigen::Vector3d axis = t.cross(target);
  const double norm = axis.norm();
  if (norm > 0.0) {
    axis *= std::acos(std::min(std::abs(t.x()) / t.norm(), 1.0)) / norm;
  }
  const Eigen::Matrix3d onto_x = so3::exp(axis);
  const double focal = std::min(smallest_focal(model_left, intrinsics_left),
                                smallest_focal(model_right, intrinsics_right));
  return Rectification{.rotation_left = onto_x * half.transpose(),
                       .rotation_right = onto_x * half,
                       .intrinsics = {focal, focal, static_cast<double>(cols - 1) / 2.0,
                                      static_cast<double>(rows - 1) / 2.0},
                       .baseline = baseline};
}

Result<image::Coordinates> rectification_map(camera::Model model,
                                             const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                             const Eigen::Matrix3d& rotation,
                                             const Eigen::Vector4d& rectified, std::int64_t rows,
                                             std::int64_t cols) {
  for (const auto& ok : {camera::check_intrinsics(model, intrinsics),
                         camera::check_intrinsics(camera::Model::kPinhole, rectified),
                         so3::check_rotation(rotation), check_shape(rows, cols)}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  const double f = rectified(0);
  const double cx = rectified(2);
  const double cy = rectified(3);
  const Eigen::Matrix3d& r = rotation;
  image::Coordinates out(rows * cols, 2);
  for (std::int64_t v = 0; v < rows; ++v) {
    for (std::int64_t u = 0; u < cols; ++u) {
      const double x = (static_cast<double>(u) - cx) / f;
      const double y = (static_cast<double>(v) - cy) / f;
      // R^T (x, y, 1), each product and sum in the Python reference's order.
      const Eigen::Vector3d ray((r(0, 0) * x) + (r(1, 0) * y) + r(2, 0),
                                (r(0, 1) * x) + (r(1, 1) * y) + r(2, 1),
                                (r(0, 2) * x) + (r(1, 2) * y) + r(2, 2));
      const Eigen::Index i = (v * cols) + u;
      if (ray.z() > camera::kMinDepth) {
        const auto pixel = camera::project(model, intrinsics, ray);
        if (!pixel) {
          return std::unexpected(pixel.error());
        }
        out.row(i) = pixel->transpose();
      } else {
        out.row(i) << kOutside, kOutside;
      }
    }
  }
  return out;
}

Result<void> check_options(const MatchOptions& options) {
  if (options.max_distance < 1 || options.max_distance > 257) {
    return fail_setting("max_distance", "must be from 1 to 257",
                        static_cast<double>(options.max_distance));
  }
  if (!std::isfinite(options.band) || options.band <= 0.0) {
    return fail_setting("band", "must be positive and finite", options.band);
  }
  if (options.window < 1 || options.window > orb::kEdgeThreshold) {
    return fail_setting("window", "must be from 1 to 19", static_cast<double>(options.window));
  }
  if (options.search < 2 || options.search > 64) {
    return fail_setting("search", "must be from 2 to 64", static_cast<double>(options.search));
  }
  if (!std::isfinite(options.outlier_factor) || options.outlier_factor < 1.0) {
    return fail_setting("outlier_factor", "must be at least 1", options.outlier_factor);
  }
  return {};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (focal, baseline), as in Python
Result<std::vector<StereoMatch>> match(const image::Image& left_image,
                                       const image::Image& right_image, const orb::Features& left,
                                       const orb::Features& right, double focal, double baseline,
                                       const orb::Options& orb_options,
                                       const MatchOptions& options) {
  for (const auto& ok : {check_camera(left_image, right_image, focal, baseline),
                         check_options(options), orb::check_options(orb_options)}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  const auto left_levels = image::pyramid(left_image, orb_options.levels, orb_options.scale_factor);
  const auto right_levels =
      image::pyramid(right_image, orb_options.levels, orb_options.scale_factor);
  if (!left_levels || !right_levels) {
    return std::unexpected(left_levels ? right_levels.error() : left_levels.error());
  }
  for (const auto& ok : {check_side(left, left_levels->size(), "left"),
                         check_side(right, left_levels->size(), "right")}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  if (auto ok = check_inside(left, *left_levels, options.window); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<double> scales{1.0};
  while (scales.size() < left_levels->size()) {
    scales.push_back(scales.back() * orb_options.scale_factor);
  }
  std::vector<Candidate> found;
  for (std::size_t i = 0; i < left.keypoints.size(); ++i) {
    const orb::Keypoint& k = left.keypoints[i];
    const auto j = nearest(k, left.descriptors[i], right, scales, focal, options);
    if (!j) {
      continue;
    }
    const double scale = scales[at(k.level)];
    const auto column =
        static_cast<std::int64_t>(std::floor((right.keypoints[*j].xy[0] / scale) + 0.5));
    const auto refined = refine((*left_levels)[at(k.level)], (*right_levels)[at(k.level)],
                                k.level_xy, column, options);
    if (!refined) {
      continue;
    }
    const double right_x = scale * (static_cast<double>(column) + refined->first);
    const double disparity = k.xy[0] - right_x;
    if (disparity <= 0.0 || disparity >= focal) {
      continue;
    }
    found.push_back({.match = {.left = static_cast<std::int64_t>(i),
                               .right = static_cast<std::int64_t>(*j),
                               .right_x = right_x,
                               .disparity = disparity,
                               .depth = (focal * baseline) / disparity},
                     .cost = refined->second});
  }
  return drop_outliers(found, options.outlier_factor);
}

}  // namespace slamlab::foundations::stereo
