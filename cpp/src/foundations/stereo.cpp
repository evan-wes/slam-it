/// @file
/// @brief Implementation of the stereo rectification declared in
/// slam_it/foundations/stereo.hpp.
#include "slam_it/foundations/stereo.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/so3.hpp"

namespace slam_it::foundations::stereo {

namespace {

/// @brief A camera's smallest focal length.
/// @param model The model.
/// @param intrinsics Its intrinsics, checked.
/// @return The smallest of its focal-length entries (`camera::focal_indices`).
double smallest_focal(camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics) {
  double out = std::numeric_limits<double>::infinity();
  for (const Eigen::Index i : camera::focal_indices(model)) {
    out = std::min(out, intrinsics(i));
  }
  return out;
}

/// @brief Check an image shape: each side from 1 to `image::kMaxSide`.
/// @param rows The rows.
/// @param cols The columns.
/// @return Nothing, or `InvalidInput` naming the side out of range.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (rows, cols), as numpy's shape
Result<void> check_shape(std::int64_t rows, std::int64_t cols) {
  if (auto ok = check_range("rows", rows, 1, image::kMaxSide); !ok) {
    return ok;
  }
  return check_range("cols", cols, 1, image::kMaxSide);
}

/// @brief A vector's index from a signed count.
/// @param index The index, non-negative.
/// @return It as a `std::size_t`.
std::size_t at(std::int64_t index) { return static_cast<std::size_t>(index); }

/// @brief Check the two sides' pyramids: a valid one, the same for both.
/// @param left The left features.
/// @param right The right features.
/// @return Nothing, or `InvalidInput`.
Result<void> check_pyramids(const orb::Features& left, const orb::Features& right) {
  if (auto ok = check_range("left.levels", left.levels, 1, orb::kMaxLevels); !ok) {
    return ok;
  }
  if (auto ok = image::check_scale_factor(left.scale_factor, "left.scale_factor"); !ok) {
    return ok;
  }
  if (right.levels != left.levels || right.scale_factor != left.scale_factor) {
    return fail(ErrorKind::kInvalidInput,
                "right: extracted on another pyramid than left (levels and scale_factor differ)");
  }
  return {};
}

/// @brief Check one camera's features against their pyramid: one descriptor per keypoint,
///        every keypoint inside its level, and its `xy` exactly its level pixel's full-image
///        coordinate.
/// @param features The features.
/// @param levels The pyramid.
/// @param name The side's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_side(const orb::Features& features, std::span<const image::Image> levels,
                        const std::string& name) {
  if (features.descriptors.size() != features.keypoints.size()) {
    return fail(ErrorKind::kInvalidInput, name + ": expected one keypoint and descriptor per row");
  }
  const image::Image& full = levels.front();
  for (const orb::Keypoint& k : features.keypoints) {
    if (k.level < 0 || std::cmp_greater_equal(k.level, levels.size())) {
      return fail(ErrorKind::kInvalidInput,
                  name + ": levels must lie in [0, " + std::to_string(levels.size()) + ")");
    }
    const image::Image& level = levels[at(k.level)];
    const auto [x, y] = k.level_xy;
    if (x < 0 || y < 0 || x >= level.cols || y >= level.rows) {
      return fail(ErrorKind::kInvalidInput, name + ": level_xy must lie inside its level");
    }
    if (k.xy[0] != image::full_image_coordinate(static_cast<double>(x), level.cols, full.cols) ||
        k.xy[1] != image::full_image_coordinate(static_cast<double>(y), level.rows, full.rows)) {
      return fail(ErrorKind::kInvalidInput,
                  name + ": xy must be level_xy mapped to the full image (orb.Keypoints.xy)");
    }
  }
  return {};
}

/// @brief Check every left keypoint lies at least `margin` inside its level.
/// @param features The left features, checked against the pyramid.
/// @param levels The pyramid.
/// @param margin The SAD window's half-width.
/// @return Nothing, or `InvalidInput`.
Result<void> check_inside(const orb::Features& features, std::span<const image::Image> levels,
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

/// @brief The column on one level nearest a pixel's centre on another: the pixel mapped to the
///        full image (`image::full_image_coordinate`) and on to the other level
///        (`image::level_coordinate`), at `((2 x + 1) to_cols - from_cols) / (2 from_cols)`,
///        rounded half up, in integers.
/// @param x The pixel's column, inside its level.
/// @param from_cols Its level's columns.
/// @param to_cols The other level's columns.
/// @return `floor((2 x + 1) to_cols / (2 from_cols))`, inside the other level.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (x, from, to), as in Python
std::int64_t nearest_column(std::int64_t x, std::int64_t from_cols, std::int64_t to_cols) {
  return (((2 * x) + 1) * to_cols) / (2 * from_cols);
}

/// @brief The zero-mean sum of absolute differences between two patches, times their size.
/// @param left The left level.
/// @param right The right level.
/// @param pixel The left patch's centre `(x, y)`.
/// @param column The right patch's centre column, on row `y`.
/// @param options The settings: the patches' half-width `w` is `options.window`.
/// @return `sum |n (L - R) - (sum L - sum R)|`, `n = (2 w + 1)^2`.
std::int64_t zero_mean_sad(const image::Image& left, const image::Image& right,
                           std::array<std::int64_t, 2> pixel, std::int64_t column,
                           const MatchOptions& options) {
  const std::int64_t w = options.window;
  const auto [x, y] = pixel;
  const std::int64_t n = ((2 * w) + 1) * ((2 * w) + 1);
  std::int64_t offset = 0;  // sum L - sum R
  for (std::int64_t dy = -w; dy <= w; ++dy) {
    for (std::int64_t dx = -w; dx <= w; ++dx) {
      offset += image::pixel(left, y + dy, x + dx) - image::pixel(right, y + dy, column + dx);
    }
  }
  std::int64_t sum = 0;
  for (std::int64_t dy = -w; dy <= w; ++dy) {
    for (std::int64_t dx = -w; dx <= w; ++dx) {
      const std::int64_t difference =
          image::pixel(left, y + dy, x + dx) - image::pixel(right, y + dy, column + dx);
      sum += std::abs((n * difference) - offset);
    }
  }
  return sum;
}

/// @brief Slide the left patch along the right row by zero-mean SAD and V-fit the best sum.
/// @param left The left level.
/// @param right The right level.
/// @param pixel The left keypoint's level pixel `(x, y)`.
/// @param column The right keypoint's column on that level, inside it.
/// @param options The settings.
/// @return The sub-pixel offset from `column` and the best sum, or nothing if the search
///         leaves the image or the best offset is an end.
std::optional<std::pair<double, std::int64_t>> refine(const image::Image& left,
                                                      const image::Image& right,
                                                      std::array<std::int64_t, 2> pixel,
                                                      std::int64_t column,
                                                      const MatchOptions& options) {
  const std::int64_t w = options.window;
  const std::int64_t reach = options.search;
  // Every value is below 2^21 (a column inside its level, reach and w small): no overflow.
  if (column - reach - w < 0 || column + reach + w >= right.cols) {
    return std::nullopt;
  }
  std::vector<std::int64_t> sums;
  for (std::int64_t step = -reach; step <= reach; ++step) {
    sums.push_back(zero_mean_sad(left, right, pixel, column + step, options));
  }
  // The first smallest sum, so d1 > d2 <= d3 below: the V fit's denominator is positive and
  // its offset within half a pixel.
  const auto best =
      static_cast<std::int64_t>(std::distance(sums.begin(), std::ranges::min_element(sums)));
  if (best == 0 || best == 2 * reach) {
    return std::nullopt;
  }
  const std::int64_t d1 = sums[at(best - 1)];
  const std::int64_t d2 = sums[at(best)];
  const std::int64_t d3 = sums[at(best + 1)];
  const double delta =
      static_cast<double>(d1 - d3) / static_cast<double>(2 * std::max(d1 - d2, d3 - d2));
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
    const std::int64_t d = orb::distance(descriptor, right.descriptors[j]);
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
    return fail(ErrorKind::kInvalidInput, "right_levels: expected the left pyramid's shapes");
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
  if (t.x() > 0.0) {
    return fail(ErrorKind::kInvalidInput,
                "T_right_left: the right camera lies left of the left one; swap the cameras");
  }
  const Eigen::Vector3d target(-1.0, 0.0, 0.0);  // the left camera's centre, seen from the right
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
  if (auto ok = check_range("max_distance", options.max_distance, 1, matching::kMaxDistance + 1);
      !ok) {
    return ok;
  }
  if (!std::isfinite(options.band) || options.band <= 0.0) {
    return fail_setting("band", "must be positive and finite", options.band);
  }
  if (auto ok = check_range("window", options.window, 1, orb::kEdgeThreshold); !ok) {
    return ok;
  }
  if (auto ok = check_range("search", options.search, 1, kMaxSearch); !ok) {
    return ok;
  }
  if (!std::isfinite(options.outlier_factor) || options.outlier_factor < 1.0) {
    return fail_setting("outlier_factor", "must be at least 1", options.outlier_factor);
  }
  return {};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (focal, baseline), as in Python
Result<std::vector<StereoMatch>> match(std::span<const image::Image> left_levels,
                                       std::span<const image::Image> right_levels,
                                       const orb::Features& left, const orb::Features& right,
                                       double focal, double baseline, const MatchOptions& options) {
  if (auto ok = check_pyramids(left, right); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  for (const auto& ok :
       {image::check_pyramid(left_levels, left.levels, left.scale_factor, "left_levels"),
        image::check_pyramid(right_levels, left.levels, left.scale_factor, "right_levels")}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  for (const auto& ok : {check_camera(left_levels.front(), right_levels.front(), focal, baseline),
                         check_options(options)}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  // One after another: check_inside indexes the pyramid by levels check_side has checked.
  if (auto ok = check_side(left, left_levels, "left"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_side(right, left_levels, "right"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_inside(left, left_levels, options.window); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<double> scales{1.0};
  while (scales.size() < left_levels.size()) {
    scales.push_back(scales.back() * left.scale_factor);
  }
  const std::int64_t cols = left_levels.front().cols;
  std::vector<Candidate> found;
  for (std::size_t i = 0; i < left.keypoints.size(); ++i) {
    const orb::Keypoint& k = left.keypoints[i];
    const auto j = nearest(k, left.descriptors[i], right, scales, focal, options);
    if (!j) {
      continue;
    }
    const orb::Keypoint& r = right.keypoints[*j];
    const std::int64_t level_cols = left_levels[at(k.level)].cols;
    const std::int64_t column =
        nearest_column(r.level_xy[0], left_levels[at(r.level)].cols, level_cols);
    const auto refined =
        refine(left_levels[at(k.level)], right_levels[at(k.level)], k.level_xy, column, options);
    if (!refined) {
      continue;
    }
    const double right_x = image::full_image_coordinate(
        static_cast<double>(column) + refined->first, level_cols, cols);
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

}  // namespace slam_it::foundations::stereo
