/// @file
/// @brief Implementation of the initialisation declared in slamlab/edge/initialization.hpp.
#include "slamlab/edge/initialization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "slamlab/edge/adjust.hpp"
#include "slamlab/foundations/twoview.hpp"

namespace slamlab::edge {

namespace {

namespace matching = foundations::matching;
namespace twoview = foundations::twoview;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

/// @brief Check the settings and a frame's pyramid against them.
/// @param frame The frame.
/// @param options The settings.
/// @param name The frame's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_frame(const Frame& frame, const Options& options, const char* name) {
  if (auto ok = check_options(options); !ok) {
    return ok;
  }
  if (frame.features.levels != options.levels ||
      frame.features.scale_factor != options.scale_factor) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": its pyramid must be the settings'");
  }
  return {};
}

/// @brief Add a keyframe that observes no point yet.
/// @param m The map.
/// @param frame The frame.
/// @param pose Its `T_cam_world`.
/// @return Nothing, or the map's error.
Result<void> insert_unobserving(Map& m, const Frame& frame, const Eigen::Matrix4d& pose) {
  const std::vector<std::int64_t> none(keypoint_count(frame), -1);
  return insert_keyframe(m, frame, pose, none);
}

/// @brief Scale the map's translations and points, and refresh the points (a monocular map's
///        gauge, `slamlab.edge.initialization._scaled`).
/// @param m The map.
/// @param factor The scale.
/// @return Nothing, or the map's error.
Result<void> scale(Map& m, double factor) {
  for (Eigen::Matrix4d& pose : m.kf_pose) {
    pose.topRightCorner<3, 1>() *= factor;
  }
  for (Eigen::Vector3d& x : m.pt_xyz) {
    x *= factor;
  }
  std::vector<std::int64_t> rows(point_count(m));
  for (std::size_t p = 0; p < rows.size(); ++p) {
    rows[p] = static_cast<std::int64_t>(p);
  }
  return refresh_points(m, rows);
}

/// @brief The median of some values: the mean of the middle two for an even count.
/// @param values The values (reordered).
/// @return The median.
double median(std::vector<double>& values) {
  const std::size_t half = values.size() / 2;
  std::ranges::nth_element(values, values.begin() + static_cast<std::ptrdiff_t>(half));
  const double upper = values[half];
  if (values.size() % 2 == 1) {
    return upper;
  }
  const double lower =
      *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(half));
  return (lower + upper) / 2.0;
}

}  // namespace

Result<std::optional<Started>> initialize_from_depth(const Frame& frame, const Options& options) {
  if (auto ok = check_frame(frame, options, "frame"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (std::cmp_less(keypoint_count(frame), options.min_init_features_depth)) {
    return std::nullopt;
  }
  std::vector<std::int64_t> has_depth;
  for (std::size_t i = 0; i < keypoint_count(frame); ++i) {
    if (!std::isnan(frame.depth[i]) && !std::isnan(frame.bearings[i].x())) {
      has_depth.push_back(static_cast<std::int64_t>(i));
    }
  }
  Started out{.map = empty_map(options.levels, options.scale_factor), .tracked = {}};
  if (auto ok = insert_unobserving(out.map, frame, Eigen::Matrix4d::Identity()); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<Eigen::Vector3d> xyz;
  std::vector<std::vector<std::int64_t>> observers;
  for (const std::int64_t i : has_depth) {
    const Eigen::Vector3d& b = frame.bearings[static_cast<std::size_t>(i)];
    xyz.emplace_back(b * (frame.depth[static_cast<std::size_t>(i)] / b.z()));
    observers.push_back({i});
  }
  if (auto ok = add_points(out.map, xyz, observers); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  out.tracked.points.assign(keypoint_count(frame), -1);
  for (std::size_t j = 0; j < has_depth.size(); ++j) {
    out.tracked.points[static_cast<std::size_t>(has_depth[j])] = out.map.pt_id[j];
  }
  return out;
}

Result<std::vector<matching::Match>> match_for_initialization(const Frame& reference,
                                                              const Frame& frame,
                                                              const Options& options) {
  if (auto ok = check_frame(reference, options, "reference"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_frame(frame, options, "frame"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<std::int64_t> rows;
  std::vector<matching::Projection> projections;
  for (std::size_t i = 0; i < keypoint_count(reference); ++i) {
    const foundations::orb::Keypoint& k = reference.features.keypoints[i];
    if (k.level == 0) {
      rows.push_back(static_cast<std::int64_t>(i));
      projections.push_back({.xy = k.xy,
                             .radius = options.init_window,
                             .min_level = 0,
                             .max_level = 0,
                             .descriptor = reference.features.descriptors[i],
                             .angle = k.angle,
                             .right = std::numeric_limits<double>::quiet_NaN()});
    }
  }
  auto found =
      search(frame, projections,
             {.max_distance = kThLow, .ratio = options.init_ratio, .check_orientation = true});
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  for (matching::Match& match : *found) {
    match.first = rows[static_cast<std::size_t>(match.first)];
  }
  return found;
}

Result<std::optional<Started>> initialize_monocular(const Camera& camera, const Frame& reference,
                                                    const Frame& frame, const Options& options) {
  if (auto ok = check_camera(camera); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  auto found = match_for_initialization(reference, frame, options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  std::vector<matching::Match> pairs;
  for (const matching::Match& match : *found) {
    if (!std::isnan(reference.bearings[static_cast<std::size_t>(match.first)].x()) &&
        !std::isnan(frame.bearings[static_cast<std::size_t>(match.second)].x())) {
      pairs.push_back(match);
    }
  }
  if (std::cmp_less(pairs.size(), options.init_min_matches)) {
    return std::nullopt;
  }
  const auto n = static_cast<Eigen::Index>(pairs.size());
  foundations::correspondences::Rows3 bearings1(n, 3);
  foundations::correspondences::Rows3 bearings2(n, 3);
  for (Eigen::Index i = 0; i < n; ++i) {
    const matching::Match& match = pairs[static_cast<std::size_t>(i)];
    bearings1.row(i) = reference.bearings[static_cast<std::size_t>(match.first)].transpose();
    bearings2.row(i) = frame.bearings[static_cast<std::size_t>(match.second)].transpose();
  }
  auto result = twoview::initialize(
      bearings1, bearings2,
      {.focal = focal(camera), .seed = static_cast<std::uint64_t>(options.seed)});
  if (!result) {
    if (result.error().kind == ErrorKind::kDegenerate) {
      return std::nullopt;
    }
    return std::unexpected(std::move(result.error()));
  }
  if (result->status != twoview::Status::kInitialized) {
    return std::nullopt;
  }
  Started out{.map = empty_map(options.levels, options.scale_factor), .tracked = {}};
  Map& m = out.map;
  if (auto ok = insert_unobserving(m, reference, Eigen::Matrix4d::Identity()); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = insert_unobserving(m, frame, result->relative_pose); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::int64_t second = m.kf_start[1];
  std::vector<Eigen::Vector3d> xyz;
  std::vector<std::vector<std::int64_t>> observers;
  for (Eigen::Index i = 0; i < n; ++i) {
    if (result->triangulated(i)) {
      const matching::Match& match = pairs[static_cast<std::size_t>(i)];
      xyz.emplace_back(result->points.row(i).transpose());
      observers.push_back({match.first, second + match.second});
    }
  }
  if (auto ok = add_points(m, xyz, observers); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  Options adjusted = options;
  adjusted.ba_iterations = kInitBaIterations;
  const std::vector<std::int64_t> both{0, 1};
  const std::vector<std::uint8_t> fixed{1, 0};
  if (auto ok = bundle_adjust(m, camera, both, fixed, adjusted); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (std::cmp_less(point_count(m), options.init_min_matches)) {
    return std::nullopt;  // ORB-SLAM resets with fewer than 100 tracked points
  }
  const Eigen::Matrix4d& first = m.kf_pose[0];
  std::vector<double> depths;
  depths.reserve(point_count(m));
  for (const Eigen::Vector3d& x : m.pt_xyz) {
    depths.push_back(first.block<1, 3>(2, 0).dot(x.transpose()) + first(2, 3));
  }
  const double middle = median(depths);
  if (!(middle > 0.0)) {
    return std::nullopt;
  }
  if (auto ok = scale(m, 1.0 / middle); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  out.tracked.pose = m.kf_pose[1];
  out.tracked.points.assign(m.kp_point.begin() + m.kf_start[1], m.kp_point.begin() + m.kf_start[2]);
  return out;
}

}  // namespace slamlab::edge
