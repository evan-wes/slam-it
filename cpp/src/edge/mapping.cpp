/// @file
/// @brief Implementation of the local mapping declared in slam_it/edge/mapping.hpp.
#include "slam_it/edge/mapping.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

#include "slam_it/edge/adjust.hpp"
#include "slam_it/edge/covisibility.hpp"
#include "slam_it/edge/residuals.hpp"
#include "slam_it/foundations/correspondences.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/triangulation.hpp"

namespace slam_it::edge {

namespace {

namespace matching = foundations::matching;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
using foundations::correspondences::Rows3;

/// @brief NaN, for a point not triangulated.
constexpr double kNanPoint = std::numeric_limits<double>::quiet_NaN();

/// @brief Move a camera-frame point into the world: `R^T (X - t)`.
/// @param pose The camera's `T_cam_world`.
/// @param x The point in the camera's frame.
/// @return The point in the world.
Eigen::Vector3d to_world(const Eigen::Matrix4d& pose, const Eigen::Vector3d& x) {
  return pose.topLeftCorner<3, 3>().transpose() * (x - pose.topRightCorner<3, 1>());
}

/// @brief A keyframe's links' rows, most shared first (`slam_it.edge.covisibility.links`).
/// @param m The map.
/// @param k The keyframe's row.
/// @param options The settings.
/// @return The rows.
std::vector<std::int64_t> linked_rows(const Map& m, std::size_t k, const Options& options) {
  std::vector<std::int64_t> out;
  for (const Link& link : links_of(m, shared_points(m, k), options.covisibility.min_points)) {
    out.push_back(link.row);
  }
  return out;
}

/// @brief The newest keyframe's fusion targets' ids (`fuse_targets`, unchecked).
/// @param m The map, with at least one keyframe.
/// @param mode The pipeline's mode.
/// @param options The settings.
/// @return The ids.
std::vector<std::int64_t> targets_of(const Map& m, Mode mode, const Options& options) {
  const auto newest = static_cast<std::int64_t>(keyframe_count(m)) - 1;
  const MappingOptions& settings = options.mapping;
  const std::int64_t n =
      mode == Mode::kMonocular ? settings.mono_fuse_neighbours : settings.fuse_neighbours;
  std::vector<std::int64_t> targets = linked_rows(m, static_cast<std::size_t>(newest), options);
  if (std::cmp_greater(targets.size(), n)) {
    targets.resize(static_cast<std::size_t>(n));
  }
  const std::size_t first = targets.size();
  for (std::size_t i = 0; i < first; ++i) {
    std::vector<std::int64_t> second =
        linked_rows(m, static_cast<std::size_t>(targets[i]), options);
    if (std::cmp_greater(second.size(), settings.fuse_second_neighbours)) {
      second.resize(static_cast<std::size_t>(settings.fuse_second_neighbours));
    }
    for (const std::int64_t row : second) {
      if (row != newest && !std::ranges::contains(targets, row)) {
        targets.push_back(row);
      }
    }
  }
  std::vector<std::int64_t> ids;
  ids.reserve(targets.size());
  for (const std::int64_t row : targets) {
    ids.push_back(m.kf_id[static_cast<std::size_t>(row)]);
  }
  return ids;
}

/// @brief One flag per keyframe: the map's first held fixed, local BA's gauge.
/// @param m The map.
/// @return The flags.
std::vector<std::uint8_t> first_fixed(const Map& m) {
  std::vector<std::uint8_t> fixed(keyframe_count(m), 0);
  for (std::size_t k = 0; k < fixed.size(); ++k) {
    fixed[k] = m.kf_id[k] == 0 ? 1 : 0;
  }
  return fixed;
}

/// @brief The newest keyframe's row and its links', ascending (`local_ba_keyframes`, unchecked).
/// @param m The map, with at least one keyframe.
/// @param options The settings.
/// @return The rows.
std::vector<std::int64_t> local_rows(const Map& m, const Options& options) {
  std::vector<std::int64_t> local = linked_rows(m, keyframe_count(m) - 1, options);
  local.push_back(static_cast<std::int64_t>(keyframe_count(m)) - 1);
  std::ranges::sort(local);
  return local;
}

/// @brief Check that the map has a keyframe.
/// @param m The map.
/// @return Nothing, or `InvalidInput`.
Result<void> has_keyframes(const Map& m) {
  if (keyframe_count(m) == 0) {
    return fail(ErrorKind::kInvalidInput, "m: expected a map with at least one keyframe");
  }
  return {};
}

/// @brief Check the settings, and that the map has a keyframe (no camera needed).
/// @param m The map.
/// @param options The settings.
/// @return Nothing, or `InvalidInput`.
Result<void> check_keyframes(const Map& m, const Options& options) {
  if (auto ok = check_options(options); !ok) {
    return ok;
  }
  return has_keyframes(m);
}

/// @brief The points from an id on that the newest keyframe observes, by keypoint.
/// @param m The map, with at least one keyframe.
/// @param first The first id.
/// @return Each keypoint's point, if its id is `first` or later; -1 elsewhere.
std::vector<std::int64_t> made_since(const Map& m, std::int64_t first) {
  std::vector<std::int64_t> out;
  for (const std::int64_t row : keypoint_rows(m, keyframe_count(m) - 1)) {
    const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)];
    out.push_back(id >= first ? id : -1);
  }
  return out;
}

/// @brief The world point a keypoint's depth gives (`UnprojectStereo`).
/// @param m The map.
/// @param pose Its keyframe's pose.
/// @param row The keypoint's row.
/// @return The point.
Eigen::Vector3d from_depth(const Map& m, const Eigen::Matrix4d& pose, std::int64_t row) {
  const auto r = static_cast<std::size_t>(row);
  const Eigen::Vector3d& b = m.kp_bearing[r];
  return to_world(pose, b * (m.kp_depth[r] / b.z()));
}

/// @brief Check the camera and the settings, and that the map has a keyframe.
/// @param m The map.
/// @param camera The camera.
/// @param options The settings.
/// @return Nothing, or `InvalidInput`.
Result<void> check_newest(const Map& m, const Camera& camera, const Options& options) {
  if (auto ok = check_setup(camera, options); !ok) {
    return ok;
  }
  return has_keyframes(m);
}

/// @brief The number of a frame's keypoints that are close and tracked, and close and not.
/// @param depth The keypoints' depths.
/// @param tracked Their matches.
/// @param limit The close depth.
/// @return Tracked close, untracked close.
std::pair<std::int64_t, std::int64_t> close_counts(std::span<const double> depth,
                                                   const Tracked& tracked, double limit) {
  std::int64_t tracked_close = 0;
  std::int64_t untracked_close = 0;
  for (std::size_t i = 0; i < depth.size(); ++i) {
    if (depth[i] > 0.0 && depth[i] < limit) {
      ++(tracked.points[i] >= 0 ? tracked_close : untracked_close);
    }
  }
  return {tracked_close, untracked_close};
}

/// @brief A keyframe's keypoints observing no point, with a bearing in front.
/// @param m The map.
/// @param k The keyframe's row.
/// @return Their rows.
std::vector<std::int64_t> free_rows(const Map& m, std::size_t k) {
  std::vector<std::int64_t> out;
  for (const std::int64_t row : keypoint_rows(m, k)) {
    const auto r = static_cast<std::size_t>(row);
    if (m.kp_point[r] < 0 && m.kp_bearing[r].z() > 0.0) {
      out.push_back(row);
    }
  }
  return out;
}

/// @brief The relative pose `T_2_1` of two keyframes: `R2 R1^T` and `t2 - R t1`.
/// @param m The map.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @return The rotation and translation.
std::pair<Eigen::Matrix3d, Eigen::Vector3d> relative(const Map& m, std::size_t k1, std::size_t k2) {
  const Eigen::Matrix4d& pose1 = m.kf_pose[k1];
  const Eigen::Matrix4d& pose2 = m.kf_pose[k2];
  const Eigen::Matrix3d rotation =
      pose2.topLeftCorner<3, 3>() * pose1.topLeftCorner<3, 3>().transpose();
  return {rotation, pose2.topRightCorner<3, 1>() - (rotation * pose1.topRightCorner<3, 1>())};
}

/// @brief A bearing on the normalised plane `z = 1`.
/// @param b The bearing.
/// @return `b / b_z` (its third coordinate exactly 1).
Eigen::Vector3d normalised(const Eigen::Vector3d& b) { return {b.x() / b.z(), b.y() / b.z(), 1.0}; }

/// @brief The epipolar test of one candidate pair for triangulation.
struct Epipolar {
  /// @brief The essential matrix `[t]x R` of `T_2_1`.
  Eigen::Matrix3d essential;
  /// @brief The epipole on keyframe 2's normalised plane, if camera 1's centre is not on its
  ///        focal plane.
  std::optional<Eigen::Vector2d> epipole;
  /// @brief The focal length, in pixels.
  double focal{1.0};
  /// @brief The band's chi-square bound (`mapping.epipolar_chi2`).
  double chi2{0.0};
  /// @brief The least squared distance from the epipole at level 0
  ///        (`mapping.min_epipole_distance2`).
  double epipole_distance2{0.0};
};

/// @brief Whether a candidate pair passes the epipolar band and the epipole test.
/// @param m The map.
/// @param epipolar The two keyframes' geometry.
/// @param line Keypoint 1's epipolar line in keyframe 2.
/// @param row1 Keypoint 1's row.
/// @param row2 Keypoint 2's row.
/// @return Whether it passes.
bool epipolar_ok(const Map& m, const Epipolar& epipolar, const Eigen::Vector3d& line,
                 std::int64_t row1, std::int64_t row2) {
  const auto r2 = static_cast<std::size_t>(row2);
  const Eigen::Vector3d x2 = normalised(m.kp_bearing[r2]);
  const double numerator = line.dot(x2);
  const double distance2 = (numerator * numerator) /
                           ((line.x() * line.x()) + (line.y() * line.y())) *
                           (epipolar.focal * epipolar.focal);
  if (!(distance2 < epipolar.chi2 * sigma2(m, m.kp_level[r2]))) {
    return false;
  }
  if (epipolar.epipole && std::isnan(m.kp_depth[static_cast<std::size_t>(row1)]) &&
      std::isnan(m.kp_depth[r2])) {
    const Eigen::Vector2d gap = x2.head<2>() - *epipolar.epipole;
    const double near =
        ((gap.x() * gap.x()) + (gap.y() * gap.y())) * (epipolar.focal * epipolar.focal);
    if (near < epipolar.epipole_distance2 *
                   std::pow(m.scale_factor, static_cast<double>(m.kp_level[r2]))) {
      return false;
    }
  }
  return true;
}

/// @brief Match free keypoints for triangulation, unchecked.
/// @param m The map.
/// @param camera The camera.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param options The epipolar band and the epipole's distance.
/// @return The pairs.
std::vector<std::array<std::int64_t, 2>> triangulation_pairs(const Map& m, const Camera& camera,
                                                             std::size_t k1, std::size_t k2,
                                                             const Options& options) {
  const std::vector<std::int64_t> rows1 = free_rows(m, k1);
  const std::vector<std::int64_t> rows2 = free_rows(m, k2);
  const auto [rotation, t] = relative(m, k1, k2);
  Eigen::Matrix3d skew;
  skew << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  Epipolar epipolar{.essential = skew * rotation,
                    .epipole = std::nullopt,
                    .focal = focal(camera),
                    .chi2 = options.mapping.epipolar_chi2,
                    .epipole_distance2 = options.mapping.min_epipole_distance2};
  if (t.z() != 0.0) {
    epipolar.epipole = Eigen::Vector2d(t.x() / t.z(), t.y() / t.z());
  }
  // Keyframe 2's keypoints under each vocabulary node, in row order: a pair shares its node
  // (ORB-SLAM3's SearchForTriangulation walks the two FeatureVectors node by node).
  std::unordered_map<std::int64_t, std::vector<std::size_t>> by_node;
  for (std::size_t j = 0; j < rows2.size(); ++j) {
    by_node[m.kp_node[static_cast<std::size_t>(rows2[j])]].push_back(j);
  }
  // For each keyframe-1 keypoint, the nearest candidate (ties: the last, as ORB-SLAM3's
  // `dist > bestDist` skips only a farther one); then one match per keyframe-2 keypoint, the
  // nearest keeping it (ties: the lower keyframe-1 keypoint).
  std::map<std::size_t, std::pair<std::int64_t, std::size_t>> claims;
  for (std::size_t i = 0; i < rows1.size(); ++i) {
    const auto r1 = static_cast<std::size_t>(rows1[i]);
    const auto node = by_node.find(m.kp_node[r1]);
    if (node == by_node.end()) {
      continue;
    }
    const Eigen::Vector3d line = epipolar.essential * normalised(m.kp_bearing[r1]);
    std::int64_t best = std::numeric_limits<std::int64_t>::max();
    std::size_t best_j = 0;
    for (const std::size_t j : node->second) {
      const std::int64_t d = foundations::orb::distance(
          m.kp_descriptor[r1], m.kp_descriptor[static_cast<std::size_t>(rows2[j])]);
      if (d <= options.matching.th_low && d <= best &&
          epipolar_ok(m, epipolar, line, rows1[i], rows2[j])) {
        best = d;
        best_j = j;
      }
    }
    if (best <= options.matching.th_low) {
      const auto it = claims.find(best_j);
      if (it == claims.end() || best < it->second.first) {
        claims[best_j] = {best, i};
      }
    }
  }
  std::vector<std::array<std::int64_t, 2>> out;
  out.reserve(claims.size());
  for (const auto& [j, claim] : claims) {
    out.push_back({rows1[claim.second], rows2[j]});
  }
  std::ranges::sort(out);
  return out;
}

/// @brief The parallax cosine a keypoint's depth gives: `cos(2 atan2(b / 2, z))`.
/// @param camera The camera.
/// @param depth The depth.
/// @return The cosine.
double stereo_cos(const Camera& camera, double depth) {
  return std::cos(2.0 * std::atan2(camera.baseline / 2.0, depth));
}

/// @brief How a pair is triangulated (`CreateNewMapPoints`' choice).
enum class Source : std::uint8_t {
  /// @brief Not at all: no depth and too little parallax.
  kNone,
  /// @brief From the two rays.
  kRays,
  /// @brief From keypoint 1's depth.
  kDepth1,
  /// @brief From keypoint 2's depth.
  kDepth2,
};

/// @brief Choose how to triangulate a pair: from the rays if their parallax beats what depth
///        gives (ORB-SLAM takes the depth parallax of the first keypoint with depth only).
/// @param m The map.
/// @param camera The camera.
/// @param ks The two keyframes' rows.
/// @param pair The keypoint rows.
/// @param options The parallax limit.
/// @return The source.
Source choose_source(const Map& m, const Camera& camera, std::array<std::size_t, 2> ks,
                     const std::array<std::int64_t, 2>& pair, const Options& options) {
  const auto r1 = static_cast<std::size_t>(pair[0]);
  const auto r2 = static_cast<std::size_t>(pair[1]);
  const Eigen::Vector3d ray1 =
      m.kf_pose[ks[0]].topLeftCorner<3, 3>().transpose() * m.kp_bearing[r1];
  const Eigen::Vector3d ray2 =
      m.kf_pose[ks[1]].topLeftCorner<3, 3>().transpose() * m.kp_bearing[r2];
  const double cos_rays = ray1.dot(ray2);
  const bool has1 = !std::isnan(m.kp_depth[r1]);
  const bool has2 = !std::isnan(m.kp_depth[r2]);
  const double cos1 = has1 ? stereo_cos(camera, m.kp_depth[r1]) : cos_rays + 1.0;
  const double cos2 = !has1 && has2 ? stereo_cos(camera, m.kp_depth[r2]) : cos_rays + 1.0;
  if (cos_rays < std::min(cos1, cos2) && cos_rays > 0.0 &&
      (has1 || has2 || cos_rays < options.mapping.max_parallax_cos)) {
    return Source::kRays;
  }
  if (has1 && cos1 < cos2) {
    return Source::kDepth1;
  }
  if (has2 && cos2 < cos1) {
    return Source::kDepth2;
  }
  return Source::kNone;
}

/// @brief Whether a point passes the chi-square test in a keyframe.
/// @param m The map.
/// @param camera The camera.
/// @param pose The keyframe's pose.
/// @param row The keypoint's row.
/// @param x The point.
/// @param options The thresholds.
/// @return Whether it passes, or the test's error.
Result<bool> reprojects(const Map& m, const Camera& camera, const Eigen::Matrix4d& pose,
                        std::int64_t row, const Eigen::Vector3d& x, const Options& options) {
  const auto r = static_cast<std::size_t>(row);
  const std::array<Eigen::Vector3d, 1> local{to_camera(pose, x)};
  const std::array<std::array<double, 2>, 1> xy{m.kp_xy[r]};
  const std::array<double, 1> right{m.kp_right[r]};
  const std::array<double, 1> variance{sigma2(m, m.kp_level[r])};
  auto error = chi2(camera, local, xy, right, variance);
  if (!error) {
    return std::unexpected(std::move(error.error()));
  }
  return (*error)[0] < threshold(m.kp_right[r], options);
}

/// @brief Whether a triangulated point's distances agree with its keypoints' levels.
/// @param m The map.
/// @param ks The two keyframes' rows.
/// @param pair The keypoint rows.
/// @param x The point.
/// @param options The scale ratio.
/// @return Whether they agree.
bool scale_consistent(const Map& m, std::array<std::size_t, 2> ks,
                      const std::array<std::int64_t, 2>& pair, const Eigen::Vector3d& x,
                      const Options& options) {
  const double distance1 = (x - centre(m, ks[0])).norm();
  const double distance2 = (x - centre(m, ks[1])).norm();
  if (!(distance1 > 0.0) || !(distance2 > 0.0)) {
    return false;
  }
  const double ratio_distance = distance2 / distance1;
  const double ratio_octave =
      std::pow(m.scale_factor, static_cast<double>(m.kp_level[static_cast<std::size_t>(pair[0])] -
                                                   m.kp_level[static_cast<std::size_t>(pair[1])]));
  const double factor = options.mapping.scale_ratio * m.scale_factor;
  return !(ratio_distance * factor < ratio_octave) && !(ratio_distance > ratio_octave * factor);
}

/// @brief Triangulate matched pairs and keep those passing every check
///        (`slam_it.edge.mapping._triangulate`).
/// @param m The map.
/// @param camera The camera.
/// @param ks The two keyframes' rows.
/// @param pairs The keypoint rows.
/// @param options The thresholds.
/// @return The kept points and their pairs' observers, or an error.
Result<std::pair<std::vector<Eigen::Vector3d>, std::vector<std::vector<std::int64_t>>>> triangulate(
    const Map& m, const Camera& camera, std::array<std::size_t, 2> ks,
    const std::vector<std::array<std::int64_t, 2>>& pairs, const Options& options) {
  std::vector<Source> sources;
  sources.reserve(pairs.size());
  std::vector<std::size_t> by_rays;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    sources.push_back(choose_source(m, camera, ks, pairs[i], options));
    if (sources.back() == Source::kRays) {
      by_rays.push_back(i);
    }
  }
  std::vector<Eigen::Vector3d> xyz(pairs.size(), Eigen::Vector3d::Constant(kNanPoint));
  if (!by_rays.empty()) {
    Rows3 bearings1(static_cast<Eigen::Index>(by_rays.size()), 3);
    Rows3 bearings2(static_cast<Eigen::Index>(by_rays.size()), 3);
    for (std::size_t j = 0; j < by_rays.size(); ++j) {
      const auto& pair = pairs[by_rays[j]];
      bearings1.row(static_cast<Eigen::Index>(j)) =
          m.kp_bearing[static_cast<std::size_t>(pair[0])].transpose();
      bearings2.row(static_cast<Eigen::Index>(j)) =
          m.kp_bearing[static_cast<std::size_t>(pair[1])].transpose();
    }
    const auto [rotation, t] = relative(m, ks[0], ks[1]);
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.topLeftCorner<3, 3>() = rotation;
    pose.topRightCorner<3, 1>() = t;
    auto found = foundations::triangulation::two_view(pose, bearings1, bearings2);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    for (std::size_t j = 0; j < by_rays.size(); ++j) {
      xyz[by_rays[j]] =
          to_world(m.kf_pose[ks[0]], found->points.row(static_cast<Eigen::Index>(j)).transpose());
    }
  }
  std::pair<std::vector<Eigen::Vector3d>, std::vector<std::vector<std::int64_t>>> out;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    if (sources[i] == Source::kDepth1) {
      xyz[i] = from_depth(m, m.kf_pose[ks[0]], pairs[i][0]);
    } else if (sources[i] == Source::kDepth2) {
      xyz[i] = from_depth(m, m.kf_pose[ks[1]], pairs[i][1]);
    }
    if (std::isnan(xyz[i].x())) {
      continue;
    }
    auto in1 = reprojects(m, camera, m.kf_pose[ks[0]], pairs[i][0], xyz[i], options);
    auto in2 = reprojects(m, camera, m.kf_pose[ks[1]], pairs[i][1], xyz[i], options);
    if (!in1 || !in2) {
      return std::unexpected(std::move(!in1 ? in1.error() : in2.error()));
    }
    if (*in1 && *in2 && scale_consistent(m, ks, pairs[i], xyz[i], options)) {
      out.first.push_back(xyz[i]);
      out.second.push_back({pairs[i][0], pairs[i][1]});
    }
  }
  return out;
}

/// @brief A keyframe's median scene depth, the lower middle (`ComputeSceneMedianDepth(2)`).
/// @param m The map.
/// @param k The keyframe's row.
/// @return The depth, or nothing if it sees no point.
std::optional<double> median_depth(const Map& m, std::size_t k) {
  std::vector<double> depths;
  for (const std::int64_t row : keypoint_rows(m, k)) {
    if (const std::int64_t p = point_row(m, m.kp_point[static_cast<std::size_t>(row)]); p >= 0) {
      depths.push_back(to_camera(m.kf_pose[k], m.pt_xyz[static_cast<std::size_t>(p)]).z());
    }
  }
  if (depths.empty()) {
    return std::nullopt;
  }
  std::ranges::sort(depths);
  return depths[(depths.size() - 1) / 2];
}

/// @brief The keypoint row a point fuses into in a keyframe, if any
///        (`slam_it.edge.mapping._fuse_target`, `Fuse`' search).
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param grid Its keypoints' grid.
/// @param p The point's row.
/// @param options The window and thresholds.
/// @return The row, nothing, or an error.
Result<std::optional<std::int64_t>> fuse_target(const Map& m, const Camera& camera, std::size_t k,
                                                const matching::FeatureGrid& grid, std::size_t p,
                                                const Options& options) {
  const Eigen::Matrix4d& pose = m.kf_pose[k];
  const Eigen::Vector3d at = to_camera(pose, m.pt_xyz[p]);
  if (!(at.z() > 0.0)) {
    return std::nullopt;
  }
  const Eigen::Vector2d pixel = project(camera, at);
  const Eigen::Vector3d offset = m.pt_xyz[p] - centre(m, k);
  const double distance = offset.norm();
  if (!inside(camera, pixel) || !in_distance_range(m, p, distance, options) ||
      !(viewing_cos(m, p, offset, distance) >= options.tracking.view_cos_limit)) {
    return std::nullopt;
  }
  auto levels = predict_level(m, std::vector<std::int64_t>{static_cast<std::int64_t>(p)},
                              std::vector<double>{distance});
  if (!levels) {
    return std::unexpected(std::move(levels.error()));
  }
  const std::int64_t level = (*levels)[0];
  auto near = grid.in_area(
      {pixel.x(), pixel.y()},
      options.mapping.fuse_window * std::pow(m.scale_factor, static_cast<double>(level)), level - 1,
      level);
  if (!near) {
    return std::unexpected(std::move(near.error()));
  }
  std::optional<std::int64_t> best_row;
  std::int64_t best = std::numeric_limits<std::int64_t>::max();
  for (const std::int64_t j : *near) {
    const std::int64_t row = m.kf_start[k] + j;
    auto ok = reprojects(m, camera, pose, row, m.pt_xyz[p], options);
    if (!ok) {
      return std::unexpected(std::move(ok.error()));
    }
    const std::int64_t d = foundations::orb::distance(
        m.pt_descriptor[p], m.kp_descriptor[static_cast<std::size_t>(row)]);
    if (*ok && d < best) {
      best = d;
      best_row = row;
    }
  }
  if (best > options.matching.th_low) {
    return std::nullopt;
  }
  return best_row;
}

/// @brief Fuse one point into a keyframe (`slam_it.edge.mapping._fuse_one`).
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param grid Its keypoints' grid.
/// @param id The point's id.
/// @param options The window and thresholds.
/// @return Nothing, or an error.
Result<void> fuse_one(Map& m, const Camera& camera, std::size_t k,
                      const matching::FeatureGrid& grid, std::int64_t id, const Options& options) {
  const std::int64_t p = point_row(m, id);
  if (p < 0 || std::ranges::any_of(
                   m.pt_observations[static_cast<std::size_t>(p)],
                   [&m, k](std::int64_t r) { return keyframe_of(m, r) == k; })) {
    return {};  // merged away, or already in the keyframe
  }
  auto target = fuse_target(m, camera, k, grid, static_cast<std::size_t>(p), options);
  if (!target) {
    return std::unexpected(std::move(target.error()));
  }
  if (!*target) {
    return {};
  }
  const auto row = static_cast<std::size_t>(**target);
  const std::int64_t held = m.kp_point[row];
  if (held < 0) {
    return observe(m, static_cast<std::int64_t>(row), id);
  }
  auto weights = observation_weight(m, std::vector<std::int64_t>{point_row(m, held), p});
  if (!weights) {
    return std::unexpected(std::move(weights.error()));
  }
  return (*weights)[0] > (*weights)[1] ? replace_point(m, id, held) : replace_point(m, held, id);
}

/// @brief Fuse points into a keyframe, by id.
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param ids The points' ids.
/// @param options The window and thresholds.
/// @return Nothing, or an error.
Result<void> fuse_into(Map& m, const Camera& camera, std::size_t k,
                       const std::vector<std::int64_t>& ids, const Options& options) {
  auto grid = matching::FeatureGrid::build(keyframe_keypoints(m, k));
  if (!grid) {
    return std::unexpected(std::move(grid.error()));
  }
  for (const std::int64_t id : ids) {
    if (auto ok = fuse_one(m, camera, k, *grid, id, options); !ok) {
      return ok;
    }
  }
  return {};
}

/// @brief The ids of the points some keyframes observe.
/// @param m The map.
/// @param keyframes Their rows.
/// @return The ids, ascending.
std::vector<std::int64_t> observed_ids(const Map& m, std::span<const std::int64_t> keyframes) {
  std::vector<std::int64_t> out;
  for (const std::int64_t p : observed_by(m, keyframes)) {
    out.push_back(m.pt_id[static_cast<std::size_t>(p)]);
  }
  return out;
}

/// @brief Count a keyframe's points that enough other keyframes see at its level or finer
///        (`slam_it.edge.mapping._redundant`).
/// @param m The map.
/// @param k The keyframe's row.
/// @param rows The keypoint rows counted.
/// @param threshold The observers needed.
/// @return The count, or the map's error.
Result<std::int64_t> redundant(const Map& m, std::size_t k, const std::vector<std::int64_t>& rows,
                               std::int64_t threshold) {
  std::vector<std::int64_t> point_rows;
  point_rows.reserve(rows.size());
  for (const std::int64_t row : rows) {
    point_rows.push_back(point_row(m, m.kp_point[static_cast<std::size_t>(row)]));
  }
  auto weights = observation_weight(m, point_rows);
  if (!weights) {
    return std::unexpected(std::move(weights.error()));
  }
  std::int64_t out = 0;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if ((*weights)[i] <= threshold) {
      continue;
    }
    const std::int64_t level = m.kp_level[static_cast<std::size_t>(rows[i])];
    const auto& seen = m.pt_observations[static_cast<std::size_t>(point_rows[i])];
    if (std::ranges::count_if(seen, [&m, k, level](std::int64_t r) {
          return keyframe_of(m, r) != k && m.kp_level[static_cast<std::size_t>(r)] <= level + 1;
        }) >= threshold) {
      ++out;
    }
  }
  return out;
}

}  // namespace

double close_depth(const Camera& camera, const Options& options) {
  return options.stereo.close_depth * camera.baseline;
}

const char* name(Scheduled scheduled) {
  switch (scheduled) {
    case Scheduled::kRan:
      return "ran";
    case Scheduled::kSkipped:
      return "skipped";
    case Scheduled::kAborted:
      return "aborted";
    case Scheduled::kCount:
      break;
  }
  return "unknown";
}

Result<bool> need_keyframe(const Map& m, const Camera& camera, std::span<const double> depth,
                           const Tracked& tracked, std::int64_t reference, Mode mode,
                           const Options& options, bool idle,
                           // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): two counts
                           std::optional<std::int64_t> since_keyframe,
                           std::optional<std::int64_t> keyframes) {
  if (auto ok = check_newest(m, camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = foundations::check_range("reference", reference, 0,
                                         static_cast<std::int64_t>(keyframe_count(m)) - 1);
      !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (tracked.points.size() != depth.size()) {
    return fail(ErrorKind::kInvalidInput,
                "tracked.points: expected one point id (or -1) per depth");
  }
  const std::int64_t total = keyframes.value_or(static_cast<std::int64_t>(keyframe_count(m)));
  const std::int64_t min_observations = total <= 2  // NeedNewKeyFrame's nKFs <= 2
                                            ? options.keyframes.ref_min_observations_early
                                            : options.keyframes.ref_min_observations;
  std::vector<std::int64_t> rows;
  for (const std::int64_t row : keypoint_rows(m, static_cast<std::size_t>(reference))) {
    if (const std::int64_t p = point_row(m, m.kp_point[static_cast<std::size_t>(row)]); p >= 0) {
      rows.push_back(p);
    }
  }
  auto weights = observation_weight(m, rows);
  if (!weights) {
    return std::unexpected(std::move(weights.error()));
  }
  const auto tracked_reference = std::ranges::count_if(
      *weights, [min_observations](std::int64_t w) { return w >= min_observations; });
  bool need_close = false;
  if (mode != Mode::kMonocular) {
    const auto [tracked_close, untracked_close] =
        close_counts(depth, tracked, close_depth(camera, options));
    need_close = tracked_close < options.keyframes.close_tracked &&
                 untracked_close > options.keyframes.close_untracked;
  }
  double ratio = total < 2 ? options.keyframes.first_ref_ratio : options.keyframes.ref_ratio;
  if (mode == Mode::kMonocular) {
    ratio = options.keyframes.mono_ref_ratio;
  }
  const std::int64_t count = inliers(tracked);
  const bool weak =
      static_cast<double>(count) < static_cast<double>(tracked_reference) * ratio || need_close;
  const bool wanted = weak && count > options.keyframes.min_inliers;  // c2
  if (idle) {  // c1b: no frames need pass, with mapping idle
    return wanted;
  }
  const bool late =
      since_keyframe.has_value() && *since_keyframe >= options.tracking.max_frames;  // c1a
  const bool poor = mode != Mode::kMonocular &&                                      // c1c
                    (static_cast<double>(count) < static_cast<double>(tracked_reference) *
                                                      options.keyframes.busy_ref_ratio ||
                     need_close);
  return wanted && (late || poor);
}

Result<void> create_points_from_depth(Map& m, const Camera& camera, const Options& options) {
  if (auto ok = check_newest(m, camera, options); !ok) {
    return ok;
  }
  const std::size_t k = keyframe_count(m) - 1;
  std::vector<std::int64_t> order;
  for (const std::int64_t row : keypoint_rows(m, k)) {
    const auto r = static_cast<std::size_t>(row);
    if (m.kp_depth[r] > 0.0 && m.kp_bearing[r].z() > 0.0) {
      order.push_back(row);
    }
  }
  std::ranges::stable_sort(order, [&m](std::int64_t a, std::int64_t b) {
    return m.kp_depth[static_cast<std::size_t>(a)] < m.kp_depth[static_cast<std::size_t>(b)];
  });
  const double limit = close_depth(camera, options);
  std::vector<Eigen::Vector3d> xyz;
  std::vector<std::vector<std::int64_t>> observers;
  std::int64_t count = 0;
  for (const std::int64_t row : order) {
    const auto r = static_cast<std::size_t>(row);
    if (m.kp_point[r] < 0) {
      xyz.push_back(from_depth(m, m.kf_pose[k], row));
      observers.push_back({row});
    }
    ++count;
    if (m.kp_depth[r] > limit && count > options.mapping.min_new_points) {
      break;
    }
  }
  if (xyz.empty()) {
    return {};
  }
  return add_points(m, xyz, observers);
}

Result<void> cull_points(Map& m, Mode mode, const Options& options) {
  if (auto ok = check_options(options); !ok) {
    return ok;
  }
  if (keyframe_count(m) == 0) {
    return fail(ErrorKind::kInvalidInput, "m: expected a map with at least one keyframe");
  }
  const std::int64_t current = m.kf_id.back();
  std::vector<std::int64_t> recent;
  for (std::size_t p = 0; p < point_count(m); ++p) {
    if (current - m.pt_first_kf[p] <= options.mapping.recent_point_keyframes) {
      recent.push_back(static_cast<std::int64_t>(p));
    }
  }
  if (recent.empty()) {
    return {};
  }
  const std::int64_t minimum = mode == Mode::kMonocular
                                   ? options.mapping.min_point_observations
                                   : options.mapping.min_point_observations_depth;
  auto weights = observation_weight(m, recent);
  if (!weights) {
    return std::unexpected(std::move(weights.error()));
  }
  std::vector<std::uint8_t> erase(point_count(m), 0);
  for (std::size_t i = 0; i < recent.size(); ++i) {
    const auto p = static_cast<std::size_t>(recent[i]);
    const double ratio = static_cast<double>(m.pt_found[p]) / static_cast<double>(m.pt_visible[p]);
    const std::int64_t age = current - m.pt_first_kf[p];
    erase[p] = (ratio < options.mapping.found_ratio ||
                (age >= options.mapping.point_cull_age && (*weights)[i] <= minimum))
                   ? 1
                   : 0;
  }
  return erase_points(m, erase, Removal::kCulled);
}

Result<std::vector<std::array<std::int64_t, 2>>> match_for_triangulation(
    const Map& m, const Camera& camera, std::int64_t k1, std::int64_t k2, const Options& options) {
  if (auto ok = check_setup(camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const auto count = static_cast<std::int64_t>(keyframe_count(m));
  if (k1 < 0 || k1 >= count || k2 < 0 || k2 >= count || k1 == k2) {
    return fail(ErrorKind::kInvalidInput, "k1, k2: expected two different keyframe rows in [0, " +
                                              std::to_string(count) + ")");
  }
  return triangulation_pairs(m, camera, static_cast<std::size_t>(k1), static_cast<std::size_t>(k2),
                             options);
}

Result<void> triangulate_new_points(Map& m, const Camera& camera, Mode mode,
                                    const Options& options) {
  if (auto ok = check_newest(m, camera, options); !ok) {
    return ok;
  }
  const std::size_t k1 = keyframe_count(m) - 1;
  std::vector<std::int64_t> others = linked_rows(m, k1, options);
  const std::int64_t neighbours = mode == Mode::kMonocular
                                      ? options.mapping.mono_triangulation_neighbours
                                      : options.mapping.triangulation_neighbours;
  if (std::cmp_greater(others.size(), neighbours)) {
    others.resize(static_cast<std::size_t>(neighbours));
  }
  for (const std::int64_t other : others) {
    const auto k2 = static_cast<std::size_t>(other);
    const double baseline = (centre(m, k2) - centre(m, k1)).norm();
    if (mode == Mode::kMonocular) {
      const std::optional<double> median = median_depth(m, k2);
      if (!median || !(baseline / *median >= options.mapping.min_baseline_depth)) {
        continue;
      }
    } else if (baseline < camera.baseline) {
      continue;
    }
    const auto pairs = triangulation_pairs(m, camera, k1, k2, options);
    if (pairs.empty()) {
      continue;
    }
    auto kept = triangulate(m, camera, {k1, k2}, pairs, options);
    if (!kept) {
      return std::unexpected(std::move(kept.error()));
    }
    if (!kept->first.empty()) {
      if (auto ok = add_points(m, kept->first, kept->second); !ok) {
        return ok;
      }
    }
  }
  return {};
}

Result<void> fuse_points(Map& m, const Camera& camera, Mode mode, const Options& options) {
  if (auto ok = check_newest(m, camera, options); !ok) {
    return ok;
  }
  const std::vector<std::int64_t> targets = targets_of(m, mode, options);
  const std::vector<std::int64_t> newest{static_cast<std::int64_t>(keyframe_count(m) - 1)};
  const std::vector<std::int64_t> own = observed_ids(m, newest);
  for (const std::int64_t target : targets) {
    const auto k = static_cast<std::size_t>(keyframe_row(m, target));
    if (auto ok = fuse_into(m, camera, k, own, options); !ok) {
      return ok;
    }
  }
  std::vector<std::int64_t> rows;
  rows.reserve(targets.size());
  for (const std::int64_t target : targets) {
    rows.push_back(keyframe_row(m, target));
  }
  if (auto ok = fuse_into(m, camera, keyframe_count(m) - 1, observed_ids(m, rows), options); !ok) {
    return ok;
  }
  return refresh_points(m, observed_by(m, newest));
}

Result<std::vector<std::int64_t>> fuse_targets(const Map& m, Mode mode, const Options& options) {
  if (auto ok = check_keyframes(m, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return targets_of(m, mode, options);
}

Result<std::vector<std::int64_t>> local_ba_keyframes(const Map& m, const Options& options) {
  if (auto ok = check_keyframes(m, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return local_rows(m, options);
}

Result<void> cull_keyframes(Map& m, const Camera& camera, Mode mode, const Options& options,
                            std::span<const std::int64_t> held) {
  if (auto ok = check_newest(m, camera, options); !ok) {
    return ok;
  }
  const double limit = close_depth(camera, options);
  std::vector<std::int64_t> ids;
  for (const std::int64_t k : linked_rows(m, keyframe_count(m) - 1, options)) {
    ids.push_back(m.kf_id[static_cast<std::size_t>(k)]);
  }
  for (const std::int64_t id : ids) {
    if (id == 0) {
      continue;
    }
    const auto k = static_cast<std::size_t>(keyframe_row(m, id));
    std::vector<std::int64_t> rows;
    for (const std::int64_t row : keypoint_rows(m, k)) {
      const auto r = static_cast<std::size_t>(row);
      if (m.kp_point[r] >= 0 &&
          (mode == Mode::kMonocular || (m.kp_depth[r] >= 0.0 && m.kp_depth[r] <= limit))) {
        rows.push_back(row);
      }
    }
    if (rows.empty()) {
      continue;
    }
    auto count = redundant(m, k, rows, options.mapping.redundant_observers);
    if (!count) {
      return std::unexpected(std::move(count.error()));
    }
    if (static_cast<double>(*count) >
        options.mapping.redundant_fraction * static_cast<double>(rows.size())) {
      if (auto ok = cull(m, static_cast<std::int64_t>(k), options, held); !ok) {
        return ok;
      }
    }
  }
  return {};
}

namespace {

/// @brief The points a keyframe inserted made, as `insert_and_make` gives them: the map's
///        point counts after each step, and each keypoint's point.
struct Made {
  /// @brief The points before the keyframe.
  std::int64_t start{0};
  /// @brief After the points from depth.
  std::int64_t with_depth{0};
  /// @brief After culling the recent points.
  std::int64_t culled{0};
  /// @brief After triangulation.
  std::int64_t triangulated{0};
  /// @brief The id of the point made for each keypoint, -1 where none.
  std::vector<std::int64_t> made;
};

/// @brief Local mapping's steps up to fusion: insert the keyframe, make its points from depth,
///        cull the recent points and triangulate new ones.
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param frame The keyframe's frame.
/// @param tracked Its pose and matches.
/// @param mode The sensor mode.
/// @param options The settings.
/// @param vocabulary The vocabulary, checked, the keyframe's features are quantised by.
/// @return The point counts and the points made; a step's error.
Result<Made> insert_and_make(Map& m, const Camera& camera, const Frame& frame,
                             const Tracked& tracked, Mode mode, const Options& options,
                             const foundations::vocabulary::Vocabulary& vocabulary) {
  const std::vector<std::int64_t> nodes =
      foundations::vocabulary::quantize(vocabulary, frame.features.descriptors).nodes;
  if (auto ok = insert_keyframe(m, frame, tracked.pose, tracked.points, nodes); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  // Its parent, as ORB-SLAM's first UpdateConnections sets it (after fusion if it shares none).
  if (auto ok = adopt(m, static_cast<std::int64_t>(keyframe_count(m)) - 1); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const auto points = [&m] { return static_cast<std::int64_t>(point_count(m)); };
  Made out;
  out.start = points();
  const std::int64_t first = m.next_pt_id;
  if (mode != Mode::kMonocular) {
    if (auto ok = create_points_from_depth(m, camera, options); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  out.with_depth = points();
  out.made = made_since(m, first);
  if (auto ok = cull_points(m, mode, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  out.culled = points();
  if (auto ok = triangulate_new_points(m, camera, mode, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  out.triangulated = points();
  // Taken before fusion, which may replace a new point; a depth point culled since stays made.
  const std::vector<std::int64_t> triangulated_ids = made_since(m, first);
  for (std::size_t j = 0; j < out.made.size(); ++j) {
    if (out.made[j] < 0) {
      out.made[j] = triangulated_ids[j];
    }
  }
  return out;
}

}  // namespace

Result<Mapped> process(Map& m, const Camera& camera, const Frame& frame, const Tracked& tracked,
                       Mode mode, const Options& options,
                       const foundations::vocabulary::Vocabulary& vocabulary,
                       std::span<const std::int64_t> held, const Interrupts* interrupts) {
  auto made = insert_and_make(m, camera, frame, tracked, mode, options, vocabulary);
  if (!made) {
    return std::unexpected(std::move(made.error()));
  }
  const auto newest = static_cast<std::int64_t>(keyframe_count(m)) - 1;
  const auto points = [&m] { return static_cast<std::int64_t>(point_count(m)); };
  Schedule schedule;
  if (interrupts != nullptr) {
    interrupts->publish(m);
  }
  if (interrupts == nullptr || !interrupts->waiting()) {
    if (auto ok = fuse_points(m, camera, mode, options); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  } else {
    schedule.fusion = Scheduled::kSkipped;
  }
  if (auto ok = adopt(m, newest); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::int64_t fused = points();
  const bool quiet = interrupts == nullptr || !(interrupts->waiting() || interrupts->stopping());
  if (!quiet) {
    schedule.adjustment = schedule.culling = Scheduled::kSkipped;
  }
  Adjustment adjustment;
  if (quiet && std::cmp_greater_equal(keyframe_count(m), options.adjustment.min_keyframes)) {
    auto adjusted = bundle_adjust(m, camera, local_rows(m, options), first_fixed(m), options,
                                  interrupts == nullptr ? nullptr : interrupts->abort,
                                  interrupts == nullptr ? nullptr : interrupts->around);
    if (!adjusted) {
      return std::unexpected(std::move(adjusted.error()));
    }
    adjustment = std::move(*adjusted);
    if (adjustment.aborted) {
      schedule.adjustment = Scheduled::kAborted;
    }
  }
  const auto keyframes = static_cast<std::int64_t>(keyframe_count(m));
  if (quiet) {
    if (auto ok = cull_keyframes(m, camera, mode, options, held); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  return Mapped{
      .summary = {.from_depth = made->with_depth - made->start,
                  .culled_points = made->with_depth - made->culled,
                  .triangulated = made->triangulated - made->culled,
                  .fused = made->triangulated - fused,
                  .solves = std::move(adjustment.solves),
                  .free = adjustment.free,
                  .fixed = adjustment.fixed,
                  .outliers = adjustment.outliers,
                  .weak_points = fused - points(),
                  .culled_keyframes = keyframes - static_cast<std::int64_t>(keyframe_count(m))},
      .made = std::move(made->made),
      .schedule = schedule};
}

}  // namespace slam_it::edge
