/// @file
/// @brief Implementation of the map declared in slam_it/edge/map.hpp.
#include "slam_it/edge/map.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <string>
#include <utility>

#include "slam_it/foundations/orb.hpp"

namespace slam_it::edge {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

/// @brief Each point's observing keypoint rows, ascending, found by scanning `kp_point`.
/// @param m The map.
/// @return One list per point.
std::vector<std::vector<std::int64_t>> indexed(const Map& m) {
  std::vector<std::vector<std::int64_t>> out(point_count(m));
  for (std::size_t row = 0; row < keypoint_count(m); ++row) {
    if (const std::int64_t p = point_row(m, m.kp_point[row]); p >= 0) {
      out[static_cast<std::size_t>(p)].push_back(static_cast<std::int64_t>(row));
    }
  }
  return out;
}

/// @brief Point a keypoint at nothing, unchecked.
/// @param m The map.
/// @param row The keypoint's row.
void detach(Map& m, std::int64_t row) {
  const auto r = static_cast<std::size_t>(row);
  if (const std::int64_t p = point_row(m, m.kp_point[r]); p >= 0) {
    std::vector<std::int64_t>& seen = m.pt_observations[static_cast<std::size_t>(p)];
    if (const auto at = std::ranges::lower_bound(seen, row); at != seen.end() && *at == row) {
      seen.erase(at);
    }
  }
  m.kp_point[r] = -1;
}

/// @brief Point a keypoint observing nothing at a point, unchecked.
/// @param m The map.
/// @param row The keypoint's row.
/// @param p The point's row.
void attach(Map& m, std::int64_t row, std::size_t p) {
  std::vector<std::int64_t>& seen = m.pt_observations[p];
  seen.insert(std::ranges::lower_bound(seen, row), row);
  m.kp_point[static_cast<std::size_t>(row)] = m.pt_id[p];
}

/// @brief Check rows against a bound.
/// @param rows The rows.
/// @param bound One past the largest valid row.
/// @param name Their name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_rows(std::span<const std::int64_t> rows, std::size_t bound, const char* name) {
  if (std::ranges::any_of(
          rows, [bound](std::int64_t r) { return r < 0 || std::cmp_greater_equal(r, bound); })) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": rows must lie in [0, " + std::to_string(bound) + ")");
  }
  return {};
}

/// @brief Keep the elements of a vector whose flag is set, in order.
/// @tparam T The element type.
/// @param values The vector.
/// @param keep One flag per element.
template <typename T>
void keep_where(std::vector<T>& values, const std::vector<bool>& keep) {
  std::size_t at = 0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (keep[i]) {
      if (at != i) {  // a vector moved onto itself is left empty
        values[at] = std::move(values[i]);
      }
      ++at;
    }
  }
  values.resize(at);
}

/// @brief Refresh points, unchecked.
/// @param m The map.
/// @param rows The points' rows, each observed.
void refresh(Map& m, std::span<const std::int64_t> rows) {
  const auto observer = [&m](std::int64_t r) { return m.kf_id[keyframe_of(m, r)]; };
  for (const std::int64_t row : rows) {
    const auto p = static_cast<std::size_t>(row);
    const std::vector<std::int64_t>& seen = m.pt_observations[p];
    std::vector<foundations::orb::Descriptor> descriptors;
    descriptors.reserve(seen.size());
    for (const std::int64_t r : seen) {
      descriptors.push_back(m.kp_descriptor[static_cast<std::size_t>(r)]);
    }
    const auto chosen = distinctive_descriptor(descriptors);
    m.pt_descriptor[p] = descriptors[static_cast<std::size_t>(*chosen)];
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const std::int64_t r : seen) {
      Eigen::Vector3d direction = m.pt_xyz[p] - centre(m, keyframe_of(m, r));
      direction /= direction.norm();
      sum += direction;
    }
    const Eigen::Vector3d mean = sum / static_cast<double>(seen.size());
    m.pt_normal[p] = mean / mean.norm();
    const bool still_reference = std::ranges::any_of(
        seen, [&](std::int64_t r) { return observer(r) == m.pt_reference_kf[p]; });
    if (!still_reference) {
      std::int64_t lowest = observer(seen.front());
      for (const std::int64_t r : seen) {
        lowest = std::min(lowest, observer(r));
      }
      m.pt_reference_kf[p] = lowest;
    }
    const std::int64_t at = *std::ranges::find_if(
        seen, [&](std::int64_t r) { return observer(r) == m.pt_reference_kf[p]; });
    const double distance = (m.pt_xyz[p] - centre(m, keyframe_of(m, at))).norm();
    m.pt_max_distance[p] =
        distance *
        std::pow(m.scale_factor, static_cast<double>(m.kp_level[static_cast<std::size_t>(at)]));
    m.pt_min_distance[p] =
        m.pt_max_distance[p] / std::pow(m.scale_factor, static_cast<double>(m.levels - 1));
  }
}

/// @brief Remove the flagged points, unchecked, keeping them aside in the archive.
/// @param m The map.
/// @param erase One flag per point.
/// @param how Why they go.
void erase(Map& m, const std::vector<bool>& erase, Removal how) {
  if (std::ranges::none_of(erase, [](bool e) { return e; })) {
    return;
  }
  std::vector<bool> keep(erase.size());
  for (std::size_t p = 0; p < erase.size(); ++p) {
    keep[p] = !erase[p];
    if (erase[p]) {
      for (const std::int64_t r : m.pt_observations[p]) {
        m.kp_point[static_cast<std::size_t>(r)] = -1;
      }
      m.gone_pt_id.push_back(m.pt_id[p]);
      m.gone_pt_xyz.push_back(m.pt_xyz[p]);
      m.gone_pt_how.push_back(static_cast<std::int64_t>(how));
    }
  }
  keep_where(m.pt_id, keep);
  keep_where(m.pt_xyz, keep);
  keep_where(m.pt_descriptor, keep);
  keep_where(m.pt_normal, keep);
  keep_where(m.pt_min_distance, keep);
  keep_where(m.pt_max_distance, keep);
  keep_where(m.pt_visible, keep);
  keep_where(m.pt_found, keep);
  keep_where(m.pt_first_kf, keep);
  keep_where(m.pt_reference_kf, keep);
  keep_where(m.pt_observations, keep);
}

/// @brief Points' observation weights, unchecked.
/// @param m The map.
/// @param rows The points' rows.
/// @return The weights.
std::vector<std::int64_t> weights(const Map& m, std::span<const std::int64_t> rows) {
  std::vector<std::int64_t> out;
  out.reserve(rows.size());
  for (const std::int64_t p : rows) {
    std::int64_t weight = 0;
    for (const std::int64_t r : m.pt_observations[static_cast<std::size_t>(p)]) {
      weight += std::isnan(m.kp_right[static_cast<std::size_t>(r)]) ? 1 : 2;
    }
    out.push_back(weight);
  }
  return out;
}

/// @brief Forget observations, unchecked.
/// @param m The map.
/// @param rows Keypoint rows.
void forget(Map& m, std::span<const std::int64_t> rows) {
  std::set<std::int64_t> touched;
  for (const std::int64_t r : rows) {
    if (m.kp_point[static_cast<std::size_t>(r)] >= 0) {
      touched.insert(m.kp_point[static_cast<std::size_t>(r)]);
    }
  }
  if (touched.empty()) {
    return;
  }
  for (const std::int64_t r : rows) {
    detach(m, r);
  }
  std::vector<std::int64_t> points;
  for (const std::int64_t id : touched) {
    if (const std::int64_t p = point_row(m, id); p >= 0) {
      points.push_back(p);
    }
  }
  const std::vector<std::int64_t> weight = weights(m, points);
  std::vector<bool> weak(point_count(m), false);
  std::vector<std::int64_t> remaining;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (weight[i] <= 2) {
      weak[static_cast<std::size_t>(points[i])] = true;
    } else {
      remaining.push_back(points[i]);
    }
  }
  refresh(m, remaining);
  erase(m, weak, Removal::kWeak);
}

/// @brief Find an id in an ascending id column.
/// @param ids The column.
/// @param id The id.
/// @return Its row, or -1.
std::int64_t row_of(const std::vector<std::int64_t>& ids, std::int64_t id) {
  if (id < 0) {
    return -1;
  }
  const auto at = std::ranges::lower_bound(ids, id);
  return at != ids.end() && *at == id ? static_cast<std::int64_t>(at - ids.begin()) : -1;
}

/// @brief Check that every array of a table has its row count.
/// @param m The map.
/// @return Nothing, or `InvalidInput`.
Result<void> check_lengths(const Map& m) {
  const std::size_t k = m.kf_id.size();
  const std::size_t t = m.kp_point.size();
  const std::size_t p = m.pt_id.size();
  const bool ok = m.kf_time.size() == k && m.kf_pose.size() == k && m.kf_parent.size() == k &&
                  m.kf_start.size() == k + 1 && m.kp_xy.size() == t && m.kp_level.size() == t &&
                  m.kp_angle.size() == t && m.kp_descriptor.size() == t &&
                  m.kp_bearing.size() == t && m.kp_depth.size() == t && m.kp_right.size() == t &&
                  m.pt_xyz.size() == p && m.pt_descriptor.size() == p && m.pt_normal.size() == p &&
                  m.pt_min_distance.size() == p && m.pt_max_distance.size() == p &&
                  m.pt_visible.size() == p && m.pt_found.size() == p && m.pt_first_kf.size() == p &&
                  m.pt_reference_kf.size() == p && m.gone_pt_xyz.size() == m.gone_pt_id.size() &&
                  m.gone_pt_how.size() == m.gone_pt_id.size() &&
                  m.gone_kf_time.size() == m.gone_kf_id.size() &&
                  m.gone_kf_pose.size() == m.gone_kf_id.size() &&
                  m.gone_kf_parent.size() == m.gone_kf_id.size() &&
                  m.gone_kf_relative.size() == m.gone_kf_id.size();
  if (!ok) {
    return fail(ErrorKind::kInvalidInput, "map: every table's arrays must have its row count");
  }
  return {};
}

/// @brief Check an id column: strictly ascending, from 0, below the next id.
/// @param ids The column.
/// @param next The next id.
/// @param name Its name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_ids(const std::vector<std::int64_t>& ids, std::int64_t next, const char* name) {
  const bool ascending = std::ranges::adjacent_find(ids, std::greater_equal<>()) == ids.end();
  if (!ascending || (!ids.empty() && (ids.front() < 0 || ids.back() >= next))) {
    return fail(ErrorKind::kInvalidInput,
                std::string("map.") + name + ": ids must be ascending, from 0, below the next id");
  }
  return {};
}

/// @brief Check an archive of erased ids: each one the map gave, once, and no longer holds.
/// @param gone The erased ids.
/// @param live The map's ids, ascending.
/// @param next The next id.
/// @return Nothing, or `InvalidInput`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (gone, live), the archive then the map
Result<void> check_gone(std::vector<std::int64_t> gone, const std::vector<std::int64_t>& live,
                        std::int64_t next) {
  std::ranges::sort(gone);
  const bool ok = std::ranges::adjacent_find(gone) == gone.end() &&
                  std::ranges::all_of(gone, [&](std::int64_t id) {
                    return id >= 0 && id < next && !std::ranges::binary_search(live, id);
                  });
  if (!ok) {
    return fail(ErrorKind::kInvalidInput,
                "map.gone_*: an erased id must be one the map gave, once, and no longer holds");
  }
  return {};
}

/// @brief Check the keyframes' links: each parent, and each loop edge.
/// @param m The map.
/// @return Nothing, or `InvalidInput`.
Result<void> check_links(const Map& m) {
  for (std::size_t k = 0; k < m.kf_parent.size(); ++k) {
    const std::int64_t parent = m.kf_parent[k];
    if (parent == m.kf_id[k] || (parent != -1 && row_of(m.kf_id, parent) < 0)) {
      return fail(ErrorKind::kInvalidInput,
                  "map.kf_parent: a parent must be -1 or another of its keyframes");
    }
  }
  for (const auto& [older, newer] : m.loop_edges) {
    if (older >= newer || row_of(m.kf_id, older) < 0 || row_of(m.kf_id, newer) < 0) {
      return fail(ErrorKind::kInvalidInput,
                  "map.loop_edges: each joins two of its keyframes, the older (lower id) first");
    }
  }
  return {};
}

/// @brief A keyframe's pose relative to its parent, `T_cam_parent`, or the identity.
/// @param m The map.
/// @param row The keyframe's row.
/// @return The relative pose.
Eigen::Matrix4d relative_to_parent(const Map& m, std::size_t row) {
  const std::int64_t parent = row_of(m.kf_id, m.kf_parent[row]);
  if (m.kf_parent[row] < 0 || parent < 0) {
    return Eigen::Matrix4d::Identity();
  }
  const Eigen::Matrix4d& pose = m.kf_pose[row];
  const Eigen::Matrix4d& of_parent = m.kf_pose[static_cast<std::size_t>(parent)];
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() =
      pose.topLeftCorner<3, 3>() * of_parent.topLeftCorner<3, 3>().transpose();
  out.topRightCorner<3, 1>() =
      pose.topRightCorner<3, 1>() - (out.topLeftCorner<3, 3>() * of_parent.topRightCorner<3, 1>());
  return out;
}

}  // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (levels, scale_factor), as in Python
Map empty_map(std::int64_t levels, double scale_factor) {
  Map m;
  m.levels = levels;
  m.scale_factor = scale_factor;
  return m;
}

Result<void> check_map(const Map& m) {
  if (auto ok = foundations::check_range("map.levels", m.levels, 1, foundations::orb::kMaxLevels);
      !ok) {
    return ok;
  }
  if (!std::isfinite(m.scale_factor) || m.scale_factor <= 1.0) {
    return foundations::fail_setting("map.scale_factor", "must be finite and above 1",
                                     m.scale_factor);
  }
  if (m.kf_start.empty() || m.kf_start.front() != 0 ||
      std::ranges::adjacent_find(m.kf_start, std::greater<>()) != m.kf_start.end() ||
      std::cmp_not_equal(m.kf_start.back(), m.kp_point.size())) {
    return fail(ErrorKind::kInvalidInput,
                "map.kf_start: offsets must run from 0 to the keypoint count, not decreasing");
  }
  if (auto ok = check_lengths(m); !ok) {
    return ok;
  }
  if (auto ok = check_ids(m.kf_id, m.next_kf_id, "kf_id"); !ok) {
    return ok;
  }
  if (auto ok = check_ids(m.pt_id, m.next_pt_id, "pt_id"); !ok) {
    return ok;
  }
  if (auto ok = check_links(m); !ok) {
    return ok;
  }
  if (std::ranges::any_of(m.kp_point, [&](std::int64_t id) {
        return id < -1 || (id >= 0 && row_of(m.pt_id, id) < 0);
      })) {
    return fail(ErrorKind::kInvalidInput, "map.kp_point: every observed point must be in the map");
  }
  if (m.pt_observations != indexed(m)) {
    return fail(ErrorKind::kInvalidInput,
                "map.pt_observations: expected each point's observing keypoint rows, ascending");
  }
  if (auto ok = check_gone(m.gone_pt_id, m.pt_id, m.next_pt_id); !ok) {
    return ok;
  }
  if (auto ok = check_gone(m.gone_kf_id, m.kf_id, m.next_kf_id); !ok) {
    return ok;
  }
  if (std::ranges::any_of(m.gone_pt_how, [](std::int64_t how) {
        return how < 0 || how >= static_cast<std::int64_t>(Removal::kCount);
      })) {
    return fail(ErrorKind::kInvalidInput, "map.gone_pt_how: expected Removal codes");
  }
  return {};
}

void clear_gone(Map& m) {
  m.gone_pt_id.clear();
  m.gone_pt_xyz.clear();
  m.gone_pt_how.clear();
  m.gone_kf_id.clear();
  m.gone_kf_time.clear();
  m.gone_kf_pose.clear();
  m.gone_kf_parent.clear();
  m.gone_kf_relative.clear();
}

void index_observations(Map& m) { m.pt_observations = indexed(m); }

std::int64_t point_row(const Map& m, std::int64_t id) { return row_of(m.pt_id, id); }

std::int64_t keyframe_row(const Map& m, std::int64_t id) { return row_of(m.kf_id, id); }

std::vector<std::int64_t> keypoint_rows(const Map& m, std::size_t k) {
  std::vector<std::int64_t> out(static_cast<std::size_t>(m.kf_start[k + 1] - m.kf_start[k]));
  std::ranges::iota(out, m.kf_start[k]);
  return out;
}

std::vector<foundations::orb::Keypoint> keyframe_keypoints(const Map& m, std::size_t k) {
  std::vector<foundations::orb::Keypoint> out;
  for (const std::int64_t row : keypoint_rows(m, k)) {
    const auto r = static_cast<std::size_t>(row);
    out.push_back({.xy = m.kp_xy[r],
                   .level = m.kp_level[r],
                   .level_xy = {},
                   .response = 0,
                   .angle = m.kp_angle[r]});
  }
  return out;
}

Result<std::size_t> checked_keyframe_row(const Map& m, std::int64_t id) {
  const std::int64_t k = keyframe_row(m, id);
  if (k < 0) {
    return fail(ErrorKind::kInvalidInput, "keyframe " + std::to_string(id) + " is not in the map");
  }
  return static_cast<std::size_t>(k);
}

std::size_t keyframe_of(const Map& m, std::int64_t row) {
  return static_cast<std::size_t>(std::ranges::upper_bound(m.kf_start, row) - m.kf_start.begin()) -
         1;
}

Eigen::Vector3d to_camera(const Eigen::Matrix4d& pose, const Eigen::Vector3d& x) {
  return (pose.topLeftCorner<3, 3>() * x) + pose.topRightCorner<3, 1>();
}

Eigen::Vector3d camera_centre(const Eigen::Matrix4d& pose) {
  return -(pose.topLeftCorner<3, 3>().transpose() * pose.topRightCorner<3, 1>());
}

Eigen::Vector3d centre(const Map& m, std::size_t k) { return camera_centre(m.kf_pose[k]); }

std::vector<std::int64_t> observed_by(const Map& m, std::span<const std::int64_t> keyframes) {
  std::set<std::int64_t> ids;
  for (const std::int64_t k : keyframes) {
    for (auto r = m.kf_start[static_cast<std::size_t>(k)];
         r < m.kf_start[static_cast<std::size_t>(k) + 1]; ++r) {
      if (m.kp_point[static_cast<std::size_t>(r)] >= 0) {
        ids.insert(m.kp_point[static_cast<std::size_t>(r)]);
      }
    }
  }
  std::vector<std::int64_t> out;
  for (const std::int64_t id : ids) {
    if (const std::int64_t p = point_row(m, id); p >= 0) {
      out.push_back(p);
    }
  }
  std::ranges::sort(out);
  return out;
}

double viewing_cos(const Map& m, std::size_t p, const Eigen::Vector3d& offset, double distance) {
  return offset.dot(m.pt_normal[p]) / distance;
}

bool in_distance_range(const Map& m, std::size_t p, double distance, const Options& options) {
  return distance >= options.tracking.min_distance_margin * m.pt_min_distance[p] &&
         distance <= options.tracking.max_distance_margin * m.pt_max_distance[p];
}

Result<std::vector<std::int64_t>> predict_level(const Map& m, std::span<const std::int64_t> rows,
                                                std::span<const double> distance) {
  if (auto ok = check_rows(rows, point_count(m), "rows"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (distance.size() != rows.size() ||
      std::ranges::any_of(distance, [](double d) { return !std::isfinite(d) || d <= 0.0; })) {
    return fail(ErrorKind::kInvalidInput, "distance: expected one positive distance per row");
  }
  const double log_scale = std::log(m.scale_factor);
  std::vector<std::int64_t> out;
  out.reserve(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const double ratio = m.pt_max_distance[static_cast<std::size_t>(rows[i])] / distance[i];
    const double level = std::ceil(std::log(ratio) / log_scale);
    out.push_back(
        static_cast<std::int64_t>(std::clamp(level, 0.0, static_cast<double>(m.levels - 1))));
  }
  return out;
}

Result<void> insert_keyframe(Map& m, const Frame& frame, const Eigen::Matrix4d& pose,
                             std::span<const std::int64_t> points) {
  if (frame.features.levels != m.levels || frame.features.scale_factor != m.scale_factor) {
    return fail(ErrorKind::kInvalidInput, "frame: its pyramid must be the map's");
  }
  if (!pose.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "pose: expected finite values");
  }
  const std::size_t n = keypoint_count(frame);
  if (points.size() != n || frame.features.descriptors.size() != n || frame.bearings.size() != n ||
      frame.depth.size() != n || frame.right.size() != n) {
    return fail(ErrorKind::kInvalidInput, "points: expected one id per keypoint");
  }
  std::set<std::int64_t> observed;
  for (const std::int64_t id : points) {
    if (id < -1 || (id >= 0 && (point_row(m, id) < 0 || !observed.insert(id).second))) {
      return fail(ErrorKind::kInvalidInput, "points: each must be in the map, and observed once");
    }
  }
  const std::int64_t first = m.kf_start.back();
  m.kf_id.push_back(m.next_kf_id++);
  m.kf_time.push_back(frame.timestamp);
  m.kf_pose.push_back(pose);
  m.kf_parent.push_back(-1);
  m.kf_start.push_back(first + static_cast<std::int64_t>(n));
  for (std::size_t i = 0; i < n; ++i) {
    const foundations::orb::Keypoint& k = frame.features.keypoints[i];
    m.kp_xy.push_back(k.xy);
    m.kp_level.push_back(k.level);
    m.kp_angle.push_back(k.angle);
    m.kp_descriptor.push_back(frame.features.descriptors[i]);
    m.kp_bearing.push_back(frame.bearings[i]);
    m.kp_depth.push_back(frame.depth[i]);
    m.kp_right.push_back(frame.right[i]);
    m.kp_point.push_back(points[i]);
    if (points[i] >= 0) {
      m.pt_observations[static_cast<std::size_t>(point_row(m, points[i]))].push_back(
          first + static_cast<std::int64_t>(i));
    }
  }
  std::vector<std::int64_t> rows;
  rows.reserve(observed.size());
  for (const std::int64_t id : observed) {
    rows.push_back(point_row(m, id));
  }
  refresh(m, rows);
  return {};
}

Result<void> add_points(Map& m, std::span<const Eigen::Vector3d> xyz,
                        std::span<const std::vector<std::int64_t>> observers) {
  if (observers.size() != xyz.size() ||
      std::ranges::any_of(xyz, [](const Eigen::Vector3d& x) { return !x.allFinite(); })) {
    return fail(ErrorKind::kInvalidInput,
                "xyz: expected finite points, and one row of observers per point");
  }
  std::set<std::int64_t> used;
  for (const std::vector<std::int64_t>& rows : observers) {
    std::set<std::size_t> keyframes;
    bool ok = !rows.empty();
    for (const std::int64_t r : rows) {
      ok = ok && r >= 0 && std::cmp_less(r, keypoint_count(m)) &&
           m.kp_point[static_cast<std::size_t>(r)] < 0 && used.insert(r).second &&
           keyframes.insert(keyframe_of(m, r)).second;
    }
    if (!ok) {
      return fail(ErrorKind::kInvalidInput,
                  "observers: each point needs free keypoint rows, used once, from different "
                  "keyframes");
    }
  }
  const std::size_t first = point_count(m);
  for (std::size_t i = 0; i < xyz.size(); ++i) {
    const std::int64_t id = m.next_pt_id++;
    std::vector<std::int64_t> seen = observers[i];
    std::ranges::sort(seen);
    for (const std::int64_t r : seen) {
      m.kp_point[static_cast<std::size_t>(r)] = id;
    }
    const std::int64_t creator = m.kf_id[keyframe_of(m, seen.back())];
    m.pt_id.push_back(id);
    m.pt_xyz.push_back(xyz[i]);
    m.pt_descriptor.push_back({});
    m.pt_normal.emplace_back(Eigen::Vector3d::Zero());
    m.pt_min_distance.push_back(0.0);
    m.pt_max_distance.push_back(0.0);
    m.pt_visible.push_back(1);
    m.pt_found.push_back(1);
    m.pt_first_kf.push_back(creator);
    m.pt_reference_kf.push_back(creator);
    m.pt_observations.push_back(std::move(seen));
  }
  std::vector<std::int64_t> rows;
  for (std::size_t p = first; p < point_count(m); ++p) {
    rows.push_back(static_cast<std::int64_t>(p));
  }
  refresh(m, rows);
  return {};
}

Result<std::int64_t> distinctive_descriptor(
    std::span<const foundations::orb::Descriptor> descriptors) {
  if (descriptors.empty()) {
    return fail(ErrorKind::kInvalidInput, "descriptors: expected at least one");
  }
  const std::size_t n = descriptors.size();
  std::size_t best = 0;
  std::int64_t best_median = std::numeric_limits<std::int64_t>::max();
  std::vector<std::int64_t> row(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      row[j] = foundations::orb::distance(descriptors[i], descriptors[j]);
    }
    std::ranges::sort(row);
    const std::int64_t median = row[(n - 1) / 2];
    if (median < best_median) {
      best_median = median;
      best = i;
    }
  }
  return static_cast<std::int64_t>(best);
}

Result<void> refresh_points(Map& m, std::span<const std::int64_t> rows) {
  if (auto ok = check_rows(rows, point_count(m), "rows"); !ok) {
    return ok;
  }
  if (std::ranges::any_of(rows, [&m](std::int64_t p) {
        return m.pt_observations[static_cast<std::size_t>(p)].empty();
      })) {
    return fail(ErrorKind::kInvalidInput, "rows: every point must have an observation");
  }
  refresh(m, rows);
  return {};
}

Result<void> erase_points(Map& m, std::span<const std::uint8_t> erase_flags, Removal how) {
  if (erase_flags.size() != point_count(m)) {
    return fail(ErrorKind::kInvalidInput, "erase: expected one flag per point");
  }
  if (how >= Removal::kCount) {
    return fail(ErrorKind::kInvalidInput, "how: expected a Removal");
  }
  std::vector<bool> flags(erase_flags.size());
  std::ranges::transform(erase_flags, flags.begin(), [](std::uint8_t f) { return f != 0; });
  erase(m, flags, how);
  return {};
}

Result<std::vector<std::int64_t>> observation_weight(const Map& m,
                                                     std::span<const std::int64_t> rows) {
  if (auto ok = check_rows(rows, point_count(m), "rows"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return weights(m, rows);
}

Result<void> forget_observations(Map& m, std::span<const std::int64_t> rows) {
  if (auto ok = check_rows(rows, keypoint_count(m), "rows"); !ok) {
    return ok;
  }
  forget(m, rows);
  return {};
}

Result<void> erase_keyframe(Map& m, std::int64_t k) {
  if (auto ok =
          foundations::check_range("k", k, 0, static_cast<std::int64_t>(keyframe_count(m)) - 1);
      !ok) {
    return ok;
  }
  const auto row = static_cast<std::size_t>(k);
  if (const auto child = std::ranges::find(m.kf_parent, m.kf_id[row]); child != m.kf_parent.end()) {
    const auto id = m.kf_id[static_cast<std::size_t>(child - m.kf_parent.begin())];
    return fail(ErrorKind::kInvalidInput,
                "k: keyframe " + std::to_string(m.kf_id[row]) + " is still keyframe " +
                    std::to_string(id) +
                    "'s parent (covisibility's cull gives its children new parents first)");
  }
  if (has_loop(m, row)) {
    return fail(ErrorKind::kInvalidInput,
                "k: keyframe " + std::to_string(m.kf_id[row]) + " has a loop edge, and is kept");
  }
  const Eigen::Matrix4d relative = relative_to_parent(m, row);
  const std::int64_t start = m.kf_start[row];
  const std::int64_t stop = m.kf_start[row + 1];
  std::vector<std::int64_t> rows;
  for (std::int64_t r = start; r < stop; ++r) {
    rows.push_back(r);
  }
  forget(m, rows);
  std::vector<bool> keep(keypoint_count(m), true);
  for (std::int64_t r = start; r < stop; ++r) {
    keep[static_cast<std::size_t>(r)] = false;
  }
  keep_where(m.kp_xy, keep);
  keep_where(m.kp_level, keep);
  keep_where(m.kp_angle, keep);
  keep_where(m.kp_descriptor, keep);
  keep_where(m.kp_bearing, keep);
  keep_where(m.kp_depth, keep);
  keep_where(m.kp_right, keep);
  keep_where(m.kp_point, keep);
  const auto at = static_cast<std::ptrdiff_t>(row);
  m.gone_kf_id.push_back(m.kf_id[row]);
  m.gone_kf_time.push_back(m.kf_time[row]);
  m.gone_kf_pose.push_back(m.kf_pose[row]);
  m.gone_kf_parent.push_back(m.kf_parent[row]);
  m.gone_kf_relative.push_back(relative);
  m.kf_id.erase(m.kf_id.begin() + at);
  m.kf_time.erase(m.kf_time.begin() + at);
  m.kf_pose.erase(m.kf_pose.begin() + at);
  m.kf_parent.erase(m.kf_parent.begin() + at);
  m.kf_start.erase(m.kf_start.begin() + at + 1);
  for (std::size_t i = row + 1; i < m.kf_start.size(); ++i) {
    m.kf_start[i] -= stop - start;
  }
  for (std::vector<std::int64_t>& seen : m.pt_observations) {
    for (std::int64_t& r : seen) {
      if (r >= stop) {
        r -= stop - start;
      }
    }
  }
  return {};
}

bool has_loop(const Map& m, std::size_t k) {
  return std::ranges::any_of(m.loop_edges, [&](const std::array<std::int64_t, 2>& edge) {
    return edge[0] == m.kf_id[k] || edge[1] == m.kf_id[k];
  });
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the edge's two ends, in either order
Result<void> add_loop_edge(Map& m, std::int64_t first, std::int64_t second) {
  if (first == second || row_of(m.kf_id, first) < 0 || row_of(m.kf_id, second) < 0) {
    return fail(ErrorKind::kInvalidInput, "loop edge " + std::to_string(first) + "-" +
                                              std::to_string(second) +
                                              ": expected two of the map's keyframes");
  }
  m.loop_edges.push_back({std::min(first, second), std::max(first, second)});
  return {};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (lose, keep), named as Python's
Result<void> replace_point(Map& m, std::int64_t lose, std::int64_t keep) {
  const std::int64_t r_lose = point_row(m, lose);
  const std::int64_t r_keep = point_row(m, keep);
  if (lose == keep || r_lose < 0 || r_keep < 0) {
    return fail(ErrorKind::kInvalidInput, "replace " + std::to_string(lose) + " by " +
                                              std::to_string(keep) +
                                              ": expected two of the map's points");
  }
  const auto p_lose = static_cast<std::size_t>(r_lose);
  const auto p_keep = static_cast<std::size_t>(r_keep);
  std::set<std::size_t> kept_in;
  for (const std::int64_t r : m.pt_observations[p_keep]) {
    kept_in.insert(keyframe_of(m, r));
  }
  const std::vector<std::int64_t> moving = m.pt_observations[p_lose];  // observe() changes it
  for (const std::int64_t r : moving) {
    if (auto ok = observe(m, r, kept_in.contains(keyframe_of(m, r)) ? -1 : keep); !ok) {
      return ok;
    }
  }
  m.pt_visible[p_keep] += m.pt_visible[p_lose];
  m.pt_found[p_keep] += m.pt_found[p_lose];
  std::vector<std::uint8_t> erase(point_count(m), 0);
  erase[p_lose] = 1;
  if (auto ok = erase_points(m, erase, Removal::kFused); !ok) {
    return ok;
  }
  const std::vector<std::int64_t> kept{point_row(m, keep)};
  return refresh_points(m, kept);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (row, id), a keypoint and its point
Result<void> observe(Map& m, std::int64_t row, std::int64_t id) {
  if (auto ok = check_rows(std::span(&row, 1), keypoint_count(m), "row"); !ok) {
    return ok;
  }
  const std::int64_t p = point_row(m, id);
  if (id != -1 && p < 0) {
    return fail(ErrorKind::kInvalidInput, "id: expected -1 or a point in the map");
  }
  detach(m, row);
  if (p >= 0) {
    attach(m, row, static_cast<std::size_t>(p));
  }
  return {};
}

Result<void> count(Map& m, std::span<const std::int64_t> ids, std::span<const std::int64_t> visible,
                   std::span<const std::int64_t> found) {
  if (visible.size() != ids.size() || found.size() != ids.size() ||
      std::ranges::any_of(visible, [](std::int64_t v) { return v < 0; }) ||
      std::ranges::any_of(found, [](std::int64_t f) { return f < 0; })) {
    return fail(ErrorKind::kInvalidInput,
                "visible, found: expected one non-negative increment per id");
  }
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (const std::int64_t p = point_row(m, ids[i]); p >= 0) {
      m.pt_visible[static_cast<std::size_t>(p)] += visible[i];
      m.pt_found[static_cast<std::size_t>(p)] += found[i];
    }
  }
  return {};
}

}  // namespace slam_it::edge
