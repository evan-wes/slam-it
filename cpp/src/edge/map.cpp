/// @file
/// @brief Implementation of the map declared in slamlab/edge/map.hpp.
#include "slamlab/edge/map.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>

#include "slamlab/foundations/matching.hpp"

namespace slamlab::edge {

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

/// @brief Remove the flagged points, unchecked.
/// @param m The map.
/// @param erase One flag per point.
void erase(Map& m, const std::vector<bool>& erase) {
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
  erase(m, weak);
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
  const bool ok = m.kf_time.size() == k && m.kf_pose.size() == k && m.kf_start.size() == k + 1 &&
                  m.kp_xy.size() == t && m.kp_level.size() == t && m.kp_angle.size() == t &&
                  m.kp_descriptor.size() == t && m.kp_bearing.size() == t &&
                  m.kp_depth.size() == t && m.kp_right.size() == t && m.pt_xyz.size() == p &&
                  m.pt_descriptor.size() == p && m.pt_normal.size() == p &&
                  m.pt_min_distance.size() == p && m.pt_max_distance.size() == p &&
                  m.pt_visible.size() == p && m.pt_found.size() == p && m.pt_first_kf.size() == p &&
                  m.pt_reference_kf.size() == p;
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
  if (std::ranges::any_of(m.kp_point, [&](std::int64_t id) {
        return id < -1 || (id >= 0 && row_of(m.pt_id, id) < 0);
      })) {
    return fail(ErrorKind::kInvalidInput, "map.kp_point: every observed point must be in the map");
  }
  if (m.pt_observations != indexed(m)) {
    return fail(ErrorKind::kInvalidInput,
                "map.pt_observations: expected each point's observing keypoint rows, ascending");
  }
  return {};
}

void index_observations(Map& m) { m.pt_observations = indexed(m); }

std::int64_t point_row(const Map& m, std::int64_t id) { return row_of(m.pt_id, id); }

std::int64_t keyframe_row(const Map& m, std::int64_t id) { return row_of(m.kf_id, id); }

std::size_t keyframe_of(const Map& m, std::int64_t row) {
  return static_cast<std::size_t>(std::ranges::upper_bound(m.kf_start, row) - m.kf_start.begin()) -
         1;
}

Eigen::Vector3d centre(const Map& m, std::size_t k) {
  const Eigen::Matrix4d& pose = m.kf_pose[k];
  return -(pose.topLeftCorner<3, 3>().transpose() * pose.topRightCorner<3, 1>());
}

std::vector<std::int64_t> window(const Map& m, std::int64_t size) {
  const auto count = static_cast<std::int64_t>(keyframe_count(m));
  std::vector<std::int64_t> out;
  for (std::int64_t k = std::max<std::int64_t>(0, count - size); k < count; ++k) {
    out.push_back(k);
  }
  return out;
}

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

bool in_distance_range(const Map& m, std::size_t p, double distance, const Options& options) {
  return distance >= options.min_distance_margin * m.pt_min_distance[p] &&
         distance <= options.max_distance_margin * m.pt_max_distance[p];
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
      row[j] = foundations::matching::distance(descriptors[i], descriptors[j]);
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

Result<void> erase_points(Map& m, std::span<const std::uint8_t> erase_flags) {
  if (erase_flags.size() != point_count(m)) {
    return fail(ErrorKind::kInvalidInput, "erase: expected one flag per point");
  }
  std::vector<bool> flags(erase_flags.size());
  std::ranges::transform(erase_flags, flags.begin(), [](std::uint8_t f) { return f != 0; });
  erase(m, flags);
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
  m.kf_id.erase(m.kf_id.begin() + at);
  m.kf_time.erase(m.kf_time.begin() + at);
  m.kf_pose.erase(m.kf_pose.begin() + at);
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

}  // namespace slamlab::edge
