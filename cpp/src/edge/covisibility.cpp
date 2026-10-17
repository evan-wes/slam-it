/// @file
/// @brief Implementation of the covisibility graph declared in slam_it/edge/covisibility.hpp.
#include "slam_it/edge/covisibility.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace slam_it::edge {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

/// @brief Sort and deduplicate a vector in place.
/// @param values The vector.
void distinct(std::vector<std::int64_t>& values) {
  std::ranges::sort(values);
  const auto [first, last] = std::ranges::unique(values);
  values.erase(first, last);
}

/// @brief The keyframes that observe a point, each once.
/// @param m The map.
/// @param p The point's row.
/// @return Their rows, ascending.
std::vector<std::int64_t> observers(const Map& m, std::size_t p) {
  std::vector<std::int64_t> out;
  for (const std::int64_t row : m.pt_observations[p]) {
    out.push_back(static_cast<std::int64_t>(keyframe_of(m, row)));
  }
  distinct(out);
  return out;
}

/// @brief The rows sharing a point, most shared first, ties by the lower id.
/// @param m The map.
/// @param shared Each row's shared points.
/// @return The rows.
std::vector<std::int64_t> ranked(const Map& m, std::span<const std::int64_t> shared) {
  std::vector<std::int64_t> out;
  for (std::size_t r = 0; r < shared.size(); ++r) {
    if (shared[r] > 0) {
      out.push_back(static_cast<std::int64_t>(r));
    }
  }
  std::ranges::sort(out, [&](std::int64_t a, std::int64_t b) {
    const auto wa = shared[static_cast<std::size_t>(a)];
    const auto wb = shared[static_cast<std::size_t>(b)];
    return wa > wb || (wa == wb &&
                       m.kf_id[static_cast<std::size_t>(a)] < m.kf_id[static_cast<std::size_t>(b)]);
  });
  return out;
}

/// @brief Check a keyframe row.
/// @param m The map.
/// @param k The row.
/// @return Nothing, or `InvalidInput`.
Result<void> check_row(const Map& m, std::int64_t k) {
  return foundations::check_range("k", k, 0, static_cast<std::int64_t>(keyframe_count(m)) - 1);
}

/// @brief The rows whose parent is a keyframe, ascending.
/// @param m The map.
/// @param k The keyframe's row.
/// @return Its children's rows.
std::vector<std::size_t> children_of(const Map& m, std::size_t k) {
  std::vector<std::size_t> out;
  for (std::size_t r = 0; r < keyframe_count(m); ++r) {
    if (m.kf_parent[r] == m.kf_id[k]) {
      out.push_back(r);
    }
  }
  return out;
}

/// @brief Add a K1 keyframe's K2 to the local map, each before the next is looked for: the
///        first of its best links not yet in it, its first child not yet in it, and its parent
///        if not yet in it.
/// @param m The map.
/// @param k The K1 keyframe's row.
/// @param settings The `covisibility` settings.
/// @param keyframes The local map's rows, added to.
/// @param inside Whether each row is in it, updated.
void expand(const Map& m, std::size_t k, const CovisibilityOptions& settings,
            std::vector<std::int64_t>& keyframes, std::vector<bool>& inside) {
  const auto add = [&](std::int64_t row) {
    keyframes.push_back(row);
    inside[static_cast<std::size_t>(row)] = true;
  };
  const std::vector<Link> best = links_of(m, shared_points(m, k), settings.min_points);
  const std::size_t n = std::min(best.size(), static_cast<std::size_t>(settings.local_neighbours));
  for (std::size_t i = 0; i < n; ++i) {
    if (!inside[static_cast<std::size_t>(best[i].row)]) {
      add(best[i].row);
      break;
    }
  }
  for (const std::size_t child : children_of(m, k)) {
    if (!inside[child]) {
      add(static_cast<std::int64_t>(child));
      break;
    }
  }
  const std::int64_t parent = m.kf_parent[k] >= 0 ? keyframe_row(m, m.kf_parent[k]) : -1;
  if (parent >= 0 && !inside[static_cast<std::size_t>(parent)]) {
    add(parent);
  }
}

}  // namespace

std::vector<std::int64_t> shared_points(const Map& m, std::size_t k) {
  std::vector<std::int64_t> out(keyframe_count(m), 0);
  std::vector<std::int64_t> points;
  for (auto row = m.kf_start[k]; row < m.kf_start[k + 1]; ++row) {
    const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)];
    if (id >= 0) {
      points.push_back(point_row(m, id));
    }
  }
  distinct(points);
  for (const std::int64_t p : points) {
    for (const std::int64_t other : observers(m, static_cast<std::size_t>(p))) {
      if (std::cmp_not_equal(other, k)) {
        ++out[static_cast<std::size_t>(other)];
      }
    }
  }
  return out;
}

std::vector<Link> links_of(const Map& m, std::span<const std::int64_t> shared,
                           std::int64_t min_points) {
  const std::vector<std::int64_t> order = ranked(m, shared);
  std::vector<Link> out;
  for (const std::int64_t r : order) {
    const std::int64_t weight = shared[static_cast<std::size_t>(r)];
    if (weight >= min_points) {
      out.push_back({.row = r, .weight = weight});
    }
  }
  if (out.empty() && !order.empty()) {
    out.push_back(
        {.row = order.front(), .weight = shared[static_cast<std::size_t>(order.front())]});
  }
  return out;
}

Result<std::vector<Link>> links(const Map& m, std::int64_t k, const Options& options) {
  if (auto ok = check_row(m, k); !ok) {
    return std::unexpected(ok.error());
  }
  return links_of(m, shared_points(m, static_cast<std::size_t>(k)),
                  options.covisibility.min_points);
}

Result<LocalMap> local_map(const Map& m, std::span<const std::int64_t> points,
                           const Options& options) {
  if (std::ranges::any_of(points, [](std::int64_t id) { return id < -1; })) {
    return fail(ErrorKind::kInvalidInput, "points: expected point ids (-1 or more)");
  }
  std::vector<std::int64_t> matched;
  for (const std::int64_t id : points) {
    const std::int64_t p = id >= 0 ? point_row(m, id) : -1;
    if (p >= 0) {
      matched.push_back(p);
    }
  }
  distinct(matched);
  std::vector<std::int64_t> shared(keyframe_count(m), 0);
  for (const std::int64_t p : matched) {
    for (const std::int64_t k : observers(m, static_cast<std::size_t>(p))) {
      ++shared[static_cast<std::size_t>(k)];
    }
  }
  const std::vector<std::int64_t> k1 = ranked(m, shared);
  LocalMap out;
  if (k1.empty()) {
    return out;
  }
  const CovisibilityOptions& settings = options.covisibility;
  std::vector<bool> inside(keyframe_count(m), false);
  for (const std::int64_t k : k1) {
    out.keyframes.push_back(k);
    inside[static_cast<std::size_t>(k)] = true;
  }
  for (const std::int64_t k : k1) {
    if (std::cmp_greater(out.keyframes.size(), settings.local_keyframes)) {
      break;
    }
    expand(m, static_cast<std::size_t>(k), settings, out.keyframes, inside);
  }
  out.reference = k1.front();
  return out;
}

Result<void> adopt(Map& m, std::int64_t k) {
  if (auto ok = check_row(m, k); !ok) {
    return ok;
  }
  const auto row = static_cast<std::size_t>(k);
  if (m.kf_id[row] == 0 || m.kf_parent[row] >= 0) {
    return {};
  }
  const std::vector<std::int64_t> order = ranked(m, shared_points(m, row));
  if (!order.empty()) {
    m.kf_parent[row] = m.kf_id[static_cast<std::size_t>(order.front())];
  }
  return {};
}

Result<void> reparent(Map& m, std::int64_t k, const Options& options) {
  if (auto ok = check_row(m, k); !ok) {
    return ok;
  }
  const auto culled = static_cast<std::size_t>(k);
  std::vector<std::size_t> children = children_of(m, culled);
  if (children.empty()) {
    return {};
  }
  // Each child's links, the culled keyframe's dropped first, as SetBadFlag erases them.
  std::vector<std::vector<Link>> linked;
  for (const std::size_t child : children) {
    std::vector<std::int64_t> shared = shared_points(m, child);
    shared[culled] = 0;
    linked.push_back(links_of(m, shared, options.covisibility.min_points));
  }
  const std::int64_t parent = m.kf_parent[culled];
  std::vector<bool> candidate(keyframe_count(m), false);
  if (const std::int64_t row = parent >= 0 ? keyframe_row(m, parent) : -1; row >= 0) {
    candidate[static_cast<std::size_t>(row)] = true;
  }
  while (!children.empty()) {
    std::optional<std::pair<std::size_t, std::int64_t>> best;  // (child's index, candidate row)
    std::int64_t most = -1;
    for (std::size_t i = 0; i < children.size(); ++i) {
      for (const Link& link : linked[i]) {
        if (candidate[static_cast<std::size_t>(link.row)] && link.weight > most) {
          best = {i, link.row};
          most = link.weight;
        }
      }
    }
    if (!best) {
      break;
    }
    const auto [i, row] = *best;
    const std::size_t child = children[i];
    m.kf_parent[child] = m.kf_id[static_cast<std::size_t>(row)];
    candidate[child] = true;
    children.erase(children.begin() + static_cast<std::ptrdiff_t>(i));
    linked.erase(linked.begin() + static_cast<std::ptrdiff_t>(i));
  }
  for (const std::size_t child : children) {
    m.kf_parent[child] = parent;  // linked to no candidate: its own parent
  }
  return {};
}

Result<void> cull(Map& m, std::int64_t k, const Options& options) {
  if (auto ok = check_row(m, k); !ok) {
    return ok;
  }
  if (m.kf_id[static_cast<std::size_t>(k)] == 0) {
    return fail(ErrorKind::kInvalidInput,
                "k: keyframe 0 is the map's first, its tree's root, and is not culled");
  }
  if (auto ok = reparent(m, k, options); !ok) {
    return ok;
  }
  return erase_keyframe(m, k);
}

std::vector<std::array<std::int64_t, 3>> strong_links(const Map& m, const Options& options) {
  std::vector<std::array<std::int64_t, 3>> out;
  for (std::size_t a = 0; a < keyframe_count(m); ++a) {
    const std::vector<std::int64_t> shared = shared_points(m, a);
    for (std::size_t b = a + 1; b < shared.size(); ++b) {
      if (shared[b] >= options.covisibility.strong_points) {
        out.push_back({m.kf_id[a], m.kf_id[b], shared[b]});
      }
    }
  }
  return out;
}

}  // namespace slam_it::edge
