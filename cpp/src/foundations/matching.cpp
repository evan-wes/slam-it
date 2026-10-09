/// @file
/// @brief Implementation of the descriptor matching declared in
/// slam_it_all/foundations/matching.hpp.
#include "slam_it_all/foundations/matching.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace slam_it_all::foundations::matching {

namespace {

/// @brief Check a descriptor set's size and its angles.
/// @param count The number of descriptors.
/// @param angles Their orientation bins.
/// @param name The set's angles' name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_set(std::size_t count, std::span<const std::int64_t> angles, const char* name) {
  if (count > kMaxDescriptors) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": too many descriptors");
  }
  if (angles.size() != count) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected " +
                                              std::to_string(count) + " integers, got " +
                                              std::to_string(angles.size()));
  }
  if (std::ranges::any_of(angles, [](std::int64_t a) { return a < 0 || a >= orb::kAngleBins; })) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": bins must lie in [0, " +
                                              std::to_string(orb::kAngleBins) + ")");
  }
  return {};
}

/// @brief A first-set descriptor's nearest neighbour in the second set.
struct Nearest {
  /// @brief The nearest's index (ties: the lowest).
  std::size_t index{0};
  /// @brief Its distance.
  std::int64_t best{std::numeric_limits<std::int64_t>::max()};
  /// @brief The second-nearest distance (the best again on a tie).
  std::int64_t second{std::numeric_limits<std::int64_t>::max()};
};

/// @brief Find a descriptor's nearest and second-nearest distances in a set.
/// @param query The descriptor.
/// @param set The set, non-empty.
/// @return The nearest.
Nearest nearest(const orb::Descriptor& query, std::span<const orb::Descriptor> set) {
  Nearest out;
  for (std::size_t j = 0; j < set.size(); ++j) {
    const std::int64_t d = distance(query, set[j]);
    if (d < out.best) {
      out.second = out.best;
      out.best = d;
      out.index = j;
    } else if (d < out.second) {
      out.second = d;
    }
  }
  return out;
}

/// @brief The rotation histogram's kept bins: its highest three, the lower two only if they
///        hold at least a tenth of the highest (ORB-SLAM's `ComputeThreeMaxima`).
/// @param counts The histogram.
/// @return The kept bins, highest first.
std::vector<std::int64_t> three_maxima(const std::array<std::int64_t, orb::kAngleBins>& counts) {
  // (count, bin), highest first; a later bin with an equal count goes after, as in ORB-SLAM.
  std::vector<std::pair<std::int64_t, std::int64_t>> top;
  for (std::int64_t k = 0; k < orb::kAngleBins; ++k) {
    const std::int64_t count = counts.at(static_cast<std::size_t>(k));
    const auto place =
        std::ranges::find_if(top, [count](const auto& e) { return count > e.first; });
    top.insert(place, {count, k});
    if (top.size() > 3) {
      top.pop_back();
    }
  }
  std::vector<std::int64_t> kept{top.front().second};
  for (std::size_t place = 1; place < top.size(); ++place) {
    if (static_cast<double>(top[place].first) <
        kSecondaryFraction * static_cast<double>(top.front().first)) {
      break;
    }
    kept.push_back(top[place].second);
  }
  return kept;
}

/// @brief A candidate's rank: its distance, then its index (the lower ranks first).
struct Ranked {
  /// @brief The Hamming distance.
  std::int64_t distance{std::numeric_limits<std::int64_t>::max()};
  /// @brief The keypoint's index (the largest `std::size_t` for no candidate).
  std::size_t index{std::numeric_limits<std::size_t>::max()};
};

/// @brief Whether one candidate ranks before another.
/// @param a A candidate.
/// @param b Another.
/// @return True if `a` is nearer, or as near with a lower index.
bool before(const Ranked& a, const Ranked& b) {
  return a.distance < b.distance || (a.distance == b.distance && a.index < b.index);
}

/// @brief Whether a rank holds a candidate at all.
/// @param r The rank.
/// @return False for the empty rank.
bool found(const Ranked& r) { return r.index != std::numeric_limits<std::size_t>::max(); }

/// @brief Each second-side item's claimant: the nearest match offered for it, the first offered
///        on a tie (so the lowest first-side index, when offered in order).
class Claims {
 public:
  /// @brief No claims yet.
  /// @param count The second side's items.
  explicit Claims(std::size_t count) : held_(count) {}

  /// @brief Offer a match; it holds its item unless a nearer one already does.
  /// @param m The match.
  void offer(const Match& m) {
    auto& held = held_[static_cast<std::size_t>(m.second)];
    if (!held.has_value() || m.distance < held->distance) {
      held = m;
    }
  }

  /// @brief The holding matches.
  /// @return Them, by first-side index.
  [[nodiscard]] std::vector<Match> by_first() const {
    std::vector<Match> out;
    for (const auto& held : held_) {
      if (held.has_value()) {
        out.push_back(*held);
      }
    }
    std::ranges::sort(out, {}, &Match::first);
    return out;
  }

 private:
  /// @brief Each item's holder, if any.
  std::vector<std::optional<Match>> held_;
};

/// @brief Check that every value is finite or NaN.
/// @param values The values.
/// @param name Their name, for the message.
/// @return Nothing, or `InvalidInput` for an infinity.
Result<void> check_not_infinite(std::span<const double> values, const char* name) {
  if (std::ranges::any_of(values, [](double v) { return std::isinf(v); })) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected finite values");
  }
  return {};
}

/// @brief Check a window's centre and radius.
/// @param center The centre.
/// @param radius The half-width.
/// @param name The window's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_window(std::array<double, 2> center, double radius, const char* name) {
  if (!std::isfinite(center[0]) || !std::isfinite(center[1])) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected a finite centre");
  }
  if (!std::isfinite(radius) || radius <= 0.0) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": the radius must be finite and positive");
  }
  return {};
}

/// @brief Check a search's projections.
/// @param projections The projections.
/// @return Nothing, or `InvalidInput`.
Result<void> check_projections(std::span<const Projection> projections) {
  if (projections.size() > kMaxDescriptors) {
    return fail(ErrorKind::kInvalidInput, "projections: too many projections");
  }
  for (const Projection& p : projections) {
    if (auto ok = check_window(p.xy, p.radius, "projections"); !ok) {
      return ok;
    }
    if (p.angle < 0 || p.angle >= orb::kAngleBins) {
      return fail(ErrorKind::kInvalidInput, "projections.angle: bins must lie in [0, " +
                                                std::to_string(orb::kAngleBins) + ")");
    }
    if (std::isinf(p.right)) {
      return fail(ErrorKind::kInvalidInput, "projections.right: expected finite values");
    }
  }
  return {};
}

/// @brief Check a frame for a search: its features, grid and per-keypoint extras agree.
/// @param features The features.
/// @param grid Their grid.
/// @param right The right columns.
/// @param taken The taken flags.
/// @return Nothing, or `InvalidInput`.
Result<void> check_frame(const orb::Features& features, const FeatureGrid& grid,
                         std::span<const double> right, std::span<const std::uint8_t> taken) {
  const std::size_t n = features.keypoints.size();
  if (features.descriptors.size() != n || grid.size() != n || right.size() != n ||
      taken.size() != n) {
    return fail(ErrorKind::kInvalidInput,
                "features: the descriptors, grid, right columns and taken flags must have one "
                "entry per keypoint");
  }
  if (auto ok = check_range("features.levels", features.levels, 1, orb::kMaxLevels); !ok) {
    return ok;
  }
  for (const orb::Keypoint& k : features.keypoints) {
    if (k.level < 0 || k.level >= features.levels) {
      return fail(ErrorKind::kInvalidInput, "features.keypoints.level: levels must lie in [0, " +
                                                std::to_string(features.levels) + ")");
    }
    if (k.angle < 0 || k.angle >= orb::kAngleBins) {
      return fail(ErrorKind::kInvalidInput, "features.keypoints.angle: bins must lie in [0, " +
                                                std::to_string(orb::kAngleBins) + ")");
    }
  }
  return check_not_infinite(right, "right");
}

/// @brief Keep only matches whose rotation bin is among the histogram's three highest.
/// @tparam First A callable from a first-side index to its orientation bin.
/// @tparam Second The same for the second side.
/// @param matches The matches.
/// @param angles1 The first side's orientation bins, by `Match::first`.
/// @param angles2 The second side's, by `Match::second`.
template <typename First, typename Second>
void keep_consistent_rotations(std::vector<Match>& matches, First angles1, Second angles2) {
  if (matches.empty()) {
    return;
  }
  const auto rotation = [&](const Match& m) {
    const std::int64_t diff = angles1(m.first) - angles2(m.second);
    return ((diff % orb::kAngleBins) + orb::kAngleBins) % orb::kAngleBins;
  };
  std::array<std::int64_t, orb::kAngleBins> counts{};
  for (const Match& m : matches) {
    ++counts.at(static_cast<std::size_t>(rotation(m)));
  }
  const std::vector<std::int64_t> kept = three_maxima(counts);
  std::erase_if(matches, [&](const Match& m) { return !std::ranges::contains(kept, rotation(m)); });
}

/// @brief A window's nearest and second-nearest candidates, by `before`.
struct NearestTwo {
  /// @brief The nearest.
  Ranked best;
  /// @brief The second nearest.
  Ranked second;
};

/// @brief Rank a projection's candidates: free, within its right column, by distance.
/// @param p The projection.
/// @param candidates The keypoints in its window.
/// @param descriptors The frame's descriptors.
/// @param right The frame's right columns (NaN for none).
/// @param taken The frame's taken flags.
/// @return The nearest two (either may be empty).
NearestTwo nearest_two(const Projection& p, std::span<const std::uint32_t> candidates,
                       std::span<const orb::Descriptor> descriptors, std::span<const double> right,
                       std::span<const std::uint8_t> taken) {
  NearestTwo out;
  for (const std::uint32_t k : candidates) {
    // A NaN column on either side compares false: no constraint.
    if (taken[k] != 0 || std::fabs(right[k] - p.right) > p.radius) {
      continue;
    }
    const Ranked here{.distance = distance(p.descriptor, descriptors[k]), .index = k};
    if (before(here, out.best)) {
      out.second = out.best;
      out.best = here;
    } else if (before(here, out.second)) {
      out.second = here;
    }
  }
  return out;
}

}  // namespace

Result<void> check_options(const Options& options) {
  if (auto ok = check_range("max_distance", options.max_distance, 0, kMaxDistance); !ok) {
    return ok;
  }
  if (!std::isfinite(options.ratio) || options.ratio <= 0.0 || options.ratio > 1.0) {
    return fail_setting("ratio", "must be in (0, 1]", options.ratio);
  }
  return {};
}

std::int64_t distance(const orb::Descriptor& a, const orb::Descriptor& b) {
  static_assert(orb::kDescriptorBytes % sizeof(std::uint64_t) == 0,
                "the distance reads whole 64-bit words");
  std::int64_t bits = 0;
  for (std::size_t offset = 0; offset < orb::kDescriptorBytes; offset += sizeof(std::uint64_t)) {
    std::uint64_t x = 0;
    std::uint64_t y = 0;
    std::memcpy(&x, &a.at(offset), sizeof(x));
    std::memcpy(&y, &b.at(offset), sizeof(y));
    bits += std::popcount(x ^ y);
  }
  return bits;
}

Result<std::vector<std::int64_t>> distances(std::span<const orb::Descriptor> descriptors1,
                                            std::span<const orb::Descriptor> descriptors2) {
  if (descriptors1.size() > kMaxDescriptors || descriptors2.size() > kMaxDescriptors) {
    return fail(ErrorKind::kInvalidInput, "descriptors: too many descriptors");
  }
  std::vector<std::int64_t> out;
  out.reserve(descriptors1.size() * descriptors2.size());
  for (const orb::Descriptor& a : descriptors1) {
    for (const orb::Descriptor& b : descriptors2) {
      out.push_back(distance(a, b));
    }
  }
  return out;
}

Result<std::vector<Match>> match(std::span<const orb::Descriptor> descriptors1,
                                 std::span<const orb::Descriptor> descriptors2,
                                 std::span<const std::int64_t> angles1,
                                 std::span<const std::int64_t> angles2, const Options& options) {
  if (auto ok = check_options(options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_set(descriptors1.size(), angles1, "angles1"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_set(descriptors2.size(), angles2, "angles2"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (descriptors1.empty() || descriptors2.empty()) {
    return std::vector<Match>{};
  }
  // Each second-set descriptor's claimant: the nearest first-set match passing the filters
  // (ties: the lowest first-set index, which is offered first).
  Claims claims(descriptors2.size());
  for (std::size_t i = 0; i < descriptors1.size(); ++i) {
    const Nearest n = nearest(descriptors1[i], descriptors2);
    const bool passes = descriptors2.size() == 1 ||
                        static_cast<double>(n.best) < options.ratio * static_cast<double>(n.second);
    if (passes && n.best <= options.max_distance) {
      claims.offer({.first = static_cast<std::int64_t>(i),
                    .second = static_cast<std::int64_t>(n.index),
                    .distance = n.best});
    }
  }
  std::vector<Match> out = claims.by_first();
  if (options.check_orientation) {
    keep_consistent_rotations(
        out, [&](std::int64_t i) { return angles1[static_cast<std::size_t>(i)]; },
        [&](std::int64_t j) { return angles2[static_cast<std::size_t>(j)]; });
  }
  return out;
}

std::int64_t FeatureGrid::cell(double t, const Axis& axis) {
  const double at = (t - axis.origin) * axis.scale;
  if (!(at >= 0.0)) {  // below the box, or NaN: a scale of 0 times an infinite offset
    return 0;
  }
  const auto last = static_cast<double>(axis.cells - 1);
  return at >= last ? axis.cells - 1 : static_cast<std::int64_t>(std::floor(at));
}

FeatureGrid::Axis FeatureGrid::axis(std::span<const double> values, std::int64_t cells) {
  if (values.empty()) {
    return {.origin = 0.0, .scale = 0.0, .cells = cells};
  }
  const auto [low, high] = std::ranges::minmax(values);
  const double scale = static_cast<double>(cells) / (high - low);
  // No width, or too narrow (or too wide) for a finite scale: every keypoint in one cell.
  return {.origin = low, .scale = std::isfinite(scale) ? scale : 0.0, .cells = cells};
}

Result<FeatureGrid> FeatureGrid::build(std::span<const orb::Keypoint> keypoints) {
  if (keypoints.size() > kMaxDescriptors) {
    return fail(ErrorKind::kInvalidInput, "xy: too many keypoints");
  }
  FeatureGrid grid;
  grid.x_.reserve(keypoints.size());
  grid.y_.reserve(keypoints.size());
  grid.level_.reserve(keypoints.size());
  for (const orb::Keypoint& k : keypoints) {
    if (!std::isfinite(k.xy[0]) || !std::isfinite(k.xy[1])) {
      return fail(ErrorKind::kInvalidInput, "xy: expected finite values");
    }
    grid.x_.push_back(k.xy[0]);
    grid.y_.push_back(k.xy[1]);
    grid.level_.push_back(k.level);
  }
  grid.x_axis_ = axis(grid.x_, kGridCols);
  grid.y_axis_ = axis(grid.y_, kGridRows);
  const auto at = [&](std::size_t i) {
    return static_cast<std::size_t>((cell(grid.y_[i], grid.y_axis_) * kGridCols) +
                                    cell(grid.x_[i], grid.x_axis_));
  };
  constexpr auto kCells = static_cast<std::size_t>(kGridCols * kGridRows);
  grid.start_.assign(kCells + 1, 0);
  for (std::size_t i = 0; i < keypoints.size(); ++i) {
    ++grid.start_[at(i) + 1];
  }
  for (std::size_t c = 0; c < kCells; ++c) {
    grid.start_[c + 1] += grid.start_[c];
  }
  grid.items_.resize(keypoints.size());
  std::vector<std::uint32_t> next(grid.start_.begin(), grid.start_.end() - 1);
  for (std::size_t i = 0; i < keypoints.size(); ++i) {
    grid.items_[next[at(i)]++] = static_cast<std::uint32_t>(i);
  }
  return grid;
}

void FeatureGrid::collect(std::array<double, 2> center, double radius, std::int64_t min_level,
                          std::int64_t max_level, std::vector<std::uint32_t>& out) const {
  if (x_.empty() || min_level > max_level) {
    return;
  }
  const auto [u, v] = center;
  const std::int64_t x0 = cell(u - radius, x_axis_);
  const std::int64_t x1 = cell(u + radius, x_axis_);
  const std::int64_t y0 = cell(v - radius, y_axis_);
  const std::int64_t y1 = cell(v + radius, y_axis_);
  for (std::int64_t cy = y0; cy <= y1; ++cy) {
    for (std::int64_t cx = x0; cx <= x1; ++cx) {
      const auto c = static_cast<std::size_t>((cy * kGridCols) + cx);
      for (std::uint32_t k = start_[c]; k < start_[c + 1]; ++k) {
        const std::uint32_t i = items_[k];
        if (std::fabs(x_[i] - u) < radius && std::fabs(y_[i] - v) < radius &&
            level_[i] >= min_level && level_[i] <= max_level) {
          out.push_back(i);
        }
      }
    }
  }
}

Result<std::vector<std::int64_t>> FeatureGrid::in_area(std::array<double, 2> center, double radius,
                                                       std::int64_t min_level,
                                                       std::int64_t max_level) const {
  if (auto ok = check_window(center, radius, "window"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<std::uint32_t> found;
  collect(center, radius, min_level, max_level, found);
  std::ranges::sort(found);
  return std::vector<std::int64_t>(found.begin(), found.end());
}

Result<void> check_search_options(const SearchOptions& options) {
  return check_options(Options{.max_distance = options.max_distance,
                               .ratio = options.ratio,
                               .check_orientation = options.check_orientation});
}

Result<std::vector<std::int64_t>> features_in_area(std::span<const orb::Keypoint> keypoints,
                                                   std::array<double, 2> center, double radius,
                                                   std::int64_t min_level, std::int64_t max_level) {
  if (std::ranges::any_of(keypoints, [](const orb::Keypoint& k) { return k.level < 0; })) {
    return fail(ErrorKind::kInvalidInput, "level: levels must be non-negative");
  }
  const auto grid = FeatureGrid::build(keypoints);
  if (!grid) {
    return std::unexpected(grid.error());
  }
  return grid->in_area(center, radius, min_level, max_level);
}

Result<std::vector<Match>> search_by_projection(std::span<const Projection> projections,
                                                const orb::Features& features,
                                                const FeatureGrid& grid,
                                                std::span<const double> right,
                                                std::span<const std::uint8_t> taken,
                                                const SearchOptions& options) {
  if (auto ok = check_search_options(options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_projections(projections); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_frame(features, grid, right, taken); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  // Each keypoint's claimant: the nearest projection choosing it (ties: the lowest index).
  Claims claims(features.keypoints.size());
  std::vector<std::uint32_t> candidates;
  for (std::size_t i = 0; i < projections.size(); ++i) {
    const Projection& p = projections[i];
    candidates.clear();
    grid.collect(p.xy, p.radius, p.min_level, p.max_level, candidates);
    const auto [best, second] = nearest_two(p, candidates, features.descriptors, right, taken);
    if (!found(best) || best.distance > options.max_distance) {
      continue;
    }
    if (found(second) &&
        features.keypoints[second.index].level == features.keypoints[best.index].level &&
        static_cast<double>(best.distance) > options.ratio * static_cast<double>(second.distance)) {
      continue;
    }
    claims.offer({.first = static_cast<std::int64_t>(i),
                  .second = static_cast<std::int64_t>(best.index),
                  .distance = best.distance});
  }
  std::vector<Match> out = claims.by_first();
  if (options.check_orientation) {
    keep_consistent_rotations(
        out, [&](std::int64_t i) { return projections[static_cast<std::size_t>(i)].angle; },
        [&](std::int64_t k) { return features.keypoints[static_cast<std::size_t>(k)].angle; });
  }
  return out;
}

}  // namespace slam_it_all::foundations::matching
