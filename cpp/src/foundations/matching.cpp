/// @file
/// @brief Implementation of the descriptor matching declared in slamlab/foundations/matching.hpp.
#include "slamlab/foundations/matching.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace slamlab::foundations::matching {

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
  // (ties: the lowest first-set index, which is seen first).
  std::vector<std::optional<Match>> claim(descriptors2.size());
  for (std::size_t i = 0; i < descriptors1.size(); ++i) {
    const Nearest n = nearest(descriptors1[i], descriptors2);
    const bool passes = descriptors2.size() == 1 ||
                        static_cast<double>(n.best) < options.ratio * static_cast<double>(n.second);
    if (!passes || n.best > options.max_distance) {
      continue;
    }
    auto& held = claim[n.index];
    if (!held.has_value() || n.best < held->distance) {
      held = Match{.first = static_cast<std::int64_t>(i),
                   .second = static_cast<std::int64_t>(n.index),
                   .distance = n.best};
    }
  }
  std::vector<Match> out;
  for (const auto& held : claim) {
    if (held.has_value()) {
      out.push_back(*held);
    }
  }
  std::ranges::sort(out, {}, &Match::first);
  if (options.check_orientation && !out.empty()) {
    const auto rotation = [&](const Match& m) {
      const std::int64_t diff =
          angles1[static_cast<std::size_t>(m.first)] - angles2[static_cast<std::size_t>(m.second)];
      return ((diff % orb::kAngleBins) + orb::kAngleBins) % orb::kAngleBins;
    };
    std::array<std::int64_t, orb::kAngleBins> counts{};
    for (const Match& m : out) {
      ++counts.at(static_cast<std::size_t>(rotation(m)));
    }
    const std::vector<std::int64_t> kept = three_maxima(counts);
    std::erase_if(out, [&](const Match& m) { return !std::ranges::contains(kept, rotation(m)); });
  }
  return out;
}

}  // namespace slamlab::foundations::matching
