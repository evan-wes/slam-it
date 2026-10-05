/// @file
/// @brief Matching binary descriptors: the C++ core's mirror of `slamlab.foundations.matching`.
///
/// Hamming distances, and ORB-SLAM's filtered nearest neighbours: a distance cap, Lowe's ratio
/// test, one match per second-set descriptor, and the rotation-consistency histogram (the
/// Python module's docstring defines each filter).
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/orb.hpp"

/// @brief Descriptor matching.
namespace slamlab::foundations::matching {

/// @brief The most descriptors a set may hold.
inline constexpr std::size_t kMaxDescriptors = std::size_t{1} << 20U;

/// @brief Matching filters (`slamlab.foundations.matching.Options`).
struct Options {
  /// @brief The largest Hamming distance a match may have, from 0 to 256.
  std::int64_t max_distance{50};
  /// @brief The ratio test's bound, in (0, 1].
  double ratio{0.8};
  /// @brief Keep only matches in the rotation histogram's three highest bins.
  bool check_orientation{true};
};

/// @brief One match.
struct Match {
  /// @brief The first-set index.
  std::int64_t first{0};
  /// @brief The second-set index.
  std::int64_t second{0};
  /// @brief The Hamming distance.
  std::int64_t distance{0};
};

/// @brief Check matching filters.
/// @param options The filters.
/// @return Nothing, or `InvalidInput` naming the first one out of range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief The Hamming distance between two descriptors: the bits in which they differ.
/// @param a A descriptor.
/// @param b Another.
/// @return From 0 to 256.
[[nodiscard]] std::int64_t distance(const orb::Descriptor& a, const orb::Descriptor& b);

/// @brief Every pair's Hamming distance.
/// @param descriptors1 The first set.
/// @param descriptors2 The second set.
/// @return The distances, row-major (first set by second set); `InvalidInput` for a set of
///         more than `kMaxDescriptors`.
[[nodiscard]] Result<std::vector<std::int64_t>> distances(
    std::span<const orb::Descriptor> descriptors1, std::span<const orb::Descriptor> descriptors2);

/// @brief Match each descriptor of the first set to its nearest in the second, filtered.
/// @param descriptors1 The first set.
/// @param descriptors2 The second set.
/// @param angles1 The first set's orientation bins.
/// @param angles2 The second set's.
/// @param options The filters.
/// @return The kept matches, by first-set index; `InvalidInput` for a bad filter, a set too
///         large, or angles that are not one bin in `[0, orb::kAngleBins)` per descriptor.
[[nodiscard]] Result<std::vector<Match>> match(std::span<const orb::Descriptor> descriptors1,
                                               std::span<const orb::Descriptor> descriptors2,
                                               std::span<const std::int64_t> angles1,
                                               std::span<const std::int64_t> angles2,
                                               const Options& options);

}  // namespace slamlab::foundations::matching
