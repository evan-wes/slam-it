/// @file
/// @brief The keyframe database: the C++ core's mirror of `slam_it.edge.database` (ORB-SLAM's
///        `KeyFrameDatabase`), whose docstring states its rules: each keyframe's word vector and
///        its features' nodes, the inverted index from each word to its keyframes, and
///        relocalisation's candidates.
#pragma once

#include <cstdint>
#include <map>
#include <span>
#include <unordered_map>
#include <vector>

#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace slam_it::edge {

/// @brief What the vocabulary made of one keyframe.
struct Words {
  /// @brief Its word vector.
  foundations::vocabulary::BowVector vector;
  /// @brief Each of its features' node at the direct index's level, in the map's keypoint order.
  std::vector<std::int64_t> nodes;
};

/// @brief The words of a map's keyframes, and the keyframes that hold each word
///        (`slam_it.edge.database.Database`).
struct Database {
  /// @brief Each keyframe's words, by its id.
  std::map<std::int64_t, Words> keyframes;
  /// @brief Each word's keyframes, by id ascending.
  std::unordered_map<std::int64_t, std::vector<std::int64_t>> inverted;
};

/// @brief Relocalisation's candidates for a frame (`slam_it.edge.database.Candidates`).
struct Candidates {
  /// @brief The candidates' ids, by their group's accumulated score, descending (ties: the lower
  ///        id).
  std::vector<std::int64_t> keyframes;
  /// @brief Each one's group's accumulated score.
  std::vector<double> scores;
  /// @brief The keyframes that share a word with the frame.
  std::int64_t sharing{0};
  /// @brief Those that shared enough to be scored.
  std::int64_t scored{0};
};

/// @brief Put a keyframe's words in the database and index them.
/// @param db The database.
/// @param keyframe Its id.
/// @param words Its word vector and nodes.
/// @return Nothing; `InvalidInput` if it is already in the database.
[[nodiscard]] foundations::Result<void> insert(Database& db, std::int64_t keyframe, Words words);

/// @brief Add a keyframe: quantise its features, and index its words
///        (`slam_it.edge.database.Database.add`).
/// @param db The database.
/// @param vocabulary The vocabulary.
/// @param keyframe Its id.
/// @param descriptors Its features' descriptors, in the map's keypoint order.
/// @return Nothing; `InvalidInput` if it is already in the database.
[[nodiscard]] foundations::Result<void> add(
    Database& db, const foundations::vocabulary::Vocabulary& vocabulary, std::int64_t keyframe,
    std::span<const foundations::orb::Descriptor> descriptors);

/// @brief Remove a keyframe (culled): it leaves the index
///        (`slam_it.edge.database.Database.erase`).
/// @param db The database.
/// @param keyframe Its id.
/// @return Nothing; `InvalidInput` if it is not in the database.
[[nodiscard]] foundations::Result<void> erase(Database& db, std::int64_t keyframe);

/// @brief Each keyframe's best covisible keyframes, from the map's observations
///        (`slam_it.edge.database.best_covisible`).
/// @param m The map.
/// @param keyframes The keyframes' ids, each in the map.
/// @param options `covisible_points` and `reloc_neighbours`.
/// @return Each keyframe's id to the ids of those observing at least `covisible_points` of its
///         points, the most shared first (ties: the lower id), at most `reloc_neighbours`.
[[nodiscard]] std::map<std::int64_t, std::vector<std::int64_t>> best_covisible(
    const Map& m, std::span<const std::int64_t> keyframes, const Options& options);

/// @brief Find the keyframes a frame may be relocalised against
///        (`slam_it.edge.database.candidates`, ORB-SLAM's `DetectRelocalizationCandidates`).
/// @param db The map's keyframe database.
/// @param m The map, whose observations give covisibility.
/// @param frame The frame's word vector.
/// @param options The `reloc_*` settings and `covisible_points`.
/// @return The candidates, best first, with their scores (none if no keyframe shares a word);
///         `InvalidInput` for a malformed vector.
[[nodiscard]] foundations::Result<Candidates> candidates(
    const Database& db, const Map& m, const foundations::vocabulary::BowVector& frame,
    const Options& options);

}  // namespace slam_it::edge
