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

/// @brief Find the keyframes a frame may be relocalised against
///        (`slam_it.edge.database.candidates`, ORB-SLAM's `DetectRelocalizationCandidates`).
/// @param db The map's keyframe database.
/// @param m The map, whose observations give covisibility.
/// @param frame The frame's word vector.
/// @param options The `relocalization` settings and `covisibility.min_points`.
/// @return The candidates, best first, with their scores (none if no keyframe shares a word);
///         `InvalidInput` for a malformed vector.
[[nodiscard]] foundations::Result<Candidates> candidates(
    const Database& db, const Map& m, const foundations::vocabulary::BowVector& frame,
    const Options& options);

/// @brief Find the keyframes a keyframe may close a loop with
///        (`slam_it.edge.database.loop_candidates`, ORB-SLAM3's `DetectNBestCandidates`):
///        relocalisation's scoring of its words with `loops.common_words` and
///        `loops.neighbours`, the keyframe and every keyframe linked to it left out, the best
///        `loops.candidates` groups.
/// @param db The map's keyframe database, holding the keyframe.
/// @param m The map, holding the keyframe.
/// @param keyframe The keyframe's id.
/// @param options The `loops` settings and `covisibility.min_points`.
/// @return The candidates, best first, with their scores; `InvalidInput` if the keyframe is not
///         in the map or the database.
[[nodiscard]] foundations::Result<Candidates> loop_candidates(const Database& db, const Map& m,
                                                              std::int64_t keyframe,
                                                              const Options& options);

/// @brief A map with its keyframe database, and its number among a run's maps
///        (`slam_it.edge.database.Indexed`).
struct Indexed {
  /// @brief The map's number: maps are numbered in the order they started.
  std::int64_t number{0};
  /// @brief The map.
  Map map;
  /// @brief Its keyframe database, holding every keyframe of the map.
  Database database;
};

/// @brief Check that no two maps share a number.
/// @param maps The maps.
/// @param name The argument's name, for the message.
/// @return Nothing, or `InvalidInput` for a number given twice.
[[nodiscard]] foundations::Result<void> check_numbers(std::span<const Indexed> maps,
                                                      const char* name);

/// @brief A candidate of one of several maps.
struct Ranked {
  /// @brief Its group's accumulated score.
  double score{0.0};
  /// @brief Its map's number.
  std::int64_t map{0};
  /// @brief Its id in that map.
  std::int64_t keyframe{0};
};

/// @brief Order several maps' candidates best first: by score, descending; ties by the lower
///        map number, then the lower id (a merge's and relocalisation's order across maps).
/// @param found The candidates, reordered in place.
void rank(std::vector<Ranked>& found);

/// @brief A merge's candidates, from every other map, best first
///        (`slam_it.edge.database.MergeCandidates`).
struct MergeCandidates {
  /// @brief Each candidate's map number.
  std::vector<std::int64_t> maps;
  /// @brief Its id in that map.
  std::vector<std::int64_t> keyframes;
  /// @brief Its group's accumulated score, descending (ties: the lower map number, then the
  ///        lower id).
  std::vector<double> scores;
  /// @brief The keyframes, over every map, that share a word with the query.
  std::int64_t sharing{0};
  /// @brief Those scored.
  std::int64_t scored{0};
};

/// @brief Find the keyframes of other maps a keyframe may merge its map with
///        (`slam_it.edge.database.merge_candidates`, `DetectNBestCandidates`' merge list): in
///        each map, `loop_candidates`' scoring of the query's words, its words counted within
///        it, nothing left out, its best `loops.candidates` groups; then the best
///        `loops.candidates` of every map's, by accumulated score.
/// @param vector The query keyframe's word vector.
/// @param others The other maps, each with its database and number.
/// @param options The `loops` settings and `covisibility.min_points`.
/// @return The candidates, best first; `InvalidInput` for a malformed vector or two maps with
///         one number.
[[nodiscard]] foundations::Result<MergeCandidates> merge_candidates(
    const foundations::vocabulary::BowVector& vector, std::span<const Indexed> others,
    const Options& options);

}  // namespace slam_it::edge
