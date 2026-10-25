/// @file
/// @brief Implementation of the keyframe database declared in slam_it/edge/database.hpp.
#include "slam_it/edge/database.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <ranges>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "slam_it/edge/covisibility.hpp"

namespace slam_it::edge {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
namespace vocabulary = foundations::vocabulary;

/// @brief Each keyframe's words shared with a frame, found through the index.
/// @param db The database.
/// @param frame The frame's word vector.
/// @param excluded The keyframes left out, by id.
/// @return Each keyframe's id: how many of the frame's words its vector holds.
std::map<std::int64_t, std::int64_t> shared_words(const Database& db,
                                                  const vocabulary::BowVector& frame,
                                                  std::span<const std::int64_t> excluded) {
  std::map<std::int64_t, std::int64_t> out;
  for (const std::int64_t word : frame.ids) {
    const auto holders = db.inverted.find(word);
    if (holders != db.inverted.end()) {
      for (const std::int64_t keyframe : holders->second) {
        if (!std::ranges::contains(excluded, keyframe)) {
          ++out[keyframe];
        }
      }
    }
  }
  return out;
}

/// @brief Each scored keyframe's group: its score accumulated over its scored neighbours, and
///        the best-scored of them.
/// @param scores Each scored keyframe's score, by id.
/// @param neighbours Each one's best covisible keyframes.
/// @return `(accumulated score, best keyframe)`, by the keyframes' ids.
std::vector<std::pair<double, std::int64_t>> groups_of(
    const std::map<std::int64_t, double>& scores,
    const std::map<std::int64_t, std::vector<std::int64_t>>& neighbours) {
  std::vector<std::pair<double, std::int64_t>> out;
  for (const auto& [keyframe, own] : scores) {
    double total = own;
    std::int64_t best = keyframe;
    double best_score = own;
    for (const std::int64_t other : neighbours.at(keyframe)) {
      const auto theirs = scores.find(other);
      if (theirs != scores.end()) {
        total += theirs->second;
        if (theirs->second > best_score) {
          best = other;
          best_score = theirs->second;
        }
      }
    }
    out.emplace_back(total, best);
  }
  return out;
}

/// @brief Each scored keyframe's best `count` links' ids (`slam_it.edge.database._best`).
/// @param m The map.
/// @param keyframes The keyframes' ids, each in the map.
/// @param count The links kept.
/// @param options The settings: `covisibility.min_points`.
/// @return Each keyframe's id to its best links' ids, most shared first.
std::map<std::int64_t, std::vector<std::int64_t>> best_links(
    const Map& m, std::span<const std::int64_t> keyframes, std::int64_t count,
    const Options& options) {
  std::map<std::int64_t, std::vector<std::int64_t>> out;
  for (const std::int64_t keyframe : keyframes) {
    std::vector<std::int64_t>& best = out[keyframe];
    const std::int64_t row = keyframe_row(m, keyframe);
    if (row < 0) {
      continue;  // a keyframe the map does not hold has none, as in Python
    }
    const auto k = static_cast<std::size_t>(row);
    for (const Link& link : links_of(m, shared_points(m, k), options.covisibility.min_points)) {
      if (std::cmp_greater_equal(best.size(), count)) {
        break;
      }
      best.push_back(m.kf_id[static_cast<std::size_t>(link.row)]);
    }
  }
  return out;
}

/// @brief A query's scoring: its groups, best first, and how many keyframes were weighed.
struct Scoring {
  /// @brief Each group's accumulated score and best keyframe, by score descending (ties: the
  ///        lower id).
  std::vector<std::pair<double, std::int64_t>> groups;
  /// @brief The keyframes sharing a word, but the excluded.
  std::int64_t sharing{0};
  /// @brief Those scored.
  std::int64_t scored{0};
};

/// @brief Score the keyframes sharing a frame's words and accumulate them by group
///        (`slam_it.edge.database._scoring`).
/// @param db The database.
/// @param m The map.
/// @param frame The frame's word vector, checked.
/// @param excluded The keyframes left out, by id.
/// @param rules `(common_words, neighbours)`.
/// @param options The settings: `covisibility.min_points`.
/// @return The scoring; a score's error.
Result<Scoring> scoring(const Database& db, const Map& m, const vocabulary::BowVector& frame,
                        std::span<const std::int64_t> excluded,
                        std::pair<double, std::int64_t> rules, const Options& options) {
  const auto [common_words, neighbours] = rules;
  const std::map<std::int64_t, std::int64_t> shared = shared_words(db, frame, excluded);
  Scoring out;
  out.sharing = static_cast<std::int64_t>(shared.size());
  if (shared.empty()) {
    return out;
  }
  const std::int64_t most = std::ranges::max(shared | std::views::values);
  const auto least =
      static_cast<std::int64_t>(std::floor(common_words * static_cast<double>(most)));
  std::map<std::int64_t, double> scores;
  std::vector<std::int64_t> scored;
  for (const auto& [keyframe, count] : shared) {
    if (count > least) {
      auto score = vocabulary::score(frame, db.keyframes.at(keyframe).vector);
      if (!score) {
        return std::unexpected(std::move(score.error()));
      }
      scores.emplace(keyframe, *score);
      scored.push_back(keyframe);
    }
  }
  out.scored = static_cast<std::int64_t>(scores.size());
  out.groups = groups_of(scores, best_links(m, scored, neighbours, options));
  std::ranges::sort(out.groups, [](const auto& a, const auto& b) {
    return a.first > b.first || (a.first == b.first && a.second < b.second);
  });
  return out;
}

/// @brief The groups' best keyframes, each once, scoring above `above`, at most `most`.
/// @param found The scoring.
/// @param above The score a group must exceed.
/// @param most The candidates kept.
/// @return The candidates, with the scoring's counts.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (above, most), a score and a count
Candidates chosen(const Scoring& found, double above, std::size_t most) {
  Candidates out{.keyframes = {}, .scores = {}, .sharing = found.sharing, .scored = found.scored};
  for (const auto& [total, best] : found.groups) {
    if (out.keyframes.size() == most) {
      break;
    }
    if (total > above && !std::ranges::contains(out.keyframes, best)) {
      out.keyframes.push_back(best);
      out.scores.push_back(total);
    }
  }
  return out;
}

}  // namespace

Result<void> insert(Database& db, std::int64_t keyframe, Words words) {
  if (db.keyframes.contains(keyframe)) {
    return fail(ErrorKind::kInvalidInput,
                "database: keyframe " + std::to_string(keyframe) + " is already in it");
  }
  for (const std::int64_t word : words.vector.ids) {
    std::vector<std::int64_t>& holders = db.inverted[word];
    holders.insert(std::ranges::lower_bound(holders, keyframe), keyframe);
  }
  db.keyframes.emplace(keyframe, std::move(words));
  return {};
}

Result<void> add(Database& db, const foundations::vocabulary::Vocabulary& vocabulary,
                 std::int64_t keyframe, std::span<const foundations::orb::Descriptor> descriptors) {
  vocabulary::Quantized quantized = vocabulary::quantize(vocabulary, descriptors);
  auto vector = vocabulary::bow(vocabulary, quantized.words);
  if (!vector) {
    return std::unexpected(std::move(vector.error()));
  }
  return insert(db, keyframe, {.vector = std::move(*vector), .nodes = std::move(quantized.nodes)});
}

Result<void> erase(Database& db, std::int64_t keyframe) {
  const auto found = db.keyframes.find(keyframe);
  if (found == db.keyframes.end()) {
    return fail(ErrorKind::kInvalidInput,
                "database: keyframe " + std::to_string(keyframe) + " is not in it");
  }
  for (const std::int64_t word : found->second.vector.ids) {
    const auto holders = db.inverted.find(word);
    std::erase(holders->second, keyframe);
    if (holders->second.empty()) {
      db.inverted.erase(holders);
    }
  }
  db.keyframes.erase(found);
  return {};
}

Result<Candidates> candidates(const Database& db, const Map& m,
                              const foundations::vocabulary::BowVector& frame,
                              const Options& options) {
  if (auto ok = vocabulary::check_vector(frame); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const RelocalizationOptions& settings = options.relocalization;
  auto found = scoring(db, m, frame, {}, {settings.common_words, settings.neighbours}, options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  if (found->groups.empty()) {
    return Candidates{.keyframes = {}, .scores = {}, .sharing = 0, .scored = 0};
  }
  const double retained = settings.group_score * found->groups.front().first;
  return chosen(*found, retained, found->groups.size());
}

Result<Candidates> loop_candidates(const Database& db, const Map& m, std::int64_t keyframe,
                                   const Options& options) {
  const std::int64_t row = keyframe_row(m, keyframe);
  if (row < 0) {
    return fail(ErrorKind::kInvalidInput,
                "keyframe " + std::to_string(keyframe) + " is not in the map");
  }
  const auto words = db.keyframes.find(keyframe);
  if (words == db.keyframes.end()) {
    return fail(ErrorKind::kInvalidInput,
                "database: keyframe " + std::to_string(keyframe) + " is not in it");
  }
  auto sharing = connected(m, row);
  if (!sharing) {
    return std::unexpected(std::move(sharing.error()));
  }
  std::vector<std::int64_t> excluded{keyframe};
  for (const Link& link : *sharing) {
    excluded.push_back(m.kf_id[static_cast<std::size_t>(link.row)]);
  }
  const LoopOptions& settings = options.loops;
  auto found = scoring(db, m, words->second.vector, excluded,
                       {settings.common_words, settings.neighbours}, options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  return chosen(*found, -std::numeric_limits<double>::infinity(),
                static_cast<std::size_t>(settings.candidates));
}

std::vector<MapRef> refs_of(std::span<const Indexed> maps) {
  std::vector<MapRef> out;
  out.reserve(maps.size());
  for (const Indexed& map : maps) {
    out.push_back({.number = map.number, .map = &map.map, .database = &map.database});
  }
  return out;
}

Result<MergeCandidates> merge_candidates(const foundations::vocabulary::BowVector& vector,
                                         std::span<const MapRef> others, const Options& options) {
  if (auto ok = vocabulary::check_vector(vector); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_numbers(others, "others"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const LoopOptions& settings = options.loops;
  std::vector<Ranked> found;
  MergeCandidates out;
  for (const MapRef& other : others) {
    auto scored = scoring(*other.database, *other.map, vector, {},
                          {settings.common_words, settings.neighbours}, options);
    if (!scored) {
      return std::unexpected(std::move(scored.error()));
    }
    out.sharing += scored->sharing;
    out.scored += scored->scored;
    const Candidates best = chosen(*scored, -std::numeric_limits<double>::infinity(),
                                   static_cast<std::size_t>(settings.candidates));
    for (std::size_t i = 0; i < best.keyframes.size(); ++i) {
      found.push_back(
          {.score = best.scores[i], .map = other.number, .keyframe = best.keyframes[i]});
    }
  }
  rank(found);
  found.resize(std::min(found.size(), static_cast<std::size_t>(settings.candidates)));
  for (const Ranked& candidate : found) {
    out.maps.push_back(candidate.map);
    out.keyframes.push_back(candidate.keyframe);
    out.scores.push_back(candidate.score);
  }
  return out;
}

void rank(std::vector<Ranked>& found) {
  std::ranges::sort(found, [](const Ranked& a, const Ranked& b) {
    return a.score > b.score ||
           (a.score == b.score && std::pair{a.map, a.keyframe} < std::pair{b.map, b.keyframe});
  });
}

}  // namespace slam_it::edge
