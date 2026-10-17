/// @file
/// @brief Implementation of the keyframe database declared in slam_it/edge/database.hpp.
#include "slam_it/edge/database.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <ranges>
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
/// @return Each keyframe's id: how many of the frame's words its vector holds.
std::map<std::int64_t, std::int64_t> shared_words(const Database& db,
                                                  const vocabulary::BowVector& frame) {
  std::map<std::int64_t, std::int64_t> out;
  for (const std::int64_t word : frame.ids) {
    const auto holders = db.inverted.find(word);
    if (holders != db.inverted.end()) {
      for (const std::int64_t keyframe : holders->second) {
        ++out[keyframe];
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

/// @brief Each scored keyframe's best `relocalization.neighbours` links' ids
///        (`slam_it.edge.database._best`).
/// @param m The map.
/// @param keyframes The keyframes' ids, each in the map.
/// @param options The settings.
/// @return Each keyframe's id to its best links' ids, most shared first.
std::map<std::int64_t, std::vector<std::int64_t>> best_links(
    const Map& m, std::span<const std::int64_t> keyframes, const Options& options) {
  std::map<std::int64_t, std::vector<std::int64_t>> out;
  for (const std::int64_t keyframe : keyframes) {
    std::vector<std::int64_t>& best = out[keyframe];
    const std::int64_t row = keyframe_row(m, keyframe);
    if (row < 0) {
      continue;  // a keyframe the map does not hold has none, as in Python
    }
    const auto k = static_cast<std::size_t>(row);
    for (const Link& link : links_of(m, shared_points(m, k), options.covisibility.min_points)) {
      if (std::cmp_greater_equal(best.size(), options.relocalization.neighbours)) {
        break;
      }
      best.push_back(m.kf_id[static_cast<std::size_t>(link.row)]);
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
  const std::map<std::int64_t, std::int64_t> shared = shared_words(db, frame);
  Candidates out;
  out.sharing = static_cast<std::int64_t>(shared.size());
  if (shared.empty()) {
    return out;
  }
  const std::int64_t most = std::ranges::max(shared | std::views::values);
  const auto least = static_cast<std::int64_t>(
      std::floor(options.relocalization.common_words * static_cast<double>(most)));
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
  auto groups = groups_of(scores, best_links(m, scored, options));
  const double retained = options.relocalization.group_score * std::ranges::max(groups).first;
  std::ranges::sort(groups, [](const auto& a, const auto& b) {
    return a.first > b.first || (a.first == b.first && a.second < b.second);
  });
  for (const auto& [total, best] : groups) {
    if (total > retained && !std::ranges::contains(out.keyframes, best)) {
      out.keyframes.push_back(best);
      out.scores.push_back(total);
    }
  }
  return out;
}

}  // namespace slam_it::edge
