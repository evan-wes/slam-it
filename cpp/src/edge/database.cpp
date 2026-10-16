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

namespace slam_it::edge {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
namespace vocabulary = foundations::vocabulary;

/// @brief The distinct points a keyframe's keypoints observe.
/// @param m The map.
/// @param k The keyframe's row.
/// @return The points' rows, ascending.
std::vector<std::int64_t> points_of(const Map& m, std::size_t k) {
  std::vector<std::int64_t> out;
  for (auto row = m.kf_start[k]; row < m.kf_start[k + 1]; ++row) {
    const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)];
    if (id >= 0) {
      out.push_back(point_row(m, id));
    }
  }
  std::ranges::sort(out);
  const auto [first, last] = std::ranges::unique(out);
  out.erase(first, last);
  return out;
}

/// @brief The keyframes that observe a point, by id, each once.
/// @param m The map.
/// @param p The point's row.
/// @return Their ids, ascending.
std::vector<std::int64_t> observers_of(const Map& m, std::int64_t p) {
  std::vector<std::int64_t> out;
  for (const std::int64_t row : m.pt_observations[static_cast<std::size_t>(p)]) {
    out.push_back(m.kf_id[keyframe_of(m, row)]);
  }
  std::ranges::sort(out);
  const auto [first, last] = std::ranges::unique(out);
  out.erase(first, last);
  return out;
}

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

std::map<std::int64_t, std::vector<std::int64_t>> best_covisible(
    const Map& m, std::span<const std::int64_t> keyframes, const Options& options) {
  std::map<std::int64_t, std::vector<std::int64_t>> out;
  for (const std::int64_t keyframe : keyframes) {
    std::vector<std::int64_t>& best = out[keyframe];
    const std::int64_t k = keyframe_row(m, keyframe);
    if (k < 0) {
      continue;  // a keyframe the map does not hold has no neighbours, as in Python
    }
    std::map<std::int64_t, std::int64_t> shared;  // each other keyframe's id: points shared
    for (const std::int64_t p : points_of(m, static_cast<std::size_t>(k))) {
      for (const std::int64_t other : observers_of(m, p)) {
        if (other != keyframe) {
          ++shared[other];
        }
      }
    }
    std::vector<std::pair<std::int64_t, std::int64_t>> ranked;  // (shared, id)
    for (const auto& [other, count] : shared) {
      if (count >= options.covisibility.min_points) {
        ranked.emplace_back(count, other);
      }
    }
    std::ranges::sort(ranked, [](const auto& a, const auto& b) {
      return a.first > b.first || (a.first == b.first && a.second < b.second);
    });
    for (std::size_t i = 0;
         i < ranked.size() && std::cmp_less(i, options.relocalization.neighbours); ++i) {
      best.push_back(ranked[i].second);
    }
  }
  return out;
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
  auto groups = groups_of(scores, best_covisible(m, scored, options));
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
