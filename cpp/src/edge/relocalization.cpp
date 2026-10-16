/// @file
/// @brief Implementation of the relocalisation declared in slam_it/edge/relocalization.hpp.
#include "slam_it/edge/relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

#include "slam_it/edge/tracking.hpp"
#include "slam_it/foundations/correspondences.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/p3p.hpp"

namespace slam_it::edge {

namespace {

namespace matching = foundations::matching;
namespace vocabulary = foundations::vocabulary;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
using foundations::correspondences::Rows3;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief A keyframe's row, refusing one the map does not hold.
/// @param m The map.
/// @param keyframe Its id.
/// @return The row; `InvalidInput` if the map does not hold it.
Result<std::size_t> row_of(const Map& m, std::int64_t keyframe) {
  const std::int64_t k = keyframe_row(m, keyframe);
  if (k < 0) {
    return fail(ErrorKind::kInvalidInput,
                "keyframe " + std::to_string(keyframe) + " is not in the map");
  }
  return static_cast<std::size_t>(k);
}

/// @brief How many keypoints matched a point.
/// @param points Each keypoint's point id, or -1.
/// @return The matched ones.
std::int64_t count_matched(std::span<const std::int64_t> points) {
  return std::ranges::count_if(points, [](std::int64_t p) { return p >= 0; });
}

/// @brief A search's matches added to a frame's.
/// @param points Each keypoint's point id, or -1.
/// @param added The search's, or -1.
/// @return The search's where it matched, the frame's elsewhere.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (points, added), as Python's _merged
std::vector<std::int64_t> merged(std::span<const std::int64_t> points,
                                 std::span<const std::int64_t> added) {
  std::vector<std::int64_t> out(points.begin(), points.end());
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (added[i] >= 0) {
      out[i] = added[i];
    }
  }
  return out;
}

/// @brief One candidate tried: the frame if it placed it, how far it got, and its matches.
struct Attempt {
  /// @brief The frame placed, if it was.
  std::optional<Tracked> tracked;
  /// @brief How far it got.
  CandidateSummary summary;
  /// @brief Each keypoint's match by word or by projection, or -1.
  std::vector<std::int64_t> matched;
};

/// @brief The frame as last optimised, and the matches that optimisation started from (or
///        would have).
using Searched = std::pair<Tracked, std::vector<std::int64_t>>;

/// @brief Search the candidate's other points twice, optimising when they bring enough
///        inliers (`slam_it.edge.relocalization._project_more`).
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param keyframe The candidate's id.
/// @param refined The frame after the first optimisation.
/// @param found The points RANSAC's inliers matched, which the first search skips.
/// @param options The settings.
/// @return The frame as last optimised, and every match before that optimisation.
Result<Searched> project_more(const Map& m, const Camera& camera, const Frame& frame,
                              std::int64_t keyframe, const Tracked& refined,
                              std::span<const std::int64_t> found, const Options& options) {
  const std::int64_t needed = options.tracking.recovery_tracked;
  auto added = search_keyframe(
      m, camera, frame, refined, keyframe, found,
      {.radius = options.relocalization.window, .distance = options.matching.th_high}, options);
  if (!added) {
    return std::unexpected(std::move(added.error()));
  }
  std::vector<std::int64_t> all = merged(refined.points, *added);
  if (inliers(refined) + count_matched(*added) < needed) {
    return Searched{refined, std::move(all)};
  }
  auto again = optimize_pose(m, camera, frame, {.pose = refined.pose, .points = all}, options);
  if (!again) {
    return std::unexpected(std::move(again.error()));
  }
  const std::int64_t good = inliers(*again);
  if (good <= options.relocalization.narrow_inliers || good >= needed) {
    return Searched{std::move(*again), std::move(all)};
  }
  // The second optimisation's outliers keep their matches until the last (ORB-SLAM's).
  const Tracked held{.pose = again->pose, .points = all};
  std::vector<std::int64_t> skip;
  std::ranges::copy_if(all, std::back_inserter(skip), [](std::int64_t p) { return p >= 0; });
  auto narrow = search_keyframe(m, camera, frame, held, keyframe, skip,
                                {.radius = options.relocalization.narrow_window,
                                 .distance = options.relocalization.narrow_distance},
                                options);
  if (!narrow) {
    return std::unexpected(std::move(narrow.error()));
  }
  if (good + count_matched(*narrow) < needed) {
    return Searched{std::move(*again), std::move(all)};
  }
  std::vector<std::int64_t> last = merged(all, *narrow);
  auto final_pose = optimize_pose(m, camera, frame, {.pose = held.pose, .points = last}, options);
  if (!final_pose) {
    return std::unexpected(std::move(final_pose.error()));
  }
  return Searched{std::move(*final_pose), std::move(last)};
}

/// @brief A candidate's keypoints that observe a point, and each one's node.
struct Observing {
  /// @brief The keypoints' rows.
  std::vector<std::int64_t> rows;
  /// @brief Each one's node at the direct index's level.
  std::vector<std::int64_t> nodes;
};

/// @brief A candidate's matches by word, with a bearing in front.
/// @param m The map.
/// @param frame The frame.
/// @param candidate The candidate's keypoints that observe a point, and their nodes.
/// @param frame_nodes Each of the frame's keypoints' node.
/// @param options `th_low` and `relocalize_ratio`.
/// @return `(observing keypoint, frame keypoint)` matches; `InvalidInput` from matching.
Result<std::vector<matching::Match>> match_by_word(const Map& m, const Frame& frame,
                                                   const Observing& candidate,
                                                   std::span<const std::int64_t> frame_nodes,
                                                   const Options& options) {
  std::vector<foundations::orb::Descriptor> descriptors;
  std::vector<std::int64_t> angles;
  descriptors.reserve(candidate.rows.size());
  angles.reserve(candidate.rows.size());
  for (const std::int64_t row : candidate.rows) {
    descriptors.push_back(m.kp_descriptor[static_cast<std::size_t>(row)]);
    angles.push_back(m.kp_angle[static_cast<std::size_t>(row)]);
  }
  std::vector<std::int64_t> frame_angles;
  frame_angles.reserve(keypoint_count(frame));
  for (const foundations::orb::Keypoint& k : frame.features.keypoints) {
    frame_angles.push_back(k.angle);
  }
  auto found = matching::search_by_bow(descriptors, candidate.nodes, frame.features.descriptors,
                                       frame_nodes, angles, frame_angles,
                                       {.max_distance = options.matching.th_low,
                                        .ratio = options.relocalization.ratio,
                                        .check_orientation = true});
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  std::erase_if(*found, [&frame](const matching::Match& match) {
    return !(frame.bearings[static_cast<std::size_t>(match.second)].z() > 0.0);
  });
  return found;
}

/// @brief Try to place the frame through one candidate
///        (`slam_it.edge.relocalization._try`, the module docstring's steps 2 to 4).
/// @param m The map.
/// @param db The database.
/// @param camera The camera.
/// @param frame The frame.
/// @param frame_nodes Each of the frame's keypoints' node.
/// @param candidate The candidate's id and score.
/// @param options The settings.
/// @return The attempt; `InvalidInput` for a candidate the map does not hold or whose features
///         are not the database's.
Result<Attempt> try_candidate(const Map& m, const Database& db, const Camera& camera,
                              const Frame& frame, std::span<const std::int64_t> frame_nodes,
                              std::pair<std::int64_t, double> candidate, const Options& options) {
  const auto [keyframe, score] = candidate;
  auto k = row_of(m, keyframe);
  if (!k) {
    return std::unexpected(std::move(k.error()));
  }
  const auto start = m.kf_start[*k];
  const auto end = m.kf_start[*k + 1];
  const std::vector<std::int64_t>& nodes = db.keyframes.at(keyframe).nodes;
  if (std::cmp_not_equal(nodes.size(), end - start)) {
    return fail(ErrorKind::kInvalidInput,
                "database: keyframe " + std::to_string(keyframe) + "'s features are not the map's");
  }
  Observing observing;
  for (auto row = start; row < end; ++row) {
    if (m.kp_point[static_cast<std::size_t>(row)] >= 0) {
      observing.rows.push_back(row);
      observing.nodes.push_back(nodes[static_cast<std::size_t>(row - start)]);
    }
  }
  auto pairs = match_by_word(m, frame, observing, frame_nodes, options);
  if (!pairs) {
    return std::unexpected(std::move(pairs.error()));
  }
  Attempt out{.tracked = std::nullopt,
              .summary = {.keyframe = keyframe,
                          .score = score,
                          .matches = static_cast<std::int64_t>(pairs->size()),
                          .iterations = std::nullopt,
                          .termination = std::nullopt,
                          .degenerate = false,
                          .inliers = std::nullopt,
                          .refined = std::nullopt,
                          .found = std::nullopt},
              .matched = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  for (const matching::Match& pair : *pairs) {
    out.matched[static_cast<std::size_t>(pair.second)] =
        m.kp_point[static_cast<std::size_t>(observing.rows[static_cast<std::size_t>(pair.first)])];
  }
  const auto [needed, samples] = ransac_budget(out.summary.matches, options);
  if (out.summary.matches < std::max(options.relocalization.min_matches, needed)) {
    return out;
  }
  Rows3 world(static_cast<Eigen::Index>(pairs->size()), 3);
  Rows3 rays(static_cast<Eigen::Index>(pairs->size()), 3);
  for (std::size_t j = 0; j < pairs->size(); ++j) {
    const auto keypoint = static_cast<std::size_t>((*pairs)[j].second);
    const std::int64_t p = point_row(m, out.matched[keypoint]);
    world.row(static_cast<Eigen::Index>(j)) = m.pt_xyz[static_cast<std::size_t>(p)].transpose();
    rays.row(static_cast<Eigen::Index>(j)) = frame.bearings[keypoint].transpose();
  }
  const foundations::ransac::Options settings{
      .threshold = options.tracking.pnp_threshold / focal(camera),
      .confidence = options.relocalization.confidence,
      .max_iterations = static_cast<int>(samples),
      .seed = static_cast<std::uint64_t>(options.seed)};
  auto consensus = pnp_consensus(world, rays, settings, needed, out.summary);
  if (!consensus) {
    return std::unexpected(std::move(consensus.error()));
  }
  if (!consensus->has_value()) {
    return out;
  }
  const auto& kept = **consensus;
  std::vector<std::int64_t> sample(keypoint_count(frame), -1);
  std::vector<std::int64_t> found;
  for (std::size_t j = 0; j < pairs->size(); ++j) {
    if (kept.inliers(static_cast<Eigen::Index>(j))) {
      const auto keypoint = static_cast<std::size_t>((*pairs)[j].second);
      sample[keypoint] = out.matched[keypoint];
    }
  }
  std::ranges::copy_if(sample, std::back_inserter(found), [](std::int64_t p) { return p >= 0; });
  auto refined = optimize_pose(m, camera, frame, {.pose = kept.model, .points = sample}, options);
  if (!refined) {
    return std::unexpected(std::move(refined.error()));
  }
  out.summary.refined = inliers(*refined);
  if (*out.summary.refined < options.relocalization.min_inliers) {
    return out;
  }
  Tracked last = std::move(*refined);
  if (inliers(last) < options.tracking.recovery_tracked) {
    auto more = project_more(m, camera, frame, keyframe, last, found, options);
    if (!more) {
      return std::unexpected(std::move(more.error()));
    }
    last = std::move(more->first);
    out.matched = merged(out.matched, more->second);
  }
  out.summary.found = inliers(last);
  if (*out.summary.found >= options.tracking.recovery_tracked) {
    out.tracked = std::move(last);
  }
  return out;
}

}  // namespace

std::pair<std::int64_t, std::int64_t> ransac_budget(std::int64_t matches, const Options& options) {
  const std::int64_t needed =
      std::max({options.relocalization.min_inliers,
                static_cast<std::int64_t>(
                    std::floor(options.relocalization.inlier_ratio * static_cast<double>(matches))),
                static_cast<std::int64_t>(foundations::p3p::kSampleSize)});
  if (needed >= matches) {
    return {needed, 1};
  }
  const double ratio = std::max(options.relocalization.inlier_ratio,
                                static_cast<double>(needed) / static_cast<double>(matches));
  const double bound = foundations::ransac::required_iterations(
      ratio, foundations::p3p::kSampleSize, options.relocalization.confidence);
  const auto samples = static_cast<std::int64_t>(std::ceil(bound));
  return {needed,
          std::max<std::int64_t>(1, std::min(samples, options.relocalization.ransac_iterations))};
}

Result<std::vector<std::int64_t>> search_keyframe(const Map& m, const Camera& camera,
                                                  const Frame& frame, const Tracked& tracked,
                                                  std::int64_t keyframe,
                                                  std::span<const std::int64_t> skip,
                                                  SearchWindow window, const Options& options) {
  auto k = row_of(m, keyframe);
  if (!k) {
    return std::unexpected(std::move(k.error()));
  }
  std::vector<std::int64_t> skipped(skip.begin(), skip.end());
  std::ranges::sort(skipped);
  const Eigen::Vector3d centre = camera_centre(tracked.pose);
  std::vector<std::int64_t> points;
  std::vector<double> gaps;
  std::vector<matching::Projection> projections;
  for (auto row = m.kf_start[*k]; row < m.kf_start[*k + 1]; ++row) {
    const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)];
    if (id < 0 || std::ranges::binary_search(skipped, id)) {
      continue;
    }
    const std::int64_t p = point_row(m, id);
    const Eigen::Vector3d& xyz = m.pt_xyz[static_cast<std::size_t>(p)];
    const Eigen::Vector3d at = to_camera(tracked.pose, xyz);
    if (!(at.z() > 0.0)) {
      continue;
    }
    const Eigen::Vector2d pixel = project(camera, at);
    const double gap = (xyz - centre).norm();
    if (!inside(camera, pixel) ||
        !in_distance_range(m, static_cast<std::size_t>(p), gap, options)) {
      continue;
    }
    points.push_back(p);
    gaps.push_back(gap);
    projections.push_back({.xy = {pixel.x(), pixel.y()},
                           .radius = 0.0,
                           .min_level = 0,
                           .max_level = 0,
                           .descriptor = m.pt_descriptor[static_cast<std::size_t>(p)],
                           .angle = m.kp_angle[static_cast<std::size_t>(row)],
                           .right = kNan});
  }
  auto levels = predict_level(m, points, gaps);
  if (!levels) {
    return std::unexpected(std::move(levels.error()));
  }
  for (std::size_t j = 0; j < projections.size(); ++j) {
    const std::int64_t level = (*levels)[j];
    projections[j].radius = window.radius * std::pow(m.scale_factor, static_cast<double>(level));
    projections[j].min_level = level - 1;
    projections[j].max_level = level + 1;
  }
  std::vector<std::uint8_t> taken;
  taken.reserve(tracked.points.size());
  for (const std::int64_t p : tracked.points) {
    taken.push_back(p >= 0 ? 1 : 0);
  }
  auto found =
      search(frame, projections,
             {.max_distance = window.distance, .ratio = 1.0, .check_orientation = true}, taken);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  std::vector<std::int64_t> out(keypoint_count(frame), -1);
  for (const matching::Match& match : *found) {
    out[static_cast<std::size_t>(match.second)] =
        m.pt_id[static_cast<std::size_t>(points[static_cast<std::size_t>(match.first)])];
  }
  return out;
}

Result<Relocalized> relocalize(const Map& m, const Database& db,
                               const foundations::vocabulary::Vocabulary& vocabulary,
                               const Camera& camera, const Frame& frame, const Options& options) {
  const vocabulary::Quantized quantized =
      vocabulary::quantize(vocabulary, frame.features.descriptors);
  if (auto ok = check_setup(camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  auto vector = vocabulary::bow(vocabulary, quantized.words);
  if (!vector) {
    return std::unexpected(std::move(vector.error()));
  }
  auto found = candidates(db, m, *vector, options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  Relocalized out{.tracked = std::nullopt,
                  .summary = {.words = static_cast<std::int64_t>(vector->ids.size()),
                              .sharing = found->sharing,
                              .scored = found->scored,
                              .candidates = {},
                              .placed = false},
                  .matched = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  for (std::size_t i = 0; i < found->keyframes.size(); ++i) {
    auto attempt = try_candidate(m, db, camera, frame, quantized.nodes,
                                 {found->keyframes[i], found->scores[i]}, options);
    if (!attempt) {
      return std::unexpected(std::move(attempt.error()));
    }
    out.summary.candidates.push_back(attempt->summary);
    out.matched = std::move(attempt->matched);
    if (attempt->tracked) {
      out.tracked = std::move(attempt->tracked);
      out.summary.placed = true;
      return out;
    }
  }
  return out;
}

}  // namespace slam_it::edge
