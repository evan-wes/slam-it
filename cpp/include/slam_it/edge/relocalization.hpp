/// @file
/// @brief Relocalisation: the C++ core's mirror of `slam_it.edge.relocalization` (ORB-SLAM's
///        `Tracking::Relocalization`), whose docstring states the steps: the database's
///        candidates, matching by word, PnP RANSAC as ORB-SLAM sets MLPnP's, the pose optimised
///        and topped up by searches by projection, 50 inliers to place the frame.
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "slam_it/edge/database.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/ransac.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace slam_it::edge {

/// @brief How far one candidate got (`slam_it.edge.relocalization.CandidateSummary`); each
///        count empty where it stopped first.
struct CandidateSummary {
  /// @brief The candidate's id in the map.
  std::int64_t keyframe{0};
  /// @brief Its group's accumulated score.
  double score{0.0};
  /// @brief The frame's keypoints matched to its points by word, with a bearing in front.
  std::int64_t matches{0};
  /// @brief RANSAC's samples, if it ran and found a pose.
  std::optional<std::int64_t> iterations;
  /// @brief Why RANSAC stopped, if it found a pose.
  std::optional<foundations::ransac::Termination> termination;
  /// @brief Whether RANSAC ran and found no pose.
  bool degenerate{false};
  /// @brief RANSAC's inliers.
  std::optional<std::int64_t> inliers;
  /// @brief The inliers the first optimisation kept.
  std::optional<std::int64_t> refined;
  /// @brief The inliers after the searches by projection: the last optimisation's.
  std::optional<std::int64_t> found;
};

/// @brief How relocalising a frame went (`slam_it.edge.relocalization.RelocalizationSummary`).
struct RelocalizationSummary {
  /// @brief The frame's words with a weight.
  std::int64_t words{0};
  /// @brief The keyframes that shared a word with it.
  std::int64_t sharing{0};
  /// @brief Those scored.
  std::int64_t scored{0};
  /// @brief The candidates tried, in order; the last placed the frame if any did.
  std::vector<CandidateSummary> candidates;
  /// @brief Whether the frame was placed.
  bool placed{false};
};

/// @brief Relocalisation's result (`slam_it.edge.relocalization.Relocalized`).
struct Relocalized {
  /// @brief The frame placed, with its inliers, if a candidate placed it.
  std::optional<Tracked> tracked;
  /// @brief How it went.
  RelocalizationSummary summary;
  /// @brief The id of the map point each keypoint matched in the last candidate tried, by word
  ///        or by its searches by projection; -1 where none.
  std::vector<std::int64_t> matched;
};

/// @brief The inliers a candidate's pose needs, and the most samples its RANSAC draws
///        (`slam_it.edge.relocalization.ransac_budget`, MLPnP's `SetRansacParameters`).
/// @param matches The candidate's matches by word.
/// @param options `relocalization.min_inliers`, `relocalization.inlier_ratio`,
///        `relocalization.confidence` and `relocalization.ransac_iterations`.
/// @return `(needed, samples)`.
[[nodiscard]] std::pair<std::int64_t, std::int64_t> ransac_budget(std::int64_t matches,
                                                                  const Options& options);

/// @brief The settings of one search by projection.
struct SearchWindow {
  /// @brief The window's half-width, in pixels at level 0.
  double radius{0.0};
  /// @brief The largest Hamming distance a match may have.
  std::int64_t distance{0};
};

/// @brief Search a keyframe's points in a frame by projection
///        (`slam_it.edge.relocalization.search_keyframe`).
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param tracked Its pose, and the keypoints already matched, never candidates.
/// @param keyframe The keyframe's id.
/// @param skip The ids of points not searched.
/// @param window The window and the distance.
/// @param options The distance margins.
/// @return The id of the point each keypoint matched by this search, -1 where none;
///         `InvalidInput` for a keyframe the map does not hold.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> search_keyframe(
    const Map& m, const Camera& camera, const Frame& frame, const Tracked& tracked,
    std::int64_t keyframe, std::span<const std::int64_t> skip, SearchWindow window,
    const Options& options);

/// @brief Place a frame in the map through the keyframes that look like it
///        (`slam_it.edge.relocalization.relocalize`).
/// @param m The map.
/// @param db Its keyframe database, holding every keyframe of the map.
/// @param vocabulary The vocabulary the database's words are of.
/// @param camera The camera.
/// @param frame The frame.
/// @param options The settings.
/// @return The frame placed with `tracking.recovery_tracked` inliers or more, or none; how the
///         search went; each keypoint's match in the last candidate tried. `InvalidInput` for a
///         candidate the map does not hold or whose features are not the database's, or an invalid
///         camera or settings.
[[nodiscard]] foundations::Result<Relocalized> relocalize(
    const Map& m, const Database& db, const foundations::vocabulary::Vocabulary& vocabulary,
    const Camera& camera, const Frame& frame, const Options& options);

}  // namespace slam_it::edge
