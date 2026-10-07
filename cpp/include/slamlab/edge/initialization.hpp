/// @file
/// @brief Initialisation, starting the map: the C++ core's mirror of
///        `slamlab.edge.initialization` (ORB-SLAM's `StereoInitialization` and
///        `MonocularInitialization`).
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "slamlab/edge/frames.hpp"
#include "slamlab/edge/map.hpp"
#include "slamlab/edge/options.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/matching.hpp"

namespace slamlab::edge {

/// @brief The monocular map's first bundle adjustment's iterations (ORB-SLAM's
///        `GlobalBundleAdjustemnt(map, 20)`).
inline constexpr std::int64_t kInitBaIterations = 20;

/// @brief A started map and the frame it was started on, tracked.
struct Started {
  /// @brief The map.
  Map map;
  /// @brief The frame (the map's last keyframe), tracked.
  Tracked tracked;
};

/// @brief Start the map from a stereo or RGB-D frame
///        (`slamlab.edge.initialization.initialize_from_depth`).
/// @param frame The frame.
/// @param options The minimum keypoints, and the pyramid.
/// @return The map of one keyframe at the origin with a point at every keypoint with depth
///         (and a bearing), or nothing if it has fewer than `min_init_features_depth` keypoints;
///         `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
[[nodiscard]] foundations::Result<std::optional<Started>> initialize_from_depth(
    const Frame& frame, const Options& options);

/// @brief Match a monocular reference frame to a later one
///        (`slamlab.edge.initialization.match_for_initialization`, ORB-SLAM's
///        `SearchForInitialization`).
/// @param reference The reference frame.
/// @param frame The later frame.
/// @param options The window and ratio.
/// @return `(reference keypoint, frame keypoint)` matches, by reference keypoint;
///         `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
[[nodiscard]] foundations::Result<std::vector<foundations::matching::Match>>
match_for_initialization(const Frame& reference, const Frame& frame, const Options& options);

/// @brief Start the map from two monocular frames
///        (`slamlab.edge.initialization.initialize_monocular`).
/// @param camera The camera.
/// @param reference The reference frame (the first keyframe, at the origin).
/// @param frame The later frame.
/// @param options The window, the minimum matches, the seed, and the pyramid.
/// @return The map of two keyframes and their points, scaled so the first's median depth is
///         1, or nothing if too few keypoints match, the views do not initialise, or too few
///         points survive the adjustment; `InvalidInput` for a frame whose pyramid is not the
///         settings', or an invalid camera or settings.
[[nodiscard]] foundations::Result<std::optional<Started>> initialize_monocular(
    const Camera& camera, const Frame& reference, const Frame& frame, const Options& options);

}  // namespace slamlab::edge
