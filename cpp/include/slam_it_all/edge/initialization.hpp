/// @file
/// @brief Initialisation, starting the map: the C++ core's mirror of
///        `slam_it_all.edge.initialization` (ORB-SLAM's `StereoInitialization` and
///        `MonocularInitialization`).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slam_it_all/edge/frames.hpp"
#include "slam_it_all/edge/map.hpp"
#include "slam_it_all/edge/options.hpp"
#include "slam_it_all/foundations/error.hpp"
#include "slam_it_all/foundations/matching.hpp"

namespace slam_it_all::edge {

/// @brief A started map and the frame it was started on, tracked.
struct Started {
  /// @brief The map.
  Map map;
  /// @brief The frame (the map's last keyframe), tracked.
  Tracked tracked;
};

/// @brief Start the map from a stereo or RGB-D frame
///        (`slam_it_all.edge.initialization.initialize_from_depth`).
/// @param frame The frame.
/// @param options The minimum keypoints, and the pyramid.
/// @return The map of one keyframe at the origin with a point at every keypoint with depth
///         (and a bearing), or nothing unless it has more than `min_init_features_depth`
///         keypoints and `min_init_depth_points` with depth;
///         `InvalidInput` for a frame whose pyramid is not the settings', or invalid settings.
[[nodiscard]] foundations::Result<std::optional<Started>> initialize_from_depth(
    const Frame& frame, const Options& options);

/// @brief Match a monocular reference frame to a later one
///        (`slam_it_all.edge.initialization.match_for_initialization`, ORB-SLAM's
///        `SearchForInitialization`).
/// @param reference The reference frame.
/// @param frame The later frame.
/// @param previous Where each reference keypoint was last matched, in pixels (its own position
///        until it is matched; ORB-SLAM's `vbPrevMatched`).
/// @param options The window and ratio.
/// @return `(reference keypoint, frame keypoint)` matches, by reference keypoint;
///         `InvalidInput` for a frame whose pyramid is not the settings', `previous` that is not
///         one finite pixel per reference keypoint, or invalid settings.
[[nodiscard]] foundations::Result<std::vector<foundations::matching::Match>>
match_for_initialization(const Frame& reference, const Frame& frame,
                         std::span<const std::array<double, 2>> previous, const Options& options);

/// @brief Start the map from two monocular frames
///        (`slam_it_all.edge.initialization.initialize_monocular`).
/// @param camera The camera.
/// @param reference The reference frame (the first keyframe, at the origin).
/// @param frame The later frame.
/// @param pairs `(reference keypoint, frame keypoint)` matches.
/// @param options The minimum matches, the parallax, the seed, and the pyramid.
/// @return The map of two keyframes and their points (the chosen motion's inliers in front of
///         both views with rays whose cosine is below `init_parallax_cos`, `CheckRT`), scaled
///         so the first's median depth is 1, or nothing if too few keypoints match, the views
///         do not initialise, or too few points survive the adjustment; `InvalidInput` for a
///         frame whose pyramid is not the settings', a pair that is not a keypoint of each
///         frame, or an invalid camera or settings.
[[nodiscard]] foundations::Result<std::optional<Started>> initialize_monocular(
    const Camera& camera, const Frame& reference, const Frame& frame,
    std::span<const std::array<std::int64_t, 2>> pairs, const Options& options);

}  // namespace slam_it_all::edge
