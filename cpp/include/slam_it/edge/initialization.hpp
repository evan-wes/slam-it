/// @file
/// @brief Initialisation, starting the map: the C++ core's mirror of
///        `slam_it.edge.initialization` (ORB-SLAM's `StereoInitialization` and
///        `MonocularInitialization`). Each attempt also says how it went, which a run records
///        for every frame before the map exists.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slam_it/edge/adjust.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace slam_it::edge {

/// @brief A started map and the frame it was started on, tracked.
struct Started {
  /// @brief The map.
  Map map;
  /// @brief The frame (the map's last keyframe), tracked.
  Tracked tracked;
};

/// @brief How a frame went before the map existed, with the codes it crosses as
///        (`slam_it.edge.initialization.Outcome`, whose members document each).
enum class Outcome : std::uint8_t {
  /// @brief It started the map.
  kInitialized = 0,
  /// @brief Too few keypoints.
  kTooFewFeatures = 1,
  /// @brief It became the monocular reference: there was none.
  kReferenceSet = 2,
  /// @brief Too few keypoints matched the reference, so it became the reference.
  kReferenceReplaced = 3,
  /// @brief Too few of the matches had bearings in both views.
  kTooFewMatches = 4,
  /// @brief Two-view found no motion.
  kDegenerate = 5,
  /// @brief Two-view: the camera only rotated.
  kPureRotation = 6,
  /// @brief Two-view: two candidate motions fit about as well.
  kAmbiguous = 7,
  /// @brief Two-view: too few points had enough parallax.
  kLowParallax = 8,
  /// @brief Too few points survived the initial bundle adjustment.
  kTooFewPoints = 9,
  /// @brief The first keyframe's median depth was not positive.
  kBadScale = 10,
  /// @brief The number of outcomes (a sentinel, not an outcome).
  kCount = 11,
};

/// @brief An outcome's name, as `slam_it.edge.initialization.Outcome` spells it.
/// @param outcome The outcome.
/// @return A static string.
[[nodiscard]] const char* name(Outcome outcome);

/// @brief How a frame went before the map existed (`slam_it.edge.initialization.InitSummary`).
struct InitSummary {
  /// @brief What happened.
  Outcome outcome{Outcome::kInitialized};
  /// @brief A monocular frame's matches to the reference, once there is one.
  std::optional<std::int64_t> matches;
  /// @brief The initial bundle adjustment's solves, if it ran.
  std::vector<Solve> solves;
};

/// @brief An attempt to start the map (`slam_it.edge.initialization.Initialization`).
struct Initialization {
  /// @brief The started map and frame, if the map started.
  std::optional<Started> started;
  /// @brief How the attempt went.
  InitSummary summary;
};

/// @brief Start the map from a stereo or RGB-D frame
///        (`slam_it.edge.initialization.initialize_from_depth`).
/// @param frame The frame.
/// @param options The minimum keypoints, and the pyramid.
/// @param vocabulary The vocabulary, checked, its features' nodes are of (`map.hpp`'s `kp_node`).
/// @return The map of one keyframe at the origin with its nodes and a point at every keypoint
///         with depth (and a bearing), or none unless it has more than
///         `initialization.min_features_depth` keypoints, and the outcome; `InvalidInput` for a
///         frame whose pyramid is not the settings', or invalid settings.
[[nodiscard]] foundations::Result<Initialization> initialize_from_depth(
    const Frame& frame, const Options& options,
    const foundations::vocabulary::Vocabulary& vocabulary);

/// @brief Match a monocular reference frame to a later one
///        (`slam_it.edge.initialization.match_for_initialization`, ORB-SLAM's
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
///        (`slam_it.edge.initialization.initialize_monocular`).
/// @param camera The camera.
/// @param reference The reference frame (the first keyframe, at the origin).
/// @param frame The later frame.
/// @param pairs `(reference keypoint, frame keypoint)` matches.
/// @param options The minimum matches, the parallax, the seed, and the pyramid.
/// @param vocabulary The vocabulary, checked, the keyframes' nodes are of (`map.hpp`'s
///        `kp_node`).
/// @return The map of two keyframes, their nodes and their points (the chosen motion's inliers in
/// front of both
///         views with rays whose cosine is below `initialization.parallax_cos`, `CheckRT`), scaled
///         so the first's median depth is 1, or none if too few keypoints match, the views do not
///         initialise, or too few points survive the adjustment; with the pairs given, the outcome
///         and the adjustment's solves; `InvalidInput` for a frame whose pyramid is not the
///         settings', a pair that is not a keypoint of each frame, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<Initialization> initialize_monocular(
    const Camera& camera, const Frame& reference, const Frame& frame,
    std::span<const std::array<std::int64_t, 2>> pairs, const Options& options,
    const foundations::vocabulary::Vocabulary& vocabulary);

}  // namespace slam_it::edge
