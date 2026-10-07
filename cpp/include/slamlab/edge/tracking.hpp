/// @file
/// @brief Tracking, placing each frame against the map: the C++ core's mirror of
///        `slamlab.edge.tracking` (ORB-SLAM's `Tracking`, lite), whose docstring states the
///        stages.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slamlab/edge/frames.hpp"
#include "slamlab/edge/map.hpp"
#include "slamlab/edge/options.hpp"
#include "slamlab/foundations/error.hpp"

namespace slamlab::edge {

/// @brief A point seen within this cosine of its mean viewing direction is searched in a
///        narrower window (`RadiusByViewingCos`: 2.5 rather than 4).
inline constexpr double kNarrowViewCos = 0.998;

/// @brief Inliers tracking the last frame must keep after refinement.
inline constexpr std::int64_t kMinMotionInliers = 10;

/// @brief Map points in a camera's view (`slamlab.edge.tracking.InView`).
struct InView {
  /// @brief The points' rows.
  std::vector<std::int64_t> rows;
  /// @brief Their projections.
  std::vector<std::array<double, 2>> xy;
  /// @brief Their predicted right columns (NaN without a baseline).
  std::vector<double> right;
  /// @brief Their predicted levels.
  std::vector<std::int64_t> level;
  /// @brief The cosine between each viewing ray and the point's mean viewing direction.
  std::vector<double> view_cos;
};

/// @brief A frame tracked against the local map, and the counts it adds to
///        (`slamlab.edge.tracking.LocalTracking`).
struct LocalTracking {
  /// @brief The refined pose and the inlier matches.
  Tracked tracked;
  /// @brief The ids of the points that were in view.
  std::vector<std::int64_t> visible;
  /// @brief The ids of the inliers.
  std::vector<std::int64_t> found;
};

/// @brief Predict a frame's pose by the constant-velocity model
///        (`slamlab.edge.tracking.predict`), its rotation projected back onto SO(3).
/// @param last_pose The last frame's `T_cam_world`.
/// @param velocity The last motion `T_last_previous`, or nothing before there is one.
/// @return `velocity * last_pose`, its rotation the nearest one; or the last pose.
[[nodiscard]] Eigen::Matrix4d predict(const Eigen::Matrix4d& last_pose,
                                      const std::optional<Eigen::Matrix4d>& velocity);

/// @brief Refine a frame's pose against its matched points, dropping outliers
///        (`slamlab.edge.tracking.optimize_pose`, ORB-SLAM's `PoseOptimization`).
/// @param m The map (the points' positions).
/// @param camera The camera.
/// @param frame The frame.
/// @param tracked The starting `T_cam_world`, and the point id each keypoint matched (or -1).
/// @param options The rounds and chi-square thresholds.
/// @return The refined pose and the inlier matches; `InvalidInput` for matches that are not
///         one per keypoint, or an invalid camera or settings.
[[nodiscard]] foundations::Result<Tracked> optimize_pose(const Map& m, const Camera& camera,
                                                         const Frame& frame, const Tracked& tracked,
                                                         const Options& options);

/// @brief Select the points a camera sees, at a distance and angle they are matchable from
///        (`slamlab.edge.tracking.in_view`, ORB-SLAM's `isInFrustum`).
/// @param m The map.
/// @param camera The camera.
/// @param pose Its `T_cam_world`.
/// @param rows The candidate points' rows.
/// @param options The viewing-angle limit.
/// @return The points in view, in the order of `rows`; `InvalidInput` for a row out of range,
///         or an invalid camera or settings.
[[nodiscard]] foundations::Result<InView> in_view(const Map& m, const Camera& camera,
                                                  const Eigen::Matrix4d& pose,
                                                  std::span<const std::int64_t> rows,
                                                  const Options& options);

/// @brief Track the last frame's points into this one
///        (`slamlab.edge.tracking.track_last_frame`, ORB-SLAM's `TrackWithMotionModel`).
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame to track.
/// @param last The last frame.
/// @param last_tracked Its pose and matches.
/// @param predicted The predicted pose.
/// @param mode The pipeline's mode.
/// @param options The windows and minimums.
/// @return The tracked frame, or nothing if too few points matched or survived refinement;
///         `InvalidInput` for the last frame's matches not one per keypoint, or an invalid
///         camera or settings.
[[nodiscard]] foundations::Result<std::optional<Tracked>> track_last_frame(
    const Map& m, const Camera& camera, const Frame& frame, const Frame& last,
    const Tracked& last_tracked, const Eigen::Matrix4d& predicted, Mode mode,
    const Options& options);

/// @brief Match the local map's points in view and refine the pose
///        (`slamlab.edge.tracking.track_local_map`, ORB-SLAM's `TrackLocalMap`).
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param tracked Its pose and matches so far.
/// @param mode The pipeline's mode.
/// @param options The window, thresholds and minimums.
/// @param recovering Whether the frame is a recovery's or the next: the window is then
///        `recovery_local_window`.
/// @return The refined frame and the ids of the points in view and of the inliers;
///         `InvalidInput` for matches that are not one per keypoint, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<LocalTracking> track_local_map(const Map& m, const Camera& camera,
                                                                 const Frame& frame,
                                                                 const Tracked& tracked, Mode mode,
                                                                 const Options& options,
                                                                 bool recovering = false);

/// @brief Place a frame against the local map by descriptors and PnP RANSAC
///        (`slamlab.edge.tracking.relocalize_locally`, the fallback).
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param options The RANSAC threshold, minimum inliers and seed.
/// @return The frame placed and refined, or nothing if too few inliers; `InvalidInput` for an
///         invalid camera or settings.
[[nodiscard]] foundations::Result<std::optional<Tracked>> relocalize_locally(
    const Map& m, const Camera& camera, const Frame& frame, const Options& options);

}  // namespace slamlab::edge
