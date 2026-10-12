/// @file
/// @brief Local mapping, growing and cleaning the map around each new keyframe: the C++ core's
///        mirror of `slam_it.edge.mapping` (ORB-SLAM's `LocalMapping`, lite), whose docstring
///        states the steps. Mapping a keyframe also says how each step went, which a run
///        records per keyframe.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/edge/adjust.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"

namespace slam_it::edge {

/// @brief How local mapping went on a keyframe (`slam_it.edge.mapping.MappingSummary`):
///        the point counts are the map's before and after each step.
struct MappingSummary {
  /// @brief Points made from the keyframe's close depth (0 in monocular).
  std::int64_t from_depth{0};
  /// @brief Recent points culled.
  std::int64_t culled_points{0};
  /// @brief Points triangulated with the window's keyframes.
  std::int64_t triangulated{0};
  /// @brief Points fused into another.
  std::int64_t fused{0};
  /// @brief The local bundle adjustment's solves; none if it did not run.
  std::vector<Solve> solves;
  /// @brief The observations it dropped.
  std::int64_t outliers{0};
  /// @brief Points erased because the observations dropped, by the adjustment or with a culled
  ///        keyframe, left them too weakly observed.
  std::int64_t weak_points{0};
  /// @brief Redundant keyframes culled.
  std::int64_t culled_keyframes{0};
};

/// @brief A keyframe mapped (`slam_it.edge.mapping.Mapped`, the map aside: it changes in
///        place): how it went, and the points made of its keypoints.
struct Mapped {
  /// @brief How each step went.
  MappingSummary summary;
  /// @brief The id of the point made for each of the keyframe's keypoints (from its depth, or
  ///        triangulated), -1 where none: as made, though fusion or culling may since have
  ///        removed it.
  std::vector<std::int64_t> made;
};

/// @brief The depth below which a point is close (`slam_it.edge.mapping.close_depth`).
/// @param camera The camera (its baseline).
/// @param options The factor.
/// @return `close_depth` baselines, in metres.
[[nodiscard]] double close_depth(const Camera& camera, const Options& options);

/// @brief Decide whether a tracked frame becomes a keyframe
///        (`slam_it.edge.mapping.need_keyframe`, ORB-SLAM's `NeedNewKeyFrame`).
/// @param m The map, with at least one keyframe.
/// @param camera The camera.
/// @param depth The frame's keypoints' depths (NaN where unknown).
/// @param tracked Its pose and matches.
/// @param mode The pipeline's mode.
/// @param options The ratios and counts.
/// @return Whether to insert a keyframe; `InvalidInput` for a map without keyframes, matches
///         that are not one per depth, or an invalid camera or settings.
[[nodiscard]] foundations::Result<bool> need_keyframe(const Map& m, const Camera& camera,
                                                      std::span<const double> depth,
                                                      const Tracked& tracked, Mode mode,
                                                      const Options& options);

/// @brief Create points from the newest keyframe's close depth
///        (`slam_it.edge.mapping.create_points_from_depth`, ORB-SLAM's `CreateNewKeyFrame`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param options The counts and close depth.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> create_points_from_depth(Map& m, const Camera& camera,
                                                                 const Options& options);

/// @brief Cull recently created points tracking rarely finds
///        (`slam_it.edge.mapping.cull_points`, ORB-SLAM's `MapPointCulling`).
/// @param m The map, changed in place.
/// @param mode The pipeline's mode.
/// @param options The ratio and minimums.
/// @return Nothing; `InvalidInput` for a map without keyframes, or invalid settings.
[[nodiscard]] foundations::Result<void> cull_points(Map& m, Mode mode, const Options& options);

/// @brief Match two keyframes' free keypoints for triangulation
///        (`slam_it.edge.mapping.match_for_triangulation`, ORB-SLAM's `SearchForTriangulation`,
///        lite).
/// @param m The map.
/// @param camera The camera.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param options The epipolar band and the epipole's distance.
/// @return Keypoint rows `(in k1, in k2)`, by `k1` keypoint; `InvalidInput` for a row out of
///         range, the same row twice, or an invalid camera or settings.
[[nodiscard]] foundations::Result<std::vector<std::array<std::int64_t, 2>>> match_for_triangulation(
    const Map& m, const Camera& camera, std::int64_t k1, std::int64_t k2, const Options& options);

/// @brief Triangulate the newest keyframe's free keypoints against the window's
///        (`slam_it.edge.mapping.triangulate_new_points`, ORB-SLAM's `CreateNewMapPoints`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The thresholds.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> triangulate_new_points(Map& m, const Camera& camera,
                                                               Mode mode, const Options& options);

/// @brief Fuse the newest keyframe's points with the window's
///        (`slam_it.edge.mapping.fuse_points`, ORB-SLAM's `SearchInNeighbors`, lite).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param options The window and thresholds.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> fuse_points(Map& m, const Camera& camera,
                                                    const Options& options);

/// @brief Cull the window's redundant keyframes (`slam_it.edge.mapping.cull_keyframes`,
///        ORB-SLAM's `KeyFrameCulling`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The fraction and count.
/// @return Nothing; `InvalidInput` for an invalid camera or settings.
[[nodiscard]] foundations::Result<void> cull_keyframes(Map& m, const Camera& camera, Mode mode,
                                                       const Options& options);

/// @brief Insert a keyframe and run local mapping on it (`slam_it.edge.mapping.process`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param frame The new keyframe's frame.
/// @param tracked Its pose and matches.
/// @param mode The pipeline's mode.
/// @param options The settings.
/// @return How each step went, and the points made for the keyframe's keypoints;
///         `InvalidInput` for matches that are not one per keypoint, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<Mapped> process(Map& m, const Camera& camera, const Frame& frame,
                                                  const Tracked& tracked, Mode mode,
                                                  const Options& options);

}  // namespace slam_it::edge
