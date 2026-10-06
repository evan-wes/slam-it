/// @file
/// @brief Local mapping, growing and cleaning the map around each new keyframe: the C++ core's
///        mirror of `slamlab.edge.mapping` (ORB-SLAM's `LocalMapping`, lite), whose docstring
///        states the steps.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "slamlab/edge/frames.hpp"
#include "slamlab/edge/map.hpp"
#include "slamlab/edge/options.hpp"
#include "slamlab/foundations/error.hpp"

namespace slamlab::edge {

/// @brief Keyframes a new keyframe triangulates against, by mode (`CreateNewMapPoints`' `nn`).
inline constexpr std::array<std::int64_t, 3> kTriangulationNeighbours = {20, 10, 10};

/// @brief A match for triangulation must lie within this chi-square (1 dof, 95%) of its
///        epipolar line (`CheckDistEpipolarLine`).
inline constexpr double kEpipolarChi2 = 3.84;

/// @brief ...and, without depth, at least this squared distance from the epipole, in pixels at
///        level 0.
inline constexpr double kEpipoleDistance2 = 100.0;

/// @brief A monocular keyframe triangulates only against one whose baseline is at least this
///        fraction of its median depth.
inline constexpr double kMinBaselineDepth = 0.01;

/// @brief The keyframe ratio while the map has a single keyframe (`NeedNewKeyFrame`).
inline constexpr double kFirstRefRatio = 0.4;

/// @brief A point is culled on evidence while created at most this many keyframes ago.
inline constexpr std::int64_t kRecentKeyframes = 3;

/// @brief The depth below which a point is close (`slamlab.edge.mapping.close_depth`).
/// @param camera The camera (its baseline).
/// @param options The factor.
/// @return `close_depth` baselines, in metres.
[[nodiscard]] double close_depth(const Camera& camera, const Options& options);

/// @brief Decide whether a tracked frame becomes a keyframe
///        (`slamlab.edge.mapping.need_keyframe`, ORB-SLAM's `NeedNewKeyFrame`).
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
///        (`slamlab.edge.mapping.create_points_from_depth`, ORB-SLAM's `CreateNewKeyFrame`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param options The counts and close depth.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> create_points_from_depth(Map& m, const Camera& camera,
                                                                 const Options& options);

/// @brief Cull recently created points tracking rarely finds
///        (`slamlab.edge.mapping.cull_points`, ORB-SLAM's `MapPointCulling`).
/// @param m The map, changed in place.
/// @param mode The pipeline's mode.
/// @param options The ratio and minimums.
/// @return Nothing; `InvalidInput` for a map without keyframes, or invalid settings.
[[nodiscard]] foundations::Result<void> cull_points(Map& m, Mode mode, const Options& options);

/// @brief Match two keyframes' free keypoints for triangulation
///        (`slamlab.edge.mapping.match_for_triangulation`, ORB-SLAM's `SearchForTriangulation`,
///        lite).
/// @param m The map.
/// @param camera The camera.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @return Keypoint rows `(in k1, in k2)`, by `k1` keypoint; `InvalidInput` for a row out of
///         range, the same row twice, or an invalid camera.
[[nodiscard]] foundations::Result<std::vector<std::array<std::int64_t, 2>>> match_for_triangulation(
    const Map& m, const Camera& camera, std::int64_t k1, std::int64_t k2);

/// @brief Triangulate the newest keyframe's free keypoints against the window's
///        (`slamlab.edge.mapping.triangulate_new_points`, ORB-SLAM's `CreateNewMapPoints`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The thresholds.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> triangulate_new_points(Map& m, const Camera& camera,
                                                               Mode mode, const Options& options);

/// @brief Fuse the newest keyframe's points with the window's
///        (`slamlab.edge.mapping.fuse_points`, ORB-SLAM's `SearchInNeighbors`, lite).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param options The window and thresholds.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> fuse_points(Map& m, const Camera& camera,
                                                    const Options& options);

/// @brief Cull the window's redundant keyframes (`slamlab.edge.mapping.cull_keyframes`,
///        ORB-SLAM's `KeyFrameCulling`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The fraction and count.
/// @return Nothing; `InvalidInput` for an invalid camera or settings.
[[nodiscard]] foundations::Result<void> cull_keyframes(Map& m, const Camera& camera, Mode mode,
                                                       const Options& options);

/// @brief Insert a keyframe and run local mapping on it (`slamlab.edge.mapping.process`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param frame The new keyframe's frame.
/// @param tracked Its pose and matches.
/// @param mode The pipeline's mode.
/// @param options The settings.
/// @return Nothing; `InvalidInput` for matches that are not one per keypoint, or an invalid
///         camera or settings.
[[nodiscard]] foundations::Result<void> process(Map& m, const Camera& camera, const Frame& frame,
                                                const Tracked& tracked, Mode mode,
                                                const Options& options);

}  // namespace slamlab::edge
