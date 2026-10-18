/// @file
/// @brief Local mapping, growing and cleaning the map around each new keyframe: the C++ core's
///        mirror of `slam_it.edge.mapping` (ORB-SLAM's `LocalMapping`, lite), whose docstring
///        states the steps. Mapping a keyframe also says how each step went, which a run
///        records per keyframe.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
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
  /// @brief Points triangulated with the keyframe's best links.
  std::int64_t triangulated{0};
  /// @brief Points fused into another.
  std::int64_t fused{0};
  /// @brief The local bundle adjustment's solves; none if it did not run.
  std::vector<Solve> solves;
  /// @brief The keyframes it adjusted (0 if it did not run).
  std::int64_t free{0};
  /// @brief The keyframes it held fixed.
  std::int64_t fixed{0};
  /// @brief The observations it dropped.
  std::int64_t outliers{0};
  /// @brief Points erased because the observations dropped, by the adjustment or with a culled
  ///        keyframe, left them too weakly observed.
  std::int64_t weak_points{0};
  /// @brief Redundant keyframes culled.
  std::int64_t culled_keyframes{0};
};

/// @brief How a step local mapping schedules went (`slam_it.edge.mapping.Scheduled`), with the
///        codes it crosses the bindings as.
enum class Scheduled : std::uint8_t {
  /// @brief It ran (and may have found nothing to do).
  kRan = 0,
  /// @brief It was passed over: a keyframe waited, or a stop was requested.
  kSkipped = 1,
  /// @brief It started and was stopped early.
  kAborted = 2,
  /// @brief The number of outcomes (a sentinel, not an outcome).
  kCount = 3,
};

/// @brief An outcome's name, as `slam_it.edge.mapping.Scheduled` spells it.
/// @param scheduled The outcome.
/// @return A static string.
[[nodiscard]] const char* name(Scheduled scheduled);

/// @brief How local mapping scheduled a keyframe's steps (`slam_it.edge.mapping.Schedule`).
struct Schedule {
  /// @brief Fusing duplicates with the neighbours (`SearchInNeighbors`).
  Scheduled fusion{Scheduled::kRan};
  /// @brief The local BA.
  Scheduled adjustment{Scheduled::kRan};
  /// @brief Culling redundant keyframes.
  Scheduled culling{Scheduled::kRan};
};

/// @brief What local mapping's own thread is asked between its steps
///        (`slam_it.edge.mapping.Interrupts`, ORB-SLAM's `LocalMapping::Run`).
struct Interrupts {
  /// @brief Whether another keyframe waits to be mapped (`CheckNewKeyFrames`).
  std::function<bool()> waiting;
  /// @brief Whether a stop of local mapping is requested (`stopRequested`).
  std::function<bool()> stopping;
  /// @brief The flag that stops the local BA (`mbAbortBA`); not owned.
  const std::atomic<bool>* abort = nullptr;
  /// @brief Publish the map so far for tracking, the keyframe's new points made (the
  ///        baseline: let the map's lock go there, between steps).
  std::function<void(const Map&)> publish;
  /// @brief What the local BA does around its solves (the baseline's lock); not owned, null
  ///        for nothing.
  const Around* around = nullptr;
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
  /// @brief How its steps were scheduled.
  Schedule schedule;
};

/// @brief The depth below which a point is close (`slam_it.edge.mapping.close_depth`).
/// @param camera The camera (its baseline).
/// @param options The factor.
/// @return `stereo.close_depth` baselines, in metres.
[[nodiscard]] double close_depth(const Camera& camera, const Options& options);

/// @brief Decide whether a tracked frame becomes a keyframe
///        (`slam_it.edge.mapping.need_keyframe`, ORB-SLAM's `NeedNewKeyFrame`).
/// @param m The map, with at least one keyframe.
/// @param camera The camera.
/// @param depth The frame's keypoints' depths (NaN where unknown).
/// @param tracked Its pose and matches.
/// @param reference Its reference keyframe's row (`LocalTracking::reference`).
/// @param mode The pipeline's mode.
/// @param options The ratios and counts.
/// @param idle Whether local mapping is idle (as when it runs inline).
/// @param since_keyframe The frames since the last keyframe, read only with mapping busy; none,
///        as if too few.
/// @return Whether to insert a keyframe; `InvalidInput` for a map without keyframes, matches
///         that are not one per depth, a reference out of range, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<bool> need_keyframe(
    const Map& m, const Camera& camera, std::span<const double> depth, const Tracked& tracked,
    std::int64_t reference, Mode mode, const Options& options, bool idle = true,
    std::optional<std::int64_t> since_keyframe = std::nullopt);

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

/// @brief Triangulate the newest keyframe's free keypoints against its best links
///        (`slam_it.edge.mapping.triangulate_new_points`, ORB-SLAM's `CreateNewMapPoints`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The thresholds.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> triangulate_new_points(Map& m, const Camera& camera,
                                                               Mode mode, const Options& options);

/// @brief Fuse the newest keyframe's points with its neighbours'
///        (`slam_it.edge.mapping.fuse_points`, ORB-SLAM's `SearchInNeighbors`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The neighbours and thresholds.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> fuse_points(Map& m, const Camera& camera, Mode mode,
                                                    const Options& options);

/// @brief The keyframes the newest keyframe's points are fused into
///        (`slam_it.edge.mapping.fuse_targets`, ORB-SLAM's `SearchInNeighbors`): its best
///        `mapping.fuse_neighbours` links (`mapping.mono_fuse_neighbours` in monocular), then each
///        one's best `mapping.fuse_second_neighbours`, leaving out the newest keyframe and repeats
///        (ORB-SLAM3's order).
/// @param m The map.
/// @param mode The pipeline's mode.
/// @param options The settings.
/// @return The targets' ids, in order; `InvalidInput` for a map without keyframes or invalid
///         settings.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> fuse_targets(const Map& m, Mode mode,
                                                                          const Options& options);

/// @brief The keyframes local BA adjusts (`slam_it.edge.mapping.local_ba_keyframes`, ORB-SLAM's
///        `LocalBundleAdjustment`): the newest and every keyframe linked to it.
/// @param m The map.
/// @param options The settings.
/// @return Their rows, ascending; `InvalidInput` for a map without keyframes or invalid
///         settings.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> local_ba_keyframes(
    const Map& m, const Options& options);

/// @brief Cull the newest keyframe's redundant links (`slam_it.edge.mapping.cull_keyframes`,
///        ORB-SLAM's `KeyFrameCulling`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param mode The pipeline's mode.
/// @param options The fraction and count.
/// @param held The ids of the keyframes loop detection holds, which `cull` spares.
/// @return Nothing; `InvalidInput` for a map without keyframes, or an invalid camera or
///         settings.
[[nodiscard]] foundations::Result<void> cull_keyframes(Map& m, const Camera& camera, Mode mode,
                                                       const Options& options,
                                                       std::span<const std::int64_t> held = {});

/// @brief Insert a keyframe and run local mapping on it (`slam_it.edge.mapping.process`).
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param frame The new keyframe's frame.
/// @param tracked Its pose and matches.
/// @param mode The pipeline's mode.
/// @param options The settings.
/// @param held The ids of the keyframes loop detection holds, which culling spares.
/// @param interrupts What local mapping's thread is asked between the steps; null inline, every
///        step run.
/// @return How each step went and was scheduled, and the points made for the keyframe's
///         keypoints; `InvalidInput` for matches that are not one per keypoint, or an invalid
///         camera or settings.
[[nodiscard]] foundations::Result<Mapped> process(Map& m, const Camera& camera, const Frame& frame,
                                                  const Tracked& tracked, Mode mode,
                                                  const Options& options,
                                                  std::span<const std::int64_t> held = {},
                                                  const Interrupts* interrupts = nullptr);

}  // namespace slam_it::edge
