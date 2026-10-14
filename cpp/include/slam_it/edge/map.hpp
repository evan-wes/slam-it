/// @file
/// @brief The map: the C++ core's mirror of `slam_it.edge.map`.
///
/// Keyframes, their keypoints and the map points, as one struct of arrays; points and
/// keyframes referred to by id, the id columns ascending (the Python module's docstring states
/// the tables and the rules a point's descriptor, normal and distances follow). Operations
/// change the map in place and leave it as Python's return it, array for array.
///
/// One column has no Python twin: `pt_observations`, each point's observing keypoint rows,
/// which the operations keep in step with `kp_point` (as ORB-SLAM's `MapPoint` keeps its
/// observations). With it, an operation on a point costs what the point's observations cost,
/// however many keyframes the map holds; Python's reference scans `kp_point` instead.
///
/// What was removed is kept aside, in the `gone_*` columns, until the pipeline takes it
/// (`clear_gone`), as in Python.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/orb.hpp"

namespace slam_it::edge {

/// @brief Why a point or keyframe left the map, with the codes a run records
///        (`slam_it.edge.map.Removal`, whose members document each).
enum class Removal : std::uint8_t {
  /// @brief Culled: a recent point tracking rarely found, or a redundant keyframe.
  kCulled = 0,
  /// @brief Fused into another point.
  kFused = 1,
  /// @brief Erased with its views: left seen too little.
  kWeak = 2,
  /// @brief Its map was reset: the track was lost while the map was too small to keep.
  kReset = 3,
  /// @brief The number of removals (a sentinel, not a removal).
  kCount = 4,
};

/// @brief Keyframes, their keypoints and the map points (`slam_it.edge.map.Map`).
struct Map {
  /// @brief The pyramid's levels.
  std::int64_t levels{8};
  /// @brief Its scale factor.
  double scale_factor{1.2};
  /// @brief Each keyframe's id, ascending.
  std::vector<std::int64_t> kf_id;
  /// @brief Its time, in seconds.
  std::vector<double> kf_time;
  /// @brief Its pose `T_cam_world`.
  std::vector<Eigen::Matrix4d> kf_pose;
  /// @brief Where each keyframe's keypoints start; the last is the keypoint count.
  std::vector<std::int64_t> kf_start{0};
  /// @brief Each keypoint's pixel.
  std::vector<std::array<double, 2>> kp_xy;
  /// @brief Its pyramid level.
  std::vector<std::int64_t> kp_level;
  /// @brief Its orientation bin.
  std::vector<std::int64_t> kp_angle;
  /// @brief Its descriptor.
  std::vector<foundations::orb::Descriptor> kp_descriptor;
  /// @brief Its unit bearing (NaN where it has none).
  std::vector<Eigen::Vector3d> kp_bearing;
  /// @brief Its depth, in metres (NaN for none).
  std::vector<double> kp_depth;
  /// @brief Its right-image column (NaN for none).
  std::vector<double> kp_right;
  /// @brief The id of the map point it observes, or -1.
  std::vector<std::int64_t> kp_point;
  /// @brief Each point's id, ascending.
  std::vector<std::int64_t> pt_id;
  /// @brief Its position in the world.
  std::vector<Eigen::Vector3d> pt_xyz;
  /// @brief Its distinctive descriptor.
  std::vector<foundations::orb::Descriptor> pt_descriptor;
  /// @brief Its mean viewing direction, a unit vector.
  std::vector<Eigen::Vector3d> pt_normal;
  /// @brief The nearest distance it is matchable from.
  std::vector<double> pt_min_distance;
  /// @brief The farthest.
  std::vector<double> pt_max_distance;
  /// @brief How many frames had it in view.
  std::vector<std::int64_t> pt_visible;
  /// @brief How many matched it.
  std::vector<std::int64_t> pt_found;
  /// @brief The id of the keyframe that created it.
  std::vector<std::int64_t> pt_first_kf;
  /// @brief The id of its reference keyframe.
  std::vector<std::int64_t> pt_reference_kf;
  /// @brief The next keyframe id to give.
  std::int64_t next_kf_id{0};
  /// @brief The next point id to give.
  std::int64_t next_pt_id{0};
  /// @brief The ids of the points erased since the archive was last cleared, in erasing order.
  std::vector<std::int64_t> gone_pt_id;
  /// @brief Their last positions.
  std::vector<Eigen::Vector3d> gone_pt_xyz;
  /// @brief Why each went (`Removal`'s code).
  std::vector<std::int64_t> gone_pt_how;
  /// @brief The ids of the keyframes erased since then (culled).
  std::vector<std::int64_t> gone_kf_id;
  /// @brief Their times.
  std::vector<double> gone_kf_time;
  /// @brief Their last poses `T_cam_world`.
  std::vector<Eigen::Matrix4d> gone_kf_pose;
  /// @brief Each point's observing keypoint rows, ascending: the index of `kp_point` the
  ///        operations here keep. It does not cross to Python; `index_observations` builds it
  ///        for a map assembled column by column.
  std::vector<std::vector<std::int64_t>> pt_observations;
};

/// @brief A map's number of keyframes.
/// @param m The map.
/// @return It.
[[nodiscard]] inline std::size_t keyframe_count(const Map& m) { return m.kf_id.size(); }

/// @brief A map's number of keypoints.
/// @param m The map.
/// @return It.
[[nodiscard]] inline std::size_t keypoint_count(const Map& m) { return m.kp_point.size(); }

/// @brief A map's number of points.
/// @param m The map.
/// @return It.
[[nodiscard]] inline std::size_t point_count(const Map& m) { return m.pt_id.size(); }

/// @brief A map with no keyframes and no points.
/// @param levels The pyramid's levels.
/// @param scale_factor Its scale factor.
/// @return The empty map.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (levels, scale_factor), as in Python
[[nodiscard]] Map empty_map(std::int64_t levels, double scale_factor);

/// @brief Check a map's structure (`Map.__post_init__`).
/// @param m The map.
/// @return Nothing, or `InvalidInput` for arrays of the wrong length, offsets that do not run
///         from 0 to the keypoint count, id columns that are not strictly ascending from 0 or
///         reach their next id, a keypoint observing a point not in the map, or observation
///         lists that are not `kp_point`'s.
[[nodiscard]] foundations::Result<void> check_map(const Map& m);

/// @brief Build `pt_observations` from `kp_point`, for a map assembled column by column.
/// @param m The map (ids in `kp_point` not in the map are left out; `check_map` refuses them).
void index_observations(Map& m);

/// @brief Find a point by id.
/// @param m The map.
/// @param id The id (-1 allowed).
/// @return Its row, or -1.
[[nodiscard]] std::int64_t point_row(const Map& m, std::int64_t id);

/// @brief Find a keyframe by id.
/// @param m The map.
/// @param id The id.
/// @return Its row, or -1.
[[nodiscard]] std::int64_t keyframe_row(const Map& m, std::int64_t id);

/// @brief A keypoint's keyframe row.
/// @param m The map.
/// @param row The keypoint's row, in range.
/// @return The row of the keyframe whose keypoints hold it.
[[nodiscard]] std::size_t keyframe_of(const Map& m, std::int64_t row);

/// @brief Move a world point into a camera's frame: `R X + t` (`slam_it.edge.map.to_camera`).
/// @param pose The camera's `T_cam_world`.
/// @param x The point.
/// @return The point in the camera's frame.
[[nodiscard]] Eigen::Vector3d to_camera(const Eigen::Matrix4d& pose, const Eigen::Vector3d& x);

/// @brief A camera's centre in the world, `-Rᵀ t` (`slam_it.edge.map.camera_centre`).
/// @param pose The camera's `T_cam_world`.
/// @return The centre.
[[nodiscard]] Eigen::Vector3d camera_centre(const Eigen::Matrix4d& pose);

/// @brief A keyframe's camera centre in the world, `-Rᵀ t`.
/// @param m The map.
/// @param k The keyframe's row.
/// @return The centre.
[[nodiscard]] Eigen::Vector3d centre(const Map& m, std::size_t k);

/// @brief The sliding window: the rows of the last `size` keyframes, oldest first.
/// @param m The map.
/// @param size The window's size.
/// @return The rows.
[[nodiscard]] std::vector<std::int64_t> window(const Map& m, std::int64_t size);

/// @brief The points the given keyframes observe.
/// @param m The map.
/// @param keyframes Keyframe rows.
/// @return The points' rows, ascending, each once.
[[nodiscard]] std::vector<std::int64_t> observed_by(const Map& m,
                                                    std::span<const std::int64_t> keyframes);

/// @brief The cosine between a point's mean viewing direction and a view of it
///        (`slam_it.edge.map.viewing_cos`, `isInFrustum`'s `viewCos`), unchecked.
/// @param m The map.
/// @param p The point's row.
/// @param offset The point minus the viewing camera's centre.
/// @param distance The offset's length, positive.
/// @return The cosine.
[[nodiscard]] double viewing_cos(const Map& m, std::size_t p, const Eigen::Vector3d& offset,
                                 double distance);

/// @brief Whether a point lies within the distances it is matchable from
///        (`slam_it.edge.map.in_distance_range`): from `min_distance_margin` of its minimum
///        distance to `max_distance_margin` of its maximum (`isInFrustum`, `Fuse`), unchecked.
/// @param m The map.
/// @param p The point's row.
/// @param distance Its distance from the camera centre.
/// @param options The margins.
/// @return Whether it is in range.
[[nodiscard]] bool in_distance_range(const Map& m, std::size_t p, double distance,
                                     const Options& options);

/// @brief Predict the pyramid level points appear at from a distance (`PredictScale`).
/// @param m The map.
/// @param rows The points' rows.
/// @param distance Their distances, positive.
/// @return `ceil(log(d_max / d) / log s)`, clamped to the pyramid; `InvalidInput` for a row
///         out of range or a distance that is not positive, or not one per row.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> predict_level(
    const Map& m, std::span<const std::int64_t> rows, std::span<const double> distance);

/// @brief Add a keyframe and refresh the points its keypoints observe.
/// @param m The map.
/// @param frame The frame (its pyramid the map's).
/// @param pose Its `T_cam_world`.
/// @param points The id each keypoint observes (each in the map, at most once), or -1.
/// @return Nothing; `InvalidInput` for another pyramid, a pose that is not finite, or ids that
///         are not one per keypoint, in the map, and distinct.
[[nodiscard]] foundations::Result<void> insert_keyframe(Map& m, const Frame& frame,
                                                        const Eigen::Matrix4d& pose,
                                                        std::span<const std::int64_t> points);

/// @brief Create points, each observed by keyframe keypoints; their creator and reference is
///        the last observer's keyframe, visible and found once.
/// @param m The map.
/// @param xyz The points in the world, finite.
/// @param observers Each point's observing keypoint rows, at least one: free, used once, and
///        each point's from different keyframes.
/// @return Nothing, or `InvalidInput` for points that are not finite or observers breaking a
///         rule.
[[nodiscard]] foundations::Result<void> add_points(
    Map& m, std::span<const Eigen::Vector3d> xyz,
    std::span<const std::vector<std::int64_t>> observers);

/// @brief Pick the descriptor with the least median distance to the others (ties: earliest).
/// @param descriptors At least one descriptor.
/// @return Its index; `InvalidInput` for none.
[[nodiscard]] foundations::Result<std::int64_t> distinctive_descriptor(
    std::span<const foundations::orb::Descriptor> descriptors);

/// @brief Recompute points' descriptors, normals and distances from their observations.
/// @param m The map.
/// @param rows The points' rows, each observed.
/// @return Nothing, or `InvalidInput` for a row out of range or a point with no observation.
[[nodiscard]] foundations::Result<void> refresh_points(Map& m, std::span<const std::int64_t> rows);

/// @brief Remove points, kept aside with why they went; the keypoints that observed them observe
///        nothing.
/// @param m The map.
/// @param erase One flag per point, nonzero to remove it.
/// @param how Why they go.
/// @return Nothing, or `InvalidInput` for flags that are not one per point.
[[nodiscard]] foundations::Result<void> erase_points(Map& m, std::span<const std::uint8_t> erase,
                                                     Removal how);

/// @brief Empty the map's archive of removed points and keyframes (`slam_it.edge.map
///        .clear_gone`); the pipeline does after dating each frame's removals.
/// @param m The map.
void clear_gone(Map& m);

/// @brief Each point's observation weight: 1 per keypoint, 2 for one with a right column.
/// @param m The map.
/// @param rows The points' rows.
/// @return The weights; `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<std::vector<std::int64_t>> observation_weight(
    const Map& m, std::span<const std::int64_t> rows);

/// @brief Remove keypoints' observations; points left with weight 2 or less are erased, the
///        others refreshed (`EraseObservation`).
/// @param m The map.
/// @param rows Keypoint rows.
/// @return Nothing, or `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<void> forget_observations(Map& m,
                                                            std::span<const std::int64_t> rows);

/// @brief Remove a keyframe and its keypoints; its points lose an observation each.
/// @param m The map.
/// @param k The keyframe's row.
/// @return Nothing, or `InvalidInput` for a row out of range.
[[nodiscard]] foundations::Result<void> erase_keyframe(Map& m, std::int64_t k);

/// @brief Point a keypoint at a point, or at none, keeping the observation lists.
/// @param m The map.
/// @param row The keypoint's row.
/// @param id The point's id, or -1.
/// @return Nothing, or `InvalidInput` for a row out of range or an id not in the map.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (row, id), a keypoint and its point
[[nodiscard]] foundations::Result<void> observe(Map& m, std::int64_t row, std::int64_t id);

/// @brief Add to points' visible and found counts (ids no longer in the map are skipped).
/// @param m The map.
/// @param ids Point ids.
/// @param visible Each id's visible increment, non-negative.
/// @param found Each id's found increment, non-negative.
/// @return Nothing, or `InvalidInput` for increments that are negative or not one per id.
[[nodiscard]] foundations::Result<void> count(Map& m, std::span<const std::int64_t> ids,
                                              std::span<const std::int64_t> visible,
                                              std::span<const std::int64_t> found);

}  // namespace slam_it::edge
