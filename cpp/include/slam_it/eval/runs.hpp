/// @file
/// @brief The run format's writer: the C++ mirror of `slam_it.eval.runs.write_run` (the
///        module's docstring states the folder's files). JSON and `.npy` are written by hand:
///        the formats written are small and fixed, and nothing native reads them back.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "slam_it/eval/vocabularies.hpp"
#include "slam_it/foundations/error.hpp"

/// @brief The run format.
namespace slam_it::eval::runs {

/// @brief `run.json`'s `format`.
inline constexpr const char* kFormat = "slam-it-run";

/// @brief `run.json`'s `version` (2 added `peak_memory`, `cpus` and `memory_cap`; 3 every point
///        and keyframe the run made, with their lives; 4 the maps, and the removal `reset`; 5 the
///        `vocabulary` a relocalising run used; 6 the settings grouped, each keyframe's parent and
///        the strong links).
inline constexpr std::int64_t kVersion = 7;

/// @brief The number of removal codes a life may hold (`slam_it.eval.runs.REMOVALS`).
inline constexpr std::int64_t kRemovals = 4;

/// @brief The number of fate codes a feature record may hold (`slam_it.eval.runs.FATES`);
///        0 is unmatched.
inline constexpr std::int64_t kFates = 4;

/// @brief The file of a run's keypoints' pixels (`slam_it.eval.runs.PIXELS`).
inline constexpr const char* kPixels = "feature_pixels.npy";

/// @brief The file of a run's keypoints' records (`slam_it.eval.runs.RECORDS`).
inline constexpr const char* kRecords = "feature_records.npy";

/// @brief A life: the frame that made a point or keyframe, the frame that removed it or -1, and
///        why (its index in `slam_it.eval.runs.REMOVALS`) or -1.
using Life = std::array<std::int64_t, 3>;

/// @brief JSON already written: an array or an object, as `json_array` and `json_object` write
///        it, to nest in a value.
struct JsonText {
  /// @brief The text.
  std::string text;
};

/// @brief A JSON value: null, an integer, a finite number, a boolean, a string, or an array or
///        object already written (`JsonText`). Nested JSON is kept as text, not as a tree, so
///        that no type or function here recurses (clang-tidy's `misc-no-recursion`; a copyable
///        tree recurses through its own copy).
using JsonValue = std::variant<std::nullptr_t, std::int64_t, double, bool, std::string, JsonText>;

/// @brief A JSON object, its keys in order.
using JsonObject = std::vector<std::pair<std::string, JsonValue>>;

/// @brief A pipeline's run (`slam_it.eval.runs.Run`).
struct Run {
  /// @brief The dataset's name.
  std::string dataset;
  /// @brief The sequence's name.
  std::string sequence;
  /// @brief `monocular`, `stereo` or `rgbd`.
  std::string mode;
  /// @brief `python`, `cpp` or `rust`.
  std::string backend;
  /// @brief Each tracked frame's timestamp, ascending.
  std::vector<double> timestamps;
  /// @brief Each tracked frame's final `T_world_cam` (`trajectory.txt`: its pose relative to its
  ///        reference keyframe, carried by that keyframe's last pose).
  std::vector<Eigen::Matrix4d> poses;
  /// @brief Each tracked frame's `T_world_cam` as tracked (`trajectory_tracked.txt`).
  std::vector<Eigen::Matrix4d> tracked_poses;
  /// @brief The timestamps of every keyframe the run made, by id.
  std::vector<double> keyframe_timestamps;
  /// @brief Their last `T_world_cam`.
  std::vector<Eigen::Matrix4d> keyframe_poses;
  /// @brief Every point the run made, by id, where it last was, in the world frame.
  std::vector<Eigen::Vector3d> points;
  /// @brief The pipeline's options.
  JsonObject settings;
  /// @brief The vocabulary a relocalising run used: its folder's header and checksums, written
  ///        as `slam_it.eval.vocabularies.record` writes them; null for a run that did not
  ///        relocalise.
  std::optional<vocabularies::Header> vocabulary;
  /// @brief One object per frame, each with a `timestamp`; the edge pipeline's also time
  ///        its steps (`read_seconds`, `build_seconds`, `track_seconds`, `map_seconds`,
  ///        `loop_seconds`, as `slam_it.eval.runs.STEPS` defines them) and say how its stages
  ///        went.
  std::vector<JsonObject> diagnostics;
  /// @brief The largest resident set of the program that ran it, in bytes, if known
  ///        (`slam_it::eval::budgets::peak_memory`); written as null otherwise.
  std::optional<std::int64_t> peak_memory;
  /// @brief The CPUs the program that ran it could run on, ascending, if known
  ///        (`slam_it::eval::budgets::allowed_cpus`); written as null otherwise.
  std::optional<std::vector<std::int64_t>> cpus;
  /// @brief Its memory cap, in bytes, if it had one (`slam_it::eval::budgets::memory_cap`);
  ///        written as null otherwise.
  std::optional<std::int64_t> memory_cap;
  /// @brief Each point's life, by id.
  std::vector<Life> point_lives;
  /// @brief Each keyframe's life, by id.
  std::vector<Life> keyframe_lives;
  /// @brief Each keyframe's parent in its map's spanning tree, the last it had, by id: a
  ///        keyframe of the same map, or -1.
  std::vector<std::int64_t> keyframe_parents;
  /// @brief The strong links: two keyframes of the same map, the lower id first, and the points
  ///        they share, ascending.
  std::vector<std::array<std::int64_t, 3>> keyframe_links;
  /// @brief The loops closed: two keyframes of the same map, the lower id first, in the order
  ///        they closed.
  std::vector<std::array<std::int64_t, 2>> keyframe_loops;
  /// @brief Whether the run recorded its features (`FeatureWriter` streams them).
  bool record_features{false};
  /// @brief The map each tracked frame's pose is in, never descending.
  std::vector<std::int64_t> pose_maps;
  /// @brief The map each keyframe is in, by id, never descending.
  std::vector<std::int64_t> keyframe_maps;
  /// @brief The map each point is in, by id, never descending.
  std::vector<std::int64_t> point_maps;
};

/// @brief Streams a run's feature records into its folder, a frame at a time
///        (`slam_it.eval.runs.FeatureWriter`): `kPixels`, (N, 2) float32, and `kRecords`,
///        (N, 4) int32 of frame, level, fate and point. After each frame the files' headers count
///        the rows so far, so each reads whole up to the last frame written. Frames are counted
///        from 0, one per `write`.
class FeatureWriter {
 public:
  /// @brief Open the two files, each with no rows yet.
  /// @param folder The run's folder, created if needed (the two files are replaced).
  /// @return The writer; `InvalidInput` for a folder or file that cannot be written.
  [[nodiscard]] static foundations::Result<FeatureWriter> open(const std::string& folder);

  /// @brief Append one frame's keypoints.
  /// @param xy Their pixels (written as float32).
  /// @param level Their pyramid levels.
  /// @param fate Their fates (indices in `slam_it.eval.runs.FATES`).
  /// @param point The point each concerns, -1 exactly where unmatched.
  /// @return Nothing; `InvalidInput` for columns that are not one row per keypoint, a pixel that
  ///         is not finite, a negative level, an unknown fate, a point that is not an id or -1
  ///         exactly where unmatched (nothing is written then), or a file that cannot be
  ///         written.
  [[nodiscard]] foundations::Result<void> write(std::span<const std::array<double, 2>> xy,
                                                std::span<const std::int64_t> level,
                                                std::span<const std::int64_t> fate,
                                                std::span<const std::int64_t> point);

 private:
  /// @brief A writer of two open files.
  /// @param pixels The pixels' file.
  /// @param records The records' file.
  FeatureWriter(std::ofstream pixels, std::ofstream records)
      : pixels_(std::move(pixels)), records_(std::move(records)) {}

  /// @brief Rewrite each header with the rows so far, and flush.
  /// @return Nothing; `InvalidInput` if a file cannot be written.
  foundations::Result<void> count();

  /// @brief The pixels' file.
  std::ofstream pixels_;
  /// @brief The records' file.
  std::ofstream records_;
  /// @brief Frames written.
  std::int64_t frames_{0};
  /// @brief Rows written.
  std::size_t rows_{0};
};

/// @brief An object as JSON text, to nest in a value: Python's `json.dumps` with its
///        separators (`", "` and `": "`).
/// @param object The object.
/// @return The text; `InvalidInput` for a number that is not finite.
[[nodiscard]] foundations::Result<JsonText> json_object(const JsonObject& object);

/// @brief An array as JSON text, to nest in a value (`json.dumps`, as `json_object`).
/// @param values The values.
/// @return The text; `InvalidInput` for a number that is not finite.
[[nodiscard]] foundations::Result<JsonText> json_array(std::span<const JsonValue> values);

/// @brief A finite number as JSON, shortest round-trip (`std::to_chars`), `.0` kept on a
///        whole number as Python writes it.
/// @param value The number, finite.
/// @return Its text.
[[nodiscard]] std::string json_number(double value);

/// @brief A `.npy` file's bytes (version 1.0, little-endian int64, C order), as numpy writes it.
/// @param shape The array's shape.
/// @param values Its values, row-major.
/// @return The bytes.
[[nodiscard]] std::string npy(const std::vector<std::size_t>& shape,
                              const std::vector<std::int64_t>& values);

/// @brief A `.npy` file's bytes (version 1.0, little-endian float64, C order), as numpy writes
///        it.
/// @param shape The array's shape.
/// @param values Its values, row-major.
/// @return The bytes.
[[nodiscard]] std::string npy(const std::vector<std::size_t>& shape,
                              const std::vector<double>& values);

/// @brief Write a run's folder, created if needed, each file replaced whole (written to a
///        temporary sibling, then renamed): `slam_it.eval.runs.write_run`.
/// @param folder The folder.
/// @param run The run.
/// @return Nothing; `InvalidInput` for an unknown mode or backend, times that are not finite
///         and strictly ascending, poses that are not rigid or not one per time, a point that
///         is not finite, lives that are not one per point or keyframe or not lives of the
///         run's frames, maps that are not one per pose, keyframe or point, or not numbers from
///         0 never descending, a diagnostic without a `timestamp`, diagnostics whose `map`s are
///         not null or map numbers never descending, or name no frame of a map with keyframes,
///         a number JSON cannot hold, a negative peak memory, CPUs that are empty or not ascending,
///         or a memory cap of 0 (nothing is written then); or a file that cannot be written.
[[nodiscard]] foundations::Result<void> write_run(const std::string& folder, const Run& run);

}  // namespace slam_it::eval::runs
