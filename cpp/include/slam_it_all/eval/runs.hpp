/// @file
/// @brief The run format's writer: the C++ mirror of `slam_it_all.eval.runs.write_run` (the
///        module's docstring states the folder's files). JSON and `.npy` are written by hand:
///        the formats written are small and fixed, and nothing native reads them back.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "slam_it_all/foundations/error.hpp"

/// @brief The run format.
namespace slam_it_all::eval::runs {

/// @brief `run.json`'s `format`.
inline constexpr const char* kFormat = "slam-it-all-run";

/// @brief `run.json`'s `version` (2 added `peak_memory`, `cpus` and `memory_cap`; 3 every point
///        and keyframe the run made, with their lives).
inline constexpr std::int64_t kVersion = 3;

/// @brief The number of removal codes a life may hold (`slam_it_all.eval.runs.REMOVALS`).
inline constexpr std::int64_t kRemovals = 3;

/// @brief A life: the frame that made a point or keyframe, the frame that removed it or -1, and
///        why (its index in `slam_it_all.eval.runs.REMOVALS`) or -1.
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

/// @brief A pipeline's run (`slam_it_all.eval.runs.Run`).
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
  /// @brief Each tracked frame's `T_world_cam`.
  std::vector<Eigen::Matrix4d> poses;
  /// @brief The timestamps of every keyframe the run made, by id.
  std::vector<double> keyframe_timestamps;
  /// @brief Their last `T_world_cam`.
  std::vector<Eigen::Matrix4d> keyframe_poses;
  /// @brief Every point the run made, by id, where it last was, in the world frame.
  std::vector<Eigen::Vector3d> points;
  /// @brief The pipeline's options.
  JsonObject settings;
  /// @brief One object per frame, each with a `timestamp`; the edge pipeline's also time
  ///        its steps (`read_seconds`, `build_seconds`, `track_seconds`, `map_seconds`, as
  ///        `slam_it_all.eval.runs.STEPS` defines them) and say how its stages went.
  std::vector<JsonObject> diagnostics;
  /// @brief The largest resident set of the program that ran it, in bytes, if known
  ///        (`slam_it_all::eval::budgets::peak_memory`); written as null otherwise.
  std::optional<std::int64_t> peak_memory;
  /// @brief The CPUs the program that ran it could run on, ascending, if known
  ///        (`slam_it_all::eval::budgets::allowed_cpus`); written as null otherwise.
  std::optional<std::vector<std::int64_t>> cpus;
  /// @brief Its memory cap, in bytes, if it had one (`slam_it_all::eval::budgets::memory_cap`);
  ///        written as null otherwise.
  std::optional<std::int64_t> memory_cap;
  /// @brief Each point's life, by id.
  std::vector<Life> point_lives;
  /// @brief Each keyframe's life, by id.
  std::vector<Life> keyframe_lives;
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
///        temporary sibling, then renamed): `slam_it_all.eval.runs.write_run`.
/// @param folder The folder.
/// @param run The run.
/// @return Nothing; `InvalidInput` for an unknown mode or backend, times that are not finite
///         and strictly ascending, poses that are not rigid or not one per time, a point that
///         is not finite, lives that are not one per point or keyframe or not lives of the
///         run's frames, a diagnostic without a `timestamp`, a number JSON cannot hold, a
///         negative peak memory, CPUs that are empty or not ascending, or a memory cap of 0
///         (nothing is written then); or a file that cannot be written.
[[nodiscard]] foundations::Result<void> write_run(const std::string& folder, const Run& run);

}  // namespace slam_it_all::eval::runs
