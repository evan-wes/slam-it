/// @file
/// @brief Native tests for the run writer (slam_it/eval/runs.hpp) against the golden folder
///        `tests/fixtures/eval/run/`, which Python wrote from `run_values.json`: the `.npy`
///        files byte for byte (the streamed feature files too), the JSON and trajectory value
///        for value; and its refusals.
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/eval/runs.hpp"

namespace {

namespace runs = slam_it::eval::runs;
namespace fs = std::filesystem;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief A file's bytes.
/// @param path The file.
/// @return Its bytes.
std::string bytes(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

/// @brief Poses from JSON.
/// @param values The JSON list of 4x4 lists.
/// @return The poses.
std::vector<Eigen::Matrix4d> poses(const nlohmann::json& values) {
  std::vector<Eigen::Matrix4d> out;
  for (const auto& pose : values) {
    Eigen::Matrix4d m;
    for (Eigen::Index r = 0; r < 4; ++r) {
      for (Eigen::Index c = 0; c < 4; ++c) {
        m(r, c) = pose[r][c].get<double>();
      }
    }
    out.push_back(m);
  }
  return out;
}

/// @brief A JSON value as the run writer takes it: a scalar, or nested JSON as text (the
///        writer's own nesting is checked by `check_nested`).
/// @param value The value.
/// @return The writer's value.
runs::JsonValue value_of(const nlohmann::json& value) {
  if (value.is_null()) {
    return nullptr;
  }
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.is_number_integer()) {
    return value.get<std::int64_t>();
  }
  if (value.is_number()) {
    return value.get<double>();
  }
  if (value.is_string()) {
    return value.get<std::string>();
  }
  return runs::JsonText{.text = value.dump()};
}

/// @brief A JSON object as the run writer takes it, its keys in order.
/// @param object The object.
/// @return The object.
runs::JsonObject object_of(const nlohmann::json& object) {
  runs::JsonObject out;
  for (const auto& [key, item] : object.items()) {
    out.emplace_back(key, value_of(item));
  }
  return out;
}

/// @brief Lives from JSON rows of three integers.
/// @param values The rows.
/// @return The lives.
std::vector<runs::Life> lives(const nlohmann::json& values) {
  std::vector<runs::Life> out;
  for (const auto& life : values) {
    out.push_back(
        {life[0].get<std::int64_t>(), life[1].get<std::int64_t>(), life[2].get<std::int64_t>()});
  }
  return out;
}

/// @brief The run Python wrote, as the C++ backend.
/// @return The run.
runs::Run golden_run() {
  const nlohmann::json values = slam_it::testing::load_fixture("eval/run_values.json");
  runs::Run run{.dataset = values["dataset"],
                .sequence = values["sequence"],
                .mode = values["mode"],
                .backend = "cpp",
                .timestamps = values["timestamps"].get<std::vector<double>>(),
                .poses = poses(values["poses"]),
                .keyframe_timestamps = values["keyframe_timestamps"].get<std::vector<double>>(),
                .keyframe_poses = poses(values["keyframe_poses"]),
                .points = {},
                .settings = object_of(values["settings"]),
                .diagnostics = {},
                .peak_memory = values["peak_memory"].get<std::int64_t>(),
                .cpus = values["cpus"].get<std::vector<std::int64_t>>(),
                .memory_cap = values["memory_cap"].get<std::int64_t>(),
                .point_lives = lives(values["point_lives"]),
                .keyframe_lives = lives(values["keyframe_lives"]),
                .record_features = values["record_features"].get<bool>(),
                .pose_maps = values["pose_maps"].get<std::vector<std::int64_t>>(),
                .keyframe_maps = values["keyframe_maps"].get<std::vector<std::int64_t>>(),
                .point_maps = values["point_maps"].get<std::vector<std::int64_t>>()};
  for (const auto& point : values["points"]) {
    run.points.emplace_back(point[0].get<double>(), point[1].get<double>(), point[2].get<double>());
  }
  for (const auto& entry : values["diagnostics"]) {
    run.diagnostics.push_back(object_of(entry));
  }
  return run;
}

/// @brief A trajectory file's numbers.
/// @param path The file.
/// @return Its rows of numbers.
std::vector<std::vector<double>> trajectory(const fs::path& path) {
  std::ifstream file(path);
  std::vector<std::vector<double>> out;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream row(line);
    out.emplace_back(std::istream_iterator<double>(row), std::istream_iterator<double>());
  }
  return out;
}

/// @brief A trajectory file's comment lines, each with the number of poses before it.
/// @param path The file.
/// @return Each comment, as "<poses before>: <line>".
std::vector<std::string> comments(const fs::path& path) {
  std::ifstream file(path);
  std::vector<std::string> out;
  std::string line;
  std::size_t poses = 0;
  while (std::getline(file, line)) {
    if (!line.empty() && line[0] == '#') {
      out.push_back(std::to_string(poses) + ": " + line);
    } else if (!line.empty()) {
      ++poses;
    }
  }
  return out;
}

/// @brief A JSON-lines file's objects.
/// @param path The file.
/// @return The objects.
std::vector<nlohmann::json> json_lines(const fs::path& path) {
  std::ifstream file(path);
  std::vector<nlohmann::json> out;
  std::string line;
  while (std::getline(file, line)) {
    out.push_back(nlohmann::json::parse(line));
  }
  return out;
}

/// @brief Whether two trajectories agree: times and positions exactly, quaternions to the
///        last bits (each language converts the rotation itself).
/// @param got The written rows.
/// @param expected The golden rows.
/// @return Whether they agree.
bool same_trajectory(const std::vector<std::vector<double>>& got,
                     const std::vector<std::vector<double>>& expected) {
  bool ok = got.size() == expected.size();
  for (std::size_t k = 0; ok && k < got.size(); ++k) {
    ok = got[k].size() == 8;
    for (std::size_t i = 0; ok && i < 8; ++i) {
      ok = i < 4 ? got[k][i] == expected[k][i] : std::abs(got[k][i] - expected[k][i]) < 1e-15;
    }
  }
  return ok;
}

/// @brief Check the writer against the golden folder.
/// @param failures Running failure count.
void check_golden(int& failures) {
  const fs::path golden = fs::path(SLAM_IT_FIXTURES_DIR) / "eval" / "run";
  const slam_it::testing::ScratchDir scratch("slam_it_test_runs");
  const fs::path& out = scratch.path();
  expect(runs::write_run(out.string(), golden_run()).has_value(), "the run is written", failures);
  for (const char* name :
       {"keyframes.npy", "keyframe_poses.npy", "keyframe_lives.npy", "points.npy",
        "point_lives.npy", "pose_maps.npy", "keyframe_maps.npy", "point_maps.npy"}) {
    expect(bytes(out / name) == bytes(golden / name),
           (std::string(name) + " matches numpy's byte for byte").c_str(), failures);
  }
  nlohmann::json written = nlohmann::json::parse(bytes(out / "run.json"));
  nlohmann::json expected = nlohmann::json::parse(bytes(golden / "run.json"));
  expect(written["backend"] == "cpp", "run.json names the backend", failures);
  written.erase("backend");
  expected.erase("backend");
  expect(written == expected, "run.json holds Python's values", failures);
  expect(same_trajectory(trajectory(out / "trajectory.txt"), trajectory(golden / "trajectory.txt")),
         "trajectory.txt holds Python's values", failures);
  expect(comments(out / "trajectory.txt") == comments(golden / "trajectory.txt"),
         "trajectory.txt marks each map where Python's does", failures);
  expect(json_lines(out / "diagnostics.jsonl") == json_lines(golden / "diagnostics.jsonl"),
         "diagnostics.jsonl holds Python's values", failures);
  expect(!fs::exists(out / ".run.json.tmp"), "no temporary file is left", failures);
  runs::Run unknown = golden_run();
  unknown.peak_memory.reset();
  unknown.cpus.reset();
  unknown.memory_cap.reset();
  const bool written_unknown = runs::write_run(out.string(), unknown).has_value();
  const nlohmann::json info = nlohmann::json::parse(bytes(out / "run.json"));
  expect(written_unknown && info["peak_memory"].is_null() && info["cpus"].is_null() &&
             info["memory_cap"].is_null(),
         "an unknown process is written as nulls", failures);
}

/// @brief One frame's keypoints, as `run_values.json` holds them.
struct Keypoints {
  /// @brief Their pixels.
  std::vector<std::array<double, 2>> xy;
  /// @brief Their levels.
  std::vector<std::int64_t> level;
  /// @brief Their fates.
  std::vector<std::int64_t> fate;
  /// @brief Their points.
  std::vector<std::int64_t> point;
};

/// @brief Write one frame's keypoints.
/// @param writer The writer.
/// @param frame The keypoints.
/// @return Whether they were written.
bool write(runs::FeatureWriter& writer, const Keypoints& frame) {
  return writer.write(frame.xy, frame.level, frame.fate, frame.point).has_value();
}

/// @brief Check the feature writer against the golden folder's two files, and its refusals.
/// @param failures Running failure count.
void check_features(int& failures) {
  const nlohmann::json values = slam_it::testing::load_fixture("eval/run_values.json");
  std::vector<Keypoints> frames;
  for (const auto& frame : values["features"]) {
    frames.push_back({.xy = frame["xy"].get<std::vector<std::array<double, 2>>>(),
                      .level = frame["level"].get<std::vector<std::int64_t>>(),
                      .fate = frame["fate"].get<std::vector<std::int64_t>>(),
                      .point = frame["point"].get<std::vector<std::int64_t>>()});
  }
  const fs::path golden = fs::path(SLAM_IT_FIXTURES_DIR) / "eval" / "run";
  const slam_it::testing::ScratchDir scratch("slam_it_test_runs_features");
  const fs::path& out = scratch.path();
  auto writer = runs::FeatureWriter::open(out.string());
  expect(writer.has_value(), "the feature files are opened", failures);
  if (!writer) {
    return;
  }
  bool written = write(*writer, frames[0]);
  const std::string after_one = bytes(out / runs::kRecords);
  expect(written && after_one.contains("'shape': (3, 4)") && after_one.size() == 128 + (3 * 16),
         "after a frame, the header counts its rows", failures);
  Keypoints bad = frames[0];
  bad.fate[2] = 2;
  expect(!write(*writer, bad), "a tracked keypoint without a point is refused", failures);
  bad = frames[0];
  bad.point[0] = -1;
  expect(!write(*writer, bad), "a made keypoint without a point is refused", failures);
  bad = frames[0];
  bad.fate[0] = runs::kFates;
  expect(!write(*writer, bad), "an unknown fate is refused", failures);
  bad = frames[0];
  bad.level[0] = -1;
  expect(!write(*writer, bad), "a negative level is refused", failures);
  bad = frames[0];
  bad.xy[1][0] = std::nan("");
  expect(!write(*writer, bad), "a pixel that is not finite is refused", failures);
  bad = frames[0];
  bad.point.pop_back();
  expect(!write(*writer, bad), "columns that are not one row per keypoint are refused", failures);
  expect(bytes(out / runs::kRecords) == after_one, "nothing is written for refused records",
         failures);
  for (std::size_t k = 1; k < frames.size(); ++k) {
    written = write(*writer, frames[k]) && written;
  }
  for (const char* name : {runs::kPixels, runs::kRecords}) {
    expect(written && bytes(out / name) == bytes(golden / name),
           (std::string(name) + " matches Python's byte for byte").c_str(), failures);
  }
}

/// @brief Check the refusals: nothing is written.
/// @param failures Running failure count.
void check_refusals(int& failures) {
  const slam_it::testing::ScratchDir scratch("slam_it_test_runs_refused");
  const fs::path& out = scratch.path();
  runs::Run run = golden_run();
  run.mode = "lidar";
  expect(is_invalid_input(runs::write_run(out.string(), run)), "an unknown mode is refused",
         failures);
  run = golden_run();
  std::swap(run.timestamps[0], run.timestamps[1]);
  expect(is_invalid_input(runs::write_run(out.string(), run)), "times that descend are refused",
         failures);
  run = golden_run();
  std::erase_if(run.diagnostics[0], [](const auto& item) { return item.first == "timestamp"; });
  expect(is_invalid_input(runs::write_run(out.string(), run)),
         "a diagnostic without a timestamp is refused", failures);
  run = golden_run();
  run.peak_memory = -1;
  expect(is_invalid_input(runs::write_run(out.string(), run)), "a negative peak memory is refused",
         failures);
  for (const std::vector<std::int64_t>& cpus :
       {std::vector<std::int64_t>{}, {5, 4}, {4, 4}, {-1, 4}}) {
    run = golden_run();
    run.cpus = cpus;
    expect(is_invalid_input(runs::write_run(out.string(), run)),
           "CPUs that are not ascending CPU numbers are refused", failures);
  }
  run = golden_run();
  run.memory_cap = 0;
  expect(is_invalid_input(runs::write_run(out.string(), run)), "a memory cap of 0 is refused",
         failures);
  run = golden_run();
  run.point_lives.pop_back();
  expect(is_invalid_input(runs::write_run(out.string(), run)),
         "lives that are not one per point are refused", failures);
  run = golden_run();
  run.point_lives[1] = {3, 0, 1};
  expect(is_invalid_input(runs::write_run(out.string(), run)),
         "a life removed before it was made is refused", failures);
  run = golden_run();
  run.keyframe_lives[0] = {0, -1, 2};
  expect(is_invalid_input(runs::write_run(out.string(), run)),
         "a removal code on a life still alive is refused", failures);
  run = golden_run();
  run.keyframe_lives[0] = {0, 2, 4};
  expect(is_invalid_input(runs::write_run(out.string(), run)),
         "a removal code past reset is refused", failures);
  run = golden_run();
  run.pose_maps.pop_back();
  expect(is_invalid_input(runs::write_run(out.string(), run)),
         "maps that are not one per pose are refused", failures);
  run = golden_run();
  run.point_maps = {0, 1, 0, 1, 1};
  expect(is_invalid_input(runs::write_run(out.string(), run)), "maps that descend are refused",
         failures);
  run = golden_run();
  run.keyframe_maps[0] = -1;
  expect(is_invalid_input(runs::write_run(out.string(), run)), "a negative map is refused",
         failures);
  expect(!fs::exists(out), "nothing is written for a refused run", failures);
  expect(runs::json_number(1.0) == "1.0" && runs::json_number(0.1) == "0.1" &&
             runs::json_number(1e-05) == "1e-05",
         "numbers are written shortest, whole ones with .0", failures);
}

/// @brief Check nested JSON: written as Python's `json.dumps` writes it, its numbers finite.
/// @param failures Running failure count.
void check_nested(int& failures) {
  const auto solve = runs::json_object({{"initial_cost", 12.5},
                                        {"iterations", std::int64_t{5}},
                                        {"termination", std::string("function")},
                                        {"inliers", nullptr}});
  expect(solve.has_value() &&
             solve->text == R"({"initial_cost": 12.5, "iterations": 5, "termination": "function", )"
                            R"("inliers": null})",
         "an object nests as json.dumps writes it", failures);
  if (solve) {
    const std::vector<runs::JsonValue> items{*solve, true};
    const auto array = runs::json_array(items);
    expect(array.has_value() && array->text == "[" + solve->text + ", true]",
           "an array nests its items, objects included", failures);
  }
  expect(is_invalid_input(runs::json_object({{"cost", std::nan("")}})),
         "an object with a number that is not finite is refused", failures);
  const std::vector<runs::JsonValue> infinite{std::numeric_limits<double>::infinity()};
  expect(is_invalid_input(runs::json_array(infinite)),
         "an array with a number that is not finite is refused", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_golden(failures);
  check_features(failures);
  check_refusals(failures);
  check_nested(failures);
  return slam_it::testing::finish("eval/runs", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("eval/runs", run); }
