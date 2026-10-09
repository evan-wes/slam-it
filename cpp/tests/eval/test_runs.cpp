/// @file
/// @brief Native tests for the run writer (slam_it_all/eval/runs.hpp) against the golden folder
///        `tests/fixtures/eval/run/`, which Python wrote from `run_values.json`: the `.npy`
///        files byte for byte, the JSON and trajectory value for value; and its refusals.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it_all/eval/runs.hpp"

namespace {

namespace runs = slam_it_all::eval::runs;
namespace fs = std::filesystem;
using slam_it_all::testing::expect;
using slam_it_all::testing::is_invalid_input;

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

/// @brief A JSON object's scalars, in order.
/// @param object The object.
/// @return The flat object.
runs::JsonObject flat(const nlohmann::json& object) {
  runs::JsonObject out;
  for (const auto& [key, value] : object.items()) {
    if (value.is_boolean()) {
      out.emplace_back(key, value.get<bool>());
    } else if (value.is_number_integer()) {
      out.emplace_back(key, value.get<std::int64_t>());
    } else if (value.is_number()) {
      out.emplace_back(key, value.get<double>());
    } else {
      out.emplace_back(key, value.get<std::string>());
    }
  }
  return out;
}

/// @brief The run Python wrote, as the C++ backend.
/// @return The run.
runs::Run golden_run() {
  const nlohmann::json values = slam_it_all::testing::load_fixture("eval/run_values.json");
  runs::Run run{.dataset = values["dataset"],
                .sequence = values["sequence"],
                .mode = values["mode"],
                .backend = "cpp",
                .timestamps = values["timestamps"].get<std::vector<double>>(),
                .poses = poses(values["poses"]),
                .keyframe_timestamps = values["keyframe_timestamps"].get<std::vector<double>>(),
                .keyframe_poses = poses(values["keyframe_poses"]),
                .points = {},
                .settings = flat(values["settings"]),
                .diagnostics = {},
                .peak_memory = values["peak_memory"].get<std::int64_t>(),
                .cpus = values["cpus"].get<std::vector<std::int64_t>>(),
                .memory_cap = values["memory_cap"].get<std::int64_t>()};
  for (const auto& point : values["points"]) {
    run.points.emplace_back(point[0].get<double>(), point[1].get<double>(), point[2].get<double>());
  }
  for (const auto& entry : values["diagnostics"]) {
    run.diagnostics.push_back(flat(entry));
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
  const fs::path golden = fs::path(SLAM_IT_ALL_FIXTURES_DIR) / "eval" / "run";
  const slam_it_all::testing::ScratchDir scratch("slam_it_all_test_runs");
  const fs::path& out = scratch.path();
  expect(runs::write_run(out.string(), golden_run()).has_value(), "the run is written", failures);
  for (const char* name : {"keyframes.npy", "keyframe_poses.npy", "points.npy"}) {
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

/// @brief Check the refusals: nothing is written.
/// @param failures Running failure count.
void check_refusals(int& failures) {
  const slam_it_all::testing::ScratchDir scratch("slam_it_all_test_runs_refused");
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
  expect(!fs::exists(out), "nothing is written for a refused run", failures);
  expect(runs::json_number(1.0) == "1.0" && runs::json_number(0.1) == "0.1" &&
             runs::json_number(1e-05) == "1e-05",
         "numbers are written shortest, whole ones with .0", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_golden(failures);
  check_refusals(failures);
  return slam_it_all::testing::finish("eval/runs", failures);
}

}  // namespace

int main() { return slam_it_all::testing::guarded("eval/runs", run); }
