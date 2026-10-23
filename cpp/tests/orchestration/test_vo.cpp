/// @file
/// @brief Native tests for the native program's run (slam_it/orchestration/vo.hpp): reading the
///        command line, the request's checks, the sessions' time offsets, and runs of no frames
///        on fixture folders (the fixtures list images without holding them). The whole room is
///        run, through every backend, by tests/orchestration.
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/orchestration/vo.hpp"

namespace {

namespace vo = slam_it::orchestration::vo;
namespace edge = slam_it::edge;
namespace fs = std::filesystem;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief A fixture sequence's folder: its files listed, its images absent.
/// @param name Its name.
/// @return The path.
std::string sequence(const std::string& name) {
  return slam_it::testing::fixture_path("eval/sequences/" + name);
}

/// @brief A request, its other fields default.
/// @param dataset The dataset.
/// @param mode The mode.
/// @param name The sequence's fixture folder.
/// @return The request, written to `run`.
vo::Request request_for(const std::string& dataset, edge::Mode mode, const std::string& name) {
  vo::Request out;
  out.dataset = dataset;
  out.sequences = {sequence(name)};
  out.mode = mode;
  out.out = "run";
  out.options.relocalization.enabled = false;  // no vocabulary: these check the request
  out.options.loops.enabled = false;
  return out;
}

/// @brief A command line with its required flags, and more.
/// @param more The flags after them.
/// @return The arguments.
std::vector<std::string> command(const std::vector<std::string>& more) {
  std::vector<std::string> out{"--dataset", "tum",  "--sequence", sequence("tum_camera"),
                               "--mode",    "rgbd", "--out",      "run"};
  out.insert(out.end(), more.begin(), more.end());
  return out;
}

/// @brief Check the exit code each kind of failure gives (Python's `_ERRORS` order).
/// @param failures Running failure count.
void check_exit_codes(int& failures) {
  using slam_it::foundations::ErrorKind;
  expect(vo::exit_code(ErrorKind::kInvalidInput) == 3 &&
             vo::exit_code(ErrorKind::kDegenerate) == 4 &&
             vo::exit_code(ErrorKind::kNotConverged) == 5,
         "each error kind exits with its own code, from kExitFailed on", failures);
}

/// @brief Check reading a command line, and its refusals.
/// @param failures Running failure count.
void check_parse(int& failures) {
  const auto request = vo::parse(
      command({"--first", "3", "--record-features", "--count", "10", "--option",
               "covisibility.local_neighbours=7", "--option", "stereo.close_depth=35.5"}));
  expect(request.has_value() && request->dataset == "tum" && request->mode == edge::Mode::kRgbd &&
             request->out == "run" && request->first == 3 && request->count == 10 &&
             request->record_features,
         "parse reads every flag, --record-features without a value", failures);
  expect(request.has_value() && request->options.covisibility.local_neighbours == 7 &&
             request->options.stereo.close_depth == 35.5 &&
             request->options.features.n_features == 1000,
         "parse sets the options named, the rest default", failures);
  const auto plain = vo::parse(command({}));
  expect(plain.has_value() && plain->first == 0 && !plain->count.has_value() &&
             !plain->record_features && plain->sequences == std::vector{sequence("tum_camera")},
         "first and count default to all frames, and no features are recorded", failures);
  const auto sessions = vo::parse(command({"--sequence", "second", "--sequence", "third"}));
  expect(sessions.has_value() &&
             sessions->sequences ==
                 std::vector<std::string>{sequence("tum_camera"), "second", "third"},
         "--sequence repeats, each a session in turn", failures);
  const std::vector<std::vector<std::string>> refused{
      {"--mode", "lidar"},
      {"--first", "x"},
      {"--count"},
      {"--option", "covisibility.local_neighbourss=7"},
      {"--option", "covisibility.local_neighbours=7.5"},
      {"--option", "stereo.close_depth=inf"},
      {"--option", "window"},
      {"--dataset", "euroc"},
      {"--verbose", "1"},
      {"--option", "stereo.close_depth=+-5"},
      {"--record-features", "--record-features"},
  };
  for (const auto& more : refused) {
    expect(is_invalid_input(vo::parse(command(more))), ("parse refuses " + more[0]).c_str(),
           failures);
  }
  expect(is_invalid_input(vo::parse(std::vector<std::string>{"--dataset", "tum"})),
         "parse refuses a command line without its required flags", failures);
}

/// @brief Check the request's checks.
/// @param failures Running failure count.
void check_requests(int& failures) {
  const vo::Request good = request_for("tum", edge::Mode::kMonocular, "tum_camera");
  expect(vo::check(good).has_value(), "a monocular TUM request checks out", failures);
  vo::Request bad = good;
  bad.dataset = "kitti";
  expect(is_invalid_input(vo::check(bad)), "check refuses an unknown dataset", failures);
  bad = good;
  bad.mode = edge::Mode::kStereo;
  expect(is_invalid_input(vo::check(bad)), "check refuses stereo from TUM", failures);
  bad = request_for("euroc", edge::Mode::kRgbd, "euroc");
  expect(is_invalid_input(vo::check(bad)), "check refuses RGB-D from EuRoC", failures);
  bad = good;
  bad.count = -1;
  expect(is_invalid_input(vo::check(bad)), "check refuses a negative count", failures);
  bad = good;
  bad.options.covisibility.local_neighbours = 0;
  expect(is_invalid_input(vo::check(bad)), "check refuses settings out of range", failures);
  bad = good;
  bad.sequences.clear();
  expect(is_invalid_input(vo::check(bad)), "check refuses a request of no sequence", failures);
}

/// @brief Check the sessions' time offsets (`slam_it.orchestration.vo.session_offsets`'s
///        examples, and more).
/// @param failures Running failure count.
void check_session_offsets(int& failures) {
  using Times = std::vector<std::vector<double>>;
  expect(vo::session_offsets(Times{{0.0, 0.5, 1.0}, {2.0, 2.5}}) == std::vector{0.0, 0.0},
         "a session after the ones before keeps its clock", failures);
  expect(vo::session_offsets(Times{{0.0, 0.5, 1.0}, {0.0, 0.5}}) == std::vector{0.0, 1.5},
         "one that starts again comes a frame period after the last time before", failures);
  expect(vo::session_offsets(Times{{0.0, 0.1, 0.2, 0.6}, {}, {1.0}, {0.0}}) ==
             std::vector{0.0, 0.0, 0.0, 2.0},
         "the period is the median interval of the session before, 1 s for one frame", failures);
}

/// @brief Check runs on the fixture folders: none of their images exist.
/// @param failures Running failure count.
void check_runs(int& failures) {
  const slam_it::testing::ScratchDir scratch("slam_it_test_vo");
  const fs::path& out = scratch.path();
  vo::Request request = request_for("tum", edge::Mode::kRgbd, "tum_camera");
  request.out = out.string();
  request.count = 0;
  const auto empty = vo::run(request);
  std::vector<std::string> recorded;  // each group, and the seed
  const edge::Options defaults{};
  edge::for_each_setting(defaults, [&recorded](const char* name, const auto&) {
    const auto [group, field] = edge::split_setting(name);
    const std::string key(group.empty() ? field : group);
    if (recorded.empty() || recorded.back() != key) {
      recorded.push_back(key);
    }
  });
  expect(empty.has_value() && empty->timestamps.empty() && empty->sequence == "tum_camera" &&
             empty->backend == "cpp" && empty->settings.size() == recorded.size(),
         "a run of no frames records the sequence and every group of settings", failures);
  expect(!fs::exists(out / "run.json"), "the run is the program's to write", failures);
  if (empty.has_value()) {
    const auto written = slam_it::eval::runs::write_run(request.out, *empty);
    expect(written.has_value(), "the run writes", failures);
  }
  std::ifstream file(out / "run.json");
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  expect(text.contains("\"window\": 10") && fs::exists(out / "trajectory.txt"),
         "it writes the run folder", failures);
  request.count = 1;
  expect(is_invalid_input(vo::run(request)), "a missing image fails the run", failures);
  request.dataset = "euroc";
  request.sequences = {sequence("euroc")};
  request.mode = edge::Mode::kStereo;
  expect(is_invalid_input(vo::run(request)), "a missing stereo image fails the run", failures);
  request.sequences = {sequence("no_such_sequence")};
  expect(is_invalid_input(vo::run(request)), "a missing sequence fails the run", failures);
}

/// @brief Check runs of several sequences on the fixture folders, of no frames.
/// @param failures Running failure count.
void check_sessions(int& failures) {
  const slam_it::testing::ScratchDir scratch("slam_it_test_vo_sessions");
  vo::Request request = request_for("tum", edge::Mode::kMonocular, "tum_camera");
  request.out = scratch.path().string();
  request.count = 0;
  request.sequences.push_back(sequence("tum_no_depth"));
  const auto both = vo::run(request);
  expect(both.has_value() && both->sequence == "tum_camera+tum_no_depth",
         "a run of two sequences is named for both", failures);
  request.sequences.back() = sequence("rgbd_dataset_freiburg1_fixture");
  expect(is_invalid_input(vo::run(request)), "sequences of different cameras are refused",
         failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_exit_codes(failures);
  check_parse(failures);
  check_requests(failures);
  check_session_offsets(failures);
  check_runs(failures);
  check_sessions(failures);
  return slam_it::testing::finish("orchestration/vo", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("orchestration/vo", run); }
