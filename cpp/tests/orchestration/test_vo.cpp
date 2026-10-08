/// @file
/// @brief Native tests for the native program's run (slamlab/orchestration/vo.hpp): reading the
///        command line, the request's checks, and a run of no frames on a fixture folder (the
///        fixtures list images without holding them). The whole room is run, through every
///        backend, by tests/orchestration.
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slamlab/edge/options.hpp"
#include "slamlab/orchestration/vo.hpp"

namespace {

namespace vo = slamlab::orchestration::vo;
namespace edge = slamlab::edge;
namespace fs = std::filesystem;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;

/// @brief A fixture sequence's folder: its files listed, its images absent.
/// @param name Its name.
/// @return The path.
std::string sequence(const std::string& name) {
  return slamlab::testing::fixture_path("eval/sequences/" + name);
}

/// @brief A request, its other fields default.
/// @param dataset The dataset.
/// @param mode The mode.
/// @param name The sequence's fixture folder.
/// @return The request, written to `run`.
vo::Request request_for(const std::string& dataset, edge::Mode mode, const std::string& name) {
  vo::Request out;
  out.dataset = dataset;
  out.sequence = sequence(name);
  out.mode = mode;
  out.out = "run";
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
  using slamlab::foundations::ErrorKind;
  expect(vo::exit_code(ErrorKind::kInvalidInput) == 3 &&
             vo::exit_code(ErrorKind::kDegenerate) == 4 &&
             vo::exit_code(ErrorKind::kNotConverged) == 5,
         "each error kind exits with its own code, from kExitFailed on", failures);
}

/// @brief Check reading a command line, and its refusals.
/// @param failures Running failure count.
void check_parse(int& failures) {
  const auto request = vo::parse(command(
      {"--first", "3", "--count", "10", "--option", "window=7", "--option", "close_depth=35.5"}));
  expect(request.has_value() && request->dataset == "tum" && request->mode == edge::Mode::kRgbd &&
             request->out == "run" && request->first == 3 && request->count == 10,
         "parse reads every flag", failures);
  expect(request.has_value() && request->options.window == 7 &&
             request->options.close_depth == 35.5 && request->options.n_features == 1000,
         "parse sets the options named, the rest default", failures);
  const auto plain = vo::parse(command({}));
  expect(plain.has_value() && plain->first == 0 && !plain->count.has_value(),
         "first and count default to all frames", failures);
  const std::vector<std::vector<std::string>> refused{
      {"--mode", "lidar"},       {"--first", "x"},           {"--count"},
      {"--option", "windows=7"}, {"--option", "window=7.5"}, {"--option", "close_depth=inf"},
      {"--option", "window"},    {"--dataset", "euroc"},     {"--verbose", "1"},
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
  bad.options.window = 0;
  expect(is_invalid_input(vo::check(bad)), "check refuses settings out of range", failures);
}

/// @brief Check runs on the fixture folders: none of their images exist.
/// @param failures Running failure count.
void check_runs(int& failures) {
  const slamlab::testing::ScratchDir scratch("slamlab_test_vo");
  const fs::path& out = scratch.path();
  vo::Request request = request_for("tum", edge::Mode::kRgbd, "tum_camera");
  request.out = out.string();
  request.count = 0;
  const auto empty = vo::run(request);
  expect(empty.has_value() && empty->timestamps.empty() && empty->sequence == "tum_camera" &&
             empty->backend == "cpp" &&
             empty->settings.size() == edge::kIntegerOptions.size() + edge::kRealOptions.size(),
         "a run of no frames records the sequence and every setting", failures);
  std::ifstream file(out / "run.json");
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  expect(text.contains("\"window\": 10") && fs::exists(out / "trajectory.txt"),
         "it writes the run folder", failures);
  request.count = 1;
  expect(is_invalid_input(vo::run(request)), "a missing image fails the run", failures);
  request.dataset = "euroc";
  request.sequence = sequence("euroc");
  request.mode = edge::Mode::kStereo;
  expect(is_invalid_input(vo::run(request)), "a missing stereo image fails the run", failures);
  request.sequence = sequence("no_such_sequence");
  expect(is_invalid_input(vo::run(request)), "a missing sequence fails the run", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_exit_codes(failures);
  check_parse(failures);
  check_requests(failures);
  check_runs(failures);
  return slamlab::testing::finish("orchestration/vo", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("orchestration/vo", run); }
