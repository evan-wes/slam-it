/// @file
/// @brief `slam_it_vo`: a whole sequence through the edge pipeline, its run folder written
///        (slam_it/orchestration/vo.hpp). `slam_it.orchestration.vo.run` runs it for the
///        `cpp` backend.
///
/// Exit codes: 0 for a run written; `kExitUsage` (2) for a command line it cannot read, with
/// the usage; `kExitFailed` (3) plus the error's kind for a failed run. Messages go to stderr,
/// a one-line summary to stdout.
#include <cstddef>
#include <cstdio>
#include <exception>
#include <print>
#include <span>
#include <string>
#include <vector>

#include "slam_it/eval/budgets.hpp"
#include "slam_it/eval/runs.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/orchestration/vo.hpp"

namespace {

namespace budgets = slam_it::eval::budgets;
namespace vo = slam_it::orchestration::vo;

/// @brief Report an error on stderr.
/// @param error The error.
void report(const slam_it::foundations::Error& error) {
  std::println(stderr, "slam_it_vo: {}", error.message);
}

/// @brief Run the program.
/// @param arguments The arguments after its name.
/// @return The exit code.
int run(std::span<const std::string> arguments) {
  const auto request = vo::parse(arguments);
  if (!request) {
    report(request.error());
    std::println(stderr, "{}", vo::kUsage);
    return vo::kExitUsage;
  }
  auto done = vo::run(*request);
  if (!done) {
    report(done.error());
    return vo::exit_code(done.error().kind);
  }
  // What the program records of its own process, before it writes the run.
  done->peak_memory = budgets::peak_memory();
  done->cpus = budgets::allowed_cpus();
  done->memory_cap = budgets::memory_cap();
  if (const auto written = slam_it::eval::runs::write_run(request->out, *done); !written) {
    report(written.error());
    return vo::exit_code(written.error().kind);
  }
  std::println("slam_it_vo: {} of {} frames tracked, {} keyframes, {} points -> {}",
               done->timestamps.size(), done->diagnostics.size(), done->keyframe_timestamps.size(),
               done->points.size(), request->out);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::span<char*> given(argv, static_cast<std::size_t>(argc));
    const std::span<char*> after = given.empty() ? given : given.subspan(1);
    const std::vector<std::string> arguments(after.begin(), after.end());
    return run(arguments);
  } catch (const std::exception& error) {
    // Not println: the handler must not throw.
    std::fputs("slam_it_vo: ", stderr);
    std::fputs(error.what(), stderr);
    std::fputs("\n", stderr);
    return 1;
  }
}
