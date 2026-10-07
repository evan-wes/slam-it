/// @file
/// @brief `slamlab_vo`: a whole sequence through the edge pipeline, its run folder written
///        (slamlab/orchestration/vo.hpp). `slamlab.orchestration.vo.run` runs it for the `cpp`
///        backend.
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

#include "slamlab/eval/runs.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/orchestration/vo.hpp"

namespace {

namespace vo = slamlab::orchestration::vo;

/// @brief Report an error on stderr.
/// @param error The error.
void report(const slamlab::foundations::Error& error) {
  std::println(stderr, "slamlab_vo: {}", error.message);
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
  const auto written = vo::run(*request);
  if (!written) {
    report(written.error());
    return vo::exit_code(written.error().kind);
  }
  std::println("slamlab_vo: {} of {} frames tracked, {} keyframes, {} points -> {}",
               written->timestamps.size(), written->diagnostics.size(),
               written->keyframe_timestamps.size(), written->points.size(), request->out);
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
    std::fputs("slamlab_vo: ", stderr);
    std::fputs(error.what(), stderr);
    std::fputs("\n", stderr);
    return 1;
  }
}
