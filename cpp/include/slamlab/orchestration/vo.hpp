/// @file
/// @brief A whole sequence through the edge pipeline: the C++ mirror of
///        `slamlab.orchestration.vo` (its docstring states the run), behind the native program
///        `slamlab_vo` (`cpp/apps/orchestration/slamlab_vo.cpp`).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

#include "slamlab/edge/options.hpp"
#include "slamlab/eval/runs.hpp"
#include "slamlab/foundations/error.hpp"

/// @brief Orchestration: running a rung's pipeline over a dataset's sequence.
namespace slamlab::orchestration::vo {

/// @brief The datasets a sequence can come from (`slamlab.orchestration.vo.DATASETS`).
inline constexpr std::array<const char*, 2> kDatasets{"tum", "euroc"};

/// @brief The program's command line.
inline constexpr const char* kUsage =
    "usage: slamlab_vo --dataset tum|euroc --sequence PATH --mode monocular|stereo|rgbd "
    "--out RUN_DIR [--first I] [--count N] [--option KEY=VALUE ...]";

/// @brief The program's exit code for a command line it cannot read.
inline constexpr int kExitUsage = 2;

/// @brief Its exit code for a failed run, plus the error's kind: 3 for `InvalidInput`, 4 for
///        `Degenerate`, 5 for `NotConverged` (`slamlab.orchestration.vo.EXIT_FAILED`).
inline constexpr int kExitFailed = 3;

/// @brief The exit code of a run that failed with an error of a kind (Rust's `exit_code`).
/// @param kind The error's kind.
/// @return `kExitFailed` for `InvalidInput`, then `Degenerate`, then `NotConverged`.
[[nodiscard]] constexpr int exit_code(foundations::ErrorKind kind) {
  switch (kind) {
    case foundations::ErrorKind::kInvalidInput:
      return kExitFailed;
    case foundations::ErrorKind::kDegenerate:
      return kExitFailed + 1;
    case foundations::ErrorKind::kNotConverged:
      return kExitFailed + 2;
  }
  return kExitFailed;
}

/// @brief A run asked for: the program's command line.
struct Request {
  /// @brief One of `kDatasets`.
  std::string dataset;
  /// @brief The sequence's folder.
  std::string sequence;
  /// @brief The sensor mode.
  edge::Mode mode{edge::Mode::kMonocular};
  /// @brief The run's folder, written.
  std::string out;
  /// @brief The first frame run.
  std::int64_t first{0};
  /// @brief The frames run from it, or all.
  std::optional<std::int64_t> count;
  /// @brief The pipeline's settings.
  edge::Options options;
};

/// @brief Read the command line: `kUsage`, flags in any order, `--option` repeatable and
///        naming a field of `slamlab.edge.options.Options`.
/// @param arguments The arguments after the program's name.
/// @return The request; `InvalidInput` for an unknown, repeated or missing flag, a flag
///         without its value, an unknown mode or setting, or a value that is not a number.
[[nodiscard]] foundations::Result<Request> parse(std::span<const std::string> arguments);

/// @brief Check a request (`slamlab.orchestration.vo.check`).
/// @param request The request.
/// @return Nothing; `InvalidInput` for an unknown dataset, a mode it does not record (TUM has
///         no stereo pairs, EuRoC no depth maps), a negative `first` or `count`, or settings
///         out of range.
[[nodiscard]] foundations::Result<void> check(const Request& request);

/// @brief Run a sequence through the pipeline: the run, not yet written, as Python's program
///        builds it before writing (`slamlab.orchestration.vo.main`). The program records its
///        own process in it (slamlab/eval/budgets.hpp) and writes it to `request.out`.
/// @param request The request.
/// @return The run; the check's, a reader's or the pipeline's error.
[[nodiscard]] foundations::Result<eval::runs::Run> run(const Request& request);

}  // namespace slamlab::orchestration::vo
