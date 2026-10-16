/// @file
/// @brief A whole sequence through the edge pipeline: the C++ mirror of
///        `slam_it.orchestration.vo` (its docstring states the run), behind the native program
///        `slam_it_vo` (`cpp/apps/orchestration/slam_it_vo.cpp`).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

#include "slam_it/edge/options.hpp"
#include "slam_it/eval/runs.hpp"
#include "slam_it/foundations/error.hpp"

/// @brief Orchestration: running a rung's pipeline over a dataset's sequence.
namespace slam_it::orchestration::vo {

/// @brief The datasets a sequence can come from (`slam_it.orchestration.vo.DATASETS`).
inline constexpr std::array<const char*, 2> kDatasets{"tum", "euroc"};

/// @brief The program's command line.
inline constexpr const char* kUsage =
    "usage: slam_it_vo --dataset tum|euroc --sequence PATH --mode monocular|stereo|rgbd "
    "--out RUN_DIR [--first I] [--count N] [--vocabulary FOLDER] [--record-features] "
    "[--option KEY=VALUE ...]";

/// @brief The program's exit code for a command line it cannot read.
inline constexpr int kExitUsage = 2;

/// @brief Its exit code for a failed run, plus the error's kind: 3 for `InvalidInput`, 4 for
///        `Degenerate`, 5 for `NotConverged` (`slam_it.orchestration.vo.EXIT_FAILED`).
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
  /// @brief Whether the run records each frame's keypoints, with their fates.
  bool record_features{false};
  /// @brief The vocabulary's folder: required with `relocalize` on, refused with it off.
  std::optional<std::string> vocabulary;
};

/// @brief Read the command line: `kUsage`, flags in any order, each with its value but
///        `--record-features`, `--option` repeatable and naming a field of
///        `slam_it.edge.options.Options`.
/// @param arguments The arguments after the program's name.
/// @return The request; `InvalidInput` for an unknown, repeated or missing flag, a flag
///         without its value, an unknown mode or setting, or a value that is not a number.
[[nodiscard]] foundations::Result<Request> parse(std::span<const std::string> arguments);

/// @brief Check a request (`slam_it.orchestration.vo.check`).
/// @param request The request.
/// @return Nothing; `InvalidInput` for an unknown dataset, a mode it does not record (TUM has
///         no stereo pairs, EuRoC no depth maps), a negative `first` or `count`, settings
///         out of range, or a vocabulary missing while relocalising or given while not.
[[nodiscard]] foundations::Result<void> check(const Request& request);

/// @brief Run a sequence through the pipeline: the run, not yet written, as Python's program
///        builds it before writing (`slam_it.orchestration.vo.main`). The program records its
///        own process in it (slam_it/eval/budgets.hpp) and writes it to `request.out`.
/// @param request The request.
/// @return The run; the check's, a reader's or the pipeline's error.
[[nodiscard]] foundations::Result<eval::runs::Run> run(const Request& request);

}  // namespace slam_it::orchestration::vo
