/// @file
/// @brief Compute budgets: the C++ mirror of `slamlab.eval.budgets` where a native program
///        needs it, its own peak memory, which it records in its run (slamlab/eval/runs.hpp).
#pragma once

#include <cstdint>
#include <optional>
#include <string>

/// @brief Compute budgets.
namespace slamlab::eval::budgets {

/// @brief Where Linux states the calling process's memory (`slamlab.eval.budgets.STATUS`).
inline constexpr const char* kStatus = "/proc/self/status";

/// @brief The calling process's largest resident set so far, in bytes
///        (`slamlab.eval.budgets.peak_memory`): Linux's `VmHWM`, which starts again at `exec`,
///        so it does not count the process that spawned this one.
/// @param status The status file to read.
/// @return The bytes, or nothing where the system does not say: no such file, or no `VmHWM`
///         line in kilobytes.
[[nodiscard]] std::optional<std::int64_t> peak_memory(const std::string& status = kStatus);

}  // namespace slamlab::eval::budgets
