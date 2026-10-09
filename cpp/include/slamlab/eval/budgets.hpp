/// @file
/// @brief Compute budgets: the C++ mirror of `slamlab.eval.budgets` where a native program
///        needs it: what it records in its run of its own process (slamlab/eval/runs.hpp),
///        its peak memory, the CPUs it may run on and its memory cap.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/// @brief Compute budgets.
namespace slamlab::eval::budgets {

/// @brief Where Linux states the calling process's memory and the CPUs it may run on
///        (`slamlab.eval.budgets.STATUS`).
inline constexpr const char* kStatus = "/proc/self/status";

/// @brief Where Linux names the calling process's control group
///        (`slamlab.eval.budgets.CGROUP`).
inline constexpr const char* kCgroup = "/proc/self/cgroup";

/// @brief Where the control groups (version 2) are mounted (`slamlab.eval.budgets.CGROUP_ROOT`).
inline constexpr const char* kCgroupRoot = "/sys/fs/cgroup";

/// @brief The calling process's largest resident set so far, in bytes
///        (`slamlab.eval.budgets.peak_memory`): Linux's `VmHWM`, which starts again at `exec`,
///        so it does not count the process that spawned this one.
/// @param status The status file to read.
/// @return The bytes, or nothing where the system does not say: no such file, or no `VmHWM`
///         line in kilobytes.
[[nodiscard]] std::optional<std::int64_t> peak_memory(const std::string& status = kStatus);

/// @brief The CPUs the calling process may run on (`slamlab.eval.budgets.allowed_cpus`):
///        Linux's `Cpus_allowed_list`, which `taskset` sets.
/// @param status The status file to read.
/// @return The CPUs, ascending, or nothing where the system does not say: no such file, or no
///         `Cpus_allowed_list` line that reads as a CPU list.
[[nodiscard]] std::optional<std::vector<std::int64_t>> allowed_cpus(
    const std::string& status = kStatus);

/// @brief The calling process's memory cap (`slamlab.eval.budgets.memory_cap`): its control
///        group's `memory.max`, which a budget's `systemd-run --scope -p MemoryMax` sets. A cap
///        on an enclosing group only is not reported.
/// @param cgroup The file naming the process's control group.
/// @param root Where the control groups are mounted.
/// @return The bytes, or nothing where there is no cap (`max`) or the system does not say: no
///         such file, no version-2 group (a `0::` line), or no `memory.max` in it.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): two paths, as Python's (cgroup, root)
[[nodiscard]] std::optional<std::int64_t> memory_cap(const std::string& cgroup = kCgroup,
                                                     const std::string& root = kCgroupRoot);

}  // namespace slamlab::eval::budgets
