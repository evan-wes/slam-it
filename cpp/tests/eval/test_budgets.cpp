/// @file
/// @brief Native tests for what a program reads of its own process (slam_it/eval/budgets.hpp),
///        as Python's `tests/eval/test_budgets.py` reads it.
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/eval/budgets.hpp"

namespace {

namespace budgets = slam_it::eval::budgets;
namespace fs = std::filesystem;
using slam_it::testing::expect;

/// @brief Write a status file.
/// @param path The file.
/// @param text Its text.
void write(const fs::path& path, const std::string& text) {
  std::ofstream file(path);
  file << text;
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const slam_it::testing::ScratchDir scratch("slam_it_test_budgets");
  fs::create_directories(scratch.path());
  const fs::path status = scratch.path() / "status";
  write(status,
        "Name:\tslam_it_vo\nVmPeak:\t  912340 kB\nVmHWM:\t   47248 kB\nVmRSS:\t   41000 kB\n");
  expect(budgets::peak_memory(status.string()) == 47248 * 1024,
         "the resident set's high-water mark is read, kilobytes to bytes", failures);
  for (const char* text : {"Name:\tx\nVmRSS:\t 41000 kB\n", "VmHWM:\t 47248 MB\n", ""}) {
    write(status, text);
    expect(!budgets::peak_memory(status.string()).has_value(),
           "no VmHWM line in kilobytes: nothing", failures);
  }
  expect(!budgets::peak_memory((scratch.path() / "no_such_file").string()).has_value(),
         "no status file: nothing", failures);
  write(status, "Name:\tslam_it_vo\nCpus_allowed:\tf0\nCpus_allowed_list:\t4-7\n");
  expect(budgets::allowed_cpus(status.string()) == std::vector<std::int64_t>{4, 5, 6, 7},
         "the CPUs allowed are read", failures);
  write(status, "Cpus_allowed_list:\t0-3,8,10-11\n");
  expect(budgets::allowed_cpus(status.string()) == std::vector<std::int64_t>{0, 1, 2, 3, 8, 10, 11},
         "a CPU list with single CPUs is read", failures);
  for (const char* text :
       {"Name:\tx\n", "Cpus_allowed_list:\t7-4\n", "Cpus_allowed_list:\tf0\n",
        "Cpus_allowed_list:\t1-\n", "Cpus_allowed_list:\t0-3,7-4\n", "Cpus_allowed_list:\t1_0\n"}) {
    write(status, text);
    expect(!budgets::allowed_cpus(status.string()).has_value(),
           "no CPU list that reads as one: nothing", failures);
  }
  const fs::path cgroup = scratch.path() / "cgroup";
  const fs::path group = scratch.path() / "root" / "app.slice" / "run-1.scope";
  fs::create_directories(group);
  write(cgroup, "0::/app.slice/run-1.scope\n");
  const std::string root = (scratch.path() / "root").string();
  write(group / "memory.max", "8589934592\n");
  expect(budgets::memory_cap(cgroup.string(), root) == 8589934592, "the group's memory cap is read",
         failures);
  write(group / "memory.max", "max\n");
  expect(!budgets::memory_cap(cgroup.string(), root).has_value(), "no cap: nothing", failures);
  write(cgroup, "1:name=systemd:/app.slice\n");
  expect(!budgets::memory_cap(cgroup.string(), root).has_value(), "no version-2 group: nothing",
         failures);
#ifdef __linux__
  const auto own = budgets::peak_memory();
  expect(own.has_value() && *own > 0, "this process's own peak is read", failures);
  const auto cpus = budgets::allowed_cpus();
  expect(cpus.has_value() && !cpus->empty(), "this process's own CPUs are read", failures);
#endif
  return slam_it::testing::finish("eval/budgets", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("eval/budgets", run); }
