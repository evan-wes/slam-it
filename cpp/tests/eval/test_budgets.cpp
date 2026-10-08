/// @file
/// @brief Native tests for a process's peak memory (slamlab/eval/budgets.hpp), as Python's
///        `tests/eval/test_budgets.py` reads it.
#include <filesystem>
#include <fstream>
#include <string>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slamlab/eval/budgets.hpp"

namespace {

namespace budgets = slamlab::eval::budgets;
namespace fs = std::filesystem;
using slamlab::testing::expect;

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
  const slamlab::testing::ScratchDir scratch("slamlab_test_budgets");
  fs::create_directories(scratch.path());
  const fs::path status = scratch.path() / "status";
  write(status,
        "Name:\tslamlab_vo\nVmPeak:\t  912340 kB\nVmHWM:\t   47248 kB\nVmRSS:\t   41000 kB\n");
  expect(budgets::peak_memory(status.string()) == 47248 * 1024,
         "the resident set's high-water mark is read, kilobytes to bytes", failures);
  for (const char* text : {"Name:\tx\nVmRSS:\t 41000 kB\n", "VmHWM:\t 47248 MB\n", ""}) {
    write(status, text);
    expect(!budgets::peak_memory(status.string()).has_value(),
           "no VmHWM line in kilobytes: nothing", failures);
  }
  expect(!budgets::peak_memory((scratch.path() / "no_such_file").string()).has_value(),
         "no status file: nothing", failures);
#ifdef __linux__
  const auto own = budgets::peak_memory();
  expect(own.has_value() && *own > 0, "this process's own peak is read", failures);
#endif
  return slamlab::testing::finish("eval/budgets", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("eval/budgets", run); }
