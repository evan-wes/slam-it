/// @file
/// @brief Shared helpers for the native rung-0 tests (no test framework yet).
#pragma once

#include <iostream>

/// @brief Helpers for slamlab's native tests.
namespace slamlab::testing {

/// @brief Count and report a failed check.
/// @param ok Whether the check passed.
/// @param what Description of the check, printed on failure.
/// @param failures Running failure count, incremented on failure.
inline void expect(bool ok, const char* what, int& failures) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

/// @brief Report the outcome of a test program.
/// @param name The test's name, printed on success.
/// @param failures The number of failed checks.
/// @return The process exit code: 0 on success, 1 on any failure.
inline int finish(const char* name, int failures) {
  if (failures == 0) {
    std::cout << name << ": ok\n";
  }
  return failures == 0 ? 0 : 1;
}

}  // namespace slamlab::testing
