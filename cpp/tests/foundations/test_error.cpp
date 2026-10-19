/// @file
/// @brief Native tests for the error contract in slam_it/foundations/error.hpp.
#include <iostream>

#include "slam_it/foundations/error.hpp"

namespace {

using slam_it::foundations::ErrorKind;
using slam_it::foundations::fail;
using slam_it::foundations::Result;

/// @brief A fallible function for the test: halves even numbers, rejects odd ones.
/// @param n The number to halve.
/// @return Half of `n`, or an InvalidInput error if `n` is odd.
Result<int> half(int n) {
  if (n % 2 != 0) {
    return fail(ErrorKind::kInvalidInput, "odd number");
  }
  return n / 2;
}

}  // namespace

int main() {
  const Result<int> ok = half(4);
  const Result<int> bad = half(3);
  const bool passed = ok.has_value() && *ok == 2 && !bad.has_value() &&
                      bad.error().kind == ErrorKind::kInvalidInput &&
                      bad.error().message == "odd number";
  if (!passed) {
    std::cerr << "FAILED: Result<int> success and failure paths\n";
    return 1;
  }
  std::cout << "foundations/error: ok\n";
  return 0;
}
