/// @file
/// @brief Native tests for PCG32 (slamlab/foundations/rng.hpp): the reference library's
///        published output, and the draws the Python reference makes from the same seed.
#include <array>
#include <cstdint>
#include <vector>

#include "slamlab/foundations/rng.hpp"
#include "test_support.hpp"

namespace {

using slamlab::foundations::rng::Pcg32;
using slamlab::testing::expect;

/// @brief `pcg32_srandom_r(&rng, 42, 54)` then six `pcg32_random_r`, from the reference
///        library's demo (and the Rust `rand_pcg` reference test).
constexpr std::array<std::uint32_t, 6> kReference = {0xa15c02b7U, 0x7b47f409U, 0xba1d3330U,
                                                     0x83d2f293U, 0xbfa4784bU, 0xcbed606eU};

}  // namespace

int main() {
  int failures = 0;
  Pcg32 reference(42U, 54U);
  bool matches = true;
  for (const std::uint32_t expected : kReference) {
    matches = matches && reference.next_u32() == expected;
  }
  expect(matches, "next_u32 matches the reference library", failures);

  // The same draws as tests/foundations/test_rng.py::test_samples_match_the_cpp_core.
  Pcg32 rng(42U, 54U);
  const auto first = rng.sample(100U, 8U);
  const auto whole = rng.sample(5U, 5U);
  const auto sparse = rng.sample(1000U, 3U);
  expect(first.has_value() && *first == std::vector<std::uint32_t>{45, 77, 54, 19, 62, 58, 44, 99},
         "sample(100, 8) matches the Python reference", failures);
  expect(whole.has_value() && *whole == std::vector<std::uint32_t>{0, 1, 2, 3, 4},
         "sample(5, 5) is every index", failures);
  expect(sparse.has_value() && *sparse == std::vector<std::uint32_t>{71, 372, 987},
         "sample(1000, 3) matches the Python reference", failures);

  Pcg32 checks(7U, 0U);
  expect(!checks.sample(3U, 4U).has_value(), "sample rejects count > population", failures);
  expect(!checks.bounded(0U).has_value(), "bounded rejects a bound of 0", failures);
  const auto one = checks.bounded(1U);
  expect(one.has_value() && *one == 0U, "bounded(1) is 0", failures);
  return slamlab::testing::finish("foundations/rng", failures);
}
