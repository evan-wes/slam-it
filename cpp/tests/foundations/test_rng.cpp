/// @file
/// @brief Native tests for PCG32 (slam_it_all/foundations/rng.hpp), against the parity fixture
///        `tests/fixtures/foundations/rng.json` that the Python and Rust tests share: the
///        reference library's published output, and the Floyd samples every backend draws.
#include <cstdint>
#include <vector>

#include "fixture_support.hpp"
#include "slam_it_all/foundations/rng.hpp"
#include "test_support.hpp"

namespace {

using slam_it_all::foundations::rng::Pcg32;
using slam_it_all::testing::expect;

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slam_it_all::testing::load_fixture("foundations/rng.json");

  const auto& reference = fixture["reference"];
  Pcg32 published(reference["seed"].get<std::uint64_t>(), reference["stream"].get<std::uint64_t>());
  bool matches = true;
  for (const auto& expected : reference["next_u32"]) {
    matches = matches && published.next_u32() == expected.get<std::uint32_t>();
  }
  expect(matches, "next_u32 matches the reference library", failures);

  const auto& samples = fixture["samples"];
  Pcg32 rng(samples["seed"].get<std::uint64_t>(), samples["stream"].get<std::uint64_t>());
  for (const auto& draw : samples["draws"]) {
    const auto drawn =
        rng.sample(draw["population"].get<std::uint32_t>(), draw["count"].get<std::uint32_t>());
    expect(drawn.has_value() && *drawn == draw["expected"].get<std::vector<std::uint32_t>>(),
           "sample matches the fixture", failures);
  }

  Pcg32 checks(7U, 0U);
  expect(!checks.sample(3U, 4U).has_value(), "sample rejects count > population", failures);
  expect(!checks.bounded(0U).has_value(), "bounded rejects a bound of 0", failures);
  const auto one = checks.bounded(1U);
  expect(one.has_value() && *one == 0U, "bounded(1) is 0", failures);
  return slam_it_all::testing::finish("foundations/rng", failures);
}

}  // namespace

int main() { return slam_it_all::testing::guarded("foundations/rng", run); }
