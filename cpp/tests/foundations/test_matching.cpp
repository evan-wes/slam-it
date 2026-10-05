/// @file
/// @brief Native tests for descriptor matching (slamlab/foundations/matching.hpp), against the
///        parity fixture `tests/fixtures/foundations/matching.json` that the Python and Rust
///        tests share, and the core's own input checks.
#include <cstddef>
#include <cstdint>
#include <vector>

#include "fixture_support.hpp"
#include "slamlab/foundations/matching.hpp"
#include "test_support.hpp"

namespace {

namespace matching = slamlab::foundations::matching;
namespace orb = slamlab::foundations::orb;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;

/// @brief Check every fixture case.
/// @param failures Running failure count.
void check_fixture(int& failures) {
  const nlohmann::json functions =
      slamlab::testing::load_fixture("foundations/matching.json")["functions"];
  for (const auto& c : functions["distances"]["cases"]) {
    const auto a = c["input"][0].get<std::vector<orb::Descriptor>>();
    const auto b = c["input"][1].get<std::vector<orb::Descriptor>>();
    const auto expected = slamlab::testing::flatten_rows<std::int64_t>(c["expected"]);
    const auto got = matching::distances(a, b);
    expect(got.has_value() && *got == expected, "distances matches the fixture", failures);
  }
  for (const auto& c : functions["match"]["cases"]) {
    const auto& input = c["input"];
    const auto& settings = input[4];
    const matching::Options options{.max_distance = settings["max_distance"].get<std::int64_t>(),
                                    .ratio = settings["ratio"].get<double>(),
                                    .check_orientation = settings["check_orientation"].get<bool>()};
    const auto got = matching::match(input[0].get<std::vector<orb::Descriptor>>(),
                                     input[1].get<std::vector<orb::Descriptor>>(),
                                     input[2].get<std::vector<std::int64_t>>(),
                                     input[3].get<std::vector<std::int64_t>>(), options);
    bool ok = got.has_value() && got->size() == c["expected"]["pairs"].size();
    for (std::size_t k = 0; ok && k < got->size(); ++k) {
      const matching::Match& m = (*got)[k];
      ok = m.first == c["expected"]["pairs"][k][0].get<std::int64_t>() &&
           m.second == c["expected"]["pairs"][k][1].get<std::int64_t>() &&
           m.distance == c["expected"]["distances"][k].get<std::int64_t>();
    }
    expect(ok, "match matches the fixture", failures);
  }
}

/// @brief Check that bad filters and angles are `InvalidInput`.
/// @param failures Running failure count.
void check_rejections(int& failures) {
  expect(is_invalid_input(matching::check_options({.max_distance = 257})),
         "check_options rejects a distance above 256", failures);
  expect(is_invalid_input(matching::check_options({.ratio = 0.0})),
         "check_options rejects a zero ratio", failures);
  const std::vector<orb::Descriptor> set(3);
  const std::vector<std::int64_t> bins{0, 1, 2};
  expect(is_invalid_input(
             matching::match(set, set, std::vector<std::int64_t>{0, 1}, bins, matching::Options{})),
         "match rejects too few angles", failures);
  expect(is_invalid_input(matching::match(set, set, bins, std::vector<std::int64_t>{0, 1, 30},
                                          matching::Options{})),
         "match rejects an angle bin out of range", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_fixture(failures);
  check_rejections(failures);
  return slamlab::testing::finish("foundations/matching", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("foundations/matching", run); }
