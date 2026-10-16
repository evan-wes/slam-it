/// @file
/// @brief Native tests for the vocabulary (slam_it/foundations/vocabulary.hpp), against the
///        parity fixture `tests/fixtures/foundations/vocabulary.json` that the Python and Rust
///        tests share (quantising, word vectors and scores, exactly), and the core's checks: a
///        malformed tree, a word outside the vocabulary, a malformed vector.
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "fixture_support.hpp"
#include "slam_it/foundations/vocabulary.hpp"
#include "test_support.hpp"

namespace {

namespace vocabulary = slam_it::foundations::vocabulary;
namespace orb = slam_it::foundations::orb;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief The fixture's tree.
/// @param tree Its JSON.
/// @return The vocabulary.
vocabulary::Vocabulary tree_of(const nlohmann::json& tree) {
  vocabulary::Vocabulary out{.k = tree["k"].get<std::int64_t>(),
                             .levels = tree["levels"].get<std::int64_t>(),
                             .descriptors = tree["descriptors"].get<std::vector<orb::Descriptor>>(),
                             .parents = tree["parents"].get<std::vector<std::int32_t>>(),
                             .children = {},
                             .words = tree["words"].get<std::vector<std::int32_t>>(),
                             .weights = tree["weights"].get<std::vector<double>>()};
  for (const auto& c : tree["children"]) {
    out.children.push_back({c[0].get<std::int32_t>(), c[1].get<std::int32_t>()});
  }
  return out;
}

/// @brief Check quantising, the vectors and the scores against the fixture, exactly.
/// @param fixture The fixture.
/// @param failures Running failure count.
void check_fixture(const nlohmann::json& fixture, int& failures) {
  const vocabulary::Vocabulary tree = tree_of(fixture["tree"]);
  expect(vocabulary::check(tree).has_value(), "the fixture's tree is well formed", failures);
  const auto& q = fixture["quantize"];
  const vocabulary::Quantized found =
      vocabulary::quantize(tree, q["descriptors"].get<std::vector<orb::Descriptor>>());
  expect(found.words == q["words"].get<std::vector<std::int64_t>>(),
         "quantize's words match the fixture", failures);
  expect(found.nodes == q["nodes"].get<std::vector<std::int64_t>>(),
         "quantize's direct nodes match the fixture", failures);
  std::vector<vocabulary::BowVector> vectors;
  for (const auto& c : fixture["bow"]) {
    const auto vector = vocabulary::bow(tree, c["words"].get<std::vector<std::int64_t>>());
    expect(vector.has_value() && vector->ids == c["ids"].get<std::vector<std::int64_t>>() &&
               vector->values == c["values"].get<std::vector<double>>(),
           "bow matches the fixture", failures);
    vectors.push_back(vector.value_or(vocabulary::BowVector{}));
  }
  for (const auto& c : fixture["score"]) {
    const auto score = vocabulary::score(vectors.at(c["first"].get<std::size_t>()),
                                         vectors.at(c["second"].get<std::size_t>()));
    expect(score.has_value() && *score == c["score"].get<double>(), "score matches the fixture",
           failures);
  }
}

/// @brief Check that a malformed tree is refused, without reading out of range: a parent after
///        its child, children past the int32 range, a leaf too deep, words out of node order;
///        and that a leaf's first child is not read.
/// @param fixture The fixture.
/// @param failures Running failure count.
void check_malformed(const nlohmann::json& fixture, int& failures) {
  const vocabulary::Vocabulary tree = tree_of(fixture["tree"]);
  vocabulary::Vocabulary bad = tree;
  bad.parents[5] = -7;
  expect(is_invalid_input(vocabulary::check(bad)), "a negative parent is refused", failures);
  bad = tree;
  bad.children[1] = {std::numeric_limits<std::int32_t>::max(), 3};
  expect(is_invalid_input(vocabulary::check(bad)), "children past the int32 range are refused",
         failures);
  bad = tree;
  bad.children[3] = {2'000'000'000, 0};  // node 3 is a leaf
  expect(vocabulary::check(bad).has_value(), "a leaf's first child is not read", failures);
  bad = tree;
  bad.levels = 2;
  expect(is_invalid_input(vocabulary::check(bad)), "a leaf deeper than the levels is refused",
         failures);
  bad = tree;
  std::swap(bad.words[3], bad.words[8]);
  expect(is_invalid_input(vocabulary::check(bad)), "words out of node order are refused", failures);
  bad = tree;
  bad.weights.pop_back();
  expect(is_invalid_input(vocabulary::check(bad)), "a word without a weight is refused", failures);
}

/// @brief Check the refusals of a word outside the vocabulary and of malformed vectors.
/// @param fixture The fixture.
/// @param failures Running failure count.
void check_refusals(const nlohmann::json& fixture, int& failures) {
  const vocabulary::Vocabulary tree = tree_of(fixture["tree"]);
  const std::vector<std::int64_t> past{static_cast<std::int64_t>(tree.weights.size())};
  expect(is_invalid_input(vocabulary::bow(tree, past)), "a word past the last is refused",
         failures);
  const vocabulary::BowVector fine{.ids = {1, 4}, .values = {0.5, 0.5}};
  const vocabulary::BowVector unsorted{.ids = {4, 1}, .values = {0.5, 0.5}};
  const vocabulary::BowVector short_values{.ids = {1, 4}, .values = {1.0}};
  const vocabulary::BowVector not_finite{.ids = {1},
                                         .values = {std::numeric_limits<double>::infinity()}};
  for (const auto* bad : {&unsorted, &short_values, &not_finite}) {
    expect(is_invalid_input(vocabulary::score(fine, *bad)) &&
               is_invalid_input(vocabulary::score(*bad, fine)),
           "a malformed vector is refused", failures);
  }
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slam_it::testing::load_fixture("foundations/vocabulary.json");
  check_fixture(fixture, failures);
  check_malformed(fixture, failures);
  check_refusals(fixture, failures);
  return slam_it::testing::finish("foundations/vocabulary", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("foundations/vocabulary", run); }
