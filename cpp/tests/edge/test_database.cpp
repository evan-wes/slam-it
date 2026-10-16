/// @file
/// @brief Native tests for the keyframe database (slam_it/edge/database.hpp) on the exact
///        scene's map: the inverted index as keyframes enter and leave it, relocalisation's
///        candidates without an erased keyframe, and the refusals (a keyframe given twice, one
///        erased that is not there, a malformed vector).
#include <algorithm>
#include <cstdint>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/database.hpp"

namespace {

namespace edge = slam_it::edge;
namespace scene = slam_it::testing::scene;
using slam_it::foundations::vocabulary::BowVector;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief The scene's three keyframes' words: keyframe k holds words k, k + 1 and 9, so
///        neighbours share a word and every keyframe shares word 9.
/// @param k The keyframe.
/// @return Its words, one node per feature.
edge::Words words_of(std::int64_t k) {
  return {.vector = {.ids = {k, k + 1, 9}, .values = {0.25, 0.25, 0.5}},
          .nodes = std::vector<std::int64_t>(scene::kPoints, 0)};
}

/// @brief Whether a word's keyframes are exactly these.
/// @param db The database.
/// @param word The word.
/// @param expected Its keyframes, ascending; none for a word no keyframe holds.
/// @return Whether they are.
bool holders(const edge::Database& db, std::int64_t word,
             const std::vector<std::int64_t>& expected) {
  const auto found = db.inverted.find(word);
  return expected.empty() ? found == db.inverted.end()
                          : found != db.inverted.end() && found->second == expected;
}

/// @brief Check the index as the scene's keyframes enter it and one leaves it, as culling
///        removes it: no word lists it, and no candidate is it.
/// @param failures Running failure count.
void check_erase(int& failures) {
  const auto m = scene::map();
  if (!m) {
    expect(false, "the scene's map is built", failures);
    return;
  }
  edge::Database db;
  bool inserted = true;
  for (std::int64_t k = 0; k < 3; ++k) {
    inserted = inserted && edge::insert(db, k, words_of(k)).has_value();
  }
  expect(inserted && holders(db, 1, {0, 1}) && holders(db, 9, {0, 1, 2}),
         "each word lists its keyframes, ascending", failures);
  edge::Options options;
  options.covisible_points = 1;
  const BowVector frame{.ids = {1, 2, 9}, .values = {0.25, 0.25, 0.5}};
  const auto before = edge::candidates(db, *m, frame, options);
  expect(before.has_value() && before->sharing == 3, "every keyframe shares a word with the frame",
         failures);
  expect(edge::erase(db, 1).has_value(), "a keyframe in the database is erased", failures);
  expect(holders(db, 1, {0}) && holders(db, 2, {2}) && holders(db, 9, {0, 2}) &&
             !db.keyframes.contains(1),
         "an erased keyframe leaves every word's list", failures);
  const auto after = edge::candidates(db, *m, frame, options);
  expect(after.has_value() && after->sharing == 2 &&
             std::ranges::find(after->keyframes, 1) == after->keyframes.end() &&
             !after->keyframes.empty(),
         "an erased keyframe is never a candidate", failures);
  expect(is_invalid_input(edge::erase(db, 1)), "erasing it again is refused", failures);
  expect(edge::erase(db, 0).has_value() && edge::erase(db, 2).has_value() && db.inverted.empty() &&
             db.keyframes.empty(),
         "the last keyframes out leave an empty index", failures);
  const auto none = edge::candidates(db, *m, frame, options);
  expect(none.has_value() && none->keyframes.empty() && none->sharing == 0,
         "an empty database has no candidates", failures);
}

/// @brief Check the refusals: a keyframe inserted twice, a malformed frame vector.
/// @param failures Running failure count.
void check_refusals(int& failures) {
  const auto m = scene::map();
  edge::Database db;
  expect(edge::insert(db, 0, words_of(0)).has_value() &&
             is_invalid_input(edge::insert(db, 0, words_of(0))) && holders(db, 0, {0}),
         "a keyframe inserted twice is refused, the index unchanged", failures);
  const BowVector unsorted{.ids = {9, 1}, .values = {0.5, 0.5}};
  expect(m.has_value() && is_invalid_input(edge::candidates(db, *m, unsorted, edge::Options{})),
         "a frame vector whose words do not ascend is refused", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_erase(failures);
  check_refusals(failures);
  return slam_it::testing::finish("edge/database", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/database", run); }
