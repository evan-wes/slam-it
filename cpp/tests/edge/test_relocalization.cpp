/// @file
/// @brief Native tests for relocalisation (slam_it/edge/relocalization.hpp) on the exact
///        scene's map, its keyframes' words those of a vocabulary of one word: a frame of the
///        scene is placed at its pose, through a candidate, with every point; a frame with no
///        features is not; and the RANSAC budget is MLPnP's.
#include <Eigen/Core>
#include <cstdint>
#include <utility>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/database.hpp"
#include "slam_it/edge/relocalization.hpp"

namespace {

namespace edge = slam_it::edge;
namespace scene = slam_it::testing::scene;
namespace vocabulary = slam_it::foundations::vocabulary;
using slam_it::testing::expect;

/// @brief The scene's map's database: each keyframe's words, from its frame's descriptors.
/// @param words The vocabulary.
/// @return The database, or the first insertion's error.
slam_it::foundations::Result<edge::Database> database_of(const vocabulary::Vocabulary& words) {
  edge::Database out;
  for (std::size_t k = 0; k < scene::kKeyframes; ++k) {
    const edge::Frame frame = scene::frame(k, true);
    if (auto ok = edge::add(out, words, static_cast<std::int64_t>(k), frame.features.descriptors);
        !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  return out;
}

/// @brief Check that a frame of the scene is placed at its pose, and an empty frame is not.
/// @param failures Running failure count.
void check_placed(int& failures) {
  const auto m = scene::map();
  const vocabulary::Vocabulary words = scene::one_word();
  const auto db = m ? database_of(words) : std::unexpected(m.error());
  if (!db) {
    expect(false, "the scene's map and its database are built", failures);
    return;
  }
  const Eigen::Matrix4d truth = scene::moved(0.15);
  const auto found = edge::relocalize(*m, *db, words, scene::camera(),
                                      scene::frame_at(truth, false), edge::Options{});
  expect(found.has_value() && found->tracked && found->summary.placed &&
             found->summary.words == 1 && found->summary.sharing == 3 &&
             !found->summary.candidates.empty(),
         "a frame of the scene is placed, through the keyframes sharing its word", failures);
  if (found && found->tracked) {
    expect((found->tracked->pose - truth).cwiseAbs().maxCoeff() < 1e-6,
           "at its pose, to the solver's precision", failures);
    expect(edge::inliers(*found->tracked) == static_cast<std::int64_t>(scene::kPoints),
           "with every point", failures);
    const edge::CandidateSummary& last = found->summary.candidates.back();
    expect(std::cmp_equal(last.matches, scene::kPoints) && last.inliers && last.found &&
               std::cmp_equal(*last.found, scene::kPoints),
           "the placing candidate matched every point by word and kept them", failures);
  }
  const edge::Frame empty =
      edge::make_frame(0.0, {.keypoints = {}, .descriptors = {}, .scale_factor = 1.2, .levels = 8},
                       {}, {}, {})
          .value();
  const auto none = edge::relocalize(*m, *db, words, scene::camera(), empty, edge::Options{});
  expect(none.has_value() && !none->tracked && !none->summary.placed && none->summary.words == 0 &&
             none->summary.candidates.empty(),
         "a frame with no features is not placed, and no candidate is tried", failures);
}

/// @brief Check the RANSAC budget: MLPnP's, 35 samples for 100 matches at the defaults and 14
///        at 15 (the review panel's check of ORB-SLAM3's `SetRansacParameters`); and off them,
///        each setting read (Python's `ransac_budget` gives the same): an inlier ratio of 0.4, a
///        minimum of 12 and a confidence of 0.95 need 40 of 100 in 46 samples, 20 if capped
///        there, and every one of 12, in one sample.
/// @param failures Running failure count.
void check_budget(int& failures) {
  using Budget = std::pair<std::int64_t, std::int64_t>;
  const edge::Options options;
  expect(edge::ransac_budget(100, options) == Budget{50, 35},
         "100 matches: 50 inliers needed, 35 samples", failures);
  expect(edge::ransac_budget(15, options).second == 14, "15 matches: 14 samples", failures);
  edge::Options moved = options;
  moved.reloc_inlier_ratio = 0.4;
  moved.reloc_min_inliers = 12;
  moved.reloc_confidence = 0.95;
  expect(edge::ransac_budget(100, moved) == Budget{40, 46},
         "moved: 100 matches need 40 inliers, in 46 samples", failures);
  expect(edge::ransac_budget(12, moved) == Budget{12, 1},
         "moved: 12 matches need all 12, in one sample", failures);
  moved.reloc_ransac_iterations = 20;
  expect(edge::ransac_budget(100, moved) == Budget{40, 20}, "moved: the samples capped at 20",
         failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_placed(failures);
  check_budget(failures);
  return slam_it::testing::finish("edge/relocalization", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/relocalization", run); }
