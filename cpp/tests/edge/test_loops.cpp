/// @file
/// @brief Native tests for loop closing (slam_it/edge/loops.hpp) on the exact scene's map:
///        similarities' scale and rigid part, the RANSAC budget, the keyframes a loop holds, a
///        keyframe's points found through its own pose, a small map not searched, global BA
///        leaving an exact map where it is, a merge's priority over a loop, and the refusals.
///        Python's tests hold every stage to the reference on the loop room
///        (tests/edge/test_loops.py).
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/database.hpp"
#include "slam_it/edge/loops.hpp"

namespace {

namespace edge = slam_it::edge;
namespace loops = slam_it::edge::loops;
namespace scene = slam_it::testing::scene;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief Check a similarity's scale and rigid part.
/// @param failures Running failure count.
void check_similarities(int& failures) {
  Eigen::Matrix4d rigid = Eigen::Matrix4d::Identity();
  rigid.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.0, 1.0, 0.0)).toRotationMatrix();
  rigid.topRightCorner<3, 1>() = Eigen::Vector3d(0.1, -0.2, 0.3);
  Eigen::Matrix4d scaled = rigid;
  scaled.topRows<3>() *= 1.7;
  expect(std::abs(loops::scale_of(scaled) - 1.7) < 1e-15, "the scale is a column's length",
         failures);
  expect(loops::rigid_of(scaled).isApprox(rigid, 1e-15), "the rigid part divides it out", failures);
}

/// @brief Check the RANSAC budget, ORB-SLAM's SetRansacParameters(0.99, 15, 300).
/// @param failures Running failure count.
void check_budget(int& failures) {
  const edge::Options options;
  expect(loops::ransac_budget(15, options) == 1 && loops::ransac_budget(10, options) == 1,
         "every match an inlier: one sample", failures);
  expect(loops::ransac_budget(30, options) == 35, "half the matches inliers: 35 samples", failures);
  expect(loops::ransac_budget(1000, options) == 300, "at most 300 samples", failures);
}

/// @brief Check the keyframes a loop being confirmed holds, and the outcomes' names.
/// @param failures Running failure count.
void check_held(int& failures) {
  loops::Pending pending;
  pending.matched = 9;
  pending.last = 4;
  expect(loops::held(pending) == std::vector<std::int64_t>{4, 9},
         "the matched keyframe and the last, ascending", failures);
  pending.last = 9;
  expect(loops::held(pending) == std::vector<std::int64_t>{9}, "once when they are one", failures);
  expect(std::string(loops::name(loops::Outcome::kSkipped)) == "skipped" &&
             std::string(loops::name(loops::Outcome::kDropped)) == "dropped" &&
             std::string(loops::name(loops::Outcome::kMerged)) == "merged",
         "the outcomes are named as Python's", failures);
}

/// @brief Check a merge's priority over a loop closed on its keyframe.
/// @param failures Running failure count.
void check_merge_first(int& failures) {
  loops::Detection closed{.pending = std::nullopt, .loop = loops::Loop{}, .summary = {}};
  closed.summary.outcome = loops::Outcome::kClosed;
  loops::MergeDetection merged{
      .pending = std::nullopt, .merge = loops::Merge{}, .summary = {}, .maps = {0}};
  const loops::Detection first = loops::merge_first(closed, merged);
  expect(!first.loop && !first.pending && first.summary.outcome == loops::Outcome::kMerged,
         "a loop closed where a merge was found is dropped, its outcome merged", failures);
  merged.merge.reset();
  const loops::Detection kept = loops::merge_first(closed, merged);
  expect(kept.loop.has_value() && kept.summary.outcome == loops::Outcome::kClosed,
         "with no merge found, the loop stands", failures);
  const loops::Detection carried{.pending = loops::Pending{}, .loop = std::nullopt, .summary = {}};
  merged.merge = loops::Merge{};
  expect(loops::merge_first(carried, merged).pending.has_value(),
         "a loop being confirmed is kept beside a merge", failures);
}

/// @brief The exact scene's map, its building checked.
/// @param failures Running failure count.
/// @return The map, if it was built.
std::optional<edge::Map> scene_map(int& failures) {
  auto m = scene::map();
  expect(m.has_value(), "the scene's map is built", failures);
  return m ? std::optional<edge::Map>(std::move(*m)) : std::nullopt;
}

/// @brief Check the searches and global BA on the exact scene.
/// @param m The scene's map.
/// @param failures Running failure count.
void check_scene(const edge::Map& m, int& failures) {
  const edge::Camera camera = scene::camera();
  const edge::Options options;
  std::vector<std::int64_t> rows(scene::kPoints);
  for (std::size_t p = 0; p < rows.size(); ++p) {
    rows[p] = static_cast<std::int64_t>(p);
  }
  // From the other keyframes, the points farther than from the first are past the farthest
  // distance they are matchable from (no margin, as ORB-SLAM's similarity search), and not
  // searched; every match made is the keypoint's own point.
  for (std::int64_t k = 0; std::cmp_less(k, scene::kKeyframes); ++k) {
    const auto found = loops::search_by_similarity(m, camera, k, m.kf_pose[k], rows,
                                                   {.radius = 5.0, .distance = 50}, options);
    bool own = found.has_value();
    std::size_t matched = 0;
    for (std::size_t j = 0; own && j < found->size(); ++j) {
      const std::int64_t id = m.kp_point[static_cast<std::size_t>(m.kf_start[k]) + j];
      own = (*found)[j] < 0 || (*found)[j] == id;
      matched += (*found)[j] >= 0 ? 1 : 0;
    }
    expect(own && matched > 0, "a keyframe's points come back to their own keypoints", failures);
    expect(k > 0 || matched == scene::kPoints, "all of them, from the keyframe that made them",
           failures);
  }
  edge::Map adjusted = m;
  const auto solved = loops::global_bundle_adjust(adjusted, camera, options);
  bool kept = solved.has_value() && solved->final_error < 1e-12 &&
              adjusted.kf_pose[0].isApprox(m.kf_pose[0], 1e-12);
  for (std::size_t k = 0; kept && k < scene::kKeyframes; ++k) {
    kept = (adjusted.kf_pose[k] - m.kf_pose[k]).cwiseAbs().maxCoeff() < 1e-9;
  }
  expect(kept, "global BA leaves an exact map where it is", failures);
}

/// @brief Check detection skipping a small map, and the refusals.
/// @param m The scene's map.
/// @param failures Running failure count.
void check_refusals(const edge::Map& m, int& failures) {
  const edge::Camera camera = scene::camera();
  const edge::Options options;
  const edge::Database db;
  loops::Pending pending;
  pending.matched = 0;
  pending.last = 1;
  pending.confirmations = 2;
  const auto skipped = loops::detect(m, db, camera, 2, pending, "rgbd", options);
  expect(skipped.has_value() && skipped->summary.outcome == loops::Outcome::kSkipped &&
             skipped->pending.has_value() && skipped->pending->confirmations == 2 &&
             skipped->summary.confirmations == 2 && !skipped->loop,
         "a map below loops.min_keyframes is not searched, the loop carried", failures);
  expect(is_invalid_input(loops::detect(m, db, camera, 7, std::nullopt, "rgbd", options)),
         "detection refuses a keyframe outside the map", failures);
  expect(is_invalid_input(loops::detect(m, db, camera, 2, std::nullopt, "lidar", options)),
         "detection refuses an unknown mode", failures);
  edge::Map changed = m;
  loops::Loop stray;
  stray.current = 7;
  expect(is_invalid_input(loops::correct(changed, camera, stray, "rgbd", options)),
         "correction refuses a keyframe outside the map", failures);
  loops::Loop short_of_points;
  short_of_points.current = 2;
  short_of_points.matched = 0;
  expect(is_invalid_input(loops::correct(changed, camera, short_of_points, "rgbd", options)),
         "correction refuses matched points not one per keypoint", failures);
  const std::vector<std::int64_t> outside{static_cast<std::int64_t>(scene::kPoints)};
  expect(is_invalid_input(loops::search_by_similarity(m, camera, 0, m.kf_pose[0], outside,
                                                      {.radius = 5.0, .distance = 50}, options)),
         "a search refuses a point row out of range", failures);
  expect(is_invalid_input(loops::search_by_similarity(m, camera, 3, m.kf_pose[0], {},
                                                      {.radius = 5.0, .distance = 50}, options)),
         "a search refuses a keyframe row out of range", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_similarities(failures);
  check_budget(failures);
  check_held(failures);
  check_merge_first(failures);
  if (const std::optional<edge::Map> m = scene_map(failures)) {
    check_scene(*m, failures);
    check_refusals(*m, failures);
  }
  return slam_it::testing::finish("edge/loops", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/loops", run); }
