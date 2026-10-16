/// @file
/// @brief Native tests for local mapping (slam_it/edge/mapping.hpp) on the exact scene: the
///        keyframe decision, points from depth, culling points and keyframes, triangulation,
///        fusion, and a whole mapping step. Parity with the Python reference is checked through
///        the bindings by tests/edge.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/mapping.hpp"

namespace {

namespace edge = slam_it::edge;
namespace scene = slam_it::testing::scene;
using slam_it::testing::expect;

/// @brief Point ids `first` to `last - 1`, then -1 to the scene's size.
/// @param first The first id.
/// @param last One past the last.
/// @return One id (or -1) per keypoint.
std::vector<std::int64_t> ids(std::int64_t first, std::int64_t last) {
  std::vector<std::int64_t> out(scene::kPoints, -1);
  for (std::int64_t p = first; p < last; ++p) {
    out[static_cast<std::size_t>(p)] = p;
  }
  return out;
}

/// @brief Add a point at each of some scene points, seen by keyframe `k`'s keypoints of the
///        same index.
/// @param m The map.
/// @param k The keyframe's row.
/// @param xyz The points: the scene's first ones.
/// @return Whether it worked.
bool add_scene_points(edge::Map& m, std::int64_t k, std::span<const Eigen::Vector3d> xyz) {
  std::vector<std::vector<std::int64_t>> observers;
  observers.reserve(xyz.size());
  for (std::size_t p = 0; p < xyz.size(); ++p) {
    observers.push_back({m.kf_start[static_cast<std::size_t>(k)] + static_cast<std::int64_t>(p)});
  }
  return edge::add_points(m, xyz, observers).has_value();
}

/// @brief The scene's first points.
/// @param count How many.
/// @return Them.
std::vector<Eigen::Vector3d> first_points(std::size_t count) {
  std::vector<Eigen::Vector3d> out = scene::points();
  out.resize(count);
  return out;
}

/// @brief Check the keyframe decision, points from depth, and culling points.
/// @param failures Running failure count.
void check_keyframes_and_points(int& failures) {
  const auto built = scene::map();
  if (!built) {
    expect(false, "the scene's map builds", failures);
    return;
  }
  const edge::Frame frame = scene::frame(2, true);
  const auto need = [&](std::int64_t kept) {
    return edge::need_keyframe(*built, scene::camera(), frame.depth,
                               {.pose = scene::pose(2), .points = ids(0, kept)}, edge::Mode::kRgbd,
                               edge::Options{});
  };
  expect(need(16).value_or(false) && !need(15).value_or(true) && !need(60).value_or(true),
         "a keyframe when tracking thins, but not below the minimum inliers", failures);
  edge::Map m = *built;
  const std::vector<std::int64_t> none(scene::kPoints, -1);
  const Eigen::Matrix4d ahead = scene::moved(0.1);
  if (!edge::insert_keyframe(m, scene::frame_at(ahead, true), ahead, none)) {
    expect(false, "the new keyframe inserts", failures);
    return;
  }
  edge::Map every = m;
  edge::Map nearest = m;
  expect(edge::create_points_from_depth(every, scene::camera(), edge::Options{}).has_value() &&
             edge::point_count(every) == 2 * scene::kPoints,
         "every close keypoint with depth makes a point", failures);
  expect(edge::create_points_from_depth(
             nearest, scene::camera(),
             edge::Options{.stereo = {.close_depth = 1.0}, .mapping = {.min_new_points = 10}})
                 .has_value() &&
             edge::point_count(nearest) == scene::kPoints + 11,
         "past the close depth, points stop once there are enough", failures);
  edge::Map counted = *built;
  counted.pt_visible[0] = 10;
  counted.pt_found[0] = 1;
  expect(edge::cull_points(counted, edge::Mode::kRgbd, edge::Options{}).has_value() &&
             edge::point_count(counted) == scene::kPoints - 1 && counted.pt_id[0] == 1,
         "a recent point found in a tenth of its views is culled", failures);
}

/// @brief Check triangulation: two monocular keyframes, ten points shared, the rest triangulated.
/// @param failures Running failure count.
void check_triangulation(int& failures) {
  edge::Map m = edge::empty_map(8, 1.2);
  if (!edge::insert_keyframe(m, scene::frame(0, false), scene::pose(0), ids(0, 0)) ||
      !add_scene_points(m, 0, first_points(10)) ||
      !edge::insert_keyframe(m, scene::frame(2, false), scene::pose(2), ids(0, 10))) {
    expect(false, "the triangulation scene builds", failures);
    return;
  }
  const auto pairs = edge::match_for_triangulation(m, scene::camera(), 1, 0, edge::Options{});
  expect(pairs.has_value() && pairs->size() == 50 && (*pairs)[0][0] == 70 && (*pairs)[0][1] == 10,
         "the free keypoints match one to one", failures);
  expect(edge::triangulate_new_points(m, scene::camera(), edge::Mode::kMonocular, edge::Options{})
                 .has_value() &&
             edge::point_count(m) == scene::kPoints,
         "every match is triangulated", failures);
  double worst = 0.0;
  const std::vector<Eigen::Vector3d> truth = scene::points();
  for (std::size_t p = 10; p < edge::point_count(m); ++p) {
    worst = std::max(worst, (m.pt_xyz[p] - truth[p]).norm());
  }
  expect(worst < 1e-9, "the triangulated points are the scene's", failures);
}

/// @brief Check fusion: each keyframe made its own points of the scene; fusing merges them.
/// @param failures Running failure count.
void check_fusion(int& failures) {
  edge::Map m = edge::empty_map(8, 1.2);
  if (!edge::insert_keyframe(m, scene::frame(0, false), scene::pose(0), ids(0, 0)) ||
      !add_scene_points(m, 0, first_points(scene::kPoints)) ||
      !edge::insert_keyframe(m, scene::frame(2, false), scene::pose(2), ids(0, 0)) ||
      !add_scene_points(m, 1, first_points(scene::kPoints))) {
    expect(false, "the fusion scene builds", failures);
    return;
  }
  const bool fused = edge::fuse_points(m, scene::camera(), edge::Options{}).has_value();
  bool linked = true;
  for (const std::int64_t point : m.kp_point) {
    linked = linked && std::cmp_greater_equal(point, scene::kPoints);
  }
  expect(
      fused && edge::point_count(m) == scene::kPoints && linked && edge::check_map(m).has_value(),
      "duplicates merge into the newest keyframe's points, seen by both, the index kept", failures);
}

/// @brief Check culling keyframes and a whole mapping step.
/// @param failures Running failure count.
void check_culling_and_process(int& failures) {
  auto built = scene::map();
  if (!built) {
    expect(false, "the scene's map builds", failures);
    return;
  }
  edge::Map m = *built;
  const auto culled = [&] {
    edge::Map copy = m;
    if (!edge::insert_keyframe(copy, scene::frame(2, true), scene::pose(2), ids(0, 60)) ||
        !edge::cull_keyframes(copy, scene::camera(), edge::Mode::kRgbd, edge::Options{}) ||
        !edge::check_map(copy)) {
      return std::vector<std::int64_t>{};
    }
    return copy.kf_id;
  }();
  expect(culled == std::vector<std::int64_t>{0, 2, 3},
         "the first redundant keyframe goes, the first keyframe stays, the index kept", failures);
  Eigen::Matrix4d ahead = scene::pose(2);
  ahead(2, 3) -= 0.05;
  const std::int64_t first = m.next_pt_id;
  const edge::Frame frame = scene::frame_at(ahead, true);
  // Half the keypoints tracked: depth makes points of the rest.
  const auto mapped =
      edge::process(m, scene::camera(), frame, {.pose = ahead, .points = ids(0, 30)},
                    edge::Mode::kRgbd, edge::Options{});
  const bool processed = mapped.has_value();
  expect(processed && m.kf_id.back() == 3 && edge::check_map(m).has_value(),
         "a whole mapping step keeps the new keyframe and a valid map", failures);
  expect(processed && mapped->made.size() == edge::keypoint_count(frame) &&
             std::ranges::any_of(mapped->made, [](std::int64_t id) { return id >= 0; }) &&
             std::ranges::all_of(mapped->made,
                                 [first](std::int64_t id) { return id == -1 || id >= first; }),
         "it names the new points made of the keyframe's keypoints", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_keyframes_and_points(failures);
  check_triangulation(failures);
  check_fusion(failures);
  check_culling_and_process(failures);
  return slam_it::testing::finish("edge/mapping", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/mapping", run); }
