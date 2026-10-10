/// @file
/// @brief Native tests for tracking (slam_it_all/edge/tracking.hpp) on the exact scene: refinement
///        and its outliers, the points in view, the motion model's level rule, the local map,
///        and the fallback. Parity with the Python reference is checked through the bindings by
///        tests/edge.
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it_all/edge/tracking.hpp"

namespace {

namespace edge = slam_it_all::edge;
namespace scene = slam_it_all::testing::scene;
using slam_it_all::testing::expect;
using slam_it_all::testing::is_invalid_input;

/// @brief The scene's point ids, one per keypoint.
/// @return 0 to 59.
std::vector<std::int64_t> every_point() {
  std::vector<std::int64_t> out(scene::kPoints);
  for (std::size_t p = 0; p < out.size(); ++p) {
    out[p] = static_cast<std::int64_t>(p);
  }
  return out;
}

/// @brief The largest entry of a pose's difference from another.
/// @param a A pose.
/// @param b Another.
/// @return The largest absolute difference.
double gap(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) {
  return (a - b).cwiseAbs().maxCoeff();
}

/// @brief Check the prediction, the refinement and the points in view.
/// @param m The scene's map.
/// @param failures Running failure count.
void check_refinement(const edge::Map& m, int& failures) {
  Eigen::Matrix4d velocity = Eigen::Matrix4d::Identity();
  velocity(0, 3) = 0.5;
  expect(edge::predict(scene::pose(1), std::nullopt) == scene::pose(1) &&
             edge::predict(scene::pose(0), velocity)(0, 3) == 0.5,
         "predict keeps the pose, or applies the velocity", failures);
  Eigen::Matrix4d start = scene::pose(2);
  start.topRightCorner<3, 1>() += Eigen::Vector3d(0.02, -0.01, 0.03);
  std::vector<std::int64_t> points = every_point();
  points[5] = 40;  // keypoint 5 matched to a point across the image
  points[40] = -1;
  const auto refined = edge::optimize_pose(m, scene::camera(), scene::frame(2, true),
                                           {.pose = start, .points = points}, edge::Options{});
  expect(refined.has_value() && gap(refined->pose, scene::pose(2)) < 1e-6 &&
             refined->points[5] == -1 && edge::inliers(*refined) == 58,
         "refinement recovers the pose and drops the wrong match", failures);
  const auto seen =
      edge::in_view(m, scene::camera(), scene::pose(2), every_point(), edge::Options{});
  Eigen::Matrix4d turned = scene::pose(2);
  turned.topLeftCorner<3, 3>() = Eigen::Vector3d(-1.0, 1.0, -1.0).asDiagonal();
  const auto behind = edge::in_view(m, scene::camera(), turned, every_point(), edge::Options{});
  expect(seen.has_value() && seen->rows.size() == scene::kPoints && behind.has_value() &&
             behind->rows.empty(),
         "every point is in view, and none from behind", failures);
  expect(is_invalid_input(edge::in_view(m, scene::camera(), scene::pose(2),
                                        std::vector<std::int64_t>{60}, edge::Options{})),
         "in_view rejects a row past the end", failures);
}

/// @brief Check the motion model's level rule: with depth, moving forward, a point seen on
///        level 2 is not sought on level 1; moving back, or without depth, it is.
/// @param failures Running failure count.
void check_level_rule(int& failures) {
  const edge::Frame last = scene::frame_at(Eigen::Matrix4d::Identity(), false, 2);
  edge::Map m = edge::empty_map(8, 1.2);
  const std::vector<std::int64_t> none(scene::kPoints, -1);
  std::vector<std::vector<std::int64_t>> observers;
  observers.reserve(scene::kPoints);
  for (std::size_t p = 0; p < scene::kPoints; ++p) {
    observers.push_back({static_cast<std::int64_t>(p)});
  }
  if (!edge::insert_keyframe(m, last, Eigen::Matrix4d::Identity(), none) ||
      !edge::add_points(m, scene::points(), observers)) {
    expect(false, "the level scene builds", failures);
    return;
  }
  const edge::Tracked last_tracked{.pose = Eigen::Matrix4d::Identity(), .points = every_point()};
  const auto track = [&](edge::Mode mode, double dz) {
    return edge::track_last_frame(m, scene::camera(), scene::frame_at(scene::moved(dz), false, 1),
                                  last, last_tracked, scene::moved(dz), mode, edge::Options{});
  };
  const auto forward = track(edge::Mode::kRgbd, 0.3);
  const auto back = track(edge::Mode::kRgbd, -0.3);
  const auto monocular = track(edge::Mode::kMonocular, 0.3);
  expect(forward.has_value() && !forward->tracked.has_value() && forward->summary.matches == 0 &&
             forward->summary.widened && !forward->summary.inliers.has_value(),
         "with depth, moving forward finds nothing on a finer level, even widened", failures);
  expect(back.has_value() && back->tracked.has_value() && edge::inliers(*back->tracked) == 60 &&
             gap(back->tracked->pose, scene::moved(-0.3)) < 1e-9 && back->summary.matches == 60 &&
             !back->summary.widened && back->summary.inliers == 60,
         "moving back finds every point in the first window", failures);
  expect(monocular.has_value() && monocular->tracked.has_value() &&
             edge::inliers(*monocular->tracked) == 60,
         "without depth, every point is found", failures);
}

/// @brief Check the local map and the fallback.
/// @param m The scene's map.
/// @param failures Running failure count.
void check_local_map(const edge::Map& m, int& failures) {
  Eigen::Matrix4d start = scene::pose(2);
  start.topRightCorner<3, 1>() += Eigen::Vector3d(0.005, 0.0, -0.005);
  const edge::Tracked none{.pose = start, .points = std::vector<std::int64_t>(scene::kPoints, -1)};
  const auto local = edge::track_local_map(m, scene::camera(), scene::frame(2, true), none,
                                           edge::Mode::kRgbd, edge::Options{});
  expect(local.has_value() && edge::inliers(local->tracked) == 60 && local->visible.size() == 60 &&
             local->found.size() == 60 && gap(local->tracked.pose, scene::pose(2)) < 1e-6 &&
             local->matches == 60 && edge::summary(*local).in_view == 60 &&
             edge::summary(*local).inliers == 60,
         "the local map matches every point and pins the pose", failures);
  const auto placed =
      edge::relocalize_locally(m, scene::camera(), scene::frame(2, true), edge::Options{});
  expect(placed.has_value() && placed->tracked.has_value() &&
             edge::inliers(*placed->tracked) == 60 &&
             gap(placed->tracked->pose, scene::pose(2)) < 1e-6,
         "the fallback places the frame from descriptors alone", failures);
  if (placed) {
    const edge::PnpSummary& s = placed->summary;
    expect(s.candidates == 60 && s.matches == 60 && s.iterations.has_value() &&
               s.termination == slam_it_all::foundations::ransac::Termination::kConfidence &&
               !s.degenerate && s.inliers == 60 && s.refined == 60,
           "the fallback's summary records each step it took", failures);
  }
  edge::Options strict;
  strict.pnp_min_inliers = 61;
  const auto short_map =
      edge::relocalize_locally(m, scene::camera(), scene::frame(2, true), strict);
  expect(short_map.has_value() && !short_map->tracked.has_value() &&
             short_map->summary.candidates == 60 && !short_map->summary.matches.has_value(),
         "a local map smaller than the minimum stops the fallback first", failures);
  const auto recovering = edge::track_local_map(m, scene::camera(), scene::frame(2, true), none,
                                                edge::Mode::kRgbd, edge::Options{}, true);
  expect(recovering.has_value() && edge::inliers(recovering->tracked) == 60,
         "a recovery's wider search matches every point too", failures);
  expect(is_invalid_input(edge::optimize_pose(m, scene::camera(), scene::frame(2, true),
                                              {.pose = start, .points = {0}}, edge::Options{})),
         "optimize_pose rejects matches that are not one per keypoint", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const auto m = scene::map();
  expect(m.has_value(), "the scene's map builds", failures);
  if (m) {
    check_refinement(*m, failures);
    check_local_map(*m, failures);
  }
  check_level_rule(failures);
  return slam_it_all::testing::finish("edge/tracking", failures);
}

}  // namespace

int main() { return slam_it_all::testing::guarded("edge/tracking", run); }
