/// @file
/// @brief Native tests for initialisation (slamlab/edge/initialization.hpp) on the exact
///        scene: from depth, the initialisation matches, and a monocular start. Parity with the
///        Python reference is checked through the bindings by tests/edge.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slamlab/edge/initialization.hpp"

namespace {

namespace edge = slamlab::edge;
namespace scene = slamlab::testing::scene;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;

/// @brief Check starting from depth.
/// @param failures Running failure count.
void check_from_depth(int& failures) {
  const edge::Frame frame = scene::frame(0, true);
  const edge::Options options{.min_init_features_depth = 10, .min_init_depth_points = 10};
  const auto started = edge::initialize_from_depth(frame, options);
  expect(started.has_value() && started->has_value(), "a frame with depth starts the map",
         failures);
  if (started && *started) {
    const edge::Map& m = (*started)->map;
    double worst = 0.0;
    const std::vector<Eigen::Vector3d> truth = scene::points();
    for (std::size_t p = 0; p < edge::point_count(m); ++p) {
      worst = std::max(worst, (m.pt_xyz[p] - truth[p]).norm());
    }
    expect(edge::keyframe_count(m) == 1 && edge::point_count(m) == scene::kPoints &&
               worst < 1e-12 && edge::inliers((*started)->tracked) == 60,
           "it puts a point at every keypoint's depth", failures);
  }
  const auto few =
      edge::initialize_from_depth(frame, edge::Options{.min_init_features_depth = 100});
  expect(few.has_value() && !few->has_value(), "too few keypoints start nothing", failures);
  const auto exactly = edge::initialize_from_depth(
      frame, edge::Options{.min_init_features_depth = 60, .min_init_depth_points = 10});
  expect(exactly.has_value() && !exactly->has_value(),
         "the start needs more keypoints than the count, as ORB-SLAM's N > 500", failures);
  const auto shallow = edge::initialize_from_depth(
      frame, edge::Options{.min_init_features_depth = 10, .min_init_depth_points = 61});
  expect(shallow.has_value() && !shallow->has_value(), "too few keypoints with depth start nothing",
         failures);
  expect(is_invalid_input(edge::initialize_from_depth(frame, edge::Options{.levels = 4})),
         "a frame of another pyramid is rejected", failures);
}

/// @brief Check the initialisation matches and a monocular start.
/// @param failures Running failure count.
void check_monocular(int& failures) {
  const edge::Frame first = scene::frame(0, false);
  const edge::Frame last = scene::frame(2, false);
  std::vector<std::array<double, 2>> own;
  own.reserve(first.features.keypoints.size());
  for (const auto& k : first.features.keypoints) {
    own.push_back(k.xy);
  }
  const auto found = edge::match_for_initialization(first, last, own, edge::Options{});
  expect(found.has_value() && found->size() == scene::kPoints &&
             std::ranges::all_of(*found, [](const auto& m) { return m.first == m.second; }),
         "each keypoint matches its own point's", failures);
  std::vector<std::array<double, 2>> away = own;
  for (auto& xy : away) {
    xy[0] += 1000.0;
  }
  const auto none = edge::match_for_initialization(first, last, away, edge::Options{});
  expect(none.has_value() && none->empty(), "the search is where each keypoint was last matched",
         failures);
  expect(is_invalid_input(edge::match_for_initialization(
             first, last, std::span(own).first(own.size() - 1), edge::Options{})),
         "match_for_initialization rejects previous positions not one per keypoint", failures);
  std::vector<std::array<std::int64_t, 2>> pairs;
  if (found) {
    pairs.reserve(found->size());
    for (const auto& match : *found) {
      pairs.push_back({match.first, match.second});
    }
  }
  const auto few = edge::initialize_monocular(scene::camera(), first, last, pairs, edge::Options{});
  expect(few.has_value() && !few->has_value(), "fewer matches than init_min_matches start nothing",
         failures);
  const std::vector<std::array<std::int64_t, 2>> beyond{{0, scene::kPoints}};
  expect(is_invalid_input(
             edge::initialize_monocular(scene::camera(), first, last, beyond, edge::Options{})),
         "initialize_monocular rejects a pair beyond a frame's keypoints", failures);
  const auto started = edge::initialize_monocular(scene::camera(), first, last, pairs,
                                                  edge::Options{.init_min_matches = 20});
  expect(started.has_value() && started->has_value(), "two views with parallax start the map",
         failures);
  if (!started || !*started) {
    return;
  }
  const edge::Map& m = (*started)->map;
  std::vector<double> depths;
  depths.reserve(m.pt_xyz.size());
  for (const Eigen::Vector3d& x : m.pt_xyz) {
    depths.push_back(x.z());
  }
  std::ranges::sort(depths);
  const double median = (depths[(depths.size() - 1) / 2] + depths[depths.size() / 2]) / 2.0;
  const Eigen::Vector3d moved = m.kf_pose[1].topRightCorner<3, 1>().normalized();
  const Eigen::Vector3d truth = scene::pose(2).topRightCorner<3, 1>().normalized();
  expect(edge::keyframe_count(m) == 2 && m.kf_pose[0] == Eigen::Matrix4d::Identity() &&
             std::abs(median - 1.0) < 1e-12 && (moved - truth).norm() < 1e-6,
         "the map is scaled to unit median depth, the motion's direction recovered", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_from_depth(failures);
  check_monocular(failures);
  return slamlab::testing::finish("edge/initialization", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("edge/initialization", run); }
