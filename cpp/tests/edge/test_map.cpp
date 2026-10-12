/// @file
/// @brief Native tests for the edge rung's state (slam_it/edge/{options,frames,map}.hpp): a
///        small hand-built map's operations and observation index, the checks, depth lookup.
///        Parity with the Python reference is checked through the bindings by tests/edge.
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/edge/frames.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"

namespace {

namespace edge = slam_it::edge;
namespace orb = slam_it::foundations::orb;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief NaN, for an unknown depth or column.
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief A frame of `n` keypoints along a row, each a distinct descriptor, with depth when
///        asked (and its right column).
/// @param n The keypoints.
/// @param with_depth Whether they have depth (2 m).
/// @return The frame.
edge::Frame frame_of(std::size_t n, bool with_depth) {
  edge::Frame frame;
  frame.features.levels = 8;
  frame.features.scale_factor = 1.2;
  for (std::size_t i = 0; i < n; ++i) {
    const double x = 100.0 + (10.0 * static_cast<double>(i));
    frame.features.keypoints.push_back({.xy = {x, 120.0}, .level = 0});
    orb::Descriptor d{};
    d.at(i % orb::kDescriptorBytes) = static_cast<std::uint8_t>(1U + i);
    frame.features.descriptors.push_back(d);
    frame.bearings.emplace_back(Eigen::Vector3d((x - 160.0) / 250.0, 0.0, 1.0).normalized());
    frame.depth.push_back(with_depth ? 2.0 : kNan);
    frame.right.push_back(with_depth ? x - 5.0 : kNan);
  }
  return edge::make_frame(frame.timestamp, std::move(frame.features), std::move(frame.bearings),
                          std::move(frame.depth), std::move(frame.right))
      .value();
}

/// @brief A map of two keyframes: the first, with depth, creating points at its first two
///        keypoints; the second, 0.2 m along x, observing them.
/// @param failures Running failure count.
/// @return The map.
edge::Map two_keyframes(int& failures) {
  edge::Map m = edge::empty_map(8, 1.2);
  const std::vector<std::int64_t> none(4, -1);
  expect(edge::insert_keyframe(m, frame_of(4, true), Eigen::Matrix4d::Identity(), none).has_value(),
         "the setup succeeds", failures);
  expect(edge::add_points(m, std::vector<Eigen::Vector3d>{{0.0, 0.0, 2.0}, {0.1, 0.0, 2.0}},
                          std::vector<std::vector<std::int64_t>>{{0}, {1}})
             .has_value(),
         "the setup succeeds", failures);
  Eigen::Matrix4d moved = Eigen::Matrix4d::Identity();
  moved(0, 3) = -0.2;
  expect(
      edge::insert_keyframe(m, frame_of(4, false), moved, std::vector<std::int64_t>{0, 1, -1, -1})
          .has_value(),
      "the setup succeeds", failures);
  return m;
}

/// @brief Check building a map: keyframes, points, ids, links and counts.
/// @param failures Running failure count.
void check_building(int& failures) {
  edge::Map m = edge::empty_map(8, 1.2);
  const std::vector<std::int64_t> none(4, -1);
  expect(edge::insert_keyframe(m, frame_of(4, true), Eigen::Matrix4d::Identity(), none).has_value(),
         "insert_keyframe adds the first keyframe", failures);
  const std::vector<Eigen::Vector3d> xyz{{0.0, 0.0, 2.0}, {0.1, 0.0, 2.0}};
  const std::vector<std::vector<std::int64_t>> observers{{0}, {1}};
  expect(edge::add_points(m, xyz, observers).has_value(), "add_points creates two points",
         failures);
  expect(edge::point_count(m) == 2 && m.pt_id == std::vector<std::int64_t>{0, 1} &&
             m.kp_point[0] == 0 && m.kp_point[1] == 1 && m.pt_visible[0] == 1 && m.pt_found[0] == 1,
         "new points have ids, links and counts of one", failures);
  expect(std::abs(m.pt_normal[0].norm() - 1.0) < 1e-12 && m.pt_max_distance[0] > 0.0,
         "new points have a unit normal and distances", failures);
  const std::vector<std::int64_t> sees{0, 1, -1, -1};
  Eigen::Matrix4d moved = Eigen::Matrix4d::Identity();
  moved(0, 3) = -0.2;
  expect(edge::insert_keyframe(m, frame_of(4, false), moved, sees).has_value() &&
             edge::keyframe_count(m) == 2 && m.kf_start == std::vector<std::int64_t>{0, 4, 8},
         "a second keyframe observes the points", failures);
  const auto weights = edge::observation_weight(m, std::vector<std::int64_t>{0, 1});
  expect(weights.has_value() && *weights == std::vector<std::int64_t>{3, 3},
         "a point seen with depth and without weighs 3", failures);
  expect(edge::count(m, std::vector<std::int64_t>{0, 0, 7}, std::vector<std::int64_t>{1, 1, 1},
                     std::vector<std::int64_t>{0, 1, 1})
                 .has_value() &&
             m.pt_visible[0] == 3 && m.pt_found[0] == 2,
         "count adds repeats and skips unknown ids", failures);
  expect(edge::check_map(m).has_value(), "the map checks out", failures);
}

/// @brief Check removing: forgetting, erasing points and keyframes.
/// @param failures Running failure count.
void check_removing(int& failures) {
  edge::Map m = two_keyframes(failures);
  const Eigen::Vector3d first = m.pt_xyz[0];
  expect(edge::forget_observations(m, std::vector<std::int64_t>{4}).has_value() &&
             edge::point_count(m) == 1 && m.pt_id[0] == 1 && m.kp_point[0] == -1,
         "a point left seen once with depth (weight 2) is erased", failures);
  const auto weak = static_cast<std::int64_t>(edge::Removal::kWeak);
  expect(m.gone_pt_id == std::vector<std::int64_t>{0} && m.gone_pt_xyz.size() == 1 &&
             m.gone_pt_xyz[0] == first && m.gone_pt_how == std::vector<std::int64_t>{weak},
         "the erased point is kept aside, where it was, as too weakly seen", failures);
  const double time = m.kf_time[0];
  expect(edge::erase_keyframe(m, 0).has_value() && edge::keyframe_count(m) == 1 &&
             m.kf_start == std::vector<std::int64_t>{0, 4} && edge::point_count(m) == 0,
         "erasing a keyframe moves the next one up and drops its weak points", failures);
  expect(m.gone_kf_id == std::vector<std::int64_t>{0} && m.gone_kf_time == std::vector{time} &&
             m.gone_kf_pose.size() == 1 && m.gone_pt_id == std::vector<std::int64_t>{0, 1} &&
             edge::check_map(m).has_value(),
         "the erased keyframe is kept aside with the point it left too weakly seen", failures);
  edge::clear_gone(m);
  expect(m.gone_pt_id.empty() && m.gone_pt_xyz.empty() && m.gone_pt_how.empty() &&
             m.gone_kf_id.empty() && m.gone_kf_time.empty() && m.gone_kf_pose.empty(),
         "clearing empties the archive", failures);
  expect(is_invalid_input(edge::erase_keyframe(m, 1)), "erase_keyframe rejects a row past the end",
         failures);
  expect(
      is_invalid_input(edge::erase_points(m, std::vector<std::uint8_t>{1}, edge::Removal::kCulled)),
      "erase_points rejects flags that are not one per point", failures);
}

/// @brief Check the observation index: kept by the operations, refused by `check_map` stale.
/// @param failures Running failure count.
void check_index(int& failures) {
  using Lists = std::vector<std::vector<std::int64_t>>;
  edge::Map m = two_keyframes(failures);
  expect(m.pt_observations == Lists{{0, 4}, {1, 5}}, "each point lists its observing keypoint rows",
         failures);
  Eigen::Matrix4d further = Eigen::Matrix4d::Identity();
  further(0, 3) = -0.4;
  expect(
      edge::insert_keyframe(m, frame_of(4, false), further, std::vector<std::int64_t>{0, 1, -1, -1})
              .has_value() &&
          edge::erase_keyframe(m, 1).has_value(),
      "the setup succeeds", failures);
  expect(m.pt_observations == Lists{{0, 4}, {1, 5}} && edge::check_map(m).has_value(),
         "erasing a keyframe moves the later keyframes' rows up", failures);
  expect(edge::observe(m, 4, -1).has_value() && m.kp_point[4] == -1 &&
             m.pt_observations[0] == std::vector<std::int64_t>{0},
         "observe points a keypoint at nothing", failures);
  expect(edge::observe(m, 4, 0).has_value() &&
             m.pt_observations[0] == std::vector<std::int64_t>{0, 4} &&
             edge::check_map(m).has_value(),
         "observe points it at a point, in row order", failures);
  expect(is_invalid_input(edge::observe(m, 8, 0)) && is_invalid_input(edge::observe(m, 2, 7)),
         "observe rejects a row out of range and a point not in the map", failures);
  edge::Map erased = m;
  expect(edge::erase_points(erased, std::vector<std::uint8_t>{0, 1}, edge::Removal::kFused)
                 .has_value() &&
             erased.pt_observations == Lists{{0, 4}} && edge::check_map(erased).has_value(),
         "erasing a point keeps the lists of the points before it", failures);
  edge::Map stale = m;
  stale.pt_observations[0].pop_back();
  expect(is_invalid_input(edge::check_map(stale)), "check_map rejects a stale index", failures);
  edge::index_observations(stale);
  expect(stale.pt_observations == m.pt_observations, "index_observations rebuilds it", failures);
}

/// @brief Check the checks: a map's structure, arguments, options, cameras, depth lookup.
/// @param failures Running failure count.
void check_checks(int& failures) {
  edge::Map m = edge::empty_map(8, 1.2);
  const std::vector<std::int64_t> none(4, -1);
  expect(edge::insert_keyframe(m, frame_of(4, true), Eigen::Matrix4d::Identity(), none).has_value(),
         "the setup succeeds", failures);
  edge::Map dangling = m;
  dangling.kp_point[0] = 9;
  expect(is_invalid_input(edge::check_map(dangling)), "check_map rejects a dangling observation",
         failures);
  expect(is_invalid_input(edge::add_points(m, std::vector<Eigen::Vector3d>{{0.0, 0.0, 1.0}},
                                           std::vector<std::vector<std::int64_t>>{{0, 1}})),
         "add_points rejects two observers in one keyframe", failures);
  expect(is_invalid_input(edge::insert_keyframe(m, frame_of(4, true), Eigen::Matrix4d::Identity(),
                                                std::vector<std::int64_t>{3, -1, -1, -1})),
         "insert_keyframe rejects a point that is not in the map", failures);
  expect(is_invalid_input(edge::check_options(edge::Options{.window = 0})),
         "check_options rejects an empty window", failures);
  expect(is_invalid_input(edge::check_options(edge::Options{.ref_ratio = 1.5})),
         "check_options rejects a ratio above 1", failures);
  const edge::Camera bad{.model = slam_it::foundations::camera::Model::kPinhole,
                         .intrinsics = Eigen::Vector4d(250.0, 250.0, 160.0, 120.0),
                         .width = 320,
                         .height = 240,
                         .baseline = -1.0};
  expect(is_invalid_input(edge::check_camera(bad)), "check_camera rejects a negative baseline",
         failures);
  const edge::DepthMap depth{.rows = 2, .cols = 2, .values = {0.0, 1.5, 2.5, kNan}};
  const auto read = edge::depth_at(
      std::vector<std::array<double, 2>>{{0.4, 0.4}, {0.5, -3.0}, {-9.0, 0.6}, {1.2, 1.2}}, depth);
  expect(read.has_value() && std::isnan((*read)[0]) && (*read)[1] == 1.5 && (*read)[2] == 2.5 &&
             std::isnan((*read)[3]),
         "depth_at rounds half up, clamps, and returns NaN for holes", failures);
  const auto levels = edge::predict_level(m, std::vector<std::int64_t>{}, std::vector<double>{});
  expect(levels.has_value() && levels->empty(), "predict_level takes no rows", failures);
}

/// @brief The test body.
/// @return The exit code.
/// @brief Check that every setting refuses a bad value by name: an integer above 2^30, a real
///        that is not a number (review panel finding H21; Python's options test holds the same).
/// @param failures Running failure count.
void check_every_setting(int& failures) {
  std::vector<std::string> unnamed;
  const auto refused = [&unnamed](const edge::Options& options, const std::string& name) {
    const auto ok = edge::check_options(options);
    if (ok || !ok.error().message.starts_with(name + ": ")) {
      unnamed.push_back(name);
    }
  };
  for (const auto& [name, member] : edge::kIntegerOptions) {
    edge::Options options{};
    options.*member = 1'073'741'825;  // 2^30 + 1, above every integer setting's range
    refused(options, name);
  }
  for (const auto& [name, member] : edge::kRealOptions) {
    edge::Options options{};
    options.*member = kNan;
    refused(options, name);
  }
  for (const std::string& name : unnamed) {
    std::cerr << "  not refused by name: " << name << "\n";
  }
  expect(unnamed.empty(), "every setting refuses a bad value by name", failures);
}

int run() {
  int failures = 0;
  check_building(failures);
  check_removing(failures);
  check_index(failures);
  check_checks(failures);
  check_every_setting(failures);
  return slam_it::testing::finish("edge/map", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/map", run); }
