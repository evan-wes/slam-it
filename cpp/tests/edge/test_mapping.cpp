/// @file
/// @brief Native tests for local mapping (slam_it/edge/mapping.hpp) on the exact scene: the
///        keyframe decision, points from depth, culling points and keyframes, triangulation,
///        fusion, and a whole mapping step; and matching for triangulation held to the fixture
///        `tests/fixtures/edge/triangulation_matches.json`, which the Python and Rust tests share.
///        Parity with the Python reference is checked through the bindings by tests/edge. What
///        fusion and local BA take from the graph
///        (`fuse_targets`, `local_ba_keyframes`) is held to the covisibility fixture in
///        test_covisibility.cpp.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
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
using slam_it::testing::is_invalid_input;

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
/// @param from The first of them to add (the keyframe's earlier keypoints observe others).
/// @return Whether it worked.
bool add_scene_points(edge::Map& m, std::int64_t k, std::span<const Eigen::Vector3d> xyz,
                      std::size_t from = 0) {
  std::vector<std::vector<std::int64_t>> observers;
  for (std::size_t p = from; p < xyz.size(); ++p) {
    observers.push_back({m.kf_start[static_cast<std::size_t>(k)] + static_cast<std::int64_t>(p)});
  }
  return edge::add_points(m, xyz.subspan(from), observers).has_value();
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
    const auto newest = static_cast<std::int64_t>(edge::keyframe_count(*built)) - 1;
    return edge::need_keyframe(*built, scene::camera(), frame.depth,
                               {.pose = scene::pose(2), .points = ids(0, kept)}, newest,
                               edge::Mode::kRgbd, edge::Options{});
  };
  expect(need(16).value_or(false) && !need(15).value_or(true) && !need(60).value_or(true),
         "a keyframe when tracking thins, but not below the minimum inliers", failures);
  for (const std::int64_t reference :
       {std::int64_t{-1}, static_cast<std::int64_t>(edge::keyframe_count(*built))}) {
    expect(is_invalid_input(edge::need_keyframe(*built, scene::camera(), frame.depth,
                                                {.pose = scene::pose(2), .points = ids(0, 16)},
                                                reference, edge::Mode::kRgbd, edge::Options{})),
           "the keyframe decision refuses a reference keyframe out of range", failures);
  }
  edge::Map m = *built;
  const std::vector<std::int64_t> none(scene::kPoints, -1);
  const Eigen::Matrix4d ahead = scene::moved(0.1);
  const edge::Frame ahead_frame = scene::frame_at(ahead, true);
  if (!edge::insert_keyframe(m, ahead_frame, ahead, none, scene::nodes(ahead_frame))) {
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
  const edge::Frame first = scene::frame(0, false);
  const edge::Frame second = scene::frame(2, false);
  if (!edge::insert_keyframe(m, first, scene::pose(0), ids(0, 0), scene::nodes(first)) ||
      !add_scene_points(m, 0, first_points(10)) ||
      !edge::insert_keyframe(m, second, scene::pose(2), ids(0, 10), scene::nodes(second))) {
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

/// @brief A triangulation case's map (`tests/fixtures/edge/triangulation_matches.json`): its two
///        keyframes' columns and the points they observe, as the Python test builds it.
/// @param c The case.
/// @return The map, its observations indexed.
edge::Map triangulation_map(const nlohmann::json& c) {
  edge::Map m = edge::empty_map(c["levels"].get<std::int64_t>(), c["scale_factor"].get<double>());
  std::int64_t id = 0;
  for (const nlohmann::json& k : c["keyframes"]) {
    m.kf_id.push_back(id);
    m.kf_time.push_back(static_cast<double>(id));
    ++id;
    Eigen::Matrix4d pose;
    for (int r = 0; r < 4; ++r) {
      for (int col = 0; col < 4; ++col) {
        pose(r, col) = k["pose"][r][col].get<double>();
      }
    }
    m.kf_pose.push_back(pose);
    m.kf_parent.push_back(-1);
    for (std::size_t i = 0; i < k["level"].size(); ++i) {
      const auto& b = k["bearing"][i];
      m.kp_xy.push_back({0.0, 0.0});
      m.kp_level.push_back(k["level"][i].get<std::int64_t>());
      m.kp_angle.push_back(0);
      slam_it::foundations::orb::Descriptor d{};
      for (std::size_t byte = 0; byte < d.size(); ++byte) {
        d.at(byte) = k["descriptor"][i][byte].get<std::uint8_t>();
      }
      m.kp_descriptor.push_back(d);
      m.kp_node.push_back(k["node"][i].get<std::int64_t>());
      m.kp_bearing.emplace_back(b[0].get<double>(), b[1].get<double>(), b[2].get<double>());
      m.kp_depth.push_back(k["depth"][i].is_null() ? std::numeric_limits<double>::quiet_NaN()
                                                   : k["depth"][i].get<double>());
      m.kp_right.push_back(std::numeric_limits<double>::quiet_NaN());
      m.kp_point.push_back(k["point"][i].get<std::int64_t>());
    }
    m.kf_start.push_back(static_cast<std::int64_t>(m.kp_point.size()));
  }
  for (const nlohmann::json& x : c["points"]) {
    m.pt_id.push_back(static_cast<std::int64_t>(m.pt_id.size()));
    m.pt_xyz.emplace_back(x[0].get<double>(), x[1].get<double>(), x[2].get<double>());
    m.pt_descriptor.emplace_back();
    m.pt_normal.emplace_back(0.0, 0.0, 1.0);
    m.pt_min_distance.push_back(1.0);
    m.pt_max_distance.push_back(1.0);
    m.pt_visible.push_back(1);
    m.pt_found.push_back(1);
    m.pt_first_kf.push_back(0);
    m.pt_reference_kf.push_back(0);
  }
  m.next_kf_id = id;
  m.next_pt_id = static_cast<std::int64_t>(m.pt_id.size());
  edge::index_observations(m);
  return m;
}
/// @brief Check matching for triangulation against the fixture's rules
///        (`tests/fixtures/edge/triangulation_matches.json`, which the Python and Rust tests
///        share): only under one vocabulary node, within `matching.th_low`, the epipolar band and
///        away from the epipole without depth; the nearest candidate (the last among equals),
///        each keyframe-2 keypoint kept by its nearest (the lower among equals).
/// @param failures Running failure count.
void check_triangulation_fixture(int& failures) {
  const nlohmann::json fixture = slam_it::testing::load_fixture("edge/triangulation_matches.json");
  for (const nlohmann::json& c : fixture["cases"]) {
    const edge::Map m = triangulation_map(c);
    edge::Camera camera = scene::camera();
    const auto focal = c["focal"].get<double>();
    camera.intrinsics = Eigen::Vector4d(focal, focal, 320.0, 240.0);
    edge::Options options;
    options.matching.th_low = c["settings"]["matching.th_low"].get<std::int64_t>();
    options.mapping.epipolar_chi2 = c["settings"]["mapping.epipolar_chi2"].get<double>();
    options.mapping.min_epipole_distance2 =
        c["settings"]["mapping.min_epipole_distance2"].get<double>();
    const auto found = edge::match_for_triangulation(m, camera, c["k1"].get<std::int64_t>(),
                                                     c["k2"].get<std::int64_t>(), options);
    std::vector<std::array<std::int64_t, 2>> expected;
    for (const nlohmann::json& pair : c["pairs"]) {
      expected.push_back({pair[0].get<std::int64_t>(), pair[1].get<std::int64_t>()});
    }
    expect(edge::check_map(m).has_value() && found.has_value() && *found == expected,
           ("the fixture's matches, " + c["name"].get<std::string>()).c_str(), failures);
  }
}

/// @brief Check fusion: the keyframes share a few points, so they are linked, and each made its
///        own of the rest; fusing merges those.
/// @param failures Running failure count.
void check_fusion(int& failures) {
  constexpr std::int64_t kShared = 5;
  edge::Map m = edge::empty_map(8, 1.2);
  const edge::Frame first = scene::frame(0, false);
  const edge::Frame second = scene::frame(2, false);
  if (!edge::insert_keyframe(m, first, scene::pose(0), ids(0, 0), scene::nodes(first)) ||
      !add_scene_points(m, 0, first_points(scene::kPoints)) ||
      !edge::insert_keyframe(m, second, scene::pose(2), ids(0, kShared), scene::nodes(second)) ||
      !add_scene_points(m, 1, first_points(scene::kPoints), kShared)) {
    expect(false, "the fusion scene builds", failures);
    return;
  }
  const bool fused =
      edge::fuse_points(m, scene::camera(), edge::Mode::kRgbd, edge::Options{}).has_value();
  bool linked = true;
  for (std::size_t row = 0; row < m.kp_point.size(); ++row) {
    const auto index = static_cast<std::int64_t>(row % scene::kPoints);  // in its keyframe
    linked = linked && (index < kShared ? m.kp_point[row] == index
                                        : std::cmp_greater_equal(m.kp_point[row], scene::kPoints));
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
  // A fourth keyframe seeing every point: each is seen by three keyframes besides keyframe 1.
  const auto culled = [&](std::int64_t observers) {
    edge::Map copy = m;
    edge::Options options;
    options.mapping.redundant_observers = observers;
    const edge::Frame again = scene::frame(2, true);
    if (!edge::insert_keyframe(copy, again, scene::pose(2), ids(0, 60), scene::nodes(again)) ||
        !edge::cull_keyframes(copy, scene::camera(), edge::Mode::kRgbd, options) ||
        !edge::check_map(copy)) {
      return std::vector<std::int64_t>{};
    }
    return copy.kf_id;
  };
  expect(culled(3) == std::vector<std::int64_t>{0, 2, 3},
         "at ORB-SLAM2's 3 observers the first redundant keyframe goes, the first keyframe stays, "
         "the index kept",
         failures);
  expect(edge::Options{}.mapping.redundant_observers == 4 &&
             culled(edge::Options{}.mapping.redundant_observers) ==
                 std::vector<std::int64_t>{0, 1, 2, 3},
         "at ORB-SLAM3's rule, more than 3 other observers, the default, every keyframe stays",
         failures);
  edge::Map none = edge::empty_map(8, 1.2);
  expect(is_invalid_input(
             edge::cull_keyframes(none, scene::camera(), edge::Mode::kRgbd, edge::Options{})),
         "culling refuses a map without keyframes", failures);
  expect(is_invalid_input(edge::fuse_targets(none, edge::Mode::kRgbd, edge::Options{})) &&
             is_invalid_input(edge::local_ba_keyframes(none, edge::Options{})),
         "fusion's targets and local BA's keyframes refuse a map without keyframes", failures);
  const edge::Options bad{.covisibility = {.local_neighbours = 0}};
  expect(is_invalid_input(edge::fuse_targets(m, edge::Mode::kRgbd, bad)) &&
             is_invalid_input(edge::local_ba_keyframes(m, bad)),
         "fusion's targets and local BA's keyframes refuse invalid settings", failures);
  Eigen::Matrix4d ahead = scene::pose(2);
  ahead(2, 3) -= 0.05;
  const std::int64_t first = m.next_pt_id;
  const edge::Frame frame = scene::frame_at(ahead, true);
  // Half the keypoints tracked: depth makes points of the rest.
  const auto mapped =
      edge::process(m, scene::camera(), frame, {.pose = ahead, .points = ids(0, 30)},
                    edge::Mode::kRgbd, edge::Options{}, scene::one_word());
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
  check_triangulation_fixture(failures);
  check_fusion(failures);
  check_culling_and_process(failures);
  return slam_it::testing::finish("edge/mapping", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/mapping", run); }
