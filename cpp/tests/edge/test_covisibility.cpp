/// @file
/// @brief Native tests for the covisibility graph (slam_it/edge/covisibility.hpp), against the
///        brute-force fixture `tests/fixtures/edge/covisibility.json` that the Python and Rust
///        tests share: each keyframe's links, tracking's local map, the parent a keyframe adopts,
///        the parents a culled keyframe's children get, a keyframe culled, and the strong links.
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/edge/covisibility.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/mapping.hpp"
#include "slam_it/edge/options.hpp"

namespace {

namespace edge = slam_it::edge;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief A case's map: the columns the graph reads (ids, parents, each keypoint's point), the
///        rest blank, as the Python test builds it, so that it can be culled.
/// @param keyframes The case's keyframes.
/// @return The map, its observations indexed.
edge::Map map_of(const nlohmann::json& keyframes) {
  edge::Map m;
  std::vector<std::int64_t> points;
  for (const nlohmann::json& k : keyframes) {
    m.kf_id.push_back(k["id"].get<std::int64_t>());
    m.kf_parent.push_back(k["parent"].get<std::int64_t>());
    m.kf_time.push_back(0.0);
    m.kf_pose.emplace_back(Eigen::Matrix4d::Identity());
    for (const nlohmann::json& p : k["points"]) {
      m.kp_point.push_back(p.get<std::int64_t>());
      m.kp_xy.push_back({0.0, 0.0});
      m.kp_level.push_back(0);
      m.kp_angle.push_back(0);
      m.kp_descriptor.emplace_back();
      m.kp_bearing.emplace_back(0.0, 0.0, 1.0);
      m.kp_depth.push_back(std::numeric_limits<double>::quiet_NaN());
      m.kp_right.push_back(std::numeric_limits<double>::quiet_NaN());
      if (p.get<std::int64_t>() >= 0) {
        points.push_back(p.get<std::int64_t>());
      }
    }
    m.kf_start.push_back(static_cast<std::int64_t>(m.kp_point.size()));
  }
  std::ranges::sort(points);
  const auto [first, last] = std::ranges::unique(points);
  points.erase(first, last);
  for (std::size_t p = 0; p < points.size(); ++p) {
    m.pt_xyz.emplace_back(0.0, 0.0, 1.0);  // in front of every camera, so normals stay finite
    m.pt_descriptor.emplace_back();
    m.pt_normal.emplace_back(0.0, 0.0, 1.0);
    m.pt_min_distance.push_back(1.0);
    m.pt_max_distance.push_back(1.0);
    m.pt_visible.push_back(1);
    m.pt_found.push_back(1);
    m.pt_first_kf.push_back(0);
    m.pt_reference_kf.push_back(0);
  }
  m.next_kf_id = std::ranges::max(m.kf_id) + 1;
  m.next_pt_id = points.empty() ? 0 : points.back() + 1;
  m.pt_id = std::move(points);
  edge::index_observations(m);
  return m;
}

/// @brief A vector without one element.
/// @param values The vector.
/// @param k The element's index.
/// @return The rest, in order.
std::vector<std::int64_t> without(std::vector<std::int64_t> values, std::size_t k) {
  values.erase(values.begin() + static_cast<std::ptrdiff_t>(k));
  return values;
}

/// @brief Check culling a case's keyframe: refused for the map's first, else its children given
///        the fixture's new parents and it erased; erasing it unculled refused if it has any.
/// @param c The case.
/// @param m Its map.
/// @param k The keyframe's row.
/// @param options Its settings.
/// @param failures The failure count.
void check_cull(const nlohmann::json& c, const edge::Map& m, std::size_t k,
                const edge::Options& options, int& failures) {
  const std::string name = c["name"].get<std::string>() + ": row " + std::to_string(k);
  edge::Map culled = m;
  const auto row = static_cast<std::int64_t>(k);
  if (m.kf_id[k] == 0) {
    expect(is_invalid_input(edge::cull(culled, row, options)),
           (name + ": the map's first keyframe is not culled").c_str(), failures);
    return;
  }
  const auto parents = c["reparented"][k].get<std::vector<std::int64_t>>();
  expect(edge::cull(culled, row, options).has_value() && culled.kf_id == without(m.kf_id, k) &&
             culled.kf_parent == without(parents, k) && edge::check_map(culled).has_value() &&
             std::ranges::all_of(culled.pt_normal,
                                 [](const Eigen::Vector3d& n) { return n.allFinite(); }),
         (name + ": culled, its children reparented first").c_str(), failures);
  if (std::ranges::find(m.kf_parent, m.kf_id[k]) != m.kf_parent.end()) {
    edge::Map erased = m;
    expect(is_invalid_input(edge::erase_keyframe(erased, row)),
           (name + ": a parent is not erased").c_str(), failures);
  }
  edge::Map held = m;
  const std::vector<std::int64_t> holding{m.kf_id[k]};
  expect(edge::cull(held, row, options, holding).has_value() && held.kf_id == m.kf_id,
         (name + ": a held keyframe is spared").c_str(), failures);
  edge::Map looped = m;
  const std::int64_t other = m.kf_id[k == 0 ? 1 : 0];
  expect(edge::add_loop_edge(looped, m.kf_id[k], other).has_value() &&
             edge::cull(looped, row, options).has_value() && looped.kf_id == m.kf_id,
         (name + ": a keyframe with a loop edge is spared").c_str(), failures);
}

/// @brief A case's settings.
/// @param settings The case's `settings`.
/// @return The options.
edge::Options options_of(const nlohmann::json& settings) {
  edge::Options out;
  out.covisibility.min_points = settings["min_points"].get<std::int64_t>();
  out.covisibility.local_keyframes = settings["local_keyframes"].get<std::int64_t>();
  out.covisibility.local_neighbours = settings["local_neighbours"].get<std::int64_t>();
  out.covisibility.strong_points = settings["strong_points"].get<std::int64_t>();
  return out;
}

/// @brief Check one case.
/// @param c The case.
/// @param failures The failure count.
void check_case(const nlohmann::json& c, int& failures) {
  const std::string name = c["name"].get<std::string>();
  const edge::Map m = map_of(c["keyframes"]);
  const edge::Options options = options_of(c["settings"]);
  for (std::size_t k = 0; k < m.kf_id.size(); ++k) {
    const auto found = edge::links(m, static_cast<std::int64_t>(k), options);
    std::vector<std::int64_t> rows;
    std::vector<std::int64_t> weights;
    for (const edge::Link& link : found.value()) {
      rows.push_back(link.row);
      weights.push_back(link.weight);
    }
    expect(rows == c["links"][k][0].get<std::vector<std::int64_t>>() &&
               weights == c["links"][k][1].get<std::vector<std::int64_t>>(),
           (name + ": links of row " + std::to_string(k)).c_str(), failures);
    edge::Map orphaned = m;
    for (std::size_t r = 0; r < orphaned.kf_parent.size(); ++r) {
      if (r == k || orphaned.kf_parent[r] == m.kf_id[k]) {
        orphaned.kf_parent[r] = -1;  // its children too, as the Python test leaves them
      }
    }
    expect(edge::adopt(orphaned, static_cast<std::int64_t>(k)).has_value() &&
               orphaned.kf_parent[k] == c["adopted"][k].get<std::int64_t>(),
           (name + ": the parent row " + std::to_string(k) + " adopts").c_str(), failures);
    edge::Map culled = m;
    expect(edge::reparent(culled, static_cast<std::int64_t>(k), options).has_value() &&
               culled.kf_parent == c["reparented"][k].get<std::vector<std::int64_t>>(),
           (name + ": the parents after culling row " + std::to_string(k)).c_str(), failures);
    check_cull(c, m, k, options, failures);
  }
  for (const nlohmann::json& local : c["local_maps"]) {
    const auto points = local["points"].get<std::vector<std::int64_t>>();
    const auto found = edge::local_map(m, points, options);
    expect(found.has_value() &&
               found->keyframes == local["keyframes"].get<std::vector<std::int64_t>>() &&
               found->reference == local["reference"].get<std::int64_t>(),
           (name + ": a local map").c_str(), failures);
  }
  edge::Options fusion = options;
  fusion.mapping.fuse_neighbours = c["fusion"]["fuse_neighbours"].get<std::int64_t>();
  fusion.mapping.mono_fuse_neighbours = c["fusion"]["mono_fuse_neighbours"].get<std::int64_t>();
  fusion.mapping.fuse_second_neighbours = c["fusion"]["fuse_second_neighbours"].get<std::int64_t>();
  for (const auto& [mode, key] :
       {std::pair{edge::Mode::kMonocular, "monocular"}, std::pair{edge::Mode::kRgbd, "other"},
        std::pair{edge::Mode::kStereo, "other"}}) {
    const auto targets = edge::fuse_targets(m, mode, fusion);
    expect(
        targets.has_value() && *targets == c["fuse_targets"][key].get<std::vector<std::int64_t>>(),
        (name + ": fusion's targets").c_str(), failures);
  }
  const auto local = edge::local_ba_keyframes(m, fusion);
  expect(local.has_value() && *local == c["local_ba"].get<std::vector<std::int64_t>>(),
         (name + ": local BA's keyframes").c_str(), failures);
  std::vector<std::array<std::int64_t, 3>> strong;
  for (const nlohmann::json& link : c["strong_links"]) {
    strong.push_back(link.get<std::array<std::int64_t, 3>>());
  }
  expect(edge::strong_links(m, options) == strong, (name + ": the strong links").c_str(), failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slam_it::testing::load_fixture("edge/covisibility.json");
  for (const nlohmann::json& c : fixture["cases"]) {
    check_case(c, failures);
  }
  const edge::Map m = map_of(fixture["cases"][0]["keyframes"]);
  const edge::Options options;
  expect(is_invalid_input(edge::links(m, static_cast<std::int64_t>(m.kf_id.size()), options)),
         "a keyframe row out of range is refused", failures);
  const std::vector<std::int64_t> bad{3, -2};
  expect(is_invalid_input(edge::local_map(m, bad, options)), "a point id below -1 is refused",
         failures);
  edge::Map past = m;
  expect(is_invalid_input(edge::cull(past, static_cast<std::int64_t>(m.kf_id.size()), options)),
         "culling a keyframe row out of range is refused", failures);
  edge::Map none;
  expect(is_invalid_input(edge::cull(none, 0, options)), "culling in an empty map is refused",
         failures);
  return slam_it::testing::finish("edge/covisibility", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/covisibility", run); }
