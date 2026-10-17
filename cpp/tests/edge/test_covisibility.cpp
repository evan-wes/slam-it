/// @file
/// @brief Native tests for the covisibility graph (slam_it/edge/covisibility.hpp), against the
///        brute-force fixture `tests/fixtures/edge/covisibility.json` that the Python and Rust
///        tests share: each keyframe's links, tracking's local map, the parent a keyframe adopts,
///        the parents a culled keyframe's children get, and the strong links.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/edge/covisibility.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"

namespace {

namespace edge = slam_it::edge;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief A case's map: the columns the graph reads (ids, parents, each keypoint's point).
/// @param keyframes The case's keyframes.
/// @return The map, its observations indexed.
edge::Map map_of(const nlohmann::json& keyframes) {
  edge::Map m;
  std::vector<std::int64_t> points;
  for (const nlohmann::json& k : keyframes) {
    m.kf_id.push_back(k["id"].get<std::int64_t>());
    m.kf_parent.push_back(k["parent"].get<std::int64_t>());
    for (const nlohmann::json& p : k["points"]) {
      m.kp_point.push_back(p.get<std::int64_t>());
      if (p.get<std::int64_t>() >= 0) {
        points.push_back(p.get<std::int64_t>());
      }
    }
    m.kf_start.push_back(static_cast<std::int64_t>(m.kp_point.size()));
  }
  std::ranges::sort(points);
  const auto [first, last] = std::ranges::unique(points);
  points.erase(first, last);
  m.pt_id = std::move(points);
  edge::index_observations(m);
  return m;
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
  }
  for (const nlohmann::json& local : c["local_maps"]) {
    const auto points = local["points"].get<std::vector<std::int64_t>>();
    const auto found = edge::local_map(m, points, options);
    expect(found.has_value() &&
               found->keyframes == local["keyframes"].get<std::vector<std::int64_t>>() &&
               found->reference == local["reference"].get<std::int64_t>(),
           (name + ": a local map").c_str(), failures);
  }
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
  return slam_it::testing::finish("edge/covisibility", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/covisibility", run); }
