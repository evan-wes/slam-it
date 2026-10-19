/// @file
/// @brief Native tests for descriptor matching (slam_it/foundations/matching.hpp), against the
///        parity fixture `tests/fixtures/foundations/matching.json` that the Python and Rust
///        tests share (matching by word's cases too), and the core's own input checks.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "fixture_support.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/rng.hpp"
#include "test_support.hpp"

namespace {

namespace matching = slam_it::foundations::matching;
namespace orb = slam_it::foundations::orb;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief Read a fixture's right column: `null` is none (NaN).
/// @param value The JSON value.
/// @return The column.
double column(const nlohmann::json& value) {
  return value.is_null() ? std::numeric_limits<double>::quiet_NaN() : value.get<double>();
}

/// @brief Read a fixture's keypoints: positions, levels and angles (angles optional).
/// @param keys The JSON object.
/// @return The keypoints.
std::vector<orb::Keypoint> keypoints_of(const nlohmann::json& keys) {
  std::vector<orb::Keypoint> out(keys["xy"].size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i].xy = {keys["xy"][i][0].get<double>(), keys["xy"][i][1].get<double>()};
    out[i].level = keys["level"][i].get<std::int64_t>();
    if (keys.contains("angle")) {
      out[i].angle = keys["angle"][i].get<std::int64_t>();
    }
  }
  return out;
}

/// @brief Check that a search's matches equal a fixture's.
/// @param got The search's result.
/// @param expected The fixture's `{pairs, distances}`.
/// @return Whether they are equal.
bool same_matches(const slam_it::foundations::Result<std::vector<matching::Match>>& got,
                  const nlohmann::json& expected) {
  bool ok = got.has_value() && got->size() == expected["pairs"].size();
  for (std::size_t k = 0; ok && k < got->size(); ++k) {
    const matching::Match& m = (*got)[k];
    ok = m.first == expected["pairs"][k][0].get<std::int64_t>() &&
         m.second == expected["pairs"][k][1].get<std::int64_t>() &&
         m.distance == expected["distances"][k].get<std::int64_t>();
  }
  return ok;
}

/// @brief Check the search-by-projection and window fixture cases.
/// @param functions The fixture's functions.
/// @param failures Running failure count.
void check_search_fixture(const nlohmann::json& functions, int& failures) {
  for (const auto& c : functions["features_in_area"]["cases"]) {
    const auto& input = c["input"];
    const auto got = matching::features_in_area(
        keypoints_of({{"xy", input[0]}, {"level", input[1]}}),
        {input[2][0].get<double>(), input[2][1].get<double>()}, input[3].get<double>(),
        input[4].get<std::int64_t>(), input[5].get<std::int64_t>());
    expect(got.has_value() && *got == c["expected"].get<std::vector<std::int64_t>>(),
           "features_in_area matches the fixture", failures);
  }
  for (const auto& c : functions["search_by_projection"]["cases"]) {
    const auto& raw = c["input"][0];
    const auto& keys = c["input"][1];
    const auto& settings = c["input"][2];
    std::vector<matching::Projection> projections(raw["xy"].size());
    for (std::size_t i = 0; i < projections.size(); ++i) {
      projections[i] = {.xy = {raw["xy"][i][0].get<double>(), raw["xy"][i][1].get<double>()},
                        .radius = raw["radius"][i].get<double>(),
                        .min_level = raw["min_level"][i].get<std::int64_t>(),
                        .max_level = raw["max_level"][i].get<std::int64_t>(),
                        .descriptor = raw["descriptors"][i].get<orb::Descriptor>(),
                        .angle = raw["angle"][i].get<std::int64_t>(),
                        .right = column(raw["right"][i])};
    }
    const orb::Features features{
        .keypoints = keypoints_of(keys),
        .descriptors = keys["descriptors"].get<std::vector<orb::Descriptor>>(),
        .scale_factor = 1.2,
        .levels = keys["levels"].get<std::int64_t>()};
    std::vector<double> right;
    std::vector<std::uint8_t> taken;
    for (std::size_t k = 0; k < features.keypoints.size(); ++k) {
      right.push_back(column(keys["right"][k]));
      taken.push_back(keys["taken"][k].get<bool>() ? 1 : 0);
    }
    const auto grid = matching::FeatureGrid::build(features.keypoints);
    const matching::SearchOptions options{
        .max_distance = settings["max_distance"].get<std::int64_t>(),
        .ratio = settings["ratio"].get<double>(),
        .check_orientation = settings["check_orientation"].get<bool>()};
    expect(
        grid.has_value() && same_matches(matching::search_by_projection(
                                             projections, features, *grid, right, taken, options),
                                         c["expected"]),
        "search_by_projection matches the fixture", failures);
  }
}

/// @brief Check the grid against a brute force on random keypoints of several spreads.
/// @param failures Running failure count.
void check_grid(int& failures) {
  slam_it::foundations::rng::Pcg32 rng(7U, 0U);
  const auto unit = [&] { return std::ldexp(static_cast<double>(rng.next_u32()), -32); };
  const auto level = [&] { return static_cast<std::int64_t>(rng.next_u32() % 8U); };
  for (const auto [width, height] : std::array<std::array<double, 2>, 4>{
           {{640.0, 480.0}, {0.0, 480.0}, {1e-9, 1e-9}, {5000.0, 3.0}}}) {
    std::vector<orb::Keypoint> keypoints(800);
    for (orb::Keypoint& k : keypoints) {
      k.xy = {unit() * width, unit() * height};
      k.level = level();
    }
    const auto grid = matching::FeatureGrid::build(keypoints);
    bool ok = grid.has_value();
    for (int trial = 0; ok && trial < 300; ++trial) {
      const orb::Keypoint& near = keypoints[static_cast<std::size_t>(trial)];
      const double r = trial % 3 == 0 ? 1e-12 : unit() * 0.2 * (width + height + 1.0);
      const std::array<double, 2> center{near.xy[0] + ((unit() - 0.5) * r), near.xy[1]};
      const std::int64_t low = level();
      std::vector<std::int64_t> brute;
      for (std::size_t i = 0; i < keypoints.size(); ++i) {
        const orb::Keypoint& k = keypoints[i];
        if (std::fabs(k.xy[0] - center[0]) < r && std::fabs(k.xy[1] - center[1]) < r &&
            k.level >= low && k.level <= low + 2) {
          brute.push_back(static_cast<std::int64_t>(i));
        }
      }
      const auto found = grid->in_area(center, r, low, low + 2);
      ok = found.has_value() && *found == brute;
    }
    expect(ok, "the grid finds what a brute force finds", failures);
  }
}

/// @brief Check that malformed searches and windows are `InvalidInput`.
/// @param failures Running failure count.
void check_search_rejections(int& failures) {
  const std::vector<orb::Keypoint> keypoints{{.xy = {1.0, 1.0}, .level = 0},
                                             {.xy = {2.0, 2.0}, .level = 1}};
  const orb::Features features{.keypoints = keypoints,
                               .descriptors = std::vector<orb::Descriptor>(2),
                               .scale_factor = 1.2,
                               .levels = 8};
  const auto grid = matching::FeatureGrid::build(keypoints);
  const std::vector<double> right(2, std::numeric_limits<double>::quiet_NaN());
  const std::vector<std::uint8_t> taken(2, 0);
  const auto search = [&](const matching::Projection& p, const orb::Features& f) {
    return matching::search_by_projection(std::vector<matching::Projection>{p}, f, *grid, right,
                                          taken, matching::SearchOptions{});
  };
  expect(grid.has_value() && search({.xy = {1.0, 1.0}}, features).has_value(),
         "a valid search succeeds", failures);
  expect(is_invalid_input(search({.xy = {1.0, 1.0}, .radius = 0.0}, features)),
         "search rejects a zero radius", failures);
  expect(is_invalid_input(search({.xy = {std::nan(""), 1.0}}, features)),
         "search rejects a projection that is not finite", failures);
  expect(is_invalid_input(search({.xy = {1.0, 1.0}, .angle = 30}, features)),
         "search rejects an angle out of range", failures);
  expect(is_invalid_input(search(
             {.xy = {1.0, 1.0}, .right = std::numeric_limits<double>::infinity()}, features)),
         "search rejects an infinite right column", failures);
  orb::Features deep = features;
  deep.keypoints[1].level = 8;
  expect(is_invalid_input(search({.xy = {1.0, 1.0}}, deep)),
         "search rejects a level outside the pyramid", failures);
  orb::Features short_of = features;
  short_of.descriptors.pop_back();
  expect(is_invalid_input(search({.xy = {1.0, 1.0}}, short_of)),
         "search rejects features whose arrays disagree", failures);
  expect(is_invalid_input(matching::search_by_projection({}, features, *grid, right, taken,
                                                         matching::SearchOptions{.ratio = 0.0})),
         "search rejects a zero ratio", failures);
  expect(is_invalid_input(matching::FeatureGrid::build(
             std::vector<orb::Keypoint>{{.xy = {std::numeric_limits<double>::infinity(), 0.0}}})),
         "the grid rejects a position that is not finite", failures);
  expect(is_invalid_input(grid->in_area({0.0, 0.0}, -1.0, 0, 7)),
         "a window rejects a negative radius", failures);
  expect(is_invalid_input(matching::features_in_area(
             std::vector<orb::Keypoint>{{.xy = {0.0, 0.0}, .level = -1}}, {0.0, 0.0}, 1.0, 0, 7)),
         "features_in_area rejects a negative level", failures);
}

/// @brief Check every fixture case.
/// @param failures Running failure count.
void check_fixture(int& failures) {
  const nlohmann::json functions =
      slam_it::testing::load_fixture("foundations/matching.json")["functions"];
  for (const auto& c : functions["distances"]["cases"]) {
    const auto a = c["input"][0].get<std::vector<orb::Descriptor>>();
    const auto b = c["input"][1].get<std::vector<orb::Descriptor>>();
    const auto expected = slam_it::testing::flatten_rows<std::int64_t>(c["expected"]);
    const auto got = matching::distances(a, b);
    expect(got.has_value() && *got == expected, "distances matches the fixture", failures);
  }
  for (const auto& c : functions["match"]["cases"]) {
    const auto& input = c["input"];
    const auto& settings = input[4];
    const matching::Options options{.max_distance = settings["max_distance"].get<std::int64_t>(),
                                    .ratio = settings["ratio"].get<double>(),
                                    .check_orientation = settings["check_orientation"].get<bool>()};
    const auto got = matching::match(input[0].get<std::vector<orb::Descriptor>>(),
                                     input[1].get<std::vector<orb::Descriptor>>(),
                                     input[2].get<std::vector<std::int64_t>>(),
                                     input[3].get<std::vector<std::int64_t>>(), options);
    bool ok = got.has_value() && got->size() == c["expected"]["pairs"].size();
    for (std::size_t k = 0; ok && k < got->size(); ++k) {
      const matching::Match& m = (*got)[k];
      ok = m.first == c["expected"]["pairs"][k][0].get<std::int64_t>() &&
           m.second == c["expected"]["pairs"][k][1].get<std::int64_t>() &&
           m.distance == c["expected"]["distances"][k].get<std::int64_t>();
    }
    expect(ok, "match matches the fixture", failures);
  }
  for (const auto& c : functions["search_by_bow"]["cases"]) {
    const auto& input = c["input"];
    const auto& settings = input[6];
    const matching::Options options{.max_distance = settings["max_distance"].get<std::int64_t>(),
                                    .ratio = settings["ratio"].get<double>(),
                                    .check_orientation = settings["check_orientation"].get<bool>()};
    const auto got = matching::search_by_bow(
        input[0].get<std::vector<orb::Descriptor>>(), input[1].get<std::vector<std::int64_t>>(),
        input[2].get<std::vector<orb::Descriptor>>(), input[3].get<std::vector<std::int64_t>>(),
        input[4].get<std::vector<std::int64_t>>(), input[5].get<std::vector<std::int64_t>>(),
        options);
    expect(same_matches(got, c["expected"]),
           ("search_by_bow matches the fixture: " + c["name"].get<std::string>()).c_str(),
           failures);
  }
  check_search_fixture(functions, failures);
}

/// @brief Check that bad filters and angles are `InvalidInput`.
/// @param failures Running failure count.
void check_rejections(int& failures) {
  expect(is_invalid_input(matching::check_options({.max_distance = 257})),
         "check_options rejects a distance above 256", failures);
  expect(is_invalid_input(matching::check_options({.ratio = 0.0})),
         "check_options rejects a zero ratio", failures);
  const std::vector<orb::Descriptor> set(3);
  const std::vector<std::int64_t> bins{0, 1, 2};
  expect(is_invalid_input(
             matching::match(set, set, std::vector<std::int64_t>{0, 1}, bins, matching::Options{})),
         "match rejects too few angles", failures);
  expect(is_invalid_input(matching::match(set, set, bins, std::vector<std::int64_t>{0, 1, 30},
                                          matching::Options{})),
         "match rejects an angle bin out of range", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_fixture(failures);
  check_rejections(failures);
  check_grid(failures);
  check_search_rejections(failures);
  return slam_it::testing::finish("foundations/matching", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("foundations/matching", run); }
