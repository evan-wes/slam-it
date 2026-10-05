/// @file
/// @brief Native tests for the ORB extractor (slamlab/foundations/orb.hpp), against the parity
///        fixture `tests/fixtures/foundations/orb.json` that the Python and Rust tests share,
///        and the core's own input checks.
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "fixture_support.hpp"
#include "image_support.hpp"
#include "slamlab/foundations/image.hpp"
#include "slamlab/foundations/orb.hpp"
#include "test_support.hpp"

namespace {

namespace image = slamlab::foundations::image;
namespace orb = slamlab::foundations::orb;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;
using slamlab::testing::read_image;

/// @brief Whether detected keypoints equal the fixture's, field by field.
/// @param got The keypoints.
/// @param expected The fixture's dict of fields.
/// @return True if every field matches exactly.
bool same_keypoints(const std::vector<orb::Keypoint>& got, const nlohmann::json& expected) {
  if (got.size() != expected["level"].size()) {
    return false;
  }
  for (std::size_t i = 0; i < got.size(); ++i) {
    const orb::Keypoint& k = got[i];
    if (k.xy != expected["xy"][i].get<std::array<double, 2>>() ||
        k.level != expected["level"][i].get<std::int64_t>() ||
        k.level_xy != expected["level_xy"][i].get<std::array<std::int64_t, 2>>() ||
        k.response != expected["response"][i].get<std::int64_t>() ||
        k.angle != expected["angle"][i].get<std::int64_t>()) {
      return false;
    }
  }
  return true;
}

/// @brief Read a fixture's settings.
/// @param settings The dict of `Options`' fields.
/// @return The settings.
orb::Options read_options(const nlohmann::json& settings) {
  return orb::Options{.n_features = settings["n_features"].get<std::int64_t>(),
                      .levels = settings["levels"].get<std::int64_t>(),
                      .scale_factor = settings["scale_factor"].get<double>(),
                      .fast_threshold = settings["fast_threshold"].get<std::int64_t>(),
                      .min_fast_threshold = settings["min_fast_threshold"].get<std::int64_t>()};
}

/// @brief Check every fixture case.
/// @param failures Running failure count.
void check_fixture(int& failures) {
  const nlohmann::json functions =
      slamlab::testing::load_fixture("foundations/orb.json")["functions"];
  for (const auto& c : functions["fast_scores"]["cases"]) {
    const auto expected = slamlab::testing::flatten_rows<std::int64_t>(c["expected"]);
    const auto got = orb::fast_scores(read_image(c["input"][0]));
    expect(got.has_value() && *got == expected, "fast_scores matches the fixture", failures);
  }
  for (const auto& c : functions["distribute"]["cases"]) {
    const auto& input = c["input"];
    const auto xy = input[0].get<std::vector<std::array<std::int64_t, 2>>>();
    const auto response = input[1].get<std::vector<std::int64_t>>();
    const auto box = input[2].get<std::array<std::int64_t, 4>>();
    const auto got = orb::distribute(
        xy, response, {.x_min = box[0], .x_max = box[1], .y_min = box[2], .y_max = box[3]},
        input[3].get<std::int64_t>());
    expect(got.has_value() && *got == c["expected"].get<std::vector<std::int64_t>>(),
           "distribute matches the fixture", failures);
  }
  for (const auto& c : functions["detect"]["cases"]) {
    const auto got = orb::detect(read_image(c["input"][0]), read_options(c["input"][1]));
    expect(got.has_value() && same_keypoints(*got, c["expected"]), "detect matches the fixture",
           failures);
  }
  for (const auto& c : functions["extract"]["cases"]) {
    const auto got = orb::extract(read_image(c["input"][0]), read_options(c["input"][1]));
    expect(got.has_value() && same_keypoints(got->keypoints, c["expected"]) &&
               got->descriptors == c["expected"]["descriptors"].get<std::vector<orb::Descriptor>>(),
           "extract matches the fixture", failures);
  }
}

/// @brief Check that bad settings, corners and boxes are `InvalidInput`.
/// @param failures Running failure count.
void check_rejections(int& failures) {
  const std::vector<orb::Options> bad_options{{.n_features = 0},
                                              {.levels = 33},
                                              {.scale_factor = 1.0},
                                              {.fast_threshold = 255},
                                              {.fast_threshold = 20, .min_fast_threshold = 21}};
  for (const orb::Options& options : bad_options) {
    expect(is_invalid_input(orb::check_options(options)), "check_options rejects a bad setting",
           failures);
  }
  const image::Image tiny{.rows = 10, .cols = 10, .pixels = std::vector<std::uint8_t>(100)};
  expect(is_invalid_input(orb::detect(tiny, {.levels = 8, .scale_factor = 2.0})),
         "detect rejects a pyramid with an empty level", failures);
  const std::vector<std::array<std::int64_t, 2>> xy{{1, 2}, {3, 4}};
  const std::vector<std::int64_t> response{1, 2};
  const orb::Bounds box{.x_min = 0, .x_max = 5, .y_min = 0, .y_max = 5};
  expect(is_invalid_input(orb::distribute(xy, std::vector<std::int64_t>{1}, box, 1)),
         "distribute rejects mismatched sizes", failures);
  expect(is_invalid_input(
             orb::distribute(xy, response, {.x_min = 5, .x_max = 5, .y_min = 0, .y_max = 5}, 1)),
         "distribute rejects an empty box", failures);
  expect(is_invalid_input(
             orb::distribute(xy, response, {.x_min = 0, .x_max = 3, .y_min = 0, .y_max = 5}, 1)),
         "distribute rejects a corner outside the box", failures);
  expect(is_invalid_input(orb::distribute(xy, response, box, -1)),
         "distribute rejects a negative target", failures);
  const auto budgets = orb::level_budgets(1000, 8, 1.2);
  expect(budgets == std::vector<std::int64_t>{217, 181, 151, 126, 105, 87, 73, 60},
         "level_budgets is ORB-SLAM's split", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_fixture(failures);
  check_rejections(failures);
  return slamlab::testing::finish("foundations/orb", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("foundations/orb", run); }
