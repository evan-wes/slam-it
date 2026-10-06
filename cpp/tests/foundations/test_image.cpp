/// @file
/// @brief Native tests for the fixed-point image operations (slamlab/foundations/image.hpp),
///        against the parity fixture `tests/fixtures/foundations/image.json` that the Python
///        and Rust tests share, and the core's own input checks.
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "fixture_support.hpp"
#include "image_support.hpp"
#include "slamlab/foundations/image.hpp"
#include "test_support.hpp"

namespace {

namespace image = slamlab::foundations::image;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;
using slamlab::testing::read_image;

/// @brief Whether a result holds exactly the expected image.
/// @param got The result.
/// @param expected The image.
/// @return True if they are equal, shape and pixels.
bool same(const slamlab::foundations::Result<image::Image>& got, const image::Image& expected) {
  return got.has_value() && got->rows == expected.rows && got->cols == expected.cols &&
         got->pixels == expected.pixels;
}

/// @brief Check every fixture case.
/// @param failures Running failure count.
void check_fixture(int& failures) {
  const nlohmann::json functions =
      slamlab::testing::load_fixture("foundations/image.json")["functions"];
  for (const auto& c : functions["to_gray"]["cases"]) {
    const auto& rows = c["input"][0];
    std::vector<std::uint8_t> rgb;
    for (const auto& row : rows) {
      for (const auto& pixel : row) {
        for (const auto& value : pixel) {
          rgb.push_back(value.get<std::uint8_t>());
        }
      }
    }
    const auto got = image::to_gray(static_cast<std::int64_t>(rows.size()),
                                    static_cast<std::int64_t>(rows[0].size()), rgb);
    expect(same(got, read_image(c["expected"])), "to_gray matches the fixture", failures);
  }
  for (const auto& c : functions["blur"]["cases"]) {
    const auto kernel = c["input"][1].get<std::vector<std::uint32_t>>();
    expect(same(image::blur(read_image(c["input"][0]), kernel), read_image(c["expected"])),
           "blur matches the fixture", failures);
  }
  for (const auto& c : functions["resize"]["cases"]) {
    const auto got = image::resize(read_image(c["input"][0]), c["input"][1].get<std::int64_t>(),
                                   c["input"][2].get<std::int64_t>());
    expect(same(got, read_image(c["expected"])), "resize matches the fixture", failures);
  }
  for (const auto& c : functions["pyramid"]["cases"]) {
    const auto got = image::pyramid(read_image(c["input"][0]), c["input"][1].get<std::int64_t>(),
                                    c["input"][2].get<double>());
    bool ok = got.has_value() && got->size() == c["expected"].size();
    for (std::size_t level = 0; ok && level < got->size(); ++level) {
      ok = same((*got)[level], read_image(c["expected"][level]));
    }
    expect(ok, "pyramid matches the fixture", failures);
  }
  for (const auto& c : functions["remap"]["cases"]) {
    const auto& map = c["input"][1];
    image::Coordinates where(static_cast<Eigen::Index>(map.size() * map[0].size()), 2);
    Eigen::Index i = 0;
    for (const auto& row : map) {
      for (const auto& point : row) {
        where.row(i++) << point[0].get<double>(), point[1].get<double>();
      }
    }
    const auto got =
        image::remap(read_image(c["input"][0]), where, static_cast<std::int64_t>(map.size()),
                     static_cast<std::int64_t>(map[0].size()));
    expect(same(got, read_image(c["expected"])), "remap matches the fixture", failures);
  }
}

/// @brief Check that bad images, kernels, shapes and settings are `InvalidInput`.
/// @param failures Running failure count.
void check_rejections(int& failures) {
  const image::Image gray{.rows = 3, .cols = 4, .pixels = std::vector<std::uint8_t>(12)};
  image::Image short_image = gray;
  short_image.pixels.pop_back();
  const std::vector<std::uint32_t> unit{1};
  expect(is_invalid_input(image::blur(short_image, unit)), "blur rejects a short buffer", failures);
  expect(is_invalid_input(image::blur(image::Image{.rows = 0, .cols = 4, .pixels = {}}, unit)),
         "blur rejects an empty image", failures);
  expect(is_invalid_input(image::blur(gray, std::vector<std::uint32_t>{1, 1})),
         "blur rejects an even kernel", failures);
  expect(is_invalid_input(image::blur(gray, std::vector<std::uint32_t>{1, 1, 1})),
         "blur rejects a sum that is not a power of two", failures);
  expect(is_invalid_input(image::blur(gray, std::vector<std::uint32_t>{1U << 16U})),
         "blur rejects a sum above 2^15", failures);
  expect(is_invalid_input(image::resize(gray, 0, 4)), "resize rejects zero rows", failures);
  expect(is_invalid_input(image::resize(gray, 3, image::kMaxSide + 1)),
         "resize rejects wide output", failures);
  expect(is_invalid_input(image::to_gray(2, 2, std::vector<std::uint8_t>(11))),
         "to_gray rejects a short buffer", failures);
  image::Coordinates nan(1, 2);
  nan << std::numeric_limits<double>::quiet_NaN(), 0.0;
  expect(is_invalid_input(image::remap(gray, nan, 1, 1)), "remap rejects NaN", failures);
  expect(is_invalid_input(image::remap(gray, image::Coordinates::Zero(2, 2), 1, 1)),
         "remap rejects a map of another size", failures);
  expect(is_invalid_input(image::pyramid(gray, 0, 1.2)), "pyramid rejects zero levels", failures);
  expect(is_invalid_input(image::pyramid(gray, 2, 1.0)), "pyramid rejects a scale of 1", failures);
  expect(is_invalid_input(image::pyramid(gray, 2, std::numeric_limits<double>::infinity())),
         "pyramid rejects an infinite scale", failures);
  expect(is_invalid_input(image::pyramid(gray, 3, 4.0)), "pyramid rejects an empty level",
         failures);
}

/// @brief The test body.
/// @return The exit code.
/// @brief Check the two coordinate maps: inverses, level 0 the identity, and their
///        composition the level-to-level map stereo matching uses.
/// @param failures Running failure count.
void check_coordinate_maps(int& failures) {
  namespace image = slamlab::foundations::image;
  constexpr std::array<std::int64_t, 8> kLevels{640, 533, 444, 370, 309, 257, 214, 179};
  expect(image::level_coordinate(7.0, 640, 640) == 7.0 &&
             image::level_coordinate(2.5, 640, 640) == 2.5,
         "level_coordinate maps level 0 to itself", failures);
  bool inverse = true;
  bool composed = true;
  for (const std::int64_t a : kLevels) {
    for (const std::int64_t b : kLevels) {
      for (std::int64_t x = 0; x < a; x += 7) {
        const double u = image::full_image_coordinate(static_cast<double>(x), a, 640);
        inverse = inverse &&
                  std::fabs(image::level_coordinate(u, a, 640) - static_cast<double>(x)) < 1e-12;
        const double on_b = static_cast<double>(((2 * x) + 1) * b - a) / static_cast<double>(2 * a);
        composed = composed && std::fabs(image::level_coordinate(u, b, 640) - on_b) < 1e-12;
      }
    }
  }
  expect(inverse, "level_coordinate inverts full_image_coordinate", failures);
  expect(composed, "the two maps compose to the level-to-level map", failures);
}

int run() {
  int failures = 0;
  check_fixture(failures);
  check_rejections(failures);
  check_coordinate_maps(failures);
  return slamlab::testing::finish("foundations/image", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("foundations/image", run); }
