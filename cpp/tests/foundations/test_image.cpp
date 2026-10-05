/// @file
/// @brief Native tests for the fixed-point image operations (slamlab/foundations/image.hpp),
///        against the parity fixture `tests/fixtures/foundations/image.json` that the Python
///        and Rust tests share, and the core's own input checks.
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "fixture_support.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"
#include "test_support.hpp"

namespace {

namespace image = slamlab::foundations::image;
using slamlab::foundations::ErrorKind;
using slamlab::testing::expect;

/// @brief Read a nested list of intensities as an image.
/// @param rows The fixture's list of rows.
/// @return The image.
image::Image read_image(const nlohmann::json& rows) {
  image::Image out{.rows = static_cast<std::int64_t>(rows.size()),
                   .cols = static_cast<std::int64_t>(rows[0].size()),
                   .pixels = {}};
  for (const auto& row : rows) {
    for (const auto& value : row) {
      out.pixels.push_back(value.get<std::uint8_t>());
    }
  }
  return out;
}

/// @brief Whether a result holds exactly the expected image.
/// @param got The result.
/// @param expected The image.
/// @return True if they are equal, shape and pixels.
bool same(const slamlab::foundations::Result<image::Image>& got, const image::Image& expected) {
  return got.has_value() && got->rows == expected.rows && got->cols == expected.cols &&
         got->pixels == expected.pixels;
}

/// @brief Whether a result failed with `InvalidInput`.
/// @tparam T The result's value type.
/// @param result The result.
/// @return True if it is an `InvalidInput` error.
template <typename T>
bool invalid(const slamlab::foundations::Result<T>& result) {
  return !result.has_value() && result.error().kind == ErrorKind::kInvalidInput;
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
  expect(invalid(image::blur(short_image, unit)), "blur rejects a short buffer", failures);
  expect(invalid(image::blur(image::Image{.rows = 0, .cols = 4, .pixels = {}}, unit)),
         "blur rejects an empty image", failures);
  expect(invalid(image::blur(gray, std::vector<std::uint32_t>{1, 1})),
         "blur rejects an even kernel", failures);
  expect(invalid(image::blur(gray, std::vector<std::uint32_t>{1, 1, 1})),
         "blur rejects a sum that is not a power of two", failures);
  expect(invalid(image::blur(gray, std::vector<std::uint32_t>{1U << 16U})),
         "blur rejects a sum above 2^15", failures);
  expect(invalid(image::resize(gray, 0, 4)), "resize rejects zero rows", failures);
  expect(invalid(image::resize(gray, 3, image::kMaxSide + 1)), "resize rejects wide output",
         failures);
  expect(invalid(image::to_gray(2, 2, std::vector<std::uint8_t>(11))),
         "to_gray rejects a short buffer", failures);
  image::Coordinates nan(1, 2);
  nan << std::numeric_limits<double>::quiet_NaN(), 0.0;
  expect(invalid(image::remap(gray, nan, 1, 1)), "remap rejects NaN", failures);
  expect(invalid(image::remap(gray, image::Coordinates::Zero(2, 2), 1, 1)),
         "remap rejects a map of another size", failures);
  expect(invalid(image::pyramid(gray, 0, 1.2)), "pyramid rejects zero levels", failures);
  expect(invalid(image::pyramid(gray, 2, 1.0)), "pyramid rejects a scale of 1", failures);
  expect(invalid(image::pyramid(gray, 2, std::numeric_limits<double>::infinity())),
         "pyramid rejects an infinite scale", failures);
  expect(invalid(image::pyramid(gray, 3, 4.0)), "pyramid rejects an empty level", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_fixture(failures);
  check_rejections(failures);
  return slamlab::testing::finish("foundations/image", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("foundations/image", run); }
