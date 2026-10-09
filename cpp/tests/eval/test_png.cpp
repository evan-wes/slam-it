/// @file
/// @brief Native tests for the PNG reader (slam_it_all/eval/png.hpp), against the shared fixtures
///        `tests/fixtures/eval/png/` and their expected values `png.json`, which the Python and
///        Rust tests read too, and its failures.
#include <cstdint>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it_all/eval/png.hpp"

namespace {

namespace png = slam_it_all::eval::png;
using slam_it_all::testing::expect;
using slam_it_all::testing::is_invalid_input;

/// @brief A fixture file's path.
/// @param name The file's name.
/// @return Its path under `tests/fixtures/eval/png/`.
std::string fixture_png(const std::string& name) {
  return std::string(SLAM_IT_ALL_FIXTURES_DIR) + "/eval/png/" + name;
}

/// @brief Flatten an (H, W, 3) nested list.
/// @param image The rows of pixels of channels.
/// @return The values, row by row, a pixel's channels together.
std::vector<std::uint8_t> flatten_pixels(const nlohmann::json& image) {
  std::vector<std::uint8_t> out;
  for (const auto& row : image) {
    for (const auto& pixel : row) {
      for (const auto& value : pixel) {
        out.push_back(value.get<std::uint8_t>());
      }
    }
  }
  return out;
}

/// @brief Whether a failure's message contains a phrase.
/// @tparam T The result's value type.
/// @param result The result.
/// @param phrase The phrase.
/// @return True for an `InvalidInput` failure whose message holds `phrase`.
template <typename T>
bool fails_with(const slam_it_all::foundations::Result<T>& result, const std::string& phrase) {
  return is_invalid_input(result) && result.error().message.find(phrase) != std::string::npos;
}

/// @brief Check every fixture file against its expected values, and bit depths refused.
/// @param failures Running failure count.
void check_fixture(int& failures) {
  const nlohmann::json cases = slam_it_all::testing::load_fixture("eval/png.json")["cases"];
  for (const auto& [name, expected] : cases.items()) {
    if (expected.contains("gray")) {
      const auto gray = png::read_gray(fixture_png(name));
      expect(gray.has_value() &&
                 gray->pixels == slam_it_all::testing::flatten_rows<std::uint8_t>(expected["gray"]),
             ("read_gray matches the fixture: " + name).c_str(), failures);
      const auto rgb = png::read_rgb(fixture_png(name));
      expect(rgb.has_value() && rgb->values == flatten_pixels(expected["rgb"]),
             ("read_rgb matches the fixture: " + name).c_str(), failures);
      expect(fails_with(png::read_depth(fixture_png(name)), "expected a 16-bit"),
             ("read_depth refuses an 8-bit file: " + name).c_str(), failures);
    } else if (expected.contains("unreadable")) {
      for (const bool refused :
           {fails_with(png::read_gray(fixture_png(name)), "not a readable"),
            fails_with(png::read_rgb(fixture_png(name)), "not a readable"),
            fails_with(png::read_depth(fixture_png(name)), "not a readable")}) {
        expect(refused, ("every reader refuses: " + name).c_str(), failures);
      }
    } else {
      const auto depth = png::read_depth(fixture_png(name));
      expect(
          depth.has_value() &&
              depth->values == slam_it_all::testing::flatten_rows<std::uint16_t>(expected["depth"]),
          ("read_depth matches the fixture: " + name).c_str(), failures);
      expect(fails_with(png::read_gray(fixture_png(name)), "expected an 8-bit"),
             ("read_gray refuses a 16-bit file: " + name).c_str(), failures);
    }
  }
}

/// @brief Check that missing and non-PNG files are refused, each named.
/// @param failures Running failure count.
void check_failures(int& failures) {
  const std::string missing = fixture_png("missing.png");
  expect(fails_with(png::read_gray(missing), "no such file"), "read_gray: a missing file",
         failures);
  expect(fails_with(png::read_depth(missing), "no such file"), "read_depth: a missing file",
         failures);
  const std::string json = std::string(SLAM_IT_ALL_FIXTURES_DIR) + "/eval/png.json";
  expect(fails_with(png::read_depth(json), "not a readable PNG"), "read_depth: not a PNG",
         failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_fixture(failures);
  check_failures(failures);
  return slam_it_all::testing::finish("eval/png", failures);
}

}  // namespace

int main() { return slam_it_all::testing::guarded("eval/png", run); }
