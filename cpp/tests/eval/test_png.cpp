/// @file
/// @brief Native tests for the PNG reader (slamlab/eval/png.hpp), against the shared fixtures
///        `tests/fixtures/eval/png/` and their expected values `png.json`, which the Python and
///        Rust tests read too.
#include <cstdint>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slamlab/eval/png.hpp"

namespace {

namespace png = slamlab::eval::png;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;

/// @brief A fixture file's path.
/// @param name The file's name.
/// @return Its path under `tests/fixtures/eval/png/`.
std::string fixture_png(const std::string& name) {
  return std::string(SLAMLAB_FIXTURES_DIR) + "/eval/png/" + name;
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json cases = slamlab::testing::load_fixture("eval/png.json")["cases"];
  for (const auto& [name, expected] : cases.items()) {
    if (expected.contains("gray")) {
      const auto gray = png::read_gray(fixture_png(name));
      expect(gray.has_value() &&
                 gray->pixels == slamlab::testing::flatten_rows<std::uint8_t>(expected["gray"]),
             ("read_gray matches the fixture: " + name).c_str(), failures);
      expect(is_invalid_input(png::read16(fixture_png(name))),
             ("read16 refuses an 8-bit file: " + name).c_str(), failures);
    } else {
      const auto depth = png::read16(fixture_png(name));
      expect(depth.has_value() &&
                 depth->values == slamlab::testing::flatten_rows<std::uint16_t>(expected["depth"]),
             ("read16 matches the fixture: " + name).c_str(), failures);
      expect(is_invalid_input(png::read8(fixture_png(name))),
             ("read8 refuses a 16-bit file: " + name).c_str(), failures);
    }
  }
  const auto rgb = png::read8(fixture_png("rgb.png"));
  expect(rgb.has_value() && rgb->channels == 3 && rgb->rows == 6 && rgb->cols == 8,
         "read8 keeps an RGB file's channels", failures);
  expect(is_invalid_input(png::read_gray(fixture_png("missing.png"))),
         "read_gray refuses a missing file", failures);
  return slamlab::testing::finish("eval/png", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("eval/png", run); }
