/// @file
/// @brief Reading images from the shared parity fixtures in the native tests.
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>

#include "slamlab/foundations/image.hpp"

/// @brief Helpers for slamlab's native tests.
namespace slamlab::testing {

/// @brief Read a fixture's nested list of intensities as an image.
/// @param rows The list of rows, each a list of intensities.
/// @return The image.
inline foundations::image::Image read_image(const nlohmann::json& rows) {
  foundations::image::Image out{.rows = static_cast<std::int64_t>(rows.size()),
                                .cols = static_cast<std::int64_t>(rows[0].size()),
                                .pixels = {}};
  for (const auto& row : rows) {
    for (const auto& value : row) {
      out.pixels.push_back(value.get<std::uint8_t>());
    }
  }
  return out;
}

}  // namespace slamlab::testing
