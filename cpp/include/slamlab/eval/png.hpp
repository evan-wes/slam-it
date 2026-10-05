/// @file
/// @brief Reading PNG files natively: the C++ side of `slamlab.eval.images`.
///
/// Decoded by the vendored stb_image (PNG only). A file reads the way Python's Pillow-based
/// reader reads it: gray from an 8-bit gray file as stored (without its alpha), from an 8-bit
/// colour file (RGB, RGBA, or a palette expanded to RGB) through `image::to_gray`; depth from
/// a 16-bit single-channel file as stored.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"

/// @brief Reading PNG files (evaluation: dataset readers outside the algorithm cores).
namespace slamlab::eval::png {

/// @brief An 8-bit image as stored: 1 (gray), 2 (gray, alpha), 3 (RGB) or 4 (RGBA) channels.
struct Raster8 {
  /// @brief Rows.
  std::int64_t rows{0};
  /// @brief Columns.
  std::int64_t cols{0};
  /// @brief Channels per pixel.
  std::int64_t channels{0};
  /// @brief The values, row by row, the channels of a pixel together.
  std::vector<std::uint8_t> values;
};

/// @brief A 16-bit single-channel image.
struct Raster16 {
  /// @brief Rows.
  std::int64_t rows{0};
  /// @brief Columns.
  std::int64_t cols{0};
  /// @brief The values, row by row.
  std::vector<std::uint16_t> values;
};

/// @brief Read an 8-bit PNG as stored (a palette expanded to RGB or RGBA).
/// @param path The file.
/// @return The raster, or `InvalidInput` if the file cannot be read or is not 8-bit.
[[nodiscard]] foundations::Result<Raster8> read8(const std::string& path);

/// @brief Read a 16-bit single-channel PNG.
/// @param path The file.
/// @return The raster, or `InvalidInput` if the file cannot be read or is not 16-bit gray.
[[nodiscard]] foundations::Result<Raster16> read16(const std::string& path);

/// @brief Read an 8-bit PNG as gray, as `slamlab.eval.images.read_gray` does.
/// @param path The file.
/// @return The gray image, or `InvalidInput` (see `read8`).
[[nodiscard]] foundations::Result<foundations::image::Image> read_gray(const std::string& path);

}  // namespace slamlab::eval::png
