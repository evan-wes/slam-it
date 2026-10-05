/// @file
/// @brief Reading PNG files natively: the C++ side of `slamlab.eval.images`.
///
/// Decoded by the vendored stb_image (PNG only, compiled private to this library). A file
/// reads the way Python's Pillow-based reader reads it, under the same names: `read_gray`
/// gives gray from an 8-bit gray file as stored (without its alpha), or from an 8-bit colour
/// file (RGB, RGBA, or a palette expanded to RGB) through `image::to_gray`; `read_rgb` gives
/// an 8-bit file's colour (gray repeated to three channels, alpha dropped); `read_depth` a
/// 16-bit single-channel file as stored.
///
/// Every failure is `InvalidInput`, its message naming the file: a missing or unreadable file
/// ("no such file, or not readable"), one stb_image cannot decode ("not a readable PNG"), or
/// the wrong bit depth. Python raises `FileNotFoundError` for a missing file instead; the
/// cores have no I/O error kind. stb_image does not check PNG checksums, so a file whose only
/// damage is a bad CRC decodes here, where Pillow and Rust's `png` crate refuse it.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/image.hpp"

/// @brief Reading PNG files (evaluation: dataset readers outside the algorithm cores).
namespace slamlab::eval::png {

/// @brief An 8-bit colour image.
struct RgbImage {
  /// @brief Rows.
  std::int64_t rows{0};
  /// @brief Columns.
  std::int64_t cols{0};
  /// @brief The values, row by row, each pixel's red, green and blue together.
  std::vector<std::uint8_t> values;
};

/// @brief A 16-bit single-channel image.
struct DepthImage {
  /// @brief Rows.
  std::int64_t rows{0};
  /// @brief Columns.
  std::int64_t cols{0};
  /// @brief The values, row by row.
  std::vector<std::uint16_t> values;
};

/// @brief Read an 8-bit PNG as gray, as `slamlab.eval.images.read_gray` does.
/// @param path The file.
/// @return The gray image, or `InvalidInput` if the file cannot be read or is not 8-bit.
[[nodiscard]] foundations::Result<foundations::image::Image> read_gray(const std::string& path);

/// @brief Read an 8-bit PNG as colour, as `slamlab.eval.images.read_rgb` does.
/// @param path The file.
/// @return The colour image, or `InvalidInput` if the file cannot be read or is not 8-bit.
[[nodiscard]] foundations::Result<RgbImage> read_rgb(const std::string& path);

/// @brief Read a 16-bit single-channel PNG, as `slamlab.eval.images.read_depth` does.
/// @param path The file.
/// @return The image, or `InvalidInput` if the file cannot be read or is not 16-bit gray.
[[nodiscard]] foundations::Result<DepthImage> read_depth(const std::string& path);

}  // namespace slamlab::eval::png
