/// @file
/// @brief Implementation of the PNG reading declared in slamlab/eval/png.hpp, on stb_image.
#include "slamlab/eval/png.hpp"

#include <cstddef>
#include <fstream>
#include <ios>
#include <iterator>
#include <limits>
#include <memory>
#include <span>
#include <utility>

// stb_image, PNG only: the one translation unit that compiles its implementation, every
// function static to it, so no stbi_* symbol leaves the library: another copy of stb_image
// in a program can neither clash with it nor share its process-wide settings.
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace slamlab::eval::png {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

/// @brief Frees stb_image's buffers.
struct StbFree {
  /// @brief Free a buffer.
  /// @param data The buffer stb_image returned.
  void operator()(void* data) const noexcept { stbi_image_free(data); }
};

/// @brief A PNG file's bytes, read once, and what its header says.
struct Encoded {
  /// @brief The file's bytes.
  std::vector<stbi_uc> bytes;
  /// @brief Their count, as stb_image takes it.
  int size{0};
  /// @brief Whether the header says 16 bits per channel.
  bool sixteen_bit{false};
};

/// @brief The failure for a file stb_image could not decode.
/// @param path The file.
/// @return `InvalidInput` naming the file and stb_image's reason.
std::unexpected<foundations::Error> unreadable(const std::string& path) {
  return fail(ErrorKind::kInvalidInput,
              path + ": not a readable PNG (" + stbi_failure_reason() + ")");
}

/// @brief Read a file whole and check that its header is a PNG's.
/// @param path The file.
/// @return Its bytes and bit depth, or `InvalidInput` if it cannot be opened or read, or is
///         not a PNG.
Result<Encoded> encoded(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return fail(ErrorKind::kInvalidInput, path + ": no such file, or not readable");
  }
  std::vector<stbi_uc> bytes((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  if (file.bad() || bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return fail(ErrorKind::kInvalidInput, path + ": could not be read whole");
  }
  const auto size = static_cast<int>(bytes.size());
  int cols = 0;
  int rows = 0;
  int channels = 0;
  if (stbi_info_from_memory(bytes.data(), size, &cols, &rows, &channels) == 0) {
    return unreadable(path);
  }
  const bool sixteen_bit = stbi_is_16_bit_from_memory(bytes.data(), size) != 0;
  return Encoded{.bytes = std::move(bytes), .size = size, .sixteen_bit = sixteen_bit};
}

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

/// @brief Read an 8-bit PNG as stored (a palette expanded to RGB or RGBA).
/// @param path The file.
/// @return The raster, or `InvalidInput` if the file cannot be read or is not 8-bit.
Result<Raster8> read8(const std::string& path) {
  const auto file = encoded(path);
  if (!file) {
    return std::unexpected(file.error());
  }
  if (file->sixteen_bit) {
    return fail(ErrorKind::kInvalidInput, path + ": expected an 8-bit image");
  }
  int cols = 0;
  int rows = 0;
  int channels = 0;
  const std::unique_ptr<stbi_uc, StbFree> data(
      stbi_load_from_memory(file->bytes.data(), file->size, &cols, &rows, &channels, 0));
  if (!data) {
    return unreadable(path);
  }
  const auto size = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols) *
                    static_cast<std::size_t>(channels);
  const std::span<const stbi_uc> decoded(data.get(), size);
  return Raster8{.rows = rows,
                 .cols = cols,
                 .channels = channels,
                 .values = std::vector<std::uint8_t>(decoded.begin(), decoded.end())};
}

/// @brief The colour of an 8-bit raster: gray repeated to three channels, alpha dropped.
/// @param raster The raster.
/// @return Three values per pixel, red first.
std::vector<std::uint8_t> rgb_values(const Raster8& raster) {
  const auto pixels = static_cast<std::size_t>(raster.rows * raster.cols);
  const auto channels = static_cast<std::size_t>(raster.channels);
  std::vector<std::uint8_t> out;
  out.reserve(3 * pixels);
  for (std::size_t i = 0; i < pixels; ++i) {
    for (std::size_t c = 0; c < 3; ++c) {
      // Gray (with or without alpha) repeats channel 0; colour takes channels 0 to 2.
      out.push_back(raster.values[(i * channels) + (channels <= 2 ? 0 : c)]);
    }
  }
  return out;
}

}  // namespace

Result<foundations::image::Image> read_gray(const std::string& path) {
  const auto raster = read8(path);
  if (!raster) {
    return std::unexpected(raster.error());
  }
  if (raster->channels <= 2) {  // gray, perhaps with alpha: the gray channel as stored
    const auto pixels = static_cast<std::size_t>(raster->rows * raster->cols);
    const auto channels = static_cast<std::size_t>(raster->channels);
    foundations::image::Image out{.rows = raster->rows, .cols = raster->cols, .pixels = {}};
    out.pixels.reserve(pixels);
    for (std::size_t i = 0; i < pixels; ++i) {
      out.pixels.push_back(raster->values[i * channels]);
    }
    return out;
  }
  return foundations::image::to_gray(raster->rows, raster->cols, rgb_values(*raster));
}

Result<RgbImage> read_rgb(const std::string& path) {
  const auto raster = read8(path);
  if (!raster) {
    return std::unexpected(raster.error());
  }
  return RgbImage{.rows = raster->rows, .cols = raster->cols, .values = rgb_values(*raster)};
}

Result<DepthImage> read_depth(const std::string& path) {
  const auto file = encoded(path);
  if (!file) {
    return std::unexpected(file.error());
  }
  if (!file->sixteen_bit) {
    return fail(ErrorKind::kInvalidInput, path + ": expected a 16-bit single-channel image");
  }
  int cols = 0;
  int rows = 0;
  int channels = 0;
  const std::unique_ptr<stbi_us, StbFree> data(
      stbi_load_16_from_memory(file->bytes.data(), file->size, &cols, &rows, &channels, 0));
  if (!data) {
    return unreadable(path);
  }
  if (channels != 1) {
    return fail(ErrorKind::kInvalidInput, path + ": expected a 16-bit single-channel image");
  }
  const auto size = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  const std::span<const stbi_us> decoded(data.get(), size);
  return DepthImage{.rows = rows,
                    .cols = cols,
                    .values = std::vector<std::uint16_t>(decoded.begin(), decoded.end())};
}

}  // namespace slamlab::eval::png
