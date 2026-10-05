/// @file
/// @brief Implementation of the PNG reading declared in slamlab/eval/png.hpp, on stb_image.
#include "slamlab/eval/png.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <utility>

// stb_image, PNG only: the one translation unit that compiles its implementation.
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

/// @brief The failure for a file stb_image could not decode.
/// @param path The file.
/// @return `InvalidInput` naming the file and stb_image's reason.
std::unexpected<foundations::Error> unreadable(const std::string& path) {
  return fail(ErrorKind::kInvalidInput,
              path + ": not a readable PNG (" + stbi_failure_reason() + ")");
}

}  // namespace

Result<Raster8> read8(const std::string& path) {
  if (stbi_is_16_bit(path.c_str()) != 0) {
    return fail(ErrorKind::kInvalidInput, path + ": expected an 8-bit image");
  }
  int cols = 0;
  int rows = 0;
  int channels = 0;
  const std::unique_ptr<stbi_uc, StbFree> data(stbi_load(path.c_str(), &cols, &rows, &channels, 0));
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

Result<Raster16> read16(const std::string& path) {
  if (stbi_is_16_bit(path.c_str()) == 0) {
    return fail(ErrorKind::kInvalidInput, path + ": expected a 16-bit single-channel image");
  }
  int cols = 0;
  int rows = 0;
  int channels = 0;
  const std::unique_ptr<stbi_us, StbFree> data(
      stbi_load_16(path.c_str(), &cols, &rows, &channels, 0));
  if (!data) {
    return unreadable(path);
  }
  if (channels != 1) {
    return fail(ErrorKind::kInvalidInput, path + ": expected a 16-bit single-channel image");
  }
  const auto size = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  const std::span<const stbi_us> decoded(data.get(), size);
  return Raster16{.rows = rows,
                  .cols = cols,
                  .values = std::vector<std::uint16_t>(decoded.begin(), decoded.end())};
}

Result<foundations::image::Image> read_gray(const std::string& path) {
  auto raster = read8(path);
  if (!raster) {
    return std::unexpected(std::move(raster.error()));
  }
  const auto pixels = static_cast<std::size_t>(raster->rows * raster->cols);
  const auto channels = static_cast<std::size_t>(raster->channels);
  if (channels <= 2) {  // gray, perhaps with alpha: the gray channel as stored
    foundations::image::Image out{.rows = raster->rows, .cols = raster->cols, .pixels = {}};
    out.pixels.reserve(pixels);
    for (std::size_t i = 0; i < pixels; ++i) {
      out.pixels.push_back(raster->values[i * channels]);
    }
    return out;
  }
  std::vector<std::uint8_t> rgb;
  rgb.reserve(3 * pixels);
  for (std::size_t i = 0; i < pixels; ++i) {
    for (std::size_t c = 0; c < 3; ++c) {
      rgb.push_back(raster->values[(i * channels) + c]);
    }
  }
  return foundations::image::to_gray(raster->rows, raster->cols, rgb);
}

}  // namespace slamlab::eval::png
