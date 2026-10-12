/// @file
/// @brief Grayscale images in fixed point: the C++ core's mirror of
///        `slam_it.foundations.image`.
///
/// Colour conversion, a separable integer blur, bilinear resizing, pyramids and remapping,
/// all in integer arithmetic so the three languages' images are bit-identical (the Python
/// module's docstring defines each operation and its rounding). Pixel `(x, y)` is
/// `pixels[y * cols + x]`; the centre of the top-left pixel is `(0, 0)`.
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/foundations/error.hpp"

/// @brief Grayscale images.
namespace slam_it::foundations::image {

/// @brief `2^bits`: a fixed-point scale.
/// @param bits The fixed-point bits, from 0 to 62.
/// @return The scale.
[[nodiscard]] constexpr std::int64_t pow2(int bits) {
  std::int64_t scale = 1;
  for (int i = 0; i < bits; ++i) {
    scale *= 2;
  }
  return scale;
}

/// @brief Divide, rounding toward negative infinity (as Python's `//` and `>>`).
/// @param numerator The numerator.
/// @param denominator The denominator, positive.
/// @return `floor(numerator / denominator)`.
[[nodiscard]] constexpr std::int64_t floor_div(std::int64_t numerator, std::int64_t denominator) {
  const std::int64_t quotient = numerator / denominator;
  return (numerator % denominator != 0 && numerator < 0) ? quotient - 1 : quotient;
}

/// @brief BT.601's red, green and blue weights over `2^kGrayBits` (OpenCV's `cvtColor`).
inline constexpr std::array<std::int64_t, 3> kGrayWeights{9798, 19235, 3735};
/// @brief The fixed-point bits of `kGrayWeights`.
inline constexpr int kGrayBits = 15;
/// @brief The fixed-point bits of a bilinear resize's weights.
inline constexpr int kResizeBits = 11;
/// @brief The fixed-point bits of a remap's coordinates: a 32nd of a pixel.
inline constexpr int kRemapBits = 5;
/// @brief A blur kernel's weights sum to `2^b` with `b` at most this.
inline constexpr int kMaxKernelBits = 15;
/// @brief The largest image side, in pixels (`slam_it.foundations.arrays.MAX_IMAGE_SIDE`):
///        the integer arithmetic's intermediate values then fit 64 bits.
inline constexpr std::int64_t kMaxSide = pow2(20);

/// @brief Check a pyramid's scale factor (`slam_it.foundations.image.as_scale_factor`).
/// @param scale_factor The factor.
/// @param name Its name, for the message.
/// @return Nothing, or `InvalidInput` unless it is finite and above 1.
[[nodiscard]] Result<void> check_scale_factor(double scale_factor, const char* name);

/// @brief A pyramid-level position's coordinate in the full image, along one axis
///        (`slam_it.foundations.image.full_image_coordinate`): the resizes' aligned-centre maps
///        composed, `((2 x + 1) n_0 - n_l) / (2 n_l)`, computed in that order. For an integer
///        `x` only the division rounds, and level 0 maps to itself.
/// @param x The position on the level, in its pixels.
/// @param level_size The level's size along the axis, `n_l`.
/// @param size The full image's size along it, `n_0`.
/// @return The full-image coordinate.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (level_size, size), as in Python
[[nodiscard]] constexpr double full_image_coordinate(double x, std::int64_t level_size,
                                                     std::int64_t size) {
  const auto level = static_cast<double>(level_size);
  return ((((2.0 * x) + 1.0) * static_cast<double>(size)) - level) / (2.0 * level);
}

/// @brief A full-image position's coordinate on a pyramid level, along one axis
///        (`slam_it.foundations.image.level_coordinate`): `full_image_coordinate`'s inverse,
///        `((2 u + 1) n_l - n_0) / (2 n_0)`, computed in that order. Level 0 maps to itself.
/// @param u The position in the full image, in its pixels.
/// @param level_size The level's size along the axis, `n_l`.
/// @param size The full image's size along it, `n_0`.
/// @return The level coordinate.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (level_size, size), as in Python
[[nodiscard]] constexpr double level_coordinate(double u, std::int64_t level_size,
                                                std::int64_t size) {
  const auto full = static_cast<double>(size);
  return ((((2.0 * u) + 1.0) * static_cast<double>(level_size)) - full) / (2.0 * full);
}

/// @brief A grayscale image, row-major.
struct Image {
  /// @brief The number of rows (the height), at least 1.
  std::int64_t rows{0};
  /// @brief The number of columns (the width), at least 1.
  std::int64_t cols{0};
  /// @brief The intensities, `rows * cols` of them, row by row.
  std::vector<std::uint8_t> pixels;
};

/// @brief Read one pixel of an image.
/// @param image The image.
/// @param row The pixel's row, in `[0, rows)`.
/// @param col The pixel's column, in `[0, cols)`.
/// @return Its intensity.
[[nodiscard]] inline std::uint8_t pixel(const Image& image, std::int64_t row, std::int64_t col) {
  return image.pixels[static_cast<std::size_t>((row * image.cols) + col)];
}

/// @brief Coordinates `(x, y)`, one per row, row-major as numpy stores them.
using Coordinates = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

/// @brief Check an image: sides from 1 to `kMaxSide`, and `rows * cols` pixels.
/// @param image The image.
/// @param name The argument's name, for the message.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_image(const Image& image, const char* name);

/// @brief Check a blur kernel and find `b` with its weights summing to `2^b`.
/// @param kernel The weights.
/// @return `b`, or `InvalidInput` for an even count or a sum that is not a power of two up to
///         `2^kMaxKernelBits`.
[[nodiscard]] Result<int> kernel_bits(std::span<const std::uint32_t> kernel);

/// @brief Each pyramid level's `(rows, cols)`: `round(n / s^l)`, half up, `s^l` by repeated
///        multiplication.
/// @param rows Level 0's rows.
/// @param cols Level 0's columns.
/// @param levels The number of levels, at least 1.
/// @param scale_factor Each level's downscale from the previous one, above 1.
/// @return One shape per level, or `InvalidInput` if a setting is out of range or a level
///         would be empty.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (rows, cols), as numpy's shape
[[nodiscard]] Result<std::vector<std::array<std::int64_t, 2>>> level_shapes(std::int64_t rows,
                                                                            std::int64_t cols,
                                                                            std::int64_t levels,
                                                                            double scale_factor);

/// @brief Convert an RGB image to gray: `Y = 0.299 R + 0.587 G + 0.114 B` in 15-bit integers.
/// @param rows The image's rows.
/// @param cols The image's columns.
/// @param rgb Interleaved red, green and blue, `rows * cols * 3` values, row by row.
/// @return The gray image, or `InvalidInput` for a bad shape.
[[nodiscard]] Result<Image> to_gray(std::int64_t rows, std::int64_t cols,
                                    std::span<const std::uint8_t> rgb);

/// @brief Filter separably with integer weights summing to a power of two, borders reflected
///        without repeating the edge.
/// @param image The image.
/// @param kernel An odd number of weights, applied along rows, then along columns.
/// @return The filtered image, rounded half up; `InvalidInput` for a bad image or kernel.
[[nodiscard]] Result<Image> blur(const Image& image, std::span<const std::uint32_t> kernel);

/// @brief Resize bilinearly, pixel centres aligned, in 11-bit fixed point.
/// @param image The image.
/// @param rows The output's rows, from 1 to `kMaxSide`.
/// @param cols The output's columns, from 1 to `kMaxSide`.
/// @return The resized image, or `InvalidInput`.
[[nodiscard]] Result<Image> resize(const Image& image, std::int64_t rows, std::int64_t cols);

/// @brief The image and its successively downscaled levels, each resized from the previous.
/// @param image Level 0.
/// @param levels The number of levels, at least 1.
/// @param scale_factor Each level's downscale, above 1.
/// @return The levels, or `InvalidInput` (see `level_shapes`).
[[nodiscard]] Result<std::vector<Image>> pyramid(const Image& image, std::int64_t levels,
                                                 double scale_factor);

/// @brief Check a pyramid (`slam_it.foundations.image.as_pyramid`): `count` images, each a
///        valid image of the shape `level_shapes` gives from level 0's.
/// @param levels The levels.
/// @param count The number of levels expected.
/// @param scale_factor The pyramid's scale factor.
/// @param name The argument's name, for the message.
/// @return Nothing, or `InvalidInput` for another number of levels, a malformed level, one of
///         another shape, or settings `level_shapes` refuses.
[[nodiscard]] Result<void> check_pyramid(std::span<const Image> levels, std::int64_t count,
                                         double scale_factor, const char* name);

/// @brief Sample an image bilinearly at a map of coordinates, rounded to a 32nd of a pixel;
///        neighbours outside the image count as 0.
/// @param image The image.
/// @param coordinates Each output pixel's source `(x, y)`, row by row (`rows * cols` rows).
/// @param rows The output's rows.
/// @param cols The output's columns.
/// @return The sampled image, or `InvalidInput` for a bad shape or a non-finite coordinate.
[[nodiscard]] Result<Image> remap(const Image& image,
                                  const Eigen::Ref<const Coordinates>& coordinates,
                                  std::int64_t rows, std::int64_t cols);

}  // namespace slam_it::foundations::image
