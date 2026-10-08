/// @file
/// @brief Implementation of the fixed-point image operations declared in
///        slamlab/foundations/image.hpp.
#include "slamlab/foundations/image.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <span>
#include <sstream>
#include <string>
#include <utility>

namespace slamlab::foundations::image {

namespace {

/// @brief A vector's index from a signed count.
/// @param index The index, non-negative.
/// @return It as a `std::size_t`.
std::size_t at(std::int64_t index) { return static_cast<std::size_t>(index); }

/// @brief An image of zeros.
/// @param rows Its rows.
/// @param cols Its columns.
/// @return The image.
Image blank(std::int64_t rows, std::int64_t cols) {
  return Image{.rows = rows, .cols = cols, .pixels = std::vector<std::uint8_t>(at(rows * cols))};
}

/// @brief Round a fixed-point value half up and drop its fraction bits.
/// @param value The value, from 0 to `255 * 2^bits`.
/// @param bits Its fraction bits.
/// @return `round(value / 2^bits)`, as an intensity.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (value, bits), as a shift
std::uint8_t round_shift(std::int64_t value, int bits) {
  const std::int64_t scale = pow2(bits);
  return static_cast<std::uint8_t>((value + (scale / 2)) / scale);
}

/// @brief Check an image's shape and its number of values.
/// @param rows Its rows, from 1 to `kMaxSide`.
/// @param cols Its columns, from 1 to `kMaxSide`.
/// @param channels Its values per pixel.
/// @param size Its number of values, `rows * cols * channels`.
/// @param name Its name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_values(std::int64_t rows, std::int64_t cols, std::int64_t channels,
                          std::size_t size, const char* name) {
  if (rows < 1 || cols < 1 || rows > kMaxSide || cols > kMaxSide) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": sides must be from 1 to " +
                                              std::to_string(kMaxSide) + ", got " +
                                              std::to_string(rows) + "x" + std::to_string(cols));
  }
  if (std::cmp_not_equal(size, rows * cols * channels)) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected " +
                                              std::to_string(rows * cols * channels) +
                                              " values, got " + std::to_string(size));
  }
  return {};
}

/// @brief Check an output side or a count: from 1 to `kMaxSide`.
/// @param value The value.
/// @param name Its name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_side(std::int64_t value, const char* name) {
  return check_range(name, value, 1, kMaxSide);
}

/// @brief Check a resampling's input image and output shape.
/// @param image The input.
/// @param rows The output's rows.
/// @param cols The output's columns.
/// @return Nothing, or `InvalidInput`.
Result<void> check_resampling(const Image& image, std::int64_t rows, std::int64_t cols) {
  if (auto ok = check_image(image, "image"); !ok) {
    return ok;
  }
  if (auto ok = check_side(rows, "rows"); !ok) {
    return ok;
  }
  return check_side(cols, "cols");
}

/// @brief Map an index outside `[0, size)` back inside, reflecting without repeating the edge
///        (OpenCV's `BORDER_REFLECT_101`).
/// @param index The index.
/// @param size The axis' length, at least 1.
/// @return The reflected index.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (index, size), as borderInterpolate
std::int64_t reflect101(std::int64_t index, std::int64_t size) {
  if (size == 1) {
    return 0;
  }
  const std::int64_t period = (2 * size) - 2;
  std::int64_t wrapped = index % period;
  if (wrapped < 0) {
    wrapped += period;
  }
  return wrapped >= size ? period - wrapped : wrapped;
}

/// @brief The reflected index of every position along an axis padded by `radius` each side.
/// @param radius The padding.
/// @param size The axis' length, at least 1.
/// @return `reflect101(i, size)` for `i` from `-radius` to `size + radius - 1`.
std::vector<std::int64_t> reflected(std::int64_t radius, std::int64_t size) {
  std::vector<std::int64_t> out;
  out.reserve(at(size + (2 * radius)));
  for (std::int64_t i = -radius; i < size + radius; ++i) {
    out.push_back(reflect101(i, size));
  }
  return out;
}

/// @brief Sum a kernel's weights times the values they apply to.
/// @tparam Read Callable as `read(k)`, returning the value under weight `k`.
/// @param kernel The weights.
/// @param read Reads the value under each weight.
/// @return `sum_k kernel[k] * read(k)`.
template <typename Read>
std::int64_t weighted_sum(std::span<const std::uint32_t> kernel, const Read& read) {
  std::int64_t sum = 0;
  for (std::size_t k = 0; k < kernel.size(); ++k) {
    sum += std::int64_t{kernel[k]} * read(k);
  }
  return sum;
}

/// @brief Where each destination index of a resized axis samples its source.
struct AxisSamples {
  /// @brief The first source index.
  std::vector<std::int64_t> first;
  /// @brief The second source index (the first's neighbour, clamped).
  std::vector<std::int64_t> second;
  /// @brief The second index's weight, over `2^kResizeBits`.
  std::vector<std::int64_t> fraction;
};

/// @brief Sample positions along one axis: destination `d` samples
///        `s = ((2d + 1) n_in - n_out) / (2 n_out)`, clamped to `[0, n_in - 1]`, its fraction
///        rounded half up to `kResizeBits` bits, all in integers.
/// @param n_in The source axis' length.
/// @param n_out The destination axis' length.
/// @return The samples.
AxisSamples axis_samples(std::int64_t n_in, std::int64_t n_out) {
  constexpr std::int64_t kOne = pow2(kResizeBits);
  AxisSamples out;
  out.first.reserve(at(n_out));
  out.second.reserve(at(n_out));
  out.fraction.reserve(at(n_out));
  const std::int64_t denominator = 2 * n_out;
  for (std::int64_t d = 0; d < n_out; ++d) {
    const std::int64_t numerator = (((2 * d) + 1) * n_in) - n_out;
    std::int64_t whole = 0;
    std::int64_t fraction = 0;
    if (numerator > 0) {  // s <= 0 clamps to the first pixel
      whole = numerator / denominator;
      const std::int64_t remainder = numerator - (whole * denominator);
      fraction = ((2 * remainder * kOne) + denominator) / (2 * denominator);
      if (fraction == kOne) {
        ++whole;
        fraction = 0;
      }
    }
    if (whole >= n_in - 1) {  // s >= n_in - 1 clamps to the last pixel
      whole = n_in - 1;
      fraction = 0;
    }
    out.first.push_back(whole);
    out.second.push_back(std::min(whole + 1, n_in - 1));
    out.fraction.push_back(fraction);
  }
  return out;
}

/// @brief Resize a checked image to a checked shape.
/// @param image The image.
/// @param rows The output's rows.
/// @param cols The output's columns.
/// @return The resized image.
Image resize_checked(const Image& image, std::int64_t rows, std::int64_t cols) {
  constexpr std::int32_t kOne = pow2(kResizeBits);
  const AxisSamples xs = axis_samples(image.cols, cols);
  const AxisSamples ys = axis_samples(image.rows, rows);
  // Every sum fits 32 bits: across a row at most 255 * 2^11, down a column 255 * 2^22.
  std::vector<std::int32_t> across(at(image.rows * cols));
  for (std::int64_t r = 0; r < image.rows; ++r) {
    const std::span<const std::uint8_t> row =
        std::span(image.pixels).subspan(at(r * image.cols), at(image.cols));
    const std::span<std::int32_t> sums = std::span(across).subspan(at(r * cols), at(cols));
    for (std::int64_t c = 0; c < cols; ++c) {
      const auto fx = static_cast<std::int32_t>(xs.fraction[at(c)]);
      sums[at(c)] = (row[at(xs.first[at(c)])] * (kOne - fx)) + (row[at(xs.second[at(c)])] * fx);
    }
  }
  Image out = blank(rows, cols);
  constexpr unsigned kShift = 2 * kResizeBits;
  constexpr std::uint32_t kHalf = pow2(kShift - 1);
  for (std::int64_t r = 0; r < rows; ++r) {
    const auto fy = static_cast<std::int32_t>(ys.fraction[at(r)]);
    const std::span<const std::int32_t> top =
        std::span(across).subspan(at(ys.first[at(r)] * cols), at(cols));
    const std::span<const std::int32_t> bottom =
        std::span(across).subspan(at(ys.second[at(r)] * cols), at(cols));
    // A local view: a byte store may alias anything, the image's own pointer included.
    const std::span<std::uint8_t> line = std::span(out.pixels).subspan(at(r * cols), at(cols));
    for (std::int64_t c = 0; c < cols; ++c) {
      const auto total =
          static_cast<std::uint32_t>((top[at(c)] * (kOne - fy)) + (bottom[at(c)] * fy));
      line[at(c)] = static_cast<std::uint8_t>((total + kHalf) >> kShift);
    }
  }
  return out;
}

/// @brief A separable blur's vertical pass, rounded to intensities.
/// @tparam Sum An unsigned integer wide enough for `255 * 2^(2 bits)`.
/// @param across The horizontal pass's sums, row-major, `out`'s shape.
/// @param kernel The weights.
/// @param row_index Each padded row's reflected row (`reflected`).
/// @param bits The kernel's bits.
/// @param out The blurred image, its pixels written.
template <typename Sum>
void blur_columns(std::span<const std::int32_t> across, std::span<const std::uint32_t> kernel,
                  std::span<const std::int64_t> row_index, int bits, Image& out) {
  const std::int64_t cols = out.cols;
  std::vector<Sum> sums(at(cols));
  for (std::int64_t r = 0; r < out.rows; ++r) {
    std::ranges::fill(sums, Sum{0});
    for (std::size_t k = 0; k < kernel.size(); ++k) {
      const Sum weight = kernel[k];
      const std::span<const std::int32_t> source =
          across.subspan(at(row_index[at(r) + k] * cols), at(cols));
      for (std::int64_t c = 0; c < cols; ++c) {
        sums[at(c)] += weight * static_cast<Sum>(source[at(c)]);
      }
    }
    // A local view: a byte store may alias anything, the image's own pointer included.
    const std::span<std::uint8_t> row = std::span(out.pixels).subspan(at(r * cols), at(cols));
    const auto shift = static_cast<unsigned>(2 * bits);
    const Sum half = (Sum{1} << shift) >> 1U;  // 0 for a 0-bit kernel, as round_shift's
    for (std::int64_t c = 0; c < cols; ++c) {
      row[at(c)] = static_cast<std::uint8_t>((sums[at(c)] + half) >> shift);
    }
  }
}

}  // namespace

Result<void> check_image(const Image& image, const char* name) {
  return check_values(image.rows, image.cols, 1, image.pixels.size(), name);
}

Result<int> kernel_bits(std::span<const std::uint32_t> kernel) {
  if (kernel.size() % 2 != 1) {
    return fail(ErrorKind::kInvalidInput, "kernel: must be an odd number of non-negative integers");
  }
  std::uint64_t total = 0;
  for (const std::uint32_t w : kernel) {
    total += w;
  }
  if (!std::has_single_bit(total) || std::countr_zero(total) > kMaxKernelBits) {
    return fail(ErrorKind::kInvalidInput, "kernel: must sum to a power of two up to 2^" +
                                              std::to_string(kMaxKernelBits) + ", got " +
                                              std::to_string(total));
  }
  return std::countr_zero(total);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (rows, cols), as numpy's shape
Result<std::vector<std::array<std::int64_t, 2>>> level_shapes(std::int64_t rows, std::int64_t cols,
                                                              std::int64_t levels,
                                                              double scale_factor) {
  if (auto ok = check_side(levels, "levels"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_scale_factor(scale_factor, "scale_factor"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<std::array<std::int64_t, 2>> shapes{{rows, cols}};
  double scale = 1.0;
  for (std::int64_t level = 1; level < levels; ++level) {
    scale *= scale_factor;
    const auto level_rows =
        static_cast<std::int64_t>(std::floor((static_cast<double>(rows) / scale) + 0.5));
    const auto level_cols =
        static_cast<std::int64_t>(std::floor((static_cast<double>(cols) / scale) + 0.5));
    if (level_rows < 1 || level_cols < 1) {
      std::ostringstream message;
      message << "pyramid: level " << level << " of a " << rows << "x" << cols
              << " image at scale factor " << scale_factor << " would be empty";
      return fail(ErrorKind::kInvalidInput, message.str());
    }
    shapes.push_back({level_rows, level_cols});
  }
  return shapes;
}

Result<Image> to_gray(std::int64_t rows, std::int64_t cols, std::span<const std::uint8_t> rgb) {
  if (auto ok = check_values(rows, cols, 3, rgb.size(), "rgb"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  Image out = blank(rows, cols);
  const auto& [r, g, b] = kGrayWeights;
  for (std::size_t i = 0; i < out.pixels.size(); ++i) {
    const std::int64_t total = (r * rgb[3 * i]) + (g * rgb[(3 * i) + 1]) + (b * rgb[(3 * i) + 2]);
    out.pixels[i] = round_shift(total, kGrayBits);
  }
  return out;
}

Result<Image> blur(const Image& image, std::span<const std::uint32_t> kernel) {
  if (auto ok = check_image(image, "image"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const Result<int> bits = kernel_bits(kernel);
  if (!bits) {
    return std::unexpected(bits.error());
  }
  const auto taps = static_cast<std::int64_t>(kernel.size());
  const std::int64_t radius = taps / 2;
  const std::int64_t rows = image.rows;
  const std::int64_t cols = image.cols;
  const std::vector<std::int64_t> column_index = reflected(radius, cols);
  const std::vector<std::int64_t> row_index = reflected(radius, rows);
  // The horizontal sums reach 255 * 2^bits, bits at most kMaxKernelBits: 32 bits hold them.
  // The interior reads its neighbours directly, a tap at a time, so the loops vectorize; the
  // border reflects. Integer sums are exact in any order.
  std::vector<std::int32_t> across(at(rows * cols), 0);
  const std::int64_t first = std::min(radius, cols);
  const std::int64_t last = std::max(first, cols - radius);
  for (std::int64_t r = 0; r < rows; ++r) {
    const std::span<const std::uint8_t> row =
        std::span(image.pixels).subspan(at(r * cols), at(cols));
    const std::span<std::int32_t> sums = std::span(across).subspan(at(r * cols), at(cols));
    for (std::int64_t k = 0; k < taps; ++k) {
      const auto weight = static_cast<std::int32_t>(kernel[at(k)]);
      const std::span<const std::uint8_t> shifted = row.subspan(at(k), at(last - first));
      for (std::int64_t c = 0; c < last - first; ++c) {
        sums[at(first + c)] += weight * shifted[at(c)];
      }
    }
    const auto reflecting = [&](std::int64_t c) {
      sums[at(c)] = static_cast<std::int32_t>(weighted_sum(
          kernel, [&](std::size_t k) -> std::int64_t { return row[at(column_index[at(c) + k])]; }));
    };
    for (std::int64_t c = 0; c < first; ++c) {
      reflecting(c);
    }
    for (std::int64_t c = last; c < cols; ++c) {
      reflecting(c);
    }
  }
  // The vertical sums reach 255 * 2^(2 bits): 32 bits hold them up to 12-bit kernels
  // (ORB's), which lets the loop vectorize; 64 bits above.
  Image out = blank(rows, cols);
  if ((2 * *bits) + 8 <= 32) {
    blur_columns<std::uint32_t>(across, kernel, row_index, *bits, out);
  } else {
    blur_columns<std::uint64_t>(across, kernel, row_index, *bits, out);
  }
  return out;
}

Result<Image> resize(const Image& image, std::int64_t rows, std::int64_t cols) {
  if (auto ok = check_resampling(image, rows, cols); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return resize_checked(image, rows, cols);
}

Result<std::vector<Image>> pyramid(const Image& image, std::int64_t levels, double scale_factor) {
  if (auto ok = check_image(image, "image"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const auto shapes = level_shapes(image.rows, image.cols, levels, scale_factor);
  if (!shapes) {
    return std::unexpected(shapes.error());
  }
  std::vector<Image> out{image};
  out.reserve(shapes->size());
  for (std::size_t level = 1; level < shapes->size(); ++level) {
    const auto [rows, cols] = (*shapes)[level];
    out.push_back(resize_checked(out.back(), rows, cols));
  }
  return out;
}

Result<void> check_pyramid(std::span<const Image> levels, std::int64_t count, double scale_factor,
                           const char* name) {
  if (std::cmp_not_equal(levels.size(), count)) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected " +
                                              std::to_string(count) + " levels, got " +
                                              std::to_string(levels.size()));
  }
  for (const Image& level : levels) {
    if (auto ok = check_image(level, name); !ok) {
      return ok;
    }
  }
  if (levels.empty()) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected at least one level");
  }
  const auto shapes = level_shapes(levels.front().rows, levels.front().cols, count, scale_factor);
  if (!shapes) {
    return std::unexpected(shapes.error());
  }
  for (std::size_t k = 0; k < levels.size(); ++k) {
    const auto [rows, cols] = (*shapes)[k];
    if (levels[k].rows != rows || levels[k].cols != cols) {
      return fail(ErrorKind::kInvalidInput, std::string(name) + ": level " + std::to_string(k) +
                                                " is " + std::to_string(levels[k].rows) + "x" +
                                                std::to_string(levels[k].cols) + ", expected " +
                                                std::to_string(rows) + "x" + std::to_string(cols));
    }
  }
  return {};
}

Result<Image> remap(const Image& image, const Eigen::Ref<const Coordinates>& coordinates,
                    std::int64_t rows, std::int64_t cols) {
  if (auto ok = check_resampling(image, rows, cols); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (coordinates.rows() != rows * cols) {
    return fail(ErrorKind::kInvalidInput, "coordinates: expected rows * cols coordinates, got " +
                                              std::to_string(coordinates.rows()));
  }
  if (!coordinates.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "coordinates: expected finite values");
  }
  constexpr std::int64_t kOne = pow2(kRemapBits);
  const auto sample = [&image](std::int64_t r, std::int64_t c) -> std::int64_t {
    const bool inside = r >= 0 && r < image.rows && c >= 0 && c < image.cols;
    return inside ? pixel(image, r, c) : 0;
  };
  Image out = blank(rows, cols);
  for (Eigen::Index i = 0; i < coordinates.rows(); ++i) {
    // Clamping keeps the fixed-point values small; past it every neighbour is outside anyway.
    // x * 2^bits is exact, so the rounding is the same whether or not the add is fused.
    const double x = std::clamp(coordinates(i, 0), -2.0, static_cast<double>(image.cols) + 1.0);
    const double y = std::clamp(coordinates(i, 1), -2.0, static_cast<double>(image.rows) + 1.0);
    const auto qx = static_cast<std::int64_t>(std::floor((x * kOne) + 0.5));
    const auto qy = static_cast<std::int64_t>(std::floor((y * kOne) + 0.5));
    const std::int64_t ix = floor_div(qx, kOne);
    const std::int64_t iy = floor_div(qy, kOne);
    const std::int64_t fx = qx - (ix * kOne);
    const std::int64_t fy = qy - (iy * kOne);
    const std::int64_t total =
        (sample(iy, ix) * (kOne - fx) * (kOne - fy)) + (sample(iy, ix + 1) * fx * (kOne - fy)) +
        (sample(iy + 1, ix) * (kOne - fx) * fy) + (sample(iy + 1, ix + 1) * fx * fy);
    out.pixels[at(i)] = round_shift(total, 2 * kRemapBits);
  }
  return out;
}

Result<void> check_scale_factor(double scale_factor, const char* name) {
  if (!std::isfinite(scale_factor) || scale_factor <= 1.0) {
    return fail_setting(name, "must be finite and above 1", scale_factor);
  }
  return {};
}

}  // namespace slamlab::foundations::image
