/// @file
/// @brief Shared by every binding that takes or returns images: a C-contiguous (H, W) uint8
///        array crossing as `image::Image`, and back as a new array; a list of them as a
///        pyramid.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "common.hpp"
#include "slam_it/foundations/image.hpp"

namespace slam_it::bindings {

/// @brief A read-only, contiguous (H, W) uint8 image.
using ImageIn = nanobind::ndarray<const std::uint8_t, nanobind::shape<-1, -1>, nanobind::c_contig,
                                  nanobind::device::cpu>;

/// @brief A new uint8 numpy array, owning its buffer.
using ImageOut = nanobind::ndarray<nanobind::numpy, std::uint8_t>;

/// @brief Copy an image in.
/// @param in The array.
/// @return The core's image.
inline foundations::image::Image image_in(const ImageIn& in) {
  const std::size_t size = in.shape(0) * in.shape(1);
  return foundations::image::Image{
      .rows = static_cast<std::int64_t>(in.shape(0)),
      .cols = static_cast<std::int64_t>(in.shape(1)),
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): the buffer's end
      .pixels = std::vector<std::uint8_t>(in.data(), in.data() + size)};
}

/// @brief Copy a pyramid in.
/// @param in The levels' arrays.
/// @return The core's images, in order.
inline std::vector<foundations::image::Image> levels_in(const std::vector<ImageIn>& in) {
  std::vector<foundations::image::Image> out;
  out.reserve(in.size());
  for (const ImageIn& level : in) {
    out.push_back(image_in(level));
  }
  return out;
}

/// @brief Copy an image out.
/// @param image The core's image.
/// @return A new (rows, cols) uint8 array.
inline ImageOut image_array(const foundations::image::Image& image) {
  auto [out, data] = new_array<std::uint8_t>(
      {static_cast<std::size_t>(image.rows), static_cast<std::size_t>(image.cols)});
  std::ranges::copy(image.pixels, data);
  return out;
}

}  // namespace slam_it::bindings
