/// @file
/// @brief Bindings for `slam_it.foundations.image` (`slam_it._cpp.foundations.image`).
#include "slam_it/foundations/image.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/image_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief A read-only, contiguous (H, W, 3) uint8 RGB image.
using RgbIn = nb::ndarray<const std::uint8_t, nb::shape<-1, -1, 3>, nb::c_contig, nb::device::cpu>;

/// @brief A read-only, contiguous (H, W, 2) float64 coordinate map.
using CoordinatesIn =
    nb::ndarray<const double, nb::shape<-1, -1, 2>, nb::c_contig, nb::device::cpu>;

/// @brief Convert an (H, W, 3) RGB image to (H, W) gray.
/// @param rgb The image.
/// @return A new (H, W) uint8 array.
/// @throws CoreError for an invalid image.
ImageOut to_gray(const RgbIn& rgb) {
  const std::size_t size = rgb.shape(0) * rgb.shape(1) * 3;
  return image_array(unwrap(foundations::image::to_gray(static_cast<std::int64_t>(rgb.shape(0)),
                                                        static_cast<std::int64_t>(rgb.shape(1)),
                                                        {rgb.data(), size})));
}

/// @brief Filter with a separable integer kernel.
/// @param image The image.
/// @param kernel The weights.
/// @return A new (H, W) uint8 array.
/// @throws CoreError for an invalid image or kernel.
ImageOut blur(const ImageIn& image, const std::vector<std::uint32_t>& kernel) {
  return image_array(unwrap(foundations::image::blur(image_in(image), kernel)));
}

/// @brief Resize bilinearly.
/// @param image The image.
/// @param rows The output's rows.
/// @param cols The output's columns.
/// @return A new (rows, cols) uint8 array.
/// @throws CoreError for an invalid image or shape.
ImageOut resize(const ImageIn& image, std::int64_t rows, std::int64_t cols) {
  return image_array(unwrap(foundations::image::resize(image_in(image), rows, cols)));
}

/// @brief Build a pyramid.
/// @param image Level 0.
/// @param levels The number of levels.
/// @param scale_factor Each level's downscale.
/// @return A list of new uint8 arrays, one per level.
/// @throws CoreError for an invalid image or setting.
nb::list pyramid(const ImageIn& image, std::int64_t levels, double scale_factor) {
  nb::list out;
  for (const auto& level :
       unwrap(foundations::image::pyramid(image_in(image), levels, scale_factor))) {
    out.append(image_array(level));
  }
  return out;
}

/// @brief Sample an image at a coordinate map.
/// @param image The image.
/// @param coordinates The (H', W', 2) map of `(x, y)`.
/// @return A new (H', W') uint8 array.
/// @throws CoreError for an invalid image or map; std::invalid_argument if misaligned.
ImageOut remap(const ImageIn& image, const CoordinatesIn& coordinates) {
  check_aligned(coordinates.data());
  const auto rows = static_cast<std::int64_t>(coordinates.shape(0));
  const auto cols = static_cast<std::int64_t>(coordinates.shape(1));
  const Eigen::Map<const foundations::image::Coordinates> where(coordinates.data(), rows * cols, 2);
  return image_array(unwrap(foundations::image::remap(image_in(image), where, rows, cols)));
}

}  // namespace

void bind_image(nb::module_& m) {
  m.def("to_gray", &to_gray, nb::arg("rgb").noconvert(),
        "Convert an (H, W, 3) RGB image to (H, W) gray.");
  m.def("blur", &blur, nb::arg("image").noconvert(), nb::arg("kernel"),
        "Filter with a separable integer kernel summing to a power of two.");
  m.def("resize", &resize, nb::arg("image").noconvert(), nb::arg("rows"), nb::arg("cols"),
        "Resize bilinearly to (rows, cols).");
  m.def("pyramid", &pyramid, nb::arg("image").noconvert(), nb::arg("levels"),
        nb::arg("scale_factor"), "Return the image and its successively downscaled levels.");
  m.def("remap", &remap, nb::arg("image").noconvert(), nb::arg("coordinates").noconvert(),
        "Sample the image at an (H', W', 2) map of (x, y) coordinates.");
}

}  // namespace slam_it::bindings
