/// @file
/// @brief Bindings for `slam_it.edge.frames` (`slam_it._cpp.edge.frames`).
///
/// Keypoints cross as an (N, 2) float64 array, the depth map as an (H, W) float64 one and a
/// mask as an (H, W) uint8 one.
#include "slam_it/edge/frames.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"
#include "foundations/image_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief A read-only (N, 2) float64 array of keypoints' pixels.
using PixelsIn = nb::ndarray<const double, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;

/// @brief Copy keypoints' pixels in.
/// @param xy The (N, 2) array.
/// @return Each row as a pair.
std::vector<std::array<double, 2>> pixels_in(const PixelsIn& xy) {
  std::vector<std::array<double, 2>> pixels(xy.shape(0));
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = {xy(i, 0), xy(i, 1)};
  }
  return pixels;
}

/// @brief Read a depth map at keypoints.
/// @param xy The keypoints' pixels, (N, 2).
/// @param depth_map The depths in metres, (H, W).
/// @return A new (N,) float64 array: each depth, or NaN.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, double> depth_at(
    const PixelsIn& xy,
    const nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig, nb::device::cpu>& depth_map) {
  const edge::DepthMap map{
      .rows = static_cast<std::int64_t>(depth_map.shape(0)),
      .cols = static_cast<std::int64_t>(depth_map.shape(1)),
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): the buffer's end
      .values = std::vector<double>(depth_map.data(), depth_map.data() + depth_map.size())};
  return edge_crossing::vector_array(unwrap(edge::depth_at(pixels_in(xy), map)));
}

/// @brief Return the keypoints a mask keeps.
/// @param xy The keypoints' pixels, (N, 2).
/// @param mask The mask, (H, W) uint8: 0 where features are dropped.
/// @return A new int64 array: the kept keypoints' indices, ascending.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> unmasked(const PixelsIn& xy, const ImageIn& mask) {
  return edge_crossing::vector_array(unwrap(edge::unmasked(pixels_in(xy), image_in(mask))));
}

}  // namespace

void bind_frames(nb::module_& m) {
  m.def("depth_at", &depth_at, nb::arg("xy").noconvert(), nb::arg("depth_map").noconvert(),
        "Read a depth map at each keypoint's nearest pixel; NaN where it holds none.");
  m.def("unmasked", &unmasked, nb::arg("xy").noconvert(), nb::arg("mask").noconvert(),
        "Return the keypoints a mask keeps: those whose nearest pixel is not 0.");
}

}  // namespace slam_it::bindings
