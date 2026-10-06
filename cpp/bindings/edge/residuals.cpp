/// @file
/// @brief Bindings for `slamlab.edge.residuals` (`slamlab._cpp.edge.residuals`).
#include "slamlab/edge/residuals.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <array>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

/// @brief Each observation's chi-square error.
/// @param camera The camera, as a dict.
/// @param local The points in the camera's frame, (N, 3).
/// @param xy Their observed pixels, (N, 2).
/// @param right Their right columns (NaN for none), (N,).
/// @param variance Their levels' variances, (N,).
/// @return A new (N,) float64 array.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, double> chi2(const nb::dict& camera, const BatchIn<3>& local,
                                    const BatchIn<2>& xy, const VectorIn<double>& right,
                                    const VectorIn<double>& variance) {
  std::vector<Eigen::Vector3d> points(local.shape(0));
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i] = {local(i, 0), local(i, 1), local(i, 2)};
  }
  std::vector<std::array<double, 2>> pixels(xy.shape(0));
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = {xy(i, 0), xy(i, 1)};
  }
  return edge_crossing::vector_array(unwrap(edge::chi2(
      edge_crossing::camera_in(camera), points, pixels, copy_vector<std::vector<double>>(right),
      copy_vector<std::vector<double>>(variance))));
}

}  // namespace

void bind_residuals(nb::module_& m) {
  m.def("chi2", &chi2, nb::arg("camera"), nb::arg("local").noconvert(), nb::arg("xy").noconvert(),
        nb::arg("right").noconvert(), nb::arg("variance").noconvert(),
        "Each observation's chi-square error (infinite behind the camera).");
}

}  // namespace slamlab::bindings
