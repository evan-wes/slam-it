/// @file
/// @brief Bindings for `slam_it_all.foundations.camera` (`slam_it_all._cpp.foundations.camera`).
#include "slam_it_all/foundations/camera.hpp"

#include <nanobind/nanobind.h>

#include <cstddef>
#include <stdexcept>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slam_it_all::bindings {

namespace {

namespace camera = foundations::camera;

/// @brief Read-only, contiguous float64 intrinsics.
using IntrinsicsIn = nb::ndarray<const double, nb::shape<-1>, nb::c_contig, nb::device::cpu>;

/// @brief Convert a model code to the core's enum.
/// @param model The integer code.
/// @return The model.
/// @throws std::invalid_argument for an unknown code.
camera::Model to_model(int model) {
  if (model == 0) {
    return camera::Model::kPinhole;
  }
  if (model == 1) {
    return camera::Model::kRadTan;
  }
  if (model == 2) {
    return camera::Model::kRadial;
  }
  throw std::invalid_argument("unknown camera model code");
}

/// @brief View intrinsics as an Eigen vector.
/// @param intrinsics The array.
/// @return A read-only map.
Eigen::Map<const Eigen::VectorXd> view(const IntrinsicsIn& intrinsics) {
  check_aligned(intrinsics.data());
  return {intrinsics.data(), static_cast<Eigen::Index>(intrinsics.shape(0))};
}

/// @brief Project (N, 3) points to (N, 2) pixels.
/// @param model The model code.
/// @param intrinsics The intrinsics.
/// @param points The points.
/// @return A new (N, 2) array.
ArrayOut project(int model, const IntrinsicsIn& intrinsics, const BatchIn<3>& points) {
  const camera::Model m = to_model(model);
  const auto k = view(intrinsics);
  return map_batch<3, 1, 2, 1>(points,
                               [&](const Eigen::Vector3d& p) { return camera::project(m, k, p); });
}

/// @brief Return (N, 2, 3 + K): d(pixel)/d(point) then d(pixel)/d(intrinsics).
/// @param model The model code.
/// @param intrinsics The intrinsics.
/// @param points The points.
/// @return A new (N, 2, 3 + K) array.
ArrayOut project_jacobians(int model, const IntrinsicsIn& intrinsics, const BatchIn<3>& points) {
  const camera::Model m = to_model(model);
  const auto k = view(intrinsics);
  check_aligned(points.data());
  const std::size_t n = points.shape(0);
  const auto cols = static_cast<std::size_t>(3 + k.size());
  auto [out, data] = new_array({n, 2, cols});
  for (std::size_t i = 0; i < n; ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offsets
    const Eigen::Map<const Eigen::Vector3d> point(points.data() + (3 * i));
    Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>> result(
        data + (2 * cols * i), 2, static_cast<Eigen::Index>(cols));
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    result = unwrap(camera::project_jacobian(m, k, point));
  }
  return out;
}

/// @brief Map (N, 2) pixels to (N, 3) unit bearing vectors.
/// @param model The model code.
/// @param intrinsics The intrinsics.
/// @param pixels The pixels.
/// @return A new (N, 3) array.
ArrayOut unproject(int model, const IntrinsicsIn& intrinsics, const BatchIn<2>& pixels) {
  const camera::Model m = to_model(model);
  const auto k = view(intrinsics);
  return map_batch<2, 1, 3, 1>(
      pixels, [&](const Eigen::Vector2d& p) { return camera::unproject(m, k, p); });
}

}  // namespace

void bind_camera(nb::module_& m) {
  m.def("project", &project, nb::arg("model"), nb::arg("intrinsics").noconvert(),
        nb::arg("points").noconvert(), "Project (N, 3) camera-frame points to (N, 2) pixels.");
  m.def("project_jacobians", &project_jacobians, nb::arg("model"),
        nb::arg("intrinsics").noconvert(), nb::arg("points").noconvert(),
        "Return (N, 2, 3 + K): d(pixel)/d(point) then d(pixel)/d(intrinsics).");
  m.def("unproject", &unproject, nb::arg("model"), nb::arg("intrinsics").noconvert(),
        nb::arg("pixels").noconvert(), "Map (N, 2) pixels to (N, 3) unit bearing vectors.");
}

}  // namespace slam_it_all::bindings
