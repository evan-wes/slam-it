/// @file
/// @brief Bindings for `slamlab.foundations.stereo` (`slamlab._cpp.foundations.stereo`).
///
/// Camera models cross as their integer codes; a rectification comes back as a dict of its
/// rotations, rectified intrinsics and baseline.
#include "slamlab/foundations/stereo.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <cstdint>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

/// @brief A read-only, contiguous float64 vector.
using Vector = VectorIn<double>;

/// @brief Map a float64 vector in place.
/// @param in The array.
/// @return An Eigen view of it.
Eigen::Map<const Eigen::VectorXd> vector_view(const Vector& in) {
  check_aligned(in.data());
  return {in.data(), static_cast<Eigen::Index>(in.shape(0))};
}

/// @brief Rectify a pair.
/// @param model_left The left camera's model code.
/// @param intrinsics_left Its intrinsics.
/// @param model_right The right camera's model code.
/// @param intrinsics_right Its intrinsics.
/// @param T_right_left The (4, 4) transform from left to right camera coordinates.
/// @param rows The images' rows.
/// @param cols The images' columns.
/// @return A dict: `rotation_left`, `rotation_right` (3, 3), `intrinsics` (4,), `baseline`.
/// @throws CoreError for invalid or degenerate input; std::invalid_argument for a bad code.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the Python signature's order
nb::dict rectify(int model_left, const Vector& intrinsics_left, int model_right,
                 const Vector& intrinsics_right, const Matrix4In& T_right_left, std::int64_t rows,
                 std::int64_t cols) {
  const foundations::stereo::Rectification r = unwrap(foundations::stereo::rectify(
      enum_code<foundations::camera::Model>(model_left, "model_left"), vector_view(intrinsics_left),
      enum_code<foundations::camera::Model>(model_right, "model_right"),
      vector_view(intrinsics_right), matrix4(T_right_left), rows, cols));
  nb::dict out;
  out["rotation_left"] = matrix3_array(r.rotation_left);
  out["rotation_right"] = matrix3_array(r.rotation_right);
  out["intrinsics"] = vector_array(r.intrinsics);
  out["baseline"] = r.baseline;
  return out;
}

/// @brief One side's rectification map.
/// @param model The camera's model code.
/// @param intrinsics Its intrinsics.
/// @param rotation Its rectifying rotation.
/// @param rectified The rectified pinhole intrinsics.
/// @param rows The images' rows.
/// @param cols The images' columns.
/// @return A new (rows, cols, 2) float64 array.
/// @throws CoreError for invalid input; std::invalid_argument for a bad code.
ArrayOut rectification_map(int model, const Vector& intrinsics, const Matrix3In& rotation,
                           const Vector& rectified, std::int64_t rows, std::int64_t cols) {
  const Eigen::Map<const Eigen::VectorXd> pinhole = vector_view(rectified);
  if (pinhole.size() != 4) {
    throw std::invalid_argument("rectified: expected 4 values");
  }
  const foundations::image::Coordinates map = unwrap(foundations::stereo::rectification_map(
      enum_code<foundations::camera::Model>(model, "model"), vector_view(intrinsics),
      matrix3(rotation), Eigen::Vector4d(pinhole), rows, cols));
  auto [out, data] = new_array({static_cast<std::size_t>(rows), static_cast<std::size_t>(cols), 2});
  Eigen::Map<foundations::image::Coordinates>(data, map.rows(), 2) = map;
  return out;
}

}  // namespace

void bind_stereo(nb::module_& m) {
  m.def("rectify", &rectify, nb::arg("model_left"), nb::arg("intrinsics_left").noconvert(),
        nb::arg("model_right"), nb::arg("intrinsics_right").noconvert(),
        nb::arg("T_right_left").noconvert(), nb::arg("rows"), nb::arg("cols"),
        "Rectify a pair; return the rotations, the rectified intrinsics and the baseline.");
  m.def("rectification_map", &rectification_map, nb::arg("model"),
        nb::arg("intrinsics").noconvert(), nb::arg("rotation").noconvert(),
        nb::arg("rectified").noconvert(), nb::arg("rows"), nb::arg("cols"),
        "Return the (rows, cols, 2) map from rectified pixels to the original image.");
}

}  // namespace slamlab::bindings
