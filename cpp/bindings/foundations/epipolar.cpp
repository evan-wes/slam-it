/// @file
/// @brief Bindings for `slamlab.foundations.epipolar` (`slamlab._cpp.foundations.epipolar`).
#include "slamlab/foundations/epipolar.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/ransac_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace epipolar = foundations::epipolar;

/// @brief Copy an (N, 3) batch in.
/// @param in The array.
/// @return The rows.
epipolar::Rows3 rows3(const BatchIn<3>& in) {
  return copy_matrix<epipolar::Rows3>(in.data(), in.shape(0), 3);
}

/// @brief Copy an (N, 2) batch in.
/// @param in The array.
/// @return The rows.
epipolar::Rows2 rows2(const BatchIn<2>& in) {
  return copy_matrix<epipolar::Rows2>(in.data(), in.shape(0), 2);
}

/// @brief E from (N, 3) bearing pairs.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @return A new (3, 3) array.
/// @throws CoreError for invalid or degenerate input.
ArrayOut essential_8pt(const BatchIn<3>& bearings1, const BatchIn<3>& bearings2) {
  return matrix3_array(unwrap(epipolar::essential_8pt(rows3(bearings1), rows3(bearings2))));
}

/// @brief F from (N, 2) pixel pairs.
/// @param pixels1 Pixels in image 1.
/// @param pixels2 Pixels in image 2.
/// @return A new (3, 3) array.
/// @throws CoreError for invalid or degenerate input.
ArrayOut fundamental_8pt(const BatchIn<2>& pixels1, const BatchIn<2>& pixels2) {
  return matrix3_array(unwrap(epipolar::fundamental_8pt(rows2(pixels1), rows2(pixels2))));
}

/// @brief Squared Sampson errors under E.
/// @param essential E.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @return A new (N,) array.
/// @throws CoreError for invalid input.
ArrayOut sampson_essential(const Matrix3In& essential, const BatchIn<3>& bearings1,
                           const BatchIn<3>& bearings2) {
  return vector_array(
      unwrap(epipolar::sampson_essential(matrix3(essential), rows3(bearings1), rows3(bearings2))));
}

/// @brief Squared Sampson errors under F.
/// @param fundamental F.
/// @param pixels1 Pixels in image 1.
/// @param pixels2 Pixels in image 2.
/// @return A new (N,) array.
/// @throws CoreError for invalid input.
ArrayOut sampson_fundamental(const Matrix3In& fundamental, const BatchIn<2>& pixels1,
                             const BatchIn<2>& pixels2) {
  return vector_array(
      unwrap(epipolar::sampson_fundamental(matrix3(fundamental), rows2(pixels1), rows2(pixels2))));
}

/// @brief E's four candidate poses.
/// @param essential E.
/// @return A new (4, 4, 4) array.
/// @throws CoreError for invalid or degenerate input.
ArrayOut decompose_essential(const Matrix3In& essential) {
  return poses_array(unwrap(epipolar::decompose_essential(matrix3(essential))));
}

/// @brief LO-MSAC with the 8-point E solver.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @param options `ransac.Options`' fields.
/// @return The solution as a dict.
/// @throws CoreError for invalid or degenerate input.
nb::dict essential_ransac(const BatchIn<3>& bearings1, const BatchIn<3>& bearings2,
                          const nb::dict& options) {
  return ransac_result(unwrap(
      epipolar::essential_ransac(rows3(bearings1), rows3(bearings2), ransac_options(options))));
}

/// @brief LO-MSAC with the normalised 8-point F solver.
/// @param pixels1 Pixels in image 1.
/// @param pixels2 Pixels in image 2.
/// @param options `ransac.Options`' fields.
/// @return The solution as a dict.
/// @throws CoreError for invalid or degenerate input.
nb::dict fundamental_ransac(const BatchIn<2>& pixels1, const BatchIn<2>& pixels2,
                            const nb::dict& options) {
  return ransac_result(unwrap(
      epipolar::fundamental_ransac(rows2(pixels1), rows2(pixels2), ransac_options(options))));
}

}  // namespace

void bind_epipolar(nb::module_& m) {
  m.def("codes", &ransac_codes, "Each enum's names in code order.");
  m.def("essential_8pt", &essential_8pt, nb::arg("bearings1").noconvert(),
        nb::arg("bearings2").noconvert(), "Estimate E from (N, 3) bearing pairs, N >= 8.");
  m.def("fundamental_8pt", &fundamental_8pt, nb::arg("pixels1").noconvert(),
        nb::arg("pixels2").noconvert(), "Estimate F from (N, 2) pixel pairs, N >= 8.");
  m.def("sampson_essential", &sampson_essential, nb::arg("essential").noconvert(),
        nb::arg("bearings1").noconvert(), nb::arg("bearings2").noconvert(),
        "Return (N,) squared Sampson errors in the normalised plane.");
  m.def("sampson_fundamental", &sampson_fundamental, nb::arg("fundamental").noconvert(),
        nb::arg("pixels1").noconvert(), nb::arg("pixels2").noconvert(),
        "Return (N,) squared Sampson errors in pixels.");
  m.def("decompose_essential", &decompose_essential, nb::arg("essential").noconvert(),
        "Return E's four candidate poses T_c2_c1, shape (4, 4, 4).");
  m.def("essential_ransac", &essential_ransac, nb::arg("bearings1").noconvert(),
        nb::arg("bearings2").noconvert(), nb::arg("options"),
        "Run LO-MSAC with the 8-point E solver.");
  m.def("fundamental_ransac", &fundamental_ransac, nb::arg("pixels1").noconvert(),
        nb::arg("pixels2").noconvert(), nb::arg("options"),
        "Run LO-MSAC with the normalised 8-point F solver.");
}

}  // namespace slamlab::bindings
