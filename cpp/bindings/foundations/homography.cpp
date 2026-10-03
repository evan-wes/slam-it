/// @file
/// @brief Bindings for `slamlab.foundations.homography`
///        (`slamlab._cpp.foundations.homography`).
#include "slamlab/foundations/homography.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/ransac_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace homography = foundations::homography;

/// @brief Copy an (N, 2) batch in.
/// @param in The array.
/// @return The rows.
homography::Rows2 rows2(const BatchIn<2>& in) {
  return copy_matrix<homography::Rows2>(in.data(), in.shape(0), 2);
}

/// @brief H from (N, 2) point pairs.
/// @param points1 Points in image 1.
/// @param points2 Points in image 2.
/// @return A new (3, 3) array.
/// @throws CoreError for invalid or degenerate input.
ArrayOut homography_dlt(const BatchIn<2>& points1, const BatchIn<2>& points2) {
  return matrix3_array(unwrap(homography::homography_dlt(rows2(points1), rows2(points2))));
}

/// @brief Squared Sampson errors under H.
/// @param h H.
/// @param points1 Points in image 1.
/// @param points2 Points in image 2.
/// @return A new (N,) array.
/// @throws CoreError for invalid input.
ArrayOut sampson_homography(const Matrix3In& h, const BatchIn<2>& points1,
                            const BatchIn<2>& points2) {
  return vector_array(
      unwrap(homography::sampson_homography(matrix3(h), rows2(points1), rows2(points2))));
}

/// @brief LO-MSAC with the normalised DLT.
/// @param points1 Points in image 1.
/// @param points2 Points in image 2.
/// @param options `ransac.Options`' fields.
/// @return The solution as a dict.
/// @throws CoreError for invalid or degenerate input.
nb::dict homography_ransac(const BatchIn<2>& points1, const BatchIn<2>& points2,
                           const nb::dict& options) {
  return ransac_result(unwrap(
      homography::homography_ransac(rows2(points1), rows2(points2), ransac_options(options))));
}

}  // namespace

void bind_homography(nb::module_& m) {
  m.def("codes", &ransac_codes, "Each enum's names in code order.");
  m.def("homography_dlt", &homography_dlt, nb::arg("points1").noconvert(),
        nb::arg("points2").noconvert(), "Estimate H from (N, 2) point pairs, N >= 4.");
  m.def("sampson_homography", &sampson_homography, nb::arg("homography").noconvert(),
        nb::arg("points1").noconvert(), nb::arg("points2").noconvert(),
        "Return (N,) squared Sampson errors.");
  m.def("homography_ransac", &homography_ransac, nb::arg("points1").noconvert(),
        nb::arg("points2").noconvert(), nb::arg("options"), "Run LO-MSAC with the normalised DLT.");
}

}  // namespace slamlab::bindings
