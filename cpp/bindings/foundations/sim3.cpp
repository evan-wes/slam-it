/// @file
/// @brief Bindings for `slam_it.foundations.sim3` (`slam_it._cpp.foundations.sim3`).
#include "slam_it/foundations/sim3.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/ransac_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace camera = foundations::camera;
namespace sim3 = foundations::sim3;

/// @brief The enum tables: RANSAC's terminations.
/// @return `{"ransac_termination": [...]}`.
nb::dict codes() { return ransac_codes(); }

/// @brief Estimate `S_12` from two cameras' matched points robustly (`sim3.sim3_ransac`).
/// @param points1 (N, 3) points in camera 1's frame.
/// @param points2 (N, 3) points in camera 2's frame.
/// @param sigma2_1 Each match's variance in image 1.
/// @param sigma2_2 Each match's variance in image 2.
/// @param model The camera model's code.
/// @param intrinsics Its intrinsics.
/// @param fixed_scale Hold the scale at 1.
/// @param options `ransac.Options`' fields.
/// @return The solution as a dict.
/// @throws CoreError for invalid or degenerate matches; std::invalid_argument for an unknown
///         model code.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict sim3_ransac(const BatchIn<3>& points1, const BatchIn<3>& points2,
                     const VectorIn<double>& sigma2_1, const VectorIn<double>& sigma2_2, int model,
                     const VectorIn<double>& intrinsics, bool fixed_scale,
                     const nb::dict& options) {
  return ransac_result(unwrap(sim3::sim3_ransac(
      rows3(points1), rows3(points2), copy_vector<Eigen::VectorXd>(sigma2_1),
      copy_vector<Eigen::VectorXd>(sigma2_2), enum_code<camera::Model>(model, "model"),
      copy_vector<Eigen::VectorXd>(intrinsics), fixed_scale, ransac_options(options))));
}

}  // namespace

void bind_sim3(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("sim3_ransac", &sim3_ransac, nb::arg("points1").noconvert(), nb::arg("points2").noconvert(),
        nb::arg("sigma2_1").noconvert(), nb::arg("sigma2_2").noconvert(), nb::arg("model"),
        nb::arg("intrinsics").noconvert(), nb::arg("fixed_scale"), nb::arg("options"),
        "Estimate S_12 from (N, 3) matched points by LO-MSAC with Umeyama's alignment.");
}

}  // namespace slam_it::bindings
