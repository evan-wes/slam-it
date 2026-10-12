/// @file
/// @brief Bindings for `slam_it.foundations.pnp` (`slam_it._cpp.foundations.pnp`).
#include "slam_it/foundations/pnp.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/nls_crossing.hpp"
#include "foundations/ransac_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace pnp = foundations::pnp;
namespace p3p = foundations::p3p;
/// @brief The enum tables: the P3P solvers, RANSAC's and NLS' terminations.
/// @return `{"minimal_solver": [...], "ransac_termination": [...], "nls_termination": [...]}`.
nb::dict codes() {
  nb::dict out = refined_codes();
  out["minimal_solver"] =
      enum_names<p3p::Solver>([](p3p::Solver solver) { return p3p::name(solver); });
  return out;
}

/// @brief EPnP's pose.
/// @param points (N, 3) world points.
/// @param bearings (N, 3) unit bearings.
/// @return A new (4, 4) array, `T_cam_world`.
/// @throws CoreError for invalid or planar input.
ArrayOut epnp(const BatchIn<3>& points, const BatchIn<3>& bearings) {
  return model_array(unwrap(pnp::epnp(rows3(points), rows3(bearings))));
}

/// @brief Refine a pose on its inliers' reprojection errors.
/// @param points The inliers' world points.
/// @param bearings Their bearings.
/// @param pose The starting pose.
/// @param scale The Cauchy loss's scale.
/// @param weights Each correspondence's weight.
/// @return `{"pose": (4, 4), "summary": the solve's summary}`.
/// @throws CoreError for invalid input.
nb::dict refine_pose(const BatchIn<3>& points, const BatchIn<3>& bearings, const Matrix4In& pose,
                     double scale, const VectorIn<double>& weights) {
  const auto solved = unwrap(pnp::refine_pose(matrix4(pose), rows3(points), rows3(bearings), scale,
                                              copy_vector<Eigen::VectorXd>(weights)));
  nb::dict out;
  out["pose"] = model_array(solved.state);
  out["summary"] = nls_summary(solved.summary);
  return out;
}

/// @brief LO-MSAC with a P3P solver and EPnP refits, then (optionally) the refinement.
/// @param points World points.
/// @param bearings Their bearings.
/// @param options `ransac.Options`' fields.
/// @param minimal The P3P solver's code (an index into `p3p.SOLVERS`).
/// @param refine Whether to refine the consensus pose.
/// @return The solution as a dict.
/// @throws CoreError for invalid or degenerate input; std::invalid_argument for an unknown
///         solver code.
nb::dict pnp_ransac(const BatchIn<3>& points, const BatchIn<3>& bearings, const nb::dict& options,
                    int minimal, bool refine) {
  return refined_result(
      unwrap(pnp::pnp_ransac(rows3(points), rows3(bearings), ransac_options(options),
                             enum_code<p3p::Solver>(minimal, "minimal_solver"), refine)));
}

}  // namespace

void bind_pnp(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("epnp", &epnp, nb::arg("points").noconvert(), nb::arg("bearings").noconvert(),
        "Return EPnP's pose T_cam_world from (N, 3) correspondences, N >= 5.");
  m.def("refine_pose", &refine_pose, nb::arg("points").noconvert(), nb::arg("bearings").noconvert(),
        nb::arg("pose").noconvert(), nb::arg("scale"), nb::arg("weights").noconvert(),
        "Refine a pose on (M, 3) weighted correspondences' reprojection errors, M >= 3.");
  m.def("pnp_ransac", &pnp_ransac, nb::arg("points").noconvert(), nb::arg("bearings").noconvert(),
        nb::arg("options"), nb::arg("minimal_solver"), nb::arg("refine"),
        "Run LO-MSAC with a P3P solver and EPnP refits, then the refinement.");
}

}  // namespace slam_it::bindings
