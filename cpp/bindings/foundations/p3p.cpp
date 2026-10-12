/// @file
/// @brief Bindings for `slam_it.foundations.p3p` (`slam_it._cpp.foundations.p3p`).
#include "slam_it/foundations/p3p.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace p3p = foundations::p3p;
/// @brief Every camera pose three world points and their bearings allow.
/// @param points (3, 3) world points.
/// @param bearings (3, 3) unit bearings.
/// @param solver The solver's code (index into `p3p.SOLVERS`).
/// @return A new (K, 4, 4) array of poses `T_cam_world`.
/// @throws CoreError for invalid input; std::invalid_argument for an unknown code.
ArrayOut solve_p3p(const BatchIn<3>& points, const BatchIn<3>& bearings, int solver) {
  const auto found =
      unwrap(p3p::p3p(rows3(points), rows3(bearings), enum_code<p3p::Solver>(solver, "solver")));
  return poses_array(found);
}

/// @brief The enum tables: the solvers' names in code order.
/// @return `{"solver": [...]}`.
nb::dict codes() {
  nb::dict out;
  out["solver"] = enum_names<p3p::Solver>([](p3p::Solver solver) { return p3p::name(solver); });
  return out;
}

}  // namespace

void bind_p3p(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("p3p", &solve_p3p, nb::arg("points").noconvert(), nb::arg("bearings").noconvert(),
        nb::arg("solver"), "Return every pose T_cam_world three correspondences allow, (K, 4, 4).");
}

}  // namespace slam_it::bindings
