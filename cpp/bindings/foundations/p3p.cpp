/// @file
/// @brief Bindings for `slamlab.foundations.p3p` (`slamlab._cpp.foundations.p3p`).
#include "slamlab/foundations/p3p.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <stdexcept>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace p3p = foundations::p3p;
using Rows3 = foundations::correspondences::Rows3;

/// @brief Every camera pose three world points and their bearings allow.
/// @param points (3, 3) world points.
/// @param bearings (3, 3) unit bearings.
/// @param solver The solver's code (index into `p3p.SOLVERS`).
/// @return A new (K, 4, 4) array of poses `T_cam_world`.
/// @throws CoreError for invalid input; std::invalid_argument for an unknown code.
ArrayOut solve_p3p(const BatchIn<3>& points, const BatchIn<3>& bearings, int solver) {
  if (solver < 0 || solver >= static_cast<int>(p3p::Solver::kCount)) {
    throw std::invalid_argument("solver: unknown code");
  }
  const auto found = unwrap(p3p::p3p(copy_matrix<Rows3>(points.data(), points.shape(0), 3),
                                     copy_matrix<Rows3>(bearings.data(), bearings.shape(0), 3),
                                     static_cast<p3p::Solver>(solver)));
  return poses_array(found);
}

/// @brief The enum tables: the solvers' names in code order.
/// @return `{"solver": [...]}`.
nb::dict codes() {
  nb::list solvers;
  for (int k = 0; k < static_cast<int>(p3p::Solver::kCount); ++k) {
    solvers.append(p3p::name(static_cast<p3p::Solver>(k)));
  }
  nb::dict out;
  out["solver"] = solvers;
  return out;
}

}  // namespace

void bind_p3p(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("p3p", &solve_p3p, nb::arg("points").noconvert(), nb::arg("bearings").noconvert(),
        nb::arg("solver"), "Return every pose T_cam_world three correspondences allow, (K, 4, 4).");
}

}  // namespace slamlab::bindings
