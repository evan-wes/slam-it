/// @file
/// @brief Bindings for `slamlab.foundations.fivepoint` (`slamlab._cpp.foundations.fivepoint`).
#include "slamlab/foundations/fivepoint.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <stdexcept>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace fivepoint = foundations::fivepoint;
using Rows3 = foundations::correspondences::Rows3;

/// @brief Every essential matrix five bearing pairs allow.
/// @param bearings1 (5, 3) bearings in camera 1.
/// @param bearings2 (5, 3) bearings in camera 2.
/// @param solver The solver's code (index into `fivepoint.SOLVERS`).
/// @return A new (K, 3, 3) array.
/// @throws CoreError for invalid input; std::invalid_argument for an unknown code.
ArrayOut essential_5pt(const BatchIn<3>& bearings1, const BatchIn<3>& bearings2, int solver) {
  if (solver < 0 || solver >= static_cast<int>(fivepoint::Solver::kCount)) {
    throw std::invalid_argument("solver: unknown code");
  }
  const auto found =
      unwrap(fivepoint::essential_5pt(copy_matrix<Rows3>(bearings1.data(), bearings1.shape(0), 3),
                                      copy_matrix<Rows3>(bearings2.data(), bearings2.shape(0), 3),
                                      static_cast<fivepoint::Solver>(solver)));
  auto [out, data] = new_array({found.size(), 3, 3});
  for (std::size_t k = 0; k < found.size(); ++k) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    Eigen::Map<Item<3, 3>> view(data + (9 * k));
    view = found[k];
  }
  return out;
}

/// @brief The enum tables: the solvers' names in code order.
/// @return `{"solver": [...]}`.
nb::dict codes() {
  nb::list solvers;
  for (int k = 0; k < static_cast<int>(fivepoint::Solver::kCount); ++k) {
    solvers.append(fivepoint::name(static_cast<fivepoint::Solver>(k)));
  }
  nb::dict out;
  out["solver"] = solvers;
  return out;
}

}  // namespace

void bind_fivepoint(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("essential_5pt", &essential_5pt, nb::arg("bearings1").noconvert(),
        nb::arg("bearings2").noconvert(), nb::arg("solver"),
        "Return every essential matrix five bearing pairs allow, shape (K, 3, 3).");
}

}  // namespace slamlab::bindings
