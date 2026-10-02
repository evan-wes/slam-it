/// @file
/// @brief The `slamlab._cpp` Python module: thin nanobind wrappers over the C++ core.
///
/// Submodules mirror the Python package tree (`slamlab._cpp.foundations.so3` implements
/// `slamlab.foundations.so3`), so names never collide across rungs. Bindings only convert
/// types: they take and return nested lists, and the Python entry points validate input.
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>

#include "slamlab/foundations/so3.hpp"

namespace nb = nanobind;

NB_MODULE(_cpp, m) {
  m.doc() = "slamlab's C++ core. Submodules mirror the slamlab package tree.";

  nb::module_ foundations = m.def_submodule("foundations", "Rung 0, foundations.");
  nb::module_ so3 = foundations.def_submodule("so3", "SO(3), the group of 3D rotations.");
  so3.def("hat", &slamlab::foundations::hat, nb::arg("v"),
          "Map a 3-vector to its skew-symmetric matrix, as row-major nested lists.");
}
