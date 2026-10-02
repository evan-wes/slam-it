/// @file
/// @brief The `slamlab._cpp` Python module: thin nanobind wrappers over the C++ core.
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>

#include "slamlab/so3.hpp"

namespace nb = nanobind;

NB_MODULE(_cpp, m) {
  m.doc() = "slamlab's C++ core.";
  m.def("hat", &slamlab::hat, nb::arg("v"),
        "Map a 3-vector to its skew-symmetric matrix, returned as nested lists.");
}
