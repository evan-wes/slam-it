#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>

#include "slamlab/so3.hpp"

namespace nb = nanobind;

NB_MODULE(_cpp, m) {
  m.doc() = "slamlab's C++ core";
  m.def("hat", &slamlab::hat, nb::arg("v"),
        "Skew-symmetric matrix [v]x of a 3-vector, as nested lists.");
}
