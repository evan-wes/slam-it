/// @file
/// @brief The `slam_it_all._cpp` Python module's root: wires one submodule per rung.
///
/// Each rung's bindings live in `cpp/bindings/<rung>/`, one file per bound module, mirroring
/// the Python package tree, so no file grows with every rung. Bindings only convert types:
/// float64 numpy arrays in and out, plain arrays and spans into the cores. The Python entry
/// points validate input before calling.
#include <nanobind/nanobind.h>

#include "common.hpp"
#include "edge/edge.hpp"
#include "foundations/foundations.hpp"
#include "slam_it_all/foundations/linalg.hpp"

NB_MODULE(_cpp, m) {
  m.doc() = "SLAM It All!'s C++ core. Submodules mirror the slam_it_all package tree.";
  slam_it_all::foundations::linalg::fix_product_blocking();
  slam_it_all::bindings::register_error_translation();
  slam_it_all::bindings::bind_foundations(m);
  slam_it_all::bindings::bind_edge(m);
}
