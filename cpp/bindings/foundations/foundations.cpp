/// @file
/// @brief The `slamlab._cpp.foundations` submodule: wires rung 0's bound modules.
#include "foundations/foundations.hpp"

namespace slamlab::bindings {

void bind_foundations(nanobind::module_& parent) {
  nanobind::module_ foundations = parent.def_submodule("foundations", "Rung 0, foundations.");
  nanobind::module_ so3 = foundations.def_submodule("so3", "SO(3), the group of 3D rotations.");
  bind_so3(so3);
}

}  // namespace slamlab::bindings
