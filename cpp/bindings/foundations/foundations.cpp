/// @file
/// @brief The `slamlab._cpp.foundations` submodule: wires rung 0's bound modules.
#include "foundations/foundations.hpp"

namespace slamlab::bindings {

void bind_foundations(nanobind::module_& parent) {
  nanobind::module_ foundations = parent.def_submodule("foundations", "Rung 0, foundations.");
  nanobind::module_ so3 = foundations.def_submodule("so3", "SO(3), the group of 3D rotations.");
  bind_so3(so3);
  nanobind::module_ se3 = foundations.def_submodule("se3", "SE(3), rigid-body transforms.");
  bind_se3(se3);
  nanobind::module_ camera = foundations.def_submodule("camera", "Camera models.");
  bind_camera(camera);
  nanobind::module_ alignment =
      foundations.def_submodule("alignment", "Point-set alignment (Umeyama).");
  bind_alignment(alignment);
  nanobind::module_ triangulation =
      foundations.def_submodule("triangulation", "Triangulation from bearing vectors.");
  bind_triangulation(triangulation);
  nanobind::module_ ba = foundations.def_submodule("ba", "Bundle adjustment.");
  bind_ba(ba);
}

}  // namespace slamlab::bindings
