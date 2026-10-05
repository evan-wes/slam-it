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
  nanobind::module_ image = foundations.def_submodule("image", "Grayscale images in fixed point.");
  bind_image(image);
  nanobind::module_ orb = foundations.def_submodule("orb", "ORB keypoints and descriptors.");
  bind_orb(orb);
  nanobind::module_ matching =
      foundations.def_submodule("matching", "Matching binary descriptors.");
  bind_matching(matching);
  nanobind::module_ alignment =
      foundations.def_submodule("alignment", "Point-set alignment (Umeyama).");
  bind_alignment(alignment);
  nanobind::module_ triangulation =
      foundations.def_submodule("triangulation", "Triangulation from bearing vectors.");
  bind_triangulation(triangulation);
  nanobind::module_ epipolar =
      foundations.def_submodule("epipolar", "Essential and fundamental matrices.");
  bind_epipolar(epipolar);
  nanobind::module_ fivepoint =
      foundations.def_submodule("fivepoint", "The 5-point essential matrix.");
  bind_fivepoint(fivepoint);
  nanobind::module_ p3p = foundations.def_submodule("p3p", "P3P: the pose from three points.");
  bind_p3p(p3p);
  nanobind::module_ pnp = foundations.def_submodule("pnp", "Absolute pose (PnP).");
  bind_pnp(pnp);
  nanobind::module_ homography = foundations.def_submodule("homography", "Homographies.");
  bind_homography(homography);
  nanobind::module_ twoview = foundations.def_submodule("twoview", "Two-view initialisation.");
  bind_twoview(twoview);
  nanobind::module_ ba = foundations.def_submodule("ba", "Bundle adjustment.");
  bind_ba(ba);
}

}  // namespace slamlab::bindings
