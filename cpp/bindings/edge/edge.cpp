/// @file
/// @brief The `slamlab._cpp.edge` submodule: wires the edge rung's bound modules.
#include "edge/edge.hpp"

namespace slamlab::bindings {

void bind_edge(nanobind::module_& parent) {
  nanobind::module_ edge = parent.def_submodule("edge", "The edge rung: sparse VO.");
  nanobind::module_ frames = edge.def_submodule("frames", "Frames: features, depth, bearings.");
  bind_frames(frames);
  nanobind::module_ map = edge.def_submodule("map", "The map: keyframes, keypoints, points.");
  bind_map(map);
}

}  // namespace slamlab::bindings
