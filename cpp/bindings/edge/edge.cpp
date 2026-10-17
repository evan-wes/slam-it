/// @file
/// @brief The `slam_it._cpp.edge` submodule: wires the edge rung's bound modules.
#include "edge/edge.hpp"

namespace slam_it::bindings {

void bind_edge(nanobind::module_& parent) {
  nanobind::module_ edge = parent.def_submodule("edge", "The edge rung: sparse VO.");
  nanobind::module_ frames = edge.def_submodule("frames", "Frames: features, depth, bearings.");
  bind_frames(frames);
  nanobind::module_ map = edge.def_submodule("map", "The map: keyframes, keypoints, points.");
  bind_map(map);
  nanobind::module_ residuals = edge.def_submodule("residuals", "Observation errors.");
  bind_residuals(residuals);
  nanobind::module_ adjust = edge.def_submodule("adjust", "Bundle adjustment over the map.");
  bind_adjust(adjust);
  nanobind::module_ initialization =
      edge.def_submodule("initialization", "Initialisation: starting the map.");
  bind_initialization(initialization);
  nanobind::module_ tracking = edge.def_submodule("tracking", "Tracking: placing each frame.");
  bind_tracking(tracking);
  nanobind::module_ mapping = edge.def_submodule("mapping", "Local mapping around each keyframe.");
  bind_mapping(mapping);
  nanobind::module_ covisibility =
      edge.def_submodule("covisibility", "The covisibility graph and the spanning tree.");
  bind_covisibility(covisibility);
  nanobind::module_ database =
      edge.def_submodule("database", "The keyframe database: place recognition's index.");
  bind_database(database);
  nanobind::module_ relocalization =
      edge.def_submodule("relocalization", "Relocalisation: placing a lost frame by its words.");
  bind_relocalization(relocalization);
}

}  // namespace slam_it::bindings
