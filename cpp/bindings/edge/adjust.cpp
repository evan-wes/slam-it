/// @file
/// @brief Bindings for `slamlab.edge.adjust` (`slamlab._cpp.edge.adjust`).
#include "slamlab/edge/adjust.hpp"

#include <nanobind/nanobind.h>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

/// @brief Bundle-adjust keyframes and the points they observe.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param keyframes The keyframes' rows.
/// @param fixed One flag per keyframe.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict bundle_adjust(const nb::dict& m, const nb::dict& camera,
                       const edge_crossing::Int64In& keyframes, const VectorIn<bool>& fixed,
                       const nb::dict& options) {
  edge::Map out = edge_crossing::map_in(m);
  unwrap(edge::bundle_adjust(out, edge_crossing::camera_in(camera),
                             edge_crossing::int64_in(keyframes), edge_crossing::flags_in(fixed),
                             edge_crossing::options_in(options)));
  return edge_crossing::map_dict(out);
}

}  // namespace

void bind_adjust(nb::module_& m) {
  m.def("bundle_adjust", &bundle_adjust, nb::arg("m"), nb::arg("camera"),
        nb::arg("keyframes").noconvert(), nb::arg("fixed").noconvert(), nb::arg("options"),
        "Bundle-adjust keyframes and the points they observe.");
}

}  // namespace slamlab::bindings
