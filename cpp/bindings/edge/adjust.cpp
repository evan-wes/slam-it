/// @file
/// @brief Bindings for `slam_it.edge.adjust` (`slam_it._cpp.edge.adjust`).
///
/// An adjustment crosses back as a dict of `map`, `solves` and `outliers`.
#include "slam_it/edge/adjust.hpp"

#include <nanobind/nanobind.h>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief Bundle-adjust keyframes and the points they observe.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param keyframes The keyframes' rows.
/// @param fixed One flag per keyframe.
/// @param options The settings, as a dict.
/// @return The new map, as a dict, the solves and the outliers dropped.
/// @throws CoreError for invalid input.
nb::dict bundle_adjust(const nb::dict& m, const nb::dict& camera,
                       const edge_crossing::Int64In& keyframes, const VectorIn<bool>& fixed,
                       const nb::dict& options) {
  edge::Map adjusted = edge_crossing::map_in(m);
  const edge::Adjustment adjustment = unwrap(edge::bundle_adjust(
      adjusted, edge_crossing::camera_in(camera), edge_crossing::int64_in(keyframes),
      edge_crossing::flags_in(fixed), edge_crossing::options_in(options)));
  nb::dict out;
  out["map"] = edge_crossing::map_dict(adjusted);
  out["solves"] = edge_crossing::solves_list(adjustment.solves);
  out["outliers"] = adjustment.outliers;
  out["free"] = adjustment.free;
  out["fixed"] = adjustment.fixed;
  return out;
}

}  // namespace

void bind_adjust(nb::module_& m) {
  m.def("codes", &edge_crossing::solve_codes, "Each enum's names in code order.");
  m.def("bundle_adjust", &bundle_adjust, nb::arg("m"), nb::arg("camera"),
        nb::arg("keyframes").noconvert(), nb::arg("fixed").noconvert(), nb::arg("options"),
        "Bundle-adjust keyframes and the points they observe.");
}

}  // namespace slam_it::bindings
