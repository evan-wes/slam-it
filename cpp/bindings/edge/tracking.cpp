/// @file
/// @brief Bindings for `slam_it_all.edge.tracking` (`slam_it_all._cpp.edge.tracking`).
///
/// A tracked frame crosses as a dict of `pose` and `points` (or None where the stage may
/// fail); `InView` and `LocalTracking` as dicts of their fields, the latter's tracked frame
/// flattened into `pose` and `points`. The mode crosses by name.
#include "slam_it_all/edge/tracking.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <string>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it_all::bindings {

namespace {

using edge_crossing::camera_in;
using edge_crossing::frame_in;
using edge_crossing::map_in;
using edge_crossing::options_in;

/// @brief Refine a frame's pose against its matched points.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param frame The frame, as a dict.
/// @param tracked The starting pose and the matches, as a dict.
/// @param options The settings, as a dict.
/// @return The tracked frame, as a dict.
/// @throws CoreError for invalid input.
nb::dict optimize_pose(const nb::dict& m, const nb::dict& camera, const nb::dict& frame,
                       const nb::dict& tracked, const nb::dict& options) {
  return edge_crossing::tracked_dict(
      unwrap(edge::optimize_pose(map_in(m), camera_in(camera), frame_in(frame),
                                 edge_crossing::tracked_in(tracked), options_in(options))));
}

/// @brief Select the points a camera sees.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param pose Its pose.
/// @param rows The candidates' rows.
/// @param options The settings, as a dict.
/// @return `rows`, `xy`, `right`, `level` and `view_cos`.
/// @throws CoreError for invalid input.
nb::dict in_view(const nb::dict& m, const nb::dict& camera, const Matrix4In& pose,
                 const edge_crossing::Int64In& rows, const nb::dict& options) {
  const edge::InView view =
      unwrap(edge::in_view(map_in(m), camera_in(camera), matrix4(pose),
                           edge_crossing::int64_in(rows), options_in(options)));
  nb::dict out;
  out["rows"] = edge_crossing::vector_array(view.rows);
  out["xy"] = edge_crossing::pixels_array(view.xy);
  out["right"] = edge_crossing::vector_array(view.right);
  out["level"] = edge_crossing::vector_array(view.level);
  out["view_cos"] = edge_crossing::vector_array(view.view_cos);
  return out;
}

/// @brief Track the last frame's points into this one.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param frame The frame, as a dict.
/// @param last The last frame, as a dict.
/// @param last_tracked Its pose and matches, as a dict.
/// @param predicted The predicted pose.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return The tracked frame as a dict, or None.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the reference's argument order
nb::object track_last_frame(const nb::dict& m, const nb::dict& camera, const nb::dict& frame,
                            const nb::dict& last, const nb::dict& last_tracked,
                            const Matrix4In& predicted, const std::string& mode,
                            const nb::dict& options) {
  return edge_crossing::maybe_tracked(
      unwrap(edge::track_last_frame(map_in(m), camera_in(camera), frame_in(frame), frame_in(last),
                                    edge_crossing::tracked_in(last_tracked), matrix4(predicted),
                                    edge_crossing::mode_in(mode), options_in(options))));
}

/// @brief Match the local map's points in view and refine the pose.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param frame The frame, as a dict.
/// @param tracked Its pose and matches so far, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @param recovering Whether the frame is a recovery's or the next.
/// @return `pose`, `points`, `visible` and `found`.
/// @throws CoreError for invalid input.
nb::dict track_local_map(const nb::dict& m, const nb::dict& camera, const nb::dict& frame,
                         const nb::dict& tracked, const std::string& mode, const nb::dict& options,
                         bool recovering) {
  const edge::LocalTracking local = unwrap(edge::track_local_map(
      map_in(m), camera_in(camera), frame_in(frame), edge_crossing::tracked_in(tracked),
      edge_crossing::mode_in(mode), options_in(options), recovering));
  nb::dict out = edge_crossing::tracked_dict(local.tracked);
  out["visible"] = edge_crossing::vector_array(local.visible);
  out["found"] = edge_crossing::vector_array(local.found);
  return out;
}

/// @brief Place a frame against the local map by descriptors and PnP RANSAC.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param frame The frame, as a dict.
/// @param options The settings, as a dict.
/// @return The tracked frame as a dict, or None.
/// @throws CoreError for invalid input.
nb::object relocalize_locally(const nb::dict& m, const nb::dict& camera, const nb::dict& frame,
                              const nb::dict& options) {
  return edge_crossing::maybe_tracked(unwrap(edge::relocalize_locally(
      map_in(m), camera_in(camera), frame_in(frame), options_in(options))));
}

}  // namespace

void bind_tracking(nb::module_& m) {
  m.def("optimize_pose", &optimize_pose, nb::arg("m"), nb::arg("camera"), nb::arg("frame"),
        nb::arg("tracked"), nb::arg("options"),
        "Refine a frame's pose against its matched points.");
  m.def("in_view", &in_view, nb::arg("m"), nb::arg("camera"), nb::arg("pose").noconvert(),
        nb::arg("rows").noconvert(), nb::arg("options"), "Select the points a camera sees.");
  m.def("track_last_frame", &track_last_frame, nb::arg("m"), nb::arg("camera"), nb::arg("frame"),
        nb::arg("last"), nb::arg("last_tracked"), nb::arg("predicted").noconvert(), nb::arg("mode"),
        nb::arg("options"), "Track the last frame's points into this one.");
  m.def("track_local_map", &track_local_map, nb::arg("m"), nb::arg("camera"), nb::arg("frame"),
        nb::arg("tracked"), nb::arg("mode"), nb::arg("options"), nb::arg("recovering"),
        "Match the local map's points in view and refine the pose.");
  m.def("relocalize_locally", &relocalize_locally, nb::arg("m"), nb::arg("camera"),
        nb::arg("frame"), nb::arg("options"),
        "Place a frame against the local map by descriptors and PnP RANSAC.");
}

}  // namespace slam_it_all::bindings
