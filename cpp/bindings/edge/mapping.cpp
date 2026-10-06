/// @file
/// @brief Bindings for `slamlab.edge.mapping` (`slamlab._cpp.edge.mapping`).
///
/// Each step copies the map in, changes the copy, and returns it as a dict; the mode crosses
/// by name, a tracked frame as `pose` and `points`.
#include "slamlab/edge/mapping.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <algorithm>
#include <cstdint>
#include <string>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

using edge_crossing::camera_in;
using edge_crossing::map_dict;
using edge_crossing::map_in;
using edge_crossing::mode_in;
using edge_crossing::options_in;

/// @brief Decide whether a tracked frame becomes a keyframe.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param depth The frame's keypoints' depths.
/// @param tracked Its pose and matches, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return Whether to insert a keyframe.
/// @throws CoreError for invalid input.
bool need_keyframe(const nb::dict& m, const nb::dict& camera, const VectorIn<double>& depth,
                   const nb::dict& tracked, const std::string& mode, const nb::dict& options) {
  return unwrap(
      edge::need_keyframe(map_in(m), camera_in(camera), copy_vector<std::vector<double>>(depth),
                          edge_crossing::tracked_in(tracked), mode_in(mode), options_in(options)));
}

/// @brief Create points from the newest keyframe's close depth.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict create_points_from_depth(const nb::dict& m, const nb::dict& camera,
                                  const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::create_points_from_depth(out, camera_in(camera), options_in(options)));
  return map_dict(out);
}

/// @brief Cull recently created points tracking rarely finds.
/// @param m The map, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict cull_points(const nb::dict& m, const std::string& mode, const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::cull_points(out, mode_in(mode), options_in(options)));
  return map_dict(out);
}

/// @brief Match two keyframes' free keypoints for triangulation.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @return A new (Q, 2) int64 array of keypoint rows.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> match_for_triangulation(const nb::dict& m,
                                                             const nb::dict& camera,
                                                             std::int64_t k1, std::int64_t k2) {
  const auto pairs = unwrap(edge::match_for_triangulation(map_in(m), camera_in(camera), k1, k2));
  auto [out, data] = new_array<std::int64_t>({pairs.size(), 2});
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
    std::ranges::copy(pairs[i], data + (2 * i));
  }
  return out;
}

/// @brief Triangulate the newest keyframe's free keypoints against the window's.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict triangulate_new_points(const nb::dict& m, const nb::dict& camera, const std::string& mode,
                                const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::triangulate_new_points(out, camera_in(camera), mode_in(mode), options_in(options)));
  return map_dict(out);
}

/// @brief Fuse the newest keyframe's points with the window's.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict fuse_points(const nb::dict& m, const nb::dict& camera, const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::fuse_points(out, camera_in(camera), options_in(options)));
  return map_dict(out);
}

/// @brief Cull the window's redundant keyframes.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict cull_keyframes(const nb::dict& m, const nb::dict& camera, const std::string& mode,
                        const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::cull_keyframes(out, camera_in(camera), mode_in(mode), options_in(options)));
  return map_dict(out);
}

/// @brief Insert a keyframe and run local mapping on it.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param frame The keyframe's frame, as a dict.
/// @param tracked Its pose and matches, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict process(const nb::dict& m, const nb::dict& camera, const nb::dict& frame,
                 const nb::dict& tracked, const std::string& mode, const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::process(out, camera_in(camera), edge_crossing::frame_in(frame),
                       edge_crossing::tracked_in(tracked), mode_in(mode), options_in(options)));
  return map_dict(out);
}

}  // namespace

void bind_mapping(nb::module_& m) {
  m.def("need_keyframe", &need_keyframe, nb::arg("m"), nb::arg("camera"),
        nb::arg("depth").noconvert(), nb::arg("tracked"), nb::arg("mode"), nb::arg("options"),
        "Decide whether a tracked frame becomes a keyframe.");
  m.def("create_points_from_depth", &create_points_from_depth, nb::arg("m"), nb::arg("camera"),
        nb::arg("options"), "Create points from the newest keyframe's close depth.");
  m.def("cull_points", &cull_points, nb::arg("m"), nb::arg("mode"), nb::arg("options"),
        "Cull recently created points tracking rarely finds.");
  m.def("match_for_triangulation", &match_for_triangulation, nb::arg("m"), nb::arg("camera"),
        nb::arg("k1"), nb::arg("k2"), "Match two keyframes' free keypoints for triangulation.");
  m.def("triangulate_new_points", &triangulate_new_points, nb::arg("m"), nb::arg("camera"),
        nb::arg("mode"), nb::arg("options"),
        "Triangulate the newest keyframe's free keypoints against the window's.");
  m.def("fuse_points", &fuse_points, nb::arg("m"), nb::arg("camera"), nb::arg("options"),
        "Fuse the newest keyframe's points with the window's.");
  m.def("cull_keyframes", &cull_keyframes, nb::arg("m"), nb::arg("camera"), nb::arg("mode"),
        nb::arg("options"), "Cull the window's redundant keyframes.");
  m.def("process", &process, nb::arg("m"), nb::arg("camera"), nb::arg("frame"), nb::arg("tracked"),
        nb::arg("mode"), nb::arg("options"), "Insert a keyframe and run local mapping on it.");
}

}  // namespace slamlab::bindings
