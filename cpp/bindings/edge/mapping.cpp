/// @file
/// @brief Bindings for `slam_it.edge.mapping` (`slam_it._cpp.edge.mapping`).
///
/// Each step copies the map in, changes the copy, and returns it as a dict; the mode crosses
/// by name, a tracked frame as `pose` and `points`.
#include "slam_it/edge/mapping.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

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
/// @param idle Whether local mapping is idle.
/// @param since_keyframe The frames since the last keyframe, or none.
/// @return Whether to insert a keyframe.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
bool need_keyframe(const nb::dict& m, const nb::dict& camera, const VectorIn<double>& depth,
                   const nb::dict& tracked, std::int64_t reference, const std::string& mode,
                   const nb::dict& options, bool idle, std::optional<std::int64_t> since_keyframe) {
  return unwrap(edge::need_keyframe(map_in(m), camera_in(camera),
                                    copy_vector<std::vector<double>>(depth),
                                    edge_crossing::tracked_in(tracked), reference, mode_in(mode),
                                    options_in(options), idle, since_keyframe));
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

/// @brief The keyframes the newest keyframe's points are fused into.
/// @param m The map, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return Their ids (int64).
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> fuse_targets(const nb::dict& m, const std::string& mode,
                                                  const nb::dict& options) {
  return edge_crossing::vector_array(
      unwrap(edge::fuse_targets(map_in(m), mode_in(mode), options_in(options))));
}

/// @brief The keyframes local BA adjusts.
/// @param m The map, as a dict.
/// @param options The settings, as a dict.
/// @return Their rows (int64).
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> local_ba_keyframes(const nb::dict& m,
                                                        const nb::dict& options) {
  return edge_crossing::vector_array(
      unwrap(edge::local_ba_keyframes(map_in(m), options_in(options))));
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
/// @param options The settings, as a dict.
/// @return A new (Q, 2) int64 array of keypoint rows.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> match_for_triangulation(const nb::dict& m,
                                                             const nb::dict& camera,
                                                             std::int64_t k1, std::int64_t k2,
                                                             const nb::dict& options) {
  const auto pairs = unwrap(
      edge::match_for_triangulation(map_in(m), camera_in(camera), k1, k2, options_in(options)));
  auto [out, data] = new_array<std::int64_t>({pairs.size(), 2});
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
    std::ranges::copy(pairs[i], data + (2 * i));
  }
  return out;
}

/// @brief Triangulate the newest keyframe's free keypoints against its best links.
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

/// @brief Fuse the newest keyframe's points with its neighbours'.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param options The settings, as a dict.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict fuse_points(const nb::dict& m, const nb::dict& camera, const std::string& mode,
                     const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::fuse_points(out, camera_in(camera), mode_in(mode), options_in(options)));
  return map_dict(out);
}

/// @brief Cull the newest keyframe's redundant links.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @param held The ids of the keyframes loop detection holds.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict cull_keyframes(const nb::dict& m, const nb::dict& camera, const std::string& mode,
                        const nb::dict& options, const edge_crossing::Int64In& held) {
  edge::Map out = map_in(m);
  unwrap(edge::cull_keyframes(out, camera_in(camera), mode_in(mode), options_in(options),
                              edge_crossing::int64_in(held)));
  return map_dict(out);
}

/// @brief Insert a keyframe and run local mapping on it.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param frame The keyframe's frame, as a dict.
/// @param tracked Its pose and matches, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @param held The ids of the keyframes loop detection holds.
/// @return The new map, as a dict, how each step went (the summary's counts and `solves`), and
///         `made`.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict process(const nb::dict& m, const nb::dict& camera, const nb::dict& frame,
                 const nb::dict& tracked, const std::string& mode, const nb::dict& options,
                 const edge_crossing::Int64In& held) {
  edge::Map mapped = map_in(m);
  const edge::Mapped done = unwrap(edge::process(
      mapped, camera_in(camera), edge_crossing::frame_in(frame), edge_crossing::tracked_in(tracked),
      mode_in(mode), options_in(options), edge_crossing::int64_in(held)));
  const edge::MappingSummary& summary = done.summary;
  nb::dict out;
  out["map"] = map_dict(mapped);
  out["from_depth"] = summary.from_depth;
  out["culled_points"] = summary.culled_points;
  out["triangulated"] = summary.triangulated;
  out["fused"] = summary.fused;
  out["solves"] = edge_crossing::solves_list(summary.solves);
  out["free"] = summary.free;
  out["fixed"] = summary.fixed;
  out["outliers"] = summary.outliers;
  out["weak_points"] = summary.weak_points;
  out["culled_keyframes"] = summary.culled_keyframes;
  out["made"] = edge_crossing::vector_array(done.made);
  nb::dict schedule;
  schedule["fusion"] = edge::name(done.schedule.fusion);
  schedule["adjustment"] = edge::name(done.schedule.adjustment);
  schedule["culling"] = edge::name(done.schedule.culling);
  out["schedule"] = schedule;
  return out;
}

}  // namespace

void bind_mapping(nb::module_& m) {
  m.def("codes", &edge_crossing::solve_codes, "Each enum's names in code order.");
  m.def("need_keyframe", &need_keyframe, nb::arg("m"), nb::arg("camera"),
        nb::arg("depth").noconvert(), nb::arg("tracked"), nb::arg("reference"), nb::arg("mode"),
        nb::arg("options"), nb::arg("idle"), nb::arg("since_keyframe").none(),
        "Decide whether a tracked frame becomes a keyframe.");
  m.def("create_points_from_depth", &create_points_from_depth, nb::arg("m"), nb::arg("camera"),
        nb::arg("options"), "Create points from the newest keyframe's close depth.");
  m.def("cull_points", &cull_points, nb::arg("m"), nb::arg("mode"), nb::arg("options"),
        "Cull recently created points tracking rarely finds.");
  m.def("match_for_triangulation", &match_for_triangulation, nb::arg("m"), nb::arg("camera"),
        nb::arg("k1"), nb::arg("k2"), nb::arg("options"),
        "Match two keyframes' free keypoints for triangulation.");
  m.def("triangulate_new_points", &triangulate_new_points, nb::arg("m"), nb::arg("camera"),
        nb::arg("mode"), nb::arg("options"),
        "Triangulate the newest keyframe's free keypoints against its best links.");
  m.def("fuse_targets", &fuse_targets, nb::arg("m"), nb::arg("mode"), nb::arg("options"),
        "The keyframes the newest keyframe's points are fused into.");
  m.def("local_ba_keyframes", &local_ba_keyframes, nb::arg("m"), nb::arg("options"),
        "The keyframes local BA adjusts.");
  m.def("fuse_points", &fuse_points, nb::arg("m"), nb::arg("camera"), nb::arg("mode"),
        nb::arg("options"), "Fuse the newest keyframe's points with its neighbours'.");
  m.def("cull_keyframes", &cull_keyframes, nb::arg("m"), nb::arg("camera"), nb::arg("mode"),
        nb::arg("options"), nb::arg("held").noconvert(),
        "Cull the newest keyframe's redundant links.");
  m.def("process", &process, nb::arg("m"), nb::arg("camera"), nb::arg("frame"), nb::arg("tracked"),
        nb::arg("mode"), nb::arg("options"), nb::arg("held").noconvert(),
        "Insert a keyframe and run local mapping on it.");
}

}  // namespace slam_it::bindings
