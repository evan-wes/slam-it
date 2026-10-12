/// @file
/// @brief Bindings for `slam_it.edge.map` (`slam_it._cpp.edge.map`).
///
/// The map crosses as a dict of `Map`'s fields both ways (`edge_crossing.hpp`); an operation
/// copies it in, changes the copy, and returns it.
#include "slam_it/edge/map.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

using edge_crossing::int64_in;
using edge_crossing::Int64In;
using edge_crossing::map_dict;
using edge_crossing::map_in;

/// @brief Predict points' levels from their distances.
/// @param m The map, as a dict.
/// @param rows The points' rows.
/// @param distance Their distances.
/// @return A new int64 array.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> predict_level(const nb::dict& m, const Int64In& rows,
                                                   const VectorIn<double>& distance) {
  return edge_crossing::vector_array(unwrap(
      edge::predict_level(map_in(m), int64_in(rows), copy_vector<std::vector<double>>(distance))));
}

/// @brief Add a keyframe.
/// @param m The map, as a dict.
/// @param frame The frame, as a dict.
/// @param pose Its `T_cam_world`.
/// @param points The point id each keypoint observes, or -1.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict insert_keyframe(const nb::dict& m, const nb::dict& frame, const Matrix4In& pose,
                         const Int64In& points) {
  edge::Map out = map_in(m);
  unwrap(
      edge::insert_keyframe(out, edge_crossing::frame_in(frame), matrix4(pose), int64_in(points)));
  return map_dict(out);
}

/// @brief Create map points.
/// @param m The map, as a dict.
/// @param xyz The points, (Q, 3).
/// @param observers Each point's observing keypoint rows, (Q, V), -1 padded.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict add_points(const nb::dict& m, const BatchIn<3>& xyz,
                    const nb::ndarray<const std::int64_t, nb::shape<-1, -1>, nb::c_contig,
                                      nb::device::cpu>& observers) {
  if (observers.shape(0) != xyz.shape(0)) {
    throw std::invalid_argument("observers: expected one row per point");
  }
  std::vector<Eigen::Vector3d> points(xyz.shape(0));
  std::vector<std::vector<std::int64_t>> rows(xyz.shape(0));
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i] = {xyz(i, 0), xyz(i, 1), xyz(i, 2)};
    for (std::size_t v = 0; v < observers.shape(1); ++v) {
      if (observers(i, v) >= 0) {
        rows[i].push_back(observers(i, v));
      }
    }
  }
  edge::Map out = map_in(m);
  unwrap(edge::add_points(out, points, rows));
  return map_dict(out);
}

/// @brief Pick the most central descriptor.
/// @param descriptors The descriptors, (n, 32) uint8.
/// @return Its index.
/// @throws CoreError for none.
std::int64_t distinctive_descriptor(
    const nb::ndarray<const std::uint8_t, nb::shape<-1, foundations::orb::kDescriptorBytes>,
                      nb::c_contig, nb::device::cpu>& descriptors) {
  std::vector<foundations::orb::Descriptor> rows(descriptors.shape(0));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    for (std::size_t b = 0; b < foundations::orb::kDescriptorBytes; ++b) {
      rows[i].at(b) = descriptors(i, b);
    }
  }
  return unwrap(edge::distinctive_descriptor(rows));
}

/// @brief Recompute points' descriptors, normals and distances.
/// @param m The map, as a dict.
/// @param rows The points' rows.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict refresh_points(const nb::dict& m, const Int64In& rows) {
  edge::Map out = map_in(m);
  unwrap(edge::refresh_points(out, int64_in(rows)));
  return map_dict(out);
}

/// @brief Remove points, kept aside with why they went.
/// @param m The map, as a dict.
/// @param erase One boolean per point.
/// @param how Why they go: a `Removal` code.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict erase_points(const nb::dict& m, const VectorIn<bool>& erase, int how) {
  edge::Map out = map_in(m);
  if (how < 0 || how >= static_cast<int>(edge::Removal::kCount)) {
    throw std::invalid_argument("how: expected a Removal code");
  }
  unwrap(edge::erase_points(out, edge_crossing::flags_in(erase), static_cast<edge::Removal>(how)));
  return map_dict(out);
}

/// @brief Weigh points' observations.
/// @param m The map, as a dict.
/// @param rows The points' rows.
/// @return A new int64 array.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> observation_weight(const nb::dict& m, const Int64In& rows) {
  return edge_crossing::vector_array(unwrap(edge::observation_weight(map_in(m), int64_in(rows))));
}

/// @brief Remove keypoints' observations.
/// @param m The map, as a dict.
/// @param rows Keypoint rows.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict forget_observations(const nb::dict& m, const Int64In& rows) {
  edge::Map out = map_in(m);
  unwrap(edge::forget_observations(out, int64_in(rows)));
  return map_dict(out);
}

/// @brief Remove a keyframe.
/// @param m The map, as a dict.
/// @param k The keyframe's row.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict erase_keyframe(const nb::dict& m, std::int64_t k) {
  edge::Map out = map_in(m);
  unwrap(edge::erase_keyframe(out, k));
  return map_dict(out);
}

/// @brief Add to points' visible and found counts.
/// @param m The map, as a dict.
/// @param ids Point ids.
/// @param visible Each id's visible increment.
/// @param found Each id's found increment.
/// @return The new map, as a dict.
/// @throws CoreError for invalid input.
nb::dict count(const nb::dict& m, const Int64In& ids, const Int64In& visible,
               const Int64In& found) {
  edge::Map out = map_in(m);
  unwrap(edge::count(out, int64_in(ids), int64_in(visible), int64_in(found)));
  return map_dict(out);
}

}  // namespace

void bind_map(nb::module_& m) {
  m.def("predict_level", &predict_level, nb::arg("m"), nb::arg("rows").noconvert(),
        nb::arg("distance").noconvert(), "Predict points' levels from their distances.");
  m.def("insert_keyframe", &insert_keyframe, nb::arg("m"), nb::arg("frame"),
        nb::arg("pose").noconvert(), nb::arg("points").noconvert(), "Add a keyframe.");
  m.def("add_points", &add_points, nb::arg("m"), nb::arg("xyz").noconvert(),
        nb::arg("observers").noconvert(), "Create map points.");
  m.def("distinctive_descriptor", &distinctive_descriptor, nb::arg("descriptors").noconvert(),
        "Pick the most central descriptor.");
  m.def("refresh_points", &refresh_points, nb::arg("m"), nb::arg("rows").noconvert(),
        "Recompute points' descriptors, normals and distances.");
  m.def("erase_points", &erase_points, nb::arg("m"), nb::arg("erase").noconvert(), nb::arg("how"),
        "Remove points, keeping them aside with why they went.");
  m.def("observation_weight", &observation_weight, nb::arg("m"), nb::arg("rows").noconvert(),
        "Weigh points' observations.");
  m.def("forget_observations", &forget_observations, nb::arg("m"), nb::arg("rows").noconvert(),
        "Remove keypoints' observations.");
  m.def("erase_keyframe", &erase_keyframe, nb::arg("m"), nb::arg("k"), "Remove a keyframe.");
  m.def("count", &count, nb::arg("m"), nb::arg("ids").noconvert(), nb::arg("visible").noconvert(),
        nb::arg("found").noconvert(), "Add to points' visible and found counts.");
}

}  // namespace slam_it::bindings
