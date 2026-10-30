/// @file
/// @brief Bindings for `slam_it.edge.covisibility` (`slam_it._cpp.edge.covisibility`).
///
/// A map crosses as `Map.as_dict()`'s dict, the settings as `Options.as_dict()`'s. Links and the
/// connected keyframes return as a pair of int64 arrays (rows, weights); the local map as a dict
/// of `keyframes` and `reference`; the strong links as an (L, 3) int64 array; a changed map as
/// its dict.
#include "slam_it/edge/covisibility.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

using edge_crossing::map_dict;
using edge_crossing::map_in;
using edge_crossing::options_in;
using edge_crossing::vector_array;

/// @brief Links as Python takes them.
/// @param found The links.
/// @return Their rows and their weights, as int64 arrays.
nb::tuple links_out(const std::vector<edge::Link>& found) {
  std::vector<std::int64_t> rows;
  std::vector<std::int64_t> weights;
  for (const edge::Link& link : found) {
    rows.push_back(link.row);
    weights.push_back(link.weight);
  }
  return nb::make_tuple(vector_array(rows), vector_array(weights));
}

/// @brief A keyframe's links.
/// @param m The map, as a dict.
/// @param k The keyframe's row.
/// @param options The settings, as a dict.
/// @return The linked rows and their weights.
/// @throws CoreError for a row out of range.
nb::tuple links(const nb::dict& m, std::int64_t k, const nb::dict& options) {
  return links_out(unwrap(edge::links(map_in(m), k, options_in(options))));
}

/// @brief A keyframe's connected keyframes.
/// @param m The map, as a dict.
/// @param k The keyframe's row.
/// @return The rows of the keyframes sharing a point with it, and their weights.
/// @throws CoreError for a row out of range.
nb::tuple connected(const nb::dict& m, std::int64_t k) {
  return links_out(unwrap(edge::connected(map_in(m), k)));
}

/// @brief Tracking's local map.
/// @param m The map, as a dict.
/// @param points The point each keypoint was matched to, -1 for none.
/// @param options The settings, as a dict.
/// @return `keyframes` (int64) and `reference`.
/// @throws CoreError for an id below -1.
nb::dict local_map(const nb::dict& m, const edge_crossing::Int64In& points,
                   const nb::dict& options) {
  const edge::LocalMap found =
      unwrap(edge::local_map(map_in(m), edge_crossing::int64_in(points), options_in(options)));
  nb::dict out;
  out["keyframes"] = vector_array(found.keyframes);
  out["reference"] = found.reference;
  return out;
}

/// @brief The keyframes tracking's next frame may read.
/// @param m The map, as a dict.
/// @param last The point each of the last frame's keypoints tracked, -1 for none.
/// @param reference Its reference keyframe's id, or -1.
/// @param options The settings, as a dict.
/// @return The keyframes' rows, ascending.
/// @throws CoreError for an id below -1.
nb::ndarray<nb::numpy, std::int64_t> view_keyframes(const nb::dict& m,
                                                    const edge_crossing::Int64In& last,
                                                    std::int64_t reference,
                                                    const nb::dict& options) {
  return vector_array(unwrap(edge::view_keyframes(map_in(m), edge_crossing::int64_in(last),
                                                  reference, options_in(options))));
}

/// @brief Give a keyframe its parent.
/// @param m The map, as a dict.
/// @param k The keyframe's row.
/// @return The map.
/// @throws CoreError for a row out of range.
nb::dict adopt(const nb::dict& m, std::int64_t k) {
  edge::Map out = map_in(m);
  unwrap(edge::adopt(out, k));
  return map_dict(out);
}

/// @brief Give a keyframe's children new parents.
/// @param m The map, as a dict.
/// @param k The keyframe's row.
/// @param options The settings, as a dict.
/// @return The map.
/// @throws CoreError for a row out of range.
nb::dict reparent(const nb::dict& m, std::int64_t k, const nb::dict& options) {
  edge::Map out = map_in(m);
  unwrap(edge::reparent(out, k, options_in(options)));
  return map_dict(out);
}

/// @brief Cull a keyframe: its children given new parents, then it erased.
/// @param m The map, as a dict.
/// @param k The keyframe's row.
/// @param options The settings, as a dict.
/// @param held The ids of the keyframes loop detection holds.
/// @return The map.
/// @throws CoreError for a row out of range or the map's first keyframe.
nb::dict cull(const nb::dict& m, std::int64_t k, const nb::dict& options,
              const edge_crossing::Int64In& held) {
  edge::Map out = map_in(m);
  unwrap(edge::cull(out, k, options_in(options), edge_crossing::int64_in(held)));
  return map_dict(out);
}

/// @brief The strong links.
/// @param m The map, as a dict.
/// @param options The settings, as a dict.
/// @return An (L, 3) int64 array.
nb::ndarray<nb::numpy, std::int64_t> strong_links(const nb::dict& m, const nb::dict& options) {
  const auto found = edge::strong_links(map_in(m), options_in(options));
  auto [out, data] = new_array<std::int64_t>({found.size(), 3});
  for (std::size_t i = 0; i < found.size(); ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
      data[3 * i + j] = found[i][j];
    }
  }
  return out;
}

}  // namespace

void bind_covisibility(nb::module_& m) {
  m.def("links", &links, nb::arg("m"), nb::arg("k"), nb::arg("options"),
        "A keyframe's links: the rows and the points each shares.");
  m.def("connected", &connected, nb::arg("m"), nb::arg("k"),
        "A keyframe's connected keyframes: every one sharing a point, and the points each shares.");
  m.def("local_map", &local_map, nb::arg("m"), nb::arg("points").noconvert(), nb::arg("options"),
        "Tracking's local map: its keyframes and the reference keyframe.");
  m.def("view_keyframes", &view_keyframes, nb::arg("m"), nb::arg("last").noconvert(),
        nb::arg("reference"), nb::arg("options"), "The keyframes tracking's next frame may read.");
  m.def("adopt", &adopt, nb::arg("m"), nb::arg("k"), "Give a keyframe its parent.");
  m.def("reparent", &reparent, nb::arg("m"), nb::arg("k"), nb::arg("options"),
        "Give a keyframe's children new parents.");
  m.def("cull", &cull, nb::arg("m"), nb::arg("k"), nb::arg("options"), nb::arg("held").noconvert(),
        "Cull a keyframe: its children given new parents, then it erased.");
  m.def("strong_links", &strong_links, nb::arg("m"), nb::arg("options"),
        "Every pair of keyframes sharing at least covisibility.strong_points.");
}

}  // namespace slam_it::bindings
