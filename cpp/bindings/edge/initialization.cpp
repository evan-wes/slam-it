/// @file
/// @brief Bindings for `slamlab.edge.initialization` (`slamlab._cpp.edge.initialization`).
///
/// A started map crosses back as a dict of `map`, `pose` and `points`, or None.
#include "slamlab/edge/initialization.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

using edge_crossing::frame_in;
using edge_crossing::options_in;
using edge_crossing::started_dict;

/// @brief Start the map from a stereo or RGB-D frame.
/// @param frame The frame, as a dict.
/// @param options The settings, as a dict.
/// @return The started map as a dict, or None.
/// @throws CoreError for invalid input.
nb::object initialize_from_depth(const nb::dict& frame, const nb::dict& options) {
  return started_dict(unwrap(edge::initialize_from_depth(frame_in(frame), options_in(options))));
}

/// @brief Match a monocular reference frame to a later one.
/// @param reference The reference frame, as a dict.
/// @param frame The later frame, as a dict.
/// @param options The settings, as a dict.
/// @return `pairs` (M, 2) and `distances` (M,), int64.
/// @throws CoreError for invalid input.
nb::dict match_for_initialization(const nb::dict& reference, const nb::dict& frame,
                                  const nb::dict& options) {
  const auto found = unwrap(
      edge::match_for_initialization(frame_in(reference), frame_in(frame), options_in(options)));
  std::vector<std::int64_t> pairs;
  std::vector<std::int64_t> distances;
  for (const foundations::matching::Match& match : found) {
    pairs.push_back(match.first);
    pairs.push_back(match.second);
    distances.push_back(match.distance);
  }
  auto [array, data] = new_array<std::int64_t>({found.size(), 2});
  std::ranges::copy(pairs, data);
  nb::dict out;
  out["pairs"] = array;
  out["distances"] = edge_crossing::vector_array(distances);
  return out;
}

/// @brief Start the map from two monocular frames.
/// @param camera The camera, as a dict.
/// @param reference The reference frame, as a dict.
/// @param frame The later frame, as a dict.
/// @param options The settings, as a dict.
/// @return The started map as a dict, or None.
/// @throws CoreError for invalid input.
nb::object initialize_monocular(const nb::dict& camera, const nb::dict& reference,
                                const nb::dict& frame, const nb::dict& options) {
  return started_dict(
      unwrap(edge::initialize_monocular(edge_crossing::camera_in(camera), frame_in(reference),
                                        frame_in(frame), options_in(options))));
}

}  // namespace

void bind_initialization(nb::module_& m) {
  m.def("initialize_from_depth", &initialize_from_depth, nb::arg("frame"), nb::arg("options"),
        "Start the map from a stereo or RGB-D frame.");
  m.def("match_for_initialization", &match_for_initialization, nb::arg("reference"),
        nb::arg("frame"), nb::arg("options"), "Match a monocular reference frame to a later one.");
  m.def("initialize_monocular", &initialize_monocular, nb::arg("camera"), nb::arg("reference"),
        nb::arg("frame"), nb::arg("options"), "Start the map from two monocular frames.");
}

}  // namespace slamlab::bindings
