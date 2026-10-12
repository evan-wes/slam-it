/// @file
/// @brief Bindings for `slam_it.edge.initialization` (`slam_it._cpp.edge.initialization`).
///
/// An attempt crosses back as `edge_crossing::initialization_dict` builds it.
#include "slam_it/edge/initialization.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

using edge_crossing::frame_in;
using edge_crossing::initialization_dict;
using edge_crossing::options_in;

/// @brief Matches as they cross: (M, 2) int64 keypoint pairs.
using Pairs = nb::ndarray<const std::int64_t, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;

/// @brief Start the map from a stereo or RGB-D frame.
/// @param frame The frame, as a dict.
/// @param options The settings, as a dict.
/// @return The attempt, as a dict.
/// @throws CoreError for invalid input.
nb::dict initialize_from_depth(const nb::dict& frame, const nb::dict& options) {
  return initialization_dict(
      unwrap(edge::initialize_from_depth(frame_in(frame), options_in(options))));
}

/// @brief Match a monocular reference frame to a later one.
/// @param reference The reference frame, as a dict.
/// @param frame The later frame, as a dict.
/// @param previous Where each reference keypoint was last matched (N, 2).
/// @param options The settings, as a dict.
/// @return `pairs` (M, 2) and `distances` (M,), int64.
/// @throws CoreError for invalid input.
nb::dict match_for_initialization(const nb::dict& reference, const nb::dict& frame,
                                  const BatchIn<2>& previous, const nb::dict& options) {
  std::vector<std::array<double, 2>> positions(previous.shape(0));
  for (std::size_t i = 0; i < positions.size(); ++i) {
    positions[i] = {previous(i, 0), previous(i, 1)};
  }
  const auto found = unwrap(edge::match_for_initialization(frame_in(reference), frame_in(frame),
                                                           positions, options_in(options)));
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
/// @param pairs The matches (M, 2), int64.
/// @param options The settings, as a dict.
/// @return The attempt, as a dict.
/// @throws CoreError for invalid input.
nb::dict initialize_monocular(const nb::dict& camera, const nb::dict& reference,
                              const nb::dict& frame, const Pairs& pairs, const nb::dict& options) {
  std::vector<std::array<std::int64_t, 2>> matches(pairs.shape(0));
  for (std::size_t i = 0; i < matches.size(); ++i) {
    matches[i] = {pairs(i, 0), pairs(i, 1)};
  }
  return initialization_dict(
      unwrap(edge::initialize_monocular(edge_crossing::camera_in(camera), frame_in(reference),
                                        frame_in(frame), matches, options_in(options))));
}

/// @brief The enum tables: the outcomes and the solves' terminations, names in code order.
/// @return `{"outcome": [...], "nls_termination": [...]}`.
nb::dict codes() {
  nb::dict out = edge_crossing::solve_codes();
  nb::list outcomes;
  for (int k = 0; k < static_cast<int>(edge::Outcome::kCount); ++k) {
    outcomes.append(edge::name(static_cast<edge::Outcome>(k)));
  }
  out["outcome"] = outcomes;
  return out;
}

}  // namespace

void bind_initialization(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("initialize_from_depth", &initialize_from_depth, nb::arg("frame"), nb::arg("options"),
        "Start the map from a stereo or RGB-D frame.");
  m.def("match_for_initialization", &match_for_initialization, nb::arg("reference"),
        nb::arg("frame"), nb::arg("previous").noconvert(), nb::arg("options"),
        "Match a monocular reference frame to a later one, where each was last matched.");
  m.def("initialize_monocular", &initialize_monocular, nb::arg("camera"), nb::arg("reference"),
        nb::arg("frame"), nb::arg("pairs").noconvert(), nb::arg("options"),
        "Start the map from two monocular frames and their matches.");
}

}  // namespace slam_it::bindings
