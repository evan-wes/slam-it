/// @file
/// @brief Bindings for `slam_it.edge.relocalization` (`slam_it._cpp.edge.relocalization`).
///
/// The map, its database (`edge_crossing::database_in`), the vocabulary
/// (`vocabulary_crossing::vocabulary_in`), the camera, the frame and the settings cross as
/// dicts. The result is a dict of `tracked` (or None), `matched` and `summary`: `words`,
/// `sharing`, `scored`, `placed` and `candidates`, a list of dicts of each candidate's
/// summary, RANSAC's termination as its code in `codes()` (or None).
#include "slam_it/edge/relocalization.hpp"

#include <nanobind/nanobind.h>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"
#include "foundations/ransac_crossing.hpp"
#include "foundations/vocabulary_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief A candidate's summary as it crosses.
/// @param candidate The summary.
/// @return The dict of its fields.
nb::dict candidate_dict(const edge::CandidateSummary& candidate) {
  nb::dict out;
  out["keyframe"] = candidate.keyframe;
  out["score"] = candidate.score;
  out["matches"] = candidate.matches;
  out["iterations"] = edge_crossing::count_or_none(candidate.iterations);
  out["termination"] = candidate.termination
                           ? nb::object(nb::int_(static_cast<int>(*candidate.termination)))
                           : nb::none();
  out["degenerate"] = candidate.degenerate;
  out["inliers"] = edge_crossing::count_or_none(candidate.inliers);
  out["refined"] = edge_crossing::count_or_none(candidate.refined);
  out["found"] = edge_crossing::count_or_none(candidate.found);
  return out;
}

/// @brief Place a frame in the map through the keyframes that look like it.
/// @param m The map, as a dict.
/// @param database Its database, as a dict.
/// @param vocabulary The vocabulary, as a dict of its fields.
/// @param camera The camera, as a dict.
/// @param frame The frame, as a dict.
/// @param options The settings, as a dict.
/// @return `tracked`, `matched` and `summary`.
/// @throws CoreError for invalid input.
nb::dict relocalize(const nb::dict& m, const nb::dict& database, const nb::dict& vocabulary,
                    const nb::dict& camera, const nb::dict& frame, const nb::dict& options) {
  const edge::Relocalized found = unwrap(edge::relocalize(
      edge_crossing::map_in(m), edge_crossing::database_in(database),
      vocabulary_crossing::vocabulary_in(vocabulary), edge_crossing::camera_in(camera),
      edge_crossing::frame_in(frame), edge_crossing::options_in(options)));
  nb::list tried;
  for (const edge::CandidateSummary& candidate : found.summary.candidates) {
    tried.append(candidate_dict(candidate));
  }
  nb::dict summary;
  summary["words"] = found.summary.words;
  summary["sharing"] = found.summary.sharing;
  summary["scored"] = found.summary.scored;
  summary["candidates"] = tried;
  summary["placed"] = found.summary.placed;
  nb::dict out;
  out["tracked"] = edge_crossing::maybe_tracked(found.tracked);
  out["summary"] = summary;
  out["matched"] = edge_crossing::vector_array(found.matched);
  return out;
}

}  // namespace

void bind_relocalization(nb::module_& m) {
  m.def("codes", &ransac_codes, "Each enum's names in code order.");
  m.def("relocalize", &relocalize, nb::arg("m"), nb::arg("database"), nb::arg("vocabulary"),
        nb::arg("camera"), nb::arg("frame"), nb::arg("options"),
        "Place a frame in the map through the keyframes that look like it.");
}

}  // namespace slam_it::bindings
