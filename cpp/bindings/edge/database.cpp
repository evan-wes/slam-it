/// @file
/// @brief Bindings for `slam_it.edge.database` (`slam_it._cpp.edge.database`).
///
/// A database crosses as `Database.as_dict()`'s dict, from which the core builds its own index
/// (`edge_crossing::database_in`); a frame's word vector as its `ids` (int64) and `values`
/// (float64); the other maps as a list of `Indexed.as_dict()`'s dicts
/// (`edge_crossing::indexed_in`). Candidates return as a dict of `keyframes`, `scores`,
/// `sharing` and `scored`, a merge's with `maps` too.
#include "slam_it/edge/database.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief Candidates as a dict.
/// @param found The candidates.
/// @return `keyframes` (int64), `scores` (float64), `sharing` and `scored`.
nb::dict candidates_dict(const edge::Candidates& found) {
  nb::dict out;
  out["keyframes"] = edge_crossing::vector_array(found.keyframes);
  out["scores"] = edge_crossing::vector_array(found.scores);
  out["sharing"] = found.sharing;
  out["scored"] = found.scored;
  return out;
}

/// @brief Find the keyframes a frame may be relocalised against.
/// @param database The database, as a dict.
/// @param m The map, as a dict.
/// @param ids The frame's words, ascending.
/// @param values Their values.
/// @param options The settings, as a dict.
/// @return `keyframes` (int64), `scores` (float64), `sharing` and `scored`.
/// @throws CoreError for invalid input.
nb::dict candidates(const nb::dict& database, const nb::dict& m, const edge_crossing::Int64In& ids,
                    const VectorIn<double>& values, const nb::dict& options) {
  const foundations::vocabulary::BowVector frame{
      .ids = edge_crossing::int64_in(ids), .values = copy_vector<std::vector<double>>(values)};
  return candidates_dict(
      unwrap(edge::candidates(edge_crossing::database_in(database), edge_crossing::map_in(m), frame,
                              edge_crossing::options_in(options))));
}

/// @brief Find the keyframes a keyframe may close a loop with.
/// @param database The database, as a dict.
/// @param m The map, as a dict.
/// @param keyframe The keyframe's id.
/// @param options The settings, as a dict.
/// @return `keyframes` (int64), `scores` (float64), `sharing` and `scored`.
/// @throws CoreError for invalid input.
nb::dict loop_candidates(const nb::dict& database, const nb::dict& m, std::int64_t keyframe,
                         const nb::dict& options) {
  return candidates_dict(
      unwrap(edge::loop_candidates(edge_crossing::database_in(database), edge_crossing::map_in(m),
                                   keyframe, edge_crossing::options_in(options))));
}

/// @brief Find the keyframes of other maps a keyframe may merge its map with.
/// @param ids The keyframe's words, ascending.
/// @param values Their values.
/// @param others The other maps, each a dict of `number`, `m` and `database`.
/// @param options The settings, as a dict.
/// @return `maps` and `keyframes` (int64), `scores` (float64), `sharing` and `scored`.
/// @throws CoreError for invalid input.
nb::dict merge_candidates(const edge_crossing::Int64In& ids, const VectorIn<double>& values,
                          const nb::list& others, const nb::dict& options) {
  const foundations::vocabulary::BowVector vector{
      .ids = edge_crossing::int64_in(ids), .values = copy_vector<std::vector<double>>(values)};
  const std::vector<edge::Indexed> maps = edge_crossing::indexed_in(others);
  edge::MergeCandidates found = unwrap(
      edge::merge_candidates(vector, edge::refs_of(maps), edge_crossing::options_in(options)));
  nb::dict out = candidates_dict({.keyframes = std::move(found.keyframes),
                                  .scores = std::move(found.scores),
                                  .sharing = found.sharing,
                                  .scored = found.scored});
  out["maps"] = edge_crossing::vector_array(found.maps);
  return out;
}

}  // namespace

void bind_database(nb::module_& m) {
  m.def("candidates", &candidates, nb::arg("database"), nb::arg("m"), nb::arg("ids").noconvert(),
        nb::arg("values").noconvert(), nb::arg("options"),
        "Find the keyframes a frame may be relocalised against.");
  m.def("loop_candidates", &loop_candidates, nb::arg("database"), nb::arg("m"), nb::arg("keyframe"),
        nb::arg("options"), "Find the keyframes a keyframe may close a loop with.");
  m.def("merge_candidates", &merge_candidates, nb::arg("ids").noconvert(),
        nb::arg("values").noconvert(), nb::arg("others"), nb::arg("options"),
        "Find the keyframes of other maps a keyframe may merge its map with.");
}

}  // namespace slam_it::bindings
