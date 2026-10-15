/// @file
/// @brief Bindings for `slam_it.edge.database` (`slam_it._cpp.edge.database`).
///
/// A database crosses as `Database.as_dict()`'s dict, from which the core builds its own index
/// (`edge_crossing::database_in`); a frame's word vector as its `ids` (int64) and `values`
/// (float64). Candidates return as a dict of `keyframes`, `scores`, `sharing` and `scored`.
#include "slam_it/edge/database.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

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
  const edge::Candidates found =
      unwrap(edge::candidates(edge_crossing::database_in(database), edge_crossing::map_in(m), frame,
                              edge_crossing::options_in(options)));
  nb::dict out;
  out["keyframes"] = edge_crossing::vector_array(found.keyframes);
  out["scores"] = edge_crossing::vector_array(found.scores);
  out["sharing"] = found.sharing;
  out["scored"] = found.scored;
  return out;
}

}  // namespace

void bind_database(nb::module_& m) {
  m.def("candidates", &candidates, nb::arg("database"), nb::arg("m"), nb::arg("ids").noconvert(),
        nb::arg("values").noconvert(), nb::arg("options"),
        "Find the keyframes a frame may be relocalised against.");
}

}  // namespace slam_it::bindings
