/// @file
/// @brief Bindings for `slam_it.edge.loops` (`slam_it._cpp.edge.loops`).
///
/// The map, its database (`edge_crossing::database_in`), the camera and the settings cross as
/// dicts; a similarity as a (4, 4) array; a loop being confirmed and a closed loop as dicts of
/// their fields (`slam_it.edge.loops._pending_as_dict` and `_loop_as_dict`); the mode by name.
/// A detection crosses back as a dict of `pending` and `loop` (each a dict or None) and
/// `summary`, its outcome as its code in `codes()["loop_outcome"]`; a correction as a dict of
/// `map` and `summary`, each solve's termination as its code in `codes()["graphs_termination"]`.
#include "slam_it/edge/loops.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "common.hpp"
#include "edge/edge.hpp"
#include "edge/edge_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace graphs = foundations::graphs;
namespace loops = edge::loops;
using edge_crossing::count_or_none;
using edge_crossing::int64_in;
using edge_crossing::Int64In;
using edge_crossing::vector_array;

/// @brief A solve's summary as it crosses.
/// @param summary The summary.
/// @return `initial_error`, `final_error`, `iterations` and `termination` (its code).
nb::dict solve_dict(const graphs::Summary& summary) {
  nb::dict out;
  out["initial_error"] = summary.initial_error;
  out["final_error"] = summary.final_error;
  out["iterations"] = summary.iterations;
  out["termination"] = static_cast<int>(summary.termination);
  return out;
}

/// @brief A (4, 4) item of a dict.
/// @param values The dict.
/// @param key The item's key.
/// @return The matrix.
Eigen::Matrix4d matrix_item(const nb::dict& values, const char* key) {
  return matrix4(edge_crossing::item<double, nb::shape<4, 4>>(values, key));
}

/// @brief A loop being confirmed, crossing in.
/// @param values Its fields.
/// @return The loop.
loops::Pending pending_in(const nb::dict& values) {
  return {.matched = nb::cast<std::int64_t>(values["matched"]),
          .last = nb::cast<std::int64_t>(values["last"]),
          .similarity = matrix_item(values, "similarity"),
          .confirmations = nb::cast<std::int64_t>(values["confirmations"]),
          .misses = nb::cast<std::int64_t>(values["misses"]),
          .points = edge_crossing::vector_item<std::int64_t>(values, "points"),
          .matched_points = edge_crossing::vector_item<std::int64_t>(values, "matched_points")};
}

/// @brief A loop being confirmed, crossing out.
/// @param pending The loop.
/// @return Its fields.
nb::dict pending_dict(const loops::Pending& pending) {
  nb::dict out;
  out["matched"] = pending.matched;
  out["last"] = pending.last;
  out["similarity"] = matrix_array(pending.similarity);
  out["confirmations"] = pending.confirmations;
  out["misses"] = pending.misses;
  out["points"] = vector_array(pending.points);
  out["matched_points"] = vector_array(pending.matched_points);
  return out;
}

/// @brief A closed loop, crossing in.
/// @param values Its fields.
/// @return The loop.
loops::Loop loop_in(const nb::dict& values) {
  return {.current = nb::cast<std::int64_t>(values["current"]),
          .matched = nb::cast<std::int64_t>(values["matched"]),
          .similarity = matrix_item(values, "similarity"),
          .points = edge_crossing::vector_item<std::int64_t>(values, "points"),
          .matched_points = edge_crossing::vector_item<std::int64_t>(values, "matched_points")};
}

/// @brief A closed loop, crossing out.
/// @param loop The loop.
/// @return Its fields.
nb::dict loop_dict(const loops::Loop& loop) {
  nb::dict out;
  out["current"] = loop.current;
  out["matched"] = loop.matched;
  out["similarity"] = matrix_array(loop.similarity);
  out["points"] = vector_array(loop.points);
  out["matched_points"] = vector_array(loop.matched_points);
  return out;
}

/// @brief A detection's summary, crossing out.
/// @param summary The summary.
/// @return Its fields, the outcome as its code.
nb::dict detection_summary_dict(const loops::DetectionSummary& summary) {
  nb::list candidates;
  for (const loops::CandidateSummary& c : summary.candidates) {
    nb::dict item;
    item["keyframe"] = c.keyframe;
    item["score"] = c.score;
    item["near"] = c.near;
    item["matches"] = count_or_none(c.matches);
    item["best"] = count_or_none(c.best);
    item["inliers"] = count_or_none(c.inliers);
    item["projected"] = count_or_none(c.projected);
    item["refined"] = count_or_none(c.refined);
    item["reprojected"] = count_or_none(c.reprojected);
    item["confirmations"] = count_or_none(c.confirmations);
    candidates.append(item);
  }
  nb::object confirmation = nb::none();
  if (summary.confirmation) {
    nb::dict carried;
    carried["matched"] = summary.confirmation->matched;
    carried["projected"] = summary.confirmation->projected;
    carried["refined"] = count_or_none(summary.confirmation->refined);
    carried["reprojected"] = count_or_none(summary.confirmation->reprojected);
    carried["confirmed"] = summary.confirmation->confirmed;
    confirmation = carried;
  }
  nb::dict out;
  out["outcome"] = static_cast<int>(summary.outcome);
  out["confirmation"] = confirmation;
  out["sharing"] = summary.sharing;
  out["scored"] = summary.scored;
  out["candidates"] = candidates;
  out["confirmations"] = summary.confirmations;
  return out;
}

/// @brief The enums' name tables, in code order.
/// @return `{"loop_outcome": [...], "graphs_termination": [...]}`.
nb::dict codes() {
  nb::dict out;
  out["loop_outcome"] = enum_names<loops::Outcome>([](loops::Outcome o) { return loops::name(o); });
  out["graphs_termination"] =
      enum_names<graphs::Termination>([](graphs::Termination t) { return graphs::name(t); });
  return out;
}

/// @brief Match points to a keyframe's keypoints through a similarity.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param k The keyframe's row.
/// @param similarity Its `S_cw`.
/// @param points The points' rows.
/// @param window The window's half-width.
/// @param distance The largest Hamming distance.
/// @param options The settings, as a dict.
/// @return Each keypoint's match.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::ndarray<nb::numpy, std::int64_t> search_by_similarity(
    const nb::dict& m, const nb::dict& camera, std::int64_t k, const Matrix4In& similarity,
    const Int64In& points, double window, std::int64_t distance, const nb::dict& options) {
  return vector_array(unwrap(loops::search_by_similarity(
      edge_crossing::map_in(m), edge_crossing::camera_in(camera), k, matrix4(similarity),
      int64_in(points), {.radius = window, .distance = distance},
      edge_crossing::options_in(options))));
}

/// @brief Match two keyframes' points by word.
/// @param m The map, as a dict.
/// @param database Its database, as a dict.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param options The settings, as a dict.
/// @return For each keypoint of `k1`, its match.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::ndarray<nb::numpy, std::int64_t> match_by_word(const nb::dict& m, const nb::dict& database,
                                                   std::int64_t k1, std::int64_t k2,
                                                   const nb::dict& options) {
  return vector_array(
      unwrap(loops::match_by_word(edge_crossing::map_in(m), edge_crossing::database_in(database),
                                  k1, k2, edge_crossing::options_in(options))));
}

/// @brief The points a matched keyframe's group sees.
/// @param m The map, as a dict.
/// @param matched The matched keyframe's row.
/// @param linked The searching keyframe's links' rows.
/// @param options The settings, as a dict.
/// @return The points' rows.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> group_points(const nb::dict& m, std::int64_t matched,
                                                  const Int64In& linked, const nb::dict& options) {
  const std::vector<std::int64_t> rows = int64_in(linked);
  return vector_array(unwrap(loops::group_points(edge_crossing::map_in(m), matched,
                                                 std::set<std::int64_t>(rows.begin(), rows.end()),
                                                 edge_crossing::options_in(options))));
}

/// @brief Refine the similarity between two keyframes against their matches.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param similarity The starting `S_12`.
/// @param matched Each keypoint of `k1`'s match.
/// @param fixed_scale Refine a rigid motion.
/// @param options The settings, as a dict.
/// @return `similarity` and `inliers`, or None.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::object refine(const nb::dict& m, const nb::dict& camera, std::int64_t k1, std::int64_t k2,
                  const Matrix4In& similarity, const Int64In& matched, bool fixed_scale,
                  const nb::dict& options) {
  const std::optional<loops::Refined> found = unwrap(loops::refine(
      edge_crossing::map_in(m), edge_crossing::camera_in(camera), k1, k2, matrix4(similarity),
      int64_in(matched), fixed_scale, edge_crossing::options_in(options)));
  if (!found) {
    return nb::none();
  }
  nb::dict out;
  out["similarity"] = matrix_array(found->similarity);
  out["inliers"] = found->inliers;
  return out;
}

/// @brief Fuse points into a keyframe through a similarity.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param k The keyframe's row.
/// @param similarity Its `S_cw`.
/// @param point_ids The points' ids.
/// @param options The settings, as a dict.
/// @return `map` and `fused`.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict fuse_by_similarity(const nb::dict& m, const nb::dict& camera, std::int64_t k,
                            const Matrix4In& similarity, const Int64In& point_ids,
                            const nb::dict& options) {
  edge::Map map = edge_crossing::map_in(m);
  const std::int64_t fused = unwrap(
      loops::fuse_by_similarity(map, edge_crossing::camera_in(camera), k, matrix4(similarity),
                                int64_in(point_ids), edge_crossing::options_in(options)));
  nb::dict out;
  out["map"] = edge_crossing::map_dict(map);
  out["fused"] = fused;
  return out;
}

/// @brief Bundle-adjust every keyframe and point.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param options The settings, as a dict.
/// @return `map` and `summary`.
/// @throws CoreError for invalid input or GTSAM's failure.
nb::dict global_bundle_adjust(const nb::dict& m, const nb::dict& camera, const nb::dict& options) {
  edge::Map map = edge_crossing::map_in(m);
  const graphs::Summary summary = unwrap(loops::global_bundle_adjust(
      map, edge_crossing::camera_in(camera), edge_crossing::options_in(options)));
  nb::dict out;
  out["map"] = edge_crossing::map_dict(map);
  out["summary"] = solve_dict(summary);
  return out;
}

/// @brief Look for a loop at a newly mapped keyframe.
/// @param m The map, as a dict.
/// @param database Its database, as a dict.
/// @param camera The camera, as a dict.
/// @param keyframe The keyframe's id.
/// @param pending The loop being confirmed, as a dict, or None.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return `pending`, `loop` and `summary`.
/// @throws CoreError for invalid input.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict detect(const nb::dict& m, const nb::dict& database, const nb::dict& camera,
                std::int64_t keyframe, const nb::object& pending, const std::string& mode,
                const nb::dict& options) {
  std::optional<loops::Pending> carried;
  if (!pending.is_none()) {
    carried = pending_in(nb::cast<nb::dict>(pending));
  }
  const loops::Detection found =
      unwrap(loops::detect(edge_crossing::map_in(m), edge_crossing::database_in(database),
                           edge_crossing::camera_in(camera), keyframe, carried, mode,
                           edge_crossing::options_in(options)));
  nb::dict out;
  out["pending"] = found.pending ? nb::object(pending_dict(*found.pending)) : nb::none();
  out["loop"] = found.loop ? nb::object(loop_dict(*found.loop)) : nb::none();
  out["summary"] = detection_summary_dict(found.summary);
  return out;
}

/// @brief Correct a loop closed at a keyframe.
/// @param m The map, as a dict.
/// @param camera The camera, as a dict.
/// @param loop The loop, as a dict.
/// @param mode The mode's name.
/// @param options The settings, as a dict.
/// @return `map` and `summary`.
/// @throws CoreError for invalid input or GTSAM's failure.
nb::dict correct(const nb::dict& m, const nb::dict& camera, const nb::dict& loop,
                 const std::string& mode, const nb::dict& options) {
  edge::Map map = edge_crossing::map_in(m);
  const loops::CorrectionSummary s =
      unwrap(loops::correct(map, edge_crossing::camera_in(camera), loop_in(loop), mode,
                            edge_crossing::options_in(options)));
  nb::dict summary;
  summary["current"] = s.current;
  summary["matched"] = s.matched;
  summary["similarity"] = matrix_array(s.similarity);
  summary["corrected"] = s.corrected;
  summary["moved"] = s.moved;
  summary["replaced"] = s.replaced;
  summary["fused"] = s.fused;
  summary["links"] = s.links;
  summary["edges"] = s.edges;
  summary["essential"] = solve_dict(s.essential);
  summary["global_ba"] = s.global_ba ? nb::object(solve_dict(*s.global_ba)) : nb::none();
  nb::dict out;
  out["map"] = edge_crossing::map_dict(map);
  out["summary"] = summary;
  return out;
}

}  // namespace

void bind_loops(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("search_by_similarity", &search_by_similarity, nb::arg("m"), nb::arg("camera"),
        nb::arg("k"), nb::arg("similarity").noconvert(), nb::arg("points").noconvert(),
        nb::arg("window"), nb::arg("distance"), nb::arg("options"),
        "Match points to a keyframe's keypoints through a similarity.");
  m.def("match_by_word", &match_by_word, nb::arg("m"), nb::arg("database"), nb::arg("k1"),
        nb::arg("k2"), nb::arg("options"), "Match two keyframes' points by word.");
  m.def("group_points", &group_points, nb::arg("m"), nb::arg("matched"),
        nb::arg("linked").noconvert(), nb::arg("options"),
        "Return the points a matched keyframe's group sees.");
  m.def("refine", &refine, nb::arg("m"), nb::arg("camera"), nb::arg("k1"), nb::arg("k2"),
        nb::arg("similarity").noconvert(), nb::arg("matched").noconvert(), nb::arg("fixed_scale"),
        nb::arg("options"), "Refine the similarity between two keyframes against their matches.");
  m.def("fuse_by_similarity", &fuse_by_similarity, nb::arg("m"), nb::arg("camera"), nb::arg("k"),
        nb::arg("similarity").noconvert(), nb::arg("point_ids").noconvert(), nb::arg("options"),
        "Fuse points into a keyframe through a similarity.");
  m.def("global_bundle_adjust", &global_bundle_adjust, nb::arg("m"), nb::arg("camera"),
        nb::arg("options"), "Bundle-adjust every keyframe and point.");
  m.def("detect", &detect, nb::arg("m"), nb::arg("database"), nb::arg("camera"),
        nb::arg("keyframe"), nb::arg("pending").none(), nb::arg("mode"), nb::arg("options"),
        "Look for a loop at a newly mapped keyframe.");
  m.def("correct", &correct, nb::arg("m"), nb::arg("camera"), nb::arg("loop"), nb::arg("mode"),
        nb::arg("options"), "Correct a loop closed at a keyframe.");
}

}  // namespace slam_it::bindings
