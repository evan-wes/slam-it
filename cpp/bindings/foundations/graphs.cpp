/// @file
/// @brief Bindings for `slam_it.foundations.graphs` (`slam_it._cpp.foundations.graphs`).
///
/// Each solve takes its arrays and options by keyword, as the Python `Implementation`
/// Protocol names them, and returns a dict: its arrays, and the summary's fields
/// (`initial_error`, `final_error`, `iterations`, `termination` as its code in `codes()`).
#include "slam_it/foundations/graphs.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace graphs = foundations::graphs;
namespace camera = foundations::camera;

/// @brief A read-only, contiguous (E, 2) int64 array.
using IndexPairsIn =
    nb::ndarray<const std::int64_t, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;

/// @brief `graphs::Options`' fields, as `graphs.Options.as_dict` names them.
constexpr std::array<const char*, 4> kOptionKeys = {"max_iterations", "initial_damping",
                                                    "relative_tolerance", "absolute_tolerance"};

/// @brief Read a solve's options from `graphs.Options.as_dict`.
/// @param options The dict.
/// @return The options (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key.
graphs::Options read_options(const nb::dict& options) {
  check_keys(options, kOptionKeys, "graphs.Options");
  return graphs::Options{.max_iterations = nb::cast<int>(options["max_iterations"]),
                         .initial_damping = nb::cast<double>(options["initial_damping"]),
                         .relative_tolerance = nb::cast<double>(options["relative_tolerance"]),
                         .absolute_tolerance = nb::cast<double>(options["absolute_tolerance"])};
}

/// @brief Write a summary's fields into a result dict.
/// @param summary The summary.
/// @param out The dict.
void write_summary(const graphs::Summary& summary, nb::dict& out) {
  out["initial_error"] = summary.initial_error;
  out["final_error"] = summary.final_error;
  out["iterations"] = summary.iterations;
  out["termination"] = static_cast<int>(summary.termination);
}

/// @brief A 1D int64 array viewed as a span.
/// @param in The array.
/// @return The span.
std::span<const std::int64_t> span_of(const VectorIn<std::int64_t>& in) {
  return {in.data(), in.shape(0)};
}

/// @brief A 1D float64 array viewed as a span.
/// @param in The array.
/// @return The span.
std::span<const double> span_of(const VectorIn<double>& in) { return {in.data(), in.shape(0)}; }

/// @brief The enums' name tables, by enum, in code order.
/// @return `{"graphs_termination": [...], "model": [...]}`.
nb::dict codes() {
  nb::dict out;
  out["graphs_termination"] =
      enum_names<graphs::Termination>([](graphs::Termination t) { return graphs::name(t); });
  out["model"] = enum_names<camera::Model>([](camera::Model m) { return camera::name(m); });
  return out;
}

/// @brief Optimise a pose graph (`graphs.optimize_pose_graph`).
/// @param poses (N, 4, 4) camera-to-world starts.
/// @param edges (E, 2) pose indices.
/// @param relatives (E, 4, 4) measured `T_i_j`.
/// @param fixed The poses held fixed.
/// @param similarity Optimise Sim(3) rather than SE(3).
/// @param options `graphs.Options.as_dict()`.
/// @return `{"poses", "initial_error", "final_error", "iterations", "termination"}`.
/// @throws CoreError for an invalid graph or a failed solve.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict optimize_pose_graph(const BatchIn<4, 4>& poses, const IndexPairsIn& edges,
                             const BatchIn<4, 4>& relatives, const VectorIn<std::int64_t>& fixed,
                             bool similarity, const nb::dict& options) {
  const std::vector<Eigen::Matrix4d> starts = copy_poses(poses);
  const std::vector<Eigen::Matrix4d> measured = copy_poses(relatives);
  const Eigen::Map<const graphs::IndexPairs> pairs(edges.data(),
                                                   static_cast<Eigen::Index>(edges.shape(0)), 2);
  const graphs::PoseGraph result = unwrap(graphs::optimize_pose_graph(
      starts, pairs, measured, span_of(fixed), similarity, read_options(options)));
  nb::dict out;
  out["poses"] = poses_array(result.poses);
  write_summary(result.summary, out);
  return out;
}

/// @brief Refine a similarity against its matches (`graphs.refine_similarity`).
/// @param similarity The starting `S_12`, (4, 4).
/// @param points1 (M, 3) points in camera 1's frame.
/// @param points2 (M, 3) points in camera 2's frame.
/// @param pixels1 (M, 2) where image 1 observes them.
/// @param pixels2 (M, 2) where image 2 observes them.
/// @param information (M, 2) informations, image 1 then image 2.
/// @param model The camera model's code.
/// @param intrinsics Its intrinsics.
/// @param fixed_scale Hold the scale at 1.
/// @param huber The Huber loss's threshold, or None for no robust loss.
/// @param options `graphs.Options.as_dict()`.
/// @return `{"similarity", "chi2", "initial_error", "final_error", "iterations",
///         "termination"}`.
/// @throws CoreError for invalid matches or a failed solve; std::invalid_argument for an
///         unknown model code.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict refine_similarity(const Matrix4In& similarity, const BatchIn<3>& points1,
                           const BatchIn<3>& points2, const BatchIn<2>& pixels1,
                           const BatchIn<2>& pixels2, const BatchIn<2>& information, int model,
                           const VectorIn<double>& intrinsics, bool fixed_scale,
                           std::optional<double> huber, const nb::dict& options) {
  const Eigen::VectorXd params = copy_vector<Eigen::VectorXd>(intrinsics);
  const graphs::SimilarityRefinement result = unwrap(graphs::refine_similarity(
      matrix4(similarity), rows3(points1), rows3(points2), rows2(pixels1), rows2(pixels2),
      rows2(information), enum_code<camera::Model>(model, "model"), params, fixed_scale, huber,
      read_options(options)));
  nb::dict out;
  out["similarity"] = matrix_array(result.similarity);
  out["chi2"] = matrix_array(result.chi2);
  write_summary(result.summary, out);
  return out;
}

/// @brief Adjust a bundle (`graphs.bundle_adjust`).
/// @param poses (C, 4, 4) camera-to-world starts.
/// @param points (P, 3) world points.
/// @param cameras Each observation's camera.
/// @param point_ids Each observation's point.
/// @param pixels (M, 2) observed pixels.
/// @param right Each observation's right column, NaN for none.
/// @param information Each observation's information.
/// @param model The camera model's code.
/// @param intrinsics Its intrinsics.
/// @param bf The baseline times the focal length.
/// @param fixed The cameras held fixed.
/// @param options `graphs.Options.as_dict()`.
/// @return `{"poses", "points", "initial_error", "final_error", "iterations", "termination"}`.
/// @throws CoreError for an invalid bundle or a failed solve; std::invalid_argument for an
///         unknown model code.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): every argument crosses by keyword
nb::dict bundle_adjust(const BatchIn<4, 4>& poses, const BatchIn<3>& points,
                       const VectorIn<std::int64_t>& cameras,
                       const VectorIn<std::int64_t>& point_ids, const BatchIn<2>& pixels,
                       const VectorIn<double>& right, const VectorIn<double>& information,
                       int model, const VectorIn<double>& intrinsics, double bf,
                       const VectorIn<std::int64_t>& fixed, const nb::dict& options) {
  const std::vector<Eigen::Matrix4d> starts = copy_poses(poses);
  const Eigen::VectorXd params = copy_vector<Eigen::VectorXd>(intrinsics);
  const auto seen = rows2(pixels);
  const graphs::Observations observations{.cameras = span_of(cameras),
                                          .point_ids = span_of(point_ids),
                                          .pixels = seen,
                                          .right = span_of(right),
                                          .information = span_of(information)};
  const graphs::Adjustment result = unwrap(graphs::bundle_adjust(
      starts, rows3(points), observations, enum_code<camera::Model>(model, "model"), params, bf,
      span_of(fixed), read_options(options)));
  nb::dict out;
  out["poses"] = poses_array(result.poses);
  out["points"] = matrix_array(result.points);
  write_summary(result.summary, out);
  return out;
}

}  // namespace

void bind_graphs(nb::module_& m) {
  m.def("codes", &codes, "The enums' name tables, by enum, in code order.");
  m.def("optimize_pose_graph", &optimize_pose_graph, nb::arg("poses").noconvert(),
        nb::arg("edges").noconvert(), nb::arg("relatives").noconvert(),
        nb::arg("fixed").noconvert(), nb::arg("similarity"), nb::arg("options"),
        "Optimise a pose graph; return its poses and the summary's fields.");
  m.def("refine_similarity", &refine_similarity, nb::arg("similarity").noconvert(),
        nb::arg("points1").noconvert(), nb::arg("points2").noconvert(),
        nb::arg("pixels1").noconvert(), nb::arg("pixels2").noconvert(),
        nb::arg("information").noconvert(), nb::arg("model"), nb::arg("intrinsics").noconvert(),
        nb::arg("fixed_scale"), nb::arg("huber").none(), nb::arg("options"),
        "Refine a similarity; return it, each match's chi2 and the summary's fields.");
  m.def("bundle_adjust", &bundle_adjust, nb::arg("poses").noconvert(),
        nb::arg("points").noconvert(), nb::arg("cameras").noconvert(),
        nb::arg("point_ids").noconvert(), nb::arg("pixels").noconvert(),
        nb::arg("right").noconvert(), nb::arg("information").noconvert(), nb::arg("model"),
        nb::arg("intrinsics").noconvert(), nb::arg("bf"), nb::arg("fixed").noconvert(),
        nb::arg("options"), "Adjust a bundle; return its poses, points and the summary's fields.");
}

}  // namespace slam_it::bindings
