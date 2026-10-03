/// @file
/// @brief Bindings for `slamlab.foundations.ba` (`slamlab._cpp.foundations.ba`).
///
/// Everything crosses by name: arrays and options are keyword arguments, enums are integer
/// codes whose names come from one table per enum here (exported by `codes()`, so the
/// Python side's tables are tested against them), and the result is a dict of named arrays
/// and scalars.
#include "slamlab/foundations/ba.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/nls.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace ba = foundations::ba;
namespace camera = foundations::camera;
namespace nls = foundations::nls;

/// @brief Camera model names, indexed by `camera::Model` code.
constexpr std::array<const char*, 3> kModels = {"pinhole", "radtan", "radial"};
/// @brief Gauge names, indexed by `ba::Gauge` code.
constexpr std::array<const char*, 6> kGauges = {"none", "first_pose", "similarity",
                                                "auto", "scale",      "partial"};
/// @brief Method names, indexed by `nls::Method` code.
constexpr std::array<const char*, 2> kMethods = {"levenberg_marquardt", "gauss_newton"};
/// @brief Loss names, indexed by `nls::LossKind` code.
constexpr std::array<const char*, 3> kLosses = {"trivial", "huber", "cauchy"};
/// @brief The number of `nls::Termination` codes (their names come from `nls::name`).
constexpr auto kTerminations = static_cast<int>(nls::Termination::kCount);

static_assert(static_cast<std::size_t>(camera::Model::kCount) == kModels.size());
static_assert(static_cast<std::size_t>(ba::Gauge::kCount) == kGauges.size());
static_assert(static_cast<std::size_t>(nls::Method::kCount) == kMethods.size());
static_assert(static_cast<std::size_t>(nls::LossKind::kCount) == kLosses.size());

/// @brief The columns of a returned trace row.
constexpr std::size_t kTraceColumns = 7;
/// @brief The trace's column names (`nls::Iteration`'s fields), in `trace_array` order; the
///        Python side reads the columns by these names.
constexpr std::array<const char*, kTraceColumns> kTraceNames = {
    "iteration", "cost", "gradient_norm", "step_norm", "damping", "relative_decrease", "accepted"};

/// @brief Decode an enum's integer code, checking it against the enum's name table.
/// @tparam E The enum.
/// @tparam N The table's size.
/// @param code The code.
/// @param names The enum's names.
/// @param what The argument's name, for the message.
/// @return The enum value.
/// @throws std::invalid_argument (Python `ValueError`) for an unknown code.
template <typename E, std::size_t N>
E decode(int code, const std::array<const char*, N>& names, const char* what) {
  if (code < 0 || std::cmp_greater_equal(code, names.size())) {
    throw std::invalid_argument(std::string(what) + ": unknown code " + std::to_string(code));
  }
  return static_cast<E>(code);
}

/// @brief The summary's trace as a new (I, 7) array.
/// @param trace The trace.
/// @return The array.
ArrayOut trace_array(const std::vector<nls::Iteration>& trace) {
  auto [out, data] = new_array({trace.size(), kTraceColumns});
  for (std::size_t i = 0; i < trace.size(); ++i) {
    const nls::Iteration& it = trace[i];
    const std::array<double, kTraceColumns> row = {static_cast<double>(it.iteration),
                                                   it.cost,
                                                   it.gradient_norm,
                                                   it.step_norm,
                                                   it.damping,
                                                   it.relative_decrease,
                                                   it.accepted ? 1.0 : 0.0};
    for (std::size_t k = 0; k < row.size(); ++k) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
      data[(kTraceColumns * i) + k] = row.at(k);
    }
  }
  return out;
}

/// @brief The enums' name tables, by enum, in code order.
/// @return `{"model": [...], "gauge": [...], "method": [...], "loss": [...],
///         "termination": [...]}`.
nb::dict codes() {
  const auto as_list = [](const auto& names) {
    nb::list out;
    for (const char* name : names) {
      out.append(name);
    }
    return out;
  };
  nb::list terminations;
  for (int k = 0; k < kTerminations; ++k) {
    terminations.append(nls::name(static_cast<nls::Termination>(k)));
  }
  nb::dict out;
  out["model"] = as_list(kModels);
  out["gauge"] = as_list(kGauges);
  out["method"] = as_list(kMethods);
  out["loss"] = as_list(kLosses);
  out["termination"] = terminations;
  return out;
}

/// @brief Run bundle adjustment on a problem given by named arrays and options.
/// @return A dict of named results; see the Python `Implementation` Protocol.
/// @throws CoreError for an invalid problem or (in strict mode) no convergence;
///         std::invalid_argument for an unknown code.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters,readability-function-size): every
// argument crosses as a keyword, by name, so positions cannot be swapped by a caller.
nb::dict solve(const MatrixIn& intrinsics, const VectorIn<std::int64_t>& intrinsics_idx,
               const BatchIn<4, 4>& poses, const BatchIn<3>& points, const BatchIn<2>& observations,
               const VectorIn<std::int64_t>& cam_idx, const VectorIn<std::int64_t>& pt_idx,
               const VectorIn<double>& weights, const VectorIn<bool>& fixed_poses,
               const VectorIn<bool>& fixed_intrinsics, const VectorIn<bool>& fixed_points,
               const VectorIn<double>& depth_prior, const VectorIn<double>& depth_prior_weights,
               const BatchIn<2>& depth_affine, const VectorIn<bool>& fixed_depth_affine,
               const BatchIn<4, 4>& pose_prior, const BatchIn<2>& pose_prior_weights,
               const VectorIn<std::int64_t>& point_anchor, int model, int gauge, int method,
               int loss, double loss_scale, int depth_loss, double depth_loss_scale,
               int max_iterations, double function_tolerance, double gradient_tolerance,
               double step_tolerance, double initial_damping, double max_damping,
               double min_diagonal, double max_diagonal, double min_relative_decrease, bool strict,
               double min_parallax_deg, double focal_std_ratio, bool scale_deviations) {
  using Pairs = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
  const ba::Options options{
      .solver = {.method = decode<nls::Method>(method, kMethods, "method"),
                 .loss = {.kind = decode<nls::LossKind>(loss, kLosses, "loss"),
                          .scale = loss_scale},
                 .block_loss = {},
                 .max_iterations = max_iterations,
                 .function_tolerance = function_tolerance,
                 .gradient_tolerance = gradient_tolerance,
                 .step_tolerance = step_tolerance,
                 .initial_damping = initial_damping,
                 .max_damping = max_damping,
                 .min_diagonal = min_diagonal,
                 .max_diagonal = max_diagonal,
                 .min_relative_decrease = min_relative_decrease,
                 .strict = strict},
      .depth_loss = {.kind = decode<nls::LossKind>(depth_loss, kLosses, "depth_loss"),
                     .scale = depth_loss_scale},
      .gauge = decode<ba::Gauge>(gauge, kGauges, "gauge"),
      .min_parallax_deg = min_parallax_deg,
      .focal_std_ratio = focal_std_ratio,
      .scale_deviations = scale_deviations};
  const ba::Problem problem{
      .model = decode<camera::Model>(model, kModels, "model"),
      .intrinsics =
          copy_matrix<ba::Intrinsics>(intrinsics.data(), intrinsics.shape(0), intrinsics.shape(1)),
      .poses = copy_poses(poses),
      .points = copy_matrix<ba::Points>(points.data(), points.shape(0), 3),
      .observations = copy_matrix<ba::Pixels>(observations.data(), observations.shape(0), 2),
      .cam_idx = copy_vector<ba::Indices>(cam_idx),
      .pt_idx = copy_vector<ba::Indices>(pt_idx),
      .intrinsics_idx = copy_vector<ba::Indices>(intrinsics_idx),
      .weights = copy_vector<Eigen::VectorXd>(weights),
      .fixed_poses = copy_vector<ba::Mask>(fixed_poses),
      .fixed_intrinsics = copy_vector<ba::Mask>(fixed_intrinsics),
      .fixed_points = copy_vector<ba::Mask>(fixed_points),
      .depth_prior = copy_vector<Eigen::VectorXd>(depth_prior),
      .depth_prior_weights = copy_vector<Eigen::VectorXd>(depth_prior_weights),
      .depth_affine = copy_matrix<Pairs>(depth_affine.data(), depth_affine.shape(0), 2),
      .fixed_depth_affine = copy_vector<ba::Mask>(fixed_depth_affine),
      .pose_prior = copy_poses(pose_prior),
      .pose_prior_weights =
          copy_matrix<Pairs>(pose_prior_weights.data(), pose_prior_weights.shape(0), 2),
      .point_anchor = copy_vector<ba::Indices>(point_anchor)};
  const ba::Solution solution = unwrap(ba::solve(problem, options));
  const ba::Summary& summary = solution.summary;
  nb::dict out;
  out["poses"] = poses_array(solution.poses);
  out["intrinsics"] = matrix_array(solution.intrinsics);
  out["points"] = matrix_array(solution.points);
  out["depth_affine"] = matrix_array(solution.depth_affine);
  out["inverse_depth"] = vector_array(solution.inverse_depth);
  out["intrinsics_std"] = matrix_array(solution.intrinsics_std);
  out["trace"] = trace_array(summary.solve.trace);
  nb::list columns;
  for (const char* name : kTraceNames) {
    columns.append(name);
  }
  out["trace_columns"] = columns;
  out["initial_cost"] = summary.solve.initial_cost;
  out["final_cost"] = summary.solve.final_cost;
  out["termination"] = static_cast<int>(summary.solve.termination);
  out["gauge"] = static_cast<int>(summary.gauge);
  out["gauge_pose"] = summary.gauge_pose;
  out["confined_pose"] = summary.confined_pose;
  out["free_gauge"] = summary.free_gauge;
  out["dropped_points"] = summary.dropped_points;
  out["dropped_observations"] = summary.dropped_observations;
  out["unobserved_poses"] = summary.unobserved_poses;
  out["unobserved_intrinsics"] = summary.unobserved_intrinsics;
  out["unobserved_depth_affines"] = summary.unobserved_depth_affines;
  out["unobservable_focal"] = summary.unobservable_focal;
  out["variance_factor"] = summary.variance_factor;
  return out;
}

}  // namespace

void bind_ba(nb::module_& m) {
  m.def("codes", &codes, "The enums' name tables, by enum, in code order.");
  m.def("solve", &solve, nb::kw_only(), nb::arg("intrinsics").noconvert(),
        nb::arg("intrinsics_idx").noconvert(), nb::arg("poses").noconvert(),
        nb::arg("points").noconvert(), nb::arg("observations").noconvert(),
        nb::arg("cam_idx").noconvert(), nb::arg("pt_idx").noconvert(),
        nb::arg("weights").noconvert(), nb::arg("fixed_poses").noconvert(),
        nb::arg("fixed_intrinsics").noconvert(), nb::arg("fixed_points").noconvert(),
        nb::arg("depth_prior").noconvert(), nb::arg("depth_prior_weights").noconvert(),
        nb::arg("depth_affine").noconvert(), nb::arg("fixed_depth_affine").noconvert(),
        nb::arg("pose_prior").noconvert(), nb::arg("pose_prior_weights").noconvert(),
        nb::arg("point_anchor").noconvert(), nb::arg("model"), nb::arg("gauge"), nb::arg("method"),
        nb::arg("loss"), nb::arg("loss_scale"), nb::arg("depth_loss"), nb::arg("depth_loss_scale"),
        nb::arg("max_iterations"), nb::arg("function_tolerance"), nb::arg("gradient_tolerance"),
        nb::arg("step_tolerance"), nb::arg("initial_damping"), nb::arg("max_damping"),
        nb::arg("min_diagonal"), nb::arg("max_diagonal"), nb::arg("min_relative_decrease"),
        nb::arg("strict"), nb::arg("min_parallax_deg"), nb::arg("focal_std_ratio"),
        nb::arg("scale_deviations"),
        "Run bundle adjustment; every argument by keyword; returns a dict of named results.");
}

}  // namespace slamlab::bindings
