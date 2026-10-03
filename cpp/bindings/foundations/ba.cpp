/// @file
/// @brief Bindings for `slamlab.foundations.ba` (`slamlab._cpp.foundations.ba`).
///
/// Everything crosses by name: the problem's arrays are keyword arguments, and the options
/// one dict of `BAOptions`' fields, nested where `BAOptions` nests (`solver`, the losses),
/// with each choice by its name. The camera model, a standalone argument, crosses as a code;
/// so do the enums in the result. Every enum's names come from one table here (exported by
/// `codes()`, so the Python side's tables are tested against them), and the result is a
/// dict of named arrays and scalars.
#include "slamlab/foundations/ba.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/nls_crossing.hpp"
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
static_assert(static_cast<std::size_t>(camera::Model::kCount) == kModels.size());
static_assert(static_cast<std::size_t>(ba::Gauge::kCount) == kGauges.size());
static_assert(static_cast<std::size_t>(nls::Method::kCount) == kMethods.size());
static_assert(static_cast<std::size_t>(nls::LossKind::kCount) == kLosses.size());

/// @brief `ba.BAOptions`' fields, as the options dict's keys.
constexpr std::array<const char*, 6> kOptionKeys = {
    "solver", "depth_loss", "gauge", "min_parallax_deg", "focal_std_ratio", "scale_deviations"};
/// @brief `nls.Options`' fields, as the `solver` dict's keys.
constexpr std::array<const char*, 13> kSolverKeys = {"method",
                                                     "loss",
                                                     "block_loss",
                                                     "max_iterations",
                                                     "function_tolerance",
                                                     "gradient_tolerance",
                                                     "step_tolerance",
                                                     "initial_damping",
                                                     "max_damping",
                                                     "min_diagonal",
                                                     "max_diagonal",
                                                     "min_relative_decrease",
                                                     "strict"};
/// @brief `nls.Loss`' fields, as a loss dict's keys.
constexpr std::array<const char*, 2> kLossKeys = {"kind", "scale"};

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

/// @brief Look an enum up by its name in the enum's name table.
/// @tparam E The enum.
/// @tparam N The table's size.
/// @param value The name, a Python `str`.
/// @param names The enum's names.
/// @param what The field's name, for the message.
/// @return The enum value.
/// @throws std::invalid_argument (Python `ValueError`) for an unknown name.
template <typename E, std::size_t N>
E by_name(const nb::handle& value, const std::array<const char*, N>& names, const char* what) {
  const std::string name = nb::cast<std::string>(value);
  const auto* found = std::ranges::find(names, name);
  if (found == names.end()) {
    throw std::invalid_argument(std::string(what) + ": unknown name " + name);
  }
  return static_cast<E>(std::distance(names.begin(), found));
}

/// @brief Read a robust loss from a dict of `nls.Loss`' fields.
/// @param loss The dict.
/// @param what The field's name, for the messages.
/// @return The loss (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key, or an unknown kind.
nls::Loss read_loss(const nb::dict& loss, const char* what) {
  check_keys(loss, kLossKeys, "nls.Loss");
  return {.kind = by_name<nls::LossKind>(loss["kind"], kLosses, what),
          .scale = nb::cast<double>(loss["scale"])};
}

/// @brief Read bundle adjustment's options from a dict of `BAOptions`' fields.
/// @param options The dict.
/// @return The options (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key, an unknown name, or a
///         `block_loss` set by the caller (bundle adjustment assigns it).
ba::Options read_options(const nb::dict& options) {
  check_keys(options, kOptionKeys, "ba.BAOptions");
  const auto solver = nb::cast<nb::dict>(options["solver"]);
  check_keys(solver, kSolverKeys, "nls.Options");
  if (!solver["block_loss"].is_none()) {
    throw std::invalid_argument("solver.block_loss: set by bundle adjustment, not the caller");
  }
  return ba::Options{
      .solver = {.method = by_name<nls::Method>(solver["method"], kMethods, "method"),
                 .loss = read_loss(nb::cast<nb::dict>(solver["loss"]), "loss"),
                 .block_loss = {},
                 .max_iterations = nb::cast<int>(solver["max_iterations"]),
                 .function_tolerance = nb::cast<double>(solver["function_tolerance"]),
                 .gradient_tolerance = nb::cast<double>(solver["gradient_tolerance"]),
                 .step_tolerance = nb::cast<double>(solver["step_tolerance"]),
                 .initial_damping = nb::cast<double>(solver["initial_damping"]),
                 .max_damping = nb::cast<double>(solver["max_damping"]),
                 .min_diagonal = nb::cast<double>(solver["min_diagonal"]),
                 .max_diagonal = nb::cast<double>(solver["max_diagonal"]),
                 .min_relative_decrease = nb::cast<double>(solver["min_relative_decrease"]),
                 .strict = nb::cast<bool>(solver["strict"])},
      .depth_loss = read_loss(nb::cast<nb::dict>(options["depth_loss"]), "depth_loss"),
      .gauge = by_name<ba::Gauge>(options["gauge"], kGauges, "gauge"),
      .min_parallax_deg = nb::cast<double>(options["min_parallax_deg"]),
      .focal_std_ratio = nb::cast<double>(options["focal_std_ratio"]),
      .scale_deviations = nb::cast<bool>(options["scale_deviations"])};
}

/// @brief The enums' name tables, by enum, in code order.
/// @return `{"model": [...], "gauge": [...], "method": [...], "loss": [...],
///         "nls_termination": [...]}`.
nb::dict codes() {
  const auto as_list = [](const auto& names) {
    nb::list out;
    for (const char* name : names) {
      out.append(name);
    }
    return out;
  };
  nb::dict out;
  out["model"] = as_list(kModels);
  out["gauge"] = as_list(kGauges);
  out["method"] = as_list(kMethods);
  out["loss"] = as_list(kLosses);
  out["nls_termination"] = nls_terminations();
  return out;
}

/// @brief Run bundle adjustment on a problem given by named arrays, and its options.
/// @return A dict of named results; see the Python `Implementation` Protocol.
/// @throws CoreError for an invalid problem or (in strict mode) no convergence;
///         std::invalid_argument for an unknown code or name, or a malformed options dict.
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
               const VectorIn<std::int64_t>& point_anchor, int model, const nb::dict& options) {
  using Pairs = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
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
  const ba::Solution solution = unwrap(ba::solve(problem, read_options(options)));
  const ba::Summary& summary = solution.summary;
  nb::dict out;
  out["poses"] = poses_array(solution.poses);
  out["intrinsics"] = matrix_array(solution.intrinsics);
  out["points"] = matrix_array(solution.points);
  out["depth_affine"] = matrix_array(solution.depth_affine);
  out["inverse_depth"] = vector_array(solution.inverse_depth);
  out["intrinsics_std"] = matrix_array(solution.intrinsics_std);
  out["solve"] = nls_summary(summary.solve);
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
        nb::arg("point_anchor").noconvert(), nb::arg("model"), nb::arg("options"),
        "Run bundle adjustment; every argument by keyword; returns a dict of named results.");
}

}  // namespace slamlab::bindings
