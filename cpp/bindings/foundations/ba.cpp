/// @file
/// @brief Bindings for `slamlab.foundations.ba` (`slamlab._cpp.foundations.ba`).
#include "slamlab/foundations/ba.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace ba = foundations::ba;
namespace nls = foundations::nls;

/// @brief A read-only, contiguous 1D array.
/// @tparam T The element type.
template <typename T>
using VectorIn = nb::ndarray<const T, nb::shape<-1>, nb::c_contig, nb::device::cpu>;

/// @brief A read-only, contiguous float64 matrix.
using MatrixIn = nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig, nb::device::cpu>;

/// @brief The number of tolerances passed as one array, in `Options` order.
constexpr std::size_t kTolerances = 8;

/// @brief Copy a 1D array into a vector.
/// @tparam Out The vector type.
/// @tparam T The element type.
/// @param in The array.
/// @return The copy.
template <typename Out, typename T>
Out copy_vector(const VectorIn<T>& in) {
  Out out(in.shape(0));
  for (std::size_t i = 0; i < in.shape(0); ++i) {
    out[i] = in(i);
  }
  return out;
}

/// @brief Copy a row-major float64 matrix of known column count into an Eigen matrix.
/// @tparam M The Eigen matrix type.
/// @param data The buffer.
/// @param rows Rows.
/// @param cols Columns.
/// @return The copy.
template <typename M>
M copy_matrix(const double* data, std::size_t rows, std::size_t cols) {
  check_aligned(data);
  return Eigen::Map<const M>(data, static_cast<Eigen::Index>(rows),
                             static_cast<Eigen::Index>(cols));
}

/// @brief Build the solver options from the scalar arguments.
/// @param method The method's code.
/// @param loss The loss's code.
/// @param loss_scale The loss scale.
/// @param max_iterations The iteration cap.
/// @param tolerances The eight tolerances and limits, in `nls::Options` order.
/// @param strict Strict mode.
/// @return The options.
nls::Options solver_options(int method, int loss, double loss_scale, int max_iterations,
                            const VectorIn<double>& tolerances, bool strict) {
  if (tolerances.shape(0) != kTolerances || method < 0 || method > 1 || loss < 0 || loss > 2) {
    throw std::invalid_argument("solver options: bad method, loss or tolerance count");
  }
  return {.method = static_cast<nls::Method>(method),
          .loss = {.kind = static_cast<nls::LossKind>(loss), .scale = loss_scale},
          .max_iterations = max_iterations,
          .function_tolerance = tolerances(0),
          .gradient_tolerance = tolerances(1),
          .step_tolerance = tolerances(2),
          .initial_damping = tolerances(3),
          .max_damping = tolerances(4),
          .min_diagonal = tolerances(5),
          .max_diagonal = tolerances(6),
          .min_relative_decrease = tolerances(7),
          .strict = strict};
}

/// @brief Run bundle adjustment on a problem given as a struct of arrays.
/// @return `(poses, intrinsics, points, trace, counts, costs)`; see the Python
///         `Implementation` Protocol.
/// @throws CoreError for an invalid problem or (in strict mode) no convergence.
nb::tuple solve(int model, const MatrixIn& intrinsics, const VectorIn<std::int64_t>& intrinsics_idx,
                const BatchIn<4, 4>& poses, const BatchIn<3>& points,
                const BatchIn<2>& observations, const VectorIn<std::int64_t>& cam_idx,
                const VectorIn<std::int64_t>& pt_idx, const VectorIn<double>& weights,
                const VectorIn<bool>& fixed_poses, const VectorIn<bool>& fixed_intrinsics,
                const VectorIn<bool>& fixed_points, int gauge, double min_parallax_deg, int method,
                int loss, double loss_scale, int max_iterations, const VectorIn<double>& tolerances,
                bool strict) {
  if (model < 0 || model > 1 || gauge < 0 || gauge > 2) {
    throw std::invalid_argument("model or gauge: unknown code");
  }
  ba::Problem problem{
      .model = static_cast<foundations::camera::Model>(model),
      .intrinsics =
          copy_matrix<ba::Intrinsics>(intrinsics.data(), intrinsics.shape(0), intrinsics.shape(1)),
      .poses = {},
      .points = copy_matrix<ba::Points>(points.data(), points.shape(0), 3),
      .observations = copy_matrix<ba::Pixels>(observations.data(), observations.shape(0), 2),
      .cam_idx = copy_vector<ba::Indices>(cam_idx),
      .pt_idx = copy_vector<ba::Indices>(pt_idx),
      .intrinsics_idx = copy_vector<ba::Indices>(intrinsics_idx),
      .weights = copy_vector<Eigen::VectorXd>(weights),
      .fixed_poses = copy_vector<ba::Mask>(fixed_poses),
      .fixed_intrinsics = copy_vector<ba::Mask>(fixed_intrinsics),
      .fixed_points = copy_vector<ba::Mask>(fixed_points)};
  check_aligned(poses.data());
  for (std::size_t c = 0; c < poses.shape(0); ++c) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    problem.poses.emplace_back(Eigen::Map<const Item<4, 4>>(poses.data() + (16 * c)));
  }
  const ba::Options options{
      .solver = solver_options(method, loss, loss_scale, max_iterations, tolerances, strict),
      .gauge = static_cast<ba::Gauge>(gauge),
      .min_parallax_deg = min_parallax_deg};
  const ba::Solution solution = unwrap(ba::solve(problem, options));

  const std::size_t n_cams = solution.poses.size();
  auto [poses_out, poses_data] = new_array({n_cams, 4, 4});
  for (std::size_t c = 0; c < n_cams; ++c) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    Eigen::Map<Item<4, 4>>(poses_data + (16 * c)) = solution.poses[c];
  }
  const auto rows = static_cast<std::size_t>(solution.intrinsics.rows());
  const auto cols = static_cast<std::size_t>(solution.intrinsics.cols());
  auto [intrinsics_out, intrinsics_data] = new_array({rows, cols});
  Eigen::Map<ba::Intrinsics>(intrinsics_data, solution.intrinsics.rows(),
                             solution.intrinsics.cols()) = solution.intrinsics;
  const auto n_points = static_cast<std::size_t>(solution.points.rows());
  auto [points_out, points_data] = new_array({n_points, 3});
  Eigen::Map<ba::Points>(points_data, solution.points.rows(), 3) = solution.points;

  const ba::Summary& summary = solution.summary;
  const std::vector<nls::Iteration>& trace = summary.solve.trace;
  auto [trace_out, trace_data] = new_array({trace.size(), 7});
  for (std::size_t i = 0; i < trace.size(); ++i) {
    const nls::Iteration& it = trace[i];
    const std::array<double, 7> row = {static_cast<double>(it.iteration),
                                       it.cost,
                                       it.gradient_norm,
                                       it.step_norm,
                                       it.damping,
                                       it.relative_decrease,
                                       it.accepted ? 1.0 : 0.0};
    for (std::size_t k = 0; k < row.size(); ++k) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
      trace_data[(7 * i) + k] = row.at(k);
    }
  }
  const std::array<std::int64_t, 7> counts = {static_cast<std::int64_t>(summary.solve.termination),
                                              summary.gauge_pose,
                                              summary.scale_pose,
                                              summary.dropped_points,
                                              summary.dropped_observations,
                                              summary.unobserved_poses,
                                              summary.unobserved_intrinsics};
  auto [counts_out, counts_data] = new_array<std::int64_t>({counts.size()});
  std::copy(counts.begin(), counts.end(), counts_data);
  auto [costs_out, costs_data] = new_array({2});
  costs_data[0] = summary.solve.initial_cost;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): second element
  costs_data[1] = summary.solve.final_cost;
  return nb::make_tuple(poses_out, intrinsics_out, points_out, trace_out, counts_out, costs_out);
}

}  // namespace

void bind_ba(nb::module_& m) {
  m.def("solve", &solve, nb::arg("model"), nb::arg("intrinsics").noconvert(),
        nb::arg("intrinsics_idx").noconvert(), nb::arg("poses").noconvert(),
        nb::arg("points").noconvert(), nb::arg("observations").noconvert(),
        nb::arg("cam_idx").noconvert(), nb::arg("pt_idx").noconvert(),
        nb::arg("weights").noconvert(), nb::arg("fixed_poses").noconvert(),
        nb::arg("fixed_intrinsics").noconvert(), nb::arg("fixed_points").noconvert(),
        nb::arg("gauge"), nb::arg("min_parallax_deg"), nb::arg("method"), nb::arg("loss"),
        nb::arg("loss_scale"), nb::arg("max_iterations"), nb::arg("tolerances").noconvert(),
        nb::arg("strict"),
        "Run bundle adjustment; return poses, intrinsics, points, trace, counts and costs.");
}

}  // namespace slamlab::bindings
