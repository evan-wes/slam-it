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

/// @brief The packed scalar settings: `[min_parallax_deg, focal_std_ratio, loss_scale,
///        depth_loss_scale]`.
constexpr std::size_t kSettings = 4;
/// @brief The packed integer codes: `[model, method, loss, depth_loss, max_iterations, strict]`.
constexpr std::size_t kCodes = 6;
/// @brief The counts returned: termination, gauge, then the summary's counts.
constexpr std::size_t kCounts = 10;
/// @brief The columns of a returned trace row.
constexpr std::size_t kTraceColumns = 7;

/// @brief Build the options from the packed arguments.
/// @param gauge The gauge's code.
/// @param settings The packed scalar settings.
/// @param codes The packed integer codes.
/// @param tolerances The eight tolerances and limits, in `nls::Options` order.
/// @return The options.
/// @throws std::invalid_argument (Python `ValueError`) for a malformed pack or unknown code.
ba::Options options_from(int gauge, const VectorIn<double>& settings,
                         const VectorIn<std::int64_t>& codes, const VectorIn<double>& tolerances) {
  if (settings.shape(0) != kSettings || codes.shape(0) != kCodes ||
      tolerances.shape(0) != kTolerances) {
    throw std::invalid_argument("settings, codes, tolerances: wrong lengths");
  }
  const auto in_range = [](std::int64_t code, std::int64_t last) {
    return code >= 0 && code <= last;
  };
  if (!in_range(gauge, 3) || !in_range(codes(1), 1) || !in_range(codes(2), 2) ||
      !in_range(codes(3), 2)) {
    throw std::invalid_argument("gauge, method or loss: unknown code");
  }
  nls::Options solver{.method = static_cast<nls::Method>(codes(1)),
                      .loss = {.kind = static_cast<nls::LossKind>(codes(2)), .scale = settings(2)},
                      .block_loss = {},
                      .max_iterations = static_cast<int>(codes(4)),
                      .function_tolerance = tolerances(0),
                      .gradient_tolerance = tolerances(1),
                      .step_tolerance = tolerances(2),
                      .initial_damping = tolerances(3),
                      .max_damping = tolerances(4),
                      .min_diagonal = tolerances(5),
                      .max_diagonal = tolerances(6),
                      .min_relative_decrease = tolerances(7),
                      .strict = codes(5) != 0};
  return {.solver = std::move(solver),
          .depth_loss = {.kind = static_cast<nls::LossKind>(codes(3)), .scale = settings(3)},
          .gauge = static_cast<ba::Gauge>(gauge),
          .min_parallax_deg = settings(0),
          .focal_std_ratio = settings(1)};
}

/// @brief Copy a (C, 4, 4) batch into poses.
/// @param in The batch.
/// @return The poses.
ba::Poses copy_poses(const BatchIn<4, 4>& in) {
  check_aligned(in.data());
  ba::Poses out;
  for (std::size_t c = 0; c < in.shape(0); ++c) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    out.emplace_back(Eigen::Map<const Item<4, 4>>(in.data() + (16 * c)));
  }
  return out;
}

/// @brief Copy poses into a new (C, 4, 4) array.
/// @param poses The poses.
/// @return The array.
ArrayOut poses_array(const ba::Poses& poses) {
  auto [out, data] = new_array({poses.size(), 4, 4});
  for (std::size_t c = 0; c < poses.size(); ++c) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    Eigen::Map<Item<4, 4>>(data + (16 * c)) = poses[c];
  }
  return out;
}

/// @brief Copy a row-major matrix into a new array of its shape.
/// @tparam M The matrix type.
/// @param matrix The matrix.
/// @return The array.
template <typename M>
ArrayOut matrix_array(const M& matrix) {
  auto [out, data] =
      new_array({static_cast<std::size_t>(matrix.rows()), static_cast<std::size_t>(matrix.cols())});
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      data, matrix.rows(), matrix.cols()) = matrix;
  return out;
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

/// @brief Run bundle adjustment on a problem given as a struct of arrays.
/// @return `(poses, intrinsics, points, depth_affine, intrinsics_std, trace, counts, costs)`;
///         see the Python `Implementation` Protocol.
/// @throws CoreError for an invalid problem or (in strict mode) no convergence.
nb::tuple solve(const MatrixIn& intrinsics, const VectorIn<std::int64_t>& intrinsics_idx,
                const BatchIn<4, 4>& poses, const BatchIn<3>& points,
                const BatchIn<2>& observations, const VectorIn<std::int64_t>& cam_idx,
                const VectorIn<std::int64_t>& pt_idx, const VectorIn<double>& weights,
                const VectorIn<bool>& fixed_poses, const VectorIn<bool>& fixed_intrinsics,
                const VectorIn<bool>& fixed_points, const VectorIn<double>& depth_prior,
                const VectorIn<double>& depth_prior_weights, const BatchIn<2>& depth_affine,
                const VectorIn<bool>& fixed_depth_affine, const BatchIn<4, 4>& pose_prior,
                const BatchIn<2>& pose_prior_weights, int gauge, const VectorIn<double>& settings,
                const VectorIn<std::int64_t>& codes, const VectorIn<double>& tolerances) {
  const ba::Options options = options_from(gauge, settings, codes, tolerances);
  if (codes(0) < 0 || codes(0) > 1) {
    throw std::invalid_argument("model: unknown code");
  }
  using Pairs = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
  const ba::Problem problem{
      .model = static_cast<foundations::camera::Model>(codes(0)),
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
          copy_matrix<Pairs>(pose_prior_weights.data(), pose_prior_weights.shape(0), 2)};
  const ba::Solution solution = unwrap(ba::solve(problem, options));
  const ba::Summary& summary = solution.summary;
  const std::array<std::int64_t, kCounts> counts = {
      static_cast<std::int64_t>(summary.solve.termination),
      static_cast<std::int64_t>(summary.gauge),
      summary.gauge_pose,
      summary.scale_pose,
      summary.dropped_points,
      summary.dropped_observations,
      summary.unobserved_poses,
      summary.unobserved_intrinsics,
      summary.unobserved_depth_affines,
      summary.unobservable_focal};
  auto [counts_out, counts_data] = new_array<std::int64_t>({counts.size()});
  std::copy(counts.begin(), counts.end(), counts_data);
  auto [costs_out, costs_data] = new_array({2});
  costs_data[0] = summary.solve.initial_cost;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): second element
  costs_data[1] = summary.solve.final_cost;
  return nb::make_tuple(poses_array(solution.poses), matrix_array(solution.intrinsics),
                        matrix_array(solution.points), matrix_array(solution.depth_affine),
                        matrix_array(solution.intrinsics_std), trace_array(summary.solve.trace),
                        counts_out, costs_out);
}

}  // namespace

void bind_ba(nb::module_& m) {
  m.def("solve", &solve, nb::arg("intrinsics").noconvert(), nb::arg("intrinsics_idx").noconvert(),
        nb::arg("poses").noconvert(), nb::arg("points").noconvert(),
        nb::arg("observations").noconvert(), nb::arg("cam_idx").noconvert(),
        nb::arg("pt_idx").noconvert(), nb::arg("weights").noconvert(),
        nb::arg("fixed_poses").noconvert(), nb::arg("fixed_intrinsics").noconvert(),
        nb::arg("fixed_points").noconvert(), nb::arg("depth_prior").noconvert(),
        nb::arg("depth_prior_weights").noconvert(), nb::arg("depth_affine").noconvert(),
        nb::arg("fixed_depth_affine").noconvert(), nb::arg("pose_prior").noconvert(),
        nb::arg("pose_prior_weights").noconvert(), nb::arg("gauge"),
        nb::arg("settings").noconvert(), nb::arg("codes").noconvert(),
        nb::arg("tolerances").noconvert(),
        "Run bundle adjustment; return poses, intrinsics, points, depth affines, intrinsics "
        "deviations, trace, counts and costs.");
}

}  // namespace slamlab::bindings
