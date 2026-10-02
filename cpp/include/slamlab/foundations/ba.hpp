/// @file
/// @brief Bundle adjustment: the C++ core.
///
/// Mirrors `slamlab.foundations.ba`, whose docstring states the model:
/// - robust reprojection errors over poses (right-perturbed `T_cam_world`), intrinsics groups
///   and points;
/// - points eliminated by the Schur complement, with the reduced camera system factored by a
///   `ReducedSystemSolver`;
/// - the gauge removed by fixing a pose and, for the similarity gauge, confining one pose's
///   step to the Householder complement of its baseline direction;
/// - degenerate observations dropped and counted.
///
/// The solve is `nls::solve` over the same loop as every other problem.
#pragma once

#include <Eigen/Core>
#include <concepts>
#include <cstdint>
#include <optional>
#include <vector>

#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/nls.hpp"

/// @brief Bundle adjustment.
namespace slamlab::foundations::ba {

/// @brief A pose's tangent dimension.
inline constexpr int kPoseDim = 6;

/// @brief How the gauge freedom is removed, with the integer codes it crosses the bindings as.
enum class Gauge : std::uint8_t {
  /// @brief The caller's fixed variables define the gauge.
  kNone = 0,
  /// @brief SE(3): fix the first free observed pose if no pose is fixed.
  kFirstPose = 1,
  /// @brief Sim(3): as `kFirstPose`, then confine one pose's step to keep its baseline.
  kSimilarity = 2,
};

/// @brief Solves the reduced camera system `S x = b`, S symmetric positive definite.
template <typename S>
concept ReducedSystemSolver =
    requires(const S& solver, const Eigen::MatrixXd& matrix, const Eigen::VectorXd& rhs) {
      { solver.solve(matrix, rhs) } -> std::same_as<std::optional<Eigen::VectorXd>>;
    };

/// @brief The dense reduced solver: Cholesky on the full matrix.
struct DenseCholesky {
  /// @brief Solve `matrix x = rhs` by Cholesky.
  /// @param matrix Symmetric positive definite.
  /// @param rhs The right-hand side.
  /// @return The solution, or nothing if `matrix` is not positive definite.
  [[nodiscard]] static std::optional<Eigen::VectorXd> solve(const Eigen::MatrixXd& matrix,
                                                            const Eigen::VectorXd& rhs);
};

/// @brief Intrinsics groups, one row each (K x P).
using Intrinsics = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
/// @brief Points, one per row (N x 3).
using Points = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
/// @brief Pixels, one per row (M x 2).
using Pixels = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
/// @brief Poses as 4x4 homogeneous matrices.
using Poses = std::vector<Eigen::Matrix4d>;
/// @brief Index arrays.
using Indices = std::vector<std::int64_t>;
/// @brief Masks of fixed variables.
using Mask = std::vector<bool>;

/// @brief A bundle-adjustment problem, as a struct of arrays.
struct Problem {
  /// @brief The camera model shared by every intrinsics group.
  camera::Model model = camera::Model::kPinhole;
  /// @brief The intrinsics groups (K x P).
  Intrinsics intrinsics;
  /// @brief Camera-to-world poses `T_world_cam` (C).
  Poses poses;
  /// @brief World points (N x 3).
  Points points;
  /// @brief Observed pixels (M x 2).
  Pixels observations;
  /// @brief Each observation's camera (M).
  Indices cam_idx;
  /// @brief Each observation's point (M).
  Indices pt_idx;
  /// @brief Each camera's intrinsics group (C).
  Indices intrinsics_idx;
  /// @brief Each observation's weight, an information in px^-2 (M); 0 excludes it.
  Eigen::VectorXd weights;
  /// @brief Poses held fixed (C).
  Mask fixed_poses;
  /// @brief Intrinsics groups held fixed (K).
  Mask fixed_intrinsics;
  /// @brief Points held fixed (N).
  Mask fixed_points;
};

/// @brief Settings for `solve`.
struct Options {
  /// @brief The NLS method, loss, tolerances and limits.
  nls::Options solver;
  /// @brief How the gauge freedom is removed.
  Gauge gauge = Gauge::kSimilarity;
  /// @brief A free point whose rays span less than this angle (degrees) is dropped.
  double min_parallax_deg = 1.0;
};

/// @brief How a bundle adjustment went.
struct Summary {
  /// @brief The NLS solve's summary.
  nls::SolveSummary solve;
  /// @brief The pose fixed to remove the rigid gauge, or -1.
  std::int64_t gauge_pose = -1;
  /// @brief The pose whose step was confined to remove the scale gauge, or -1.
  std::int64_t scale_pose = -1;
  /// @brief Free points dropped as degenerate.
  std::int64_t dropped_points = 0;
  /// @brief Observations left out (of dropped points, or behind their camera).
  std::int64_t dropped_observations = 0;
  /// @brief Free poses held fixed because nothing observes them.
  std::int64_t unobserved_poses = 0;
  /// @brief Free intrinsics groups held fixed because nothing observes them.
  std::int64_t unobserved_intrinsics = 0;
};

/// @brief The refined variables and the summary.
struct Solution {
  /// @brief Refined camera-to-world poses.
  Poses poses;
  /// @brief Refined intrinsics groups.
  Intrinsics intrinsics;
  /// @brief Refined points (dropped points unchanged).
  Points points;
  /// @brief How the solve went.
  Summary summary;
};

/// @brief Check a problem's sizes, indices and values.
/// @param problem The problem.
/// @return Nothing, or `InvalidInput` naming the offending array.
[[nodiscard]] Result<void> check_problem(const Problem& problem);

/// @brief Refine a problem's poses, intrinsics and points.
/// @param problem The problem.
/// @param options Solver settings, gauge and degeneracy threshold.
/// @return The solution; `InvalidInput` for an invalid problem or options; `NotConverged` if
///         `options.solver.strict` and the solve stopped without converging.
[[nodiscard]] Result<Solution> solve(const Problem& problem, const Options& options);

}  // namespace slamlab::foundations::ba
