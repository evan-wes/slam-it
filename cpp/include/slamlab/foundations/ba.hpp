/// @file
/// @brief Bundle adjustment: the C++ core.
///
/// Mirrors `slamlab.foundations.ba`, whose docstring states the model:
/// - robust reprojection errors over poses (right-perturbed `T_cam_world`), intrinsics groups
///   and points (plain, or anchored inverse depths), plus inverse-depth priors through
///   per-camera affines and pose priors, each kind of block with its own loss;
/// - points eliminated by the Schur complement, with the reduced camera system factored by a
///   `ReducedSystemSolver`;
/// - the gauge removed by fixing a pose and, for the similarity gauge, confining one pose's
///   step to the Householder complement of its baseline direction;
/// - degenerate observations dropped and counted;
/// - free intrinsics reported with their marginal standard deviations, and focal lengths the
///   data does not pin counted.
///
/// The solve is `nls::solve` over the same loop as every other problem.
#pragma once

#include <Eigen/Core>
#include <concepts>
#include <cstdint>
#include <limits>
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
  /// @brief Exactly what the fixed variables and priors leave free, whatever part of the
  ///        similarity it is (reported as `kNone`, `kFirstPose`, `kScale`, `kSimilarity` or
  ///        `kPartial`).
  kAuto = 3,
  /// @brief Scale only: confine one pose's step to keep its baseline to the pose the rigid
  ///        motion is pinned to (a fixed pose or a full pose prior).
  kScale = 4,
  /// @brief Reported only: `kAuto` found a part of the similarity no other name describes.
  kPartial = 5,
  /// @brief The number of gauges (a sentinel, not a gauge): the bindings' name tables are
  ///        checked against it, so an enumerator added without its name fails to build.
  kCount = 6,
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
  /// @brief Intrinsics held fixed, per group and parameter (K x P, row-major).
  Mask fixed_intrinsics;
  /// @brief Points held fixed (N).
  Mask fixed_points;
  /// @brief Each observation's prior inverse depth (M); NaN for none.
  Eigen::VectorXd depth_prior;
  /// @brief Each depth prior's weight, an information in inverse-depth units (M).
  Eigen::VectorXd depth_prior_weights;
  /// @brief Each camera's prior-to-model map `(scale, shift)` (C x 2).
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> depth_affine;
  /// @brief Affines held fixed (C); a metric prior's stays at `(1, 0)`.
  Mask fixed_depth_affine;
  /// @brief Each camera's prior camera-to-world pose (C), used where its weights are positive.
  Poses pose_prior;
  /// @brief Each pose prior's `(translation, rotation)` information (C x 2).
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> pose_prior_weights;
  /// @brief Each point's anchor observation (N), or -1: an anchored point is one inverse
  ///        depth along its anchor pixel's ray, in the anchor's camera.
  Indices point_anchor;
};

/// @brief Settings for `solve`.
struct Options {
  /// @brief The NLS method, tolerances and limits; its `loss` is the reprojection loss and
  ///        its `block_loss` must be empty (bundle adjustment assigns it).
  nls::Options solver;
  /// @brief The depth priors' robust loss.
  nls::Loss depth_loss;
  /// @brief How the gauge freedom is removed.
  Gauge gauge = Gauge::kAuto;
  /// @brief A free point whose rays span less than this angle (degrees) is dropped (default
  ///        0: only points seen fewer than twice are).
  double min_parallax_deg = 0.0;
  /// @brief A free focal length whose deviation exceeds this fraction of it is unobservable.
  double focal_std_ratio = 0.01;
  /// @brief Scale the intrinsics' deviations by the a-posteriori variance factor (see
  ///        `Summary::variance_factor`); off, the weights are taken as exact informations.
  bool scale_deviations = true;
};

/// @brief How a bundle adjustment went.
struct Summary {
  /// @brief The NLS solve's summary.
  nls::SolveSummary solve;
  /// @brief The gauge applied (`kAuto` resolved).
  Gauge gauge = Gauge::kAuto;
  /// @brief The pose fixed to remove the rigid gauge, or -1.
  std::int64_t gauge_pose = -1;
  /// @brief The first pose whose step was confined to remove the rest of the gauge, or -1.
  std::int64_t confined_pose = -1;
  /// @brief Free points dropped as degenerate.
  std::int64_t dropped_points = 0;
  /// @brief Observations left out (of dropped points, or behind their camera).
  std::int64_t dropped_observations = 0;
  /// @brief Free poses held fixed because nothing observes them.
  std::int64_t unobserved_poses = 0;
  /// @brief Free intrinsics groups held fixed because nothing observes them.
  std::int64_t unobserved_intrinsics = 0;
  /// @brief Free depth affines held fixed because no depth prior observes them.
  std::int64_t unobserved_depth_affines = 0;
  /// @brief Free intrinsics groups whose focal length the data does not pin.
  std::int64_t unobservable_focal = 0;
  /// @brief The a-posteriori variance factor `sum(w s) / (m - n)` at the solution: the
  ///        blocks' squared norms under their final IRLS weights (`2 cost` under the trivial
  ///        loss), over m weighted residual rows less n free parameters (NaN when m <= n).
  double variance_factor = std::numeric_limits<double>::quiet_NaN();
};

/// @brief The refined variables and the summary.
struct Solution {
  /// @brief Refined camera-to-world poses.
  Poses poses;
  /// @brief Refined intrinsics groups.
  Intrinsics intrinsics;
  /// @brief Refined points (dropped points unchanged).
  Points points;
  /// @brief Anchored points' refined inverse depths (N); NaN for the others.
  Eigen::VectorXd inverse_depth;
  /// @brief Refined depth affines (C x 2).
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> depth_affine;
  /// @brief Free intrinsics' standard deviations (K x P); NaN where fixed, inf if singular.
  Intrinsics intrinsics_std;
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
