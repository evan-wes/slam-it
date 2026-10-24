/// @file
/// @brief Factor graphs on GTSAM: a pose graph, a similarity refined against matches, and BA.
///
/// Mirrors `slam_it.foundations.graphs`, whose module docstring states the conventions:
/// poses cross camera-to-world (`T_world_cam`), a similarity as `[[s R, t], [0, 1]]`
/// mapping `p` to `s R p + t` (GTSAM's `Similarity3` maps it to `s (R p + t')`, `t' = t / s`);
/// Jacobians are on GTSAM's right-perturbed tangents, rotation first: `(w, v)` for a pose,
/// `(w, v, lambda)` for a similarity. GTSAM stays inside the implementation: this header
/// needs only Eigen.
#pragma once

#include <Eigen/Core>
#include <atomic>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/error.hpp"

/// @brief Factor graphs on GTSAM.
namespace slam_it::foundations::graphs {

/// @brief A pose's tangent dimension, `(w, v)`.
inline constexpr int kPoseDim = 6;
/// @brief A similarity's tangent dimension, `(w, v, lambda)`.
inline constexpr int kSimilarityDim = 7;

/// @brief GTSAM's own damping's factor, its `lambdaFactor` default
///        (`graphs.ABSOLUTE_DAMPING_FACTOR`): the damping divided by it after a step that lowers
///        the error, multiplied by it after one that does not.
inline constexpr double kAbsoluteDampingFactor = 10.0;
/// @brief GTSAM's own damping's upper bound, its `lambdaUpperBound` default
///        (`graphs.ABSOLUTE_DAMPING_LIMIT`): a search that reaches it gives up.
inline constexpr double kAbsoluteDampingLimit = 1e5;
/// @brief Nielsen's first factor for the scaled damping (`graphs.SCALED_DAMPING_FACTOR`): a failed
///        trial multiplies the damping by 2, 4, 8, ..., as g2o's `_ni`; an accepted step by
///        `max(1/3, 1 - (2 rho - 1)^3)` (GTSAM's `useFixedLambdaFactor = false`, without g2o's cap
///        at 2/3).
inline constexpr double kScaledDampingFactor = 2.0;
/// @brief The scaled damping's upper bound, GTSAM's own for its Ceres-like parameters
///        (`graphs.SCALED_DAMPING_LIMIT`): in practice none, as g2o has none.
inline constexpr double kScaledDampingLimit = 1e32;

/// @brief Why a solve stopped, with the codes it crosses the bindings as.
enum class Termination : std::uint8_t {
  /// @brief Converged: an iteration lowered the error by no more than the tolerances, or left
  ///        it at zero.
  kFunction = 0,
  /// @brief Not converged: `max_iterations` iterations ran.
  kMaxIterations = 1,
  /// @brief Not converged: no step lowered the error before the damping reached its upper
  ///        bound (also how a solve at its minimum ends when both tolerances are 0).
  kDampingLimit = 2,
  /// @brief Not converged: stopped from outside, between iterations (`Options::stop`).
  kStopped = 3,
  /// @brief The number of terminations (a sentinel, not a termination).
  kCount = 4,
};

/// @brief The termination's name, as `slam_it.foundations.graphs.Termination` spells it.
/// @param termination The termination.
/// @return A static string.
[[nodiscard]] const char* name(Termination termination);

/// @brief Whether a termination counts as converged.
/// @param termination The termination.
/// @return True for `kFunction`.
[[nodiscard]] bool is_converged(Termination termination);

/// @brief How a solve runs (`slam_it.foundations.graphs.Options`).
struct Options {
  /// @brief Iterations at most, at least 1: each linearises once and tries dampings until a
  ///        step lowers the error.
  int max_iterations = 10;
  /// @brief Levenberg-Marquardt's first damping, added to the normal equations' diagonal
  ///        (GTSAM's `lambdaInitial`), positive.
  double initial_damping = 1e-5;
  /// @brief An iteration lowering the error by at most this fraction of it ends the solve as
  ///        converged; non-negative.
  double relative_tolerance = 1e-10;
  /// @brief An iteration lowering the error by at most this ends the solve as converged;
  ///        non-negative.
  double absolute_tolerance = 0.0;
  /// @brief Follow g2o's damping (`OptimizationAlgorithmLevenberg`, which ORB-SLAM3's BAs run)
  ///        rather than GTSAM's own: the first damping `initial_damping` times the largest
  ///        diagonal entry of the normal equations at the start (g2o's `tau`), then Nielsen's
  ///        schedule (`kScaledDampingFactor`) up to `kScaledDampingLimit`; GTSAM's own starts at
  ///        `initial_damping`, divides it by 10 after a step that lowers the error and multiplies
  ///        it by 10 after one that does not, up to `kAbsoluteDampingLimit`.
  bool scaled_damping = false;
  /// @brief End the solve as converged once this many iterations in a row have each lowered the
  ///        error by less than `small_decrease` of it (ORB-SLAM3's g2o: 3); 0, never.
  int small_steps = 0;
  /// @brief See `small_steps` (ORB-SLAM3's 1e-3); positive.
  double small_decrease = 1e-3;
  /// @brief A flag another thread may set to stop the solve before its next iteration (a
  ///        global BA superseded, ORB-SLAM's `mbStopGBA`); not owned, null for never.
  const std::atomic<bool>* stop = nullptr;
};

/// @brief Check a solve's options.
/// @param options The options.
/// @return Nothing, or `InvalidInput` naming the option out of its range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief How a solve went. Satisfies `solve::Summary`.
struct Summary {
  /// @brief GTSAM's error, `1/2 sum rho(|r|^2_Sigma)`, at the start.
  double initial_error = 0.0;
  /// @brief GTSAM's error at the returned values.
  double final_error = 0.0;
  /// @brief The iterations run.
  int iterations = 0;
  /// @brief Why the solve stopped.
  Termination termination = Termination::kMaxIterations;
  /// @brief Whether `termination` counts as converged.
  bool converged = false;
};

/// @brief A row-major (N, 3) matrix of points.
using Points = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
/// @brief A row-major (N, 2) matrix: pixels, or two values per row.
using Pairs = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
/// @brief A row-major (E, 2) matrix of indices.
using IndexPairs = Eigen::Matrix<std::int64_t, Eigen::Dynamic, 2, Eigen::RowMajor>;

/// @brief A pose graph's solution.
struct PoseGraph {
  /// @brief Each pose, camera-to-world: rigid, or a similarity `[[s R, t], [0, 1]]`.
  std::vector<Eigen::Matrix4d> poses;
  /// @brief How the solve went.
  Summary summary;
};

/// @brief Optimise a pose graph, each edge's measured relative pose with unit information.
///
/// ORB-SLAM's `OptimizeEssentialGraph` on GTSAM: edge `(i, j)` measures `T_i_j =
/// T_world_i^-1 T_world_j` (`BetweenFactor`); the poses in `fixed` are held as constants, as
/// g2o's fixed vertices hold ORB-SLAM's: an edge with one held end is a factor on its other end,
/// and one with both ends held a constant error, counted in the summary's errors. With every
/// pose held, nothing is solved: the poses come back as given, after no iteration.
/// @param poses Each pose's start, camera-to-world: rigid, or a similarity with `similarity`.
/// @param edges Each edge's two poses `(i, j)`, `i != j`.
/// @param relatives Each edge's measured `T_i_j`, of the poses' kind.
/// @param fixed The poses held fixed, at least one; each comes back exactly as given.
/// @param similarity Optimise Sim(3) rather than SE(3).
/// @param options How the solve runs.
/// @return The poses and summary; `InvalidInput` for a pose or relative not of the kind,
///         mismatched sizes, an index out of range, a self-edge, no fixed pose or bad options;
///         `NotConverged` if GTSAM fails with no usable estimate.
[[nodiscard]] Result<PoseGraph> optimize_pose_graph(std::span<const Eigen::Matrix4d> poses,
                                                    const Eigen::Ref<const IndexPairs>& edges,
                                                    std::span<const Eigen::Matrix4d> relatives,
                                                    std::span<const std::int64_t> fixed,
                                                    bool similarity, const Options& options);

/// @brief A refined similarity (or rigid motion), and how far each match is from it.
struct SimilarityRefinement {
  /// @brief `S_12`, `[[s R, t], [0, 1]]`, carrying camera 2's points into camera 1's frame.
  Eigen::Matrix4d similarity;
  /// @brief Each match's whitened squared error at the result, before the robust loss: in
  ///        image 1, then image 2; infinite where the carried point is behind the camera, NaN
  ///        where image 2 does not observe the match.
  Pairs chi2;
  /// @brief How the solve went.
  Summary summary;
};

/// @brief Refine the similarity carrying one keyframe's matched points into another's images.
///
/// ORB-SLAM's `OptimizeSim3` on GTSAM, one round: `S_12` is the only variable, each match two
/// errors (its camera-2 point in image 1, its camera-1 point in image 2), with a Huber loss or,
/// as in ORB-SLAM's second round, none.
/// @param similarity The starting `S_12`, `s > 0` (`s = 1` with `fixed_scale`).
/// @param points1 Each match's point in camera 1's frame (M x 3), M >= 1.
/// @param points2 Each match's point in camera 2's frame (M x 3).
/// @param pixels1 Where image 1 observes each (M x 2).
/// @param pixels2 Where image 2 observes each (M x 2); a NaN row where it does not (the point
///        found in a neighbour of keyframe 2), whose match then counts its image-1 error alone.
/// @param information Each observation's information, image 1 then image 2 (M x 2), positive.
/// @param model The camera model, the same for both images.
/// @param intrinsics Its intrinsics.
/// @param fixed_scale Hold the scale at 1, refining a rigid motion.
/// @param huber The Huber loss's threshold on the whitened error's norm, positive; none for no
///        robust loss.
/// @param options How the solve runs.
/// @return The motion, each match's chi-squares and the summary; `InvalidInput` for a motion
///         not of the kind, mismatched sizes, no match, a non-positive information or
///         threshold, a bad camera or bad options; `NotConverged` if GTSAM fails.
[[nodiscard]] Result<SimilarityRefinement> refine_similarity(
    const Eigen::Matrix4d& similarity, const Eigen::Ref<const Points>& points1,
    const Eigen::Ref<const Points>& points2, const Eigen::Ref<const Pairs>& pixels1,
    const Eigen::Ref<const Pairs>& pixels2, const Eigen::Ref<const Pairs>& information,
    camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics, bool fixed_scale,
    std::optional<double> huber, const Options& options);

/// @brief A bundle adjustment's solution.
struct Adjustment {
  /// @brief Each camera's `T_world_cam`.
  std::vector<Eigen::Matrix4d> poses;
  /// @brief Each point, in the world.
  Points points;
  /// @brief How the solve went.
  Summary summary;
};

/// @brief A bundle's observations, row for row.
struct Observations {
  /// @brief Each observation's camera.
  std::span<const std::int64_t> cameras;
  /// @brief Each observation's point.
  std::span<const std::int64_t> point_ids;
  /// @brief Each observation's `(u, v)` (M x 2).
  Eigen::Ref<const Pairs> pixels;
  /// @brief Each observation's right column, NaN for none.
  std::span<const double> right;
  /// @brief Each observation's information, `1 / sigma^2` in px^-2, positive.
  std::span<const double> information;
  /// @brief Each observation's Huber threshold on its whitened error's norm, positive; empty for
  ///        no robust loss.
  std::span<const double> huber;
};

/// @brief Which observations a bundle adjustment keeps (`graphs.kept_observations`).
///
/// With `near` 0, every one. Otherwise those whose point, at the start, is in front of its camera
/// and no nearer than `near` times the median of every observation's depth, the `floor(M/2)`-th
/// smallest of the M counting from 0; each depth `R(0,2) dx + R(1,2) dy + R(2,2) dz`, summed in
/// that order, `(dx, dy, dz)` the point less the camera's centre, as every language takes it.
/// @param poses Each camera's `T_world_cam`, checked.
/// @param points Each point, checked.
/// @param observations The observations, checked, at least one.
/// @param near The fraction of the median depth, non-negative.
/// @return A flag per observation.
[[nodiscard]] std::vector<bool> kept_observations(std::span<const Eigen::Matrix4d> poses,
                                                  const Eigen::Ref<const Points>& points,
                                                  const Observations& observations, double near);

/// @brief Adjust every pose and point against every observation, with no robust loss or Huber's.
///
/// ORB-SLAM's `GlobalBundleAdjustemnt` as loop closing calls it (`bRobust` false), on GTSAM; with
/// the observations' `huber`, each whitened error through a Huber loss at its threshold, as
/// ORB-SLAM's local and welding BAs weigh theirs. The cameras in `fixed` are held as constants,
/// as g2o's fixed vertices hold ORB-SLAM's: each of their observations is a factor on its point
/// alone. With `near`, the observations near their camera's plane are left out
/// (`kept_observations`): only the points and free cameras the others reach are solved, the rest
/// coming back as given, and the summary's errors are the kept observations'; with none kept,
/// nothing is solved: 0 iterations, converged, errors of 0.
/// @param poses Each camera's starting `T_world_cam`.
/// @param points Each point's start, in the world (P x 3), P >= 1.
/// @param observations The observations; every point must be observed.
/// @param model The camera model.
/// @param intrinsics Its intrinsics.
/// @param bf The baseline times the focal length, positive if any observation has a right
///        column.
/// @param fixed The cameras held fixed; each comes back exactly as given.
/// @param near Leave out each observation whose point, at the start, is behind its camera or
///        nearer than this fraction of the median depth, non-negative; 0 leaves none out.
/// @param options How the solve runs.
/// @return The poses, points and summary; `InvalidInput` for a pose not rigid, mismatched
///         sizes, an index out of range, an unobserved point, a non-positive information or
///         Huber threshold, a right column without `bf`, a negative `near`, a bad camera or
///         bad options; `NotConverged` if GTSAM fails.
[[nodiscard]] Result<Adjustment> bundle_adjust(std::span<const Eigen::Matrix4d> poses,
                                               const Eigen::Ref<const Points>& points,
                                               const Observations& observations,
                                               camera::Model model,
                                               const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                               double bf, std::span<const std::int64_t> fixed,
                                               double near, const Options& options);

/// @brief A point's reprojection error in one camera, and its Jacobians.
struct ProjectionResidual {
  /// @brief The predicted minus the measured, 2, or 3 with a right column.
  Eigen::VectorXd residual;
  /// @brief Its Jacobian on the pose's tangent `(w, v)`, perturbed on the right (D x 6).
  Eigen::MatrixXd d_pose;
  /// @brief Its Jacobian on the point (D x 3).
  Eigen::MatrixXd d_point;
  /// @brief Whether the point is in front of the camera; one that is not costs GTSAM's own
  ///        factors' constant, twice the focal length in every row, with zero Jacobians.
  bool in_front = false;
};

/// @brief A point's reprojection error in one camera (`graphs.projection_residual`).
/// @param pose The camera's `T_world_cam`.
/// @param point The point, in the world.
/// @param measured The observed `(u, v)`, and the right column for a stereo observation.
/// @param model The camera model.
/// @param intrinsics Its intrinsics.
/// @param bf The baseline times the focal length, for a right column `u - bf / z`.
/// @return The residual and Jacobians; `InvalidInput` if `measured` has neither 2 nor 3 values.
[[nodiscard]] Result<ProjectionResidual> projection_residual(
    const Eigen::Matrix4d& pose, const Eigen::Vector3d& point,
    const Eigen::Ref<const Eigen::VectorXd>& measured, camera::Model model,
    const Eigen::Ref<const Eigen::VectorXd>& intrinsics, double bf);

/// @brief A match's two reprojection errors through a similarity, and their Jacobians.
struct SimilarityResiduals {
  /// @brief `pi(S_12 X_2) - u_1`.
  Eigen::Vector2d residual1;
  /// @brief Its Jacobian on the motion's tangent, on the right (2 x 6 or 7).
  Eigen::MatrixXd d_similarity1;
  /// @brief `pi(S_12^-1 X_1) - u_2`.
  Eigen::Vector2d residual2;
  /// @brief Its Jacobian (2 x 6 or 7).
  Eigen::MatrixXd d_similarity2;
  /// @brief Whether `S_12 X_2` is in front of camera 1.
  bool in_front1 = false;
  /// @brief Whether `S_12^-1 X_1` is in front of camera 2.
  bool in_front2 = false;
};

/// @brief A match's two reprojection errors through a similarity
///        (`graphs.similarity_residuals`).
/// @param similarity `S_12`, `[[s R, t], [0, 1]]`.
/// @param point1 The match's point in camera 1's frame.
/// @param point2 Its point in camera 2's frame.
/// @param pixel1 Where image 1 observes it.
/// @param pixel2 Where image 2 observes it.
/// @param model The camera model, the same for both images.
/// @param intrinsics Its intrinsics.
/// @param fixed_scale Whether the motion is rigid: its tangent is then `(w, v)`.
/// @return The residuals and Jacobians.
[[nodiscard]] SimilarityResiduals similarity_residuals(
    const Eigen::Matrix4d& similarity, const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
    const Eigen::Vector2d& pixel1, const Eigen::Vector2d& pixel2, camera::Model model,
    const Eigen::Ref<const Eigen::VectorXd>& intrinsics, bool fixed_scale);

}  // namespace slam_it::foundations::graphs
