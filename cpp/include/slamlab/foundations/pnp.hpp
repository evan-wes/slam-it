/// @file
/// @brief Absolute pose (PnP): the C++ core's EPnP, pose refinement and PnP RANSAC.
///
/// Mirrors `slamlab.foundations.pnp`, whose docstring states the maths: the reprojection
/// error in the normalised image plane (infinite behind the camera), EPnP's control points,
/// null space and three linearisations of the distance constraints (Lepetit et al., IJCV
/// 2009), the Levenberg-Marquardt refinement on SE(3) with a Cauchy loss, and LO-MSAC with a
/// P3P minimal solver and EPnP refits, its consensus refined and kept only if the MSAC cost
/// drops. Poses are `T_cam_world`.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slamlab/foundations/correspondences.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/nls.hpp"
#include "slamlab/foundations/p3p.hpp"
#include "slamlab/foundations/ransac.hpp"
#include "slamlab/foundations/solve.hpp"

/// @brief Absolute pose.
namespace slamlab::foundations::pnp {

using correspondences::Rows2;
using correspondences::Rows3;

/// @brief Correspondences EPnP takes (see `slamlab.foundations.pnp.EPNP_MINIMUM`).
inline constexpr std::uint32_t kEpnpMinimum = 5;
/// @brief The points are planar for EPnP when their covariance's smallest eigenvalue is below
///        this fraction of its largest.
inline constexpr double kPlanarTolerance = 1e-12;
/// @brief At most this many Gauss-Newton steps polish EPnP's betas.
inline constexpr int kPolishMaxSteps = 10;
/// @brief Polishing stops when a step is below this, relative to the betas.
inline constexpr double kPolishTolerance = 1e-14;
/// @brief Levenberg-Marquardt iterations of the pose refinement.
inline constexpr int kRefineIterations = 20;

/// @brief Squared normalised-plane reprojection errors; infinity behind the camera.
/// @param pose `T_cam_world`.
/// @param points World points (N x 3).
/// @param image Their bearings' normalised image points (N x 2).
/// @return The errors.
[[nodiscard]] Eigen::VectorXd reprojection(const Eigen::Matrix4d& pose,
                                           const Eigen::Ref<const Rows3>& points,
                                           const Eigen::Ref<const Rows2>& image);

/// @brief EPnP on checked correspondences (unchecked).
/// @param points World points (N x 3), N >= 5.
/// @param bearings Their unit bearings, positive z.
/// @return The pose, or nothing if the points are planar or degenerate.
[[nodiscard]] std::optional<Eigen::Matrix4d> solve_epnp(const Eigen::Ref<const Rows3>& points,
                                                        const Eigen::Ref<const Rows3>& bearings);

/// @brief Estimate the camera pose from five or more correspondences by EPnP.
/// @param points World points (N x 3), spanning three dimensions.
/// @param bearings Their unit bearings, positive z.
/// @return `T_cam_world`; `InvalidInput` for invalid or too few correspondences; `Degenerate`
///         for planar points.
[[nodiscard]] Result<Eigen::Matrix4d> epnp(const Eigen::Ref<const Rows3>& points,
                                           const Eigen::Ref<const Rows3>& bearings);

/// @brief Refine a pose by Levenberg-Marquardt on the Cauchy-robust reprojection error
///        (unchecked).
/// @param pose The starting `T_cam_world`.
/// @param points World points.
/// @param bearings Their unit bearings, positive z.
/// @param scale The Cauchy loss's scale, a whitened error in the normalised plane.
/// @param weights Each correspondence's weight, positive.
/// @return The refined pose and the solve's summary; `InvalidInput` if a point is behind the
///         starting camera (an infinite start).
[[nodiscard]] Result<solve::Solution<Eigen::Matrix4d, nls::Summary>> refine(
    const Eigen::Matrix4d& pose, const Eigen::Ref<const Rows3>& points,
    const Eigen::Ref<const Rows3>& bearings, double scale,
    const Eigen::Ref<const Eigen::VectorXd>& weights);

/// @brief Refine a pose by minimising the inliers' robust reprojection error over SE(3).
/// @param pose The starting `T_cam_world`.
/// @param points The inliers' world points (M x 3), M >= 3.
/// @param bearings Their unit bearings, positive z.
/// @param scale The Cauchy loss's scale, a whitened error in the normalised plane (positive).
/// @param weights Each correspondence's weight, its information in the units of `scale`
///        (ORB-SLAM's keypoint-level weighting): its residual is scaled by the square root. Not
///        relative: a common factor k on the weights acts as 1/√k on the scale.
/// @return The refined pose and the summary; `InvalidInput` for an invalid pose or
///         correspondences, M < 3, a non-positive scale, weights that are not positive and one
///         per correspondence, or a point behind the start.
[[nodiscard]] Result<solve::Solution<Eigen::Matrix4d, nls::Summary>> refine_pose(
    const Eigen::Matrix4d& pose, const Eigen::Ref<const Rows3>& points,
    const Eigen::Ref<const Rows3>& bearings, double scale,
    const Eigen::Ref<const Eigen::VectorXd>& weights);

/// @brief P3P and EPnP with the reprojection error, as a `ransac::Estimator`.
class PoseEstimator {
 public:
  /// @brief The model: `T_cam_world`.
  using Model = Eigen::Matrix4d;

  /// @brief Hold checked correspondences.
  /// @param points World points.
  /// @param bearings Their unit bearings, positive z.
  /// @param solver The P3P solver.
  PoseEstimator(Rows3 points, Rows3 bearings, p3p::Solver solver);

  /// @brief Three correspondences.
  /// @return The minimal sample size.
  [[nodiscard]] static std::uint32_t sample_size() { return p3p::kSampleSize; }

  /// @brief The number of correspondences.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const;

  /// @brief Every pose P3P gives for three correspondences.
  /// @param sample The correspondences.
  /// @return The poses (none for collinear points).
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const;

  /// @brief EPnP's pose for five or more correspondences.
  /// @param subset The correspondences.
  /// @return The pose, or none for fewer than five or planar points.
  [[nodiscard]] std::vector<Model> fit_nonminimal(std::span<const std::uint32_t> subset) const;

  /// @brief The squared reprojection errors in the normalised plane.
  /// @param model The pose.
  /// @return The errors; infinity behind the camera.
  [[nodiscard]] Eigen::VectorXd residuals(const Model& model) const;

 private:
  /// @brief The world points.
  Rows3 points_;
  /// @brief Their bearings.
  Rows3 bearings_;
  /// @brief Their normalised image points.
  Rows2 image_;
  /// @brief The P3P solver.
  p3p::Solver solver_;
};

/// @brief Estimate the camera pose robustly: LO-MSAC with a P3P solver and EPnP refits, then
///        (optionally) the refinement on the inliers, kept only if it lowers the MSAC cost.
/// @param points World points (N x 3), N >= 3.
/// @param bearings Their unit bearings, positive z.
/// @param options RANSAC's settings; the threshold is in the normalised plane.
/// @param minimal The P3P solver.
/// @param refine Refine the consensus.
/// @return The pose, its inliers and the summary; `InvalidInput` for invalid input or
///         settings; `Degenerate` if no sample gave a pose.
[[nodiscard]] Result<ransac::RefinedSolution<Eigen::Matrix4d>> pnp_ransac(
    const Eigen::Ref<const Rows3>& points, const Eigen::Ref<const Rows3>& bearings,
    const ransac::Options& options, p3p::Solver minimal = p3p::Solver::kDing, bool refine = true);

}  // namespace slamlab::foundations::pnp
