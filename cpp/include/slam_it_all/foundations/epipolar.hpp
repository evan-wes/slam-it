/// @file
/// @brief Epipolar geometry: the C++ core's 8-point essential and fundamental matrices,
///        their Sampson errors, and their RANSAC estimators.
///
/// Mirrors `slam_it_all.foundations.epipolar`, whose docstring states the maths: the 8-point
/// algorithm (normalised for pixels), projection onto the essential manifold or rank 2, and
/// the Sampson error (in the normalised plane for E, in pixels for F). E is returned with
/// singular values (1, 1, 0), F with unit Frobenius norm, both up to sign.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slam_it_all/foundations/correspondences.hpp"
#include "slam_it_all/foundations/error.hpp"
#include "slam_it_all/foundations/fivepoint.hpp"
#include "slam_it_all/foundations/nls.hpp"
#include "slam_it_all/foundations/ransac.hpp"

/// @brief Epipolar geometry.
namespace slam_it_all::foundations::epipolar {

using correspondences::Rows2;
using correspondences::Rows3;

/// @brief Correspondences the 8-point algorithm needs (its minimal sample).
inline constexpr std::uint32_t kSampleSize = 8;
/// @brief Levenberg-Marquardt iterations of the Sampson refinement.
inline constexpr int kRefineIterations = 20;

/// @brief E-RANSAC's minimal solver, with the codes it crosses the bindings as.
enum class MinimalSolver : std::uint8_t {
  /// @brief Nistér's 5-point solver.
  kNister = 0,
  /// @brief Stewénius' 5-point solver.
  kStewenius = 1,
  /// @brief The 8-point algorithm.
  k8pt = 2,
  /// @brief The number of solvers (a sentinel).
  kCount = 3,
};

/// @brief The solver's name, as `slam_it_all.foundations.epipolar.MINIMAL_SOLVERS` spells it.
/// @param solver The solver.
/// @return A static string.
[[nodiscard]] const char* name(MinimalSolver solver);

/// @brief The squared Sampson error of homogeneous point pairs (z = 1) under E or F.
/// @param matrix E (on normalised points) or F (on pixels).
/// @param x1 Points in image 1 (N x 3).
/// @param x2 Points in image 2 (N x 3).
/// @return One error per pair; infinity where the denominator vanishes.
[[nodiscard]] Eigen::VectorXd sampson(const Eigen::Matrix3d& matrix,
                                      const Eigen::Ref<const Rows3>& x1,
                                      const Eigen::Ref<const Rows3>& x2);

/// @brief The normalised 8-point E on the bearings' normalised image points, projected onto
///        the essential manifold (unchecked).
/// @param bearings1 Bearings in camera 1 (N x 3), N >= 8.
/// @param bearings2 Bearings in camera 2 (N x 3).
/// @return E, or nothing if the pairs do not determine it.
[[nodiscard]] std::optional<Eigen::Matrix3d> fit_essential(
    const Eigen::Ref<const Rows3>& bearings1, const Eigen::Ref<const Rows3>& bearings2);

/// @brief The normalised 8-point F on pixel pairs, rank 2, unit norm (unchecked).
/// @param pixels1 Pixels in image 1 (N x 2), N >= 8.
/// @param pixels2 Pixels in image 2 (N x 2).
/// @return F, or nothing if the pairs do not determine it.
[[nodiscard]] std::optional<Eigen::Matrix3d> fit_fundamental(
    const Eigen::Ref<const Rows2>& pixels1, const Eigen::Ref<const Rows2>& pixels2);

/// @brief Estimate E from eight or more bearing pairs.
/// @param bearings1 Unit bearings in camera 1 (N x 3), positive z.
/// @param bearings2 Unit bearings in camera 2 (N x 3), positive z.
/// @return E; `InvalidInput` for bad bearings or N < 8; `Degenerate` if not unique.
[[nodiscard]] Result<Eigen::Matrix3d> essential_8pt(const Eigen::Ref<const Rows3>& bearings1,
                                                    const Eigen::Ref<const Rows3>& bearings2);

/// @brief Estimate F from eight or more pixel pairs (normalised 8-point).
/// @param pixels1 Pixels in image 1 (N x 2).
/// @param pixels2 Pixels in image 2 (N x 2).
/// @return F; `InvalidInput` for mismatched sizes or N < 8; `Degenerate` if not unique.
[[nodiscard]] Result<Eigen::Matrix3d> fundamental_8pt(const Eigen::Ref<const Rows2>& pixels1,
                                                      const Eigen::Ref<const Rows2>& pixels2);

/// @brief Squared Sampson errors of bearing pairs under E, in the normalised plane.
/// @param essential E.
/// @param bearings1 Unit bearings in camera 1 (N x 3), positive z.
/// @param bearings2 Unit bearings in camera 2 (N x 3), positive z.
/// @return One error per pair, or `InvalidInput`.
[[nodiscard]] Result<Eigen::VectorXd> sampson_essential(const Eigen::Matrix3d& essential,
                                                        const Eigen::Ref<const Rows3>& bearings1,
                                                        const Eigen::Ref<const Rows3>& bearings2);

/// @brief Squared Sampson errors of pixel pairs under F.
/// @param fundamental F.
/// @param pixels1 Pixels in image 1 (N x 2).
/// @param pixels2 Pixels in image 2 (N x 2).
/// @return One error per pair, or `InvalidInput`.
[[nodiscard]] Result<Eigen::VectorXd> sampson_fundamental(const Eigen::Matrix3d& fundamental,
                                                          const Eigen::Ref<const Rows2>& pixels1,
                                                          const Eigen::Ref<const Rows2>& pixels2);

/// @brief The 8-point E solver and its Sampson error, as a `ransac::Estimator`.
class EssentialEstimator {
 public:
  /// @brief The model: an essential matrix.
  using Model = Eigen::Matrix3d;

  /// @brief Hold checked bearing pairs.
  /// @param bearings1 Bearings in camera 1 (N x 3), unit, positive z.
  /// @param bearings2 Bearings in camera 2 (N x 3), unit, positive z.
  /// @param minimal The minimal solver.
  EssentialEstimator(Rows3 bearings1, Rows3 bearings2, MinimalSolver minimal);

  /// @brief The minimal sample: five, or eight for the 8-point algorithm.
  /// @return The size.
  [[nodiscard]] std::uint32_t sample_size() const;

  /// @brief The number of correspondences.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const;

  /// @brief Fit E to a minimal sample.
  /// @param sample The indices.
  /// @return Every E the minimal solver allows (none if degenerate).
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const;

  /// @brief Fit E to eight or more correspondences (the 8-point algorithm).
  /// @param subset The indices.
  /// @return E, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit_nonminimal(std::span<const std::uint32_t> subset) const;

  /// @brief The squared Sampson errors in the normalised plane.
  /// @param model E.
  /// @return One per correspondence.
  [[nodiscard]] Eigen::VectorXd residuals(const Model& model) const;

 private:
  /// @brief The bearings in camera 1.
  Rows3 bearings1_;
  /// @brief The bearings in camera 2.
  Rows3 bearings2_;
  /// @brief Their normalised image points in camera 1.
  Rows3 x1_;
  /// @brief Their normalised image points in camera 2.
  Rows3 x2_;
  /// @brief The minimal solver.
  MinimalSolver minimal_;
};

/// @brief E's motion on its manifold: `E = [t]x R` with |t| = 1.
struct Motion {
  /// @brief R.
  Eigen::Matrix3d rotation;
  /// @brief The unit translation t.
  Eigen::Vector3d translation;
};

/// @brief A motion's E (`Motion.essential` in Python).
/// @param motion The motion.
/// @return `[t]x R`: singular values (1, 1, 0).
[[nodiscard]] Eigen::Matrix3d essential_of(const Motion& motion);

/// @brief An orthonormal basis of the plane tangent to the unit sphere at a direction, built
///        from the coordinate axis least aligned with it (as in Python).
/// @param direction A unit vector.
/// @return Two orthonormal columns perpendicular to it.
[[nodiscard]] Eigen::Matrix<double, 3, 2> tangent_basis(const Eigen::Vector3d& direction);

/// @brief The motion a refinement of E starts from (see
/// `slam_it_all.foundations.epipolar.start_motion`):
///        the candidate rotation with the largest trace, and the translation's sign that makes
///        its largest-magnitude component positive.
/// @param essential E, of rank 2.
/// @return The start; `decompose_essential`'s errors if E is not finite or its rank is below 2.
[[nodiscard]] Result<Motion> start_motion(const Eigen::Matrix3d& essential);

/// @brief Refine E by Levenberg-Marquardt on the Cauchy-robust Sampson distance (unchecked).
/// @param essential The starting E.
/// @param bearings1 Bearings in camera 1 (M x 3), positive z, M >= 5.
/// @param bearings2 Bearings in camera 2 (M x 3).
/// @param scale The Cauchy scale, a Sampson distance in the normalised plane.
/// @return The refined E and the solve's summary; `InvalidInput` if a match's Sampson distance
///         is not finite at the start (a match at both epipoles).
[[nodiscard]] Result<solve::Solution<Eigen::Matrix3d, nls::Summary>> refine(
    const Eigen::Matrix3d& essential, const Eigen::Ref<const Rows3>& bearings1,
    const Eigen::Ref<const Rows3>& bearings2, double scale);

/// @brief Refine E on its inliers by minimising the robust Sampson distance over E's manifold.
/// @param essential The starting E.
/// @param bearings1 Unit bearings in camera 1 (M x 3), positive z, M >= 5.
/// @param bearings2 Unit bearings in camera 2 (M x 3), positive z.
/// @param scale The Cauchy scale (positive).
/// @return The refined E and the summary; `InvalidInput` for bad input; `Degenerate` if E's
///         rank is below 2.
[[nodiscard]] Result<solve::Solution<Eigen::Matrix3d, nls::Summary>> refine_essential(
    const Eigen::Matrix3d& essential, const Eigen::Ref<const Rows3>& bearings1,
    const Eigen::Ref<const Rows3>& bearings2, double scale);

/// @brief The normalised 8-point F solver and its Sampson error, as a `ransac::Estimator`.
class FundamentalEstimator {
 public:
  /// @brief The model: a fundamental matrix.
  using Model = Eigen::Matrix3d;

  /// @brief Hold checked pixel pairs.
  /// @param pixels1 Pixels in image 1 (N x 2).
  /// @param pixels2 Pixels in image 2 (N x 2).
  FundamentalEstimator(Rows2 pixels1, Rows2 pixels2);

  /// @brief Eight correspondences.
  /// @return 8.
  [[nodiscard]] static std::uint32_t sample_size() { return kSampleSize; }

  /// @brief The number of correspondences.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const;

  /// @brief Fit F to eight correspondences.
  /// @param sample The indices.
  /// @return F, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const;

  /// @brief Fit F to eight or more correspondences.
  /// @param subset The indices.
  /// @return F, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit_nonminimal(std::span<const std::uint32_t> subset) const;

  /// @brief The squared Sampson errors in pixels.
  /// @param model F.
  /// @return One per correspondence.
  [[nodiscard]] Eigen::VectorXd residuals(const Model& model) const;

 private:
  /// @brief The pixels in image 1.
  Rows2 pixels1_;
  /// @brief The pixels in image 2.
  Rows2 pixels2_;
  /// @brief Their homogeneous points in image 1.
  Rows3 x1_;
  /// @brief Their homogeneous points in image 2.
  Rows3 x2_;
};

/// @brief The four relative poses an essential matrix allows: `(R1, t)`, `(R1, -t)`,
///        `(R2, t)`, `(R2, -t)` with `R = U W V^T` or `U W^T V^T` and `t = u3` (unit).
/// @param essential E (any scale).
/// @return The candidates `T_c2_c1`; `InvalidInput` if not finite; `Degenerate` if its rank
///         is below 2.
[[nodiscard]] Result<std::vector<Eigen::Matrix4d>> decompose_essential(
    const Eigen::Matrix3d& essential);

/// @brief Estimate E robustly: LO-MSAC, then (optionally) the Sampson refinement on the
///        inliers, kept only if it lowers the MSAC cost.
/// @param bearings1 Unit bearings in camera 1 (N x 3), positive z, N >= the minimal sample.
/// @param bearings2 Unit bearings in camera 2 (N x 3), positive z.
/// @param options The settings (threshold: a Sampson distance in the normalised plane).
/// @param minimal The minimal solver.
/// @param refine Refine the best model on its inliers.
/// @return E, its inliers and the run's summary; `InvalidInput` or `Degenerate`.
[[nodiscard]] Result<ransac::RefinedSolution<Eigen::Matrix3d>> essential_ransac(
    const Eigen::Ref<const Rows3>& bearings1, const Eigen::Ref<const Rows3>& bearings2,
    const ransac::Options& options, MinimalSolver minimal = MinimalSolver::kStewenius,
    bool refine = true);

/// @brief Estimate F robustly: LO-MSAC with the normalised 8-point solver.
/// @param pixels1 Pixels in image 1 (N x 2), N >= 8.
/// @param pixels2 Pixels in image 2 (N x 2).
/// @param options The settings (threshold: a Sampson distance in pixels).
/// @return F, its inliers and the run's summary; `InvalidInput` or `Degenerate`.
[[nodiscard]] Result<ransac::Solution<Eigen::Matrix3d>> fundamental_ransac(
    const Eigen::Ref<const Rows2>& pixels1, const Eigen::Ref<const Rows2>& pixels2,
    const ransac::Options& options);

}  // namespace slam_it_all::foundations::epipolar
