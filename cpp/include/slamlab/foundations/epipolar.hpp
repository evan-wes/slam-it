/// @file
/// @brief Epipolar geometry: the C++ core's 8-point essential and fundamental matrices,
///        their Sampson errors, and their RANSAC estimators.
///
/// Mirrors `slamlab.foundations.epipolar`, whose docstring states the maths: the 8-point
/// algorithm (normalised for pixels), projection onto the essential manifold or rank 2, and
/// the Sampson error (in the normalised plane for E, in pixels for F). E is returned with
/// singular values (1, 1, 0), F with unit Frobenius norm, both up to sign.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/ransac.hpp"

/// @brief Epipolar geometry.
namespace slamlab::foundations::epipolar {

/// @brief The 8-point system's second-smallest singular value must exceed this fraction of its
///        largest; below it the solution is not unique.
inline constexpr double kRankTolerance = 1e-10;
/// @brief Correspondences in a minimal sample.
inline constexpr std::uint32_t kSampleSize = 8;

/// @brief Rows of 3-vectors (bearings, homogeneous points), row-major as numpy stores them.
using Rows3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
/// @brief Rows of 2-vectors (pixels, image points), row-major as numpy stores them.
using Rows2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

/// @brief Append a 1 to each 2D point.
/// @param points The points (N x 2).
/// @return `(x, y, 1)` per point.
[[nodiscard]] Rows3 homogeneous(const Eigen::Ref<const Rows2>& points);

/// @brief Map bearings with positive z to homogeneous normalised image points `f / f_z`.
/// @param bearings The bearings (N x 3).
/// @return `(x, y, 1)` per bearing.
[[nodiscard]] Rows3 normalised_plane(const Eigen::Ref<const Rows3>& bearings);

/// @brief Hartley's similarity: centroid to the origin, mean distance sqrt(2).
/// @param points The points (N x 2).
/// @return `T`, or nothing if every point coincides.
[[nodiscard]] std::optional<Eigen::Matrix3d> hartley_normalisation(
    const Eigen::Ref<const Rows2>& points);

/// @brief Solve `A v = 0` for a unit `v` by SVD, if the solution is unique.
/// @param system `A` (M x 9), M >= 8.
/// @return The right singular vector of the smallest singular value, or nothing if the
///         second-smallest is below `kRankTolerance` of the largest.
[[nodiscard]] std::optional<Eigen::Matrix<double, 9, 1>> null_vector(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 9>>& system);

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

/// @brief Check bearing pairs: equal sizes, at least `minimum`, unit vectors with positive z.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @param minimum The fewest pairs accepted.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_bearing_pairs(const Eigen::Ref<const Rows3>& bearings1,
                                               const Eigen::Ref<const Rows3>& bearings2,
                                               Eigen::Index minimum);

/// @brief Check point pairs: equal sizes, at least `minimum`, finite.
/// @param points1 Points in image 1.
/// @param points2 Points in image 2.
/// @param minimum The fewest pairs accepted.
/// @param name The arguments' name, for messages.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_point_pairs(const Eigen::Ref<const Rows2>& points1,
                                             const Eigen::Ref<const Rows2>& points2,
                                             Eigen::Index minimum, const char* name);

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
  EssentialEstimator(Rows3 bearings1, Rows3 bearings2);

  /// @brief Eight correspondences.
  /// @return 8.
  [[nodiscard]] static std::uint32_t sample_size() { return kSampleSize; }

  /// @brief The number of correspondences.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const;

  /// @brief Fit E to eight correspondences.
  /// @param sample The indices.
  /// @return E, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const;

  /// @brief Fit E to eight or more correspondences.
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
};

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

/// @brief Gather the rows of a matrix at the given indices.
/// @tparam M The row-major matrix type.
/// @param rows The matrix.
/// @param indices The rows to take.
/// @return The gathered rows.
template <typename M>
[[nodiscard]] M gather(const M& rows, std::span<const std::uint32_t> indices) {
  M out(static_cast<Eigen::Index>(indices.size()), rows.cols());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    out.row(static_cast<Eigen::Index>(i)) = rows.row(indices[i]);
  }
  return out;
}

/// @brief Estimate E robustly: LO-MSAC with the 8-point solver.
/// @param bearings1 Unit bearings in camera 1 (N x 3), positive z, N >= 8.
/// @param bearings2 Unit bearings in camera 2 (N x 3), positive z.
/// @param options The settings (threshold: a Sampson distance in the normalised plane).
/// @return E, its inliers and the run's summary; `InvalidInput` or `Degenerate`.
[[nodiscard]] Result<ransac::Solution<Eigen::Matrix3d>> essential_ransac(
    const Eigen::Ref<const Rows3>& bearings1, const Eigen::Ref<const Rows3>& bearings2,
    const ransac::Options& options);

/// @brief Estimate F robustly: LO-MSAC with the normalised 8-point solver.
/// @param pixels1 Pixels in image 1 (N x 2), N >= 8.
/// @param pixels2 Pixels in image 2 (N x 2).
/// @param options The settings (threshold: a Sampson distance in pixels).
/// @return F, its inliers and the run's summary; `InvalidInput` or `Degenerate`.
[[nodiscard]] Result<ransac::Solution<Eigen::Matrix3d>> fundamental_ransac(
    const Eigen::Ref<const Rows2>& pixels1, const Eigen::Ref<const Rows2>& pixels2,
    const ransac::Options& options);

}  // namespace slamlab::foundations::epipolar
