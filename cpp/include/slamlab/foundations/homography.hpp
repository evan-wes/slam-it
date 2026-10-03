/// @file
/// @brief Homographies: the C++ core's normalised DLT, its Sampson error, and its RANSAC
///        estimator.
///
/// Mirrors `slamlab.foundations.homography`, whose docstring states the maths: the DLT on
/// Hartley-normalised points, from the first two rows of `x2 x H x1 = 0`, and the Sampson
/// error `eps^T (J J^T)^-1 eps`. H is returned with unit Frobenius norm, up to sign.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "slamlab/foundations/epipolar.hpp"
#include "slamlab/foundations/error.hpp"
#include "slamlab/foundations/ransac.hpp"

/// @brief Homographies.
namespace slamlab::foundations::homography {

/// @brief Correspondences in a minimal sample.
inline constexpr std::uint32_t kSampleSize = 4;

using epipolar::Rows2;

/// @brief The normalised DLT (unchecked).
/// @param points1 Points in image 1 (N x 2), N >= 4.
/// @param points2 Points in image 2 (N x 2).
/// @return H with unit norm, or nothing if the pairs do not determine it.
[[nodiscard]] std::optional<Eigen::Matrix3d> fit(const Eigen::Ref<const Rows2>& points1,
                                                 const Eigen::Ref<const Rows2>& points2);

/// @brief The squared Sampson error of point pairs under H (unchecked).
/// @param homography H.
/// @param points1 Points in image 1 (N x 2).
/// @param points2 Points in image 2 (N x 2).
/// @return One error per pair; infinity where `J J^T` is singular.
[[nodiscard]] Eigen::VectorXd sampson(const Eigen::Matrix3d& homography,
                                      const Eigen::Ref<const Rows2>& points1,
                                      const Eigen::Ref<const Rows2>& points2);

/// @brief Estimate H from four or more point pairs (normalised DLT).
/// @param points1 Points in image 1 (N x 2).
/// @param points2 Points in image 2 (N x 2).
/// @return H; `InvalidInput` for mismatched sizes or N < 4; `Degenerate` if not unique.
[[nodiscard]] Result<Eigen::Matrix3d> homography_dlt(const Eigen::Ref<const Rows2>& points1,
                                                     const Eigen::Ref<const Rows2>& points2);

/// @brief Squared Sampson errors of point pairs under H.
/// @param homography H.
/// @param points1 Points in image 1 (N x 2).
/// @param points2 Points in image 2 (N x 2).
/// @return One error per pair, or `InvalidInput`.
[[nodiscard]] Result<Eigen::VectorXd> sampson_homography(const Eigen::Matrix3d& homography,
                                                         const Eigen::Ref<const Rows2>& points1,
                                                         const Eigen::Ref<const Rows2>& points2);

/// @brief The normalised DLT and its Sampson error, as a `ransac::Estimator`.
class HomographyEstimator {
 public:
  /// @brief The model: a homography.
  using Model = Eigen::Matrix3d;

  /// @brief Hold checked point pairs.
  /// @param points1 Points in image 1 (N x 2).
  /// @param points2 Points in image 2 (N x 2).
  HomographyEstimator(Rows2 points1, Rows2 points2);

  /// @brief Four correspondences.
  /// @return 4.
  [[nodiscard]] static std::uint32_t sample_size() { return kSampleSize; }

  /// @brief The number of correspondences.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const;

  /// @brief Fit H to four correspondences.
  /// @param sample The indices.
  /// @return H, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const;

  /// @brief Fit H to four or more correspondences.
  /// @param subset The indices.
  /// @return H, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit_nonminimal(std::span<const std::uint32_t> subset) const;

  /// @brief The squared Sampson errors.
  /// @param model H.
  /// @return One per correspondence.
  [[nodiscard]] Eigen::VectorXd residuals(const Model& model) const;

 private:
  /// @brief The points in image 1.
  Rows2 points1_;
  /// @brief The points in image 2.
  Rows2 points2_;
};

/// @brief Estimate H robustly: LO-MSAC with the normalised DLT.
/// @param points1 Points in image 1 (N x 2), N >= 4.
/// @param points2 Points in image 2 (N x 2).
/// @param options The settings (threshold: a Sampson distance in the points' units).
/// @return H, its inliers and the run's summary; `InvalidInput` or `Degenerate`.
[[nodiscard]] Result<ransac::Solution<Eigen::Matrix3d>> homography_ransac(
    const Eigen::Ref<const Rows2>& points1, const Eigen::Ref<const Rows2>& points2,
    const ransac::Options& options);

}  // namespace slamlab::foundations::homography
