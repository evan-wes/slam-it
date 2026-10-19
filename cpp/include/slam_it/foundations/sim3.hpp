/// @file
/// @brief Sim(3) RANSAC: the C++ core's implementation.
///
/// Mirrors `slam_it.foundations.sim3`, whose docstring states the method (ORB-SLAM's
/// `Sim3Solver`): Umeyama's alignment of three matches, a match an inlier when each point,
/// carried into the other camera, projects within the chi-square bound of its partner's
/// projection, and rung 0's LO-MSAC run to its end.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/correspondences.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/ransac.hpp"

/// @brief Sim(3) RANSAC.
namespace slam_it::foundations::sim3 {

using correspondences::Rows2;
using correspondences::Rows3;

/// @brief A minimal sample: three matches.
inline constexpr std::uint32_t kSampleSize = 3;

/// @brief The inverse of `[[s R, t], [0, 1]]`.
/// @param similarity The similarity, its scale positive.
/// @return `[[R^T / s, -R^T t / s], [0, 1]]`.
[[nodiscard]] Eigen::Matrix4d inverse(const Eigen::Matrix4d& similarity);

/// @brief Umeyama's alignment with the reprojection test in both images, as a
///        `ransac::Estimator`.
class SimilarityEstimator {
 public:
  /// @brief The model: `S_12`, `[[s R, t], [0, 1]]`.
  using Model = Eigen::Matrix4d;

  /// @brief The matches, checked, and where each point projects in its own image.
  struct Matches {
    /// @brief Each point in camera 1's frame (N x 3).
    Rows3 points1;
    /// @brief Each point in camera 2's frame (N x 3).
    Rows3 points2;
    /// @brief `points1` projected into image 1 (N x 2).
    Rows2 image1;
    /// @brief `points2` projected into image 2 (N x 2).
    Rows2 image2;
    /// @brief Each match's variance in image 1, px².
    Eigen::VectorXd sigma2_1;
    /// @brief Each match's variance in image 2, px².
    Eigen::VectorXd sigma2_2;
  };

  /// @brief Hold checked matches.
  /// @param matches The matches.
  /// @param model The camera model.
  /// @param intrinsics Its intrinsics, checked.
  /// @param fixed_scale Fit rigid motions (`s = 1`).
  SimilarityEstimator(Matches matches, camera::Model model, Eigen::VectorXd intrinsics,
                      bool fixed_scale);

  /// @brief Three matches.
  /// @return The minimal sample size.
  [[nodiscard]] static std::uint32_t sample_size() { return kSampleSize; }

  /// @brief The number of matches.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const;

  /// @brief The similarity of three matches.
  /// @param sample The matches.
  /// @return `S_12`, or none for collinear or coincident points.
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const;

  /// @brief The least-squares similarity of three or more matches.
  /// @param subset The matches.
  /// @return `S_12`, or none for collinear or coincident points.
  [[nodiscard]] std::vector<Model> fit_nonminimal(std::span<const std::uint32_t> subset) const;

  /// @brief Each match's squared residual: its larger error over its variance.
  /// @param model `S_12`.
  /// @return `max(|e1|² / σ1², |e2|² / σ2²)`; infinity where a point is carried behind the
  ///         other camera.
  [[nodiscard]] Eigen::VectorXd residuals(const Model& model) const;

 private:
  /// @brief The least-squares similarity carrying the rows' `points2` onto their `points1`.
  /// @param rows The matches.
  /// @return `S_12`, or none if degenerate.
  [[nodiscard]] std::vector<Model> fit(std::span<const std::uint32_t> rows) const;

  /// @brief The matches.
  Matches matches_;
  /// @brief The camera model.
  camera::Model model_;
  /// @brief Its intrinsics.
  Eigen::VectorXd intrinsics_;
  /// @brief Whether the scale is held at 1.
  bool fixed_scale_;
};

/// @brief Estimate `S_12` from two cameras' matched points robustly (`sim3.sim3_ransac`).
/// @param points1 Each match's point in camera 1's frame (N x 3), in front of it, N >= 3.
/// @param points2 Each match's point in camera 2's frame (N x 3), in front of it.
/// @param sigma2_1 Each match's variance in image 1, px², positive.
/// @param sigma2_2 Each match's variance in image 2, px², positive.
/// @param model The camera model, the same for both images.
/// @param intrinsics Its intrinsics.
/// @param fixed_scale Estimate a rigid motion, the scale held at 1.
/// @param options RANSAC's settings; the threshold is the chi-square bound's square root.
/// @return `S_12`, its inliers and the run's summary; `InvalidInput` for invalid matches, camera
///         or settings; `Degenerate` if no sample gave a similarity.
[[nodiscard]] Result<ransac::Solution<Eigen::Matrix4d>> sim3_ransac(
    const Eigen::Ref<const Rows3>& points1, const Eigen::Ref<const Rows3>& points2,
    const Eigen::Ref<const Eigen::VectorXd>& sigma2_1,
    const Eigen::Ref<const Eigen::VectorXd>& sigma2_2, camera::Model model,
    const Eigen::Ref<const Eigen::VectorXd>& intrinsics, bool fixed_scale,
    const ransac::Options& options);

}  // namespace slam_it::foundations::sim3
