/// @file
/// @brief Implementation of the homographies declared in slamlab/foundations/homography.hpp.
#include "slamlab/foundations/homography.hpp"

#include <Eigen/Dense>
#include <limits>
#include <utility>

namespace slamlab::foundations::homography {

std::optional<Eigen::Matrix3d> fit(const Eigen::Ref<const Rows2>& points1,
                                   const Eigen::Ref<const Rows2>& points2) {
  const auto t1 = epipolar::hartley_normalisation(points1);
  const auto t2 = epipolar::hartley_normalisation(points2);
  if (!t1 || !t2) {
    return std::nullopt;
  }
  const epipolar::Rows3 x1 = epipolar::homogeneous(points1) * t1->transpose();
  const epipolar::Rows3 x2 = epipolar::homogeneous(points2) * t2->transpose();
  const Eigen::Index n = x1.rows();
  Eigen::Matrix<double, Eigen::Dynamic, 9> system =
      Eigen::Matrix<double, Eigen::Dynamic, 9>::Zero(2 * n, 9);
  // Rows of x2 x H x1 = 0 (with w2 = 1): -(h2 . x1) + y2 (h3 . x1), (h1 . x1) - x2 (h3 . x1).
  for (Eigen::Index i = 0; i < n; ++i) {
    system.block<1, 3>(2 * i, 3) = -x1.row(i);
    system.block<1, 3>(2 * i, 6) = x2(i, 1) * x1.row(i);
    system.block<1, 3>((2 * i) + 1, 0) = x1.row(i);
    system.block<1, 3>((2 * i) + 1, 6) = -x2(i, 0) * x1.row(i);
  }
  const auto solution = epipolar::null_vector(system);
  if (!solution) {
    return std::nullopt;
  }
  const Eigen::Matrix3d normalised =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(solution->data());
  const Eigen::Matrix3d homography = t2->inverse() * normalised * *t1;
  return homography / homography.norm();
}

Eigen::VectorXd sampson(const Eigen::Matrix3d& homography, const Eigen::Ref<const Rows2>& points1,
                        const Eigen::Ref<const Rows2>& points2) {
  const Eigen::Matrix3d& h = homography;
  Eigen::VectorXd out(points1.rows());
  for (Eigen::Index i = 0; i < points1.rows(); ++i) {
    const Eigen::Vector3d mapped = h * Eigen::Vector3d(points1(i, 0), points1(i, 1), 1.0);
    const double x2 = points2(i, 0);
    const double y2 = points2(i, 1);
    const double w = mapped(2);
    const Eigen::Vector2d eps((y2 * w) - mapped(1), mapped(0) - (x2 * w));
    // J = d(eps1, eps2) / d(x1, y1, x2, y2).
    const Eigen::Vector4d j1((y2 * h(2, 0)) - h(1, 0), (y2 * h(2, 1)) - h(1, 1), 0.0, w);
    const Eigen::Vector4d j2(h(0, 0) - (x2 * h(2, 0)), h(0, 1) - (x2 * h(2, 1)), -w, 0.0);
    const double a = j1.squaredNorm();
    const double b = j1.dot(j2);
    const double c = j2.squaredNorm();
    const double determinant = (a * c) - (b * b);
    out(i) = determinant > 0.0
                 ? ((c * eps(0) * eps(0)) - (2.0 * b * eps(0) * eps(1)) + (a * eps(1) * eps(1))) /
                       determinant
                 : std::numeric_limits<double>::infinity();
  }
  return out;
}

Result<Eigen::Matrix3d> homography_dlt(const Eigen::Ref<const Rows2>& points1,
                                       const Eigen::Ref<const Rows2>& points2) {
  if (auto ok = epipolar::check_point_pairs(points1, points2, kSampleSize, "points"); !ok) {
    return std::unexpected(ok.error());
  }
  const auto homography = fit(points1, points2);
  if (!homography) {
    return fail(ErrorKind::kDegenerate,
                "H: the correspondences do not determine a unique solution");
  }
  return *homography;
}

Result<Eigen::VectorXd> sampson_homography(const Eigen::Matrix3d& homography,
                                           const Eigen::Ref<const Rows2>& points1,
                                           const Eigen::Ref<const Rows2>& points2) {
  if (auto ok = epipolar::check_point_pairs(points1, points2, 0, "points"); !ok) {
    return std::unexpected(ok.error());
  }
  if (!homography.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "homography: expected a finite (3, 3) matrix");
  }
  return sampson(homography, points1, points2);
}

HomographyEstimator::HomographyEstimator(Rows2 points1, Rows2 points2)
    : points1_(std::move(points1)), points2_(std::move(points2)) {}

std::uint32_t HomographyEstimator::size() const {
  return static_cast<std::uint32_t>(points1_.rows());
}

std::vector<HomographyEstimator::Model> HomographyEstimator::fit_minimal(
    std::span<const std::uint32_t> sample) const {
  return fit_nonminimal(sample);
}

std::vector<HomographyEstimator::Model> HomographyEstimator::fit_nonminimal(
    std::span<const std::uint32_t> subset) const {
  const auto homography =
      fit(epipolar::gather(points1_, subset), epipolar::gather(points2_, subset));
  return homography ? std::vector<Model>{*homography} : std::vector<Model>{};
}

Eigen::VectorXd HomographyEstimator::residuals(const Model& model) const {
  return sampson(model, points1_, points2_);
}

Result<ransac::Solution<Eigen::Matrix3d>> homography_ransac(const Eigen::Ref<const Rows2>& points1,
                                                            const Eigen::Ref<const Rows2>& points2,
                                                            const ransac::Options& options) {
  if (auto ok = epipolar::check_point_pairs(points1, points2, kSampleSize, "points"); !ok) {
    return std::unexpected(ok.error());
  }
  return ransac::estimate(HomographyEstimator(points1, points2), options);
}

}  // namespace slamlab::foundations::homography
