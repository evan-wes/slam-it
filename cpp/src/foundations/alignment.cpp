/// @file
/// @brief Implementation of the Umeyama alignment declared in
/// slam_it/foundations/alignment.hpp.
#include "slam_it/foundations/alignment.hpp"

#include <Eigen/Dense>

namespace slam_it::foundations::alignment {

Result<Eigen::Matrix4d> umeyama(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>& source,
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>& target,
    const Eigen::Ref<const Eigen::VectorXd>& weights, bool with_scale) {
  const Eigen::Index n = source.rows();
  if (target.rows() != n || weights.size() != n) {
    return fail(ErrorKind::kInvalidInput, "umeyama: source, target and weights differ in length");
  }
  if (!source.allFinite() || !target.allFinite() || !weights.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "source, target, weights: expected finite values");
  }
  if ((weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "weights: expected N non-negative values");
  }
  if ((weights.array() > 0.0).count() < 3) {
    return fail(ErrorKind::kDegenerate, "umeyama: fewer than three points with positive weight");
  }
  const Eigen::VectorXd w = weights / weights.sum();
  const Eigen::RowVector3d mu_x = w.transpose() * source;
  const Eigen::RowVector3d mu_y = w.transpose() * target;
  const Eigen::MatrixX3d xc = source.rowwise() - mu_x;
  const Eigen::MatrixX3d yc = target.rowwise() - mu_y;
  const Eigen::Matrix3d cov = (yc.array().colwise() * w.array()).matrix().transpose() * xc;
  const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(cov);
  const Eigen::Vector3d d = svd.singularValues();
  if (d(1) <= kDegeneracyRatio * d(0)) {
    return fail(ErrorKind::kDegenerate, "umeyama: points are collinear or coincident");
  }
  Eigen::Vector3d sign(1.0, 1.0, 1.0);
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
    sign(2) = -1.0;
  }
  const Eigen::Matrix3d rotation = svd.matrixU() * sign.asDiagonal() * svd.matrixV().transpose();
  const double scale = with_scale ? d.dot(sign) / w.dot(xc.rowwise().squaredNorm()) : 1.0;
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = scale * rotation;
  out.topRightCorner<3, 1>() = mu_y.transpose() - (scale * rotation * mu_x.transpose());
  return out;
}

}  // namespace slam_it::foundations::alignment
