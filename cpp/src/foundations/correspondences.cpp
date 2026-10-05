/// @file
/// @brief Implementation of the correspondence helpers declared in
///        slamlab/foundations/correspondences.hpp.
#include "slamlab/foundations/correspondences.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace slamlab::foundations::correspondences {

Rows3 homogeneous(const Eigen::Ref<const Rows2>& points) {
  Rows3 out(points.rows(), 3);
  out.leftCols<2>() = points;
  out.col(2).setOnes();
  return out;
}

Rows3 normalised_plane(const Eigen::Ref<const Rows3>& bearings) {
  return (bearings.array().colwise() / bearings.col(2).array()).matrix();
}

Rows2 normalised_xy(const Eigen::Ref<const Rows3>& bearings) {
  return (bearings.leftCols<2>().array().colwise() / bearings.col(2).array()).matrix();
}

std::optional<Eigen::Matrix3d> hartley_normalisation(const Eigen::Ref<const Rows2>& points) {
  const Eigen::RowVector2d centroid = points.colwise().mean();
  const double spread = (points.rowwise() - centroid).rowwise().norm().mean();
  if (!(spread > 0.0)) {
    return std::nullopt;
  }
  const double scale = std::numbers::sqrt2 / spread;
  Eigen::Matrix3d t;
  t << scale, 0.0, -scale * centroid(0), 0.0, scale, -scale * centroid(1), 0.0, 0.0, 1.0;
  return t;
}

std::optional<Eigen::Matrix<double, 9, 1>> null_vector(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 9>>& system) {
  // A has at least eight rows; pad an eight-row system to nine so V is the full 9x9 basis.
  Eigen::Matrix<double, Eigen::Dynamic, 9> padded = system;
  if (padded.rows() < 9) {
    padded.conservativeResize(9, Eigen::NoChange);
    padded.bottomRows(9 - system.rows()).setZero();
  }
  const Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>, Eigen::ComputeFullV> svd(padded);
  const Eigen::VectorXd& singular = svd.singularValues();
  if (!(singular(7) > kRankTolerance * singular(0))) {
    return std::nullopt;
  }
  return svd.matrixV().col(8);
}

// check_bearing's message spells the tolerance out ("1e-09", as Python prints it), so the
// cores' messages match; this keeps the two in step.
static_assert(kBearingTolerance == 1e-9, "update check_bearing's message");

Result<void> check_bearing(const Eigen::Ref<const Rows3>& bearings, Eigen::Index row,
                           const char* name) {
  if (!bearings.row(row).allFinite() ||
      std::abs(bearings.row(row).norm() - 1.0) > kBearingTolerance) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": expected unit vectors within 1e-09");
  }
  return {};
}

Result<void> check_bearing_pairs(const Eigen::Ref<const Rows3>& bearings1,
                                 const Eigen::Ref<const Rows3>& bearings2, Eigen::Index minimum,
                                 bool positive_z) {
  if (bearings1.rows() != bearings2.rows()) {
    return fail(ErrorKind::kInvalidInput, "bearings: expected equal (N, 3) shapes");
  }
  if (bearings1.rows() < minimum) {
    return fail(ErrorKind::kInvalidInput, "bearings: need at least " + std::to_string(minimum) +
                                              " pairs, got " + std::to_string(bearings1.rows()));
  }
  for (Eigen::Index i = 0; i < bearings1.rows(); ++i) {
    for (const auto& [bearings, name] :
         {std::pair{&bearings1, "bearings1"}, std::pair{&bearings2, "bearings2"}}) {
      if (auto ok = check_bearing(*bearings, i, name); !ok) {
        return ok;
      }
      if (positive_z && !((*bearings)(i, 2) > 0.0)) {
        return fail(ErrorKind::kInvalidInput, "bearings: expected positive z (a pinhole view)");
      }
    }
  }
  return {};
}

Result<void> check_correspondences(const Eigen::Ref<const Rows3>& points,
                                   const Eigen::Ref<const Rows3>& bearings, Eigen::Index minimum,
                                   bool positive_z) {
  if (points.rows() != bearings.rows()) {
    return fail(ErrorKind::kInvalidInput, "points, bearings: expected equal (N, 3) shapes");
  }
  if (points.rows() < minimum) {
    return fail(ErrorKind::kInvalidInput, "points: need at least " + std::to_string(minimum) +
                                              ", got " + std::to_string(points.rows()));
  }
  if (!points.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "points: expected finite values");
  }
  for (Eigen::Index i = 0; i < bearings.rows(); ++i) {
    if (auto ok = check_bearing(bearings, i, "bearings"); !ok) {
      return ok;
    }
    if (positive_z && !(bearings(i, 2) > 0.0)) {
      return fail(ErrorKind::kInvalidInput, "bearings: expected positive z (a pinhole view)");
    }
  }
  return {};
}

Result<void> check_point_pairs(const Eigen::Ref<const Rows2>& points1,
                               const Eigen::Ref<const Rows2>& points2, Eigen::Index minimum,
                               const char* name) {
  if (points1.rows() != points2.rows()) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected equal (N, 2) shapes");
  }
  if (points1.rows() < minimum) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": need at least " +
                                              std::to_string(minimum) + " pairs, got " +
                                              std::to_string(points1.rows()));
  }
  if (!points1.allFinite() || !points2.allFinite()) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected finite values");
  }
  return {};
}

}  // namespace slamlab::foundations::correspondences
