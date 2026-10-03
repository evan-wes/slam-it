/// @file
/// @brief Implementation of the epipolar geometry declared in slamlab/foundations/epipolar.hpp.
#include "slamlab/foundations/epipolar.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

namespace slamlab::foundations::epipolar {

namespace {

/// @brief Unit bearings must have norm 1 within this (as in triangulation).
constexpr double kBearingTolerance = 1e-9;

/// @brief Stack `x2 (x) x1` per correspondence: the rows of `A vec(M) = 0` (vec row-major).
/// @param x1 Homogeneous points in image 1 (N x 3).
/// @param x2 Homogeneous points in image 2 (N x 3).
/// @return `A` (N x 9).
Eigen::Matrix<double, Eigen::Dynamic, 9> epipolar_system(const Eigen::Ref<const Rows3>& x1,
                                                         const Eigen::Ref<const Rows3>& x2) {
  Eigen::Matrix<double, Eigen::Dynamic, 9> system(x1.rows(), 9);
  for (Eigen::Index i = 0; i < x1.rows(); ++i) {
    for (Eigen::Index r = 0; r < 3; ++r) {
      system.block<1, 3>(i, 3 * r) = x2(i, r) * x1.row(i);
    }
  }
  return system;
}

/// @brief Reshape a row-major vec back into a 3x3 matrix.
/// @param v The 9 entries, row by row.
/// @return The matrix.
Eigen::Matrix3d unvec(const Eigen::Matrix<double, 9, 1>& v) {
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(v.data());
}

}  // namespace

Rows3 homogeneous(const Eigen::Ref<const Rows2>& points) {
  Rows3 out(points.rows(), 3);
  out.leftCols<2>() = points;
  out.col(2).setOnes();
  return out;
}

Rows3 normalised_plane(const Eigen::Ref<const Rows3>& bearings) {
  return (bearings.array().colwise() / bearings.col(2).array()).matrix();
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

Eigen::VectorXd sampson(const Eigen::Matrix3d& matrix, const Eigen::Ref<const Rows3>& x1,
                        const Eigen::Ref<const Rows3>& x2) {
  Eigen::VectorXd out(x1.rows());
  for (Eigen::Index i = 0; i < x1.rows(); ++i) {
    const Eigen::Vector3d line2 = matrix * x1.row(i).transpose();
    const Eigen::Vector3d line1 = matrix.transpose() * x2.row(i).transpose();
    const double algebraic = x2.row(i).dot(line2);
    const double gradient = line2.head<2>().squaredNorm() + line1.head<2>().squaredNorm();
    out(i) =
        gradient > 0.0 ? algebraic * algebraic / gradient : std::numeric_limits<double>::infinity();
  }
  return out;
}

namespace {

/// @brief The 8-point system solved on Hartley-normalised points.
struct Normalised {
  /// @brief The solution in normalised coordinates.
  Eigen::Matrix3d raw;
  /// @brief Image 1's similarity.
  Eigen::Matrix3d t1;
  /// @brief Image 2's similarity.
  Eigen::Matrix3d t2;
};

/// @brief Solve the 8-point system on Hartley-normalised 2D points.
/// @param points1 Points in image 1 (N x 2).
/// @param points2 Points in image 2 (N x 2).
/// @return The solution and the similarities, or nothing if every point coincides or the
///         solution is not unique.
std::optional<Normalised> normalised_8pt(const Eigen::Ref<const Rows2>& points1,
                                         const Eigen::Ref<const Rows2>& points2) {
  const auto t1 = hartley_normalisation(points1);
  const auto t2 = hartley_normalisation(points2);
  if (!t1 || !t2) {
    return std::nullopt;
  }
  const Rows3 x1 = homogeneous(points1) * t1->transpose();
  const Rows3 x2 = homogeneous(points2) * t2->transpose();
  const auto solution = null_vector(epipolar_system(x1, x2));
  if (!solution) {
    return std::nullopt;
  }
  return Normalised{.raw = unvec(*solution), .t1 = *t1, .t2 = *t2};
}

}  // namespace

std::optional<Eigen::Matrix3d> fit_essential(const Eigen::Ref<const Rows3>& bearings1,
                                             const Eigen::Ref<const Rows3>& bearings2) {
  const auto solved = normalised_8pt(normalised_plane(bearings1).leftCols<2>(),
                                     normalised_plane(bearings2).leftCols<2>());
  if (!solved) {
    return std::nullopt;
  }
  const auto& [raw, t1, t2] = *solved;
  // Project after mapping back: the normalisation does not preserve the essential structure.
  const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(
      t2.transpose() * raw * t1);
  return svd.matrixU() * Eigen::Vector3d(1.0, 1.0, 0.0).asDiagonal() * svd.matrixV().transpose();
}

std::optional<Eigen::Matrix3d> fit_fundamental(const Eigen::Ref<const Rows2>& pixels1,
                                               const Eigen::Ref<const Rows2>& pixels2) {
  const auto solved = normalised_8pt(pixels1, pixels2);
  if (!solved) {
    return std::nullopt;
  }
  const auto& [raw, t1, t2] = *solved;
  const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(raw);
  const Eigen::Vector3d singular = svd.singularValues();
  const Eigen::Matrix3d rank2 = svd.matrixU() *
                                Eigen::Vector3d(singular(0), singular(1), 0.0).asDiagonal() *
                                svd.matrixV().transpose();
  const Eigen::Matrix3d fundamental = t2.transpose() * rank2 * t1;
  return fundamental / fundamental.norm();
}

Result<void> check_bearing_pairs(const Eigen::Ref<const Rows3>& bearings1,
                                 const Eigen::Ref<const Rows3>& bearings2, Eigen::Index minimum) {
  if (bearings1.rows() != bearings2.rows()) {
    return fail(ErrorKind::kInvalidInput, "bearings: expected equal (N, 3) shapes");
  }
  if (bearings1.rows() < minimum) {
    return fail(ErrorKind::kInvalidInput, "bearings: need at least " + std::to_string(minimum) +
                                              " pairs, got " + std::to_string(bearings1.rows()));
  }
  for (const auto* bearings : {&bearings1, &bearings2}) {
    if (!bearings->allFinite() ||
        ((bearings->rowwise().norm().array() - 1.0).abs() > kBearingTolerance).any()) {
      return fail(ErrorKind::kInvalidInput, "bearings: expected unit vectors within 1e-09");
    }
    if ((bearings->col(2).array() <= 0.0).any()) {
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

Result<Eigen::Matrix3d> essential_8pt(const Eigen::Ref<const Rows3>& bearings1,
                                      const Eigen::Ref<const Rows3>& bearings2) {
  if (auto ok = check_bearing_pairs(bearings1, bearings2, kSampleSize); !ok) {
    return std::unexpected(ok.error());
  }
  const auto essential = fit_essential(bearings1, bearings2);
  if (!essential) {
    return fail(ErrorKind::kDegenerate,
                "E: the correspondences do not determine a unique solution");
  }
  return *essential;
}

Result<Eigen::Matrix3d> fundamental_8pt(const Eigen::Ref<const Rows2>& pixels1,
                                        const Eigen::Ref<const Rows2>& pixels2) {
  if (auto ok = check_point_pairs(pixels1, pixels2, kSampleSize, "pixels"); !ok) {
    return std::unexpected(ok.error());
  }
  const auto fundamental = fit_fundamental(pixels1, pixels2);
  if (!fundamental) {
    return fail(ErrorKind::kDegenerate,
                "F: the correspondences do not determine a unique solution");
  }
  return *fundamental;
}

Result<Eigen::VectorXd> sampson_essential(const Eigen::Matrix3d& essential,
                                          const Eigen::Ref<const Rows3>& bearings1,
                                          const Eigen::Ref<const Rows3>& bearings2) {
  if (auto ok = check_bearing_pairs(bearings1, bearings2, 0); !ok) {
    return std::unexpected(ok.error());
  }
  return sampson(essential, normalised_plane(bearings1), normalised_plane(bearings2));
}

Result<Eigen::VectorXd> sampson_fundamental(const Eigen::Matrix3d& fundamental,
                                            const Eigen::Ref<const Rows2>& pixels1,
                                            const Eigen::Ref<const Rows2>& pixels2) {
  if (auto ok = check_point_pairs(pixels1, pixels2, 0, "pixels"); !ok) {
    return std::unexpected(ok.error());
  }
  return sampson(fundamental, homogeneous(pixels1), homogeneous(pixels2));
}

EssentialEstimator::EssentialEstimator(Rows3 bearings1, Rows3 bearings2)
    : bearings1_(std::move(bearings1)),
      bearings2_(std::move(bearings2)),
      x1_(normalised_plane(bearings1_)),
      x2_(normalised_plane(bearings2_)) {}

std::uint32_t EssentialEstimator::size() const {
  return static_cast<std::uint32_t>(bearings1_.rows());
}

std::vector<EssentialEstimator::Model> EssentialEstimator::fit_minimal(
    std::span<const std::uint32_t> sample) const {
  return fit_nonminimal(sample);
}

std::vector<EssentialEstimator::Model> EssentialEstimator::fit_nonminimal(
    std::span<const std::uint32_t> subset) const {
  const auto essential = fit_essential(gather(bearings1_, subset), gather(bearings2_, subset));
  return essential ? std::vector<Model>{*essential} : std::vector<Model>{};
}

Eigen::VectorXd EssentialEstimator::residuals(const Model& model) const {
  return sampson(model, x1_, x2_);
}

FundamentalEstimator::FundamentalEstimator(Rows2 pixels1, Rows2 pixels2)
    : pixels1_(std::move(pixels1)),
      pixels2_(std::move(pixels2)),
      x1_(homogeneous(pixels1_)),
      x2_(homogeneous(pixels2_)) {}

std::uint32_t FundamentalEstimator::size() const {
  return static_cast<std::uint32_t>(pixels1_.rows());
}

std::vector<FundamentalEstimator::Model> FundamentalEstimator::fit_minimal(
    std::span<const std::uint32_t> sample) const {
  return fit_nonminimal(sample);
}

std::vector<FundamentalEstimator::Model> FundamentalEstimator::fit_nonminimal(
    std::span<const std::uint32_t> subset) const {
  const auto fundamental = fit_fundamental(gather(pixels1_, subset), gather(pixels2_, subset));
  return fundamental ? std::vector<Model>{*fundamental} : std::vector<Model>{};
}

Eigen::VectorXd FundamentalEstimator::residuals(const Model& model) const {
  return sampson(model, x1_, x2_);
}

Result<std::vector<Eigen::Matrix4d>> decompose_essential(const Eigen::Matrix3d& essential) {
  if (!essential.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "essential: expected a finite (3, 3) matrix");
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(essential);
  const Eigen::Vector3d singular = svd.singularValues();
  if (!(singular(1) > kRankTolerance * singular(0))) {
    return fail(ErrorKind::kDegenerate, "essential: rank below 2, no translation direction");
  }
  // E = U diag(1, 1, 0) V^T holds with either sign of U or V, so make both proper rotations.
  Eigen::Matrix3d u = svd.matrixU();
  Eigen::Matrix3d v = svd.matrixV();
  if (u.determinant() < 0.0) {
    u = -u;
  }
  if (v.determinant() < 0.0) {
    v = -v;
  }
  Eigen::Matrix3d w;
  w << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  const Eigen::Vector3d t = u.col(2);
  std::vector<Eigen::Matrix4d> poses;
  for (const Eigen::Matrix3d& rotation : {Eigen::Matrix3d(u * w * v.transpose()),
                                          Eigen::Matrix3d(u * w.transpose() * v.transpose())}) {
    for (const double sign : {1.0, -1.0}) {
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      pose.topLeftCorner<3, 3>() = rotation;
      pose.topRightCorner<3, 1>() = sign * t;
      poses.push_back(pose);
    }
  }
  return poses;
}

Result<ransac::Solution<Eigen::Matrix3d>> essential_ransac(const Eigen::Ref<const Rows3>& bearings1,
                                                           const Eigen::Ref<const Rows3>& bearings2,
                                                           const ransac::Options& options) {
  if (auto ok = check_bearing_pairs(bearings1, bearings2, kSampleSize); !ok) {
    return std::unexpected(ok.error());
  }
  return ransac::estimate(EssentialEstimator(bearings1, bearings2), options);
}

Result<ransac::Solution<Eigen::Matrix3d>> fundamental_ransac(const Eigen::Ref<const Rows2>& pixels1,
                                                             const Eigen::Ref<const Rows2>& pixels2,
                                                             const ransac::Options& options) {
  if (auto ok = check_point_pairs(pixels1, pixels2, kSampleSize, "pixels"); !ok) {
    return std::unexpected(ok.error());
  }
  return ransac::estimate(FundamentalEstimator(pixels1, pixels2), options);
}

}  // namespace slamlab::foundations::epipolar
