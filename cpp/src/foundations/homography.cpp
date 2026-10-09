/// @file
/// @brief Implementation of the homographies declared in slam_it_all/foundations/homography.hpp.
#include "slam_it_all/foundations/homography.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "slam_it_all/foundations/so3.hpp"

namespace slam_it_all::foundations::homography {

std::optional<Eigen::Matrix3d> fit(const Eigen::Ref<const Rows2>& points1,
                                   const Eigen::Ref<const Rows2>& points2) {
  const auto t1 = correspondences::hartley_normalisation(points1);
  const auto t2 = correspondences::hartley_normalisation(points2);
  if (!t1 || !t2) {
    return std::nullopt;
  }
  const correspondences::Rows3 x1 = correspondences::homogeneous(points1) * t1->transpose();
  const correspondences::Rows3 x2 = correspondences::homogeneous(points2) * t2->transpose();
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
  const auto solution = correspondences::null_vector(system);
  if (!solution) {
    return std::nullopt;
  }
  const Eigen::Matrix3d normalised =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(solution->data());
  const Eigen::Vector3d singular = Eigen::JacobiSVD<Eigen::Matrix3d>(normalised).singularValues();
  if (!(singular(2) > kConditionTolerance * singular(0))) {
    return std::nullopt;  // numerically of rank below 3: no homography
  }
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
  if (auto ok = correspondences::check_point_pairs(points1, points2, kSampleSize, "points"); !ok) {
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
  if (auto ok = correspondences::check_point_pairs(points1, points2, 0, "points"); !ok) {
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
      fit(correspondences::gather(points1_, subset), correspondences::gather(points2_, subset));
  return homography ? std::vector<Model>{*homography} : std::vector<Model>{};
}

Eigen::VectorXd HomographyEstimator::residuals(const Model& model) const {
  return sampson(model, points1_, points2_);
}

namespace {

/// @brief The negated (row, col) minor of a symmetric 3x3 matrix.
/// @param m The matrix.
/// @param row The row removed.
/// @param col The column removed.
/// @return `-M_ij`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (row, col), as in M(row, col)
double opposite_of_minor(const Eigen::Matrix3d& m, int row, int col) {
  const int a = row == 0 ? 1 : 0;
  const int b = row == 2 ? 1 : 2;
  const int c = col == 0 ? 1 : 0;
  const int d = col == 2 ? 1 : 2;
  return (m(a, d) * m(b, c)) - (m(a, c) * m(b, d));
}

/// @brief `R = H (I - (2 / v) t* n^T)`, made proper and projected onto SO(3).
/// @param h The scaled homography.
/// @param t_star The candidate's `t*`.
/// @param n The candidate's normal.
/// @param v Malis and Vargas' `v`.
/// @return The rotation.
Eigen::Matrix3d rotation_from(const Eigen::Matrix3d& h, const Eigen::Vector3d& t_star,
                              const Eigen::Vector3d& n, double v) {
  const Eigen::Matrix3d rotation =
      h * (Eigen::Matrix3d::Identity() - ((2.0 / v) * t_star * n.transpose()));
  // The closed form assumes an exact H; for an estimated one the product is only near a
  // rotation, so it is projected onto SO(3).
  return so3::project_to_so3(rotation.determinant() > 0.0 ? rotation : Eigen::Matrix3d(-rotation));
}

/// @brief A non-negative square root (rounding can push a zero just below).
/// @param x The value.
/// @return `sqrt(max(x, 0))`.
double root(double x) { return std::sqrt(std::max(x, 0.0)); }

/// @brief +1 or -1 by sign, with +1 for zero.
/// @param x The value.
/// @return The sign.
double sign_of(double x) { return x >= 0.0 ? 1.0 : -1.0; }

/// @brief Malis and Vargas' two normal candidates `n_a`, `n_b` (unnormalised).
/// @param s `H^T H - I`.
/// @param k The index of the largest `|s_kk|`.
/// @return The pair.
std::pair<Eigen::Vector3d, Eigen::Vector3d> normal_candidates(const Eigen::Matrix3d& s, int k) {
  const double r00 = root(opposite_of_minor(s, 0, 0));
  const double r11 = root(opposite_of_minor(s, 1, 1));
  const double r22 = root(opposite_of_minor(s, 2, 2));
  const double e01 = sign_of(opposite_of_minor(s, 0, 1));
  const double e02 = sign_of(opposite_of_minor(s, 0, 2));
  const double e12 = sign_of(opposite_of_minor(s, 1, 2));
  if (k == 0) {
    return {{s(0, 0), s(0, 1) + r22, s(0, 2) + (e12 * r11)},
            {s(0, 0), s(0, 1) - r22, s(0, 2) - (e12 * r11)}};
  }
  if (k == 1) {
    return {{s(0, 1) + r22, s(1, 1), s(1, 2) - (e02 * r00)},
            {s(0, 1) - r22, s(1, 1), s(1, 2) + (e02 * r00)}};
  }
  return {{s(0, 2) + (e01 * r11), s(1, 2) + r00, s(2, 2)},
          {s(0, 2) - (e01 * r11), s(1, 2) - r00, s(2, 2)}};
}

/// @brief Malis and Vargas' scalars.
struct MotionScalars {
  /// @brief nu = 2 sqrt(1 + tr S - M_11 - M_22 - M_33).
  double nu;
  /// @brief rho = sqrt(2 + tr S + nu).
  double rho;
  /// @brief |t_e| = sqrt(2 + tr S - nu).
  double t_norm;
};

/// @brief Malis and Vargas' scalars, each radicand clamped at zero.
/// @param s `H^T H - I`.
/// @return nu, rho and |t_e|.
MotionScalars motion_scalars(const Eigen::Matrix3d& s) {
  const double trace = s.trace();
  const double nu = 2.0 * root(1.0 + trace - opposite_of_minor(s, 0, 0) -
                               opposite_of_minor(s, 1, 1) - opposite_of_minor(s, 2, 2));
  return {.nu = nu, .rho = root(2.0 + trace + nu), .t_norm = root(2.0 + trace - nu)};
}

}  // namespace

Result<PlanarMotions> decompose_homography(const Eigen::Matrix3d& homography) {
  if (!homography.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "homography: expected a finite (3, 3) matrix");
  }
  const Eigen::Vector3d singular = Eigen::JacobiSVD<Eigen::Matrix3d>(homography).singularValues();
  if (!(singular(1) > correspondences::kRankTolerance * singular(0))) {
    return fail(ErrorKind::kDegenerate, "homography: rank below 2");
  }
  const Eigen::Matrix3d h = homography / singular(1);
  const Eigen::Matrix3d s = (h.transpose() * h) - Eigen::Matrix3d::Identity();
  PlanarMotions out;
  if (s.cwiseAbs().maxCoeff() < kPureRotationTolerance) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    // An estimated H is only near a rotation: project it, as for the four candidates.
    pose.topLeftCorner<3, 3>() =
        so3::project_to_so3(h.determinant() > 0.0 ? h : Eigen::Matrix3d(-h));
    out.poses.push_back(pose);
    out.normals.emplace_back(Eigen::Vector3d::Zero());
    return out;
  }
  // Names follow Malis and Vargas: S, the plane normals n'_a and n'_b, epsilon = sign(s_kk),
  // and t*_a, t*_b the translations before rotation.
  int k = 0;
  s.diagonal().cwiseAbs().maxCoeff(&k);
  auto [na, nb] = normal_candidates(s, k);
  na.normalize();
  nb.normalize();
  const MotionScalars scalars = motion_scalars(s);
  const double epsilon = sign_of(s(k, k));
  const double t_norm = scalars.t_norm;
  const Eigen::Vector3d ta_star = 0.5 * t_norm * ((epsilon * scalars.rho * nb) - (t_norm * na));
  const Eigen::Vector3d tb_star = 0.5 * t_norm * ((epsilon * scalars.rho * na) - (t_norm * nb));
  for (const auto& [t_star, n] : {std::pair{ta_star, na}, std::pair{tb_star, nb}}) {
    const Eigen::Matrix3d rotation = rotation_from(h, t_star, n, scalars.nu);
    const Eigen::Vector3d t = rotation * t_star;
    for (const double sign : {1.0, -1.0}) {
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      pose.topLeftCorner<3, 3>() = rotation;
      pose.topRightCorner<3, 1>() = sign * t;
      out.poses.push_back(pose);
      out.normals.emplace_back(sign * n);
    }
  }
  return out;
}

Result<ransac::Solution<Eigen::Matrix3d>> homography_ransac(const Eigen::Ref<const Rows2>& points1,
                                                            const Eigen::Ref<const Rows2>& points2,
                                                            const ransac::Options& options) {
  if (auto ok = correspondences::check_point_pairs(points1, points2, kSampleSize, "points"); !ok) {
    return std::unexpected(ok.error());
  }
  return ransac::estimate(HomographyEstimator(points1, points2), options);
}

}  // namespace slam_it_all::foundations::homography
