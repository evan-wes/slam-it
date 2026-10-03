/// @file
/// @brief Implementation of the epipolar geometry declared in slamlab/foundations/epipolar.hpp.
#include "slamlab/foundations/epipolar.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::epipolar {

using correspondences::check_bearing_pairs;
using correspondences::check_point_pairs;
using correspondences::gather;
using correspondences::hartley_normalisation;
using correspondences::homogeneous;
using correspondences::kRankTolerance;
using correspondences::normalised_plane;
using correspondences::null_vector;

namespace {

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

const char* name(MinimalSolver solver) {
  switch (solver) {
    case MinimalSolver::kNister:
      return "nister";
    case MinimalSolver::kStewenius:
      return "stewenius";
    case MinimalSolver::k8pt:
      return "8pt";
    case MinimalSolver::kCount:
      break;
  }
  return "unknown";
}

EssentialEstimator::EssentialEstimator(Rows3 bearings1, Rows3 bearings2, MinimalSolver minimal)
    : bearings1_(std::move(bearings1)),
      bearings2_(std::move(bearings2)),
      x1_(normalised_plane(bearings1_)),
      x2_(normalised_plane(bearings2_)),
      minimal_(minimal) {}

std::uint32_t EssentialEstimator::sample_size() const {
  return minimal_ == MinimalSolver::k8pt ? kSampleSize : fivepoint::kSampleSize;
}

std::uint32_t EssentialEstimator::size() const {
  return static_cast<std::uint32_t>(bearings1_.rows());
}

std::vector<EssentialEstimator::Model> EssentialEstimator::fit_minimal(
    std::span<const std::uint32_t> sample) const {
  if (minimal_ == MinimalSolver::k8pt) {
    return fit_nonminimal(sample);
  }
  const auto solver = minimal_ == MinimalSolver::kNister ? fivepoint::Solver::kNister
                                                         : fivepoint::Solver::kStewenius;
  return fivepoint::solve(gather(bearings1_, sample), gather(bearings2_, sample), solver);
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

Eigen::Matrix3d essential_of(const Motion& motion) {
  return so3::hat(motion.translation) * motion.rotation;
}

Eigen::Matrix<double, 3, 2> tangent_basis(const Eigen::Vector3d& direction) {
  Eigen::Index k = 0;
  direction.cwiseAbs().minCoeff(&k);
  const Eigen::Vector3d first = direction.cross(Eigen::Vector3d::Unit(k)).normalized();
  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = first;
  basis.col(1) = direction.cross(first);
  return basis;
}

namespace {

/// @brief One match's Sampson terms under E: the epipolar lines, `c = x2^T E x1` and `g`, the
///        sum of both lines' first two coordinates squared; the residual is `c / sqrt(g)`.
struct SampsonTerms {
  /// @brief `E x1`.
  Eigen::Vector3d line2;
  /// @brief `E^T x2`.
  Eigen::Vector3d line1;
  /// @brief The algebraic error.
  double c;
  /// @brief The squared gradient norm.
  double g;
};

/// @brief One match's Sampson terms (`SampsonProblem._terms` in Python).
/// @param e E.
/// @param x1 The point in image 1, z = 1.
/// @param x2 The point in image 2, z = 1.
/// @return The terms.
SampsonTerms sampson_terms(const Eigen::Matrix3d& e, const Eigen::Vector3d& x1,
                           const Eigen::Vector3d& x2) {
  const Eigen::Vector3d line2 = e * x1;
  const Eigen::Vector3d line1 = e.transpose() * x2;
  return {.line2 = line2,
          .line1 = line1,
          .c = x2.dot(line2),
          .g = line2.head<2>().squaredNorm() + line1.head<2>().squaredNorm()};
}

/// @brief Least squares on the signed Sampson distance of E's inliers, over E's manifold
///        (see `slamlab.foundations.epipolar.SampsonProblem`).
class SampsonProblem {
 public:
  /// @brief The state: R and a unit t.
  using State = Motion;

  /// @brief Hold the inliers' normalised image points.
  /// @param x1 Points in image 1 (M x 3), z = 1.
  /// @param x2 Points in image 2 (M x 3), z = 1.
  SampsonProblem(Rows3 x1, Rows3 x2) : x1_(std::move(x1)), x2_(std::move(x2)) {}

  /// @brief Each match's signed Sampson distance.
  /// @param state The motion.
  /// @return The residuals.
  [[nodiscard]] Eigen::VectorXd residuals(const State& state) const {
    const Eigen::Matrix3d e = essential_of(state);
    Eigen::VectorXd out(x1_.rows());
    for (Eigen::Index i = 0; i < x1_.rows(); ++i) {
      const SampsonTerms terms = sampson_terms(e, x1_.row(i).transpose(), x2_.row(i).transpose());
      out(i) = terms.c / std::sqrt(terms.g);
    }
    return out;
  }

  /// @brief Each block's squared residual.
  /// @param state The motion.
  /// @return The squared residuals.
  [[nodiscard]] Eigen::VectorXd squared_norms(const State& state) const {
    return residuals(state).array().square().matrix();
  }

  /// @brief The weighted normal equations over (dtheta, dt).
  /// @param state The motion.
  /// @param weights The blocks' weights.
  /// @return The dense system.
  [[nodiscard]] nls::DenseSystem linearize(const State& state,
                                           const Eigen::VectorXd& weights) const {
    const Eigen::Matrix3d e = essential_of(state);
    const Eigen::Matrix3d t_hat = so3::hat(state.translation);
    const Eigen::Matrix<double, 3, 2> basis = tangent_basis(state.translation);
    // dE along each of (dtheta, dt): [t]x R [e_k]x, then [b_j]x R.
    std::array<Eigen::Matrix3d, 5> directions{};
    for (std::size_t k = 0; k < 3; ++k) {
      directions.at(k) =
          t_hat * state.rotation * so3::hat(Eigen::Vector3d::Unit(static_cast<Eigen::Index>(k)));
    }
    for (std::size_t j = 0; j < 2; ++j) {
      directions.at(3 + j) = so3::hat(basis.col(static_cast<Eigen::Index>(j))) * state.rotation;
    }
    Eigen::MatrixXd jacobian(x1_.rows(), 5);
    Eigen::VectorXd residual(x1_.rows());
    for (Eigen::Index i = 0; i < x1_.rows(); ++i) {
      const Eigen::Vector3d x1 = x1_.row(i).transpose();
      const Eigen::Vector3d x2 = x2_.row(i).transpose();
      const auto [line2, line1, c, g] = sampson_terms(e, x1, x2);
      const double root = std::sqrt(g);
      // dc/dE = x2 x1^T; dg/dE = 2 sum_{i=0,1} (line2_i e_i x1^T + line1_i x2 e_i^T).
      Eigen::Matrix3d dg = Eigen::Matrix3d::Zero();
      for (int k = 0; k < 2; ++k) {
        dg.row(k) += 2.0 * line2(k) * x1.transpose();
        dg.col(k) += 2.0 * line1(k) * x2;
      }
      const Eigen::Matrix3d dr = ((x2 * x1.transpose()) / root) - ((c / (2.0 * g * root)) * dg);
      for (std::size_t k = 0; k < directions.size(); ++k) {
        jacobian(i, static_cast<Eigen::Index>(k)) = dr.cwiseProduct(directions.at(k)).sum();
      }
      residual(i) = c / root;
    }
    return *nls::DenseSystem::build(jacobian, residual, weights);
  }

  /// @brief Step on E's manifold: R exp(dtheta), and t along the sphere.
  /// @param state The motion.
  /// @param delta (dtheta, dt).
  /// @return The moved motion.
  [[nodiscard]] static State retract(const State& state, const Eigen::VectorXd& delta) {
    const Eigen::Vector3d moved =
        state.translation + (tangent_basis(state.translation) * delta.tail<2>());
    return {.rotation = state.rotation * so3::exp(delta.head<3>()),
            .translation = moved.normalized()};
  }

 private:
  /// @brief Points in image 1.
  Rows3 x1_;
  /// @brief Points in image 2.
  Rows3 x2_;
};

}  // namespace

Motion start_motion(const Eigen::Matrix3d& essential) {
  // decompose_essential succeeds on any matrix of rank 2, which every caller has checked.
  const std::vector<Eigen::Matrix4d> candidates = *decompose_essential(essential);
  const auto best = std::ranges::max_element(candidates, {}, [](const Eigen::Matrix4d& pose) {
    return pose.topLeftCorner<3, 3>().trace();
  });
  Eigen::Vector3d translation = best->topRightCorner<3, 1>();
  Eigen::Index k = 0;
  translation.cwiseAbs().maxCoeff(&k);
  if (translation(k) < 0.0) {
    translation = -translation;
  }
  return {.rotation = best->topLeftCorner<3, 3>(), .translation = translation};
}

solve::Solution<Eigen::Matrix3d, nls::Summary> refine(const Eigen::Matrix3d& essential,
                                                      const Eigen::Ref<const Rows3>& bearings1,
                                                      const Eigen::Ref<const Rows3>& bearings2,
                                                      double scale) {
  const SampsonProblem problem(normalised_plane(bearings1), normalised_plane(bearings2));
  nls::Options options;
  options.loss = {.kind = nls::LossKind::kCauchy, .scale = scale};
  options.max_iterations = kRefineIterations;
  // With the strict flag off the solve never fails: it returns its best state.
  auto solved = *nls::solve(problem, start_motion(essential), options);
  return {.state = essential_of(solved.state), .summary = std::move(solved.summary)};
}

Result<solve::Solution<Eigen::Matrix3d, nls::Summary>> refine_essential(
    const Eigen::Matrix3d& essential, const Eigen::Ref<const Rows3>& bearings1,
    const Eigen::Ref<const Rows3>& bearings2, double scale) {
  if (!essential.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "essential: expected a finite (3, 3) matrix");
  }
  if (auto ok = check_bearing_pairs(bearings1, bearings2, fivepoint::kSampleSize); !ok) {
    return std::unexpected(ok.error());
  }
  if (!std::isfinite(scale) || scale <= 0.0) {
    return fail_setting("scale", "must be positive and finite", scale);
  }
  const Eigen::Vector3d singular = Eigen::JacobiSVD<Eigen::Matrix3d>(essential).singularValues();
  if (!(singular(1) > kRankTolerance * essential.norm())) {
    return fail(ErrorKind::kDegenerate, "essential: rank below 2, no translation direction");
  }
  return refine(essential, bearings1, bearings2, scale);
}

Result<ransac::RefinedSolution<Eigen::Matrix3d>> essential_ransac(
    const Eigen::Ref<const Rows3>& bearings1, const Eigen::Ref<const Rows3>& bearings2,
    const ransac::Options& options, MinimalSolver minimal, bool refine) {
  if (minimal != MinimalSolver::kNister && minimal != MinimalSolver::kStewenius &&
      minimal != MinimalSolver::k8pt) {
    return fail(ErrorKind::kInvalidInput, "minimal_solver: unknown code");
  }
  const Eigen::Index minimum =
      minimal == MinimalSolver::k8pt ? kSampleSize : fivepoint::kSampleSize;
  if (auto ok = check_bearing_pairs(bearings1, bearings2, minimum); !ok) {
    return std::unexpected(ok.error());
  }
  const Rows3 all1 = bearings1;
  const Rows3 all2 = bearings2;
  return ransac::estimate_refined(
      EssentialEstimator(bearings1, bearings2, minimal), options, refine,
      [&](const Eigen::Matrix3d& model, const ransac::Indices& inliers) {
        return epipolar::refine(model, gather(all1, inliers), gather(all2, inliers),
                                options.threshold);
      },
      fivepoint::kSampleSize);
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
