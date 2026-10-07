/// @file
/// @brief Absolute pose: EPnP, the pose refinement and PnP RANSAC (see
///        `slamlab/foundations/pnp.hpp`).
#include "slamlab/foundations/pnp.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "slamlab/foundations/alignment.hpp"
#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::pnp {

namespace {

/// @brief The control points' six pairs, in the rows of EPnP's distance system.
constexpr std::array<std::pair<int, int>, 6> kPairs = {
    {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}};
/// @brief The ten products `b_k b_l`, in the columns of EPnP's distance system.
constexpr std::array<std::pair<int, int>, 10> kMonomials = {
    {{0, 0}, {0, 1}, {1, 1}, {0, 2}, {1, 2}, {2, 2}, {0, 3}, {1, 3}, {2, 3}, {3, 3}}};
/// @brief One linearisation: the columns of the distance system it keeps.
struct Approximation {
  /// @brief The columns, the first `count` of them used.
  std::array<Eigen::Index, 5> columns;
  /// @brief How many columns.
  Eigen::Index count;
};
/// @brief The columns each linearisation keeps (`_APPROXIMATIONS` in Python).
constexpr std::array<Approximation, 3> kApproximations = {{
    {.columns = {0, 1, 3, 6, 0}, .count = 4},
    {.columns = {0, 1, 2, 0, 0}, .count = 3},
    {.columns = {0, 1, 2, 3, 4}, .count = 5},
}};

/// @brief EPnP's distance system: `L (b_k b_l) = rho` keeps the control-point distances.
struct DistanceSystem {
  /// @brief `L`, 6 x 10.
  Eigen::Matrix<double, 6, 10> system;
  /// @brief `rho`, the world control points' squared distances.
  Eigen::Matrix<double, 6, 1> rho;
};

/// @brief Flip each column so its largest-magnitude entry is positive (`_deterministic_sign`).
/// @param vectors The eigenvectors, as columns.
/// @return The signed eigenvectors.
Eigen::MatrixXd deterministic_sign(Eigen::MatrixXd vectors) {
  for (Eigen::Index c = 0; c < vectors.cols(); ++c) {
    Eigen::Index largest = 0;
    vectors.col(c).cwiseAbs().maxCoeff(&largest);
    if (vectors(largest, c) < 0.0) {
      vectors.col(c) = -vectors.col(c);
    }
  }
  return vectors;
}

/// @brief EPnP's four world control points (`_control_points`).
/// @param points The world points.
/// @return The control points as rows (4 x 3), or nothing if the points are planar.
std::optional<Eigen::Matrix<double, 4, 3>> control_points(const Rows3& points) {
  const Eigen::RowVector3d centroid = points.colwise().mean();
  const Rows3 centred = points.rowwise() - centroid;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(centred.transpose() * centred);
  const Eigen::Vector3d& values = eigen.eigenvalues();
  if (!(values(0) > kPlanarTolerance * values(2))) {
    return std::nullopt;
  }
  const Eigen::MatrixXd axes = deterministic_sign(eigen.eigenvectors().rowwise().reverse());
  Eigen::Matrix<double, 4, 3> out;
  out.row(0) = centroid;
  for (Eigen::Index k = 0; k < 3; ++k) {
    const double spread = std::sqrt(values(2 - k) / static_cast<double>(points.rows()));
    out.row(k + 1) = centroid + (spread * axes.col(k).transpose());
  }
  return out;
}

/// @brief EPnP's distance system for a null-space basis (`_distance_system`).
/// @param basis The null-space vectors as rows of 12 (4 x 12).
/// @param controls The world control points (4 x 3).
/// @return `L` and `rho`.
DistanceSystem distance_system(const Eigen::Matrix<double, 4, 12>& basis,
                               const Eigen::Matrix<double, 4, 3>& controls) {
  DistanceSystem out{.system = Eigen::Matrix<double, 6, 10>::Zero(),
                     .rho = Eigen::Matrix<double, 6, 1>::Zero()};
  for (std::size_t row = 0; row < kPairs.size(); ++row) {
    const auto [a, b] = kPairs.at(row);
    const auto r = static_cast<Eigen::Index>(row);
    std::array<Eigen::Vector3d, 4> dv{};
    for (std::size_t k = 0; k < dv.size(); ++k) {
      const auto vk = static_cast<Eigen::Index>(k);
      const Eigen::Index first = Eigen::Index{3} * a;
      const Eigen::Index second = Eigen::Index{3} * b;
      dv.at(k) = (basis.block<1, 3>(vk, first) - basis.block<1, 3>(vk, second)).transpose();
    }
    for (std::size_t column = 0; column < kMonomials.size(); ++column) {
      const auto [k, m] = kMonomials.at(column);
      const double factor = k == m ? 1.0 : 2.0;
      out.system(r, static_cast<Eigen::Index>(column)) =
          factor * dv.at(static_cast<std::size_t>(k)).dot(dv.at(static_cast<std::size_t>(m)));
    }
    out.rho(r) = (controls.row(a) - controls.row(b)).squaredNorm();
  }
  return out;
}

/// @brief One linearisation of the distance equations, factored back into betas
///        (`_initial_betas`).
/// @param distances The distance system.
/// @param kind 0 (N = 4), 1 (N = 2) or 2 (N = 3).
/// @return The betas.
Eigen::Vector4d initial_betas(const DistanceSystem& distances, std::size_t kind) {
  const Approximation& approximation = kApproximations.at(kind);
  Eigen::MatrixXd kept(6, approximation.count);
  for (Eigen::Index j = 0; j < approximation.count; ++j) {
    kept.col(j) = distances.system.col(approximation.columns.at(static_cast<std::size_t>(j)));
  }
  const Eigen::VectorXd products = kept.completeOrthogonalDecomposition().solve(distances.rho);
  Eigen::Vector4d betas = Eigen::Vector4d::Zero();
  if (kind == 0) {
    const double first = std::sqrt(std::abs(products(0)));
    const double sign = products(0) < 0.0 ? -1.0 : 1.0;
    betas(0) = first;
    if (first > 0.0) {
      betas.tail<3>() = sign * products.tail<3>() / first;
    }
    return betas;
  }
  if (products(0) < 0.0) {
    betas(0) = std::sqrt(-products(0));
    betas(1) = products(2) < 0.0 ? std::sqrt(-products(2)) : 0.0;
  } else {
    betas(0) = std::sqrt(products(0));
    betas(1) = products(2) > 0.0 ? std::sqrt(products(2)) : 0.0;
  }
  if (products(1) < 0.0) {
    betas(0) = -betas(0);
  }
  if (kind == 2 && betas(0) != 0.0) {
    betas(2) = products(3) / betas(0);
  }
  return betas;
}

/// @brief The ten products `b_k b_l`.
/// @param betas The betas.
/// @return The products, in `kMonomials` order.
Eigen::Matrix<double, 10, 1> monomials(const Eigen::Vector4d& betas) {
  Eigen::Matrix<double, 10, 1> out;
  for (std::size_t column = 0; column < kMonomials.size(); ++column) {
    const auto [k, m] = kMonomials.at(column);
    out(static_cast<Eigen::Index>(column)) = betas(k) * betas(m);
  }
  return out;
}

/// @brief Polish the betas by Gauss-Newton on the distance equations (`_polish_betas`).
/// @param betas The betas.
/// @param distances The distance system.
/// @return The polished betas.
Eigen::Vector4d polish_betas(Eigen::Vector4d betas, const DistanceSystem& distances) {
  Eigen::Matrix<double, 6, 1> residual = distances.rho - (distances.system * monomials(betas));
  for (int step_count = 0; step_count < kPolishMaxSteps; ++step_count) {
    Eigen::Matrix<double, 6, 4> jacobian = Eigen::Matrix<double, 6, 4>::Zero();
    for (std::size_t column = 0; column < kMonomials.size(); ++column) {
      const auto [k, m] = kMonomials.at(column);
      const auto c = static_cast<Eigen::Index>(column);
      jacobian.col(k) += distances.system.col(c) * betas(m);
      jacobian.col(m) += distances.system.col(c) * betas(k);
    }
    const Eigen::Vector4d step = jacobian.completeOrthogonalDecomposition().solve(residual);
    const Eigen::Vector4d moved = betas + step;
    const Eigen::Matrix<double, 6, 1> moved_residual =
        distances.rho - (distances.system * monomials(moved));
    if (!(moved_residual.norm() < residual.norm())) {
      break;
    }
    betas = moved;
    residual = moved_residual;
    if (step.norm() <= kPolishTolerance * betas.norm()) {
      break;
    }
  }
  return betas;
}

/// @brief Least squares on the reprojection errors of correspondences, over SE(3) (see
///        `slamlab.foundations.pnp.ReprojectionProblem`).
class ReprojectionProblem {
 public:
  /// @brief The state: `T_cam_world`.
  using State = Eigen::Matrix4d;

  /// @brief Hold the inliers.
  /// @param points World points (M x 3).
  /// @param image Their normalised image points (M x 2).
  /// @param weights Each block's weight, positive: its residual and Jacobian are scaled by the
  ///        weight's square root.
  ReprojectionProblem(Rows3 points, Rows2 image, Eigen::VectorXd weights)
      : points_(std::move(points)),
        image_(std::move(image)),
        weights_(std::move(weights)),
        root_(weights_.cwiseSqrt()) {}

  /// @brief Each block's squared residual; infinity behind the camera, so a step that moves a
  ///        point there is rejected (cheirality).
  /// @param state The pose.
  /// @return The squared residuals.
  [[nodiscard]] Eigen::VectorXd squared_norms(const State& state) const {
    return reprojection(state, points_, image_).cwiseProduct(weights_);
  }

  /// @brief The weighted normal equations over `(rho, theta)`.
  /// @param state The pose.
  /// @param weights The blocks' weights.
  /// @return The dense system.
  [[nodiscard]] nls::DenseSystem linearize(const State& state,
                                           const Eigen::VectorXd& weights) const {
    const Eigen::Matrix3d rotation = state.topLeftCorner<3, 3>();
    Eigen::MatrixXd jacobian(2 * points_.rows(), 6);
    Eigen::VectorXd residuals(2 * points_.rows());
    for (Eigen::Index i = 0; i < points_.rows(); ++i) {
      const Eigen::Vector3d world = points_.row(i).transpose();
      const Eigen::Vector3d p = (rotation * world) + state.topRightCorner<3, 1>();
      Eigen::Matrix<double, 2, 3> projection;
      projection << 1.0 / p(2), 0.0, -p(0) / (p(2) * p(2)), 0.0, 1.0 / p(2), -p(1) / (p(2) * p(2));
      Eigen::Matrix<double, 3, 6> motion;
      motion << rotation, -rotation * so3::hat(world);
      jacobian.middleRows<2>(2 * i) = root_(i) * (projection * motion);
      residuals.segment<2>(2 * i) = root_(i) * residual(state, i);
    }
    return *nls::DenseSystem::build(jacobian, residuals, weights);
  }

  /// @brief Step on SE(3): `T exp(delta)`.
  /// @param state The pose.
  /// @param delta `(rho, theta)`.
  /// @return The moved pose.
  [[nodiscard]] static State retract(const State& state, const Eigen::VectorXd& delta) {
    return state * se3::exp(delta.head<6>());
  }

 private:
  /// @brief One correspondence's residual, `pi(R X + t) - x`.
  /// @param state The pose.
  /// @param i The correspondence.
  /// @return The residual.
  [[nodiscard]] Eigen::Vector2d residual(const State& state, Eigen::Index i) const {
    const Eigen::Vector3d p =
        (state.topLeftCorner<3, 3>() * points_.row(i).transpose()) + state.topRightCorner<3, 1>();
    return (p.head<2>() / p(2)) - image_.row(i).transpose();
  }

  /// @brief The world points.
  Rows3 points_;
  /// @brief Their normalised image points.
  Rows2 image_;
  /// @brief Each block's weight.
  Eigen::VectorXd weights_;
  /// @brief Its square root.
  Eigen::VectorXd root_;
};

/// @brief Check one rigid (4, 4) transform (`_as_pose`).
/// @param pose The pose.
/// @return Nothing, or `InvalidInput`.
Result<void> check_pose(const Eigen::Matrix4d& pose) {
  if (!pose.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "pose: expected one finite (4, 4) matrix");
  }
  return se3::check_transform(pose);
}

}  // namespace

Eigen::VectorXd reprojection(const Eigen::Matrix4d& pose, const Eigen::Ref<const Rows3>& points,
                             const Eigen::Ref<const Rows2>& image) {
  Eigen::VectorXd out(points.rows());
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const Eigen::Vector3d p =
        (pose.topLeftCorner<3, 3>() * points.row(i).transpose()) + pose.topRightCorner<3, 1>();
    out(i) = p(2) > 0.0 ? ((p.head<2>() / p(2)) - image.row(i).transpose()).squaredNorm()
                        : std::numeric_limits<double>::infinity();
  }
  return out;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (points, bearings), as in Python
std::optional<Eigen::Matrix4d> solve_epnp(const Eigen::Ref<const Rows3>& points,
                                          const Eigen::Ref<const Rows3>& bearings) {
  const Rows3 world = points;
  const auto controls = control_points(world);
  if (!controls) {
    return std::nullopt;
  }
  const Eigen::Index n = world.rows();
  Eigen::MatrixXd homogeneous(n, 4);
  homogeneous << Eigen::VectorXd::Ones(n), world;
  Eigen::Matrix4d control_homogeneous;
  control_homogeneous << Eigen::Vector4d::Ones(), *controls;
  const Eigen::MatrixXd alphas = homogeneous * control_homogeneous.inverse();
  const Rows2 image = correspondences::normalised_xy(bearings);
  Eigen::MatrixXd system = Eigen::MatrixXd::Zero(2 * n, 12);
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < 4; ++j) {
      system(2 * i, 3 * j) = alphas(i, j);
      system(2 * i, (3 * j) + 2) = -alphas(i, j) * image(i, 0);
      system((2 * i) + 1, (3 * j) + 1) = alphas(i, j);
      system((2 * i) + 1, (3 * j) + 2) = -alphas(i, j) * image(i, 1);
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(system.transpose() * system);
  const Eigen::Matrix<double, 4, 12> basis =
      deterministic_sign(eigen.eigenvectors().leftCols<4>()).transpose();
  const DistanceSystem distances = distance_system(basis, *controls);
  std::optional<std::pair<double, Eigen::Matrix4d>> best;
  for (std::size_t kind = 0; kind < kApproximations.size(); ++kind) {
    const Eigen::Vector4d betas = polish_betas(initial_betas(distances, kind), distances);
    const Eigen::Matrix<double, 1, 12> combined = betas.transpose() * basis;
    const Eigen::Matrix<double, 4, 3, Eigen::RowMajor> camera_controls =
        Eigen::Map<const Eigen::Matrix<double, 4, 3, Eigen::RowMajor>>(combined.data());
    Rows3 camera = alphas * camera_controls;
    if (camera.col(2).sum() < 0.0) {
      camera = -camera;
    }
    const auto fitted = alignment::umeyama(world, camera, Eigen::VectorXd::Ones(n), false);
    if (!fitted) {
      continue;
    }
    const double error = reprojection(*fitted, world, image).sum();
    if (!best || error < best->first) {
      best = std::pair{error, *fitted};
    }
  }
  if (!best) {
    return std::nullopt;
  }
  return best->second;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (points, bearings), as in Python
Result<Eigen::Matrix4d> epnp(const Eigen::Ref<const Rows3>& points,
                             const Eigen::Ref<const Rows3>& bearings) {
  if (auto ok = correspondences::check_correspondences(points, bearings, kEpnpMinimum); !ok) {
    return std::unexpected(ok.error());
  }
  auto pose = solve_epnp(points, bearings);
  if (!pose) {
    return fail(ErrorKind::kDegenerate,
                "epnp: the points are planar, the control points undetermined");
  }
  return *pose;
}

Result<solve::Solution<Eigen::Matrix4d, nls::Summary>> refine(
    const Eigen::Matrix4d& pose, const Eigen::Ref<const Rows3>& points,
    const Eigen::Ref<const Rows3>& bearings, double scale,
    const Eigen::Ref<const Eigen::VectorXd>& weights) {
  const ReprojectionProblem problem(points, correspondences::normalised_xy(bearings), weights);
  nls::Options options;
  options.loss = {.kind = nls::LossKind::kCauchy, .scale = scale};
  options.max_iterations = kRefineIterations;
  // Not strict, so the solve fails only on a non-finite start: a point behind the camera.
  return nls::solve(problem, pose, options);
}

Result<solve::Solution<Eigen::Matrix4d, nls::Summary>> refine_pose(
    const Eigen::Matrix4d& pose, const Eigen::Ref<const Rows3>& points,
    const Eigen::Ref<const Rows3>& bearings, double scale,
    const Eigen::Ref<const Eigen::VectorXd>& weights) {
  if (auto ok = check_pose(pose); !ok) {
    return std::unexpected(ok.error());
  }
  if (auto ok = correspondences::check_correspondences(points, bearings, p3p::kSampleSize); !ok) {
    return std::unexpected(ok.error());
  }
  if (!std::isfinite(scale) || scale <= 0.0) {
    return fail(ErrorKind::kInvalidInput,
                "scale: must be positive and finite, got " + std::to_string(scale));
  }
  if (weights.size() != points.rows() || !weights.allFinite() || !(weights.array() > 0.0).all()) {
    return fail(ErrorKind::kInvalidInput,
                "weights: expected positive values, one per correspondence");
  }
  const Eigen::VectorXd depths = (points * pose.block<1, 3>(2, 0).transpose()).array() + pose(2, 3);
  if (!(depths.array() > 0.0).all()) {
    return fail(ErrorKind::kInvalidInput, "pose: a point is behind the starting camera");
  }
  return refine(pose, points, bearings, scale, weights);
}

PoseEstimator::PoseEstimator(Rows3 points, Rows3 bearings, p3p::Solver solver)
    : points_(std::move(points)),
      bearings_(std::move(bearings)),
      image_(correspondences::normalised_xy(bearings_)),
      solver_(solver) {}

std::uint32_t PoseEstimator::size() const { return static_cast<std::uint32_t>(points_.rows()); }

std::vector<PoseEstimator::Model> PoseEstimator::fit_minimal(
    std::span<const std::uint32_t> sample) const {
  return p3p::solve(correspondences::gather(points_, sample),
                    correspondences::gather(bearings_, sample), solver_);
}

std::vector<PoseEstimator::Model> PoseEstimator::fit_nonminimal(
    std::span<const std::uint32_t> subset) const {
  if (subset.size() < kEpnpMinimum) {
    return {};
  }
  auto pose = solve_epnp(correspondences::gather(points_, subset),
                         correspondences::gather(bearings_, subset));
  return pose ? std::vector<Model>{*pose} : std::vector<Model>{};
}

Eigen::VectorXd PoseEstimator::residuals(const Model& model) const {
  return reprojection(model, points_, image_);
}

Result<ransac::RefinedSolution<Eigen::Matrix4d>> pnp_ransac(const Eigen::Ref<const Rows3>& points,
                                                            const Eigen::Ref<const Rows3>& bearings,
                                                            const ransac::Options& options,
                                                            p3p::Solver minimal, bool refine) {
  if (minimal != p3p::Solver::kDing && minimal != p3p::Solver::kLambdaTwist) {
    return fail(ErrorKind::kInvalidInput, "minimal_solver: unknown code");
  }
  if (auto ok = correspondences::check_correspondences(points, bearings, p3p::kSampleSize); !ok) {
    return std::unexpected(ok.error());
  }
  const Rows3 world = points;
  const Rows3 rays = bearings;
  return ransac::estimate_refined(
      PoseEstimator(points, bearings, minimal), options, refine,
      [&](const Eigen::Matrix4d& model, const ransac::Indices& inliers) {
        return pnp::refine(model, correspondences::gather(world, inliers),
                           correspondences::gather(rays, inliers), options.threshold,
                           Eigen::VectorXd::Ones(static_cast<Eigen::Index>(inliers.size())));
      },
      p3p::kSampleSize);
}

}  // namespace slamlab::foundations::pnp
