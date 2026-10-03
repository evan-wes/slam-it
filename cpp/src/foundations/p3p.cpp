/// @file
/// @brief P3P: Lambda Twist and Ding et al.'s solver (see `slamlab/foundations/p3p.hpp`).
#include "slamlab/foundations/p3p.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "slamlab/foundations/alignment.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::p3p {

using correspondences::Rows3;

namespace {

/// @brief The distance equations' data: `b_ij` (bearing cosines) and `a_ij` (squared world
///        distances), for `ij = 12, 13, 23`.
struct Geometry {
  /// @brief The cosines `b_ij`.
  Eigen::Vector3d cosines;
  /// @brief The squared distances `a_ij`.
  Eigen::Vector3d squared;
};

/// @brief The distance equations' data as named scalars, `b_ij` then `a_ij`.
struct Terms {
  /// @brief `b_12`.
  double b12;
  /// @brief `b_13`.
  double b13;
  /// @brief `b_23`.
  double b23;
  /// @brief `a_12`.
  double a12;
  /// @brief `a_13`.
  double a13;
  /// @brief `a_23`.
  double a23;
};

/// @brief Unpack the distance equations' data.
/// @param g The cosines and squared distances.
/// @return The named scalars.
Terms terms_of(const Geometry& g) {
  return {.b12 = g.cosines(0),
          .b13 = g.cosines(1),
          .b23 = g.cosines(2),
          .a12 = g.squared(0),
          .a13 = g.squared(1),
          .a23 = g.squared(2)};
}

/// @brief The point pairs of the three distance equations, in `Geometry`'s order.
constexpr std::array<std::pair<int, int>, 3> kPairs = {{{0, 1}, {0, 2}, {1, 2}}};

/// @brief The monic cubic's value at `z`.
/// @param z The argument.
/// @param c The coefficients `(c2, c1, c0)`.
/// @return `z^3 + c2 z^2 + c1 z + c0`.
double cubic(double z, const std::array<double, 3>& c) {
  return (((z + c[0]) * z + c[1]) * z) + c[2];
}

/// @brief Polish a root of the monic cubic by Newton's method (`_polish_cubic` in Python).
/// @param root The root.
/// @param c The coefficients `(c2, c1, c0)`.
/// @return The polished root.
double polish_cubic(double root, const std::array<double, 3>& c) {
  double value = cubic(root, c);
  for (int k = 0; k < kPolishMaxSteps; ++k) {
    const double slope = ((3.0 * root + 2.0 * c[0]) * root) + c[1];
    if (!(slope != 0.0)) {
      break;
    }
    const double step = value / slope;
    const double moved = root - step;
    const double moved_value = cubic(moved, c);
    if (!(std::abs(moved_value) < std::abs(value))) {
      break;
    }
    root = moved;
    value = moved_value;
    if (std::abs(step) <= kPolishTolerance * std::max(1.0, std::abs(root))) {
      break;
    }
  }
  return root;
}

/// @brief A symmetric 3x3 matrix's cofactor matrix, column k the cross product of the other
///        two columns (`pencil_cubic`'s `cofactor` in Python).
/// @param m The matrix.
/// @return The cofactor matrix.
Eigen::Matrix3d cofactor(const Eigen::Matrix3d& m) {
  Eigen::Matrix3d out;
  out.col(0) = m.col(1).cross(m.col(2));
  out.col(1) = m.col(2).cross(m.col(0));
  out.col(2) = m.col(0).cross(m.col(1));
  return out;
}

/// @brief `det(first + s second)` as a cubic in `s`, highest power first.
/// @param first The first conic.
/// @param second The second conic.
/// @return `(det B, <A, cof B>, <B, cof A>, det A)`.
std::array<double, 4> pencil_cubic(const Eigen::Matrix3d& first, const Eigen::Matrix3d& second) {
  const Eigen::Matrix3d cof_first = cofactor(first);
  const Eigen::Matrix3d cof_second = cofactor(second);
  return {second.col(0).dot(cof_second.col(0)), first.cwiseProduct(cof_second).sum(),
          second.cwiseProduct(cof_first).sum(), first.col(0).dot(cof_first.col(0))};
}

/// @brief The real roots of `a t^2 + b t + c`, without cancellation (`quadratic_roots`).
/// @param a The `t^2` coefficient.
/// @param b The `t` coefficient.
/// @param c The constant.
/// @return Zero, one or two roots.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the coefficients, highest first
std::vector<double> quadratic_roots(double a, double b, double c) {
  const double discriminant = (b * b) - (4.0 * a * c);
  const double scale = (b * b) + std::abs(4.0 * a * c);
  if (discriminant < -kRealTolerance * scale) {
    return {};
  }
  if (a == 0.0) {
    return b != 0.0 ? std::vector<double>{-c / b} : std::vector<double>{};
  }
  if (discriminant < 0.0) {
    return {-0.5 * b / a};
  }
  const double half = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
  return half != 0.0 ? std::vector<double>{half / a, c / half} : std::vector<double>{half / a};
}

/// @brief The depth `sqrt(a / (t^2 - 2 b t + 1))` that one distance equation fixes (`_scaled`).
/// @param squared The squared distance `a`.
/// @param tau The depth ratio `t`.
/// @param cosine The bearings' cosine `b`.
/// @return The depth, or nothing where rounding pushed the denominator to zero or below (two
///         coinciding bearings).
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (a, t, b), as in Python
std::optional<double> scaled(double squared, double tau, double cosine) {
  const double denominator = (tau * (tau - (2.0 * cosine))) + 1.0;
  if (!(denominator > 0.0)) {
    return std::nullopt;
  }
  return std::sqrt(squared / denominator);
}

/// @brief The distance equations' residuals at the depths.
/// @param depths The depths.
/// @param geometry The cosines and squared distances.
/// @return `r_ij`, for `ij = 12, 13, 23`.
Eigen::Vector3d distance_residuals(const Eigen::Vector3d& depths, const Geometry& geometry) {
  Eigen::Vector3d out;
  for (std::size_t k = 0; k < kPairs.size(); ++k) {
    const auto [i, j] = kPairs.at(k);
    const auto row = static_cast<Eigen::Index>(k);
    out(row) = (depths(i) * depths(i)) - (2.0 * geometry.cosines(row) * depths(i) * depths(j)) +
               (depths(j) * depths(j)) - geometry.squared(row);
  }
  return out;
}

/// @brief Polish the depths by Newton's method on the distance equations (`_polish_depths`).
/// @param depths The depths.
/// @param geometry The cosines and squared distances.
/// @return The polished depths and the final residual's norm.
std::pair<Eigen::Vector3d, double> polish_depths(Eigen::Vector3d depths, const Geometry& geometry) {
  Eigen::Vector3d current = distance_residuals(depths, geometry);
  for (int step_count = 0; step_count < kPolishMaxSteps; ++step_count) {
    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
    for (std::size_t k = 0; k < kPairs.size(); ++k) {
      const auto [i, j] = kPairs.at(k);
      const auto row = static_cast<Eigen::Index>(k);
      jacobian(row, i) = 2.0 * (depths(i) - (geometry.cosines(row) * depths(j)));
      jacobian(row, j) = 2.0 * (depths(j) - (geometry.cosines(row) * depths(i)));
    }
    const Eigen::FullPivLU<Eigen::Matrix3d> lu(jacobian);
    if (!lu.isInvertible()) {
      break;
    }
    const Eigen::Vector3d step = lu.solve(current);
    const Eigen::Vector3d moved = depths - step;
    const Eigen::Vector3d moved_residuals = distance_residuals(moved, geometry);
    if (!(moved_residuals.norm() < current.norm())) {
      break;
    }
    depths = moved;
    current = moved_residuals;
    if (step.norm() <= kPolishTolerance * depths.norm()) {
      break;
    }
  }
  return {depths, current.norm()};
}

/// @brief `T_cam_world` aligning the world triangle to `l_i y_i` (`_pose`).
/// @param depths The depths.
/// @param bearings The bearings.
/// @param points The world points.
/// @return The pose, or nothing if the alignment is degenerate.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (bearings, points), as in Python
std::optional<Eigen::Matrix4d> pose_of(const Eigen::Vector3d& depths, const Rows3& bearings,
                                       const Rows3& points) {
  const Rows3 camera = depths.asDiagonal() * bearings;
  auto fitted = alignment::umeyama(points, camera, Eigen::Vector3d::Ones(), false);
  if (!fitted) {
    return std::nullopt;
  }
  return *fitted;
}

/// @brief Whether three world points are (nearly) collinear (`_collinear`).
/// @param points The points.
/// @return True if they have no unique pose.
bool collinear(const Rows3& points) {
  const Eigen::Vector3d d1 = (points.row(0) - points.row(1)).transpose();
  const Eigen::Vector3d d2 = (points.row(0) - points.row(2)).transpose();
  return !(d1.cross(d2).norm() > kCollinearTolerance * d1.norm() * d2.norm());
}

/// @brief Polish candidate depths and add the pose, if all three stay positive and satisfy
///        the distance equations (`_finish`).
/// @param depths The candidate depths.
/// @param bearings The bearings.
/// @param points The world points.
/// @param geometry The cosines and squared distances.
/// @param poses The poses found so far.
void finish(const Eigen::Vector3d& depths, const Rows3& bearings, const Rows3& points,
            const Geometry& geometry, std::vector<Eigen::Matrix4d>& poses) {
  const auto [polished, residual] = polish_depths(depths, geometry);
  const bool positive = (polished.array() > 0.0).all();
  if (!positive || residual > kDistanceTolerance * geometry.squared.maxCoeff()) {
    return;
  }
  auto pose = pose_of(polished, bearings, points);
  if (pose && pose->allFinite()) {
    poses.push_back(*pose);
  }
}

/// @brief The squared distances and cosines of three correspondences.
/// @param points The world points.
/// @param bearings The bearings.
/// @return `a_ij` and `b_ij`, for `ij = 12, 13, 23`.
Geometry geometry_of(const Rows3& points, const Rows3& bearings) {
  Geometry out{.cosines = Eigen::Vector3d::Zero(), .squared = Eigen::Vector3d::Zero()};
  for (std::size_t k = 0; k < kPairs.size(); ++k) {
    const auto [i, j] = kPairs.at(k);
    const auto row = static_cast<Eigen::Index>(k);
    out.squared(row) = (points.row(i) - points.row(j)).squaredNorm();
    out.cosines(row) = bearings.row(i).dot(bearings.row(j));
  }
  return out;
}

/// @brief A symmetric matrix with a zero eigenvalue, eigen-decomposed in closed form
///        (`_eigen_known_zero`).
struct KnownZeroEigen {
  /// @brief The eigenvectors of the two nonzero eigenvalues, as columns.
  Eigen::Matrix<double, 3, 2> vectors;
  /// @brief The larger eigenvalue in magnitude.
  double sigma1 = 0.0;
  /// @brief The smaller.
  double sigma2 = 0.0;
};

/// @brief Eigen-decompose a symmetric 3x3 matrix with a zero eigenvalue, in closed form.
/// @param m The matrix.
/// @return The decomposition, or nothing if the eigenvalues are not real or an eigenvector's
///         last entry vanishes.
std::optional<KnownZeroEigen> eigen_known_zero(const Eigen::Matrix3d& m) {
  const double p1 = -m.trace();
  const double p0 = -(m(0, 1) * m(0, 1)) - (m(0, 2) * m(0, 2)) - (m(1, 2) * m(1, 2)) +
                    (m(0, 0) * (m(1, 1) + m(2, 2))) + (m(1, 1) * m(2, 2));
  const double inside = (p1 * p1 / 4.0) - p0;
  if (!(inside >= 0.0)) {
    return std::nullopt;
  }
  const double half = std::sqrt(inside);
  KnownZeroEigen out{.vectors = Eigen::Matrix<double, 3, 2>::Zero(),
                     .sigma1 = (-p1 / 2.0) + half,
                     .sigma2 = (-p1 / 2.0) - half};
  if (std::abs(out.sigma1) < std::abs(out.sigma2)) {
    std::swap(out.sigma1, out.sigma2);
  }
  const std::array<double, 2> sigmas = {out.sigma1, out.sigma2};
  for (Eigen::Index column = 0; column < 2; ++column) {
    const double sigma = sigmas.at(static_cast<std::size_t>(column));
    const double c =
        (sigma * sigma) + (m(0, 0) * m(1, 1)) - (sigma * (m(0, 0) + m(1, 1))) - (m(0, 1) * m(0, 1));
    if (!(c != 0.0)) {
      return std::nullopt;
    }
    const double a1 = ((sigma * m(0, 2)) + (m(0, 1) * m(1, 2)) - (m(0, 2) * m(1, 1))) / c;
    const double a2 = ((sigma * m(1, 2)) + (m(0, 1) * m(0, 2)) - (m(0, 0) * m(1, 2))) / c;
    out.vectors.col(column) = Eigen::Vector3d(a1, a2, 1.0) / std::sqrt(1.0 + (a1 * a1) + (a2 * a2));
  }
  return out;
}

/// @brief Intersect one of Lambda Twist's lines, `u . L = 0`, with the distance conics
///        (`_twist_depths`).
/// @param u The line.
/// @param geometry The cosines and squared distances.
/// @return The candidate depths, every one positive.
std::vector<Eigen::Vector3d> twist_depths(const Eigen::Vector3d& u, const Geometry& geometry) {
  const auto [b12, b13, b23, a12, a13, a23] = terms_of(geometry);
  std::vector<Eigen::Vector3d> candidates;
  if (std::abs(u(0)) < std::abs(u(1))) {
    // l2 = w0 l1 + w1 l3; t = l3 / l1.
    const double w0 = -u(0) / u(1);
    const double w1 = -u(2) / u(1);
    for (const double tau :
         quadratic_roots((a13 * w1 * w1) - (2.0 * a13 * b23 * w1) + a13 - a23,
                         (2.0 * a13 * w0 * w1) - (2.0 * a13 * b23 * w0) + (2.0 * a23 * b13),
                         (a13 * w0 * w0) - a23)) {
      const auto lam1 = tau > 0.0 ? scaled(a13, tau, b13) : std::nullopt;
      if (lam1) {
        const double lam3 = tau * *lam1;
        candidates.emplace_back(*lam1, (w0 * *lam1) + (w1 * lam3), lam3);
      }
    }
  } else {
    // l1 = w0 l2 + w1 l3; t = l3 / l2.
    const double w0 = -u(1) / u(0);
    const double w1 = -u(2) / u(0);
    for (const double tau : quadratic_roots(
             ((a13 - a12) * w1 * w1) + (2.0 * a12 * b13 * w1) - a12,
             (2.0 * (a13 - a12) * w0 * w1) - (2.0 * a13 * b12 * w1) + (2.0 * a12 * b13 * w0),
             ((a13 - a12) * w0 * w0) - (2.0 * a13 * b12 * w0) + a13)) {
      const auto lam2 = tau > 0.0 ? scaled(a23, tau, b23) : std::nullopt;
      if (lam2) {
        const double lam3 = tau * *lam2;
        candidates.emplace_back((w0 * *lam2) + (w1 * lam3), *lam2, lam3);
      }
    }
  }
  std::erase_if(candidates, [](const Eigen::Vector3d& d) { return !(d.array() > 0.0).all(); });
  return candidates;
}

/// @brief Lambda Twist (`_lambda_twist`).
/// @param points The world points.
/// @param bearings The bearings.
/// @return The poses.
std::vector<Eigen::Matrix4d> lambda_twist(const Rows3& points, const Rows3& bearings) {
  const Geometry geometry = geometry_of(points, bearings);
  const auto [b12, b13, b23, a12, a13, a23] = terms_of(geometry);
  Eigen::Matrix3d d1;
  d1 << a23, -a23 * b12, 0.0, -a23 * b12, a23 - a12, a12 * b23, 0.0, a12 * b23, -a12;
  Eigen::Matrix3d d2;
  d2 << a23, 0.0, -a23 * b13, 0.0, -a13, a13 * b23, -a23 * b13, a13 * b23, a23 - a13;
  const auto [c3, c2, c1, c0] = pencil_cubic(d1, d2);
  if (!(c3 != 0.0)) {
    return {};
  }
  const double gamma = cubic_root(c2 / c3, c1 / c3, c0 / c3).first;
  const auto decomposed = eigen_known_zero(d1 + (gamma * d2));
  if (!decomposed || !(-decomposed->sigma2 / decomposed->sigma1 >= 0.0)) {
    return {};
  }
  const double s = std::sqrt(-decomposed->sigma2 / decomposed->sigma1);
  std::vector<Eigen::Matrix4d> poses;
  for (const double sign : {1.0, -1.0}) {
    const Eigen::Vector3d u = decomposed->vectors.col(0) - (sign * s * decomposed->vectors.col(1));
    for (const Eigen::Vector3d& depths : twist_depths(u, geometry)) {
      finish(depths, bearings, points, geometry, poses);
    }
  }
  return poses;
}

/// @brief Split a degenerate conic into its two lines through its adjugate (`_line_pair`).
/// @param c The conic, rank 2.
/// @return The lines, or nothing if they are complex.
std::optional<std::array<Eigen::Vector3d, 2>> line_pair(const Eigen::Matrix3d& c) {
  Eigen::Matrix3d adjugate;
  adjugate << (c(1, 1) * c(2, 2)) - (c(1, 2) * c(1, 2)), (c(0, 2) * c(1, 2)) - (c(0, 1) * c(2, 2)),
      (c(0, 1) * c(1, 2)) - (c(0, 2) * c(1, 1)), (c(0, 2) * c(1, 2)) - (c(0, 1) * c(2, 2)),
      (c(0, 0) * c(2, 2)) - (c(0, 2) * c(0, 2)), (c(0, 1) * c(0, 2)) - (c(0, 0) * c(1, 2)),
      (c(0, 1) * c(1, 2)) - (c(0, 2) * c(1, 1)), (c(0, 1) * c(0, 2)) - (c(0, 0) * c(1, 2)),
      (c(0, 0) * c(1, 1)) - (c(0, 1) * c(0, 1));
  adjugate = -adjugate;
  Eigen::Index k = 0;
  adjugate.diagonal().maxCoeff(&k);
  if (!(adjugate(k, k) > 0.0)) {
    return std::nullopt;
  }
  const Eigen::Vector3d w = adjugate.col(k) / std::sqrt(adjugate(k, k));
  const Eigen::Matrix3d rank_one = c + so3::hat(w);
  Eigen::Index row = 0;
  Eigen::Index column = 0;
  rank_one.cwiseAbs().maxCoeff(&row, &column);
  return std::array<Eigen::Vector3d, 2>{rank_one.col(column), rank_one.row(row).transpose()};
}

/// @brief Intersect one of Ding et al.'s lines, `p . (d2, d0, d1) = 0`, with a conic
///        (`_ding_depths`).
/// @param p The line.
/// @param geometry The reordered points' cosines and squared distances.
/// @return The candidate depths, every one positive.
std::vector<Eigen::Vector3d> ding_depths(const Eigen::Vector3d& p, const Geometry& geometry) {
  const double m01 = geometry.cosines(0);
  const double m02 = geometry.cosines(1);
  const double m12 = geometry.cosines(2);
  const double a01 = geometry.squared(0);
  const double a12 = geometry.squared(2);
  const double a = a01 / a12;
  const double b = geometry.squared(1) / a12;
  std::vector<Eigen::Vector3d> candidates;
  if (std::abs(p(0)) <= std::abs(p(1))) {
    // d0 = w0 d2 + w1 d1; t = d1 / d2.
    const double w0 = -p(0) / p(1);
    const double w1 = -p(2) / p(1);
    for (const double tau :
         quadratic_roots((w1 * w1) - b, 2.0 * ((b * m12) - (m02 * w1) + (w0 * w1)),
                         (w0 * w0) - (2.0 * m02 * w0) - b + 1.0)) {
      const auto d2 = tau > 0.0 ? scaled(a12, tau, m12) : std::nullopt;
      if (d2) {
        const double d1 = tau * *d2;
        candidates.emplace_back((w0 * *d2) + (w1 * d1), d1, *d2);
      }
    }
  } else {
    // d2 = w0 d0 + w1 d1; t = d1 / d0.
    const double w0 = -p(1) / p(0);
    const double w1 = -p(2) / p(0);
    for (const double tau :
         quadratic_roots((-a * w1 * w1) + (2.0 * a * m12 * w1) - a + 1.0,
                         2.0 * ((a * m12 * w0) - m01 - (a * w0 * w1)), 1.0 - (a * w0 * w0))) {
      const auto d0 = tau > 0.0 ? scaled(a01, tau, m01) : std::nullopt;
      if (d0) {
        const double d1 = tau * *d0;
        candidates.emplace_back(*d0, d1, (w0 * *d0) + (w1 * d1));
      }
    }
  }
  std::erase_if(candidates, [](const Eigen::Vector3d& d) { return !(d.array() > 0.0).all(); });
  return candidates;
}

/// @brief Ding et al.'s solver (`_ding`).
/// @param points The world points.
/// @param bearings The bearings.
/// @return The poses.
std::vector<Eigen::Matrix4d> ding(const Rows3& points, const Rows3& bearings) {
  const Geometry unordered = geometry_of(points, bearings);
  std::array<Eigen::Index, 3> order = {0, 1, 2};
  // Reorder so the distance between the second and third points is the longest.
  if (unordered.squared(0) > unordered.squared(1)) {
    if (unordered.squared(0) > unordered.squared(2)) {
      order = {2, 1, 0};
    }
  } else if (unordered.squared(1) > unordered.squared(2)) {
    order = {1, 0, 2};
  }
  Rows3 x(3, 3);
  Rows3 world(3, 3);
  for (Eigen::Index i = 0; i < 3; ++i) {
    x.row(i) = bearings.row(order.at(static_cast<std::size_t>(i)));
    world.row(i) = points.row(order.at(static_cast<std::size_t>(i)));
  }
  const Geometry geometry = geometry_of(world, x);
  const double m01 = geometry.cosines(0);
  const double m02 = geometry.cosines(1);
  const double m12 = geometry.cosines(2);
  const double a = geometry.squared(0) / geometry.squared(2);
  const double b = geometry.squared(1) / geometry.squared(2);
  Eigen::Matrix3d first;
  first << -a, 0.0, a * m12, 0.0, 1.0, -m01, a * m12, -m01, 1.0 - a;
  Eigen::Matrix3d second;
  second << 1.0 - b, -m02, b * m12, -m02, 1.0, 0.0, b * m12, 0.0, -b;
  const auto [c3, c2, c1, c0] = pencil_cubic(first, second);
  if (!(c3 != 0.0)) {
    return {};
  }
  const auto [s, single] = cubic_root(c2 / c3, c1 / c3, c0 / c3);
  const auto lines = line_pair(first + (s * second));
  if (!lines) {
    return {};
  }
  std::vector<Eigen::Matrix4d> poses;
  for (const Eigen::Vector3d& line : *lines) {
    for (const Eigen::Vector3d& depths : ding_depths(line, geometry)) {
      finish(depths, x, world, geometry, poses);
    }
    if (!poses.empty() && single) {
      break;
    }
  }
  return poses;
}

}  // namespace

const char* name(Solver solver) {
  switch (solver) {
    case Solver::kDing:
      return "ding";
    case Solver::kLambdaTwist:
      return "lambda_twist";
    case Solver::kCount:
      break;
  }
  return "unknown";
}

std::pair<double, bool> cubic_root(double c2, double c1, double c0) {
  const double p = c1 - (c2 * c2 / 3.0);
  const double q = ((2.0 * c2 * c2 * c2 - 9.0 * c2 * c1) / 27.0) + c0;
  const double discriminant = (q * q / 4.0) + (p * p * p / 27.0);
  double u = 0.0;
  bool single = false;
  if (discriminant >= 0.0) {
    const double root = std::sqrt(discriminant);
    u = std::cbrt((-0.5 * q) + root) + std::cbrt((-0.5 * q) - root);
    single = discriminant > 0.0;
  } else {
    const double cosine = std::clamp(1.5 * q / p * std::sqrt(-3.0 / p), -1.0, 1.0);
    u = 2.0 * std::sqrt(-p / 3.0) * std::cos(std::acos(cosine) / 3.0);
  }
  return {polish_cubic(u - (c2 / 3.0), {c2, c1, c0}), single};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (points, bearings), as in Python
std::vector<Eigen::Matrix4d> solve(const Eigen::Ref<const correspondences::Rows3>& points,
                                   const Eigen::Ref<const correspondences::Rows3>& bearings,
                                   Solver solver) {
  const Rows3 world = points;
  const Rows3 rays = bearings;
  if (collinear(world)) {
    return {};
  }
  return solver == Solver::kDing ? ding(world, rays) : lambda_twist(world, rays);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (points, bearings), as in Python
Result<std::vector<Eigen::Matrix4d>> p3p(const Eigen::Ref<const correspondences::Rows3>& points,
                                         const Eigen::Ref<const correspondences::Rows3>& bearings,
                                         Solver solver) {
  if (solver != Solver::kDing && solver != Solver::kLambdaTwist) {
    return fail(ErrorKind::kInvalidInput, "solver: unknown code");
  }
  if (auto ok = correspondences::check_correspondences(points, bearings, kSampleSize, false); !ok) {
    return std::unexpected(ok.error());
  }
  if (points.rows() != kSampleSize) {
    return fail(ErrorKind::kInvalidInput,
                "points: expected exactly 3, got " + std::to_string(points.rows()));
  }
  return solve(points, bearings, solver);
}

}  // namespace slamlab::foundations::p3p
