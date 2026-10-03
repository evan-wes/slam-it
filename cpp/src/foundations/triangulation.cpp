/// @file
/// @brief Implementation of the triangulation declared in slamlab/foundations/triangulation.hpp.
#include "slamlab/foundations/triangulation.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <utility>

#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::triangulation {

namespace {

/// @brief The angle between two vectors, from `atan2` (accurate at 0 and pi).
/// @param a The first vector.
/// @param b The second vector.
/// @return The angle in [0, pi].
double angle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

/// @brief Check that every listed row is a unit vector.
/// @param bearings The bearings.
/// @param row The row to check.
/// @param name The argument's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_unit(const Eigen::Ref<const Rows3>& bearings, Eigen::Index row,
                        const char* name) {
  if (!bearings.row(row).allFinite() ||
      std::abs(bearings.row(row).norm() - 1.0) > kBearingTolerance) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": expected unit vectors within 1e-09");
  }
  return {};
}

/// @brief An empty result of `n` points, to be filled.
/// @param n The number of points.
/// @return Points, cheirality and parallax of size `n`.
Triangulation sized(Eigen::Index n) {
  return Triangulation{.points = Rows3(n, 3),
                       .in_front = Eigen::Array<bool, Eigen::Dynamic, 1>(n),
                       .parallax = Eigen::VectorXd(n)};
}

/// @brief Record one point, or a point at infinity (NaN, behind, no parallax).
/// @param out The result.
/// @param i The point's index.
/// @param point The point.
/// @param in_front Its cheirality.
/// @param parallax Its parallax.
/// @param bad Whether it is at infinity.
void store(Triangulation& out, Eigen::Index i, const Eigen::Vector3d& point, bool in_front,
           double parallax, bool bad) {
  if (bad) {
    out.points.row(i).setConstant(std::numeric_limits<double>::quiet_NaN());
    out.in_front(i) = false;
    out.parallax(i) = 0.0;
    return;
  }
  out.points.row(i) = point.transpose();
  out.in_front(i) = in_front;
  out.parallax(i) = parallax;
}

/// @brief One point's observations, for the DLT.
struct Track {
  /// @brief Every view's `T_cam_world`.
  std::span<const Eigen::Matrix4d> poses;
  /// @brief Every view's centre, `-R^T t`.
  std::span<const Eigen::Vector3d> centres;
  /// @brief The point's bearing in every view (V x 3).
  Eigen::Ref<const Rows3> bearings;
  /// @brief The views that observe it.
  std::span<const std::size_t> seen;
};

/// @brief Check `n_view`'s input, and find each view's centre.
/// @param poses Each view's pose.
/// @param bearings The bearings, row `p * V + v`.
/// @param observed The observation mask.
/// @return The centres, or `InvalidInput`.
Result<std::vector<Eigen::Vector3d>> check_n_view(const std::vector<Eigen::Matrix4d>& poses,
                                                  const Eigen::Ref<const Rows3>& bearings,
                                                  const MaskRef& observed) {
  const auto views = static_cast<Eigen::Index>(poses.size());
  const Eigen::Index n = observed.rows();
  if (observed.cols() != views || bearings.rows() != n * views) {
    return fail(ErrorKind::kInvalidInput, "bearings: expected (P, V, 3) with observed (P, V)");
  }
  std::vector<Eigen::Vector3d> centres;
  centres.reserve(poses.size());
  for (const Eigen::Matrix4d& pose : poses) {
    if (auto checked = se3::check_transform(pose); !checked) {
      return std::unexpected(checked.error());
    }
    centres.emplace_back(-pose.topLeftCorner<3, 3>().transpose() * pose.topRightCorner<3, 1>());
  }
  for (Eigen::Index p = 0; p < n; ++p) {
    if (observed.row(p).count() < 2) {
      return fail(ErrorKind::kInvalidInput,
                  "observed: every point needs at least two observing views");
    }
    for (Eigen::Index v = 0; v < views; ++v) {
      auto ok = observed(p, v) ? check_unit(bearings, (p * views) + v, "bearings") : Result<void>{};
      if (!ok) {
        return std::unexpected(ok.error());
      }
    }
  }
  return centres;
}

/// @brief The normalised DLT on one track's bearings.
/// @param track The point's observations.
/// @return The point, and whether it is undetermined (at infinity, or the system leaves a line
///         of solutions).
std::pair<Eigen::Vector3d, bool> dlt(const Track& track) {
  const auto k = static_cast<Eigen::Index>(track.seen.size());
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const std::size_t v : track.seen) {
    mean += track.centres[v];
  }
  mean /= static_cast<double>(k);
  double scale = 0.0;
  for (const std::size_t v : track.seen) {
    scale += (track.centres[v] - mean).norm();
  }
  scale /= static_cast<double>(k);
  scale = scale > 0.0 ? scale : 1.0;
  Eigen::MatrixXd system(3 * k, 4);
  for (Eigen::Index j = 0; j < k; ++j) {
    const std::size_t v = track.seen[static_cast<std::size_t>(j)];
    const Eigen::Matrix3d r = track.poses[v].topLeftCorner<3, 3>();
    // In the normalised world X = mean + scale X', so R X + t = scale (R X' + t').
    const Eigen::Vector3d t_norm = (track.poses[v].topRightCorner<3, 1>() + (r * mean)) / scale;
    const Eigen::Matrix3d skew =
        so3::hat(track.bearings.row(static_cast<Eigen::Index>(v)).transpose());
    system.block<3, 3>(3 * j, 0) = skew * r;
    system.block<3, 1>(3 * j, 3) = skew * t_norm;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeFullV> svd(system);
  const Eigen::VectorXd& singular = svd.singularValues();
  const Eigen::Vector4d homogeneous = svd.matrixV().col(3);
  const double w = homogeneous(3);
  // Rays along one line leave a two-dimensional null space: the point is undetermined.
  const bool undetermined = !(singular(2) > kParallelSine * singular(0)) ||
                            !(std::abs(w) > kParallelSine * homogeneous.head<3>().norm());
  return {mean + (scale * homogeneous.head<3>() / w), undetermined};
}

/// @brief A point's cheirality over the views that observe it, and its largest parallax.
/// @param track The point's observations.
/// @param point The point.
/// @return Whether it is ahead along every ray, and the largest angle between two centres.
std::pair<bool, double> cheirality_and_parallax(const Track& track, const Eigen::Vector3d& point) {
  bool in_front = true;
  double parallax = 0.0;
  for (std::size_t j = 0; j < track.seen.size(); ++j) {
    const std::size_t v = track.seen[j];
    const Eigen::Matrix4d& pose = track.poses[v];
    const Eigen::Vector3d in_cam =
        (pose.topLeftCorner<3, 3>() * point) + pose.topRightCorner<3, 1>();
    in_front =
        in_front && in_cam.dot(track.bearings.row(static_cast<Eigen::Index>(v)).transpose()) > 0.0;
    for (std::size_t m = j + 1; m < track.seen.size(); ++m) {
      const Eigen::Vector3d other = point - track.centres[track.seen[m]];
      parallax = std::max(parallax, angle(point - track.centres[v], other));
    }
  }
  return {in_front, parallax};
}

}  // namespace

Result<Triangulation> two_view(const Eigen::Matrix4d& relative_pose,
                               const Eigen::Ref<const Rows3>& bearings1,
                               const Eigen::Ref<const Rows3>& bearings2) {
  if (auto checked = se3::check_transform(relative_pose); !checked) {
    return std::unexpected(checked.error());
  }
  const Eigen::Matrix3d rotation = relative_pose.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = relative_pose.topRightCorner<3, 1>();
  if (!(t.norm() > 0.0)) {
    return fail(ErrorKind::kInvalidInput,
                "relative_pose: zero translation; the depth is undetermined");
  }
  const Eigen::Index n = bearings1.rows();
  if (bearings2.rows() != n) {
    return fail(ErrorKind::kInvalidInput, "bearings: expected equal (P, 3) shapes");
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    if (auto ok = check_unit(bearings1, i, "bearings1"); !ok) {
      return std::unexpected(ok.error());
    }
    if (auto ok = check_unit(bearings2, i, "bearings2"); !ok) {
      return std::unexpected(ok.error());
    }
    if (!(bearings1(i, 2) > 0.0) || !(bearings2(i, 2) > 0.0)) {
      return fail(ErrorKind::kInvalidInput, "bearings: expected positive z (a pinhole view)");
    }
  }
  const Eigen::Matrix3d essential = so3::hat(t) * rotation;
  const Eigen::Matrix2d e2 = essential.topLeftCorner<2, 2>();
  const Eigen::Vector3d centre2 = -rotation.transpose() * t;
  Triangulation out = sized(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    const Eigen::Vector3d x1 = bearings1.row(i).transpose() / bearings1(i, 2);
    const Eigen::Vector3d x2 = bearings2.row(i).transpose() / bearings2(i, 2);
    // niter2, with the paper's (x, x') = (x2, x1): the constraint is x2^T E x1 = 0.
    Eigen::Vector2d n2 = (essential * x1).head<2>();
    Eigen::Vector2d n1 = (essential.transpose() * x2).head<2>();
    const double a = n2.dot(e2 * n1);
    const double b = 0.5 * (n2.squaredNorm() + n1.squaredNorm());
    const double c = x2.dot(essential * x1);
    const double d = std::sqrt(std::max((b * b) - (a * c), 0.0));
    double lambda = c / (b + d);
    const Eigen::Vector2d step2 = lambda * n2;
    const Eigen::Vector2d step1 = lambda * n1;
    n2 -= e2 * step1;
    n1 -= e2.transpose() * step2;
    lambda *= 2.0 * d / (n2.squaredNorm() + n1.squaredNorm());
    Eigen::Vector3d x2c = x2;
    Eigen::Vector3d x1c = x1;
    x2c.head<2>() -= lambda * n2;
    x1c.head<2>() -= lambda * n1;
    // Intersect the corrected rays: d2 x2c = d1 R x1c + t.
    const Eigen::Vector3d rotated = rotation * x1c;
    const Eigen::Vector3d across = x2c.cross(rotated);
    const bool parallel = !(across.norm() > kParallelSine * x2c.norm() * rotated.norm());
    const double depth1 = -x2c.cross(t).dot(across) / across.squaredNorm();
    const Eigen::Vector3d point = depth1 * x1c;
    const bool in_front = point.dot(bearings1.row(i).transpose()) > 0.0 &&
                          (rotation * point + t).dot(bearings2.row(i).transpose()) > 0.0;
    const bool bad = parallel || !point.allFinite();
    store(out, i, point, in_front, angle(point, point - centre2), bad);
  }
  return out;
}

Result<Triangulation> n_view(const std::vector<Eigen::Matrix4d>& poses,
                             const Eigen::Ref<const Rows3>& bearings, const MaskRef& observed) {
  auto centres = check_n_view(poses, bearings, observed);
  if (!centres) {
    return std::unexpected(centres.error());
  }
  const Eigen::Index n = observed.rows();
  const auto views = static_cast<Eigen::Index>(poses.size());
  Triangulation out = sized(n);
  std::vector<std::size_t> seen;
  for (Eigen::Index p = 0; p < n; ++p) {
    seen.clear();
    for (Eigen::Index v = 0; v < views; ++v) {
      if (observed(p, v)) {
        seen.push_back(static_cast<std::size_t>(v));
      }
    }
    const Track track{.poses = poses,
                      .centres = *centres,
                      .bearings = bearings.middleRows(p * views, views),
                      .seen = seen};
    const auto [point, undetermined] = dlt(track);
    const auto [in_front, parallax] = cheirality_and_parallax(track, point);
    store(out, p, point, in_front, parallax, undetermined || !point.allFinite());
  }
  return out;
}

}  // namespace slamlab::foundations::triangulation
