/// @file
/// @brief Native tests for triangulation (slam_it_all/foundations/triangulation.hpp): exact
///        recovery in both methods, parallel rays at infinity, and input checks.
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "slam_it_all/foundations/triangulation.hpp"
#include "test_support.hpp"

namespace {

namespace triangulation = slam_it_all::foundations::triangulation;
using slam_it_all::testing::expect;
using Mask = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/// @brief A pose `[R | t]` from an axis-angle rotation and a translation.
/// @param angle The rotation angle.
/// @param axis The rotation axis.
/// @param t The translation.
/// @return The pose.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (axis, t) of an axis-angle pose
Eigen::Matrix4d pose(double angle, const Eigen::Vector3d& axis, const Eigen::Vector3d& t) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  out.topRightCorner<3, 1>() = t;
  return out;
}

/// @brief A point's unit bearing in a camera.
/// @param camera The camera's `T_cam_world`.
/// @param point The world point.
/// @return The bearing.
Eigen::RowVector3d bearing(const Eigen::Matrix4d& camera, const Eigen::Vector3d& point) {
  return ((camera.topLeftCorner<3, 3>() * point) + camera.topRightCorner<3, 1>())
      .normalized()
      .transpose();
}

}  // namespace

int main() {
  int failures = 0;
  const std::vector<Eigen::Matrix4d> poses = {Eigen::Matrix4d::Identity(),
                                              pose(0.1, {0.2, -1.0, 0.3}, {0.5, -0.1, 0.05}),
                                              pose(0.08, {-0.6, 1.0, 0.0}, {-0.4, 0.2, 0.1})};
  const std::vector<Eigen::Vector3d> points = {{0.3, -0.2, 5.0}, {-1.5, 1.0, 7.0}, {1.0, 1.8, 4.2}};
  triangulation::Rows3 b1(3, 3);
  triangulation::Rows3 b2(3, 3);
  triangulation::Rows3 all(9, 3);
  for (int i = 0; i < 3; ++i) {
    b1.row(i) = bearing(poses[0], points[i]);
    b2.row(i) = bearing(poses[1], points[i]);
    for (int v = 0; v < 3; ++v) {
      all.row((3 * i) + v) = bearing(poses[v], points[i]);
    }
  }
  const auto pair = triangulation::two_view(poses[1], b1, b2);
  const Mask observed = Mask::Constant(3, 3, true);
  const auto many = triangulation::n_view(poses, all, observed);
  bool exact = pair.has_value() && many.has_value();
  for (int i = 0; exact && i < 3; ++i) {
    exact = (pair->points.row(i).transpose() - points[i]).norm() < 1e-12 &&
            (many->points.row(i).transpose() - points[i]).norm() < 1e-12 && pair->in_front(i) &&
            many->in_front(i);
  }
  expect(exact, "noise-free points are recovered by both methods", failures);

  // Camera 2 one metre ahead: a point on the optical axis has parallel rays.
  const Eigen::Matrix4d forward = pose(0.0, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0});
  triangulation::Rows3 axis(1, 3);
  axis << 0.0, 0.0, 1.0;
  const auto infinite = triangulation::two_view(forward, axis, axis);
  expect(infinite.has_value() && std::isnan(infinite->points(0, 0)) && !infinite->in_front(0) &&
             infinite->parallax(0) == 0.0,
         "parallel rays give a point at infinity", failures);

  expect(!triangulation::two_view(Eigen::Matrix4d::Identity(), b1, b2).has_value(),
         "a zero translation is rejected", failures);
  expect(!triangulation::n_view(poses, all, Mask::Constant(3, 3, false)).has_value(),
         "a point seen once is rejected", failures);
  return slam_it_all::testing::finish("foundations/triangulation", failures);
}
