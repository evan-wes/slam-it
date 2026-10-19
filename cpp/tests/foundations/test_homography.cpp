/// @file
/// @brief Native tests for homographies (slam_it/foundations/homography.hpp): the DLT on a
///        plane, the Sampson error, the decomposition, and collinear points.
#include <Eigen/Core>
#include <cstddef>

#include "scene_support.hpp"
#include "slam_it/foundations/homography.hpp"
#include "test_support.hpp"

int main() {
  namespace homography = slam_it::foundations::homography;
  using slam_it::testing::expect;
  using slam_it::testing::up_to_sign;
  int failures = 0;
  // The plane z = 5 in camera 1, seen after the scenes' motion.
  const Eigen::Vector3d t = slam_it::testing::scene_translation();
  const Eigen::Matrix3d rotation = slam_it::testing::scene_rotation();
  homography::Rows2 x1(20, 2);
  homography::Rows2 x2(20, 2);
  for (int i = 0; i < 20; ++i) {
    const Eigen::Vector3d x = slam_it::testing::scene_point(i, true);
    const Eigen::Vector3d y = (rotation * x) + t;
    x1.row(i) = x.head<2>().transpose() / x(2);
    x2.row(i) = y.head<2>().transpose() / y(2);
  }
  // The plane z = 5 in camera 1: n = (0, 0, 1), d = 5.
  const Eigen::Matrix3d truth =
      (rotation + (t * Eigen::Vector3d::UnitZ().transpose() / 5.0)).normalized();
  const auto h = homography::homography_dlt(x1, x2);
  expect(h.has_value() && up_to_sign(*h, truth) < 1e-12, "H = R + t n^T / d", failures);
  expect(homography::sampson(truth, x1, x2).maxCoeff() < 1e-25,
         "the Sampson error vanishes on the model", failures);

  const auto motions = homography::decompose_homography(-2.0 * truth);
  bool has_plane = false;
  if (motions) {
    for (std::size_t i = 0; i < motions->poses.size(); ++i) {
      const Eigen::Matrix4d& pose = motions->poses[i];
      has_plane =
          has_plane || ((pose.topLeftCorner<3, 3>() - rotation).cwiseAbs().maxCoeff() < 1e-10 &&
                        (pose.topRightCorner<3, 1>() - (t / 5.0)).norm() < 1e-10 &&
                        (motions->normals[i] - Eigen::Vector3d::UnitZ()).norm() < 1e-10);
    }
  }
  expect(motions.has_value() && motions->poses.size() == 4 && has_plane,
         "H's candidates include the true motion and plane", failures);

  homography::Rows2 line(4, 2);
  line << 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 0.0, 1.0;
  expect(!homography::homography_dlt(line, line).has_value(),
         "three collinear points do not determine H", failures);
  return slam_it::testing::finish("foundations/homography", failures);
}
