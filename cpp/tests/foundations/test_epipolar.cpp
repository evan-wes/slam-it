/// @file
/// @brief Native tests for epipolar geometry (slamlab/foundations/epipolar.hpp): exact E,
///        RANSAC with outliers, a planar scene's degeneracy, and the decomposition.
#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "scene_support.hpp"
#include "slamlab/foundations/epipolar.hpp"
#include "slamlab/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

namespace epipolar = slamlab::foundations::epipolar;
namespace ransac = slamlab::foundations::ransac;
using slamlab::testing::expect;
using slamlab::testing::up_to_sign;

}  // namespace

int main() {
  int failures = 0;
  const Eigen::Matrix3d rotation = slamlab::testing::scene_rotation();
  const Eigen::Vector3d t = slamlab::testing::scene_translation();
  const Eigen::Matrix3d truth = slamlab::foundations::so3::hat(t.normalized()) * rotation;
  epipolar::Rows3 b1;
  epipolar::Rows3 b2;
  slamlab::testing::scene_bearings(40, false, {rotation, t}, b1, b2);
  const auto e = epipolar::essential_8pt(b1, b2);
  expect(e.has_value() && up_to_sign(*e, truth) < 1e-12, "E is [t]x R up to sign", failures);

  // Every third pair an outlier; RANSAC must find E and exactly the clean pairs.
  epipolar::Rows3 dirty = b2;
  for (int i = 0; i < 40; i += 3) {
    dirty.row(i) = Eigen::Vector3d(std::sin(5.0 * i), std::cos(3.0 * i), 2.0).normalized();
  }
  const auto run = epipolar::essential_ransac(b1, dirty, ransac::Options{.threshold = 1e-6});
  bool clean = run.has_value() && up_to_sign(run->state.model, truth) < 1e-10;
  for (int i = 0; clean && i < 40; ++i) {
    clean = run->state.inliers(i) == (i % 3 != 0);
  }
  expect(clean, "E-RANSAC finds E and exactly the clean pairs", failures);

  epipolar::Rows3 p1;
  epipolar::Rows3 p2;
  slamlab::testing::scene_bearings(20, true, {rotation, t}, p1, p2);
  const auto planar = epipolar::essential_8pt(p1, p2);
  expect(!planar.has_value() && planar.error().kind == slamlab::foundations::ErrorKind::kDegenerate,
         "a planar scene does not determine E", failures);

  const auto poses = epipolar::decompose_essential(truth);
  bool has_truth = false;
  for (const Eigen::Matrix4d& pose : poses.value_or(std::vector<Eigen::Matrix4d>{})) {
    has_truth =
        has_truth || ((pose.topLeftCorner<3, 3>() - rotation).cwiseAbs().maxCoeff() < 1e-12 &&
                      (pose.topRightCorner<3, 1>() - t.normalized()).norm() < 1e-12);
  }
  expect(poses.has_value() && poses->size() == 4 && has_truth,
         "E's four candidates include the true pose", failures);
  return slamlab::testing::finish("foundations/epipolar", failures);
}
