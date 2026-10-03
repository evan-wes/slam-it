/// @file
/// @brief Native tests for epipolar geometry (slamlab/foundations/epipolar.hpp): exact E,
///        RANSAC with outliers, a planar scene's degeneracy, and the decomposition.
#include <Eigen/Core>
#include <cmath>
#include <string>
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
  const Eigen::Matrix3d truth = slamlab::testing::scene_essential();
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

  // Every minimal solver, refined or not, finds E and the clean pairs.
  for (const epipolar::MinimalSolver minimal :
       {epipolar::MinimalSolver::kNister, epipolar::MinimalSolver::kStewenius,
        epipolar::MinimalSolver::k8pt}) {
    for (const bool refine : {false, true}) {
      const auto found = epipolar::essential_ransac(b1, dirty, ransac::Options{.threshold = 1e-6},
                                                    minimal, refine);
      bool ok = found.has_value() && up_to_sign(found->state.model, truth) < 1e-9;
      for (int i = 0; ok && i < 40; ++i) {
        ok = found->state.inliers(i) == (i % 3 != 0);
      }
      const std::string what = std::string(epipolar::name(minimal)) + (refine ? " (refined)" : "") +
                               " finds E and the clean pairs";
      expect(ok, what.c_str(), failures);
    }
  }
  const auto unknown = epipolar::essential_ransac(b1, b2, ransac::Options{.threshold = 1e-6},
                                                  epipolar::MinimalSolver::kCount, true);
  expect(!unknown.has_value(), "an unknown minimal solver is rejected", failures);

  // On exact data the Sampson cost's minimum is the true E: refining reaches it from a
  // perturbed start.
  const Eigen::Matrix3d start =
      slamlab::foundations::so3::hat(
          slamlab::foundations::so3::exp(Eigen::Vector3d(0.0, 0.02, 0.0)) * t.normalized()) *
      slamlab::foundations::so3::exp(Eigen::Vector3d(0.01, 0.0, 0.0)) * rotation;
  const auto refined = epipolar::refine_essential(start, b1, b2, 1e-3);
  expect(
      refined.has_value() && refined->summary.converged && up_to_sign(refined->state, truth) < 1e-9,
      "refining recovers the true E", failures);
  const auto bad_scale = epipolar::refine_essential(start, b1, b2, 0.0);
  expect(!bad_scale.has_value(), "a non-positive scale is rejected", failures);
  return slamlab::testing::finish("foundations/epipolar", failures);
}
