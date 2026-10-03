/// @file
/// @brief Native tests for absolute pose (slamlab/foundations/pnp.hpp): EPnP exact on exact
///        data, the refinement converging, RANSAC through outliers, and bad input.
#include <Eigen/Dense>
#include <cmath>

#include "scene_support.hpp"
#include "slamlab/foundations/pnp.hpp"
#include "slamlab/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

namespace pnp = slamlab::foundations::pnp;
using slamlab::foundations::ErrorKind;
using slamlab::testing::expect;
using slamlab::testing::Rows3;

/// @brief Correspondences and the pose that explains them.
struct Scene {
  /// @brief World points.
  Rows3 points;
  /// @brief Their exact bearings.
  Rows3 bearings;
  /// @brief The true `T_cam_world`.
  Eigen::Matrix4d truth;
};

/// @brief The deterministic scene's points as world points, seen by the camera at the
///        scene's motion.
/// @param n How many points.
/// @param planar Put the points on the plane z = 5.
/// @return The scene.
Scene scene(int n, bool planar) {
  Scene out{.points = Rows3(n, 3), .bearings = Rows3(), .truth = Eigen::Matrix4d::Identity()};
  out.truth.topLeftCorner<3, 3>() = slamlab::testing::scene_rotation();
  out.truth.topRightCorner<3, 1>() = slamlab::testing::scene_translation();
  Rows3 unused;
  slamlab::testing::scene_bearings(
      n, planar, {slamlab::testing::scene_rotation(), slamlab::testing::scene_translation()},
      unused, out.bearings);
  for (int i = 0; i < n; ++i) {
    out.points.row(i) = slamlab::testing::scene_point(i, planar).transpose();
  }
  return out;
}

}  // namespace

int main() {
  int failures = 0;
  const Scene full = scene(40, false);
  const Rows3& points = full.points;
  const Rows3& bearings = full.bearings;
  const Eigen::Matrix4d& truth = full.truth;

  const auto all = pnp::epnp(points, bearings);
  expect(all.has_value() && (*all - truth).cwiseAbs().maxCoeff() < 1e-9,
         "EPnP returns the true pose of 40 exact points", failures);
  const auto five = pnp::epnp(points.topRows(5), bearings.topRows(5));
  expect(five.has_value() && (*five - truth).cwiseAbs().maxCoeff() < 1e-9,
         "EPnP returns the true pose of 5 exact points", failures);

  Eigen::Matrix4d start = truth;
  start.topLeftCorner<3, 3>() =
      slamlab::foundations::so3::exp(Eigen::Vector3d(0.01, -0.02, 0.015)) *
      truth.topLeftCorner<3, 3>();
  start.topRightCorner<3, 1>() += Eigen::Vector3d(0.03, -0.02, 0.05);
  const auto refined = pnp::refine_pose(start, points, bearings, 1e-3);
  expect(refined.has_value() && refined->summary.converged &&
             (refined->state - truth).cwiseAbs().maxCoeff() < 1e-8,
         "the refinement converges to the true pose", failures);

  // Every third bearing replaced by a direction no pose explains.
  Rows3 dirty = bearings;
  for (Eigen::Index i = 0; i < dirty.rows(); i += 3) {
    dirty.row(i) =
        Eigen::RowVector3d(std::sin(1.7 * static_cast<double>(i)), 0.4, 1.0).normalized();
  }
  for (const auto solver : {slamlab::foundations::p3p::Solver::kDing,
                            slamlab::foundations::p3p::Solver::kLambdaTwist}) {
    const auto run = pnp::pnp_ransac(
        points, dirty, slamlab::foundations::ransac::Options{.threshold = 1e-6}, solver);
    bool clean = run.has_value();
    for (Eigen::Index i = 0; clean && i < dirty.rows(); ++i) {
      clean = run->state.inliers(i) == (i % 3 != 0);
    }
    expect(clean && (run->state.model - truth).cwiseAbs().maxCoeff() < 1e-9,
           "RANSAC returns the true pose and exactly the clean correspondences", failures);
  }

  const Scene flat = scene(20, true);
  const auto planar = pnp::epnp(flat.points, flat.bearings);
  expect(!planar.has_value() && planar.error().kind == ErrorKind::kDegenerate,
         "EPnP rejects planar points as degenerate", failures);
  const auto four = pnp::epnp(points.topRows(4), bearings.topRows(4));
  expect(!four.has_value() && four.error().kind == ErrorKind::kInvalidInput,
         "EPnP rejects four points", failures);
  Eigen::Matrix4d behind = truth;
  behind(2, 3) -= 20.0;
  const auto rejected = pnp::refine_pose(behind, points, bearings, 1e-3);
  expect(!rejected.has_value() && rejected.error().kind == ErrorKind::kInvalidInput,
         "the refinement rejects a start with points behind the camera", failures);
  return slamlab::testing::finish("foundations/pnp", failures);
}
