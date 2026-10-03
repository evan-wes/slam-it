/// @file
/// @brief Native tests for two-view initialisation (slamlab/foundations/twoview.hpp): a
///        general scene picks E, a plane picks H, a rotation does not initialise.
#include <Eigen/Geometry>
#include <cmath>

#include "scene_support.hpp"
#include "slamlab/foundations/twoview.hpp"
#include "test_support.hpp"

namespace {

namespace twoview = slamlab::foundations::twoview;
namespace epipolar = slamlab::foundations::epipolar;
using slamlab::testing::expect;

/// @brief Exact bearing pairs of deterministic points through `[R | t]`.
/// @param rotation R.
/// @param t t.
/// @param planar Put the points on the plane z = 5.
/// @param b1 Bearings in camera 1 (filled).
/// @param b2 Bearings in camera 2 (filled).
void scene(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& t, bool planar,
           epipolar::Rows3& b1, epipolar::Rows3& b2) {
  constexpr int kPoints = 120;
  b1.resize(kPoints, 3);
  b2.resize(kPoints, 3);
  for (int i = 0; i < kPoints; ++i) {
    const Eigen::Vector3d x = slamlab::testing::scene_point(i, planar);
    b1.row(i) = x.normalized().transpose();
    b2.row(i) = ((rotation * x) + t).normalized().transpose();
  }
}

}  // namespace

int main() {
  int failures = 0;
  const Eigen::Matrix3d rotation = slamlab::testing::scene_rotation();
  const Eigen::Vector3d t = slamlab::testing::scene_translation();
  const twoview::Options options{.focal = 520.0};
  epipolar::Rows3 b1;
  epipolar::Rows3 b2;

  scene(rotation, t, false, b1, b2);
  const auto general = twoview::initialize(b1, b2, options);
  expect(general.has_value() && general->status == twoview::Status::kInitialized &&
             general->model == twoview::Model::kEssential &&
             (general->relative_pose.topRightCorner<3, 1>() - t.normalized()).norm() < 1e-9,
         "a general scene initialises with E and the true motion", failures);

  scene(rotation, t, true, b1, b2);
  const auto plane = twoview::initialize(b1, b2, options);
  expect(plane.has_value() && plane->status == twoview::Status::kInitialized &&
             plane->model == twoview::Model::kHomography,
         "a plane initialises with H", failures);

  scene(rotation, Eigen::Vector3d::Zero(), false, b1, b2);
  const auto spin = twoview::initialize(b1, b2, options);
  expect(spin.has_value() && spin->status == twoview::Status::kPureRotation &&
             !spin->triangulated.any(),
         "a pure rotation does not initialise", failures);

  expect(!twoview::check_options(twoview::Options{.focal = 0.0}).has_value(),
         "a zero focal length is rejected", failures);
  return slamlab::testing::finish("foundations/twoview", failures);
}
