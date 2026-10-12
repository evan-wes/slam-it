/// @file
/// @brief Native tests for two-view initialisation (slam_it/foundations/twoview.hpp): a
///        general scene picks E, a plane picks H, a rotation does not initialise.
#include <Eigen/Geometry>
#include <cmath>

#include "scene_support.hpp"
#include "slam_it/foundations/twoview.hpp"
#include "test_support.hpp"

namespace {

namespace twoview = slam_it::foundations::twoview;
namespace epipolar = slam_it::foundations::epipolar;
using slam_it::testing::expect;

}  // namespace

int main() {
  int failures = 0;
  const Eigen::Matrix3d rotation = slam_it::testing::scene_rotation();
  const Eigen::Vector3d t = slam_it::testing::scene_translation();
  const twoview::Options options{.focal = 520.0};
  epipolar::Rows3 b1;
  epipolar::Rows3 b2;

  slam_it::testing::scene_bearings(120, false, {rotation, t}, b1, b2);
  const auto general = twoview::initialize(b1, b2, options);
  expect(general.has_value() && general->status == twoview::Status::kInitialized &&
             general->model == twoview::Model::kEssential &&
             (general->relative_pose.topRightCorner<3, 1>() - t.normalized()).norm() < 1e-9,
         "a general scene initialises with E and the true motion", failures);

  slam_it::testing::scene_bearings(120, true, {rotation, t}, b1, b2);
  const auto plane = twoview::initialize(b1, b2, options);
  expect(plane.has_value() && plane->status == twoview::Status::kInitialized &&
             plane->model == twoview::Model::kHomography,
         "a plane initialises with H", failures);

  slam_it::testing::scene_bearings(120, false, {rotation, Eigen::Vector3d::Zero()}, b1, b2);
  const auto spin = twoview::initialize(b1, b2, options);
  expect(spin.has_value() && spin->status == twoview::Status::kPureRotation &&
             !spin->triangulated.any(),
         "a pure rotation does not initialise", failures);

  expect(!twoview::check_options(twoview::Options{.focal = 0.0}).has_value(),
         "a zero focal length is rejected", failures);
  expect(std::abs(twoview::kHomographyThresholdScale - std::sqrt(5.991 / 3.841)) < 1e-15,
         "H's threshold scale is the ratio of the 95% chi-squared quantiles", failures);
  return slam_it::testing::finish("foundations/twoview", failures);
}
