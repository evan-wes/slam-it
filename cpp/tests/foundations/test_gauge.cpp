/// @file
/// @brief Native unit tests for the bundle-adjustment gauge (`ba/gauge.hpp`), mirroring
///        tests/foundations/test_ba_gauge.py.
#include <Eigen/Dense>
#include <vector>

#include "foundations/ba/gauge.hpp"
#include "slam_it_all/foundations/ba.hpp"
#include "test_support.hpp"

namespace {

namespace ba = slam_it_all::foundations::ba;
namespace detail = slam_it_all::foundations::ba::detail;
using slam_it_all::testing::expect;

/// @brief Five camera centres along a gentle arc.
/// @param offset Added to every centre.
/// @return The centres.
detail::Positions centres(const Eigen::Vector3d& offset = Eigen::Vector3d::Zero()) {
  return {Eigen::Vector3d(0.0, 0.0, 0.0) + offset, Eigen::Vector3d(0.4, 0.1, 0.0) + offset,
          Eigen::Vector3d(0.8, -0.1, 0.05) + offset, Eigen::Vector3d(1.2, 0.2, -0.05) + offset,
          Eigen::Vector3d(1.6, 0.0, 0.1) + offset};
}

/// @brief `anchors` over `cams`, in the frame the layout would build.
/// @param cams The centres.
/// @param anchors The anchors (their frame is set here).
/// @return The anchors.
detail::Anchors framed(const detail::Positions& cams, detail::Anchors anchors = {}) {
  detail::Positions scene = cams;
  scene.insert(scene.end(), anchors.points.begin(), anchors.points.end());
  anchors.frame = detail::frame_of(scene);
  return anchors;
}

/// @brief Anchors with pose 0 fixed (its centre and orientation pinned).
/// @return The anchors.
detail::Anchors pose_zero_fixed() {
  detail::Anchors anchors;
  anchors.centre_cams = {0};
  anchors.orientation_cams = {0};
  return anchors;
}

/// @brief Anchors with metric depth.
/// @return The anchors.
detail::Anchors metric_depth() {
  detail::Anchors anchors;
  anchors.metric = true;
  return anchors;
}

/// @brief Anchors with two fixed points.
/// @return The anchors.
detail::Anchors two_points() {
  detail::Anchors anchors;
  anchors.points = {Eigen::Vector3d(0.0, 0.0, 6.0), Eigen::Vector3d(1.0, 0.0, 6.0)};
  return anchors;
}

/// @brief Whether the remaining gauge has `dimension` directions and is named `name`.
/// @param anchors The anchors.
/// @param dimension The expected dimension.
/// @param name The expected name.
/// @return Whether both hold.
bool remains(const detail::Anchors& anchors, Eigen::Index dimension, ba::Gauge name) {
  const Eigen::MatrixXd null = detail::remaining(anchors, centres());
  return null.cols() == dimension && detail::classify(null) == name;
}

}  // namespace

int main() {
  int failures = 0;
  const detail::Positions cams = centres();
  const detail::IndexList all = {0, 1, 2, 3, 4};

  expect(remains(framed(cams), 7, ba::Gauge::kSimilarity), "remaining: nothing anchored", failures);
  expect(remains(framed(cams, metric_depth()), 6, ba::Gauge::kFirstPose),
         "remaining: metric depth leaves the rigid motions", failures);
  expect(remains(framed(cams, pose_zero_fixed()), 1, ba::Gauge::kScale),
         "remaining: a fixed pose leaves the scale", failures);
  expect(remains(framed(cams, two_points()), 1, ba::Gauge::kPartial),
         "remaining: two fixed points leave the rotation about their line", failures);

  const detail::Plan chosen = detail::plan(framed(cams), cams, all);
  expect(chosen.fixed == detail::IndexList{0} && chosen.confined == detail::IndexList{4} &&
             chosen.ranks == detail::IndexList{1} && chosen.free == 0,
         "plan: fix the first pose, confine the farthest", failures);

  const detail::Positions coincident = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  const detail::Plan stuck = detail::plan(framed(coincident, pose_zero_fixed()), coincident, {1});
  expect(stuck.fixed.empty() && stuck.confined.empty() && stuck.free == 1,
         "plan: a pose at the scale's centre cannot take it; it is counted", failures);

  const detail::Positions far = centres(Eigen::Vector3d(1e6, 3e5, 0.0));
  const detail::Plan far_plan = detail::plan(framed(far), far, all);
  expect(far_plan.fixed == chosen.fixed && far_plan.confined == chosen.confined &&
             far_plan.ranks == chosen.ranks && far_plan.free == 0,
         "plan: 1e6 m from the origin, the same plan", failures);

  const std::vector<Eigen::MatrixXd> bases =
      detail::confinement(framed(cams, pose_zero_fixed()), cams, chosen);
  bool orthonormal = bases.size() == 1 && bases[0].rows() == 6 && bases[0].cols() == 5;
  if (orthonormal) {
    orthonormal =
        (bases[0].transpose() * bases[0] - Eigen::MatrixXd::Identity(5, 5)).cwiseAbs().maxCoeff() <
        1e-14;
  }
  expect(orthonormal, "confinement: one orthonormal 6 x 5 basis for the scale pose", failures);

  return slam_it_all::testing::finish("test_gauge", failures);
}
