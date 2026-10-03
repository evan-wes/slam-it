/// @file
/// @brief Native tests for the P3P solvers (slamlab/foundations/p3p.hpp): the true pose among
///        each solver's solutions, every solution a rigid motion, the cubic root, bad input.
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

#include "scene_support.hpp"
#include "slamlab/foundations/p3p.hpp"
#include "test_support.hpp"

namespace {

namespace p3p = slamlab::foundations::p3p;
using slamlab::testing::expect;
using slamlab::testing::Rows3;

/// @brief Whether a solver finds the true pose among rigid solutions, on three points.
/// @param points Three world points.
/// @param bearings Their bearings.
/// @param truth The true `T_cam_world`.
/// @param solver The solver.
/// @return True if one solution is the truth and every solution is a rotation.
bool finds_truth(const Rows3& points, const Rows3& bearings, const Eigen::Matrix4d& truth,
                 p3p::Solver solver) {
  const auto found = p3p::p3p(points, bearings, solver);
  if (!found.has_value() || found->empty()) {
    return false;
  }
  bool has_truth = false;
  for (const Eigen::Matrix4d& pose : *found) {
    const Eigen::Matrix3d rotation = pose.topLeftCorner<3, 3>();
    if ((rotation * rotation.transpose() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
            1e-9 ||
        !(rotation.determinant() > 0.0)) {
      return false;
    }
    has_truth = has_truth || (pose - truth).cwiseAbs().maxCoeff() < 1e-9;
  }
  return has_truth;
}

}  // namespace

int main() {
  int failures = 0;
  // The deterministic scene's points, seen by the camera at T_cam_world = (R, t).
  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  truth.topLeftCorner<3, 3>() = slamlab::testing::scene_rotation();
  truth.topRightCorner<3, 1>() = slamlab::testing::scene_translation();
  Rows3 points;
  Rows3 bearings;
  slamlab::testing::scene_bearings(
      30, false, {slamlab::testing::scene_rotation(), slamlab::testing::scene_translation()},
      points, bearings);
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    points.row(i) = slamlab::testing::scene_point(static_cast<int>(i), false).transpose();
  }
  for (const p3p::Solver solver : {p3p::Solver::kDing, p3p::Solver::kLambdaTwist}) {
    bool all = true;
    for (Eigen::Index start = 0; start + 3 <= points.rows(); start += 3) {
      all = all &&
            finds_truth(points.middleRows(start, 3), bearings.middleRows(start, 3), truth, solver);
    }
    const std::string what = std::string(p3p::name(solver)) + " finds the pose on every sample";
    expect(all, what.c_str(), failures);
  }

  // (z - 2)(z + 1)(z - 0.5): three real roots, the largest 2.
  const auto [three, three_single] = p3p::cubic_root(-1.5, -1.5, 1.0);
  expect(std::abs(three - 2.0) < 1e-12 && !three_single, "the largest of three real roots",
         failures);
  // (z + 4)(z^2 - z + 2): one real root.
  const auto [one, one_single] = p3p::cubic_root(3.0, -2.0, 8.0);
  expect(std::abs(one + 4.0) < 1e-12 && one_single, "a cubic's only real root", failures);

  Rows3 line(3, 3);
  line << 0.0, 0.0, 5.0, 1.0, 0.5, 6.0, 2.0, 1.0, 7.0;
  Rows3 line_rays = line.rowwise().normalized();
  for (const p3p::Solver solver : {p3p::Solver::kDing, p3p::Solver::kLambdaTwist}) {
    const auto collinear = p3p::p3p(line, line_rays, solver);
    expect(collinear.has_value() && collinear->empty(), "collinear points give no solution",
           failures);
  }

  const auto four = p3p::p3p(points.topRows(4), bearings.topRows(4), p3p::Solver::kDing);
  expect(!four.has_value() && four.error().kind == slamlab::foundations::ErrorKind::kInvalidInput,
         "four correspondences are rejected", failures);
  const auto unknown = p3p::p3p(points.topRows(3), bearings.topRows(3), p3p::Solver::kCount);
  expect(!unknown.has_value(), "an unknown solver is rejected", failures);
  const Rows3 long_rays = 2.0 * bearings.topRows(3);
  const auto not_unit = p3p::p3p(points.topRows(3), long_rays, p3p::Solver::kDing);
  expect(!not_unit.has_value(), "bearings that are not unit are rejected", failures);
  return slamlab::testing::finish("foundations/p3p", failures);
}
