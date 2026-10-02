/// @file
/// @brief Native tests for slamlab::foundations::ba, runnable without Python.
#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "slamlab/foundations/ba.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/se3.hpp"
#include "test_support.hpp"

namespace {

namespace ba = slamlab::foundations::ba;
namespace camera = slamlab::foundations::camera;
namespace se3 = slamlab::foundations::se3;
using slamlab::foundations::ErrorKind;
using slamlab::testing::expect;

constexpr int kCameras = 4;
constexpr int kPoints = 30;

/// @brief A deterministic scene: cameras along x, points in a box ahead, every point seen.
/// The start perturbs the poses and points; the observations are noise-free.
/// @return The problem and the true points.
std::pair<ba::Problem, ba::Points> scene() {
  ba::Problem problem;
  problem.model = camera::Model::kPinhole;
  problem.intrinsics.resize(1, 4);
  problem.intrinsics << 500.0, 505.0, 320.0, 240.0;
  ba::Points truth(kPoints, 3);
  for (int p = 0; p < kPoints; ++p) {
    truth.row(p) << 2.0 * std::sin(1.3 * p), 1.5 * std::cos(0.7 * p),
        6.0 + (2.0 * std::sin(0.4 * p));
  }
  problem.points = truth.array() + 0.05;
  problem.observations.resize(static_cast<Eigen::Index>(kCameras) * kPoints, 2);
  for (int c = 0; c < kCameras; ++c) {
    se3::Vector6d twist;
    twist << 0.4 * c, 0.1 * std::sin(c), 0.0, 0.02 * c, -0.03 * c, 0.01;
    const Eigen::Matrix4d pose = se3::exp(twist);
    const se3::Vector6d nudge = se3::Vector6d::Constant(c == 0 ? 0.0 : 0.005);
    problem.poses.emplace_back(pose * se3::exp(nudge));
    const Eigen::Matrix4d t_cam_world = *se3::inverse(pose);
    for (int p = 0; p < kPoints; ++p) {
      const Eigen::Vector3d in_camera =
          (t_cam_world.topLeftCorner<3, 3>() * truth.row(p).transpose()) +
          t_cam_world.topRightCorner<3, 1>();
      const auto row = (static_cast<Eigen::Index>(c) * kPoints) + p;
      problem.observations.row(row) =
          camera::project(problem.model, problem.intrinsics.row(0).transpose(), in_camera)
              ->transpose();
      problem.cam_idx.emplace_back(c);
      problem.pt_idx.emplace_back(p);
    }
  }
  problem.intrinsics_idx.assign(kCameras, 0);
  problem.weights = Eigen::VectorXd::Ones(problem.observations.rows());
  problem.fixed_poses.assign(kCameras, false);
  problem.fixed_intrinsics.assign(1, true);
  problem.fixed_points.assign(kPoints, false);
  return {problem, truth};
}

}  // namespace

int main() {
  int failures = 0;
  const auto [problem, truth] = scene();
  ba::Options options;
  options.solver.function_tolerance = 1e-12;
  options.solver.max_iterations = 100;

  // Noise-free observations: the solve drives every reprojection error to zero, keeps the
  // gauge pose bit-exact, and confines one pose to remove the scale.
  const auto solved = ba::solve(problem, options);
  expect(solved.has_value() && solved->summary.solve.converged &&
             solved->summary.solve.final_cost < 1e-16,
         "solve: converges to zero reprojection error", failures);
  expect(solved.has_value() && solved->summary.gauge_pose == 0 && solved->summary.scale_pose == 3 &&
             solved->poses[0] == problem.poses[0],
         "solve: the similarity gauge fixes pose 0 and confines pose 3", failures);

  // A point seen once is dropped, with its observation, and returned unchanged.
  ba::Problem lonely = problem;
  lonely.points.conservativeResize(kPoints + 1, 3);
  lonely.points.row(kPoints) << 0.0, 0.0, 7.0;
  lonely.fixed_points.push_back(false);
  lonely.observations.conservativeResize(lonely.observations.rows() + 1, 2);
  lonely.observations.row(lonely.observations.rows() - 1) << 320.0, 240.0;
  lonely.cam_idx.push_back(0);
  lonely.pt_idx.push_back(kPoints);
  lonely.weights.conservativeResize(lonely.weights.size() + 1);
  lonely.weights(lonely.weights.size() - 1) = 1.0;
  const auto dropped = ba::solve(lonely, options);
  expect(dropped.has_value() && dropped->summary.dropped_points == 1 &&
             dropped->summary.dropped_observations == 1 &&
             dropped->points.row(kPoints) == lonely.points.row(kPoints),
         "solve: a single-view point is dropped and counted", failures);

  // The core checks indices itself, for callers that bypass Python.
  ba::Problem bad = problem;
  bad.pt_idx[0] = kPoints + 5;
  const auto rejected = ba::solve(bad, options);
  expect(!rejected.has_value() && rejected.error().kind == ErrorKind::kInvalidInput,
         "solve: an out-of-range point index is rejected", failures);

  return slamlab::testing::finish("test_ba", failures);
}
