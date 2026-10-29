/// @file
/// @brief Native tests for slam_it::foundations::ba, runnable without Python.
#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "slam_it/foundations/ba.hpp"
#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

namespace ba = slam_it::foundations::ba;
namespace camera = slam_it::foundations::camera;
namespace se3 = slam_it::foundations::se3;
namespace so3 = slam_it::foundations::so3;
using slam_it::foundations::ErrorKind;
using slam_it::testing::expect;

constexpr int kCameras = 4;
constexpr int kPoints = 30;

/// @brief Give a problem's prior arrays their defaults: no depth priors, fixed unit affines,
///        no pose priors.
/// @param problem The problem.
void without_priors(ba::Problem& problem) {
  const Eigen::Index n_obs = problem.observations.rows();
  const auto n_cams = static_cast<Eigen::Index>(problem.poses.size());
  problem.depth_prior = Eigen::VectorXd::Constant(n_obs, std::numeric_limits<double>::quiet_NaN());
  problem.depth_prior_weights = Eigen::VectorXd::Ones(n_obs);
  problem.depth_affine.resize(n_cams, 2);
  problem.depth_affine.col(0).setOnes();
  problem.depth_affine.col(1).setZero();
  problem.fixed_depth_affine.assign(problem.poses.size(), true);
  problem.pose_prior = problem.poses;
  problem.pose_prior_weights = Eigen::MatrixX2d::Zero(n_cams, 2);
  problem.point_anchor.assign(static_cast<std::size_t>(problem.points.rows()), -1);
}

/// @brief A deterministic scene: cameras along x, points in a box ahead, every point seen.
/// The start perturbs the poses and points; the observations are noise-free.
/// @return The problem, the true points, and each observation's true inverse depth.
std::tuple<ba::Problem, ba::Points, Eigen::VectorXd> scene() {
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
  Eigen::VectorXd inverse_depth(problem.observations.rows());
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
      inverse_depth(row) = 1.0 / in_camera.z();
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
  problem.fixed_intrinsics.assign(4, true);  // one group, four parameters
  problem.fixed_points.assign(kPoints, false);
  without_priors(problem);
  return {problem, truth, inverse_depth};
}

/// @brief Points 1e-9 m in front of a held camera, at its centre (advio-01's local BA, B1):
///        camera 0, held, turned and off the origin, sees 101 points within 1e-16 m of 1e-9 m in
///        front of it (`camera::kMinDepth`) and 12 more two to three metres out; cameras 1 and
///        2, free, a metre behind it and 0.2 m to either side, see all of them; every point is
///        held.
/// @return The problem.
ba::Problem camera_plane_problem() {
  const Eigen::Matrix3d rotation = so3::exp(Eigen::Vector3d(0.3, -0.5, 0.7));
  const Eigen::Vector3d centre(-0.0145, -0.0038, -0.0345);
  std::vector<Eigen::Vector3d> local;
  local.reserve(113);  // 101 near, 12 far
  for (int k = 0; k < 101; ++k) {
    local.emplace_back(3.49e-11, 1.458e-10, 1e-9 + (static_cast<double>(k - 50) * 2e-18));
  }
  for (const double x : {-1.0, 0.0, 1.0}) {
    for (const double y : {-0.5, 0.5}) {
      for (const double z : {2.0, 3.0}) {
        local.emplace_back(x, y, z);
      }
    }
  }
  const auto n_points = static_cast<Eigen::Index>(local.size());
  ba::Problem problem;
  problem.model = camera::Model::kPinhole;
  problem.intrinsics.resize(1, 4);
  problem.intrinsics << 500.0, 505.0, 320.0, 240.0;
  problem.points.resize(n_points, 3);
  for (Eigen::Index p = 0; p < n_points; ++p) {
    problem.points.row(p) = (centre + (rotation * local[static_cast<std::size_t>(p)])).transpose();
  }
  for (const Eigen::Vector3d& offset :
       {Eigen::Vector3d::Zero().eval(), Eigen::Vector3d(-0.2, 0.0, 1.0),
        Eigen::Vector3d(0.2, 0.0, 1.0)}) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.topLeftCorner<3, 3>() = rotation;
    pose.topRightCorner<3, 1>() = centre - (rotation * offset);
    problem.poses.push_back(pose);
  }
  problem.observations.resize(3 * n_points, 2);
  for (int c = 0; c < 3; ++c) {
    const Eigen::Matrix4d t_cam_world = *se3::inverse(problem.poses[static_cast<std::size_t>(c)]);
    for (Eigen::Index p = 0; p < n_points; ++p) {
      // Camera 0's as meant; by hand, as camera::project refuses those behind 1e-9 m.
      const Eigen::Vector3d x = c == 0 ? local[static_cast<std::size_t>(p)]
                                       : Eigen::Vector3d((t_cam_world.topLeftCorner<3, 3>() *
                                                          problem.points.row(p).transpose()) +
                                                         t_cam_world.topRightCorner<3, 1>());
      problem.observations.row((c * n_points) + p) << (500.0 * x.x() / x.z()) + 320.0,
          (505.0 * x.y() / x.z()) + 240.0;
      problem.cam_idx.emplace_back(c);
      problem.pt_idx.emplace_back(p);
    }
  }
  problem.intrinsics_idx.assign(3, 0);
  problem.weights = Eigen::VectorXd::Ones(problem.observations.rows());
  problem.fixed_poses = {true, false, false};
  problem.fixed_intrinsics.assign(4, true);
  problem.fixed_points.assign(static_cast<std::size_t>(n_points), true);
  without_priors(problem);
  return problem;
}

}  // namespace

int main() {
  int failures = 0;
  const auto [problem, truth, inverse_depth] = scene();
  ba::Options options;
  options.solver.function_tolerance = 1e-12;
  options.solver.max_iterations = 100;

  // Noise-free observations: the solve drives every reprojection error to zero, keeps the
  // gauge pose bit-exact, and confines one pose to remove the scale.
  const auto solved = ba::solve(problem, options);
  expect(solved.has_value() && solved->summary.solve.converged &&
             solved->summary.solve.final_cost < 1e-16,
         "solve: converges to zero reprojection error", failures);
  expect(solved.has_value() && solved->summary.gauge_pose == 0 &&
             solved->summary.confined_pose == 3 && solved->poses[0] == problem.poses[0],
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
  without_priors(lonely);
  const auto dropped = ba::solve(lonely, options);
  expect(dropped.has_value() && dropped->summary.dropped_points == 1 &&
             dropped->summary.dropped_observations == 1 &&
             dropped->points.row(kPoints) == lonely.points.row(kPoints),
         "solve: a single-view point is dropped and counted", failures);

  // Metric depth priors fix the scale reprojection leaves free: from a start scaled 1.5x
  // about the first camera, the auto gauge fixes only that camera and the truth returns.
  ba::Problem scaled = problem;
  const Eigen::Vector3d origin = scaled.poses[0].topRightCorner<3, 1>();
  for (Eigen::Matrix4d& pose : scaled.poses) {
    pose.topRightCorner<3, 1>() = origin + (1.5 * (pose.topRightCorner<3, 1>() - origin));
  }
  scaled.points =
      (1.5 * (scaled.points.rowwise() - origin.transpose())).rowwise() + origin.transpose();
  scaled.depth_prior = inverse_depth;
  const auto metric = ba::solve(scaled, options);
  expect(metric.has_value() && metric->summary.gauge == ba::Gauge::kFirstPose &&
             (metric->points - truth).cwiseAbs().maxCoeff() < 1e-6,
         "solve: metric depth priors recover the true scale", failures);

  // Every point anchored at its first observation as an inverse depth: noise-free data still
  // reaches zero cost, and each inverse depth is its point's 1/z in the anchor camera.
  ba::Problem anchored = problem;
  for (int pt = 0; pt < kPoints; ++pt) {
    anchored.point_anchor[static_cast<std::size_t>(pt)] = pt;  // camera 0's observation
  }
  const auto inverse = ba::solve(anchored, options);
  bool inverse_ok = inverse.has_value() && inverse->summary.solve.converged &&
                    inverse->summary.solve.final_cost < 1e-16;
  for (int pt = 0; inverse_ok && pt < kPoints; ++pt) {
    const Eigen::Matrix4d host = *se3::inverse(inverse->poses[0]);
    const Eigen::Vector3d in_host =
        (host.topLeftCorner<3, 3>() * inverse->points.row(pt).transpose()) +
        host.topRightCorner<3, 1>();
    inverse_ok = std::abs((inverse->inverse_depth(pt) * in_host.z()) - 1.0) < 1e-12;
  }
  expect(inverse_ok, "solve: anchored inverse-depth points converge", failures);

  // The core checks indices itself, for callers that bypass Python.
  ba::Problem bad = problem;
  bad.pt_idx[0] = kPoints + 5;
  const auto rejected = ba::solve(bad, options);
  expect(!rejected.has_value() && rejected.error().kind == ErrorKind::kInvalidInput,
         "solve: an out-of-range point index is rejected", failures);

  // Lengths are checked before anything indexes them: a short pt_idx with an anchor is
  // rejected, not read out of bounds.
  ba::Problem short_index = problem;
  short_index.point_anchor[0] = 0;
  short_index.pt_idx.resize(3);
  const auto shortened = ba::solve(short_index, options);
  expect(!shortened.has_value() && shortened.error().kind == ErrorKind::kInvalidInput,
         "solve: a short pt_idx is rejected before the anchors read it", failures);

  // With noisy observations and free intrinsics, the variance factor is positive and the
  // scaled deviations are the unscaled ones times its square root.
  ba::Problem noisy = problem;
  noisy.fixed_intrinsics.assign(4, false);
  for (Eigen::Index row = 0; row < noisy.observations.rows(); ++row) {
    noisy.observations(row, 0) += 0.5 * std::sin(3.7 * static_cast<double>(row));
    noisy.observations(row, 1) += 0.5 * std::cos(2.3 * static_cast<double>(row));
  }
  ba::Options unscaled = options;
  unscaled.scale_deviations = false;
  const auto scaled_fit = ba::solve(noisy, options);
  const auto raw_fit = ba::solve(noisy, unscaled);
  bool scaled_ok = scaled_fit.has_value() && raw_fit.has_value() &&
                   std::isfinite(scaled_fit->summary.variance_factor) &&
                   scaled_fit->summary.variance_factor > 0.0;
  if (scaled_ok) {
    const double root = std::sqrt(scaled_fit->summary.variance_factor);
    for (Eigen::Index k = 0; k < 4; ++k) {
      const double want = root * raw_fit->intrinsics_std(0, k);
      scaled_ok = scaled_ok && std::abs(scaled_fit->intrinsics_std(0, k) - want) <= 1e-9 * want;
    }
  }
  expect(scaled_ok, "solve: deviations scale by the root of the variance factor", failures);

  // The in-front test and the residuals take a point's depth from one computation, so an
  // observation the projection would refuse is left out at the start: B1's Rust inline run of
  // advio-01 kept one, 1e-9 m in front of its camera by the in-front test's formula and not by
  // the projection's, and the solve refused its infinite starting cost.
  const auto plane = ba::solve(camera_plane_problem(), ba::Options{});
  expect(plane.has_value() && plane->summary.dropped_observations >= 1 &&
             plane->summary.dropped_observations <= 101 &&
             std::isfinite(plane->summary.solve.final_cost),
         "solve: a point at a camera's centre is left out at the start", failures);

  return slam_it::testing::finish("test_ba", failures);
}
