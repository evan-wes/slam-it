/// @file
/// @brief Native tests for the factor graphs (slam_it/foundations/graphs.hpp): the factors'
///        Jacobians against central differences, each solve recovering a known answer, how a
///        solve stops, and bad input.
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/graphs.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

namespace graphs = slam_it::foundations::graphs;
namespace camera = slam_it::foundations::camera;
namespace se3 = slam_it::foundations::se3;
namespace so3 = slam_it::foundations::so3;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief The central differences' step.
constexpr double kStep = 1e-6;
/// @brief A stereo rig's baseline times its focal length, in pixel-metres.
constexpr double kBf = 40.0;

/// @brief A pinhole camera's intrinsics.
Eigen::VectorXd pinhole() { return Eigen::Vector4d(520.0, 515.0, 320.0, 240.0); }

/// @brief A radtan camera's intrinsics (TUM fr1's).
Eigen::VectorXd radtan() {
  Eigen::VectorXd out(9);
  out << 517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633;
  return out;
}

/// @brief `[[s R, t], [0, 1]]`.
/// @param rotation The rotation.
/// @param translation The translation.
/// @param scale The scale.
/// @return The similarity.
Eigen::Matrix4d similarity(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation,
                           double scale) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = scale * rotation;
  out.topRightCorner<3, 1>() = translation;
  return out;
}

/// @brief A pose from rung 0's SE(3) exponential, its tangent `(rho, theta)`.
/// @param rho The translation part.
/// @param theta The rotation part.
/// @return `exp((rho, theta))`.
Eigen::Matrix4d pose(const Eigen::Vector3d& rho, const Eigen::Vector3d& theta) {
  se3::Vector6d xi;
  xi << rho, theta;
  return se3::exp(xi);
}

/// @brief Retract a motion on the right by GTSAM's tangent, to first order its retraction:
///        `(w, v)` moves `T` to `T exp((v, w))`; `(w, v, lambda)` composes a similarity with
///        `[[e^lambda exp(w), e^lambda v], [0, 1]]`.
/// @param motion The motion.
/// @param delta The tangent, 6 or 7 long.
/// @return The moved motion.
Eigen::Matrix4d retract(const Eigen::Matrix4d& motion, const Eigen::VectorXd& delta) {
  const Eigen::Vector3d w = delta.head<3>();
  const Eigen::Vector3d v = delta.segment<3>(3);
  if (delta.size() == graphs::kPoseDim) {
    return motion * pose(v, w);
  }
  const double grow = std::exp(delta(6));
  return motion * similarity(so3::exp(w), grow * v, grow);
}

/// @brief Differentiate a function at 0 by central differences, a column per coordinate.
/// @param function The function.
/// @param dimension The tangent's dimension.
/// @return The Jacobian.
Eigen::MatrixXd numeric(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& function,
                        int dimension) {
  Eigen::MatrixXd out;
  for (int k = 0; k < dimension; ++k) {
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(dimension);
    delta(k) = kStep;
    const Eigen::VectorXd column = (function(delta) - function(-delta)) / (2.0 * kStep);
    if (k == 0) {
      out.resize(column.size(), dimension);
    }
    out.col(k) = column;
  }
  return out;
}

/// @brief Whether two matrices agree within `atol + rtol |expected|`, entry by entry.
/// @param actual The actual.
/// @param expected The expected.
/// @param rtol The relative tolerance.
/// @param atol The absolute tolerance.
/// @return Whether they agree.
bool close(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected, double rtol,
           double atol) {
  return actual.rows() == expected.rows() && actual.cols() == expected.cols() &&
         ((actual - expected).array().abs() <= atol + (rtol * expected.array().abs())).all();
}

/// @brief The projection factor's Jacobians match central differences.
/// @param failures The running failure count.
void test_projection_jacobians(int& failures) {
  const Eigen::Matrix4d camera_pose = pose({0.1, -0.2, 0.05}, {0.2, -0.1, 0.3});
  const Eigen::Vector3d point =
      camera_pose.topLeftCorner<3, 3>() * Eigen::Vector3d(0.2, -0.1, 2.5) +
      camera_pose.topRightCorner<3, 1>();
  for (const auto& [model, intrinsics] : std::array{std::pair{camera::Model::kPinhole, pinhole()},
                                                    std::pair{camera::Model::kRadTan, radtan()}}) {
    for (const bool stereo : {false, true}) {
      Eigen::VectorXd measured = stereo ? Eigen::VectorXd(Eigen::Vector3d(300.0, 250.0, 290.0))
                                        : Eigen::VectorXd(Eigen::Vector2d(300.0, 250.0));
      const auto at =
          graphs::projection_residual(camera_pose, point, measured, model, intrinsics, kBf);
      expect(at.has_value() && at->in_front, "the point projects", failures);
      const auto on_pose = [&](const Eigen::VectorXd& delta) {
        return graphs::projection_residual(retract(camera_pose, delta), point, measured, model,
                                           intrinsics, kBf)
            ->residual;
      };
      const auto on_point = [&](const Eigen::VectorXd& delta) {
        return graphs::projection_residual(camera_pose, point + delta.head<3>(), measured, model,
                                           intrinsics, kBf)
            ->residual;
      };
      expect(close(at->d_pose, numeric(on_pose, graphs::kPoseDim), 1e-6, 1e-5),
             "d_pose is the residual's derivative on the pose", failures);
      expect(close(at->d_point, numeric(on_point, 3), 1e-6, 1e-5),
             "d_point is the residual's derivative on the point", failures);
    }
  }
}

/// @brief The similarity factors' Jacobians match central differences, both directions.
/// @param failures The running failure count.
void test_similarity_jacobians(int& failures) {
  const Eigen::Vector3d point2(0.3, -0.2, 2.4);
  const Eigen::Vector2d pixel1(350.0, 220.0);
  const Eigen::Vector2d pixel2(330.0, 230.0);
  for (const bool fixed_scale : {false, true}) {
    const Eigen::Matrix4d motion =
        similarity(so3::exp({0.05, -0.1, 0.02}), {0.1, -0.05, 0.2}, fixed_scale ? 1.0 : 1.3);
    const Eigen::Vector3d point1 = motion.topLeftCorner<3, 3>() * point2 +
                                   motion.topRightCorner<3, 1>() +
                                   Eigen::Vector3d(0.01, -0.02, 0.03);
    const graphs::SimilarityResiduals at = graphs::similarity_residuals(
        motion, point1, point2, pixel1, pixel2, camera::Model::kRadTan, radtan(), fixed_scale);
    expect(at.in_front1 && at.in_front2, "both carried points are in front", failures);
    const int dimension = fixed_scale ? graphs::kPoseDim : graphs::kSimilarityDim;
    const auto side = [&](bool first) {
      return [&, first](const Eigen::VectorXd& delta) -> Eigen::VectorXd {
        const graphs::SimilarityResiduals moved =
            graphs::similarity_residuals(retract(motion, delta), point1, point2, pixel1, pixel2,
                                         camera::Model::kRadTan, radtan(), fixed_scale);
        return first ? moved.residual1 : moved.residual2;
      };
    };
    expect(close(at.d_similarity1, numeric(side(true), dimension), 1e-6, 1e-5),
           "d_similarity1 is its residual's derivative", failures);
    expect(close(at.d_similarity2, numeric(side(false), dimension), 1e-6, 1e-5),
           "d_similarity2 is its residual's derivative", failures);
  }
}

/// @brief A point behind its camera costs a constant, as GTSAM's own factors make it, with no
///        gradient; a carried point behind its camera contributes nothing to a refinement.
/// @param failures The running failure count.
void test_behind_the_camera(int& failures) {
  const auto behind = graphs::projection_residual(Eigen::Matrix4d::Identity(), {0.0, 0.0, -1.0},
                                                  Eigen::Vector3d(1.0, 2.0, 3.0),
                                                  camera::Model::kPinhole, pinhole(), kBf);
  expect(behind.has_value() && !behind->in_front &&
             behind->residual.isApprox(Eigen::Vector3d::Constant(2.0 * pinhole()(0)), 0.0) &&
             behind->d_pose.isZero(0.0) && behind->d_point.isZero(0.0),
         "a point behind its camera costs 2 fx in each row, with zero Jacobians", failures);
  const graphs::SimilarityResiduals both = graphs::similarity_residuals(
      Eigen::Matrix4d::Identity(), {0.0, 0.0, 2.0}, {0.0, 0.0, -2.0}, Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(), camera::Model::kPinhole, pinhole(), false);
  expect(!both.in_front1 && both.in_front2 && both.residual1.isZero(0.0) &&
             both.d_similarity1.isZero(0.0),
         "a carried point behind its camera contributes nothing", failures);
}

/// @brief A ring of camera-to-world poses round the origin, scaled with `with_scale`.
/// @param count How many.
/// @param with_scale Give each a scale, as a monocular map's drift does.
/// @return The poses.
std::vector<Eigen::Matrix4d> ring(int count, bool with_scale) {
  std::vector<Eigen::Matrix4d> out;
  for (int k = 0; k < count; ++k) {
    const double angle = 2.0 * std::numbers::pi * k / count;
    const Eigen::Matrix4d rigid =
        pose({3.0 * std::cos(angle), 3.0 * std::sin(angle), 0.0}, {0.0, 0.0, angle});
    out.push_back(similarity(rigid.topLeftCorner<3, 3>(), rigid.topRightCorner<3, 1>(),
                             with_scale ? 1.0 + (0.1 * k) : 1.0));
  }
  return out;
}

/// @brief A pose graph closes its ring from drifted poses, the fixed pose held exactly.
/// @param failures The running failure count.
void test_pose_graph(int& failures) {
  for (const bool with_scale : {false, true}) {
    const std::vector<Eigen::Matrix4d> truth = ring(8, with_scale);
    graphs::IndexPairs edges(8, 2);
    std::vector<Eigen::Matrix4d> relatives;
    for (int k = 0; k < 8; ++k) {
      edges.row(k) << k, (k + 1) % 8;
      relatives.emplace_back(truth[k].inverse() * truth[(k + 1) % 8]);
    }
    std::vector<Eigen::Matrix4d> start = truth;
    for (int k = 1; k < 8; ++k) {
      start[k] = start[k] * pose({0.05 * k, -0.03 * k, 0.02}, {0.01 * k, 0.0, -0.02 * k});
      if (with_scale) {
        start[k].topLeftCorner<3, 3>() *= 1.0 + (0.02 * k);
      }
    }
    const std::array<std::int64_t, 1> fixed = {0};
    const auto result = graphs::optimize_pose_graph(start, edges, relatives, fixed, with_scale,
                                                    graphs::Options{.max_iterations = 30});
    expect(result.has_value(), "the pose graph solves", failures);
    if (!result) {
      continue;
    }
    bool recovered = result->poses[0] == truth[0];
    for (int k = 0; k < 8; ++k) {
      recovered = recovered && close(result->poses[k], truth[k], 0.0, 1e-8);
    }
    expect(recovered, "every pose returns to the truth, the fixed one exactly", failures);
    expect(result->summary.final_error < 1e-15 && result->summary.initial_error > 1e-15,
           "the error falls to rounding", failures);
  }
}

/// @brief Held poses are constants: an edge with a held end pulls its free end alone, from either
///        side; one between two held poses costs a constant, counted in the errors; and with
///        every pose held nothing is solved.
/// @param failures The running failure count.
void test_held_poses(int& failures) {
  for (const bool with_scale : {false, true}) {
    const std::vector<Eigen::Matrix4d> truth = ring(4, with_scale);
    graphs::IndexPairs edges(4, 2);
    edges << 0, 1, 1, 2, 3, 1, 2, 3;  // both held; held first; held second; free
    std::vector<Eigen::Matrix4d> relatives;
    relatives.reserve(edges.rows());
    for (Eigen::Index e = 0; e < edges.rows(); ++e) {
      relatives.emplace_back(truth[edges(e, 0)].inverse() * truth[edges(e, 1)]);
    }
    relatives[0] = relatives[0] * pose({0.1, 0.0, 0.0}, {0.0, 0.0, 0.05});  // costs a constant
    std::vector<Eigen::Matrix4d> start = truth;
    start[2] = start[2] * pose({0.05, -0.03, 0.02}, {0.01, 0.0, -0.02});
    start[3] = start[3] * pose({-0.04, 0.02, 0.01}, {0.0, 0.02, 0.01});
    const std::array<std::int64_t, 2> fixed = {0, 1};
    const auto result = graphs::optimize_pose_graph(start, edges, relatives, fixed, with_scale,
                                                    graphs::Options{.max_iterations = 30});
    const std::array<std::int64_t, 4> every = {0, 1, 2, 3};
    const auto held = graphs::optimize_pose_graph(truth, edges, relatives, every, with_scale, {});
    expect(result.has_value() && held.has_value(), "both graphs solve", failures);
    if (!result || !held) {
      continue;
    }
    expect(result->poses[0] == start[0] && result->poses[1] == start[1] &&
               close(result->poses[2], truth[2], 0.0, 1e-8) &&
               close(result->poses[3], truth[3], 0.0, 1e-8),
           "the held poses come back exactly, the free ones to the truth", failures);
    expect(held->poses == truth && held->summary.iterations == 0 &&
               held->summary.termination == graphs::Termination::kFunction &&
               held->summary.converged &&
               held->summary.initial_error == held->summary.final_error &&
               held->summary.initial_error > 1e-4,
           "every pose held: nothing solved, the held edge's cost reported", failures);
    const double constant = held->summary.initial_error;
    expect(std::abs(result->summary.final_error - constant) <= 1e-9 * constant &&
               result->summary.initial_error > result->summary.final_error,
           "the edge between the held poses costs its constant in the errors", failures);
  }
}

/// @brief ORB-SLAM's two rounds: a wrong match stands out; without it the motion is exact.
/// @param failures The running failure count.
void test_refine_similarity(int& failures) {
  constexpr int kMatches = 40;
  for (const bool fixed_scale : {false, true}) {
    const Eigen::Matrix4d truth =
        similarity(so3::exp({0.05, 0.1, -0.03}), {0.2, -0.1, 0.1}, fixed_scale ? 1.0 : 0.8);
    graphs::Points points1(kMatches, 3);
    graphs::Points points2(kMatches, 3);
    graphs::Pairs pixels1(kMatches, 2);
    graphs::Pairs pixels2(kMatches, 2);
    for (int k = 0; k < kMatches; ++k) {
      const Eigen::Vector3d p2(std::sin(1.3 * k), std::cos(0.7 * k),
                               3.5 + (1.5 * std::sin(0.4 * k)));
      const Eigen::Vector3d p1 = truth.topLeftCorner<3, 3>() * p2 + truth.topRightCorner<3, 1>();
      points1.row(k) = p1.transpose();
      points2.row(k) = p2.transpose();
      pixels1.row(k) = camera::project(camera::Model::kPinhole, pinhole(), p1)->transpose();
      pixels2.row(k) = camera::project(camera::Model::kPinhole, pinhole(), p2)->transpose();
    }
    pixels1.row(0) += Eigen::RowVector2d(40.0, -35.0);  // one wrong match
    const graphs::Pairs ones = graphs::Pairs::Ones(kMatches, 2);
    const graphs::Options options{.max_iterations = 20};
    const auto first = graphs::refine_similarity(
        truth * pose({0.01, -0.01, 0.02}, {0.02, -0.02, 0.01}), points1, points2, pixels1, pixels2,
        ones, camera::Model::kPinhole, pinhole(), fixed_scale, std::sqrt(10.0), options);
    expect(first.has_value(), "the first round solves", failures);
    if (!first) {
      continue;
    }
    expect(close(first->similarity, truth, 0.0, 1e-2), "the first round lands near", failures);
    expect(first->chi2(0, 0) > 1000.0, "the wrong match's chi2 stands out", failures);
    const auto second = graphs::refine_similarity(
        first->similarity, points1.bottomRows(kMatches - 1), points2.bottomRows(kMatches - 1),
        pixels1.bottomRows(kMatches - 1), pixels2.bottomRows(kMatches - 1),
        ones.bottomRows(kMatches - 1), camera::Model::kPinhole, pinhole(), fixed_scale,
        std::nullopt, options);
    expect(second.has_value() && close(second->similarity, truth, 0.0, 1e-12),
           "without it the second round, with no robust loss, returns the true motion", failures);
  }
}

/// @brief A stereo scene: five cameras along x looking down +z, 60 points in front of them,
///        every camera's view of every point (every second one with its right column), and a
///        start with the cameras after the first and every point perturbed.
struct Scene {
  /// @brief Each camera's true `T_world_cam`.
  std::vector<Eigen::Matrix4d> truth;
  /// @brief Each point's true position.
  graphs::Points points;
  /// @brief Each observation's camera.
  std::vector<std::int64_t> cameras;
  /// @brief Each observation's point.
  std::vector<std::int64_t> ids;
  /// @brief Each observation's pixel.
  graphs::Pairs pixels;
  /// @brief Each observation's right column, NaN for none.
  std::vector<double> right;
  /// @brief Each observation's information.
  std::vector<double> information;
  /// @brief Each camera's starting pose.
  std::vector<Eigen::Matrix4d> start;
  /// @brief Each point's start.
  graphs::Points start_points;
};

/// @brief The stereo scene, seen through the radtan camera.
/// @return The scene.
Scene stereo_scene() {
  constexpr int kCameras = 5;
  constexpr int kPoints = 60;
  Scene scene;
  for (int c = 0; c < kCameras; ++c) {
    scene.truth.push_back(pose({0.4 * c, 0.0, 0.0}, {0.0, 0.05 * c, 0.0}));
  }
  scene.points.resize(kPoints, 3);
  for (int p = 0; p < kPoints; ++p) {
    scene.points.row(p) << -1.0 + (0.065 * p), std::sin(p) * 0.9, 3.0 + (0.05 * p);
  }
  scene.pixels.resize(static_cast<Eigen::Index>(kCameras) * kPoints, 2);
  for (int c = 0; c < kCameras; ++c) {
    for (int p = 0; p < kPoints; ++p) {
      const Eigen::Matrix4d& at = scene.truth[c];
      const Eigen::Vector3d local = at.topLeftCorner<3, 3>().transpose() *
                                    (scene.points.row(p).transpose() - at.topRightCorner<3, 1>());
      const Eigen::Vector2d seen = *camera::project(camera::Model::kRadTan, radtan(), local);
      scene.pixels.row(static_cast<Eigen::Index>(scene.cameras.size())) = seen.transpose();
      scene.right.push_back(p % 2 == 1 ? seen.x() - (kBf / local.z())
                                       : std::numeric_limits<double>::quiet_NaN());
      scene.cameras.push_back(c);
      scene.ids.push_back(p);
    }
  }
  scene.information.assign(scene.cameras.size(), 1.0);
  scene.start = scene.truth;
  for (int c = 1; c < kCameras; ++c) {
    scene.start[c] = scene.start[c] * pose({0.01 * c, -0.02, 0.015}, {0.01, -0.005 * c, 0.02});
  }
  scene.start_points = scene.points;
  for (int p = 0; p < kPoints; ++p) {
    scene.start_points.row(p) += Eigen::RowVector3d(0.03, -0.02, 0.04) * std::cos(p);
  }
  return scene;
}

/// @brief Adjust the scene from its start, the cameras in `fixed` held, for 20 iterations.
/// @param scene The scene.
/// @param fixed The cameras held.
/// @return The adjustment.
slam_it::foundations::Result<graphs::Adjustment> adjust(const Scene& scene,
                                                        std::span<const std::int64_t> fixed) {
  return graphs::bundle_adjust(scene.start, scene.start_points,
                               graphs::Observations{.cameras = scene.cameras,
                                                    .point_ids = scene.ids,
                                                    .pixels = scene.pixels,
                                                    .right = scene.right,
                                                    .information = scene.information,
                                                    .huber = {}},
                               camera::Model::kRadTan, radtan(), kBf, fixed,
                               graphs::Options{.max_iterations = 20});
}

/// @brief Whether an adjustment returned the held cameras exactly as they started, and every
///        other camera and every point to the truth.
/// @param result The adjustment.
/// @param scene Its scene.
/// @param fixed The cameras held.
/// @return Whether it did.
bool recovered(const graphs::Adjustment& result, const Scene& scene,
               std::span<const std::int64_t> fixed) {
  bool out = close(result.points, scene.points, 0.0, 1e-6);
  for (std::size_t c = 0; c < scene.truth.size(); ++c) {
    out = out && (std::ranges::contains(fixed, static_cast<std::int64_t>(c))
                      ? result.poses[c] == scene.start[c]
                      : close(result.poses[c], scene.truth[c], 0.0, 1e-7));
  }
  return out;
}

/// @brief Stereo observations fix the scale: perturbed cameras and points return to the truth.
/// @param failures The running failure count.
void test_bundle_adjust(int& failures) {
  const Scene scene = stereo_scene();
  const std::array<std::int64_t, 1> fixed = {0};
  const auto result = adjust(scene, fixed);
  expect(result.has_value() && recovered(*result, scene, fixed),
         "cameras and points return to the truth, the fixed camera exactly", failures);
}

/// @brief Held cameras are constants: each comes back bit for bit as it was given, with three of
///        five held, and with every one held, the points alone adjusted.
/// @param failures The running failure count.
void test_held_cameras(int& failures) {
  Scene scene = stereo_scene();
  scene.start = scene.truth;
  scene.start[4] = scene.start[4] * pose({0.02, -0.02, 0.01}, {0.01, 0.0, -0.01});
  const std::array<std::int64_t, 3> three = {0, 1, 2};
  const auto some = adjust(scene, three);
  expect(some.has_value() && recovered(*some, scene, three),
         "three held come back exactly, the free ones and the points to the truth", failures);
  scene.start[4] = scene.truth[4];
  const std::array<std::int64_t, 5> every = {0, 1, 2, 3, 4};
  const auto all = adjust(scene, every);
  expect(all.has_value() && recovered(*all, scene, every) && all->summary.iterations > 0,
         "every camera held: the points alone return to the truth", failures);
}

/// @brief Capped; converged by the relative tolerance; and, at a minimum, no step lowers it.
/// @param failures The running failure count.
void test_termination(int& failures) {
  const std::vector<Eigen::Matrix4d> truth = ring(6, false);
  graphs::IndexPairs edges(6, 2);
  std::vector<Eigen::Matrix4d> relatives;
  for (int k = 0; k < 6; ++k) {
    edges.row(k) << k, (k + 1) % 6;
    relatives.emplace_back(truth[k].inverse() * truth[(k + 1) % 6]);
  }
  relatives[0] = relatives[0] * pose({0.1, 0.0, 0.0}, {0.0, 0.0, 0.05});  // inconsistent
  std::vector<Eigen::Matrix4d> start;
  start.reserve(truth.size());
  for (const Eigen::Matrix4d& p : truth) {
    start.emplace_back(p * pose({0.1, 0.1, 0.0}, {0.0, 0.0, 0.1}));
  }
  start[0] = truth[0];
  const std::array<std::int64_t, 1> fixed = {0};
  const auto capped = graphs::optimize_pose_graph(start, edges, relatives, fixed, false,
                                                  graphs::Options{.max_iterations = 1});
  expect(capped && capped->summary.iterations == 1 &&
             capped->summary.termination == graphs::Termination::kMaxIterations &&
             !capped->summary.converged,
         "one iteration, capped", failures);
  const auto tolerant = graphs::optimize_pose_graph(
      start, edges, relatives, fixed, false,
      graphs::Options{.max_iterations = 50, .relative_tolerance = 1e-6});
  expect(tolerant && tolerant->summary.termination == graphs::Termination::kFunction &&
             tolerant->summary.converged,
         "converged by the relative tolerance", failures);
  if (!tolerant) {
    return;
  }
  const auto at_minimum =
      graphs::optimize_pose_graph(tolerant->poses, edges, relatives, fixed, false,
                                  graphs::Options{.max_iterations = 50, .relative_tolerance = 0.0});
  expect(at_minimum && at_minimum->summary.termination == graphs::Termination::kDampingLimit,
         "at the minimum with no tolerance, no step lowers the error", failures);
  const std::atomic<bool> stop{true};
  const auto stopped = graphs::optimize_pose_graph(
      start, edges, relatives, fixed, false, graphs::Options{.max_iterations = 50, .stop = &stop});
  expect(stopped && stopped->summary.iterations == 0 &&
             stopped->summary.termination == graphs::Termination::kStopped &&
             !stopped->summary.converged,
         "a stop set ends the solve before its first iteration", failures);
}

/// @brief Every input is checked before GTSAM sees it.
/// @param failures The running failure count.
void test_bad_input(int& failures) {
  const std::vector<Eigen::Matrix4d> truth = ring(4, false);
  graphs::IndexPairs edges(1, 2);
  edges << 0, 1;
  const std::vector<Eigen::Matrix4d> relatives = {truth[0].inverse() * truth[1]};
  const std::array<std::int64_t, 1> fixed = {0};
  const std::vector<std::int64_t> none;
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, edges, relatives, none, false, {})),
         "no fixed pose is refused", failures);
  graphs::IndexPairs self(1, 2);
  self << 1, 1;
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, self, relatives, fixed, false, {})),
         "a self-edge is refused", failures);
  graphs::IndexPairs outside(1, 2);
  outside << 0, 4;
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, outside, relatives, fixed, false, {})),
         "an index out of range is refused", failures);
  std::vector<Eigen::Matrix4d> scaled = truth;
  scaled[1].topLeftCorner<3, 3>() *= 2.0;
  expect(is_invalid_input(graphs::optimize_pose_graph(scaled, edges, relatives, fixed, false, {})),
         "a scaled pose in a rigid graph is refused", failures);
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, edges, relatives, fixed, false,
                                                      graphs::Options{.max_iterations = 0})),
         "max_iterations below 1 is refused", failures);
  graphs::Points points(2, 3);
  points << 0.0, 0.0, 3.0, 1.0, 0.0, 3.0;
  const std::array<std::int64_t, 1> one_camera = {0};
  const std::array<std::int64_t, 1> one_point = {0};
  const graphs::Pairs pixel = graphs::Pairs::Constant(1, 2, 1.0);
  const std::array<double, 1> mono = {std::numeric_limits<double>::quiet_NaN()};
  const std::array<double, 1> info = {1.0};
  expect(
      is_invalid_input(graphs::bundle_adjust(truth, points,
                                             graphs::Observations{.cameras = one_camera,
                                                                  .point_ids = one_point,
                                                                  .pixels = pixel,
                                                                  .right = mono,
                                                                  .information = info,
                                                                  .huber = {}},
                                             camera::Model::kPinhole, pinhole(), kBf, fixed, {})),
      "an unobserved point is refused", failures);
  const std::array<double, 1> stereo = {3.0};
  expect(
      is_invalid_input(graphs::bundle_adjust(truth, points.topRows(1),
                                             graphs::Observations{.cameras = one_camera,
                                                                  .point_ids = one_point,
                                                                  .pixels = pixel,
                                                                  .right = stereo,
                                                                  .information = info,
                                                                  .huber = {}},
                                             camera::Model::kPinhole, pinhole(), 0.0, fixed, {})),
      "a right column without bf is refused", failures);
  // A row of pixels2 is finite, or wholly NaN (unseen): anything else is refused (H1, L1).
  graphs::Points same(3, 3);
  same << 0.0, 0.0, 3.0, 0.1, 0.0, 3.0, 0.2, 0.1, 3.0;
  graphs::Pairs seen(3, 2);
  for (int k = 0; k < 3; ++k) {
    const Eigen::Vector3d p = same.row(k).transpose();
    seen.row(k) = camera::project(camera::Model::kPinhole, pinhole(), p)->transpose();
  }
  const auto refine = [&](const graphs::Pairs& pixels2) {
    return graphs::refine_similarity(Eigen::Matrix4d::Identity(), same, same, seen, pixels2,
                                     graphs::Pairs::Ones(3, 2), camera::Model::kPinhole, pinhole(),
                                     true, std::nullopt, {});
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::array<Eigen::RowVector2d, 3> bad = {Eigen::RowVector2d(nan, 210.0),
                                                 Eigen::RowVector2d(inf, 210.0),
                                                 Eigen::RowVector2d(300.0, -inf)};
  for (const Eigen::RowVector2d& row : bad) {
    graphs::Pairs pixels2 = seen;
    pixels2.row(1) = row;
    expect(is_invalid_input(refine(pixels2)), "a half-unseen or infinite pixels2 row is refused",
           failures);
  }
  graphs::Pairs unseen = seen;
  unseen.row(1).setConstant(nan);
  expect(refine(unseen).has_value(), "a wholly unseen pixels2 row is taken", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  test_projection_jacobians(failures);
  test_similarity_jacobians(failures);
  test_behind_the_camera(failures);
  test_pose_graph(failures);
  test_held_poses(failures);
  test_refine_similarity(failures);
  test_bundle_adjust(failures);
  test_held_cameras(failures);
  test_termination(failures);
  test_bad_input(failures);
  expect(std::string(graphs::name(graphs::Termination::kDampingLimit)) == "damping_limit",
         "the terminations are named as Python names them", failures);
  return slam_it::testing::finish("foundations/graphs", failures);
}

}  // namespace

int main() { return run(); }
