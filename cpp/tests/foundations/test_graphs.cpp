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
#include <utility>
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

/// @brief An edge between two held poses costs its error inside GTSAM's own rules too: with a
///        free pose in no edge, the one iteration lowers nothing and the solve converges, as when
///        the held poses were variables (0.3.17.2's review panel, L1), not at the damping's limit.
/// @param failures The running failure count.
void test_held_edge_alone(int& failures) {
  const std::vector<Eigen::Matrix4d> truth = {pose({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}),
                                              pose({1.0, 0.2, 0.0}, {0.0, 0.0, 2.0}),
                                              pose({2.0, -0.2, 0.0}, {0.0, 0.0, 4.0})};
  graphs::IndexPairs edges(1, 2);
  edges << 0, 1;
  const std::vector<Eigen::Matrix4d> relatives = {truth[0].inverse() * truth[1] *
                                                  pose({0.1, 0.0, 0.0}, {0.0, 0.0, 0.05})};
  const std::array<std::int64_t, 2> fixed = {0, 1};
  const auto result = graphs::optimize_pose_graph(truth, edges, relatives, fixed, false,
                                                  graphs::Options{.max_iterations = 10});
  expect(result.has_value() && result->summary.iterations == 1 &&
             result->summary.termination == graphs::Termination::kFunction &&
             result->summary.converged && result->summary.initial_error > 1e-3 &&
             result->summary.final_error == result->summary.initial_error,
         "a free pose in no edge, the held edge's error seen: converged after one iteration",
         failures);
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

/// @brief Adjust the scene from its start, the cameras in `fixed` held, by default for 20
///        iterations.
/// @param scene The scene.
/// @param fixed The cameras held.
/// @param options How the solve runs.
/// @return The adjustment.
slam_it::foundations::Result<graphs::Adjustment> adjust(
    const Scene& scene, std::span<const std::int64_t> fixed,
    const graphs::Options& options = graphs::Options{.max_iterations = 20}) {
  return graphs::bundle_adjust(scene.start, scene.start_points,
                               graphs::Observations{.cameras = scene.cameras,
                                                    .point_ids = scene.ids,
                                                    .pixels = scene.pixels,
                                                    .right = scene.right,
                                                    .information = scene.information,
                                                    .huber = {}},
                               camera::Model::kRadTan, radtan(), kBf, fixed, 0.0, options);
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
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, edges, relatives, fixed, false,
                                                      graphs::Options{.small_steps = -1})),
         "small_steps below 0 is refused", failures);
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, edges, relatives, fixed, false,
                                                      graphs::Options{.damping_scale = 0.0})),
         "a damping_scale of 0 is refused", failures);
  expect(is_invalid_input(graphs::optimize_pose_graph(truth, edges, relatives, fixed, false,
                                                      graphs::Options{.small_decrease = 0.0})),
         "a small_decrease of 0 is refused", failures);
  graphs::Points points(2, 3);
  points << 0.0, 0.0, 3.0, 1.0, 0.0, 3.0;
  const std::array<std::int64_t, 1> one_camera = {0};
  const std::array<std::int64_t, 1> one_point = {0};
  const graphs::Pairs pixel = graphs::Pairs::Constant(1, 2, 1.0);
  const std::array<double, 1> mono = {std::numeric_limits<double>::quiet_NaN()};
  const std::array<double, 1> info = {1.0};
  expect(is_invalid_input(graphs::bundle_adjust(truth, points,
                                                graphs::Observations{.cameras = one_camera,
                                                                     .point_ids = one_point,
                                                                     .pixels = pixel,
                                                                     .right = mono,
                                                                     .information = info,
                                                                     .huber = {}},
                                                camera::Model::kPinhole, pinhole(), kBf, fixed, 0.0,
                                                {})),
         "an unobserved point is refused", failures);
  const std::array<double, 1> stereo = {3.0};
  expect(is_invalid_input(graphs::bundle_adjust(truth, points.topRows(1),
                                                graphs::Observations{.cameras = one_camera,
                                                                     .point_ids = one_point,
                                                                     .pixels = pixel,
                                                                     .right = stereo,
                                                                     .information = info,
                                                                     .huber = {}},
                                                camera::Model::kPinhole, pinhole(), 0.0, fixed, 0.0,
                                                {})),
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

/// @brief Two held cameras 0.3 m apart along x, looking down +z: a point alone is solved.
/// @return Their `T_world_cam`.
std::vector<Eigen::Matrix4d> held_pair() {
  return {Eigen::Matrix4d::Identity(), pose({0.3, 0.0, 0.0}, {0.0, 0.0, 0.0})};
}

/// @brief The point the held pair sees, 4 m ahead.
/// @return The point.
Eigen::Vector3d far_truth() { return {0.2, -0.1, 4.0}; }

/// @brief Its start, three times as far: GTSAM's first trial steps fail there.
/// @return The point.
Eigen::Vector3d far_start() { return {0.2, -0.1, 12.0}; }

/// @brief Where the held pair's cameras see `far_truth`.
/// @return Their pixels (2 x 2).
graphs::Pairs held_pair_pixels() {
  graphs::Pairs out(2, 2);
  const std::vector<Eigen::Matrix4d> cameras = held_pair();
  for (std::size_t c = 0; c < 2; ++c) {
    const Eigen::Vector3d local = cameras[c].topLeftCorner<3, 3>().transpose() *
                                  (far_truth() - cameras[c].topRightCorner<3, 1>());
    out.row(static_cast<Eigen::Index>(c)) =
        camera::project(camera::Model::kPinhole, pinhole(), local)->transpose();
  }
  return out;
}

/// @brief A point's damped search, emulated: where it is, its damping and the damping's factor.
struct Damped {
  /// @brief The point.
  Eigen::Vector3d point;
  /// @brief The damping.
  double damping;
  /// @brief The damping's factor.
  double factor;
};

/// @brief The held pair's residuals at a point (unit information), stacked, and their Jacobian.
/// @param point The point.
/// @return The residuals (4) and the Jacobian (4 x 3).
std::pair<Eigen::VectorXd, Eigen::MatrixXd> pair_residuals(const Eigen::Vector3d& point) {
  Eigen::VectorXd residual(4);
  Eigen::MatrixXd jacobian(4, 3);
  const std::vector<Eigen::Matrix4d> cameras = held_pair();
  const graphs::Pairs pixels = held_pair_pixels();
  for (std::size_t c = 0; c < 2; ++c) {
    const auto row = static_cast<Eigen::Index>(c);
    const Eigen::VectorXd seen = pixels.row(row).transpose();
    const auto at = graphs::projection_residual(cameras[c], point, seen, camera::Model::kPinhole,
                                                pinhole(), kBf);
    residual.segment<2>(2 * row) = at->residual;
    jacobian.middleRows<2>(2 * row) = at->d_point;
  }
  return {residual, jacobian};
}

/// @brief One of GTSAM's Levenberg-Marquardt iterations on the held pair's point, emulated
///        (`tests/foundations/test_graphs.py`'s `_gtsam_iteration`): trials of
///        `(H + lambda I) delta = -g` until the gain ratio passes GTSAM's `minModelFidelity`,
///        1e-3; a failed one multiplies the damping by the factor, which Nielsen's schedule
///        doubles; an accepted one divides it by the factor, or with Nielsen's schedule scales it
///        by `max(1/3, 1 - (2 rho - 1)^3)` and resets the factor to 2.
/// @param at The point, damping and factor before the iteration.
/// @param nielsen Whether Nielsen's schedule moves the damping.
/// @param limit The damping's upper bound.
/// @return The point, damping and factor after it.
Damped gtsam_iteration(Damped at, bool nielsen, double limit) {
  constexpr double kMinFidelity = 1e-3;
  const auto [residual, jacobian] = pair_residuals(at.point);
  const double error = 0.5 * residual.squaredNorm();
  const Eigen::Matrix3d hessian = jacobian.transpose() * jacobian;
  const Eigen::Vector3d gradient = jacobian.transpose() * residual;
  while (at.damping < limit) {
    const Eigen::Vector3d step =
        -(hessian + (at.damping * Eigen::Matrix3d::Identity())).ldlt().solve(gradient);
    const double predicted = error - (0.5 * (residual + (jacobian * step)).squaredNorm());
    const double ratio =
        (error - (0.5 * pair_residuals(at.point + step).first.squaredNorm())) / predicted;
    if (ratio > kMinFidelity) {
      if (nielsen) {
        const double scale = std::max(1.0 / 3.0, 1.0 - std::pow((2.0 * ratio) - 1.0, 3));
        return {.point = at.point + step, .damping = at.damping * scale, .factor = 2.0};
      }
      return {.point = at.point + step, .damping = at.damping / at.factor, .factor = at.factor};
    }
    at.damping *= at.factor;
    at.factor *= nielsen ? 2.0 : 1.0;
  }
  return at;
}

/// @brief The scaled damping starts at tau times the largest diagonal entry and follows
///        Nielsen's schedule (its first iteration fails at 1, 2 and 8 times its start before 64
///        times steps); GTSAM's own starts at tau and moves by 10. Each of the first three
///        iterations lands where the emulated search does. The other damping's field is set far
///        off (1.0), so each solve reads its own alone.
/// @param failures The running failure count.
void test_damping_schedules(int& failures) {
  constexpr double kTau = 1e-5;
  const std::array<std::int64_t, 2> cameras = {0, 1};
  const std::array<std::int64_t, 2> ids = {0, 0};
  const std::array<double, 2> right = {std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::quiet_NaN()};
  const std::array<double, 2> information = {1.0, 1.0};
  const graphs::Pairs pixels = held_pair_pixels();
  graphs::Points start(1, 3);
  start.row(0) = far_start().transpose();
  const Eigen::MatrixXd jacobian = pair_residuals(far_start()).second;
  const double largest = (jacobian.transpose() * jacobian).diagonal().maxCoeff();
  for (const bool scaled : {true, false}) {
    Damped expected{
        .point = far_start(),
        .damping = scaled ? kTau * largest : kTau,
        .factor = scaled ? graphs::kScaledDampingFactor : graphs::kAbsoluteDampingFactor};
    for (int iterations = 1; iterations <= 3; ++iterations) {
      expected = gtsam_iteration(
          expected, scaled, scaled ? graphs::kScaledDampingLimit : graphs::kAbsoluteDampingLimit);
      const auto result =
          graphs::bundle_adjust(held_pair(), start,
                                graphs::Observations{.cameras = cameras,
                                                     .point_ids = ids,
                                                     .pixels = pixels,
                                                     .right = right,
                                                     .information = information,
                                                     .huber = {}},
                                camera::Model::kPinhole, pinhole(), kBf, cameras, 0.0,
                                graphs::Options{.max_iterations = iterations,
                                                .initial_damping = scaled ? 1.0 : kTau,
                                                .scaled_damping = scaled,
                                                .damping_scale = scaled ? kTau : 1.0});
      expect(result.has_value() && result->summary.iterations == iterations &&
                 close(result->points.row(0).transpose(), expected.point, 0.0, 1e-9),
             scaled ? "the scaled damping follows Nielsen's schedule from tau max diag(H)"
                    : "GTSAM's own damping starts at tau and moves by 10",
             failures);
    }
  }
}

/// @brief ORB-SLAM3's stop: from a heavily damped start (tau 3000) on the stereo scene, its
///        pixels perturbed, the decreases grow from small through large, then shrink. With the
///        small decrease set between the second and the third, the two first count as small and
///        the third resets them: the solve ends at the third small one in a row after it,
///        converged (`kSmallSteps`), later than it would without the reset; with every decrease
///        small, after three iterations.
/// @param failures The running failure count.
void test_small_steps(int& failures) {
  constexpr int kMost = 30;
  Scene scene = stereo_scene();
  for (Eigen::Index m = 0; m < scene.pixels.rows(); ++m) {
    const auto k = static_cast<double>(m);
    scene.pixels.row(m) += Eigen::RowVector2d(0.5 * std::sin(1.7 * k), 0.5 * std::cos(2.3 * k));
  }
  const std::array<std::int64_t, 1> fixed = {0};
  const auto solve = [&](graphs::Options options) {
    options.damping_scale = 3000.0;
    options.scaled_damping = true;
    return adjust(scene, fixed, options);
  };
  std::vector<double> errors{solve({.max_iterations = 1})->summary.initial_error};
  for (int k = 1; k <= kMost; ++k) {
    errors.push_back(solve({.max_iterations = k})->summary.final_error);
  }
  std::vector<double> decrease;
  for (std::size_t k = 1; k < errors.size(); ++k) {
    decrease.push_back((errors[k - 1] - errors[k]) / errors[k - 1]);
  }
  const double threshold = std::sqrt(decrease[1] * decrease[2]);
  std::vector<bool> small;
  small.reserve(decrease.size());
  for (const double d : decrease) {
    small.push_back(d < threshold);
  }
  expect(small[0] && small[1] && !small[2], "two small decreases, then one that resets them",
         failures);
  std::size_t ends = 0;
  for (std::size_t k = 3; k < small.size() && ends == 0; ++k) {
    ends = small[k] && small[k - 1] && small[k - 2] ? k + 1 : 0;
  }
  std::size_t without_reset = 0;  // where a count that ignored the reset would end
  for (std::size_t k = 3; k < small.size() && without_reset == 0; ++k) {
    without_reset = small[k] ? k + 1 : 0;
  }
  const auto stopped =
      solve({.max_iterations = kMost, .small_steps = 3, .small_decrease = threshold});
  expect(ends > without_reset && stopped.has_value() &&
             std::cmp_equal(stopped->summary.iterations, ends) &&
             stopped->summary.termination == graphs::Termination::kSmallSteps &&
             stopped->summary.converged,
         "the third small decrease in a row after the large ones ends the solve, converged",
         failures);
  const auto three = solve({.max_iterations = kMost, .small_steps = 3, .small_decrease = 0.9});
  expect(three.has_value() && three->summary.iterations == 3 &&
             three->summary.termination == graphs::Termination::kSmallSteps,
         "with every decrease small, three iterations end the solve", failures);
}

/// @brief Each observation's depth: its point along its camera's optical axis.
/// @param poses Each camera's `T_world_cam`.
/// @param points Each point.
/// @param observations The observations.
/// @return A depth per observation.
std::vector<double> depths_of(const std::vector<Eigen::Matrix4d>& poses,
                              const graphs::Points& points,
                              const graphs::Observations& observations) {
  std::vector<double> out;
  for (std::size_t m = 0; m < observations.cameras.size(); ++m) {
    const Eigen::Matrix4d& at = poses[static_cast<std::size_t>(observations.cameras[m])];
    out.push_back(at.block<3, 1>(0, 2).dot(points.row(observations.point_ids[m]).transpose() -
                                           at.topRightCorner<3, 1>()));
  }
  return out;
}

/// @brief The near-plane filter leaves out exactly the observations near or behind their
///        cameras, and counts them: the stereo scene, camera 0 held, a sixth camera 2 m ahead,
///        point 60 1 mm in front of it (seen too by cameras 0 and 4), point 61 behind cameras 0,
///        1 and 5 (seen by them). The starting error is the kept observations'; point 61 and
///        camera 5, every observation of which is left out, and the held camera come back
///        exactly as given; the kept observations ending behind or below the limit are counted
///        as recounted from the result. With near 0, every observation counts, and point 61's
///        three end behind their cameras.
/// @param failures The running failure count.
void test_near_filter(int& failures) {
  Scene scene = stereo_scene();
  std::vector<Eigen::Matrix4d> poses = scene.start;
  poses.push_back(pose({0.8, 0.0, 2.0}, {0.0, 0.0, 0.0}));
  const Eigen::Index count = scene.start_points.rows();
  graphs::Points points(count + 2, 3);
  points.topRows(count) = scene.start_points;
  points.row(count) = ((poses[5].topLeftCorner<3, 3>() * Eigen::Vector3d(0.0, 0.0, 1e-3)) +
                       poses[5].topRightCorner<3, 1>())
                          .transpose();
  points.row(count + 1) = ((poses[0].topLeftCorner<3, 3>() * Eigen::Vector3d(0.1, 0.0, -1.0)) +
                           poses[0].topRightCorner<3, 1>())
                              .transpose();
  const std::array<std::array<std::int64_t, 2>, 6> extra = {
      {{5, count}, {0, count}, {4, count}, {0, count + 1}, {1, count + 1}, {5, count + 1}}};
  constexpr std::array<bool, 6> kExtraKept = {false, true, true, false, false, false};
  std::vector<bool> expected(scene.cameras.size(), true);
  graphs::Pairs pixels(scene.pixels.rows() + 6, 2);
  pixels.topRows(scene.pixels.rows()) = scene.pixels;
  pixels.bottomRows(6).setConstant(300.0);
  for (std::size_t e = 0; e < extra.size(); ++e) {
    scene.cameras.push_back(extra.at(e)[0]);
    scene.ids.push_back(extra.at(e)[1]);
    scene.right.push_back(std::numeric_limits<double>::quiet_NaN());
    scene.information.push_back(1.0);
    expected.push_back(kExtraKept.at(e));
  }
  double every = 0.0;
  double kept = 0.0;
  for (std::size_t m = 0; m < scene.cameras.size(); ++m) {
    const auto row = static_cast<Eigen::Index>(m);
    Eigen::VectorXd measured(std::isnan(scene.right[m]) ? 2 : 3);
    measured.head<2>() = pixels.row(row).transpose();
    if (measured.size() == 3) {
      measured(2) = scene.right[m];
    }
    const double cost =
        0.5 * graphs::projection_residual(poses[static_cast<std::size_t>(scene.cameras[m])],
                                          points.row(scene.ids[m]).transpose(), measured,
                                          camera::Model::kRadTan, radtan(), kBf)
                  ->residual.squaredNorm();
    every += cost;
    kept += expected[m] ? cost : 0.0;
  }
  const graphs::Observations observations{.cameras = scene.cameras,
                                          .point_ids = scene.ids,
                                          .pixels = pixels,
                                          .right = scene.right,
                                          .information = scene.information,
                                          .huber = {}};
  const std::array<std::int64_t, 1> fixed = {0};
  const auto adjust_near = [&](double near) {
    return graphs::bundle_adjust(poses, points, observations, camera::Model::kRadTan, radtan(), kBf,
                                 fixed, near, graphs::Options{.max_iterations = 5});
  };
  std::vector<double> start = depths_of(poses, points, observations);
  std::ranges::nth_element(start, start.begin() + static_cast<std::ptrdiff_t>(start.size() / 2));
  const double limit = 1e-3 * start[start.size() / 2];
  const auto ended = [&](const graphs::Adjustment& result, double below,
                         const std::vector<bool>& which) {
    const std::vector<double> end = depths_of(result.poses, result.points, observations);
    std::int64_t out = 0;
    for (std::size_t m = 0; m < end.size(); ++m) {
      out += which[m] && (end[m] <= 0.0 || end[m] < below) ? 1 : 0;
    }
    return out;
  };
  const auto filtered = adjust_near(1e-3);
  expect(filtered.has_value() && std::abs(filtered->summary.initial_error - kept) <= 1e-12 * kept &&
             filtered->summary.left_out == std::ranges::count(expected, false) &&
             filtered->summary.ended_near == ended(*filtered, limit, expected) &&
             filtered->points.row(count + 1) == points.row(count + 1) &&
             filtered->poses[0] == poses[0] && filtered->poses[5] == poses[5] &&
             filtered->points.row(count) != points.row(count),
         "exactly the near and behind observations are left out and counted; what they alone "
         "reach stays",
         failures);
  const auto all = adjust_near(0.0);
  const std::vector<bool> everyone(scene.cameras.size(), true);
  expect(all.has_value() && std::abs(all->summary.initial_error - every) <= 1e-12 * every &&
             all->summary.left_out == 0 && all->summary.ended_near == ended(*all, 0.0, everyone) &&
             all->summary.ended_near >= 3,
         "with near 0 every observation counts, and those ending behind are counted", failures);
  for (const double bad :
       {-1e-3, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    expect(is_invalid_input(adjust_near(bad)), "a negative or infinite near fraction is refused",
           failures);
  }
}

/// @brief A bundle whose every observation is behind its camera comes back as given, after no
///        iteration, converged, with no error counted.
/// @param failures The running failure count.
void test_everything_left_out(int& failures) {
  const std::vector<Eigen::Matrix4d> two = {Eigen::Matrix4d::Identity(),
                                            pose({0.2, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  graphs::Points behind(2, 3);
  behind << 0.1, 0.2, -1.0, 0.0, -0.1, -2.0;
  const std::array<std::int64_t, 3> which = {0, 1, 1};
  const std::array<std::int64_t, 3> whose = {0, 0, 1};
  const std::array<std::int64_t, 1> fixed = {0};
  const graphs::Pairs seen = graphs::Pairs::Constant(3, 2, 300.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::array<double, 3> mono = {nan, nan, nan};
  const std::array<double, 3> unit = {1.0, 1.0, 1.0};
  const auto none = graphs::bundle_adjust(two, behind,
                                          graphs::Observations{.cameras = which,
                                                               .point_ids = whose,
                                                               .pixels = seen,
                                                               .right = mono,
                                                               .information = unit,
                                                               .huber = {}},
                                          camera::Model::kPinhole, pinhole(), kBf, fixed, 1e-3,
                                          graphs::Options{});
  expect(none.has_value() && none->poses == two && none->points == behind &&
             none->summary.iterations == 0 && none->summary.converged &&
             none->summary.initial_error == 0.0 && none->summary.final_error == 0.0 &&
             none->summary.left_out == 3 && none->summary.ended_near == 0,
         "a bundle whose every observation is left out comes back as given", failures);
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
  test_held_edge_alone(failures);
  test_refine_similarity(failures);
  test_bundle_adjust(failures);
  test_held_cameras(failures);
  test_termination(failures);
  test_damping_schedules(failures);
  test_small_steps(failures);
  test_near_filter(failures);
  test_everything_left_out(failures);
  test_bad_input(failures);
  expect(std::string(graphs::name(graphs::Termination::kDampingLimit)) == "damping_limit" &&
             std::string(graphs::name(graphs::Termination::kSmallSteps)) == "small_steps" &&
             graphs::is_converged(graphs::Termination::kSmallSteps),
         "the terminations are named as Python names them", failures);
  return slam_it::testing::finish("foundations/graphs", failures);
}

}  // namespace

int main() { return run(); }
