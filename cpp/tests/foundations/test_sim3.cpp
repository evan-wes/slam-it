/// @file
/// @brief Native tests for Sim(3) RANSAC (slam_it/foundations/sim3.hpp): the similarity and the
///        clean inliers recovered through outliers, rigid and scaled; the inverse; a point carried
///        behind the other camera never an inlier; and bad input.
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/ransac.hpp"
#include "slam_it/foundations/se3.hpp"
#include "slam_it/foundations/sim3.hpp"
#include "test_support.hpp"

namespace {

namespace sim3 = slam_it::foundations::sim3;
namespace camera = slam_it::foundations::camera;
namespace ransac = slam_it::foundations::ransac;
namespace se3 = slam_it::foundations::se3;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief The chi-square bound with two degrees of freedom at 99% (ORB-SLAM's 9.21).
constexpr double kChi2 = 9.21;
/// @brief The matches in each scene.
constexpr int kMatches = 80;
/// @brief Every this many matches is an outlier.
constexpr int kOutlierEvery = 4;

/// @brief A pinhole camera's intrinsics.
Eigen::VectorXd pinhole() { return Eigen::Vector4d(500.0, 500.0, 320.0, 240.0); }

/// @brief A scene: matches with every fourth an outlier, and the similarity of the rest.
struct Scene {
  /// @brief The true `S_12`.
  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  /// @brief Points in camera 1's frame.
  sim3::Rows3 points1;
  /// @brief Points in camera 2's frame.
  sim3::Rows3 points2;
  /// @brief Which matches are true.
  Eigen::Array<bool, Eigen::Dynamic, 1> clean;
};

/// @brief Build a scene: points 3-8 m before camera 2, carried by `S_12`, a quarter replaced.
/// @param scale The similarity's scale.
/// @return The scene.
Scene scene(double scale) {
  Scene out;
  Eigen::Matrix<double, 6, 1> tangent;
  tangent << 0.3, -0.1, 0.2, 0.1, -0.2, 0.05;
  out.truth = se3::exp(tangent);
  out.truth.topLeftCorner<3, 3>() *= scale;
  out.points1.resize(kMatches, 3);
  out.points2.resize(kMatches, 3);
  out.clean.resize(kMatches);
  for (int i = 0; i < kMatches; ++i) {
    // A spread that no three points share a line on.
    const double u = std::sin(1.7 * i);
    const double v = std::cos(2.3 * i);
    const Eigen::Vector3d p2(2.0 * u, 1.5 * v, 5.5 + (2.5 * std::sin(0.9 * i)));
    out.points2.row(i) = p2.transpose();
    const Eigen::Vector3d p1 =
        (out.truth.topLeftCorner<3, 3>() * p2) + out.truth.topRightCorner<3, 1>();
    out.clean(i) = i % kOutlierEvery != 0;
    out.points1.row(i) =
        out.clean(i) ? p1.transpose() : Eigen::RowVector3d(-1.5 * v, 1.2 * u, 3.0 + std::abs(u));
  }
  return out;
}

/// @brief RANSAC's settings: the bound's square root, ORB-SLAM's confidence and cap.
/// @return The settings.
ransac::Options settings() {
  return ransac::Options{.threshold = std::sqrt(kChi2), .confidence = 0.99, .max_iterations = 300};
}

/// @brief Through a quarter of outliers the similarity is exact, its inliers the clean matches.
/// @param failures The running failure count.
void test_recovers_the_similarity(int& failures) {
  for (const bool fixed_scale : {false, true}) {
    const Scene s = scene(fixed_scale ? 1.0 : 0.7);
    const Eigen::VectorXd sigma2 = Eigen::VectorXd::Ones(kMatches);
    const auto found =
        sim3::sim3_ransac(s.points1, s.points2, sigma2, sigma2, camera::Model::kPinhole, pinhole(),
                          fixed_scale, settings());
    expect(found.has_value(), "the RANSAC runs", failures);
    if (!found) {
      continue;
    }
    expect((found->state.model - s.truth).cwiseAbs().maxCoeff() < 1e-10,
           "the similarity is exact on exact inliers", failures);
    expect((found->state.inliers == s.clean).all(), "the inliers are the clean matches", failures);
    expect(found->summary.converged, "the RANSAC reaches its confidence", failures);
  }
}

/// @brief The inverse undoes the similarity, scale and all.
/// @param failures The running failure count.
void test_inverse(int& failures) {
  const Scene s = scene(1.3);
  const Eigen::Matrix4d product = sim3::inverse(s.truth) * s.truth;
  expect((product - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff() < 1e-14,
         "S^-1 S is the identity", failures);
}

/// @brief A match whose point is carried behind the other camera has an infinite residual.
/// @param failures The running failure count.
void test_behind_the_camera(int& failures) {
  sim3::Rows3 points1(3, 3);
  sim3::Rows3 points2(3, 3);
  points2 << 0.0, 0.0, 4.0, 1.0, 0.0, 5.0, 0.0, 1.0, 6.0;
  points1 = points2;
  points1(0, 2) = 0.5;  // its partner carried by the identity lands in front: still fine
  const Eigen::VectorXd ones = Eigen::VectorXd::Ones(3);
  const sim3::SimilarityEstimator estimator(
      sim3::SimilarityEstimator::Matches{.points1 = points1,
                                         .points2 = points2,
                                         .image1 = sim3::Rows2::Zero(3, 2),
                                         .image2 = sim3::Rows2::Zero(3, 2),
                                         .sigma2_1 = ones,
                                         .sigma2_2 = ones},
      camera::Model::kPinhole, pinhole(), false);
  Eigen::Matrix4d flip = Eigen::Matrix4d::Identity();
  flip(2, 3) = -10.0;  // every point of camera 2 lands behind camera 1
  const Eigen::VectorXd residuals = estimator.residuals(flip);
  expect((residuals.array() == std::numeric_limits<double>::infinity()).all(),
         "a point behind the other camera is never an inlier", failures);
}

/// @brief Malformed matches and settings are refused.
/// @param failures The running failure count.
void test_bad_input(int& failures) {
  const Scene s = scene(1.0);
  const Eigen::VectorXd ones = Eigen::VectorXd::Ones(kMatches);
  expect(is_invalid_input(sim3::sim3_ransac(s.points1.topRows(2), s.points2.topRows(2),
                                            ones.head(2), ones.head(2), camera::Model::kPinhole,
                                            pinhole(), false, settings())),
         "fewer than three matches are refused", failures);
  sim3::Rows3 behind = s.points2;
  behind(1, 2) = -1.0;
  expect(is_invalid_input(sim3::sim3_ransac(s.points1, behind, ones, ones, camera::Model::kPinhole,
                                            pinhole(), false, settings())),
         "a point behind its own camera is refused", failures);
  Eigen::VectorXd zero = ones;
  zero(3) = 0.0;
  expect(is_invalid_input(sim3::sim3_ransac(s.points1, s.points2, zero, ones,
                                            camera::Model::kPinhole, pinhole(), false, settings())),
         "a variance that is not positive is refused", failures);
  expect(
      is_invalid_input(sim3::sim3_ransac(s.points1, s.points2, ones, ones, camera::Model::kPinhole,
                                         Eigen::Vector3d(1, 2, 3), false, settings())),
      "a camera's wrong intrinsics are refused", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  test_recovers_the_similarity(failures);
  test_inverse(failures);
  test_behind_the_camera(failures);
  test_bad_input(failures);
  return slam_it::testing::finish("foundations/sim3", failures);
}

}  // namespace

int main() { return run(); }
