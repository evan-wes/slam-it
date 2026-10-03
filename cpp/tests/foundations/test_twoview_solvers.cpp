/// @file
/// @brief Native tests for the two-view solvers (slamlab/foundations/epipolar.hpp and
///        homography.hpp): exact recovery, degenerate configurations, and RANSAC with
///        outliers, built without Python.
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>

#include "slamlab/foundations/epipolar.hpp"
#include "slamlab/foundations/homography.hpp"
#include "slamlab/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

namespace epipolar = slamlab::foundations::epipolar;
namespace homography = slamlab::foundations::homography;
namespace ransac = slamlab::foundations::ransac;
using slamlab::testing::expect;

/// @brief The smaller of `|a - b|` and `|a + b|` (entrywise max): equality up to sign.
/// @param a A matrix.
/// @param b A matrix.
/// @return The distance.
double up_to_sign(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return std::min((a - b).cwiseAbs().maxCoeff(), (a + b).cwiseAbs().maxCoeff());
}

/// @brief Deterministic points 4-8 m ahead (or on the plane z = 5 when `planar`).
/// @param n How many.
/// @param planar Whether to put them on a plane.
/// @return The points (n x 3).
Eigen::MatrixX3d points(int n, bool planar) {
  Eigen::MatrixX3d out(n, 3);
  for (int i = 0; i < n; ++i) {
    const double a = std::sin(1.3 * i);
    const double b = std::cos(0.7 * i);
    out.row(i) << 2.0 * a, 2.0 * b, planar ? 5.0 : 6.0 + (2.0 * std::sin(2.9 * i));
  }
  return out;
}

}  // namespace

int main() {
  int failures = 0;
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.2, -1.0, 0.3).normalized()).toRotationMatrix();
  const Eigen::Vector3d t(0.5, -0.1, 0.05);
  const Eigen::Matrix3d truth = slamlab::foundations::so3::hat(t.normalized()) * rotation;

  const Eigen::MatrixX3d scene = points(40, false);
  epipolar::Rows3 b1(40, 3);
  epipolar::Rows3 b2(40, 3);
  for (int i = 0; i < 40; ++i) {
    const Eigen::Vector3d x = scene.row(i).transpose();
    b1.row(i) = x.normalized().transpose();
    b2.row(i) = ((rotation * x) + t).normalized().transpose();
  }
  const auto e = epipolar::essential_8pt(b1, b2);
  expect(e.has_value() && up_to_sign(*e, truth) < 1e-12, "E is [t]x R up to sign", failures);

  // Every third pair an outlier; RANSAC must find E and exactly the clean pairs.
  epipolar::Rows3 dirty = b2;
  for (int i = 0; i < 40; i += 3) {
    dirty.row(i) = Eigen::Vector3d(std::sin(5.0 * i), std::cos(3.0 * i), 2.0).normalized();
  }
  const auto run = epipolar::essential_ransac(b1, dirty, ransac::Options{.threshold = 1e-6});
  bool clean = run.has_value() && up_to_sign(run->state.model, truth) < 1e-10;
  for (int i = 0; clean && i < 40; ++i) {
    clean = run->state.inliers(i) == (i % 3 != 0);
  }
  expect(clean, "E-RANSAC finds E and exactly the clean pairs", failures);

  // A plane: E is not unique, H is.
  const Eigen::MatrixX3d flat = points(20, true);
  epipolar::Rows3 p1(20, 3);
  epipolar::Rows3 p2(20, 3);
  epipolar::Rows2 x1(20, 2);
  epipolar::Rows2 x2(20, 2);
  for (int i = 0; i < 20; ++i) {
    const Eigen::Vector3d x = flat.row(i).transpose();
    const Eigen::Vector3d y = (rotation * x) + t;
    p1.row(i) = x.normalized().transpose();
    p2.row(i) = y.normalized().transpose();
    x1.row(i) = x.head<2>().transpose() / x(2);
    x2.row(i) = y.head<2>().transpose() / y(2);
  }
  const auto planar_e = epipolar::essential_8pt(p1, p2);
  expect(!planar_e.has_value() &&
             planar_e.error().kind == slamlab::foundations::ErrorKind::kDegenerate,
         "a planar scene does not determine E", failures);
  const Eigen::Matrix3d h_truth =
      (rotation + (t * Eigen::Vector3d(0.0, 0.0, 1.0).transpose() / 5.0)).normalized();
  const auto h = homography::homography_dlt(x1, x2);
  expect(h.has_value() && up_to_sign(*h, h_truth) < 1e-12, "H = R + t n^T / d", failures);
  const Eigen::VectorXd errors = homography::sampson(h_truth, x1, x2);
  expect(errors.maxCoeff() < 1e-25, "the Sampson error vanishes on the model", failures);

  epipolar::Rows2 line(4, 2);
  line << 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 0.0, 1.0;
  expect(!homography::homography_dlt(line, line).has_value(),
         "three collinear points do not determine H", failures);
  return slamlab::testing::finish("foundations/twoview_solvers", failures);
}
