/// @file
/// @brief Native tests for the correspondence building blocks
///        (slam_it/foundations/correspondences.hpp).
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "slam_it/foundations/correspondences.hpp"
#include "test_support.hpp"

int main() {
  namespace co = slam_it::foundations::correspondences;
  using slam_it::testing::expect;
  int failures = 0;

  co::Rows2 points(4, 2);
  points << 100.0, 200.0, 300.0, 250.0, 150.0, 400.0, 500.0, 120.0;
  const auto t = co::hartley_normalisation(points);
  bool centred = t.has_value();
  if (t) {
    const co::Rows3 mapped = co::homogeneous(points) * t->transpose();
    centred = mapped.leftCols<2>().colwise().mean().norm() < 1e-12 &&
              std::abs(mapped.leftCols<2>().rowwise().norm().mean() - std::numbers::sqrt2) < 1e-12;
  }
  expect(centred, "Hartley's similarity centres and scales to sqrt(2)", failures);
  expect(!co::hartley_normalisation(co::Rows2::Ones(3, 2)).has_value(),
         "coincident points have no normalisation", failures);

  const Eigen::Matrix<double, Eigen::Dynamic, 9> rank7 =
      Eigen::Matrix<double, Eigen::Dynamic, 9>::Random(12, 9).leftCols<7>() *
      Eigen::Matrix<double, 7, 9>::Random();
  expect(!co::null_vector(rank7).has_value(), "a rank-7 system has no unique null vector",
         failures);

  const co::Rows3 unit = co::Rows3::Constant(8, 3, std::numbers::inv_sqrt3);
  expect(co::check_bearing_pairs(unit, unit, 8).has_value(), "valid bearings pass", failures);
  expect(!co::check_bearing_pairs(unit, unit, 9).has_value(), "too few pairs fail", failures);
  const co::Rows3 backward = -unit;
  expect(!co::check_bearing_pairs(unit, backward, 0).has_value(), "negative z fails", failures);
  const co::Rows3 long_rays = 2.0 * unit;
  expect(!co::check_bearing(long_rays, 0, "b").has_value(), "a non-unit bearing fails", failures);
  expect(!co::check_point_pairs(co::Rows2::Ones(3, 2), co::Rows2::Ones(4, 2), 0, "p").has_value(),
         "unequal point counts fail", failures);

  const std::vector<std::uint32_t> pick = {3, 0};
  const co::Rows2 gathered = co::gather(points, pick);
  expect(
      gathered.rows() == 2 && gathered.row(0) == points.row(3) && gathered.row(1) == points.row(0),
      "gather takes the listed rows in order", failures);
  return slam_it::testing::finish("foundations/correspondences", failures);
}
