/// @file
/// @brief Native tests for the 5-point solvers (slam_it/foundations/fivepoint.hpp): the true E
///        among each solver's solutions, every solution essential, Sturm roots, bad input.
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "scene_support.hpp"
#include "slam_it/foundations/fivepoint.hpp"
#include "test_support.hpp"

namespace {

namespace fivepoint = slam_it::foundations::fivepoint;
using slam_it::testing::expect;
using slam_it::testing::Rows3;
using slam_it::testing::up_to_sign;

/// @brief Whether a solver finds the true E among essential solutions, on five pairs.
/// @param b1 Five bearings in camera 1.
/// @param b2 Five bearings in camera 2.
/// @param truth The true E.
/// @param solver The solver.
/// @return True if one solution is the truth and every solution is essential.
bool finds_truth(const Rows3& b1, const Rows3& b2, const Eigen::Matrix3d& truth,
                 fivepoint::Solver solver) {
  const auto found = fivepoint::essential_5pt(b1, b2, solver);
  if (!found.has_value() || found->empty()) {
    return false;
  }
  bool has_truth = false;
  for (const Eigen::Matrix3d& e : *found) {
    const Eigen::Vector3d singular = Eigen::JacobiSVD<Eigen::Matrix3d>(e).singularValues();
    if ((singular - Eigen::Vector3d(1.0, 1.0, 0.0)).cwiseAbs().maxCoeff() > 1e-8) {
      return false;
    }
    has_truth = has_truth || up_to_sign(e, truth) < 1e-8;
  }
  return has_truth;
}

}  // namespace

int main() {
  int failures = 0;
  const Eigen::Matrix3d truth = slam_it::testing::scene_essential();
  Rows3 b1;
  Rows3 b2;
  slam_it::testing::scene_bearings(
      25, false, {slam_it::testing::scene_rotation(), slam_it::testing::scene_translation()}, b1,
      b2);
  for (const fivepoint::Solver solver :
       {fivepoint::Solver::kNister, fivepoint::Solver::kStewenius}) {
    bool all = true;
    for (int start = 0; start + 5 <= 25; start += 5) {
      all = all && finds_truth(b1.middleRows(start, 5), b2.middleRows(start, 5), truth, solver);
    }
    const std::string what = std::string(fivepoint::name(solver)) + " finds E on every sample";
    expect(all, what.c_str(), failures);
  }

  // (z + 2)(z - 0.5)(z - 1)(z - 3.5)(z^2 + 1): four real roots, two complex.
  const std::vector<double> poly = {-3.5, 9.75, -7.75, 6.75, -3.25, -3.0, 1.0};
  const std::vector<double> roots = fivepoint::real_roots(poly);
  const std::vector<double> expected = {-2.0, 0.5, 1.0, 3.5};
  bool matches = roots.size() == expected.size();
  for (std::size_t i = 0; matches && i < roots.size(); ++i) {
    matches = std::abs(roots[i] - expected[i]) < 1e-12;
  }
  expect(matches, "Sturm sequences find exactly the real roots", failures);

  const auto four =
      fivepoint::essential_5pt(b1.topRows(4), b2.topRows(4), fivepoint::Solver::kNister);
  expect(!four.has_value() && four.error().kind == slam_it::foundations::ErrorKind::kInvalidInput,
         "four pairs are rejected", failures);
  const auto six =
      fivepoint::essential_5pt(b1.topRows(6), b2.topRows(6), fivepoint::Solver::kStewenius);
  expect(!six.has_value(), "six pairs are rejected", failures);
  const auto unknown =
      fivepoint::essential_5pt(b1.topRows(5), b2.topRows(5), fivepoint::Solver::kCount);
  expect(!unknown.has_value(), "an unknown solver is rejected", failures);

  // A repeated pair leaves a rank-4 system: no isolated solution.
  Rows3 r1 = b1.topRows(5);
  Rows3 r2 = b2.topRows(5);
  r1.row(4) = r1.row(3);
  r2.row(4) = r2.row(3);
  // No baseline: every [t]x R fits, and only essential matrices may come back.
  const Eigen::Matrix3d spin =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.0, 1.0, 0.2).normalized()).toRotationMatrix();
  const Rows3 p2 = b1.topRows(5) * spin.transpose();
  for (const fivepoint::Solver solver :
       {fivepoint::Solver::kNister, fivepoint::Solver::kStewenius}) {
    const auto repeated = fivepoint::essential_5pt(r1, r2, solver);
    expect(repeated.has_value() && repeated->empty(), "a repeated pair gives no solution",
           failures);
    const auto rotated = fivepoint::essential_5pt(b1.topRows(5), p2, solver);
    bool essential = rotated.has_value();
    for (const Eigen::Matrix3d& e : rotated.value_or(std::vector<Eigen::Matrix3d>{})) {
      const Eigen::Vector3d singular = Eigen::JacobiSVD<Eigen::Matrix3d>(e).singularValues();
      essential = essential && (singular - Eigen::Vector3d(1.0, 1.0, 0.0)).cwiseAbs().maxCoeff() <=
                                   fivepoint::kEssentialTolerance;
    }
    expect(essential, "a pure rotation returns only essential matrices", failures);
  }
  return slam_it::testing::finish("foundations/fivepoint", failures);
}
