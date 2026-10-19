/// @file
/// @brief Native tests for the observation errors and the bundle adjustment
///        (slam_it/edge/{residuals,adjust}.hpp): the chi-square terms, and an adjustment of the
///        exact scene that recovers it and forgets an observation moved off its point. Parity
///        with the Python reference is checked through the bindings by tests/edge.
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/adjust.hpp"
#include "slam_it/edge/residuals.hpp"

namespace {

namespace edge = slam_it::edge;
namespace scene = slam_it::testing::scene;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief NaN, for a missing right column.
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief Check the chi-square errors: both terms, points behind, the checks.
/// @param failures Running failure count.
void check_chi2(int& failures) {
  const edge::Camera camera = scene::camera();
  const std::vector<Eigen::Vector3d> local{{0.0, 0.0, 2.0}, {0.0, 0.0, 2.0}, {0.0, 0.0, -1.0}};
  // The second's predicted right column is 160 - 250 * 0.1 / 2 = 147.5.
  const std::vector<std::array<double, 2>> xy{{163.0, 124.0}, {160.0, 120.0}, {160.0, 120.0}};
  const std::vector<double> right{kNan, 145.5, 147.5};
  const auto errors = edge::chi2(camera, local, xy, right, std::vector<double>{1.0, 4.0, 1.0});
  expect(errors.has_value() && std::abs((*errors)[0] - 25.0) < 1e-12 &&
             std::abs((*errors)[1] - 1.0) < 1e-12 && std::isinf((*errors)[2]),
         "chi2 weighs both terms, and a point behind is infinitely wrong", failures);
  const std::vector<Eigen::Vector3d> one{{0.0, 0.0, 2.0}};
  const std::vector<std::array<double, 2>> at{{0.0, 0.0}};
  expect(is_invalid_input(edge::chi2(camera, one, at,
                                     std::vector<double>{std::numeric_limits<double>::infinity()},
                                     std::vector<double>{1.0})),
         "chi2 rejects an infinite right column", failures);
  expect(is_invalid_input(
             edge::chi2(camera, one, at, std::vector<double>{kNan}, std::vector<double>{0.0})),
         "chi2 rejects a variance that is not positive", failures);
  const edge::Options options;
  expect(edge::threshold(kNan, options) == options.adjustment.chi2_mono &&
             edge::threshold(10.0, options) == options.adjustment.chi2_stereo,
         "the threshold follows the right column", failures);
}

/// @brief Check the adjustment of the exact scene from a perturbed start.
/// @param failures Running failure count.
void check_recovery(int& failures) {
  auto built = scene::map();
  expect(built.has_value(), "the scene's map builds", failures);
  if (!built) {
    return;
  }
  edge::Map m = *built;
  for (std::size_t k = 1; k < scene::kKeyframes; ++k) {
    m.kf_pose[k].topRightCorner<3, 1>() += Eigen::Vector3d(0.01, -0.01, 0.02);
  }
  for (std::size_t p = 0; p < scene::kPoints; ++p) {
    const auto s = static_cast<double>(p);
    m.pt_xyz[p] += 0.01 * Eigen::Vector3d(std::sin(s), std::cos(s), std::sin(2.0 * s));
  }
  const auto moved = static_cast<std::size_t>(m.kf_start[2]);
  m.kp_xy[moved][0] += 30.0;  // point 0's third view, pulling it off along the first's ray
  const Eigen::Vector3d start = m.pt_xyz[0];
  const std::vector<std::int64_t> every{0, 1, 2};
  const std::vector<std::uint8_t> fixed{1, 0, 0};
  const auto adjusted = edge::bundle_adjust(m, scene::camera(), every, fixed, edge::Options{});
  expect(adjusted.has_value(), "the adjustment runs", failures);
  if (adjusted) {
    expect(adjusted->solves.size() == 2 && adjusted->outliers == 1 &&
               adjusted->solves[0].final_cost < adjusted->solves[0].initial_cost,
           "it records its two solves and the one observation it dropped", failures);
  }
  expect(m.kf_pose[0] == Eigen::Matrix4d::Identity(), "the fixed keyframe keeps its pose exactly",
         failures);
  double worst = 0.0;
  for (std::size_t k = 1; k < scene::kKeyframes; ++k) {
    worst = std::max(worst, (m.kf_pose[k] - scene::pose(k)).cwiseAbs().maxCoeff());
  }
  const std::vector<Eigen::Vector3d> truth = scene::points();
  for (std::size_t p = 1; p < edge::point_count(m); ++p) {
    worst = std::max(worst, (m.pt_xyz[p] - truth[static_cast<std::size_t>(m.pt_id[p])]).norm());
  }
  expect(worst < 1e-6, "the adjustment recovers the scene's poses and points", failures);
  expect(m.kp_point[moved] == -1 && edge::point_count(m) == scene::kPoints && m.pt_id[0] == 0 &&
             m.kp_point[0] == 0 && m.kp_point[scene::kPoints] == 0,
         "the moved observation is forgotten, its point and other views kept", failures);
  expect(m.pt_xyz[0] == start, "a point left with one inlier view goes back to where it started",
         failures);
}

/// @brief Check that with no keyframe held fixed nothing is solved: ORB-SLAM3 skips it.
/// @param failures Running failure count.
void check_no_gauge(int& failures) {
  auto built = scene::map();
  if (!built) {
    expect(false, "the scene's map builds", failures);
    return;
  }
  edge::Map m = *built;
  m.kf_pose[1].topRightCorner<3, 1>() += Eigen::Vector3d(0.01, -0.01, 0.02);
  const edge::Map before = m;
  const std::vector<std::int64_t> every{0, 1, 2};
  const std::vector<std::uint8_t> free{0, 0, 0};
  const auto adjusted = edge::bundle_adjust(m, scene::camera(), every, free, edge::Options{});
  expect(adjusted.has_value() && adjusted->solves.empty() && adjusted->free == 0 &&
             adjusted->fixed == 0 && m.kf_pose == before.kf_pose && m.pt_xyz == before.pt_xyz,
         "with no keyframe held fixed, nothing is solved", failures);
}

/// @brief Check the argument checks.
/// @param failures Running failure count.
void check_arguments(int& failures) {
  auto built = scene::map();
  if (!built) {
    expect(false, "the scene's map builds", failures);
    return;
  }
  edge::Map m = *built;
  expect(is_invalid_input(edge::bundle_adjust(m, scene::camera(), std::vector<std::int64_t>{3},
                                              std::vector<std::uint8_t>{1, 0, 0}, edge::Options{})),
         "bundle_adjust rejects a keyframe row past the end", failures);
  expect(is_invalid_input(edge::bundle_adjust(m, scene::camera(), std::vector<std::int64_t>{0},
                                              std::vector<std::uint8_t>{1}, edge::Options{})),
         "bundle_adjust rejects flags that are not one per keyframe", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_chi2(failures);
  check_recovery(failures);
  check_no_gauge(failures);
  check_arguments(failures);
  return slam_it::testing::finish("edge/adjust", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/adjust", run); }
