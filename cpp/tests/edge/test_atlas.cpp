/// @file
/// @brief Native tests for the atlas (slam_it/edge/atlas.hpp) through a monocular merge, on the
///        exact scene's map: frames tracked against the active map's keyframes, one keyframe
///        then culled and one point removed, all carried into the matched map's world and units
///        by a scaled `G`, against the known answer `rigid_of(T G^-1)` (as
///        `tests/edge/test_atlas.py` holds Python's through a whole merge).
#include <Eigen/Core>
#include <cstdint>
#include <numbers>
#include <optional>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "scene.hpp"
#include "slam_it/edge/atlas.hpp"
#include "slam_it/edge/loops.hpp"
#include "slam_it/edge/map.hpp"
#include "slam_it/edge/options.hpp"
#include "slam_it/foundations/sim3.hpp"
#include "slam_it/foundations/so3.hpp"

namespace {

namespace edge = slam_it::edge;
namespace scene = slam_it::testing::scene;
namespace sim3 = slam_it::foundations::sim3;
using slam_it::testing::expect;

/// @brief The scale `G` carries the active map's world into the matched map's by: its world
///        is the matched map's scaled by 0.7, as a monocular map started apart has its own units.
constexpr double kScale = 1.0 / 0.7;

/// @brief The point removed from the active map before the merge.
constexpr std::int64_t kRemoved = 5;

/// @brief Poses and points agree with the known answer to this: exact arithmetic, rounded.
constexpr double kTolerance = 1e-12;

/// @brief `G`: a turn of 30 degrees about y, a shift and `kScale`.
/// @return The similarity.
Eigen::Matrix4d to_world() {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() =
      kScale * slam_it::foundations::so3::exp(Eigen::Vector3d(0.0, std::numbers::pi / 6.0, 0.0));
  out.topRightCorner<3, 1>() = Eigen::Vector3d(0.4, -0.1, 0.25);
  return out;
}

/// @brief A camera's pose as the merge carries it into the matched map: `rigid_of(T G^-1)`.
/// @param pose Its `T_cam_world` in the active map.
/// @return Its `T_cam_world` in the matched map.
Eigen::Matrix4d carried(const Eigen::Matrix4d& pose) {
  return edge::loops::rigid_of(pose * sim3::inverse(to_world()));
}

/// @brief Whether two matrices agree to `kTolerance`.
/// @param got One.
/// @param expected The other.
/// @return Whether they do.
template <typename Matrix>
bool near(const Matrix& got, const Matrix& expected) {
  return (got - expected).cwiseAbs().maxCoeff() < kTolerance;
}

/// @brief Check the records a monocular merge carries: two frames tracked in the active map,
///        one against keyframe 1, one against keyframe 2, which is then culled, so its frame is
///        placed through its parent; and the culled keyframe's and the removed point's last
///        places.
/// @param failures Running failure count.
void check_merge_carries_the_records(int& failures) {
  auto matched = scene::map();
  auto active = scene::map();
  if (!matched || !active) {
    expect(false, "the scene's maps are built", failures);
    return;
  }
  active->kf_parent = {-1, 0, 1};
  edge::Atlas atlas(edge::Options{});
  atlas.start();
  atlas.date(&*matched);
  atlas.close(*matched, std::nullopt, false, true);
  atlas.start();
  atlas.date(&*active);
  const Eigen::Matrix4d through_culled = scene::moved(0.2) * scene::pose(2);
  atlas.place_frame(*active, 1.0, 1, scene::pose(2));
  atlas.place_frame(*active, 2.0, 2, through_culled);
  std::vector<std::uint8_t> removed(scene::kPoints, 0);
  removed[kRemoved] = 1;
  if (!edge::erase_keyframe(*active, 2) ||
      !edge::erase_points(*active, removed, edge::Removal::kCulled)) {
    expect(false, "the active map's keyframe and point are removed", failures);
    return;
  }
  atlas.record(&*active);
  static_cast<void>(atlas.merge(0, to_world()));
  // The merged map: the matched map's keyframes, then the active map's left, carried.
  edge::Map merged = *matched;
  const std::vector<std::int64_t> none(scene::kPoints, -1);
  for (std::size_t k = 0; k < 2; ++k) {
    if (!edge::insert_keyframe(merged, scene::frame(k, false), carried(scene::pose(k)), none)) {
      expect(false, "the merged map is built", failures);
      return;
    }
  }
  const edge::Trajectory trajectory = atlas.trajectory(&merged);
  expect(trajectory.poses.size() == 2 && near(trajectory.poses[0], carried(scene::pose(2))) &&
             near(trajectory.poses[1], carried(through_culled)),
         "each frame lands where the merge carries it, through a culled keyframe too", failures);
  expect(trajectory.maps == std::vector<std::int64_t>{0, 0} &&
             trajectory.tracked_maps == std::vector<std::int64_t>{1, 1},
         "the frames end in the matched map, tracked in the active one", failures);
  const edge::Lifetimes lives = atlas.lifetimes(&merged);
  const auto keyframe = static_cast<std::size_t>(scene::kKeyframes + 2);
  const auto point = static_cast<std::size_t>(scene::kPoints) + kRemoved;
  const Eigen::Matrix4d g = to_world();
  const Eigen::Vector3d moved =
      (g.topLeftCorner<3, 3>() * scene::points()[kRemoved]) + g.topRightCorner<3, 1>();
  expect(near(lives.keyframe_poses[keyframe], carried(scene::pose(2))) &&
             lives.keyframe_maps[keyframe] == 0,
         "the culled keyframe's last pose is carried, rigidly", failures);
  expect(near(lives.points[point], moved) && lives.point_maps[point] == 0,
         "the removed point's last place is carried by G", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_merge_carries_the_records(failures);
  return slam_it::testing::finish("edge/atlas", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/atlas", run); }
