/// @file
/// @brief A small exact scene shared by the native edge tests: 60 points before three
///        keyframes, every keypoint at its point's projection, so a bundle adjustment's
///        optimum is the truth.
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "slam_it_all/edge/frames.hpp"
#include "slam_it_all/edge/map.hpp"

namespace slam_it_all::testing::scene {

/// @brief The scene's points.
inline constexpr std::size_t kPoints = 60;
/// @brief Its keyframes.
inline constexpr std::size_t kKeyframes = 3;

/// @brief The scene's camera: 250 px focal, 320 x 240, a 0.1 m baseline.
/// @return The camera.
inline edge::Camera camera() {
  return {.model = foundations::camera::Model::kPinhole,
          .intrinsics = Eigen::Vector4d(250.0, 250.0, 160.0, 120.0),
          .width = 320,
          .height = 240,
          .baseline = 0.1};
}

/// @brief The points: a 10 x 6 grid three metres out, its depths rippled so it is no plane.
/// @return The points.
inline std::vector<Eigen::Vector3d> points() {
  std::vector<Eigen::Vector3d> out;
  for (std::size_t p = 0; p < kPoints; ++p) {
    const auto i = static_cast<double>(p % 10);
    const auto j = static_cast<double>(p / 10);
    out.emplace_back(-1.0 + (i * 2.0 / 9.0), -0.6 + (j * 0.24),
                     3.0 + (0.4 * std::sin(1.7 * static_cast<double>(p))));
  }
  return out;
}

/// @brief A keyframe's true `T_cam_world`: the camera moves right, then up and forward.
/// @param k The keyframe.
/// @return The pose.
inline Eigen::Matrix4d pose(std::size_t k) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  if (k == 1) {
    out.topRightCorner<3, 1>() = Eigen::Vector3d(-0.2, 0.0, 0.0);
  } else if (k == 2) {
    out.topRightCorner<3, 1>() = Eigen::Vector3d(-0.4, 0.05, 0.1);
  }
  return out;
}

/// @brief A frame of the scene: one keypoint at each point's projection, on one level, each
///        with its own descriptor, and its depth and right column when asked.
/// @param t The camera's `T_cam_world`.
/// @param with_depth Whether the keypoints have depth.
/// @param level The keypoints' level.
/// @return The frame.
inline edge::Frame frame_at(const Eigen::Matrix4d& t, bool with_depth, std::int64_t level = 0) {
  const edge::Camera cam = camera();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  edge::Frame out;
  out.features.levels = 8;
  out.features.scale_factor = 1.2;
  const double scale = std::pow(1.2, static_cast<double>(level));
  const std::vector<Eigen::Vector3d> world = points();
  for (std::size_t p = 0; p < kPoints; ++p) {
    const Eigen::Vector3d local = (t.topLeftCorner<3, 3>() * world[p]) + t.topRightCorner<3, 1>();
    const Eigen::Vector2d px = edge::project(cam, local);
    out.features.keypoints.push_back(
        {.xy = {px.x(), px.y()},
         .level = level,
         .level_xy = {std::llround(px.x() / scale), std::llround(px.y() / scale)},
         .response = 0,
         .angle = 0});
    foundations::orb::Descriptor d{};
    for (std::size_t b = 0; b < d.size(); ++b) {
      d.at(b) = static_cast<std::uint8_t>(((p * 37U) + (b * 11U)) & 0xFFU);
    }
    out.features.descriptors.push_back(d);
    out.bearings.push_back(edge::bearing(cam, px));
    out.depth.push_back(with_depth ? local.z() : nan);
    out.right.push_back(with_depth ? px.x() - (edge::focal(cam) * cam.baseline / local.z()) : nan);
  }
  return edge::make_frame(out.timestamp, std::move(out.features), std::move(out.bearings),
                          std::move(out.depth), std::move(out.right))
      .value();
}

/// @brief A keyframe's frame: its keypoints on level 0 (see `frame_at`).
/// @param k The keyframe.
/// @param with_depth Whether the keypoints have depth.
/// @return The frame.
inline edge::Frame frame(std::size_t k, bool with_depth) {
  edge::Frame out = frame_at(pose(k), with_depth);
  out.timestamp = static_cast<double>(k);
  return out;
}

/// @brief The pose of a camera moved along its axis from the origin.
/// @param dz How far, in metres.
/// @return Its `T_cam_world`.
inline Eigen::Matrix4d moved(double dz) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out(2, 3) = -dz;
  return out;
}

/// @brief The scene's map: the first keyframe makes a point at each keypoint, the others
///        observe them all.
/// @return The map, or the map's error.
inline foundations::Result<edge::Map> map() {
  edge::Map m = edge::empty_map(8, 1.2);
  const std::vector<std::int64_t> none(kPoints, -1);
  if (auto ok = edge::insert_keyframe(m, frame(0, true), pose(0), none); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<std::vector<std::int64_t>> observers;
  std::vector<std::int64_t> ids;
  for (std::size_t p = 0; p < kPoints; ++p) {
    observers.push_back({static_cast<std::int64_t>(p)});
    ids.push_back(static_cast<std::int64_t>(p));
  }
  if (auto ok = edge::add_points(m, points(), observers); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  for (std::size_t k = 1; k < kKeyframes; ++k) {
    if (auto ok = edge::insert_keyframe(m, frame(k, true), pose(k), ids); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  return m;
}

}  // namespace slam_it_all::testing::scene
