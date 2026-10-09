/// @file
/// @brief Reading a Bundle Adjustment in the Large (BAL) problem into a `ba::Problem`,
///        natively, for the benchmarks.
///
/// The same conversion as the Python reader (`slam_it_all.eval.readers.read_bal`) and the Rust
/// example's: BAL's camera frame flips `y` and `z` relative to this package's, its cameras are
/// world-to-camera `(omega, t, f, k1, k2)`, and its observations are centred pixels with `y`
/// up. One radial intrinsics group per camera, `f, k1, k2` free, the principal point fixed.
/// Plain text only: decompress a downloaded `.bz2` first (`bunzip2 -k`).
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "slam_it_all/foundations/ba.hpp"
#include "slam_it_all/foundations/se3.hpp"
#include "slam_it_all/foundations/so3.hpp"

/// @brief Benchmark support, outside the cores.
namespace slam_it_all::bench {

/// @brief Read a BAL problem from a plain-text file.
/// @param path The file.
/// @return The problem, or nothing if the file is not a BAL problem.
inline std::optional<foundations::ba::Problem> read_bal(const std::string& path) {
  namespace ba = foundations::ba;
  std::ifstream file(path);
  std::size_t n_cams = 0;
  std::size_t n_points = 0;
  std::size_t n_obs = 0;
  if (!(file >> n_cams >> n_points >> n_obs)) {
    return std::nullopt;
  }
  const auto obs = static_cast<Eigen::Index>(n_obs);
  ba::Pixels observations(obs, 2);
  ba::Indices cam_idx;
  ba::Indices pt_idx;
  for (Eigen::Index m = 0; m < obs; ++m) {
    std::int64_t c = 0;
    std::int64_t p = 0;
    double x = 0.0;
    double y = 0.0;
    if (!(file >> c >> p >> x >> y)) {
      return std::nullopt;
    }
    cam_idx.push_back(c);
    pt_idx.push_back(p);
    observations.row(m) << x, -y;
  }
  const Eigen::Matrix3d flip = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  ba::Intrinsics intrinsics(static_cast<Eigen::Index>(n_cams), 5);
  ba::Poses poses;
  for (std::size_t c = 0; c < n_cams; ++c) {
    std::array<double, 9> v{};
    for (double& value : v) {
      if (!(file >> value)) {
        return std::nullopt;
      }
    }
    Eigen::Matrix4d t_cam_world = Eigen::Matrix4d::Identity();
    t_cam_world.topLeftCorner<3, 3>() = flip * foundations::so3::exp({v[0], v[1], v[2]});
    t_cam_world.topRightCorner<3, 1>() = flip * Eigen::Vector3d(v[3], v[4], v[5]);
    poses.push_back(*foundations::se3::inverse(t_cam_world));
    intrinsics.row(static_cast<Eigen::Index>(c)) << v[6], 0.0, 0.0, v[7], v[8];
  }
  ba::Points points(static_cast<Eigen::Index>(n_points), 3);
  for (Eigen::Index p = 0; p < static_cast<Eigen::Index>(n_points); ++p) {
    if (!(file >> points(p, 0) >> points(p, 1) >> points(p, 2))) {
      return std::nullopt;
    }
  }
  ba::Problem problem = ba::Problem::with_defaults(
      foundations::camera::Model::kRadial, std::move(intrinsics), std::move(poses),
      std::move(points), std::move(observations), std::move(cam_idx), std::move(pt_idx));
  // One intrinsics group per camera, f, k1, k2 free and the principal point fixed.
  problem.fixed_intrinsics.clear();
  for (std::size_t c = 0; c < n_cams; ++c) {
    problem.intrinsics_idx[c] = static_cast<std::int64_t>(c);
    problem.fixed_intrinsics.insert(problem.fixed_intrinsics.end(),
                                    {false, true, true, false, false});
  }
  return problem;
}

}  // namespace slam_it_all::bench
