/// @file
/// @brief P3P: the C++ core's camera pose from three world points and their bearings.
///
/// Mirrors `slam_it_all.foundations.p3p`, whose docstring states the maths: the three distance
/// equations in the depths, two conics whose pencil holds a degenerate member (a root of a
/// cubic), split into two lines and intersected with a conic, the depths polished by Newton's
/// method, and the pose as the rigid alignment of the two triangles. Lambda Twist (Persson and
/// Nordberg, ECCV 2018) splits the line pair by its eigen-decomposition; Ding et al. (CVPR 2023) by
/// its adjugate, on depth ratios referred to the longest side. Every pose with all three depths
/// positive is returned, as `T_cam_world`.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "slam_it_all/foundations/correspondences.hpp"
#include "slam_it_all/foundations/error.hpp"

/// @brief P3P.
namespace slam_it_all::foundations::p3p {

/// @brief Correspondences a P3P solver takes.
inline constexpr std::uint32_t kSampleSize = 3;
/// @brief At most this many Newton steps polish the cubic's root, and the depths.
inline constexpr int kPolishMaxSteps = 10;
/// @brief Polishing stops when a step is below this, relative to the value polished.
inline constexpr double kPolishTolerance = 1e-14;
/// @brief A negative discriminant above this times `-(b^2 + |4ac|)` counts as zero (a double
///        root); a positive one, however small, gives two roots.
inline constexpr double kRealTolerance = 1e-12;
/// @brief A solution is kept only if its polished depths satisfy the distance equations to
///        within this times the largest squared distance (see
///        `slam_it_all.foundations.p3p.DISTANCE_TOLERANCE`).
inline constexpr double kDistanceTolerance = 1e-6;
/// @brief The world points are collinear when `|X12 x X13|` is below this times
///        `|X12| |X13|`.
inline constexpr double kCollinearTolerance = 1e-10;

/// @brief The solvers, with the codes they cross the bindings as.
enum class Solver : std::uint8_t {
  /// @brief Ding et al.'s solver.
  kDing = 0,
  /// @brief Lambda Twist.
  kLambdaTwist = 1,
  /// @brief The number of solvers (a sentinel).
  kCount = 2,
};

/// @brief The solver's name, as `slam_it_all.foundations.p3p.SOLVERS` spells it.
/// @param solver The solver.
/// @return A static string.
[[nodiscard]] const char* name(Solver solver);

/// @brief The largest real root of the monic cubic `z^3 + c2 z^2 + c1 z + c0`, in closed form
///        and polished by Newton's method (see `slam_it_all.foundations.p3p.cubic_root`).
/// @param c2 The `z^2` coefficient.
/// @param c1 The `z` coefficient.
/// @param c0 The constant.
/// @return The root.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the coefficients, highest first
[[nodiscard]] double cubic_root(double c2, double c1, double c0);

/// @brief Every camera pose three world points and their bearings allow (unchecked).
/// @param points Three world points.
/// @param bearings Their unit bearings in the camera.
/// @param solver The solver.
/// @return The poses `T_cam_world` with every depth positive (none for collinear points).
[[nodiscard]] std::vector<Eigen::Matrix4d> solve(
    const Eigen::Ref<const correspondences::Rows3>& points,
    const Eigen::Ref<const correspondences::Rows3>& bearings, Solver solver);

/// @brief Every camera pose three world points and their bearings allow.
/// @param points Three world points.
/// @param bearings Their unit bearings in the camera (any sign of z).
/// @param solver The solver.
/// @return The poses; `InvalidInput` unless there are three finite points and unit bearings,
///         or for an unknown solver.
[[nodiscard]] Result<std::vector<Eigen::Matrix4d>> p3p(
    const Eigen::Ref<const correspondences::Rows3>& points,
    const Eigen::Ref<const correspondences::Rows3>& bearings, Solver solver);

}  // namespace slam_it_all::foundations::p3p
