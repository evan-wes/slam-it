/// @file
/// @brief Triangulation: the C++ core's two-view optimal (Lindstrom's niter2) and N-view
///        linear (DLT on bearings) methods.
///
/// Mirrors `slam_it.foundations.triangulation`, whose docstring states the maths. Rays are
/// unit bearing vectors; poses are `T_cam_world`. A point whose rays are parallel is NaN, with
/// `in_front` false and parallax 0: counted by the caller, never raised.
#pragma once

#include <Eigen/Core>
#include <vector>

#include "slam_it/foundations/correspondences.hpp"
#include "slam_it/foundations/error.hpp"

/// @brief Triangulation.
namespace slam_it::foundations::triangulation {

/// @brief Rays meeting at an angle whose sine is below this are parallel; the DLT uses the
///        same ratio between the homogeneous point's last coordinate and the rest.
inline constexpr double kParallelSine = 1e-12;

using correspondences::Rows3;

/// @brief Which views observe each point (P x V), row-major as numpy stores it.
using MaskRef =
    Eigen::Ref<const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

/// @brief Triangulated points, with their cheirality and parallax.
struct Triangulation {
  /// @brief The points (P x 3); NaN where the rays are parallel.
  Rows3 points;
  /// @brief Whether each point lies ahead along every ray that observes it.
  Eigen::Array<bool, Eigen::Dynamic, 1> in_front;
  /// @brief The largest angle at each point between two observing camera centres (radians).
  Eigen::VectorXd parallax;
};

/// @brief Triangulate bearing pairs optimally through a known relative pose (Lindstrom's
///        niter2, then the exact intersection of the corrected rays).
/// @param relative_pose `T_c2_c1`, with a non-zero translation.
/// @param bearings1 Unit bearings in camera 1 (P x 3), positive z.
/// @param bearings2 Unit bearings in camera 2 (P x 3), positive z.
/// @return The points in camera 1's frame; `InvalidInput` for a bad pose, a zero translation,
///         mismatched sizes, or bearings that are not unit vectors with positive z.
[[nodiscard]] Result<Triangulation> two_view(const Eigen::Matrix4d& relative_pose,
                                             const Eigen::Ref<const Rows3>& bearings1,
                                             const Eigen::Ref<const Rows3>& bearings2);

/// @brief Triangulate points seen in two or more views, by the DLT on bearing vectors.
/// @param poses Each view's `T_cam_world` (V).
/// @param bearings Each point's bearing in each view, row `p * V + v` (P*V x 3).
/// @param observed Which views observe each point (P x V); each point needs two.
/// @return The points in the world frame (NaN where undetermined: at infinity, or rays along
///         one line); `InvalidInput` for a bad pose, mismatched sizes, an
///         observed bearing that is not a unit vector, or a point seen in fewer than two views.
[[nodiscard]] Result<Triangulation> n_view(const std::vector<Eigen::Matrix4d>& poses,
                                           const Eigen::Ref<const Rows3>& bearings,
                                           const MaskRef& observed);

}  // namespace slam_it::foundations::triangulation
