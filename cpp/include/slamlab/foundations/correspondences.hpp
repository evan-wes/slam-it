/// @file
/// @brief Building blocks for estimating geometry from correspondences between two views: the
///        C++ core's mirror of `slamlab.foundations.correspondences`.
///
/// Unchecked helpers shared by the two-view estimators (`epipolar`, `homography`) and, later,
/// absolute pose: homogeneous and normalised image points, Hartley's normalisation, the null
/// vector of a linear system, row gathering. The checks the estimators run on their input
/// (`check_bearing_pairs`, `check_point_pairs`; in Python, `slamlab.foundations.arrays`) are
/// here too, so each core validates once with the same rules.
#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "slamlab/foundations/error.hpp"

/// @brief Correspondences between two views.
namespace slamlab::foundations::correspondences {

/// @brief The 8-point system's second-smallest singular value must exceed this fraction of its
///        largest; below it the solution is not unique.
inline constexpr double kRankTolerance = 1e-10;
/// @brief Rows of 3-vectors (bearings, homogeneous points), row-major as numpy stores them.
using Rows3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
/// @brief Rows of 2-vectors (pixels, image points), row-major as numpy stores them.
using Rows2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

/// @brief Append a 1 to each 2D point.
/// @param points The points (N x 2).
/// @return `(x, y, 1)` per point.
[[nodiscard]] Rows3 homogeneous(const Eigen::Ref<const Rows2>& points);

/// @brief Map bearings with positive z to homogeneous normalised image points `f / f_z`.
/// @param bearings The bearings (N x 3).
/// @return `(x, y, 1)` per bearing.
[[nodiscard]] Rows3 normalised_plane(const Eigen::Ref<const Rows3>& bearings);

/// @brief Map bearings with positive z to normalised image points `(f_x / f_z, f_y / f_z)`.
/// @param bearings The bearings (N x 3).
/// @return `(x, y)` per bearing: `normalised_plane` without its ones.
[[nodiscard]] Rows2 normalised_xy(const Eigen::Ref<const Rows3>& bearings);

/// @brief Hartley's similarity: centroid to the origin, mean distance sqrt(2).
/// @param points The points (N x 2).
/// @return `T`, or nothing if every point coincides.
[[nodiscard]] std::optional<Eigen::Matrix3d> hartley_normalisation(
    const Eigen::Ref<const Rows2>& points);

/// @brief Solve `A v = 0` for a unit `v` by SVD, if the solution is unique.
/// @param system `A` (M x 9), M >= 8.
/// @return The right singular vector of the smallest singular value, or nothing if the
///         second-smallest is below `kRankTolerance` of the largest.
[[nodiscard]] std::optional<Eigen::Matrix<double, 9, 1>> null_vector(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 9>>& system);

/// @brief Unit bearings must have norm 1 within this (`slamlab.foundations.arrays`' value).
inline constexpr double kBearingTolerance = 1e-9;

/// @brief Check one bearing: finite, with unit norm within `kBearingTolerance`.
/// @param bearings The bearings.
/// @param row The row to check.
/// @param name The argument's name, for the message.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_bearing(const Eigen::Ref<const Rows3>& bearings, Eigen::Index row,
                                         const char* name);

/// @brief Check bearing pairs: equal sizes, at least `minimum`, unit vectors, and (by default)
///        positive z, which methods working in the normalised image plane need.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @param minimum The fewest pairs accepted.
/// @param positive_z Require positive z.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_bearing_pairs(const Eigen::Ref<const Rows3>& bearings1,
                                               const Eigen::Ref<const Rows3>& bearings2,
                                               Eigen::Index minimum, bool positive_z = true);

/// @brief Check world points and their bearings (`slamlab.foundations.arrays.
///        as_correspondences`): equal sizes, at least `minimum`, finite points, unit bearings,
///        and (by default) positive z.
/// @param points World points.
/// @param bearings Their bearings in the camera.
/// @param minimum The fewest correspondences accepted.
/// @param positive_z Require positive z.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_correspondences(const Eigen::Ref<const Rows3>& points,
                                                 const Eigen::Ref<const Rows3>& bearings,
                                                 Eigen::Index minimum, bool positive_z = true);

/// @brief Check point pairs: equal sizes, at least `minimum`, finite.
/// @param points1 Points in image 1.
/// @param points2 Points in image 2.
/// @param minimum The fewest pairs accepted.
/// @param name The arguments' name, for messages.
/// @return Nothing, or `InvalidInput`.
[[nodiscard]] Result<void> check_point_pairs(const Eigen::Ref<const Rows2>& points1,
                                             const Eigen::Ref<const Rows2>& points2,
                                             Eigen::Index minimum, const char* name);

/// @brief Gather the rows of a matrix at the given indices.
/// @tparam M The row-major matrix type.
/// @param rows The matrix.
/// @param indices The rows to take.
/// @return The gathered rows.
template <typename M>
[[nodiscard]] M gather(const M& rows, std::span<const std::uint32_t> indices) {
  M out(static_cast<Eigen::Index>(indices.size()), rows.cols());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    out.row(static_cast<Eigen::Index>(i)) = rows.row(indices[i]);
  }
  return out;
}

}  // namespace slamlab::foundations::correspondences
