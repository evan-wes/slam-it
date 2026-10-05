/// @file
/// @brief The 5-point essential matrix: the C++ core's Nistér and Stewénius solvers.
///
/// Mirrors `slamlab.foundations.fivepoint`, whose docstring states the maths: the null space
/// of five bearing pairs' epipolar equations, Demazure's ten cubic constraints, then either
/// Nistér's degree-10 polynomial (real roots by Sturm sequences and bisection) or Stewénius'
/// 10x10 action matrix (real eigenvectors), each solution then polished by Gauss-Newton on the
/// constraints. Every real solution that is essential is returned, scaled to singular values
/// (1, 1, 0) (a sample with no baseline gives arbitrary ones, some not essential).
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "slamlab/foundations/correspondences.hpp"
#include "slamlab/foundations/error.hpp"

/// @brief The 5-point essential matrix.
namespace slamlab::foundations::fivepoint {

/// @brief Correspondences a 5-point solver takes.
inline constexpr std::uint32_t kSampleSize = 5;
/// @brief At most this many Gauss-Newton steps polish each solution on the ten cubics.
inline constexpr int kPolishMaxSteps = 10;
/// @brief Polishing stops when a step is below this, relative to the solution (at least 1).
inline constexpr double kPolishTolerance = 1e-14;
/// @brief An eigenvalue counts as real when its imaginary part is below this, relative to its
///        modulus (at least 1).
inline constexpr double kRealTolerance = 1e-10;
/// @brief Bisection stops when a root's bracket is below this, relative to the root (at least
///        1).
inline constexpr double kRootTolerance = 1e-14;
/// @brief A solution is kept only if, scaled to norm sqrt(2), `s1 - s2` and `s3` are both below
///        this (see `slamlab.foundations.fivepoint.ESSENTIAL_TOLERANCE`).
inline constexpr double kEssentialTolerance = 1e-6;
/// @brief A solution within this (max-abs, up to sign) of one already kept is the same solution
///        (see `slamlab.foundations.fivepoint.DUPLICATE_TOLERANCE`).
inline constexpr double kDuplicateTolerance = 1e-8;

/// @brief The solvers, with the codes they cross the bindings as.
enum class Solver : std::uint8_t {
  /// @brief Nistér's polynomial and Sturm sequence.
  kNister = 0,
  /// @brief Stewénius' action matrix.
  kStewenius = 1,
  /// @brief The number of solvers (a sentinel).
  kCount = 2,
};

/// @brief The solver's name, as `slamlab.foundations.fivepoint.SOLVERS` spells it.
/// @param solver The solver.
/// @return A static string.
[[nodiscard]] const char* name(Solver solver);

/// @brief A polynomial's distinct real roots, ascending, by Sturm sequences and bisection.
/// @param poly Coefficients, ascending (`poly[k]` multiplies `z^k`).
/// @return The real roots.
[[nodiscard]] std::vector<double> real_roots(std::vector<double> poly);

/// @brief Every essential matrix five bearing pairs allow (unchecked).
/// @param bearings1 Five unit bearings in camera 1.
/// @param bearings2 Five unit bearings in camera 2.
/// @param solver The solver.
/// @return The real solutions (none for a degenerate sample), in canonical order: each
///         signed so its first largest-magnitude entry, row by row, is positive, then sorted
///         lexicographically by its row-major entries.
[[nodiscard]] std::vector<Eigen::Matrix3d> solve(
    const Eigen::Ref<const correspondences::Rows3>& bearings1,
    const Eigen::Ref<const correspondences::Rows3>& bearings2, Solver solver);

/// @brief Every essential matrix five bearing pairs allow.
/// @param bearings1 Five unit bearings in camera 1.
/// @param bearings2 Five unit bearings in camera 2.
/// @param solver The solver.
/// @return The solutions, in canonical order (see `solve`); `InvalidInput` unless there are
///         five unit pairs.
[[nodiscard]] Result<std::vector<Eigen::Matrix3d>> essential_5pt(
    const Eigen::Ref<const correspondences::Rows3>& bearings1,
    const Eigen::Ref<const correspondences::Rows3>& bearings2, Solver solver);

}  // namespace slamlab::foundations::fivepoint
