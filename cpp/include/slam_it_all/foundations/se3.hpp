/// @file
/// @brief SE(3), rigid-body transforms: the C++ core's implementation.
///
/// Mirrors `slam_it_all.foundations.se3`. A transform is a 4x4 matrix `[[R, t], [0, 1]]`; the
/// tangent is `ξ = (ρ, θ)`, translation first; perturbations are right (local). The
/// Jacobians use Barfoot's block form with `Q(ρ, θ)`, whose coefficients come from their
/// power series below `kSeriesThreshold` (design note `docs/notes/lie-groups.md`).
#pragma once

#include <Eigen/Core>

#include "slam_it_all/foundations/error.hpp"

/// @brief SE(3), rigid-body transforms.
namespace slam_it_all::foundations::se3 {

/// @brief A twist `(ρ, θ)`.
using Vector6d = Eigen::Matrix<double, 6, 1>;
/// @brief A 6x6 matrix (adjoints, Jacobians).
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// @brief Below this rotation angle, `Q`'s coefficients come from their power series.
inline constexpr double kSeriesThreshold = 1.0;
/// @brief Terms summed for each series.
inline constexpr int kSeriesTerms = 10;
/// @brief A transform's last row must equal `[0, 0, 0, 1]` within this.
inline constexpr double kBottomRowTolerance = 1e-12;

/// @brief Check that a matrix is a rigid transform.
/// @param transform The matrix to check.
/// @return Nothing on success, or `InvalidInput` naming what failed.
[[nodiscard]] Result<void> check_transform(const Eigen::Matrix4d& transform);

/// @brief Map a twist to a rigid transform: `[[exp(θ), Jl(θ) ρ], [0, 1]]`.
/// @param xi The twist `(ρ, θ)`.
/// @return The transform.
[[nodiscard]] Eigen::Matrix4d exp(const Vector6d& xi) noexcept;

/// @brief Map a rigid transform to its twist, rotation angle in [0, π].
/// @param transform A rigid transform.
/// @return The twist, or `InvalidInput` if `transform` is not a rigid transform.
[[nodiscard]] Result<Vector6d> log(const Eigen::Matrix4d& transform);

/// @brief Invert a rigid transform: `[[Rᵀ, -Rᵀ t], [0, 1]]`.
/// @param transform A rigid transform.
/// @return The inverse, or `InvalidInput` if `transform` is not a rigid transform.
[[nodiscard]] Result<Eigen::Matrix4d> inverse(const Eigen::Matrix4d& transform);

/// @brief Compose rigid transforms, `a · b`, keeping the last row exact.
/// @param a The left transform.
/// @param b The right transform.
/// @return The product, or `InvalidInput` if either is not a rigid transform.
[[nodiscard]] Result<Eigen::Matrix4d> compose(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b);

/// @brief Apply a rigid transform to a point: `R p + t`.
/// @param transform A rigid transform.
/// @param point The point.
/// @return The transformed point, or `InvalidInput` if `transform` is not rigid.
[[nodiscard]] Result<Eigen::Vector3d> act(const Eigen::Matrix4d& transform,
                                          const Eigen::Vector3d& point);

/// @brief The adjoint `[[R, hat(t) R], [0, R]]`, with `T exp(ξ) T⁻¹ = exp(Ad_T ξ)`.
/// @param transform A rigid transform.
/// @return The adjoint, or `InvalidInput` if `transform` is not rigid.
[[nodiscard]] Result<Matrix6d> adjoint(const Eigen::Matrix4d& transform);

/// @brief The left Jacobian `[[Jl(θ), Q(ρ, θ)], [0, Jl(θ)]]`.
/// @param xi The twist.
/// @return The left Jacobian.
[[nodiscard]] Matrix6d left_jacobian(const Vector6d& xi) noexcept;

/// @brief The right Jacobian `Jr(ξ) = Jl(-ξ)`.
/// @param xi The twist.
/// @return The right Jacobian.
[[nodiscard]] Matrix6d right_jacobian(const Vector6d& xi) noexcept;

/// @brief The inverse left Jacobian `[[J⁻¹, -J⁻¹ Q J⁻¹], [0, J⁻¹]]`, for `|θ| < 2π`.
/// @param xi The twist.
/// @return The inverse, or `InvalidInput` if `|θ| >= 2π`.
[[nodiscard]] Result<Matrix6d> left_jacobian_inverse(const Vector6d& xi);

/// @brief The inverse right Jacobian `Jl(-ξ)⁻¹`, for `|θ| < 2π`.
/// @param xi The twist.
/// @return The inverse, or `InvalidInput` if `|θ| >= 2π`.
[[nodiscard]] Result<Matrix6d> right_jacobian_inverse(const Vector6d& xi);

}  // namespace slam_it_all::foundations::se3
