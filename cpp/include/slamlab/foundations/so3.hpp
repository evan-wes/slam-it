/// @file
/// @brief SO(3), the group of 3D rotations: the C++ core's implementation.
///
/// Mirrors `slamlab.foundations.so3`, the Python reference, which the parity tests hold this
/// implementation to. Conventions (design doc, "Conventions"): right (local) perturbations,
/// `R ⊕ δ = R · exp(δ)`; Hamilton quaternions stored `(x, y, z, w)`; `log` returns angles in
/// [0, π]. The thresholds below are the same constants as the Python module's.
#pragma once

#include <Eigen/Core>
#include <numbers>

#include "slamlab/foundations/error.hpp"

/// @brief SO(3), the group of 3D rotations.
namespace slamlab::foundations::so3 {

/// @brief Below this angle, `sin θ / θ` and `θ / sin θ` use their series.
inline constexpr double kExpTaylorThreshold = 1e-4;
/// @brief Below this angle, the Jacobians' coefficients use their series.
inline constexpr double kJacobianTaylorThreshold = 0.05;
/// @brief Near π (with `cos θ < 0`), `log` takes the axis from the symmetric part below this.
inline constexpr double kNearPiSine = 1e-3;
/// @brief Accepted deviation of `RᵀR` from the identity, element-wise.
inline constexpr double kRotationTolerance = 1e-6;
/// @brief Accepted asymmetry `|W + Wᵀ|`, relative to `max(1, |W|)`.
inline constexpr double kSkewTolerance = 1e-9;
/// @brief Accepted deviation of a quaternion's norm from 1.
inline constexpr double kQuaternionTolerance = 1e-6;
/// @brief The inverse Jacobians are singular at `|ω| = 2π`; larger angles are rejected.
inline constexpr double kInverseJacobianLimit = (2.0 * std::numbers::pi) - 1e-6;

/// @brief Map a vector to its skew-symmetric matrix, so that `hat(ω) u == ω × u`.
/// @param omega A vector in R^3, the tangent space of SO(3).
/// @return The skew-symmetric matrix `[ω]x`.
[[nodiscard]] Eigen::Matrix3d hat(const Eigen::Vector3d& omega) noexcept;

/// @brief Map a skew-symmetric matrix back to its vector; the inverse of `hat`.
/// @param skew A skew-symmetric matrix (within `kSkewTolerance`).
/// @return The vector `ω` with `hat(ω) == skew`, or `InvalidInput` if `skew` is not
///         skew-symmetric (or has a non-finite entry).
[[nodiscard]] Result<Eigen::Vector3d> vee(const Eigen::Matrix3d& skew);

/// @brief Map a rotation vector to a rotation matrix (Rodrigues' formula).
/// @param omega A rotation vector: axis times angle in radians (any angle).
/// @return The rotation matrix `exp(hat(ω))`.
[[nodiscard]] Eigen::Matrix3d exp(const Eigen::Vector3d& omega) noexcept;

/// @brief Map a rotation matrix to its rotation vector, with angle in [0, π].
///
/// At exactly θ = π the axis is ambiguous up to sign; the vector whose largest-magnitude
/// component is positive is returned, as in every backend.
/// @param rotation A rotation matrix (orthonormal within `kRotationTolerance`, det > 0).
/// @return The rotation vector, or `InvalidInput` if `rotation` is not a rotation.
[[nodiscard]] Result<Eigen::Vector3d> log(const Eigen::Matrix3d& rotation);

/// @brief The right Jacobian `Jr(ω)`, with `exp(ω + δ) ≈ exp(ω) exp(Jr(ω) δ)`.
/// @param omega A rotation vector.
/// @return `I - (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²`, with `W = hat(ω)`.
[[nodiscard]] Eigen::Matrix3d right_jacobian(const Eigen::Vector3d& omega) noexcept;

/// @brief The left Jacobian `Jl(ω) = Jr(-ω)`.
/// @param omega A rotation vector.
/// @return `I + (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²`.
[[nodiscard]] Eigen::Matrix3d left_jacobian(const Eigen::Vector3d& omega) noexcept;

/// @brief The inverse of the right Jacobian, defined for `|ω| < 2π`.
/// @param omega A rotation vector with norm below `kInverseJacobianLimit`.
/// @return `I + ½ W + (1/θ² - cot(θ/2)/(2θ)) W²`, or `InvalidInput` if `|ω| >= 2π`.
[[nodiscard]] Result<Eigen::Matrix3d> right_jacobian_inverse(const Eigen::Vector3d& omega);

/// @brief The inverse of the left Jacobian, defined for `|ω| < 2π`.
/// @param omega A rotation vector with norm below `kInverseJacobianLimit`.
/// @return `I - ½ W + (1/θ² - cot(θ/2)/(2θ)) W²`, or `InvalidInput` if `|ω| >= 2π`.
[[nodiscard]] Result<Eigen::Matrix3d> left_jacobian_inverse(const Eigen::Vector3d& omega);

/// @brief Map a unit quaternion (Hamilton, stored xyzw) to a rotation matrix.
/// @param quaternion `(x, y, z, w)` with norm within `kQuaternionTolerance` of 1; it is
///                   normalised before use.
/// @return The rotation matrix, or `InvalidInput` if the norm is not close to 1.
[[nodiscard]] Result<Eigen::Matrix3d> from_quaternion(const Eigen::Vector4d& quaternion);

/// @brief Map a rotation matrix to a unit quaternion (xyzw), canonically `w >= 0`.
/// @param rotation A rotation matrix (orthonormal within `kRotationTolerance`, det > 0).
/// @return `(x, y, z, w)` with `w >= 0`, or `InvalidInput` if `rotation` is not a rotation.
[[nodiscard]] Result<Eigen::Vector4d> to_quaternion(const Eigen::Matrix3d& rotation);

/// @brief The nearest rotation to a matrix in the Frobenius norm (via an SVD).
/// @param matrix Any real 3x3 matrix.
/// @return `U diag(1, 1, det(U Vᵀ)) Vᵀ` for `matrix = U S Vᵀ`.
[[nodiscard]] Eigen::Matrix3d project_to_so3(const Eigen::Matrix3d& matrix);

/// @brief Check that a matrix is a rotation within `kRotationTolerance`.
/// @param rotation The matrix to check.
/// @return Nothing on success, or `InvalidInput` naming what failed.
[[nodiscard]] Result<void> check_rotation(const Eigen::Matrix3d& rotation);

}  // namespace slamlab::foundations::so3
