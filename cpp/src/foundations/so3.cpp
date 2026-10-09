/// @file
/// @brief Implementation of the SO(3) functions declared in slam_it_all/foundations/so3.hpp.
///
/// Each formula mirrors the Python reference (`slam_it_all.foundations.so3`) term for term, so
/// the two agree to rounding: cancellation-free forms where they exist, Taylor series below
/// the named thresholds, `atan2` for the angle in `log`, and the symmetric-part axis near π.
#include "slam_it_all/foundations/so3.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>

namespace slam_it_all::foundations::so3 {

namespace {

/// @brief `sin θ / θ`, with its series below `kExpTaylorThreshold`.
/// @param theta The angle, `>= 0`.
/// @return The coefficient.
double sinc(double theta) noexcept {
  if (theta < kExpTaylorThreshold) {
    return 1.0 - (theta * theta / 6.0);
  }
  return std::sin(theta) / theta;
}

/// @brief `(1 - cos θ) / θ²` as `½ (sin(θ/2) / (θ/2))²`: no cancellation at small θ.
/// @param theta The angle, `>= 0`.
/// @return The coefficient.
double half_versine_coefficient(double theta) noexcept {
  const double s = sinc(0.5 * theta);
  return 0.5 * s * s;
}

/// @brief `(θ - sin θ) / θ³`, with its series below `kJacobianTaylorThreshold`.
/// @param theta The angle, `>= 0`.
/// @return The coefficient.
double third_coefficient(double theta) noexcept {
  if (theta < kJacobianTaylorThreshold) {
    const double t2 = theta * theta;
    return (1.0 / 6.0) - (t2 / 120.0) + (t2 * t2 / 5040.0) - (t2 * t2 * t2 / 362880.0);
  }
  return (theta - std::sin(theta)) / (theta * theta * theta);
}

/// @brief `1/θ² - cot(θ/2) / (2θ)`, with its series below `kJacobianTaylorThreshold`.
/// @param theta The angle, in `[0, 2π)`.
/// @return The coefficient.
double inverse_coefficient(double theta) noexcept {
  if (theta < kJacobianTaylorThreshold) {
    const double t2 = theta * theta;
    return (1.0 / 12.0) + (t2 / 720.0) + (t2 * t2 / 30240.0) + (t2 * t2 * t2 / 1209600.0);
  }
  return (1.0 / (theta * theta)) - (1.0 / (2.0 * theta * std::tan(0.5 * theta)));
}

/// @brief `I + sign · B W + C W²`: the left (sign +1) or right (sign -1) Jacobian.
/// @param omega A rotation vector.
/// @param sign +1 for the left Jacobian, -1 for the right.
/// @return The Jacobian.
Eigen::Matrix3d jacobian(const Eigen::Vector3d& omega, double sign) noexcept {
  const double theta = omega.norm();
  const Eigen::Matrix3d w = hat(omega);
  return Eigen::Matrix3d::Identity() + (sign * half_versine_coefficient(theta) * w) +
         (third_coefficient(theta) * w * w);
}

/// @brief `I - sign · ½ W + D W²`: the left (sign +1) or right (sign -1) inverse Jacobian.
/// @param omega A rotation vector with `|ω| < kInverseJacobianLimit`.
/// @param sign +1 for the left inverse Jacobian, -1 for the right.
/// @return The inverse Jacobian, or `InvalidInput` beyond the limit.
Result<Eigen::Matrix3d> jacobian_inverse(const Eigen::Vector3d& omega, double sign) {
  const double theta = omega.norm();
  if (theta >= kInverseJacobianLimit) {
    return fail(ErrorKind::kInvalidInput, "omega: inverse Jacobian undefined at |omega| >= 2*pi");
  }
  const Eigen::Matrix3d w = hat(omega);
  return Eigen::Matrix3d(Eigen::Matrix3d::Identity() - (sign * 0.5 * w) +
                         (inverse_coefficient(theta) * w * w));
}

/// @brief The rotation axis near θ = π, from `(R + Rᵀ)/2 - cos θ · I = (1 - cos θ) n nᵀ`.
///
/// The sign follows `v = sin θ · n`; when that vanishes the largest-magnitude component is
/// made positive, matching the Python reference.
/// @param rotation The rotation.
/// @param cosine `cos θ`, clamped to [-1, 1].
/// @param v The skew part `sin θ · n`.
/// @return The unit axis.
Eigen::Vector3d axis_near_pi(const Eigen::Matrix3d& rotation, double cosine,
                             const Eigen::Vector3d& v) noexcept {
  const Eigen::Matrix3d outer =
      (0.5 * (rotation + rotation.transpose())) - (cosine * Eigen::Matrix3d::Identity());
  Eigen::Index k = 0;
  outer.diagonal().maxCoeff(&k);
  Eigen::Vector3d axis = outer.col(k) / std::sqrt(outer(k, k) * (1.0 - cosine));
  axis.normalize();
  const double alignment = axis.dot(v);
  Eigen::Index largest = 0;
  axis.cwiseAbs().maxCoeff(&largest);
  if (alignment < 0.0 || (alignment == 0.0 && axis(largest) < 0.0)) {
    axis = -axis;
  }
  return axis;
}

}  // namespace

Eigen::Matrix3d hat(const Eigen::Vector3d& omega) noexcept {
  Eigen::Matrix3d w;
  w << 0.0, -omega.z(), omega.y(), omega.z(), 0.0, -omega.x(), -omega.y(), omega.x(), 0.0;
  return w;
}

Result<Eigen::Vector3d> vee(const Eigen::Matrix3d& skew) {
  if (!skew.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "skew: expected finite values");
  }
  const double scale = std::max(1.0, skew.cwiseAbs().maxCoeff());
  if ((skew + skew.transpose()).cwiseAbs().maxCoeff() > kSkewTolerance * scale) {
    return fail(ErrorKind::kInvalidInput, "skew: not skew-symmetric within 1e-09");
  }
  return Eigen::Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
}

Eigen::Matrix3d exp(const Eigen::Vector3d& omega) noexcept {
  const double theta = omega.norm();
  const Eigen::Matrix3d w = hat(omega);
  return Eigen::Matrix3d::Identity() + (sinc(theta) * w) +
         (half_versine_coefficient(theta) * w * w);
}

Result<void> check_rotation(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "rotation: expected finite values");
  }
  const Eigen::Matrix3d gram = rotation.transpose() * rotation;
  if ((gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > kRotationTolerance) {
    return fail(ErrorKind::kInvalidInput,
                "rotation: not orthonormal within 1e-06; use project_to_so3 to repair it "
                "explicitly");
  }
  if (rotation.determinant() <= 0.0) {
    return fail(ErrorKind::kInvalidInput, "rotation: determinant is not positive (a reflection)");
  }
  return {};
}

Result<Eigen::Vector3d> log(const Eigen::Matrix3d& rotation) {
  if (auto checked = check_rotation(rotation); !checked) {
    return std::unexpected(checked.error());
  }
  const Eigen::Vector3d v(0.5 * (rotation(2, 1) - rotation(1, 2)),
                          0.5 * (rotation(0, 2) - rotation(2, 0)),
                          0.5 * (rotation(1, 0) - rotation(0, 1)));
  const double sine = v.norm();
  const double cosine = std::clamp(0.5 * (rotation.trace() - 1.0), -1.0, 1.0);
  const double theta = std::atan2(sine, cosine);
  if (cosine < 0.0 && sine < kNearPiSine) {
    return Eigen::Vector3d(theta * axis_near_pi(rotation, cosine, v));
  }
  const double factor = theta < kExpTaylorThreshold ? 1.0 + (theta * theta / 6.0) : theta / sine;
  return Eigen::Vector3d(factor * v);
}

Eigen::Matrix3d right_jacobian(const Eigen::Vector3d& omega) noexcept {
  return jacobian(omega, -1.0);
}

Eigen::Matrix3d left_jacobian(const Eigen::Vector3d& omega) noexcept {
  return jacobian(omega, 1.0);
}

Result<Eigen::Matrix3d> right_jacobian_inverse(const Eigen::Vector3d& omega) {
  return jacobian_inverse(omega, -1.0);
}

Result<Eigen::Matrix3d> left_jacobian_inverse(const Eigen::Vector3d& omega) {
  return jacobian_inverse(omega, 1.0);
}

Result<Eigen::Matrix3d> from_quaternion(const Eigen::Vector4d& quaternion) {
  if (!quaternion.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "quaternion: expected finite values");
  }
  const double norm = quaternion.norm();
  if (std::abs(norm - 1.0) > kQuaternionTolerance) {
    return fail(ErrorKind::kInvalidInput, "quaternion: norm not within 1e-06 of 1");
  }
  const Eigen::Vector4d q = quaternion / norm;
  const double x = q(0);
  const double y = q(1);
  const double z = q(2);
  const double w = q(3);
  Eigen::Matrix3d r;
  r << 1.0 - (2.0 * ((y * y) + (z * z))), 2.0 * ((x * y) - (w * z)), 2.0 * ((x * z) + (w * y)),
      2.0 * ((x * y) + (w * z)), 1.0 - (2.0 * ((x * x) + (z * z))), 2.0 * ((y * z) - (w * x)),
      2.0 * ((x * z) - (w * y)), 2.0 * ((y * z) + (w * x)), 1.0 - (2.0 * ((x * x) + (y * y)));
  return r;
}

Result<Eigen::Vector4d> to_quaternion(const Eigen::Matrix3d& rotation) {
  if (auto checked = check_rotation(rotation); !checked) {
    return std::unexpected(checked.error());
  }
  const Eigen::Matrix3d& r = rotation;
  const double trace = r.trace();
  const Eigen::Vector4d candidates(trace, r(0, 0), r(1, 1), r(2, 2));
  Eigen::Index k = 0;
  candidates.maxCoeff(&k);
  Eigen::Vector4d q;
  double s = 0.0;
  if (k == 0) {
    s = 2.0 * std::sqrt(1.0 + trace);
    q << r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1), 0.25 * s * s;
  } else if (k == 1) {
    s = 2.0 * std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2));
    q << 0.25 * s * s, r(0, 1) + r(1, 0), r(0, 2) + r(2, 0), r(2, 1) - r(1, 2);
  } else if (k == 2) {
    s = 2.0 * std::sqrt(1.0 - r(0, 0) + r(1, 1) - r(2, 2));
    q << r(0, 1) + r(1, 0), 0.25 * s * s, r(1, 2) + r(2, 1), r(0, 2) - r(2, 0);
  } else {
    s = 2.0 * std::sqrt(1.0 - r(0, 0) - r(1, 1) + r(2, 2));
    q << r(0, 2) + r(2, 0), r(1, 2) + r(2, 1), 0.25 * s * s, r(1, 0) - r(0, 1);
  }
  q /= s;
  if (q(3) < 0.0) {
    q = -q;
  }
  return q;
}

Eigen::Matrix3d project_to_so3(const Eigen::Matrix3d& matrix) {
  const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(matrix);
  Eigen::Matrix3d u = svd.matrixU();
  const Eigen::Matrix3d& v = svd.matrixV();
  if ((u * v.transpose()).determinant() < 0.0) {
    u.col(2) = -u.col(2);
  }
  return u * v.transpose();
}

}  // namespace slam_it_all::foundations::so3
