/// @file
/// @brief Implementation of the SE(3) functions declared in slamlab/foundations/se3.hpp.
#include "slamlab/foundations/se3.hpp"

#include <array>
#include <cmath>

#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::se3 {

namespace {

/// @brief The three coefficients of `Q`: series below the threshold, closed form above.
/// @param theta The rotation angle, `>= 0`.
/// @return `(θ - sin θ)/θ³`, `(θ² + 2 cos θ - 2)/(2θ⁴)`, `(2θ - 3 sin θ + θ cos θ)/(2θ⁵)`.
std::array<double, 3> q_coefficients(double theta) noexcept {
  if (theta < kSeriesThreshold) {
    const double t2 = theta * theta;
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    double power = 1.0;
    // Each coefficient is Σ_k (-1)^k θ^{2k} c_k with c_k = 1/(2k+3)!, 1/(2k+4)!,
    // (k+1)/(2k+5)!; the factorials are built incrementally.
    double f3 = 6.0;    // (2k+3)!
    double f4 = 24.0;   // (2k+4)!
    double f5 = 120.0;  // (2k+5)!
    for (int k = 0; k < kSeriesTerms; ++k) {
      const double sign = (k % 2 == 0) ? 1.0 : -1.0;
      sum[0] += sign * power / f3;
      sum[1] += sign * power / f4;
      sum[2] += sign * power * (k + 1) / f5;
      power *= t2;
      const double kk = k;
      f3 *= ((2.0 * kk) + 4.0) * ((2.0 * kk) + 5.0);
      f4 *= ((2.0 * kk) + 5.0) * ((2.0 * kk) + 6.0);
      f5 *= ((2.0 * kk) + 6.0) * ((2.0 * kk) + 7.0);
    }
    return sum;
  }
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  return {(theta - s) / (theta * theta * theta),
          ((theta * theta) + (2.0 * c) - 2.0) / (2.0 * std::pow(theta, 4)),
          ((2.0 * theta) - (3.0 * s) + (theta * c)) / (2.0 * std::pow(theta, 5))};
}

/// @brief Barfoot's `Q(ρ, θ)`, the coupling block of the left Jacobian.
/// @param xi The twist.
/// @return `Q`.
Eigen::Matrix3d q_matrix(const Vector6d& xi) noexcept {
  const Eigen::Matrix3d r = so3::hat(xi.head<3>());
  const Eigen::Matrix3d p = so3::hat(xi.tail<3>());
  const auto [c1, c2, c3] = q_coefficients(xi.tail<3>().norm());
  const Eigen::Matrix3d pr = p * r;
  const Eigen::Matrix3d rp = r * p;
  const Eigen::Matrix3d prp = p * r * p;
  return (0.5 * r) + (c1 * (pr + rp + prp)) + (c2 * ((p * pr) + (rp * p) - (3.0 * prp))) +
         (c3 * ((prp * p) + (p * prp)));
}

}  // namespace

Result<void> check_transform(const Eigen::Matrix4d& transform) {
  const Eigen::RowVector4d bottom(0.0, 0.0, 0.0, 1.0);
  if ((transform.row(3) - bottom).cwiseAbs().maxCoeff() > kBottomRowTolerance) {
    return fail(ErrorKind::kInvalidInput, "transform: last row is not [0, 0, 0, 1]");
  }
  return so3::check_rotation(transform.topLeftCorner<3, 3>());
}

Eigen::Matrix4d exp(const Vector6d& xi) noexcept {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = so3::exp(xi.tail<3>());
  out.topRightCorner<3, 1>() = so3::left_jacobian(xi.tail<3>()) * xi.head<3>();
  return out;
}

Result<Vector6d> log(const Eigen::Matrix4d& transform) {
  if (auto checked = check_transform(transform); !checked) {
    return std::unexpected(checked.error());
  }
  const auto theta = so3::log(transform.topLeftCorner<3, 3>());
  if (!theta) {
    return std::unexpected(theta.error());
  }
  const auto jl_inv = so3::left_jacobian_inverse(*theta);
  if (!jl_inv) {
    return std::unexpected(jl_inv.error());
  }
  Vector6d xi;
  xi << *jl_inv * transform.topRightCorner<3, 1>(), *theta;
  return xi;
}

Result<Eigen::Matrix4d> inverse(const Eigen::Matrix4d& transform) {
  if (auto checked = check_transform(transform); !checked) {
    return std::unexpected(checked.error());
  }
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d rt = transform.topLeftCorner<3, 3>().transpose();
  out.topLeftCorner<3, 3>() = rt;
  out.topRightCorner<3, 1>() = -rt * transform.topRightCorner<3, 1>();
  return out;
}

Result<Eigen::Matrix4d> compose(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) {
  if (auto checked = check_transform(a); !checked) {
    return std::unexpected(checked.error());
  }
  if (auto checked = check_transform(b); !checked) {
    return std::unexpected(checked.error());
  }
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = a.topLeftCorner<3, 3>() * b.topLeftCorner<3, 3>();
  out.topRightCorner<3, 1>() =
      (a.topLeftCorner<3, 3>() * b.topRightCorner<3, 1>()) + a.topRightCorner<3, 1>();
  return out;
}

Result<Eigen::Vector3d> act(const Eigen::Matrix4d& transform, const Eigen::Vector3d& point) {
  if (auto checked = check_transform(transform); !checked) {
    return std::unexpected(checked.error());
  }
  return Eigen::Vector3d((transform.topLeftCorner<3, 3>() * point) +
                         transform.topRightCorner<3, 1>());
}

Result<Matrix6d> adjoint(const Eigen::Matrix4d& transform) {
  if (auto checked = check_transform(transform); !checked) {
    return std::unexpected(checked.error());
  }
  const Eigen::Matrix3d r = transform.topLeftCorner<3, 3>();
  Matrix6d out = Matrix6d::Zero();
  out.topLeftCorner<3, 3>() = r;
  out.bottomRightCorner<3, 3>() = r;
  out.topRightCorner<3, 3>() = so3::hat(transform.topRightCorner<3, 1>()) * r;
  return out;
}

Matrix6d left_jacobian(const Vector6d& xi) noexcept {
  const Eigen::Matrix3d jl = so3::left_jacobian(xi.tail<3>());
  Matrix6d out = Matrix6d::Zero();
  out.topLeftCorner<3, 3>() = jl;
  out.bottomRightCorner<3, 3>() = jl;
  out.topRightCorner<3, 3>() = q_matrix(xi);
  return out;
}

Matrix6d right_jacobian(const Vector6d& xi) noexcept { return left_jacobian(-xi); }

Result<Matrix6d> left_jacobian_inverse(const Vector6d& xi) {
  const auto jl_inv = so3::left_jacobian_inverse(xi.tail<3>());
  if (!jl_inv) {
    return std::unexpected(jl_inv.error());
  }
  Matrix6d out = Matrix6d::Zero();
  out.topLeftCorner<3, 3>() = *jl_inv;
  out.bottomRightCorner<3, 3>() = *jl_inv;
  out.topRightCorner<3, 3>() = -*jl_inv * q_matrix(xi) * *jl_inv;
  return out;
}

Result<Matrix6d> right_jacobian_inverse(const Vector6d& xi) { return left_jacobian_inverse(-xi); }

}  // namespace slamlab::foundations::se3
