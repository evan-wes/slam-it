/// @file
/// @brief Native tests for slam_it_all::foundations::so3, runnable without Python.
///
/// The parity fixtures (tests/fixtures/foundations/) are the main check of every value;
/// these tests cover what the standalone C++ core must get right on its own: exact `hat`
/// entries, round trips, the θ = π convention, and the error paths.
#include <cmath>
#include <limits>
#include <numbers>

#include "slam_it_all/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

using slam_it_all::testing::expect;
namespace so3 = slam_it_all::foundations::so3;
using slam_it_all::foundations::ErrorKind;

}  // namespace

int main() {
  int failures = 0;

  // hat: the basis vectors fix all nine entries; it is exact.
  Eigen::Matrix3d hx;
  hx << 0, 0, 0, 0, 0, -1, 0, 1, 0;
  expect(so3::hat(Eigen::Vector3d::UnitX()) == hx, "hat(e_x)", failures);

  // exp/log round trip across angles, including both sides of the Taylor switch.
  const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
  for (const double theta : {0.0, 1e-12, 1e-4 * (1 - 1e-9), 1e-4 * (1 + 1e-9), 0.5, 3.0}) {
    const Eigen::Vector3d omega = theta * axis;
    const auto back = so3::log(so3::exp(omega));
    expect(back.has_value() && (*back - omega).cwiseAbs().maxCoeff() < 1e-14,
           "log(exp(omega)) == omega", failures);
  }

  // θ = π exactly: the largest-magnitude component of the result is positive.
  const Eigen::Matrix3d r_pi = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  const auto at_pi = so3::log(r_pi);
  expect(at_pi.has_value() && std::abs((*at_pi)(0) - std::numbers::pi) < 1e-15,
         "log(diag(1,-1,-1)) == (pi, 0, 0)", failures);

  // Jr⁻¹ · Jr = I.
  const Eigen::Vector3d w(0.4, -1.1, 0.7);
  const auto jr_inv = so3::right_jacobian_inverse(w);
  expect(
      jr_inv.has_value() &&
          ((*jr_inv * so3::right_jacobian(w)) - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <
              1e-14,
      "Jr^-1 Jr == I", failures);

  // Error paths return InvalidInput rather than garbage.
  expect(so3::log(Eigen::Matrix3d::Constant(0.5)).error().kind == ErrorKind::kInvalidInput,
         "log rejects a non-rotation", failures);
  expect(so3::log(-Eigen::Matrix3d::Identity()).error().kind == ErrorKind::kInvalidInput,
         "log rejects a reflection", failures);
  expect(so3::vee(Eigen::Matrix3d::Identity()).error().kind == ErrorKind::kInvalidInput,
         "vee rejects a non-skew matrix", failures);
  expect(so3::from_quaternion(Eigen::Vector4d(1, 1, 0, 0)).error().kind == ErrorKind::kInvalidInput,
         "from_quaternion rejects a non-unit quaternion", failures);
  expect(so3::right_jacobian_inverse(Eigen::Vector3d(7.0, 0, 0)).error().kind ==
             ErrorKind::kInvalidInput,
         "inverse Jacobian rejects |omega| >= 2 pi", failures);

  // NaN passes every tolerance comparison, so finiteness is checked first.
  Eigen::Matrix3d nan = Eigen::Matrix3d::Identity();
  nan(0, 1) = std::numeric_limits<double>::quiet_NaN();
  expect(so3::log(nan).error().kind == ErrorKind::kInvalidInput, "log rejects a NaN entry",
         failures);
  expect(so3::vee(nan).error().kind == ErrorKind::kInvalidInput, "vee rejects a NaN entry",
         failures);

  return slam_it_all::testing::finish("foundations/so3", failures);
}
