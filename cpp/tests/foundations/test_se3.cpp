/// @file
/// @brief Native tests for slamlab::foundations::se3 and ::alignment, runnable without Python.
///
/// The parity fixtures are the main check of every value; these cover what the standalone
/// core must get right alone: round trips, inverses, the Jacobian inverse, Umeyama recovery,
/// and the error paths.
#include <Eigen/Dense>

#include "slamlab/foundations/alignment.hpp"
#include "slamlab/foundations/se3.hpp"
#include "test_support.hpp"

namespace {

using slamlab::testing::expect;
namespace se3 = slamlab::foundations::se3;
namespace alignment = slamlab::foundations::alignment;
using slamlab::foundations::ErrorKind;

}  // namespace

int main() {
  int failures = 0;
  se3::Vector6d xi;
  xi << 0.4, -1.3, 2.2, 0.9, -0.2, 1.7;

  const Eigen::Matrix4d t = se3::exp(xi);
  const auto back = se3::log(t);
  expect(back.has_value() && (*back - xi).cwiseAbs().maxCoeff() < 1e-13, "log(exp(xi)) == xi",
         failures);

  const auto inv = se3::inverse(t);
  const auto product = inv.has_value() ? se3::compose(t, *inv) : inv;
  expect(
      product.has_value() && (*product - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff() < 1e-14,
      "T T^-1 == I", failures);

  const auto jr_inv = se3::right_jacobian_inverse(xi);
  expect(
      jr_inv.has_value() &&
          ((*jr_inv * se3::right_jacobian(xi)) - se3::Matrix6d::Identity()).cwiseAbs().maxCoeff() <
              1e-12,
      "Jr^-1 Jr == I", failures);

  // Umeyama recovers an exact similarity transform.
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> source;
  source << 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0.5, -1, 2, -2, 0.3, 1;
  Eigen::Matrix<double, 6, 3, Eigen::RowMajor> target;
  const double scale = 1.8;
  for (int i = 0; i < 6; ++i) {
    target.row(i) =
        ((scale * t.topLeftCorner<3, 3>() * source.row(i).transpose()) + t.topRightCorner<3, 1>())
            .transpose();
  }
  const auto aligned = alignment::umeyama(source, target, Eigen::VectorXd::Ones(6), true);
  expect(aligned.has_value() && (aligned->topLeftCorner<3, 3>() - (scale * t.topLeftCorner<3, 3>()))
                                        .cwiseAbs()
                                        .maxCoeff() < 1e-13,
         "umeyama recovers s R", failures);

  // Error paths.
  expect(se3::log(Eigen::Matrix4d::Zero()).error().kind == ErrorKind::kInvalidInput,
         "log rejects a non-transform", failures);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> line;
  line << 0, 0, 0, 1, 2, 3, 2, 4, 6;
  expect(alignment::umeyama(line, line, Eigen::VectorXd::Ones(3), false).error().kind ==
             ErrorKind::kDegenerate,
         "umeyama reports collinear points as degenerate", failures);

  return slamlab::testing::finish("foundations/se3", failures);
}
