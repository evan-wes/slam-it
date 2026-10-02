/// @file
/// @brief Bindings for `slamlab.foundations.se3` (`slamlab._cpp.foundations.se3`).
#include "slamlab/foundations/se3.hpp"

#include <nanobind/nanobind.h>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace se3 = foundations::se3;

}  // namespace

void bind_se3(nb::module_& m) {
  m.def(
      "exp",
      [](const BatchIn<6>& xi) {
        return map_batch<6, 1, 4, 4>(xi, [](const se3::Vector6d& x) { return se3::exp(x); });
      },
      nb::arg("xi").noconvert(), "Map (N, 6) twists (rho, theta) to (N, 4, 4) transforms.");
  m.def(
      "log",
      [](const BatchIn<4, 4>& transform) {
        return map_batch<4, 4, 6, 1>(transform,
                                     [](const Eigen::Matrix4d& t) { return se3::log(t); });
      },
      nb::arg("transform").noconvert(), "Map (N, 4, 4) transforms to (N, 6) twists.");
  m.def(
      "inverse",
      [](const BatchIn<4, 4>& transform) {
        return map_batch<4, 4, 4, 4>(transform,
                                     [](const Eigen::Matrix4d& t) { return se3::inverse(t); });
      },
      nb::arg("transform").noconvert(), "Invert (N, 4, 4) transforms.");
  m.def(
      "compose",
      [](const BatchIn<4, 4>& a, const BatchIn<4, 4>& b) {
        return map_batch2<4, 4, 4, 4, 4, 4>(
            a, b,
            [](const Eigen::Matrix4d& x, const Eigen::Matrix4d& y) { return se3::compose(x, y); });
      },
      nb::arg("a").noconvert(), nb::arg("b").noconvert(), "Compose (N, 4, 4) transforms.");
  m.def(
      "act",
      [](const BatchIn<4, 4>& transform, const BatchIn<3>& points) {
        return map_batch2<4, 4, 3, 1, 3, 1>(
            transform, points,
            [](const Eigen::Matrix4d& t, const Eigen::Vector3d& p) { return se3::act(t, p); });
      },
      nb::arg("transform").noconvert(), nb::arg("points").noconvert(),
      "Apply (N, 4, 4) transforms to (N, 3) points.");
  m.def(
      "adjoint",
      [](const BatchIn<4, 4>& transform) {
        return map_batch<4, 4, 6, 6>(transform,
                                     [](const Eigen::Matrix4d& t) { return se3::adjoint(t); });
      },
      nb::arg("transform").noconvert(), "Map (N, 4, 4) transforms to (N, 6, 6) adjoints.");
  m.def(
      "right_jacobian",
      [](const BatchIn<6>& xi) {
        return map_batch<6, 1, 6, 6>(xi,
                                     [](const se3::Vector6d& x) { return se3::right_jacobian(x); });
      },
      nb::arg("xi").noconvert(), "Map (N, 6) twists to their (N, 6, 6) right Jacobians.");
  m.def(
      "left_jacobian",
      [](const BatchIn<6>& xi) {
        return map_batch<6, 1, 6, 6>(xi,
                                     [](const se3::Vector6d& x) { return se3::left_jacobian(x); });
      },
      nb::arg("xi").noconvert(), "Map (N, 6) twists to their (N, 6, 6) left Jacobians.");
  m.def(
      "right_jacobian_inverse",
      [](const BatchIn<6>& xi) {
        return map_batch<6, 1, 6, 6>(
            xi, [](const se3::Vector6d& x) { return se3::right_jacobian_inverse(x); });
      },
      nb::arg("xi").noconvert(), "Map (N, 6) twists to their inverse right Jacobians.");
  m.def(
      "left_jacobian_inverse",
      [](const BatchIn<6>& xi) {
        return map_batch<6, 1, 6, 6>(
            xi, [](const se3::Vector6d& x) { return se3::left_jacobian_inverse(x); });
      },
      nb::arg("xi").noconvert(), "Map (N, 6) twists to their inverse left Jacobians.");
}

}  // namespace slamlab::bindings
