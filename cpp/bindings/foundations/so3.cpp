/// @file
/// @brief Bindings for `slamlab.foundations.so3` (`slamlab._cpp.foundations.so3`).
///
/// Every function is batched: float64 arrays of shape (N, ...) in, new arrays out, each item
/// computed by the core in `slamlab/foundations/so3.hpp`. Core errors become the matching
/// `slamlab.foundations.errors` exception.
#include "slamlab/foundations/so3.hpp"

#include <nanobind/nanobind.h>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace so3 = foundations::so3;

}  // namespace

void bind_so3(nb::module_& m) {
  m.def(
      "hat",
      [](const BatchIn<3>& omega) {
        return map_batch<3, 1, 3, 3>(omega, [](const Eigen::Vector3d& w) { return so3::hat(w); });
      },
      nb::arg("omega").noconvert(), "Map (N, 3) vectors to (N, 3, 3) skew-symmetric matrices.");
  m.def(
      "vee",
      [](const BatchIn<3, 3>& skew) {
        return map_batch<3, 3, 3, 1>(skew, [](const Eigen::Matrix3d& s) { return so3::vee(s); });
      },
      nb::arg("skew").noconvert(), "Map (N, 3, 3) skew-symmetric matrices to (N, 3) vectors.");
  m.def(
      "exp",
      [](const BatchIn<3>& omega) {
        return map_batch<3, 1, 3, 3>(omega, [](const Eigen::Vector3d& w) { return so3::exp(w); });
      },
      nb::arg("omega").noconvert(), "Map (N, 3) rotation vectors to (N, 3, 3) rotations.");
  m.def(
      "log",
      [](const BatchIn<3, 3>& rotation) {
        return map_batch<3, 3, 3, 1>(rotation,
                                     [](const Eigen::Matrix3d& r) { return so3::log(r); });
      },
      nb::arg("rotation").noconvert(),
      "Map (N, 3, 3) rotations to (N, 3) rotation vectors with angle in [0, pi].");
  m.def(
      "right_jacobian",
      [](const BatchIn<3>& omega) {
        return map_batch<3, 1, 3, 3>(
            omega, [](const Eigen::Vector3d& w) { return so3::right_jacobian(w); });
      },
      nb::arg("omega").noconvert(), "Map (N, 3) vectors to their (N, 3, 3) right Jacobians.");
  m.def(
      "left_jacobian",
      [](const BatchIn<3>& omega) {
        return map_batch<3, 1, 3, 3>(
            omega, [](const Eigen::Vector3d& w) { return so3::left_jacobian(w); });
      },
      nb::arg("omega").noconvert(), "Map (N, 3) vectors to their (N, 3, 3) left Jacobians.");
  m.def(
      "right_jacobian_inverse",
      [](const BatchIn<3>& omega) {
        return map_batch<3, 1, 3, 3>(
            omega, [](const Eigen::Vector3d& w) { return so3::right_jacobian_inverse(w); });
      },
      nb::arg("omega").noconvert(), "Map (N, 3) vectors to their inverse right Jacobians.");
  m.def(
      "left_jacobian_inverse",
      [](const BatchIn<3>& omega) {
        return map_batch<3, 1, 3, 3>(
            omega, [](const Eigen::Vector3d& w) { return so3::left_jacobian_inverse(w); });
      },
      nb::arg("omega").noconvert(), "Map (N, 3) vectors to their inverse left Jacobians.");
  m.def(
      "from_quaternion",
      [](const BatchIn<4>& quaternion) {
        return map_batch<4, 1, 3, 3>(
            quaternion, [](const Eigen::Vector4d& q) { return so3::from_quaternion(q); });
      },
      nb::arg("quaternion").noconvert(), "Map (N, 4) xyzw quaternions to (N, 3, 3) rotations.");
  m.def(
      "to_quaternion",
      [](const BatchIn<3, 3>& rotation) {
        return map_batch<3, 3, 4, 1>(
            rotation, [](const Eigen::Matrix3d& r) { return so3::to_quaternion(r); });
      },
      nb::arg("rotation").noconvert(), "Map (N, 3, 3) rotations to (N, 4) quaternions, w >= 0.");
  m.def(
      "project_to_so3",
      [](const BatchIn<3, 3>& matrix) {
        return map_batch<3, 3, 3, 3>(
            matrix, [](const Eigen::Matrix3d& x) { return so3::project_to_so3(x); });
      },
      nb::arg("matrix").noconvert(), "Map (N, 3, 3) matrices to their nearest rotations.");
}

}  // namespace slamlab::bindings
