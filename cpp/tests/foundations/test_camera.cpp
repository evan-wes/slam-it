/// @file
/// @brief Native tests for slamlab::foundations::camera, runnable without Python.
#include <Eigen/Dense>

#include "slamlab/foundations/camera.hpp"
#include "test_support.hpp"

namespace {

namespace camera = slamlab::foundations::camera;
using slamlab::foundations::ErrorKind;
using slamlab::testing::expect;

}  // namespace

int main() {
  int failures = 0;
  Eigen::VectorXd radtan(9);
  radtan << 520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172;
  const Eigen::Vector3d point(0.4, -0.3, 2.5);

  // Unprojecting a projected point gives the unit ray through it.
  const auto pixel = camera::project(camera::Model::kRadTan, radtan, point);
  const auto ray = pixel.has_value() ? camera::unproject(camera::Model::kRadTan, radtan, *pixel)
                                     : std::unexpected(pixel.error());
  expect(ray.has_value() && (*ray - point.normalized()).cwiseAbs().maxCoeff() < 1e-14,
         "unproject(project(p)) == p / |p|", failures);

  // The Jacobian with respect to the point matches central differences.
  const auto jac = camera::project_jacobian(camera::Model::kRadTan, radtan, point);
  bool jac_ok = jac.has_value();
  const double h = 1e-6;
  for (int k = 0; k < 3 && jac_ok; ++k) {
    const Eigen::Vector3d step = h * Eigen::Vector3d::Unit(k);
    const auto plus = camera::project(camera::Model::kRadTan, radtan, point + step);
    const auto minus = camera::project(camera::Model::kRadTan, radtan, point - step);
    jac_ok = ((*plus - *minus) / (2.0 * h) - jac->col(k)).cwiseAbs().maxCoeff() < 1e-5;
  }
  expect(jac_ok, "project_jacobian matches central differences", failures);

  // Error paths.
  expect(camera::project(camera::Model::kRadTan, radtan, Eigen::Vector3d(0, 0, -1)).error().kind ==
             ErrorKind::kInvalidInput,
         "project rejects a point behind the camera", failures);
  expect(camera::project(camera::Model::kPinhole, radtan, point).error().kind ==
             ErrorKind::kInvalidInput,
         "project rejects intrinsics of the wrong length", failures);
  Eigen::VectorXd barrel(9);
  barrel << 500, 500, 320, 240, -0.9, 0, 0, 0, 0;
  expect(
      camera::unproject(camera::Model::kRadTan, barrel, Eigen::Vector2d(2000, 1500)).error().kind ==
          ErrorKind::kNotConverged,
      "unproject reports an uninvertible pixel as NotConverged", failures);

  return slamlab::testing::finish("foundations/camera", failures);
}
