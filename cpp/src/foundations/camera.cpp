/// @file
/// @brief Implementation of the camera models declared in slam_it/foundations/camera.hpp.
#include "slam_it/foundations/camera.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

namespace slam_it::foundations::camera {

namespace {

/// @brief The linear map from radial `[f, cx, cy, k1, k2]` to radtan
///        `[f, f, cx, cy, k1, k2, 0, 0, 0]`.
/// @return The 9 x 5 map.
Eigen::Matrix<double, 9, 5> radial_to_radtan() {
  Eigen::Matrix<double, 9, 5> map = Eigen::Matrix<double, 9, 5>::Zero();
  map(0, 0) = 1.0;
  map(1, 0) = 1.0;
  map(2, 1) = 1.0;
  map(3, 2) = 1.0;
  map(4, 3) = 1.0;
  map(5, 4) = 1.0;
  return map;
}

/// @brief The terms shared by radtan distortion and its Jacobian.
struct RadialTerms {
  /// @brief Normalised x.
  double x;
  /// @brief Normalised y.
  double y;
  /// @brief `x² + y²`.
  double r2;
  /// @brief The radial factor `1 + k1 r² + k2 r⁴ + k3 r⁶`.
  double radial;
};

/// @brief Compute the terms shared by distortion and its Jacobian.
/// @param k The intrinsics (radtan layout).
/// @param xy The normalised coordinates.
/// @return The terms.
RadialTerms radial_terms(const Eigen::Ref<const Eigen::VectorXd>& k, const Eigen::Vector2d& xy) {
  const double r2 = (xy.x() * xy.x()) + (xy.y() * xy.y());
  return {.x = xy.x(),
          .y = xy.y(),
          .r2 = r2,
          .radial = 1.0 + (r2 * (k(4) + (r2 * (k(5) + (r2 * k(8))))))};
}

/// @brief Apply radtan distortion to normalised coordinates.
/// @param k The intrinsics (radtan layout).
/// @param xy The normalised coordinates.
/// @return The distorted coordinates.
Eigen::Vector2d distort(const Eigen::Ref<const Eigen::VectorXd>& k, const Eigen::Vector2d& xy) {
  const auto [x, y, r2, radial] = radial_terms(k, xy);
  return {(x * radial) + (2.0 * k(6) * x * y) + (k(7) * (r2 + (2.0 * x * x))),
          (y * radial) + (k(6) * (r2 + (2.0 * y * y))) + (2.0 * k(7) * x * y)};
}

/// @brief The Jacobian of radtan distortion with respect to the normalised coordinates.
/// @param k The intrinsics (radtan layout).
/// @param xy The normalised coordinates.
/// @return d(distorted)/d(normalised).
Eigen::Matrix2d distortion_jacobian(const Eigen::Ref<const Eigen::VectorXd>& k,
                                    const Eigen::Vector2d& xy) {
  const auto [x, y, r2, radial] = radial_terms(k, xy);
  const double d_radial = k(4) + (r2 * ((2.0 * k(5)) + (3.0 * k(8) * r2)));
  const double cross = (2.0 * x * y * d_radial) + (2.0 * k(6) * x) + (2.0 * k(7) * y);
  Eigen::Matrix2d j;
  j << radial + (2.0 * x * x * d_radial) + (2.0 * k(6) * y) + (6.0 * k(7) * x), cross, cross,
      radial + (2.0 * y * y * d_radial) + (6.0 * k(6) * y) + (2.0 * k(7) * x);
  return j;
}

/// @brief Whether a distortion residual is within the Newton tolerance.
/// @param residual The residual.
/// @param target The distorted coordinates being inverted.
/// @return True when converged.
bool converged(const Eigen::Vector2d& residual, const Eigen::Vector2d& target) {
  for (int i = 0; i < 2; ++i) {
    if (std::abs(residual(i)) > kUndistortTolerance * std::max(1.0, std::abs(target(i)))) {
      return false;
    }
  }
  return true;
}

/// @brief Check a projection's inputs and return the point on the normalised image plane.
/// @param model The model.
/// @param intrinsics The intrinsics.
/// @param point The point, which must be in front of the camera.
/// @return `(X/Z, Y/Z)`, or `InvalidInput`.
Result<Eigen::Vector2d> normalise(Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                  const Eigen::Vector3d& point) {
  if (auto checked = check_intrinsics(model, intrinsics); !checked) {
    return std::unexpected(checked.error());
  }
  if (point.z() <= kMinDepth) {
    return fail(ErrorKind::kInvalidInput, "points: Z must exceed 1e-09 (in front of the camera)");
  }
  return Eigen::Vector2d(point.head<2>() / point.z());
}

}  // namespace

const char* name(Model model) {
  switch (model) {
    case Model::kPinhole:
      return "pinhole";
    case Model::kRadTan:
      return "radtan";
    case Model::kRadial:
      return "radial";
    case Model::kCount:
      break;
  }
  return "unknown";
}

std::span<const Eigen::Index> focal_indices(Model model) {
  static constexpr std::array<Eigen::Index, 2> kBoth{0, 1};
  static constexpr std::array<Eigen::Index, 1> kOne{0};
  if (model == Model::kRadial) {
    return kOne;
  }
  return kBoth;
}

Result<void> check_intrinsics(Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics) {
  if (model >= Model::kCount) {
    return fail(ErrorKind::kInvalidInput, "model: not a camera model (kCount)");
  }
  if (intrinsics.size() != intrinsics_size(model)) {
    return fail(ErrorKind::kInvalidInput, "intrinsics: wrong length for the camera model");
  }
  if (std::ranges::any_of(focal_indices(model),
                          [&](Eigen::Index i) { return intrinsics(i) <= 0.0; })) {
    return fail(ErrorKind::kInvalidInput, "intrinsics: focal lengths must be positive");
  }
  return {};
}

namespace {

/// @brief Project with a pinhole or radtan model (radial already expanded).
/// @param model The model (pinhole or radtan).
/// @param intrinsics The model's intrinsics.
/// @param point The point.
/// @return The pixel, or `InvalidInput`.
Result<Eigen::Vector2d> project_expanded(Model model,
                                         const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                         const Eigen::Vector3d& point) {
  return normalise(model, intrinsics, point).transform([&](const Eigen::Vector2d& xy) {
    const Eigen::Vector2d xd = model == Model::kRadTan ? distort(intrinsics, xy) : xy;
    return Eigen::Vector2d((intrinsics(0) * xd.x()) + intrinsics(2),
                           (intrinsics(1) * xd.y()) + intrinsics(3));
  });
}

/// @brief Differentiate a pinhole or radtan projection (radial already expanded).
/// @param model The model (pinhole or radtan).
/// @param intrinsics The model's intrinsics.
/// @param point The point.
/// @return The Jacobian, or `InvalidInput`.
Result<ProjectionJacobian> project_jacobian_expanded(
    Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
    const Eigen::Vector3d& point) {
  const Result<Eigen::Vector2d> normalised = normalise(model, intrinsics, point);
  if (!normalised) {
    return std::unexpected(normalised.error());
  }
  const Eigen::Vector2d& xy = *normalised;
  const double z = point.z();
  Eigen::Matrix<double, 2, 3> d_xy_d_point;
  d_xy_d_point << 1.0 / z, 0.0, -xy.x() / z, 0.0, 1.0 / z, -xy.y() / z;
  const Eigen::Matrix2d focal = Eigen::Vector2d(intrinsics(0), intrinsics(1)).asDiagonal();
  const bool radtan = model == Model::kRadTan;
  const Eigen::Vector2d xd = radtan ? distort(intrinsics, xy) : xy;
  const Eigen::Matrix2d d_xd_d_xy =
      radtan ? distortion_jacobian(intrinsics, xy) : Eigen::Matrix2d::Identity();
  ProjectionJacobian out = ProjectionJacobian::Zero(2, 3 + intrinsics.size());
  out.leftCols<3>() = focal * d_xd_d_xy * d_xy_d_point;
  out(0, 3) = xd.x();
  out(1, 4) = xd.y();
  out(0, 5) = 1.0;
  out(1, 6) = 1.0;
  if (radtan) {
    const double x = xy.x();
    const double y = xy.y();
    const double r2 = (x * x) + (y * y);
    Eigen::Matrix<double, 2, 5> d_dist;
    d_dist << x * r2, x * r2 * r2, 2.0 * x * y, r2 + (2.0 * x * x), x * r2 * r2 * r2, y * r2,
        y * r2 * r2, r2 + (2.0 * y * y), 2.0 * x * y, y * r2 * r2 * r2;
    out.rightCols<5>() = focal * d_dist;
  }
  return out;
}

/// @brief Unproject with a pinhole or radtan model (radial already expanded).
/// @param model The model (pinhole or radtan).
/// @param intrinsics The model's intrinsics.
/// @param pixel The pixel.
/// @return The bearing, or `NotConverged`.
Result<Eigen::Vector3d> unproject_expanded(Model model,
                                           const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                           const Eigen::Vector2d& pixel) {
  if (auto checked = check_intrinsics(model, intrinsics); !checked) {
    return std::unexpected(checked.error());
  }
  const Eigen::Vector2d target((pixel.x() - intrinsics(2)) / intrinsics(0),
                               (pixel.y() - intrinsics(3)) / intrinsics(1));
  Eigen::Vector2d xy = target;
  if (model == Model::kRadTan) {
    bool done = false;
    for (int it = 0; it <= kUndistortMaxIterations && !done; ++it) {
      const Eigen::Vector2d residual = distort(intrinsics, xy) - target;
      if (converged(residual, target)) {
        done = true;
      } else if (it < kUndistortMaxIterations) {
        const Eigen::Matrix2d jac = distortion_jacobian(intrinsics, xy);
        if (jac.determinant() <= 0.0) {
          return fail(ErrorKind::kNotConverged,
                      "undistort: pixel outside the invertible region of the distortion");
        }
        xy -= jac.partialPivLu().solve(residual);
      }
    }
    if (!done) {
      return fail(ErrorKind::kNotConverged, "undistort: Newton did not converge in 30 iterations");
    }
  }
  return Eigen::Vector3d(xy.x(), xy.y(), 1.0).normalized();
}

/// @brief A camera's intrinsics in the form the projection code computes with: radial
///        expanded to radtan, the others unchanged.
struct Expanded {
  /// @brief Pinhole or radtan.
  Model model;
  /// @brief The intrinsics in that model's layout.
  Eigen::VectorXd intrinsics;
};

/// @brief Check a model's intrinsics and expand radial to radtan.
/// @param model The model.
/// @param intrinsics The intrinsics.
/// @return The expanded form, or `InvalidInput`.
Result<Expanded> expand(Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics) {
  if (auto checked = check_intrinsics(model, intrinsics); !checked) {
    return std::unexpected(checked.error());
  }
  if (model == Model::kRadial) {
    return Expanded{.model = Model::kRadTan, .intrinsics = radial_to_radtan() * intrinsics};
  }
  return Expanded{.model = model, .intrinsics = intrinsics};
}

}  // namespace

Result<Eigen::Vector2d> project(Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                const Eigen::Vector3d& point) {
  return expand(model, intrinsics).and_then([&](const Expanded& camera) {
    return project_expanded(camera.model, camera.intrinsics, point);
  });
}

Result<ProjectionJacobian> project_jacobian(Model model,
                                            const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                            const Eigen::Vector3d& point) {
  return expand(model, intrinsics)
      .and_then([&](const Expanded& camera) {
        return project_jacobian_expanded(camera.model, camera.intrinsics, point);
      })
      .transform([&](ProjectionJacobian full) -> ProjectionJacobian {
        if (model != Model::kRadial) {
          return full;
        }
        ProjectionJacobian out(2, 3 + 5);
        out << full.leftCols<3>(), full.rightCols<9>() * radial_to_radtan();
        return out;
      });
}

Result<Eigen::Vector3d> unproject(Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                  const Eigen::Vector2d& pixel) {
  return expand(model, intrinsics).and_then([&](const Expanded& camera) {
    return unproject_expanded(camera.model, camera.intrinsics, pixel);
  });
}

}  // namespace slam_it::foundations::camera
