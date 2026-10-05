/// @file
/// @brief Implementation of the stereo rectification declared in slamlab/foundations/stereo.hpp.
#include "slamlab/foundations/stereo.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::stereo {

namespace {

/// @brief A camera's smallest focal length.
/// @param model The model.
/// @param intrinsics Its intrinsics, checked.
/// @return `min(fx, fy)`, or `f` for the radial model.
double smallest_focal(camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics) {
  return model == camera::Model::kRadial ? intrinsics(0) : std::min(intrinsics(0), intrinsics(1));
}

/// @brief Check an image shape: each side from 1 to `image::kMaxSide`.
/// @param rows The rows.
/// @param cols The columns.
/// @return Nothing, or `InvalidInput`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (rows, cols), as numpy's shape
Result<void> check_shape(std::int64_t rows, std::int64_t cols) {
  if (rows < 1 || cols < 1 || rows > image::kMaxSide || cols > image::kMaxSide) {
    return fail(ErrorKind::kInvalidInput,
                "shape: expected (rows, cols), each from 1 to " + std::to_string(image::kMaxSide));
  }
  return {};
}

}  // namespace

Result<Rectification> rectify(camera::Model model_left,
                              const Eigen::Ref<const Eigen::VectorXd>& intrinsics_left,
                              camera::Model model_right,
                              const Eigen::Ref<const Eigen::VectorXd>& intrinsics_right,
                              const Eigen::Matrix4d& T_right_left, std::int64_t rows,
                              std::int64_t cols) {
  for (const auto& ok : {camera::check_intrinsics(model_left, intrinsics_left),
                         camera::check_intrinsics(model_right, intrinsics_right),
                         se3::check_transform(T_right_left), check_shape(rows, cols)}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  const Eigen::Matrix3d rotation = T_right_left.topLeftCorner<3, 3>();
  const Eigen::Vector3d translation = T_right_left.topRightCorner<3, 1>();
  const double baseline = translation.norm();
  if (baseline <= kMinBaseline) {
    return fail(ErrorKind::kDegenerate, "T_right_left: the camera centres coincide");
  }
  const auto omega = so3::log(rotation);
  if (!omega) {
    return std::unexpected(omega.error());
  }
  // Each camera turns half the way, then both turn the baseline onto the x axis.
  const Eigen::Matrix3d half = so3::exp(-0.5 * *omega);
  const Eigen::Vector3d t = half * translation;
  if (std::abs(t.x()) <= std::abs(t.y())) {
    return fail(ErrorKind::kInvalidInput,
                "T_right_left: the baseline is closer to vertical; only side-by-side pairs "
                "rectify");
  }
  const Eigen::Vector3d target(std::copysign(1.0, t.x()), 0.0, 0.0);
  Eigen::Vector3d axis = t.cross(target);
  const double norm = axis.norm();
  if (norm > 0.0) {
    axis *= std::acos(std::min(std::abs(t.x()) / t.norm(), 1.0)) / norm;
  }
  const Eigen::Matrix3d onto_x = so3::exp(axis);
  const double focal = std::min(smallest_focal(model_left, intrinsics_left),
                                smallest_focal(model_right, intrinsics_right));
  return Rectification{.rotation_left = onto_x * half.transpose(),
                       .rotation_right = onto_x * half,
                       .intrinsics = {focal, focal, static_cast<double>(cols - 1) / 2.0,
                                      static_cast<double>(rows - 1) / 2.0},
                       .baseline = baseline};
}

Result<image::Coordinates> rectification_map(camera::Model model,
                                             const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                             const Eigen::Matrix3d& rotation,
                                             const Eigen::Vector4d& rectified, std::int64_t rows,
                                             std::int64_t cols) {
  for (const auto& ok : {camera::check_intrinsics(model, intrinsics),
                         camera::check_intrinsics(camera::Model::kPinhole, rectified),
                         so3::check_rotation(rotation), check_shape(rows, cols)}) {
    if (!ok) {
      return std::unexpected(ok.error());
    }
  }
  const double f = rectified(0);
  const double cx = rectified(2);
  const double cy = rectified(3);
  const Eigen::Matrix3d& r = rotation;
  image::Coordinates out(rows * cols, 2);
  for (std::int64_t v = 0; v < rows; ++v) {
    for (std::int64_t u = 0; u < cols; ++u) {
      const double x = (static_cast<double>(u) - cx) / f;
      const double y = (static_cast<double>(v) - cy) / f;
      // R^T (x, y, 1), each product and sum in the Python reference's order.
      const Eigen::Vector3d ray((r(0, 0) * x) + (r(1, 0) * y) + r(2, 0),
                                (r(0, 1) * x) + (r(1, 1) * y) + r(2, 1),
                                (r(0, 2) * x) + (r(1, 2) * y) + r(2, 2));
      const Eigen::Index i = (v * cols) + u;
      if (ray.z() > camera::kMinDepth) {
        const auto pixel = camera::project(model, intrinsics, ray);
        if (!pixel) {
          return std::unexpected(pixel.error());
        }
        out.row(i) = pixel->transpose();
      } else {
        out.row(i) << kOutside, kOutside;
      }
    }
  }
  return out;
}

}  // namespace slamlab::foundations::stereo
