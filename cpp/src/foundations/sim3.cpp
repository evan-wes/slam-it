/// @file
/// @brief Sim(3) RANSAC: the C++ core's implementation (`sim3.hpp`).
#include "slam_it/foundations/sim3.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "slam_it/foundations/alignment.hpp"

namespace slam_it::foundations::sim3 {

namespace {

/// @brief Project points into an image, each in front of the camera.
/// @param points The points (N x 3), checked to be in front.
/// @param model The camera model.
/// @param intrinsics Its intrinsics, checked.
/// @param name The points' name, for a message.
/// @return The pixels (N x 2), or the projection's error.
Result<Rows2> project_all(const Eigen::Ref<const Rows3>& points, camera::Model model,
                          const Eigen::Ref<const Eigen::VectorXd>& intrinsics, const char* name) {
  Rows2 out(points.rows(), 2);
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const auto pixel = camera::project(model, intrinsics, points.row(i).transpose());
    if (!pixel) {
      return fail(ErrorKind::kInvalidInput, std::string(name) + ": " + pixel.error().message);
    }
    out.row(i) = pixel->transpose();
  }
  return out;
}

/// @brief A point's squared reprojection error over its variance, if it is in front.
/// @param model The camera model.
/// @param intrinsics Its intrinsics.
/// @param point The point carried into the camera.
/// @param observed Where its partner projects.
/// @param sigma2 The variance.
/// @return The ratio, or none behind the camera.
std::optional<double> scaled_error(camera::Model model,
                                   const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                   const Eigen::Vector3d& point, const Eigen::Vector2d& observed,
                                   double sigma2) {
  if (!(point.z() > camera::kMinDepth)) {
    return std::nullopt;
  }
  const auto pixel = camera::project(model, intrinsics, point);
  if (!pixel) {
    return std::nullopt;
  }
  return (*pixel - observed).squaredNorm() / sigma2;
}

/// @brief Check one variance per match: finite and positive.
/// @param values The variances.
/// @param count The matches.
/// @param name Their name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_variances(const Eigen::Ref<const Eigen::VectorXd>& values, Eigen::Index count,
                             const char* name) {
  if (values.size() != count || !values.allFinite() || !(values.array() > 0.0).all()) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": expected " + std::to_string(count) + " positive variances");
  }
  return {};
}

}  // namespace

Eigen::Matrix4d inverse(const Eigen::Matrix4d& similarity) {
  const Eigen::Matrix3d block = similarity.topLeftCorner<3, 3>();
  const double scale2 = block.col(0).squaredNorm();  // a column of s R has length s
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = block.transpose() / scale2;
  out.topRightCorner<3, 1>() = -out.topLeftCorner<3, 3>() * similarity.topRightCorner<3, 1>();
  return out;
}

SimilarityEstimator::SimilarityEstimator(Matches matches, camera::Model model,
                                         Eigen::VectorXd intrinsics, bool fixed_scale)
    : matches_(std::move(matches)),
      model_(model),
      intrinsics_(std::move(intrinsics)),
      fixed_scale_(fixed_scale) {}

std::uint32_t SimilarityEstimator::size() const {
  return static_cast<std::uint32_t>(matches_.points1.rows());
}

std::vector<SimilarityEstimator::Model> SimilarityEstimator::fit(
    std::span<const std::uint32_t> rows) const {
  const auto found = alignment::umeyama(
      correspondences::gather(matches_.points2, rows),
      correspondences::gather(matches_.points1, rows),
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(rows.size())), !fixed_scale_);
  return found ? std::vector<Model>{*found} : std::vector<Model>{};
}

std::vector<SimilarityEstimator::Model> SimilarityEstimator::fit_minimal(
    std::span<const std::uint32_t> sample) const {
  return fit(sample);
}

std::vector<SimilarityEstimator::Model> SimilarityEstimator::fit_nonminimal(
    std::span<const std::uint32_t> subset) const {
  return fit(subset);
}

Eigen::VectorXd SimilarityEstimator::residuals(const Model& model) const {
  const Model back = inverse(model);
  const Eigen::Index n = matches_.points1.rows();
  Eigen::VectorXd out(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    const Eigen::Vector3d into1 =
        (model.topLeftCorner<3, 3>() * matches_.points2.row(i).transpose()) +
        model.topRightCorner<3, 1>();
    const Eigen::Vector3d into2 =
        (back.topLeftCorner<3, 3>() * matches_.points1.row(i).transpose()) +
        back.topRightCorner<3, 1>();
    const auto error1 = scaled_error(model_, intrinsics_, into1, matches_.image1.row(i).transpose(),
                                     matches_.sigma2_1(i));
    const auto error2 = scaled_error(model_, intrinsics_, into2, matches_.image2.row(i).transpose(),
                                     matches_.sigma2_2(i));
    out(i) =
        (error1 && error2) ? std::max(*error1, *error2) : std::numeric_limits<double>::infinity();
  }
  return out;
}

Result<ransac::Solution<Eigen::Matrix4d>> sim3_ransac(
    const Eigen::Ref<const Rows3>& points1, const Eigen::Ref<const Rows3>& points2,
    const Eigen::Ref<const Eigen::VectorXd>& sigma2_1,
    const Eigen::Ref<const Eigen::VectorXd>& sigma2_2, camera::Model model,
    const Eigen::Ref<const Eigen::VectorXd>& intrinsics, bool fixed_scale,
    const ransac::Options& options) {
  const Eigen::Index n = points1.rows();
  if (points2.rows() != n || std::cmp_less(n, kSampleSize) || !points1.allFinite() ||
      !points2.allFinite()) {
    return fail(ErrorKind::kInvalidInput,
                "points1, points2: expected equal (N, 3) arrays, N >= 3, finite");
  }
  if ((points1.col(2).array() <= camera::kMinDepth).any() ||
      (points2.col(2).array() <= camera::kMinDepth).any()) {
    return fail(ErrorKind::kInvalidInput,
                "points1, points2: every point must be in front of its camera");
  }
  for (const auto& [values, name] :
       {std::pair{&sigma2_1, "sigma2_1"}, std::pair{&sigma2_2, "sigma2_2"}}) {
    if (auto checked = check_variances(*values, n, name); !checked) {
      return std::unexpected(checked.error());
    }
  }
  if (auto checked = camera::check_intrinsics(model, intrinsics); !checked) {
    return std::unexpected(checked.error());
  }
  auto image1 = project_all(points1, model, intrinsics, "points1");
  auto image2 = project_all(points2, model, intrinsics, "points2");
  if (!image1 || !image2) {
    return std::unexpected(image1 ? image2.error() : image1.error());
  }
  SimilarityEstimator::Matches matches{.points1 = points1,
                                       .points2 = points2,
                                       .image1 = std::move(*image1),
                                       .image2 = std::move(*image2),
                                       .sigma2_1 = sigma2_1,
                                       .sigma2_2 = sigma2_2};
  return ransac::estimate(SimilarityEstimator(std::move(matches), model, intrinsics, fixed_scale),
                          options);
}

}  // namespace slam_it::foundations::sim3
