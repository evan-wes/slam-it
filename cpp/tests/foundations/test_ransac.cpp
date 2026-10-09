/// @file
/// @brief Native tests for the RANSAC core (slam_it_all/foundations/ransac.hpp), through a
///        line-fitting estimator: the same run as tests/foundations/test_ransac.py, so the
///        two backends draw the same samples and stop on the same iteration.
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "slam_it_all/foundations/error.hpp"
#include "slam_it_all/foundations/nls.hpp"
#include "slam_it_all/foundations/ransac.hpp"
#include "slam_it_all/foundations/solve.hpp"
#include "test_support.hpp"

namespace {

namespace ransac = slam_it_all::foundations::ransac;
using slam_it_all::testing::expect;

/// @brief Fit 2D lines `a x + b y + c = 0` (with `a^2 + b^2 = 1`): two points minimally,
///        total least squares otherwise. Mirrors `LineEstimator` in test_ransac.py.
class LineEstimator {
 public:
  /// @brief A line `(a, b, c)`.
  using Model = Eigen::Vector3d;

  /// @brief Hold the points.
  /// @param points The points (N x 2).
  explicit LineEstimator(Eigen::MatrixX2d points) : points_(std::move(points)) {}

  /// @brief Two points determine a line.
  /// @return 2.
  [[nodiscard]] static std::uint32_t sample_size() { return 2; }

  /// @brief The number of points.
  /// @return N.
  [[nodiscard]] std::uint32_t size() const { return static_cast<std::uint32_t>(points_.rows()); }

  /// @brief Fit the line through two points.
  /// @param sample Two indices.
  /// @return The line, or none if the points coincide.
  [[nodiscard]] std::vector<Model> fit_minimal(std::span<const std::uint32_t> sample) const {
    const Eigen::Vector2d p = points_.row(sample[0]);
    const Eigen::Vector2d q = points_.row(sample[1]);
    const Eigen::Vector2d direction = q - p;
    const double length = std::hypot(direction(0), direction(1));
    if (length == 0.0) {
      return {};
    }
    const double a = -direction(1) / length;
    const double b = direction(0) / length;
    return {Model(a, b, -((a * p(0)) + (b * p(1))))};
  }

  /// @brief Fit by total least squares.
  /// @param subset The indices.
  /// @return The line.
  [[nodiscard]] std::vector<Model> fit_nonminimal(std::span<const std::uint32_t> subset) const {
    Eigen::MatrixX2d chosen(static_cast<Eigen::Index>(subset.size()), 2);
    for (std::size_t i = 0; i < subset.size(); ++i) {
      chosen.row(static_cast<Eigen::Index>(i)) = points_.row(subset[i]);
    }
    const Eigen::RowVector2d mean = chosen.colwise().mean();
    const Eigen::MatrixX2d centred = chosen.rowwise() - mean;
    const Eigen::JacobiSVD<Eigen::MatrixX2d, Eigen::ComputeFullV> svd(centred);
    const Eigen::Vector2d normal = svd.matrixV().col(1);
    return {Model(normal(0), normal(1), -normal.dot(mean.transpose()))};
  }

  /// @brief Squared point-to-line distances.
  /// @param model The line.
  /// @return One per point.
  [[nodiscard]] Eigen::VectorXd residuals(const Model& model) const {
    return ((points_ * model.head<2>()).array() + model(2)).square().matrix();
  }

 private:
  /// @brief The points.
  Eigen::MatrixX2d points_;
};

/// @brief The points of test_ransac.py's `parity_points`, built the same way.
/// @return 100 points, every third an outlier below the line.
Eigen::MatrixX2d parity_points() {
  Eigen::MatrixX2d points(100, 2);
  for (int i = 0; i < 100; ++i) {
    const double x = i / 10.0;
    const double y = i % 3 == 0 ? (-((37 * i) % 100) / 10.0) - 1.0
                                : (2.0 * x) + 1.0 + (0.01 * std::sin(static_cast<double>(i)));
    points.row(i) << x, y;
  }
  return points;
}

}  // namespace

// The refine-and-reselect cap is pinned to the value Python's test_ransac.py pins
// `ransac.REFINE_ROUNDS` to, so the two constants cannot drift apart.
static_assert(ransac::kRefineRounds == 4);

int main() {
  int failures = 0;
  const LineEstimator estimator(parity_points());
  const ransac::Options options{.threshold = 0.05, .seed = 42};
  const auto result = ransac::estimate(estimator, options);
  expect(result.has_value(), "the parity run succeeds", failures);
  if (result) {
    const ransac::Summary& summary = result->summary;
    expect(summary.iterations == 13 && summary.local_optimizations == 2 &&
               summary.degenerate_samples == 0 && summary.inlier_count == 66 && summary.converged,
           "the parity run stops where the Python reference does", failures);
    const Eigen::Vector3d python(0.8944198368027669, -0.4472283035919258, 0.4474675589870847);
    // The SVD's sign is free: compare up to sign.
    const Eigen::Vector3d model = result->state.model;
    const double error =
        std::min((model - python).cwiseAbs().maxCoeff(), (model + python).cwiseAbs().maxCoeff());
    expect(error < 1e-12, "the parity run's line matches the Python reference", failures);
    bool mask = true;
    for (int i = 0; i < 100; ++i) {
      mask = mask && result->state.inliers(i) == (i % 3 != 0);
    }
    expect(mask, "the inliers are the non-outliers", failures);
  }

  const auto capped =
      ransac::estimate(estimator, ransac::Options{.threshold = 0.05, .max_iterations = 1});
  expect(capped.has_value() && capped->summary.iterations == 1 && !capped->summary.converged &&
             capped->summary.termination == ransac::Termination::kMaxIterations,
         "an iteration cap is a result, not an error", failures);

  const LineEstimator coincident(Eigen::MatrixX2d::Zero(10, 2));
  const auto none = ransac::estimate(coincident, options);
  expect(!none.has_value() && none.error().kind == slam_it_all::foundations::ErrorKind::kDegenerate,
         "no model from any sample is Degenerate", failures);
  expect(!ransac::estimate(estimator, ransac::Options{.threshold = 0.0}).has_value(),
         "a zero threshold is rejected", failures);
  if (result) {
    // A failed refinement counts as a round and is not kept (panel A2 on PR 6).
    const auto failed = ransac::refine_consensus(
        estimator, *result,
        [](const LineEstimator::Model& /*model*/, const ransac::Indices& /*inliers*/)
            -> slam_it_all::foundations::Result<slam_it_all::foundations::solve::Solution<
                LineEstimator::Model, slam_it_all::foundations::nls::Summary>> {
          return slam_it_all::foundations::fail(slam_it_all::foundations::ErrorKind::kInvalidInput,
                                                "solve: the initial cost is not finite");
        },
        options.threshold, 2);
    expect(failed.summary.rounds == 1 && !failed.summary.refined &&
               !failed.summary.refinement.has_value() && failed.state.model == result->state.model,
           "a failed refinement counts as a round and is not kept", failures);
  }
  expect(ransac::required_iterations(1.0, 4, 0.99) == 0.0 &&
             std::isinf(ransac::required_iterations(0.0, 4, 0.99)),
         "the bound's edges", failures);
  return slam_it_all::testing::finish("foundations/ransac", failures);
}
