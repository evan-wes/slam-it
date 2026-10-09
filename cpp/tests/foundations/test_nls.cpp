/// @file
/// @brief Native tests for slam_it_all::foundations::nls, runnable without Python.
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

#include "slam_it_all/foundations/nls.hpp"
#include "slam_it_all/foundations/so3.hpp"
#include "test_support.hpp"

namespace {

namespace nls = slam_it_all::foundations::nls;
namespace so3 = slam_it_all::foundations::so3;
using slam_it_all::foundations::ErrorKind;
using slam_it_all::testing::expect;

/// @brief Rosenbrock's function as two residual blocks, plus `unused` parameters it ignores.
class Rosenbrock {
 public:
  /// @brief `(x, y)` followed by the unused parameters.
  using State = Eigen::VectorXd;

  /// @brief Build the problem.
  /// @param unused How many parameters the residuals ignore (making the undamped system
  ///        singular).
  explicit Rosenbrock(Eigen::Index unused = 0) : unused_(unused) {}

  /// @brief The residuals `(10 (y - x^2), 1 - x)`.
  /// @param state The state.
  /// @return The residuals.
  [[nodiscard]] static Eigen::Vector2d residuals(const State& state) {
    return {10.0 * (state(1) - (state(0) * state(0))), 1.0 - state(0)};
  }

  /// @brief The blocks' squared norms.
  /// @param state The state.
  /// @return Two squared norms.
  [[nodiscard]] static Eigen::VectorXd squared_norms(const State& state) {
    return residuals(state).array().square();
  }

  /// @brief The dense normal equations.
  /// @param state The state.
  /// @param weights The block weights.
  /// @return The system.
  [[nodiscard]] nls::DenseSystem linearize(const State& state,
                                           const Eigen::VectorXd& weights) const {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(2, 2 + unused_);
    jacobian(0, 0) = -20.0 * state(0);
    jacobian(0, 1) = 10.0;
    jacobian(1, 0) = -1.0;
    return *nls::DenseSystem::build(jacobian, residuals(state), weights);
  }

  /// @brief Euclidean retraction.
  /// @param state The state.
  /// @param delta The step.
  /// @return `state + delta`.
  [[nodiscard]] static State retract(const State& state, const Eigen::VectorXd& delta) {
    return state + delta;
  }

 private:
  Eigen::Index unused_;
};

/// @brief Rosenbrock with a NaN in the Jacobian column of a parameter the residuals ignore.
class NanJacobian {
 public:
  /// @brief `(x, y, unused)`.
  using State = Eigen::VectorXd;

  /// @brief The blocks' squared norms.
  /// @param state The state.
  /// @return Two squared norms.
  [[nodiscard]] static Eigen::VectorXd squared_norms(const State& state) {
    return Rosenbrock::squared_norms(state);
  }

  /// @brief Rosenbrock's normal equations, with a NaN in the unused column.
  /// @param state The state.
  /// @param weights The block weights.
  /// @return The system.
  [[nodiscard]] static nls::DenseSystem linearize(const State& state,
                                                  const Eigen::VectorXd& weights) {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(2, 3);
    jacobian(0, 0) = -20.0 * state(0);
    jacobian(0, 1) = 10.0;
    jacobian(1, 0) = -1.0;
    jacobian(0, 2) = std::numeric_limits<double>::quiet_NaN();
    return *nls::DenseSystem::build(jacobian, Rosenbrock::residuals(state), weights);
  }

  /// @brief Euclidean retraction.
  /// @param state The state.
  /// @param delta The step.
  /// @return `state + delta`.
  [[nodiscard]] static State retract(const State& state, const Eigen::VectorXd& delta) {
    return state + delta;
  }
};

/// @brief Fit `y = a x + b` to points, one block per point.
class LineFit {
 public:
  /// @brief `(a, b)`.
  using State = Eigen::Vector2d;

  /// @brief Build the problem.
  /// @param xs The abscissae.
  /// @param ys The ordinates.
  LineFit(Eigen::VectorXd xs, Eigen::VectorXd ys) : xs_(std::move(xs)), ys_(std::move(ys)) {}

  /// @brief The residuals `a x_i + b - y_i`.
  /// @param state The line.
  /// @return The residuals.
  [[nodiscard]] Eigen::VectorXd residuals(const State& state) const {
    return ((state(0) * xs_).array() + state(1)).matrix() - ys_;
  }

  /// @brief The blocks' squared norms.
  /// @param state The line.
  /// @return One squared norm per point.
  [[nodiscard]] Eigen::VectorXd squared_norms(const State& state) const {
    return residuals(state).array().square();
  }

  /// @brief The dense normal equations.
  /// @param state The line.
  /// @param weights The block weights.
  /// @return The system.
  [[nodiscard]] nls::DenseSystem linearize(const State& state,
                                           const Eigen::VectorXd& weights) const {
    Eigen::MatrixXd jacobian(xs_.size(), 2);
    jacobian.col(0) = xs_;
    jacobian.col(1).setOnes();
    return *nls::DenseSystem::build(jacobian, residuals(state), weights);
  }

  /// @brief Euclidean retraction.
  /// @param state The line.
  /// @param delta The step.
  /// @return `state + delta`.
  [[nodiscard]] static State retract(const State& state, const Eigen::VectorXd& delta) {
    return state + delta;
  }

 private:
  Eigen::VectorXd xs_;
  Eigen::VectorXd ys_;
};

/// @brief Average rotations on SO(3): blocks `log(R^T R_i)`, with `R + delta = R exp(delta)`.
class RotationAverage {
 public:
  /// @brief The rotation.
  using State = Eigen::Matrix3d;

  /// @brief Build the problem.
  /// @param measured The measured rotations.
  explicit RotationAverage(std::vector<Eigen::Matrix3d> measured)
      : measured_(std::move(measured)) {}

  /// @brief One block's residual.
  /// @param state The rotation.
  /// @param i The block.
  /// @return `log(R^T R_i)`.
  [[nodiscard]] Eigen::Vector3d residual(const State& state, std::size_t i) const {
    return *so3::log(state.transpose() * measured_[i]);
  }

  /// @brief The blocks' squared norms.
  /// @param state The rotation.
  /// @return One squared norm per measurement.
  [[nodiscard]] Eigen::VectorXd squared_norms(const State& state) const {
    Eigen::VectorXd out(static_cast<Eigen::Index>(measured_.size()));
    for (std::size_t i = 0; i < measured_.size(); ++i) {
      out(static_cast<Eigen::Index>(i)) = residual(state, i).squaredNorm();
    }
    return out;
  }

  /// @brief The dense normal equations; each block's Jacobian is `-Jl^-1(r_i)`.
  /// @param state The rotation.
  /// @param weights The block weights.
  /// @return The system.
  [[nodiscard]] nls::DenseSystem linearize(const State& state,
                                           const Eigen::VectorXd& weights) const {
    const auto rows = static_cast<Eigen::Index>(3 * measured_.size());
    Eigen::MatrixXd jacobian(rows, 3);
    Eigen::VectorXd residuals(rows);
    for (std::size_t i = 0; i < measured_.size(); ++i) {
      const Eigen::Vector3d r = residual(state, i);
      const auto row = static_cast<Eigen::Index>(3 * i);
      residuals.segment<3>(row) = r;
      jacobian.middleRows<3>(row) = -*so3::left_jacobian_inverse(r);
    }
    return *nls::DenseSystem::build(jacobian, residuals, weights);
  }

  /// @brief Right retraction.
  /// @param state The rotation.
  /// @param delta The step.
  /// @return `R exp(delta)`.
  [[nodiscard]] static State retract(const State& state, const Eigen::VectorXd& delta) {
    return state * so3::exp(Eigen::Vector3d(delta));
  }

 private:
  std::vector<Eigen::Matrix3d> measured_;
};

/// @brief A line `y = 2 x + 1` with small deterministic noise and three gross outliers.
/// @return The problem.
LineFit line_with_outliers() {
  constexpr int kPoints = 30;
  Eigen::VectorXd xs(kPoints);
  Eigen::VectorXd ys(kPoints);
  for (int i = 0; i < kPoints; ++i) {
    xs(i) = 0.1 * i;
    ys(i) = (2.0 * xs(i)) + 1.0 + (0.01 * std::sin(1.7 * i));
  }
  ys(4) += 8.0;
  ys(17) -= 6.0;
  ys(25) += 9.0;
  return {std::move(xs), std::move(ys)};
}

/// @brief Tight tolerances, so a solve stops on the gradient or the step, not the cost.
/// @param method The step rule.
/// @param loss The loss.
/// @return The options.
nls::Options tight(nls::Method method, nls::Loss loss = {}) {
  return {.method = method,
          .loss = loss,
          .block_loss = {},
          .max_iterations = 200,
          .function_tolerance = 0.0,
          .gradient_tolerance = 1e-12,
          .step_tolerance = 1e-14};
}

}  // namespace

int main() {
  int failures = 0;

  // The losses and their weights, at an inlier and an outlier.
  const nls::Loss huber{.kind = nls::LossKind::kHuber, .scale = 1.0};
  const nls::Loss cauchy{.kind = nls::LossKind::kCauchy, .scale = 2.0};
  const nls::LossValue h_in = nls::evaluate(huber, 0.25);
  const nls::LossValue h_out = nls::evaluate(huber, 4.0);
  const nls::LossValue c_out = nls::evaluate(cauchy, 4.0);
  expect(h_in.rho == 0.25 && h_in.weight == 1.0 && h_out.rho == 3.0 && h_out.weight == 0.5,
         "huber: rho and weight on both sides of the scale", failures);
  expect(std::abs(c_out.rho - (4.0 * std::numbers::ln2)) < 1e-15 && c_out.weight == 0.5,
         "cauchy: rho and weight", failures);

  // From the classic start, Levenberg-Marquardt reaches Rosenbrock's minimum at (1, 1). The
  // undamped Gauss-Newton step overshoots to (1, -3.84), raising the cost from 12.1 to 1171,
  // so Gauss-Newton stops there; from nearer, it converges in two steps.
  const Eigen::Vector2d start(-1.2, 1.0);
  const auto at_minimum = [](const auto& solved) {
    return solved.has_value() && solved->summary.converged &&
           (solved->state - Eigen::Vector2d(1.0, 1.0)).norm() < 1e-10 &&
           solved->summary.final_cost < 1e-20;
  };
  expect(at_minimum(nls::solve(Rosenbrock{}, Eigen::VectorXd(start),
                               tight(nls::Method::kLevenbergMarquardt))),
         "solve: Levenberg-Marquardt converges to (1, 1)", failures);
  const auto overshoot =
      nls::solve(Rosenbrock{}, Eigen::VectorXd(start), tight(nls::Method::kGaussNewton));
  expect(overshoot.has_value() && overshoot->summary.termination == nls::Termination::kNoDecrease &&
             overshoot->state == start && overshoot->summary.iterations == 1,
         "solve: Gauss-Newton stops at a step that raises the cost", failures);
  expect(at_minimum(nls::solve(Rosenbrock{}, Eigen::VectorXd(Eigen::Vector2d(0.5, 0.5)),
                               tight(nls::Method::kGaussNewton))),
         "solve: Gauss-Newton converges from nearby", failures);

  // A parameter the residuals ignore leaves the undamped system singular; damping copes.
  Eigen::VectorXd padded(3);
  padded << start, 0.5;
  const auto singular = nls::solve(Rosenbrock(1), padded, tight(nls::Method::kGaussNewton));
  expect(singular.has_value() && singular->summary.termination == nls::Termination::kSingular &&
             !singular->summary.converged,
         "solve: Gauss-Newton stops on a singular system", failures);
  const auto damped = nls::solve(Rosenbrock(1), padded, tight(nls::Method::kLevenbergMarquardt));
  expect(damped.has_value() && damped->summary.converged &&
             (damped->state.head<2>() - Eigen::Vector2d(1.0, 1.0)).norm() < 1e-10 &&
             damped->state(2) == 0.5,
         "solve: Levenberg-Marquardt converges with an unobserved parameter", failures);

  // Robust losses resist the outliers that drag the plain least-squares line.
  const LineFit fit = line_with_outliers();
  const Eigen::Vector2d truth(2.0, 1.0);
  const auto plain =
      nls::solve(fit, Eigen::Vector2d::Zero(), tight(nls::Method::kLevenbergMarquardt));
  const auto robust = nls::solve(
      fit, Eigen::Vector2d::Zero(),
      tight(nls::Method::kLevenbergMarquardt, {.kind = nls::LossKind::kCauchy, .scale = 0.05}));
  expect(plain.has_value() && (plain->state - truth).norm() > 0.3,
         "solve: outliers drag least squares", failures);
  expect(robust.has_value() && robust->summary.converged && (robust->state - truth).norm() < 0.02,
         "solve: Cauchy recovers the line", failures);

  // On SO(3): the robust mean of noisy rotations with two outliers.
  const Eigen::Matrix3d rotation = so3::exp(Eigen::Vector3d(0.3, -0.2, 0.5));
  std::vector<Eigen::Matrix3d> measured;
  for (int i = 0; i < 12; ++i) {
    const Eigen::Vector3d noise(0.01 * std::sin(i), 0.01 * std::cos(2.0 * i),
                                0.01 * std::sin(3.0 * i));
    measured.emplace_back(rotation * so3::exp(noise));
  }
  measured.emplace_back(rotation * so3::exp(Eigen::Vector3d(0.9, 0.0, 0.0)));
  measured.emplace_back(rotation * so3::exp(Eigen::Vector3d(0.0, -1.1, 0.4)));
  const auto mean = nls::solve(
      RotationAverage(std::move(measured)), Eigen::Matrix3d::Identity(),
      tight(nls::Method::kLevenbergMarquardt, {.kind = nls::LossKind::kCauchy, .scale = 0.05}));
  const bool mean_ok = mean.has_value() && mean->summary.converged;
  const auto error =
      mean_ok ? so3::log(rotation.transpose() * mean->state) : so3::log(Eigen::Matrix3d::Zero());
  expect(mean_ok && error.has_value() && error->norm() < 0.01 &&
             (mean->state.transpose() * mean->state - Eigen::Matrix3d::Identity()).norm() < 1e-12,
         "solve: robust rotation averaging on SO(3)", failures);

  // A NaN gradient entry is never read as convergence, even at the minimum (panel L1).
  for (const nls::Method method : {nls::Method::kLevenbergMarquardt, nls::Method::kGaussNewton}) {
    const auto poisoned =
        nls::solve(NanJacobian{}, Eigen::VectorXd(Eigen::Vector3d(1.0, 1.0, 0.0)), tight(method));
    expect(poisoned.has_value() && !poisoned->summary.converged,
           "solve: a NaN gradient entry does not converge", failures);
  }

  // Strict mode turns an unconverged stop into an error; bad settings are rejected.
  nls::Options one_step = tight(nls::Method::kLevenbergMarquardt);
  one_step.max_iterations = 1;
  one_step.strict = true;
  const auto strict = nls::solve(Rosenbrock{}, Eigen::VectorXd(start), one_step);
  expect(!strict.has_value() && strict.error().kind == ErrorKind::kNotConverged,
         "solve: strict mode fails when not converged", failures);
  nls::Options bad;
  bad.initial_damping = -1.0;
  const auto rejected = nls::solve(Rosenbrock{}, Eigen::VectorXd(start), bad);
  expect(!rejected.has_value() && rejected.error().kind == ErrorKind::kInvalidInput,
         "solve: a negative damping is rejected", failures);
  const auto mismatched = nls::DenseSystem::build(
      Eigen::MatrixXd::Zero(3, 2), Eigen::VectorXd::Zero(3), Eigen::VectorXd::Ones(2));
  expect(!mismatched.has_value() && mismatched.error().kind == ErrorKind::kInvalidInput,
         "DenseSystem::build: mismatched sizes are rejected", failures);

  // A per-block loss naming one loss everywhere reproduces the uniform solve exactly; an
  // index outside the losses is rejected.
  const nls::Loss cauchy_line{.kind = nls::LossKind::kCauchy, .scale = 0.05};
  nls::Options per_block = tight(nls::Method::kLevenbergMarquardt);
  per_block.block_loss = {.losses = {nls::Loss{}, cauchy_line},
                          .index = std::vector<std::size_t>(30, 1)};
  const auto blockwise = nls::solve(fit, Eigen::Vector2d::Zero(), per_block);
  expect(blockwise.has_value() && robust.has_value() && blockwise->state == robust->state,
         "solve: a per-block loss matches the uniform loss it names", failures);
  per_block.block_loss.index[3] = 2;
  const auto outside = nls::solve(fit, Eigen::Vector2d::Zero(), per_block);
  expect(!outside.has_value() && outside.error().kind == ErrorKind::kInvalidInput,
         "solve: a block-loss index outside the losses is rejected", failures);

  return slam_it_all::testing::finish("test_nls", failures);
}
