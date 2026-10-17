/// @file
/// @brief The Rust core's bridge to GTSAM: its implementation (`graphs_bridge.hpp`).
#include "slam-it-foundations/cxx/graphs_bridge.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/Core>
#include <stdexcept>
#include <utility>
#include <vector>

#include "slam-it-foundations/src/graphs.rs.h"

namespace slam_it::rust_bridge {

namespace {

/// @brief A row-major 4x4 matrix.
using Matrix4Rows = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/// @brief Read 16 row-major doubles as a 4x4 matrix.
/// @param values The doubles.
/// @return The matrix.
/// @throws std::invalid_argument for a length other than 16.
Eigen::Matrix4d matrix_of(rust::Slice<const double> values) {
  if (values.size() != 16) {
    throw std::invalid_argument("graphs bridge: expected 16 values for a 4x4 matrix");
  }
  return Eigen::Map<const Matrix4Rows>(values.data());
}

/// @brief Write a 4x4 matrix as 16 row-major doubles.
/// @param matrix The matrix.
/// @param out The 16 doubles.
/// @throws std::invalid_argument for a length other than 16.
void write_matrix(const Eigen::Matrix4d& matrix, rust::Slice<double> out) {
  if (out.size() != 16) {
    throw std::invalid_argument("graphs bridge: expected room for 16 values");
  }
  Eigen::Map<Matrix4Rows>(out.data()) = matrix;
}

/// @brief A variable's kind, as Rust passes it.
enum class Kind : std::uint8_t {
  /// @brief A pose, `T_world_cam`: 16 values, tangent 6.
  kPose = 0,
  /// @brief A similarity: 16 values, tangent 7.
  kSimilarity = 1,
  /// @brief A point: 3 values, tangent 3.
  kPoint = 2,
};

/// @brief The evaluator the current call holds, shared by the factors and the optimizer.
using Slot = std::shared_ptr<const Evaluator*>;

/// @brief A factor whose residual and Jacobians Rust computes.
class RustFactor final : public gtsam::NoiseModelFactor {
 public:
  /// @brief Build the factor.
  /// @param noise Its noise model.
  /// @param keys Its variables' keys.
  /// @param kinds Their kinds.
  /// @param index Its index for `evaluate`.
  /// @param slot Where the current call's evaluator is.
  RustFactor(const gtsam::SharedNoiseModel& noise, const gtsam::KeyVector& keys,
             std::vector<Kind> kinds, std::size_t index, Slot slot)
      : gtsam::NoiseModelFactor(noise, keys),
        kinds_(std::move(kinds)),
        index_(index),
        slot_(std::move(slot)) {}

  using gtsam::NoiseModelFactor::unwhitenedError;

  /// @brief The residual and, if asked, each variable's Jacobian block, from Rust.
  /// @param x The values.
  /// @param jacobians Where to write the Jacobian blocks, if asked.
  /// @return The residual.
  /// @throws std::logic_error if no evaluator is held (a call that does not pass one).
  gtsam::Vector unwhitenedError(const gtsam::Values& x,
                                gtsam::OptionalMatrixVecType jacobians) const override {
    if (*slot_ == nullptr) {
      throw std::logic_error("graphs bridge: a factor was evaluated with no evaluator held");
    }
    std::vector<double> flat;
    std::size_t tangent = 0;
    for (std::size_t k = 0; k < kinds_.size(); ++k) {
      const gtsam::Key key = keys()[k];
      switch (kinds_[k]) {
        case Kind::kPose: {
          const Matrix4Rows matrix = x.at<gtsam::Pose3>(key).matrix();
          flat.insert(flat.end(), matrix.data(), matrix.data() + 16);
          tangent += 6;
          break;
        }
        case Kind::kSimilarity: {
          const Matrix4Rows matrix = x.at<gtsam::Similarity3>(key).matrix();
          flat.insert(flat.end(), matrix.data(), matrix.data() + 16);
          tangent += 7;
          break;
        }
        case Kind::kPoint: {
          const gtsam::Point3 point = x.at<gtsam::Point3>(key);
          flat.insert(flat.end(), point.data(), point.data() + 3);
          tangent += 3;
          break;
        }
      }
    }
    const auto rows = static_cast<Eigen::Index>(dim());
    gtsam::Vector residual(rows);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(
        rows, static_cast<Eigen::Index>(tangent));
    evaluate(**slot_, index_, rust::Slice<const double>(flat.data(), flat.size()),
             rust::Slice<double>(residual.data(), static_cast<std::size_t>(residual.size())),
             rust::Slice<double>(jacobian.data(), static_cast<std::size_t>(jacobian.size())));
    if (jacobians != nullptr) {
      jacobians->resize(kinds_.size());
      Eigen::Index column = 0;
      for (std::size_t k = 0; k < kinds_.size(); ++k) {
        const Eigen::Index width = kinds_[k] == Kind::kPose         ? 6
                                   : kinds_[k] == Kind::kSimilarity ? 7
                                                                    : 3;
        (*jacobians)[k] = jacobian.middleCols(column, width);
        column += width;
      }
    }
    return residual;
  }

 private:
  /// @brief Each variable's kind.
  std::vector<Kind> kinds_;
  /// @brief The factor's index for `evaluate`.
  std::size_t index_;
  /// @brief Where the current call's evaluator is.
  Slot slot_;
};

/// @brief Hold an evaluator in a slot for the length of a call.
class Holding {
 public:
  /// @brief Put the evaluator in the slot.
  /// @param slot The slot.
  /// @param evaluator The evaluator.
  Holding(const Slot& slot, const Evaluator& evaluator) : slot_(slot) { *slot_ = &evaluator; }
  /// @brief Empty the slot.
  ~Holding() { *slot_ = nullptr; }
  Holding(const Holding&) = delete;
  Holding& operator=(const Holding&) = delete;
  Holding(Holding&&) = delete;
  Holding& operator=(Holding&&) = delete;

 private:
  /// @brief The slot.
  Slot slot_;
};

}  // namespace

/// @brief A problem's GTSAM state.
struct Problem::State {
  /// @brief The factor graph.
  gtsam::NonlinearFactorGraph graph;
  /// @brief The starting values.
  gtsam::Values values;
  /// @brief Where the current call's evaluator is.
  Slot slot = std::make_shared<const Evaluator*>(nullptr);
};

Problem::Problem() : state(std::make_unique<State>()) {}

Problem::~Problem() = default;

/// @brief An optimizer's GTSAM state.
struct Optimizer::State {
  /// @brief Start Levenberg-Marquardt on a problem.
  /// @param problem The problem.
  /// @param params Its parameters.
  State(const Problem& problem, const gtsam::LevenbergMarquardtParams& params)
      : slot(problem.state->slot),
        params(params),
        optimizer(problem.state->graph, problem.state->values, params) {}

  /// @brief Where the current call's evaluator is.
  Slot slot;
  /// @brief The parameters.
  gtsam::LevenbergMarquardtParams params;
  /// @brief GTSAM's optimizer.
  gtsam::LevenbergMarquardtOptimizer optimizer;
};

Optimizer::Optimizer(std::unique_ptr<State> state) : state(std::move(state)) {}

Optimizer::~Optimizer() = default;

std::unique_ptr<Problem> new_problem() { return std::make_unique<Problem>(); }

void add_pose(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix) {
  problem.state->values.insert(key, gtsam::Pose3(matrix_of(matrix)));
}

void add_similarity(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix) {
  problem.state->values.insert(key, gtsam::Similarity3(matrix_of(matrix)));
}

void add_point(Problem& problem, std::uint64_t key, rust::Slice<const double> point) {
  if (point.size() != 3) {
    throw std::invalid_argument("graphs bridge: expected 3 values for a point");
  }
  problem.state->values.insert(key, gtsam::Point3(point[0], point[1], point[2]));
}

void add_between_poses(Problem& problem, std::uint64_t first, std::uint64_t second,
                       rust::Slice<const double> relative) {
  problem.state->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      first, second, gtsam::Pose3(matrix_of(relative)), gtsam::noiseModel::Unit::Create(6));
}

void add_between_similarities(Problem& problem, std::uint64_t first, std::uint64_t second,
                              rust::Slice<const double> relative) {
  problem.state->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(
      first, second, gtsam::Similarity3(matrix_of(relative)), gtsam::noiseModel::Unit::Create(7));
}

void fix_pose(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix) {
  problem.state->graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3>>(
      key, gtsam::Pose3(matrix_of(matrix)));
}

void fix_similarity(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix) {
  problem.state->graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Similarity3>>(
      key, gtsam::Similarity3(matrix_of(matrix)));
}

void add_rust_factor(Problem& problem, rust::Slice<const std::uint64_t> keys,
                     rust::Slice<const std::uint8_t> kinds, std::size_t dimension,
                     double information, double huber, std::size_t index) {
  if (keys.size() != kinds.size()) {
    throw std::invalid_argument("graphs bridge: expected a kind per key");
  }
  std::vector<Kind> variables;
  for (const std::uint8_t kind : kinds) {
    if (kind > static_cast<std::uint8_t>(Kind::kPoint)) {
      throw std::invalid_argument("graphs bridge: unknown variable kind");
    }
    variables.push_back(static_cast<Kind>(kind));
  }
  gtsam::SharedNoiseModel noise =
      gtsam::noiseModel::Isotropic::Precision(static_cast<std::size_t>(dimension), information);
  if (huber > 0.0) {
    noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber),
                                              noise);
  }
  problem.state->graph.emplace_shared<RustFactor>(noise, gtsam::KeyVector(keys.begin(), keys.end()),
                                                  std::move(variables), index, problem.state->slot);
}

std::unique_ptr<Optimizer> new_optimizer(const Problem& problem, const Evaluator& evaluator,
                                         double initial_damping, double relative_tolerance,
                                         double absolute_tolerance) {
  gtsam::LevenbergMarquardtParams params;
  params.setlambdaInitial(initial_damping);
  params.setRelativeErrorTol(relative_tolerance);
  params.setAbsoluteErrorTol(absolute_tolerance);
  const Holding holding(problem.state->slot, evaluator);
  return std::make_unique<Optimizer>(std::make_unique<Optimizer::State>(problem, params));
}

void iterate(Optimizer& optimizer, const Evaluator& evaluator) {
  const Holding holding(optimizer.state->slot, evaluator);
  optimizer.state->optimizer.iterate();
}

double error(const Optimizer& optimizer) { return optimizer.state->optimizer.error(); }

double damping(const Optimizer& optimizer) { return optimizer.state->optimizer.lambda(); }

double damping_limit(const Optimizer& optimizer) {
  return optimizer.state->params.getlambdaUpperBound();
}

void pose_at(const Optimizer& optimizer, std::uint64_t key, rust::Slice<double> out) {
  write_matrix(optimizer.state->optimizer.values().at<gtsam::Pose3>(key).matrix(), out);
}

void similarity_at(const Optimizer& optimizer, std::uint64_t key, rust::Slice<double> out) {
  write_matrix(optimizer.state->optimizer.values().at<gtsam::Similarity3>(key).matrix(), out);
}

void point_at(const Optimizer& optimizer, std::uint64_t key, rust::Slice<double> out) {
  if (out.size() != 3) {
    throw std::invalid_argument("graphs bridge: expected room for 3 values");
  }
  const gtsam::Point3 point = optimizer.state->optimizer.values().at<gtsam::Point3>(key);
  out[0] = point.x();
  out[1] = point.y();
  out[2] = point.z();
}

}  // namespace slam_it::rust_bridge
