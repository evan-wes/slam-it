/// @file
/// @brief The Rust core's bridge to GTSAM (`slam_it_foundations::graphs`): GTSAM's graph,
///        values and Levenberg-Marquardt, with factors that call back into Rust.
///
/// Rust builds a problem through these functions: its variables (poses, similarities, points),
/// GTSAM's own between and equality factors, and Rust factors, whose residual and Jacobians a
/// Rust `Evaluator` computes (`evaluate`, declared by the bridge). The solve is driven from
/// Rust one iteration at a time, so the stopping rule is Rust's own, as in
/// `slam_it.foundations.graphs`. Every function that evaluates the factors takes the evaluator,
/// and holds it only for the call. A pose crosses as 16 row-major doubles, `T_world_cam`; a
/// similarity as GTSAM's own 4x4, `[[R, t'], [0, 1/s]]` (mapping `p` to `s (R p + t')`), which
/// Rust converts to and from its `[[s R, t], [0, 1]]`, so the bridge holds no geometry of its
/// own; a point as 3.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "rust/cxx.h"

/// @brief The Rust core's bridge to GTSAM.
namespace slam_it::rust_bridge {

struct Evaluator;

/// @brief A problem: GTSAM's graph and starting values, and the slot its Rust factors read the
///        current call's evaluator from. Its GTSAM state is defined in the implementation, so
///        that this header, which the bridge's generated code includes, needs no GTSAM.
class Problem {
 public:
  /// @brief The problem's GTSAM state.
  struct State;

  /// @brief An empty problem.
  Problem();
  /// @brief Release the GTSAM state.
  ~Problem();
  Problem(const Problem&) = delete;
  Problem& operator=(const Problem&) = delete;
  Problem(Problem&&) = delete;
  Problem& operator=(Problem&&) = delete;

  /// @brief The GTSAM state.
  std::unique_ptr<State> state;
};

/// @brief Levenberg-Marquardt on a problem; its GTSAM state is defined in the implementation.
class Optimizer {
 public:
  /// @brief The optimizer's GTSAM state.
  struct State;

  /// @brief Take the GTSAM state.
  /// @param state The state.
  explicit Optimizer(std::unique_ptr<State> state);
  /// @brief Release the GTSAM state.
  ~Optimizer();
  Optimizer(const Optimizer&) = delete;
  Optimizer& operator=(const Optimizer&) = delete;
  Optimizer(Optimizer&&) = delete;
  Optimizer& operator=(Optimizer&&) = delete;

  /// @brief The GTSAM state.
  std::unique_ptr<State> state;
};

/// @brief Make an empty problem.
/// @return The problem.
std::unique_ptr<Problem> new_problem();

/// @brief Add a pose variable.
/// @param problem The problem.
/// @param key Its key.
/// @param matrix Its `T_world_cam`, 16 row-major.
void add_pose(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix);

/// @brief Add a similarity variable.
/// @param problem The problem.
/// @param key Its key.
/// @param matrix GTSAM's `[[R, t'], [0, 1/s]]`, 16 row-major, `s > 0`.
void add_similarity(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix);

/// @brief Add a point variable.
/// @param problem The problem.
/// @param key Its key.
/// @param point Its position, 3.
void add_point(Problem& problem, std::uint64_t key, rust::Slice<const double> point);

/// @brief Add GTSAM's between factor on two poses, with unit information.
/// @param problem The problem.
/// @param first The first pose's key.
/// @param second The second's.
/// @param relative The measured `T_first_second`, 16 row-major.
void add_between_poses(Problem& problem, std::uint64_t first, std::uint64_t second,
                       rust::Slice<const double> relative);

/// @brief Add GTSAM's between factor on two similarities, with unit information.
/// @param problem The problem.
/// @param first The first similarity's key.
/// @param second The second's.
/// @param relative The measured relative similarity, 16 row-major.
void add_between_similarities(Problem& problem, std::uint64_t first, std::uint64_t second,
                              rust::Slice<const double> relative);

/// @brief Hold a pose exactly at a value (GTSAM's `NonlinearEquality`).
/// @param problem The problem.
/// @param key The pose's key.
/// @param matrix The value, 16 row-major.
void fix_pose(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix);

/// @brief Hold a similarity exactly at a value.
/// @param problem The problem.
/// @param key The similarity's key.
/// @param matrix The value, 16 row-major.
void fix_similarity(Problem& problem, std::uint64_t key, rust::Slice<const double> matrix);

/// @brief Add a factor Rust evaluates.
/// @param problem The problem.
/// @param keys Its variables' keys.
/// @param kinds Each variable's kind: 0 a pose, 1 a similarity, 2 a point.
/// @param dimension The residual's length.
/// @param information Its isotropic information, positive.
/// @param huber The Huber loss's threshold on the whitened norm, or 0 for none.
/// @param index The factor's index, which `evaluate` is given.
void add_rust_factor(Problem& problem, rust::Slice<const std::uint64_t> keys,
                     rust::Slice<const std::uint8_t> kinds, std::size_t dimension,
                     double information, double huber, std::size_t index);

/// @brief Start Levenberg-Marquardt on a problem; its starting error is evaluated here.
/// @param problem The problem.
/// @param evaluator The Rust factors' evaluator.
/// @param initial_damping The first damping.
/// @param relative_tolerance GTSAM's relative error tolerance.
/// @param absolute_tolerance GTSAM's absolute error tolerance.
/// @return The optimizer.
/// @throws std::exception (a Rust `cxx::Exception`) if GTSAM fails.
std::unique_ptr<Optimizer> new_optimizer(const Problem& problem, const Evaluator& evaluator,
                                         double initial_damping, double relative_tolerance,
                                         double absolute_tolerance);

/// @brief Run one Levenberg-Marquardt iteration.
/// @param optimizer The optimizer.
/// @param evaluator The Rust factors' evaluator.
/// @throws std::exception if GTSAM fails.
void iterate(Optimizer& optimizer, const Evaluator& evaluator);

/// @brief The error at the optimizer's values.
/// @param optimizer The optimizer.
/// @return GTSAM's error.
double error(const Optimizer& optimizer);

/// @brief The optimizer's current damping.
/// @param optimizer The optimizer.
/// @return GTSAM's lambda.
double damping(const Optimizer& optimizer);

/// @brief The damping at which the optimizer gives up a search.
/// @param optimizer The optimizer.
/// @return GTSAM's lambda upper bound.
double damping_limit(const Optimizer& optimizer);

/// @brief Read a pose from the optimizer's values.
/// @param optimizer The optimizer.
/// @param key The pose's key.
/// @param out 16 doubles, row-major.
/// @throws std::exception if the key holds no pose.
void pose_at(const Optimizer& optimizer, std::uint64_t key, rust::Slice<double> out);

/// @brief Read a similarity from the optimizer's values.
/// @param optimizer The optimizer.
/// @param key The similarity's key.
/// @param out 16 doubles, row-major.
/// @throws std::exception if the key holds no similarity.
void similarity_at(const Optimizer& optimizer, std::uint64_t key, rust::Slice<double> out);

/// @brief Read a point from the optimizer's values.
/// @param optimizer The optimizer.
/// @param key The point's key.
/// @param out 3 doubles.
/// @throws std::exception if the key holds no point.
void point_at(const Optimizer& optimizer, std::uint64_t key, rust::Slice<double> out);

}  // namespace slam_it::rust_bridge
