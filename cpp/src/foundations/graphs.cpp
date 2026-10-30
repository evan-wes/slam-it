/// @file
/// @brief Factor graphs on GTSAM: the C++ core's implementation (`graphs.hpp`).
#include "slam_it/foundations/graphs.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include "slam_it/foundations/so3.hpp"

namespace slam_it::foundations::graphs {

namespace {

/// @brief Each termination's name, in code order (`graphs.Termination`).
constexpr std::array<const char*, static_cast<std::size_t>(Termination::kCount)> kTerminationNames =
    {"function", "max_iterations", "damping_limit", "stopped", "small_steps"};

/// @brief A similarity's rotation, translation and scale.
struct Parts {
  /// @brief The rotation `R`.
  Eigen::Matrix3d rotation;
  /// @brief The translation `t`.
  Eigen::Vector3d translation;
  /// @brief The scale `s`.
  double scale;
};

/// @brief Split `[[s R, t], [0, 1]]` into `R`, `t` and `s`.
/// @param matrix The similarity, its scale positive.
/// @return Its parts.
Parts split(const Eigen::Matrix4d& matrix) {
  const double scale = std::cbrt(matrix.topLeftCorner<3, 3>().determinant());
  return Parts{.rotation = matrix.topLeftCorner<3, 3>() / scale,
               .translation = matrix.topRightCorner<3, 1>(),
               .scale = scale};
}

/// @brief GTSAM's `Similarity3` for `[[s R, t], [0, 1]]` (it maps `p` to `s (R p + t / s)`).
/// @param matrix The similarity.
/// @return The `Similarity3`.
gtsam::Similarity3 to_similarity3(const Eigen::Matrix4d& matrix) {
  const Parts parts = split(matrix);
  return {gtsam::Rot3(parts.rotation), gtsam::Point3(parts.translation / parts.scale), parts.scale};
}

/// @brief `[[s R, t], [0, 1]]` for GTSAM's `Similarity3`, whose matrix is `[[R, t'], [0, 1/s]]`.
/// @param value The `Similarity3`.
/// @return The similarity.
Eigen::Matrix4d from_similarity3(const gtsam::Similarity3& value) {
  const Eigen::Matrix4d gtsam_matrix = value.matrix();
  const double scale = 1.0 / gtsam_matrix(3, 3);
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = scale * gtsam_matrix.topLeftCorner<3, 3>();
  out.topRightCorner<3, 1>() = scale * gtsam_matrix.topRightCorner<3, 1>();
  return out;
}

/// @brief A camera-frame point's pixel and its Jacobian on the point.
struct Projected {
  /// @brief The pixel.
  Eigen::Vector2d pixel;
  /// @brief `d(pixel) / d(point)`.
  Eigen::Matrix<double, 2, 3> d_point;
};

/// @brief Project a camera-frame point, if it is in front of the camera.
/// @param model The camera model.
/// @param intrinsics Its intrinsics, checked.
/// @param point The point.
/// @return The pixel and its Jacobian, or nothing if the point is not in front.
std::optional<Projected> projected(camera::Model model,
                                   const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                   const Eigen::Vector3d& point) {
  if (!(point.z() > camera::kMinDepth)) {
    return std::nullopt;
  }
  const auto pixel = camera::project(model, intrinsics, point);
  const auto jacobian = camera::project_jacobian(model, intrinsics, point);
  if (!pixel || !jacobian) {
    return std::nullopt;
  }
  return Projected{.pixel = *pixel, .d_point = jacobian->leftCols<3>()};
}

/// @brief The motion's `[[s R, t], [0, 1]]`, from either kind of GTSAM value.
/// @tparam Motion `gtsam::Pose3` (rigid) or `gtsam::Similarity3`.
/// @param motion The value.
/// @return Its matrix.
template <typename Motion>
Eigen::Matrix4d motion_matrix(const Motion& motion) {
  if constexpr (std::is_same_v<Motion, gtsam::Pose3>) {
    return motion.matrix();
  } else {
    return from_similarity3(motion);
  }
}

/// @brief The GTSAM value of either kind for `[[s R, t], [0, 1]]`.
/// @tparam Motion `gtsam::Pose3` (rigid) or `gtsam::Similarity3`.
/// @param matrix The pose or similarity, checked.
/// @return The value.
template <typename Motion>
Motion to_motion(const Eigen::Matrix4d& matrix) {
  if constexpr (std::is_same_v<Motion, gtsam::Pose3>) {
    return gtsam::Pose3(matrix);
  } else {
    return to_similarity3(matrix);
  }
}

/// @brief One observation of a point, as a projection factor holds it.
struct Sighting {
  /// @brief The observed `(u, v)`, and the right column for a stereo observation.
  Eigen::VectorXd measured;
  /// @brief The camera model.
  camera::Model model;
  /// @brief Its intrinsics, checked.
  Eigen::VectorXd intrinsics;
  /// @brief The baseline times the focal length.
  double bf;
};

/// @brief An observation's reprojection error in a camera at a pose (`projection_residual`).
/// @param sighting The observation, checked.
/// @param pose The camera's `T_world_cam`.
/// @param point The point.
/// @return The residual and its Jacobians.
ProjectionResidual reproject(const Sighting& sighting, const Eigen::Matrix4d& pose,
                             const Eigen::Vector3d& point) {
  // The measurement's size was checked when the bundle was, so this cannot fail.
  return *projection_residual(pose, point, sighting.measured, sighting.model, sighting.intrinsics,
                              sighting.bf);
}

/// @brief A point's reprojection in a camera: a factor on the camera's pose and the point.
class ProjectionFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
 public:
  /// @brief The factor's base.
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;
  using Base::evaluateError;

  /// @brief Build the factor.
  /// @param pose The camera's key.
  /// @param point The point's key.
  /// @param sighting The observation.
  /// @param noise The observation's noise model.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (pose, point), GTSAM's key order
  ProjectionFactor(gtsam::Key pose, gtsam::Key point, Sighting sighting,
                   const gtsam::SharedNoiseModel& noise)
      : Base(noise, pose, point), sighting_(std::move(sighting)) {}

  /// @brief The residual and, where asked, its Jacobians (`projection_residual`).
  /// @param pose The camera's `T_world_cam`.
  /// @param point The point.
  /// @param d_pose Where to write the Jacobian on the pose, if asked.
  /// @param d_point Where to write the Jacobian on the point, if asked.
  /// @return The residual.
  // NOLINTBEGIN(bugprone-easily-swappable-parameters): GTSAM's override, (d_pose, d_point)
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& point,
                              gtsam::OptionalMatrixType d_pose,
                              gtsam::OptionalMatrixType d_point) const override {
    const ProjectionResidual result = reproject(sighting_, pose.matrix(), point);
    if (d_pose != nullptr) {
      *d_pose = result.d_pose;
    }
    if (d_point != nullptr) {
      *d_point = result.d_point;
    }
    return result.residual;
  }
  // NOLINTEND(bugprone-easily-swappable-parameters)

 private:
  /// @brief The observation.
  Sighting sighting_;
};

/// @brief A point's reprojection in a held camera: a factor on the point alone, the camera's
///        pose a constant (g2o's fixed vertex, as ORB-SLAM holds a camera).
class HeldProjectionFactor final : public gtsam::NoiseModelFactorN<gtsam::Point3> {
 public:
  /// @brief The factor's base.
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3>;
  using Base::evaluateError;

  /// @brief Build the factor.
  /// @param point The point's key.
  /// @param pose The held camera's `T_world_cam`.
  /// @param sighting The observation.
  /// @param noise The observation's noise model.
  HeldProjectionFactor(gtsam::Key point, Eigen::Matrix4d pose, Sighting sighting,
                       const gtsam::SharedNoiseModel& noise)
      : Base(noise, point), pose_(std::move(pose)), sighting_(std::move(sighting)) {}

  /// @brief The residual and, where asked, its Jacobian on the point (`projection_residual`).
  /// @param point The point.
  /// @param d_point Where to write the Jacobian on the point, if asked.
  /// @return The residual.
  gtsam::Vector evaluateError(const gtsam::Point3& point,
                              gtsam::OptionalMatrixType d_point) const override {
    const ProjectionResidual result = reproject(sighting_, pose_, point);
    if (d_point != nullptr) {
      *d_point = result.d_point;
    }
    return result.residual;
  }

 private:
  /// @brief The held camera's `T_world_cam`.
  Eigen::Matrix4d pose_;
  /// @brief The observation.
  Sighting sighting_;
};

/// @brief A pose graph's edge with one end held: GTSAM's between factor, its held end a
///        constant, so a factor on the free end alone.
/// @tparam Motion `gtsam::Pose3` (rigid) or `gtsam::Similarity3`.
template <typename Motion>
class HeldBetweenFactor final : public gtsam::NoiseModelFactorN<Motion> {
 public:
  /// @brief The factor's base.
  using Base = gtsam::NoiseModelFactorN<Motion>;
  using Base::evaluateError;

  /// @brief Build the factor.
  /// @param edge The edge's between factor, on both its ends' keys.
  /// @param held The held end's pose, checked.
  /// @param held_first Whether the held end is the edge's first.
  HeldBetweenFactor(const gtsam::BetweenFactor<Motion>& edge, const Eigen::Matrix4d& held,
                    bool held_first)
      : Base(edge.noiseModel(), held_first ? edge.key2() : edge.key1()),
        edge_(edge),
        held_(to_motion<Motion>(held)),
        held_first_(held_first) {}

  /// @brief The edge's residual and, where asked, its Jacobian on the free end, as GTSAM's
  ///        between factor computes them with its held end at the held value.
  /// @param free The free end's value.
  /// @param d_free Where to write the Jacobian on the free end, if asked.
  /// @return The residual.
  gtsam::Vector evaluateError(const Motion& free, gtsam::OptionalMatrixType d_free) const override {
    if (held_first_) {
      return edge_.evaluateError(held_, free, nullptr, d_free);
    }
    return edge_.evaluateError(free, held_, d_free, nullptr);
  }

 private:
  /// @brief The edge's between factor.
  gtsam::BetweenFactor<Motion> edge_;
  /// @brief The held end's value.
  Motion held_;
  /// @brief Whether the held end is the edge's first.
  bool held_first_;
};

/// @brief A pose graph's edges whose ends are both held: their stacked residuals, a constant,
///        as one factor on a free pose with a zero Jacobian. It moves nothing, but every error
///        GTSAM reads, its search over dampings' included, holds their error, as when the held
///        poses were variables.
/// @tparam Motion `gtsam::Pose3` (rigid) or `gtsam::Similarity3`.
template <typename Motion>
class HeldEdgesFactor final : public gtsam::NoiseModelFactorN<Motion> {
 public:
  /// @brief The factor's base.
  using Base = gtsam::NoiseModelFactorN<Motion>;
  using Base::evaluateError;

  /// @brief Build the factor, with unit information.
  /// @param key A free pose's key, which the residuals do not depend on.
  /// @param residuals The edges' residuals, stacked.
  HeldEdgesFactor(gtsam::Key key, const Eigen::VectorXd& residuals)
      : Base(gtsam::noiseModel::Unit::Create(residuals.size()), key), residuals_(residuals) {}

  /// @brief The stacked residuals and, where asked, their Jacobian on the free pose: zero.
  /// @param d_free Where to write the Jacobian, if asked.
  /// @return The residuals.
  gtsam::Vector evaluateError(const Motion& /*free*/,
                              gtsam::OptionalMatrixType d_free) const override {
    if (d_free != nullptr) {
      *d_free = gtsam::Matrix::Zero(residuals_.size(), gtsam::traits<Motion>::dimension);
    }
    return residuals_;
  }

 private:
  /// @brief The edges' residuals, stacked.
  Eigen::VectorXd residuals_;
};

/// @brief One side of a match carried through the motion: a unary factor on the motion.
/// @tparam Motion `gtsam::Pose3` (a rigid motion) or `gtsam::Similarity3`.
template <typename Motion>
class SimilarityFactor final : public gtsam::NoiseModelFactorN<Motion> {
 public:
  /// @brief The factor's base.
  using Base = gtsam::NoiseModelFactorN<Motion>;
  using Base::evaluateError;

  /// @brief A match's data, both sides, and which side this factor is.
  struct Match {
    /// @brief The point in camera 1's frame.
    Eigen::Vector3d point1;
    /// @brief The point in camera 2's frame.
    Eigen::Vector3d point2;
    /// @brief Where image 1 observes it.
    Eigen::Vector2d pixel1;
    /// @brief Where image 2 observes it.
    Eigen::Vector2d pixel2;
    /// @brief True for the error in image 1 (camera 2's point carried by `S_12`).
    bool first;
  };

  /// @brief Build the factor.
  /// @param key The motion's key.
  /// @param match The match and its side.
  /// @param model The camera model.
  /// @param intrinsics Its intrinsics, checked.
  /// @param noise The observation's noise model.
  SimilarityFactor(gtsam::Key key, Match match, camera::Model model, Eigen::VectorXd intrinsics,
                   const gtsam::SharedNoiseModel& noise)
      : Base(noise, key),
        match_(std::move(match)),
        model_(model),
        intrinsics_(std::move(intrinsics)) {}

  /// @brief The residual and, where asked, its Jacobian (`similarity_residuals`).
  /// @param motion The motion `S_12`.
  /// @param d_motion Where to write the Jacobian on the motion, if asked.
  /// @return The residual.
  gtsam::Vector evaluateError(const Motion& motion,
                              gtsam::OptionalMatrixType d_motion) const override {
    const SimilarityResiduals both = similarity_residuals(
        motion_matrix(motion), match_.point1, match_.point2, match_.pixel1, match_.pixel2, model_,
        intrinsics_, std::is_same_v<Motion, gtsam::Pose3>);
    if (d_motion != nullptr) {
      *d_motion = match_.first ? both.d_similarity1 : both.d_similarity2;
    }
    return match_.first ? both.residual1 : both.residual2;
  }

 private:
  /// @brief The match and its side.
  Match match_;
  /// @brief The camera model.
  camera::Model model_;
  /// @brief Its intrinsics.
  Eigen::VectorXd intrinsics_;
};

/// @brief A solve's values and how it went.
struct Solved {
  /// @brief The values at the end.
  gtsam::Values values;
  /// @brief How it went.
  Summary summary;
};

/// @brief The largest diagonal entry of a graph's normal equations at its values
///        (`graphs._largest_hessian_diagonal`): g2o's `computeLambdaInit`'s maximum, over the
///        variables as g2o's over its free vertices.
/// @param graph The factor graph.
/// @param values Its variables' values.
/// @return The largest entry, 0 for a graph with no factor.
/// @throws std::exception if GTSAM fails to linearise the graph.
double largest_hessian_diagonal(const gtsam::NonlinearFactorGraph& graph,
                                const gtsam::Values& values) {
  double largest = 0.0;
  for (const auto& [key, entries] : graph.linearize(values)->hessianDiagonal()) {
    if (entries.size() > 0) {
      largest = std::max(largest, entries.cwiseAbs().maxCoeff());
    }
  }
  return largest;
}

/// @brief GTSAM's parameters for a solve: its tolerances and damping (`graphs._params`).
/// @param graph The factor graph.
/// @param values The starting values.
/// @param options How the solve runs, checked.
/// @return The parameters.
/// @throws std::exception if GTSAM fails to linearise the graph (scaled damping).
gtsam::LevenbergMarquardtParams solve_params(const gtsam::NonlinearFactorGraph& graph,
                                             const gtsam::Values& values, const Options& options) {
  gtsam::LevenbergMarquardtParams params;
  params.setRelativeErrorTol(options.relative_tolerance);
  params.setAbsoluteErrorTol(options.absolute_tolerance);
  if (!options.scaled_damping) {
    params.setlambdaInitial(options.initial_damping);
    params.setlambdaFactor(kAbsoluteDampingFactor);
    params.setlambdaUpperBound(kAbsoluteDampingLimit);
    return params;
  }
  const double largest = largest_hessian_diagonal(graph, values);
  // With no gradient anywhere (every Jacobian zero), the damping cannot be scaled to it: tau alone.
  params.setlambdaInitial(options.damping_scale * (largest > 0.0 ? largest : 1.0));
  params.setlambdaFactor(kScaledDampingFactor);
  params.setUseFixedLambdaFactor(false);
  params.setlambdaUpperBound(kScaledDampingLimit);
  return params;
}

/// @brief Run Levenberg-Marquardt by `Termination`'s rule (`graphs._solve`).
/// @param graph The factor graph.
/// @param values The starting values.
/// @param options How the solve runs, checked.
/// @return The values and summary, or `NotConverged` if GTSAM throws.
Result<Solved> solve(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                     const Options& options) {
  try {
    const gtsam::LevenbergMarquardtParams params = solve_params(graph, values, options);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, params);
    const double initial = optimizer.error();
    double current = initial;
    Termination termination = Termination::kMaxIterations;
    int iterations = 0;
    int small = 0;  // iterations in a row each lowering the error by under small_decrease of it
    while (current > 0.0 && iterations < options.max_iterations) {
      if (options.stop != nullptr && options.stop->load(std::memory_order_relaxed)) {
        termination = Termination::kStopped;
        break;
      }
      optimizer.iterate();
      ++iterations;
      const double error = optimizer.error();
      if (error >= current && optimizer.lambda() >= params.getlambdaUpperBound()) {
        termination = Termination::kDampingLimit;
        break;
      }
      const double decrease = current - error;
      const double previous = current;
      current = error;
      if (decrease <= options.absolute_tolerance ||
          decrease <= options.relative_tolerance * previous) {
        termination = Termination::kFunction;
        break;
      }
      small = decrease < options.small_decrease * previous ? small + 1 : 0;
      if (options.small_steps > 0 && small >= options.small_steps) {
        termination = Termination::kSmallSteps;
        break;
      }
    }
    if (current == 0.0) {
      termination = Termination::kFunction;
    }
    return Solved{.values = optimizer.values(),
                  .summary = Summary{.initial_error = initial,
                                     .final_error = current,
                                     .iterations = iterations,
                                     .termination = termination,
                                     .converged = is_converged(termination)}};
  } catch (const std::exception& error) {
    return fail(ErrorKind::kNotConverged, std::string("GTSAM failed: ") + error.what());
  }
}

/// @brief Check a 4x4 pose: a last row of `[0, 0, 0, 1]` and a rotation, scaled for a
///        similarity.
/// @param pose The pose.
/// @param similarity Whether a positive scale is allowed.
/// @param name What it is, for the message.
/// @param index Its index, for the message.
/// @return Nothing, or `InvalidInput` naming it.
Result<void> check_pose(const Eigen::Matrix4d& pose, bool similarity, const char* name,
                        std::size_t index) {
  std::ostringstream where;
  where << name << "[" << index << "]: ";
  if (!pose.allFinite()) {
    return fail(ErrorKind::kInvalidInput, where.str() + "expected finite values");
  }
  if (pose.row(3) != Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)) {
    return fail(ErrorKind::kInvalidInput, where.str() + "expected a last row of [0, 0, 0, 1]");
  }
  const double determinant = pose.topLeftCorner<3, 3>().determinant();
  if (!(determinant > 0.0)) {
    return fail(ErrorKind::kInvalidInput, where.str() + "expected a positive determinant");
  }
  const double scale = similarity ? std::cbrt(determinant) : 1.0;
  if (auto rotation = so3::check_rotation(pose.topLeftCorner<3, 3>() / scale); !rotation) {
    return fail(ErrorKind::kInvalidInput, where.str() + rotation.error().message);
  }
  return {};
}

/// @brief Check a list of poses (`check_pose`), at least one.
/// @param poses The poses.
/// @param similarity Whether a positive scale is allowed.
/// @param name What they are, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_poses(std::span<const Eigen::Matrix4d> poses, bool similarity,
                         const char* name) {
  for (std::size_t k = 0; k < poses.size(); ++k) {
    if (auto checked = check_pose(poses[k], similarity, name, k); !checked) {
      return checked;
    }
  }
  return {};
}

/// @brief Check a solve's options and its starting poses, at least one.
/// @param poses The poses.
/// @param similarity Whether a positive scale is allowed.
/// @param options The options.
/// @return Nothing, or `InvalidInput`.
Result<void> check_start(std::span<const Eigen::Matrix4d> poses, bool similarity,
                         const Options& options) {
  if (auto checked = check_options(options); !checked) {
    return checked;
  }
  if (poses.empty()) {
    return fail(ErrorKind::kInvalidInput, "poses: expected N >= 1");
  }
  return check_poses(poses, similarity, "poses");
}

/// @brief Check indices against `[0, bound)`.
/// @param indices The indices.
/// @param bound The bound.
/// @param name What they are, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_indices(std::span<const std::int64_t> indices, std::size_t bound,
                           const char* name) {
  const auto bad = std::ranges::find_if(indices, [bound](std::int64_t index) {
    return index < 0 || std::cmp_greater_equal(index, bound);
  });
  if (bad != indices.end()) {
    std::ostringstream message;
    message << name << ": expected indices in [0, " << bound << ")";
    return fail(ErrorKind::kInvalidInput, message.str());
  }
  return {};
}

/// @brief Check that every value is positive and finite.
/// @param values The values.
/// @param name What they are, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_positive(std::span<const double> values, const char* name) {
  if (!std::ranges::all_of(values,
                           [](double value) { return std::isfinite(value) && value > 0.0; })) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected positive values");
  }
  return {};
}

/// @brief The sorted, distinct indices.
/// @param indices The indices.
/// @return Them, each once, ascending.
std::vector<std::int64_t> distinct(std::span<const std::int64_t> indices) {
  std::vector<std::int64_t> out(indices.begin(), indices.end());
  std::ranges::sort(out);
  out.erase(std::ranges::unique(out).begin(), out.end());
  return out;
}

/// @brief Which of `count` indices are held.
/// @param fixed The held indices, checked.
/// @param count How many there are.
/// @return A flag per index.
std::vector<bool> held_flags(std::span<const std::int64_t> fixed, std::size_t count) {
  std::vector<bool> out(count, false);
  for (const std::int64_t k : fixed) {
    out[static_cast<std::size_t>(k)] = true;
  }
  return out;
}

/// @brief Solve a pose graph whose poses are all of one kind (`optimize_pose_graph`'s solve).
///
/// The held poses are constants, not variables (g2o's fixed vertices, as ORB-SLAM holds them):
/// an edge with one held end is a factor on its free end, and the edges with both ends held,
/// whose error is a constant, one factor with a zero Jacobian (`HeldEdgesFactor`), so that the
/// solve's errors are the whole graph's. With every pose held, nothing is solved.
/// @tparam Motion `gtsam::Pose3` (rigid) or `gtsam::Similarity3`.
/// @param poses The starting poses, checked.
/// @param edges The edges, checked.
/// @param relatives Each edge's measured relative pose, checked.
/// @param fixed The poses held fixed, checked.
/// @param options How the solve runs, checked.
/// @return The solved poses, the held ones as given, or `NotConverged` if GTSAM fails.
template <typename Motion>
Result<PoseGraph> solve_pose_graph(std::span<const Eigen::Matrix4d> poses,
                                   const Eigen::Ref<const IndexPairs>& edges,
                                   std::span<const Eigen::Matrix4d> relatives,
                                   std::span<const std::int64_t> fixed, const Options& options) {
  const std::vector<bool> held = held_flags(fixed, poses.size());
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    if (!held[k]) {
      values.insert(k, to_motion<Motion>(poses[k]));
    }
  }
  const auto unit = gtsam::noiseModel::Unit::Create(
      std::is_same_v<Motion, gtsam::Pose3> ? kPoseDim : kSimilarityDim);
  std::vector<gtsam::Vector> parts;
  for (Eigen::Index e = 0; e < edges.rows(); ++e) {
    const auto first = static_cast<std::size_t>(edges(e, 0));
    const auto second = static_cast<std::size_t>(edges(e, 1));
    const gtsam::BetweenFactor<Motion> edge(
        first, second, to_motion<Motion>(relatives[static_cast<std::size_t>(e)]), unit);
    if (held[first] && held[second]) {
      parts.emplace_back(edge.evaluateError(to_motion<Motion>(poses[first]),
                                            to_motion<Motion>(poses[second]), nullptr, nullptr));
    } else if (held[first] || held[second]) {
      graph.emplace_shared<HeldBetweenFactor<Motion>>(edge, poses[held[first] ? first : second],
                                                      held[first]);
    } else {
      graph.emplace_shared<gtsam::BetweenFactor<Motion>>(edge);
    }
  }
  constexpr Eigen::Index kDimension = gtsam::traits<Motion>::dimension;
  Eigen::VectorXd residuals(static_cast<Eigen::Index>(parts.size()) * kDimension);
  for (std::size_t k = 0; k < parts.size(); ++k) {
    residuals.segment(static_cast<Eigen::Index>(k) * kDimension, kDimension) = parts[k];
  }
  // With every pose held nothing is solved: the error is the held edges', as the factor's.
  const double held_error = 0.5 * residuals.squaredNorm();
  PoseGraph out{.poses = {poses.begin(), poses.end()},
                .summary = Summary{.initial_error = held_error,
                                   .final_error = held_error,
                                   .iterations = 0,
                                   .termination = Termination::kFunction,
                                   .converged = true}};
  if (values.empty()) {
    return out;
  }
  if (residuals.size() > 0) {
    const auto free = static_cast<gtsam::Key>(std::ranges::find(held, false) - held.begin());
    graph.emplace_shared<HeldEdgesFactor<Motion>>(free, residuals);
  }
  auto solved = solve(graph, values, options);
  if (!solved) {
    return std::unexpected(solved.error());
  }
  out.summary = solved->summary;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    if (!held[k]) {
      out.poses[k] = motion_matrix(solved->values.at<Motion>(k));
    }
  }
  return out;
}

}  // namespace

const char* name(Termination termination) {
  const auto index = static_cast<std::size_t>(termination);
  return index < kTerminationNames.size() ? kTerminationNames.at(index) : "unknown";
}

bool is_converged(Termination termination) {
  return termination == Termination::kFunction || termination == Termination::kSmallSteps;
}

Result<void> check_options(const Options& options) {
  if (options.max_iterations < 1) {
    return fail_setting("max_iterations", "expected >= 1", options.max_iterations);
  }
  if (!std::isfinite(options.initial_damping) || options.initial_damping <= 0.0) {
    return fail_setting("initial_damping", "expected > 0", options.initial_damping);
  }
  if (!std::isfinite(options.damping_scale) || options.damping_scale <= 0.0) {
    return fail_setting("damping_scale", "expected > 0", options.damping_scale);
  }
  if (!std::isfinite(options.relative_tolerance) || options.relative_tolerance < 0.0) {
    return fail_setting("relative_tolerance", "expected >= 0", options.relative_tolerance);
  }
  if (!std::isfinite(options.absolute_tolerance) || options.absolute_tolerance < 0.0) {
    return fail_setting("absolute_tolerance", "expected >= 0", options.absolute_tolerance);
  }
  if (options.small_steps < 0) {
    return fail_setting("small_steps", "expected >= 0", options.small_steps);
  }
  if (!std::isfinite(options.small_decrease) || options.small_decrease <= 0.0) {
    return fail_setting("small_decrease", "expected > 0", options.small_decrease);
  }
  return {};
}

Result<ProjectionResidual> projection_residual(const Eigen::Matrix4d& pose,
                                               const Eigen::Vector3d& point,
                                               const Eigen::Ref<const Eigen::VectorXd>& measured,
                                               camera::Model model,
                                               const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                               double bf) {
  const Eigen::Index size = measured.size();
  if (size != 2 && size != 3) {
    return fail(ErrorKind::kInvalidInput, "measured: expected (u, v) or (u, v, u_r)");
  }
  const Eigen::Matrix3d rotation = pose.topLeftCorner<3, 3>();
  const Eigen::Vector3d in_camera = rotation.transpose() * (point - pose.topRightCorner<3, 1>());
  const std::optional<Projected> seen = projected(model, intrinsics, in_camera);
  if (!seen) {
    // GTSAM's own projection factors' convention: a constant error, no gradient.
    const double fx = intrinsics(camera::focal_indices(model).front());
    return ProjectionResidual{.residual = Eigen::VectorXd::Constant(size, 2.0 * fx),
                              .d_pose = Eigen::MatrixXd::Zero(size, kPoseDim),
                              .d_point = Eigen::MatrixXd::Zero(size, 3),
                              .in_front = false};
  }
  Eigen::VectorXd predicted(size);
  Eigen::MatrixXd d_predicted(size, 3);
  predicted.head<2>() = seen->pixel;
  d_predicted.topRows<2>() = seen->d_point;
  if (size == 3) {
    const double z = in_camera.z();
    predicted(2) = seen->pixel.x() - bf / z;
    d_predicted.row(2) = seen->d_point.row(0) + Eigen::RowVector3d(0.0, 0.0, bf / (z * z));
  }
  // (T Exp(xi))^-1 X = Exp(-xi) T^-1 X ~ X_c - w x X_c - v.
  Eigen::Matrix<double, 3, kPoseDim> d_camera_d_pose;
  d_camera_d_pose << so3::hat(in_camera), -Eigen::Matrix3d::Identity();
  return ProjectionResidual{.residual = predicted - measured,
                            .d_pose = d_predicted * d_camera_d_pose,
                            .d_point = d_predicted * rotation.transpose(),
                            .in_front = true};
}

SimilarityResiduals similarity_residuals(
    const Eigen::Matrix4d& similarity, const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
    const Eigen::Vector2d& pixel1, const Eigen::Vector2d& pixel2, camera::Model model,
    const Eigen::Ref<const Eigen::VectorXd>& intrinsics, bool fixed_scale) {
  const Parts parts = split(similarity);
  const int dimension = fixed_scale ? kPoseDim : kSimilarityDim;
  const Eigen::Vector3d forward = parts.scale * parts.rotation * point2 + parts.translation;
  const Eigen::Vector3d backward =
      parts.rotation.transpose() * (point1 - parts.translation) / parts.scale;
  // S Exp(xi) p ~ S (p + w x p + v + lambda p), and (S Exp(xi))^-1 q = Exp(-xi) S^-1 q.
  Eigen::Matrix<double, 3, kSimilarityDim> d_forward;
  d_forward << -so3::hat(point2), Eigen::Matrix3d::Identity(), point2;
  d_forward = parts.scale * parts.rotation * d_forward;
  Eigen::Matrix<double, 3, kSimilarityDim> d_backward;
  d_backward << so3::hat(backward), -Eigen::Matrix3d::Identity(), -backward;
  SimilarityResiduals out{.residual1 = Eigen::Vector2d::Zero(),
                          .d_similarity1 = Eigen::MatrixXd::Zero(2, dimension),
                          .residual2 = Eigen::Vector2d::Zero(),
                          .d_similarity2 = Eigen::MatrixXd::Zero(2, dimension),
                          .in_front1 = false,
                          .in_front2 = false};
  if (const std::optional<Projected> seen = projected(model, intrinsics, forward); seen) {
    out.residual1 = seen->pixel - pixel1;
    out.d_similarity1 = seen->d_point * d_forward.leftCols(dimension);
    out.in_front1 = true;
  }
  if (const std::optional<Projected> seen = projected(model, intrinsics, backward); seen) {
    out.residual2 = seen->pixel - pixel2;
    out.d_similarity2 = seen->d_point * d_backward.leftCols(dimension);
    out.in_front2 = true;
  }
  return out;
}

Result<PoseGraph> optimize_pose_graph(std::span<const Eigen::Matrix4d> poses,
                                      const Eigen::Ref<const IndexPairs>& edges,
                                      std::span<const Eigen::Matrix4d> relatives,
                                      std::span<const std::int64_t> fixed, bool similarity,
                                      const Options& options) {
  if (auto checked = check_start(poses, similarity, options); !checked) {
    return std::unexpected(checked.error());
  }
  if (static_cast<std::size_t>(edges.rows()) != relatives.size()) {
    return fail(ErrorKind::kInvalidInput, "relatives: expected one per edge");
  }
  if (auto checked = check_poses(relatives, similarity, "relatives"); !checked) {
    return std::unexpected(checked.error());
  }
  const std::span<const std::int64_t> ends(edges.data(), static_cast<std::size_t>(edges.size()));
  if (auto checked = check_indices(ends, poses.size(), "edges"); !checked) {
    return std::unexpected(checked.error());
  }
  for (Eigen::Index e = 0; e < edges.rows(); ++e) {
    if (edges(e, 0) == edges(e, 1)) {
      return fail(ErrorKind::kInvalidInput, "edges: an edge joins a pose to itself");
    }
  }
  if (auto checked = check_indices(fixed, poses.size(), "fixed"); !checked) {
    return std::unexpected(checked.error());
  }
  if (fixed.empty()) {
    return fail(ErrorKind::kInvalidInput,
                "fixed: expected at least one pose held fixed (the gauge)");
  }
  if (similarity) {
    return solve_pose_graph<gtsam::Similarity3>(poses, edges, relatives, fixed, options);
  }
  return solve_pose_graph<gtsam::Pose3>(poses, edges, relatives, fixed, options);
}

namespace {

/// @brief A refinement's matches, row for row, as `refine_similarity` takes them.
struct MatchRows {
  /// @brief Each match's point in camera 1's frame.
  Eigen::Ref<const Points> points1;
  /// @brief Each match's point in camera 2's frame.
  Eigen::Ref<const Points> points2;
  /// @brief Where image 1 observes each.
  Eigen::Ref<const Pairs> pixels1;
  /// @brief Where image 2 observes each, NaN where it does not.
  Eigen::Ref<const Pairs> pixels2;
  /// @brief Each observation's information.
  Eigen::Ref<const Pairs> information;
};

/// @brief An observation's chi2: infinite for a point behind its camera.
/// @param in_front Whether the point is in front of the camera.
/// @param information The observation's information.
/// @param residual Its residual.
/// @return The chi2.
double chi2_of(bool in_front, double information, const Eigen::Vector2d& residual) {
  return in_front ? information * residual.squaredNorm() : std::numeric_limits<double>::infinity();
}

/// @brief Refine the motion, of one kind, against its matches (`refine_similarity`'s solve).
/// @tparam Motion `gtsam::Pose3` (rigid) or `gtsam::Similarity3`.
/// @param start The starting motion, checked.
/// @param matches The matches, checked.
/// @param model The camera model.
/// @param intrinsics Its intrinsics, checked.
/// @param huber The Huber loss's threshold, or none.
/// @param options How the solve runs.
/// @return The refinement, or `NotConverged` if GTSAM fails.
template <typename Motion>
Result<SimilarityRefinement> refine(const Motion& start, const MatchRows& matches,
                                    camera::Model model,
                                    const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                                    std::optional<double> huber, const Options& options) {
  using Factor = SimilarityFactor<Motion>;
  const auto& [points1, points2, pixels1, pixels2, information] = matches;
  constexpr bool kRigid = std::is_same_v<Motion, gtsam::Pose3>;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  values.insert(0, start);
  const Eigen::VectorXd camera = intrinsics;
  for (Eigen::Index k = 0; k < points1.rows(); ++k) {
    // A match image 2 does not observe (a NaN row) counts its image-1 error alone.
    const bool seen2 = !std::isnan(pixels2(k, 0));
    for (const bool first : {true, false}) {
      if (!first && !seen2) {
        continue;
      }
      const gtsam::SharedNoiseModel weighted =
          gtsam::noiseModel::Isotropic::Precision(2, information(k, first ? 0 : 1));
      const gtsam::SharedNoiseModel noise =
          huber ? gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Huber::Create(*huber), weighted)
                : weighted;
      graph.emplace_shared<Factor>(0,
                                   typename Factor::Match{.point1 = points1.row(k).transpose(),
                                                          .point2 = points2.row(k).transpose(),
                                                          .pixel1 = pixels1.row(k).transpose(),
                                                          .pixel2 = pixels2.row(k).transpose(),
                                                          .first = first},
                                   model, camera, noise);
    }
  }
  auto solved = solve(graph, values, options);
  if (!solved) {
    return std::unexpected(solved.error());
  }
  const Eigen::Matrix4d motion = motion_matrix(solved->values.at<Motion>(0));
  Pairs chi2(points1.rows(), 2);
  for (Eigen::Index k = 0; k < points1.rows(); ++k) {
    const SimilarityResiduals both = similarity_residuals(
        motion, points1.row(k).transpose(), points2.row(k).transpose(), pixels1.row(k).transpose(),
        pixels2.row(k).transpose(), model, intrinsics, kRigid);
    chi2(k, 0) = chi2_of(both.in_front1, information(k, 0), both.residual1);
    chi2(k, 1) = std::isnan(pixels2(k, 0))
                     ? std::numeric_limits<double>::quiet_NaN()
                     : chi2_of(both.in_front2, information(k, 1), both.residual2);
  }
  return SimilarityRefinement{.similarity = motion, .chi2 = chi2, .summary = solved->summary};
}

/// @brief Check a bundle adjustment's input (`bundle_adjust`'s arguments).
/// @param poses The cameras' `T_world_cam`.
/// @param points The points.
/// @param observations The observations.
/// @param model The camera model.
/// @param intrinsics Its intrinsics.
/// @param bf The baseline times the focal length.
/// @param fixed The cameras held fixed.
/// @param options How the solve runs.
/// @return Nothing, or `InvalidInput` naming what is wrong.
Result<void> check_bundle(std::span<const Eigen::Matrix4d> poses,
                          const Eigen::Ref<const Points>& points, const Observations& observations,
                          camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics,
                          double bf, std::span<const std::int64_t> fixed, const Options& options) {
  if (auto checked = check_start(poses, false, options); !checked) {
    return checked;
  }
  if (points.rows() == 0 || !points.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "points: expected (P, 3) finite values, P >= 1");
  }
  const std::size_t count = observations.cameras.size();
  if (observations.point_ids.size() != count ||
      std::cmp_not_equal(observations.pixels.rows(), count) || observations.right.size() != count ||
      observations.information.size() != count) {
    return fail(ErrorKind::kInvalidInput,
                "observations: expected cameras, point_ids, pixels, right alike");
  }
  if (!observations.huber.empty()) {
    if (observations.huber.size() != count) {
      return fail(ErrorKind::kInvalidInput, "huber: expected one value per observation");
    }
    if (auto checked = check_positive(observations.huber, "huber"); !checked) {
      return checked;
    }
  }
  if (!observations.pixels.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "pixels: expected finite values");
  }
  if (std::ranges::any_of(observations.right, [](double value) { return std::isinf(value); })) {
    return fail(ErrorKind::kInvalidInput, "right: expected finite values or NaN");
  }
  if (auto checked = check_indices(observations.cameras, poses.size(), "cameras"); !checked) {
    return checked;
  }
  const auto point_count = static_cast<std::size_t>(points.rows());
  if (auto checked = check_indices(observations.point_ids, point_count, "point_ids"); !checked) {
    return checked;
  }
  if (auto checked = check_positive(observations.information, "information"); !checked) {
    return checked;
  }
  if (distinct(observations.point_ids).size() != point_count) {
    return fail(ErrorKind::kInvalidInput, "point_ids: every point must be observed");
  }
  const bool stereo =
      std::ranges::any_of(observations.right, [](double value) { return !std::isnan(value); });
  if (stereo && (!std::isfinite(bf) || bf <= 0.0)) {
    return fail_setting("bf", "expected > 0 with a right column", bf);
  }
  if (auto checked = check_indices(fixed, poses.size(), "fixed"); !checked) {
    return checked;
  }
  return camera::check_intrinsics(model, intrinsics);
}

/// @brief Each observation's depth: its point in front of its camera along the optical axis,
///        `R(0,2) dx + R(1,2) dy + R(2,2) dz` summed in that order, `(dx, dy, dz)` the point less
///        the camera's centre, as every language takes it (`graphs._depths`).
/// @param poses Each camera's `T_world_cam`, checked.
/// @param points Each point, checked.
/// @param observations The observations, checked.
/// @return A depth per observation.
std::vector<double> depths(std::span<const Eigen::Matrix4d> poses,
                           const Eigen::Ref<const Points>& points,
                           const Observations& observations) {
  std::vector<double> out(observations.cameras.size());
  for (std::size_t m = 0; m < out.size(); ++m) {
    const Eigen::Matrix4d& pose = poses[static_cast<std::size_t>(observations.cameras[m])];
    const Eigen::Vector3d offset =
        points.row(static_cast<Eigen::Index>(observations.point_ids[m])).transpose() -
        pose.topRightCorner<3, 1>();
    out[m] = pose(0, 2) * offset.x() + pose(1, 2) * offset.y() + pose(2, 2) * offset.z();
  }
  return out;
}

/// @brief Which observations a bundle adjustment keeps, and its depth limit.
struct Kept {
  /// @brief A flag per observation.
  std::vector<bool> flags;
  /// @brief `near` times the median depth at the start; 0 with `near` 0.
  double limit = 0.0;
};

/// @brief Which observations a bundle adjustment keeps (`graphs._kept_observations`): with `near`
///        0, every one; otherwise those whose depth at the start is positive and not below `near`
///        times the median of every observation's depth, the `floor(M/2)`-th smallest of the M
///        counting from 0.
/// @param poses Each camera's `T_world_cam`, checked.
/// @param points Each point, checked.
/// @param observations The observations, checked, at least one.
/// @param near The fraction of the median depth, non-negative.
/// @return The flags and the limit.
Kept kept_observations(std::span<const Eigen::Matrix4d> poses,
                       const Eigen::Ref<const Points>& points, const Observations& observations,
                       double near) {
  const std::size_t count = observations.cameras.size();
  Kept out{.flags = std::vector<bool>(count, true), .limit = 0.0};
  if (near == 0.0 || count == 0) {
    return out;
  }
  const std::vector<double> depth = depths(poses, points, observations);
  std::vector<double> sorted = depth;
  const auto middle = static_cast<std::ptrdiff_t>(count / 2);
  std::ranges::nth_element(sorted, sorted.begin() + middle);
  out.limit = near * sorted[static_cast<std::size_t>(middle)];
  for (std::size_t m = 0; m < count; ++m) {
    out.flags[m] = depth[m] > 0.0 && !(depth[m] < out.limit);
  }
  return out;
}

/// @brief How many kept observations' points end behind their camera or below the limit
///        (`graphs._ended_near`).
/// @param poses Each camera's `T_world_cam` at the end.
/// @param points Each point at the end.
/// @param observations The observations, checked.
/// @param kept Which were kept, and the limit.
/// @return The count.
std::int64_t ended_near(std::span<const Eigen::Matrix4d> poses,
                        const Eigen::Ref<const Points>& points, const Observations& observations,
                        const Kept& kept) {
  const std::vector<double> depth = depths(poses, points, observations);
  std::int64_t out = 0;
  for (std::size_t m = 0; m < depth.size(); ++m) {
    if (kept.flags[m] && (!(depth[m] > 0.0) || depth[m] < kept.limit)) {
      ++out;
    }
  }
  return out;
}

/// @brief The free cameras and the points a bundle adjustment's kept observations reach: those
///        it solves.
struct Reached {
  /// @brief A flag per camera.
  std::vector<bool> cameras;
  /// @brief A flag per point.
  std::vector<bool> points;
};

/// @brief What the kept observations reach.
/// @param observations The observations, checked.
/// @param kept Which are kept.
/// @param held Which cameras are held.
/// @param point_count The points.
/// @return The cameras and points reached, the held cameras never.
Reached reached(const Observations& observations, const std::vector<bool>& kept,
                const std::vector<bool>& held, std::size_t point_count) {
  Reached out{.cameras = std::vector<bool>(held.size(), false),
              .points = std::vector<bool>(point_count, false)};
  for (std::size_t m = 0; m < kept.size(); ++m) {
    if (kept[m]) {
      const auto c = static_cast<std::size_t>(observations.cameras[m]);
      out.cameras[c] = !held[c];
      out.points[static_cast<std::size_t>(observations.point_ids[m])] = true;
    }
  }
  return out;
}

/// @brief A bundle adjustment's factors: one per kept observation, on its camera and point, or
///        on its point alone through a held camera (g2o's fixed vertex, as ORB-SLAM holds one).
/// @param poses The cameras' `T_world_cam`, checked.
/// @param held Which cameras are held.
/// @param observations The observations, checked.
/// @param kept Which are kept.
/// @param sighting The camera model, its intrinsics and `bf`, each observation's `measured` set
///        here.
/// @return The graph, the points keyed from `poses.size()`.
gtsam::NonlinearFactorGraph bundle_graph(std::span<const Eigen::Matrix4d> poses,
                                         const std::vector<bool>& held,
                                         const Observations& observations,
                                         const std::vector<bool>& kept, const Sighting& sighting) {
  gtsam::NonlinearFactorGraph graph;
  const std::size_t first_point = poses.size();
  for (std::size_t m = 0; m < kept.size(); ++m) {
    if (!kept[m]) {
      continue;
    }
    const bool has_right = !std::isnan(observations.right[m]);
    Sighting one = sighting;
    one.measured.resize(has_right ? 3 : 2);
    one.measured.head<2>() = observations.pixels.row(static_cast<Eigen::Index>(m)).transpose();
    if (has_right) {
      one.measured(2) = observations.right[m];
    }
    gtsam::SharedNoiseModel noise =
        gtsam::noiseModel::Isotropic::Precision(one.measured.size(), observations.information[m]);
    if (!observations.huber.empty()) {
      noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(observations.huber[m]), noise);
    }
    const auto c = static_cast<std::size_t>(observations.cameras[m]);
    const std::size_t p = first_point + static_cast<std::size_t>(observations.point_ids[m]);
    if (held[c]) {
      graph.emplace_shared<HeldProjectionFactor>(p, poses[c], std::move(one), noise);
    } else {
      graph.emplace_shared<ProjectionFactor>(c, p, std::move(one), noise);
    }
  }
  return graph;
}

}  // namespace

Result<SimilarityRefinement> refine_similarity(
    const Eigen::Matrix4d& similarity, const Eigen::Ref<const Points>& points1,
    const Eigen::Ref<const Points>& points2, const Eigen::Ref<const Pairs>& pixels1,
    const Eigen::Ref<const Pairs>& pixels2, const Eigen::Ref<const Pairs>& information,
    camera::Model model, const Eigen::Ref<const Eigen::VectorXd>& intrinsics, bool fixed_scale,
    std::optional<double> huber, const Options& options) {
  if (auto checked = check_options(options); !checked) {
    return std::unexpected(checked.error());
  }
  if (auto checked = check_pose(similarity, !fixed_scale, "similarity", 0); !checked) {
    return std::unexpected(checked.error());
  }
  const Eigen::Index count = points1.rows();
  if (count == 0 || points2.rows() != count || pixels1.rows() != count || pixels2.rows() != count ||
      information.rows() != count) {
    return fail(ErrorKind::kInvalidInput,
                "matches: expected equal (M, 3) and (M, 2) arrays, M >= 1");
  }
  // pixels2's rows are finite, or NaN where image 2 does not observe the match.
  const auto unseen = pixels2.array().isNaN().rowwise().all();
  if (!points1.allFinite() || !points2.allFinite() || !pixels1.allFinite() ||
      !(pixels2.array().isFinite().rowwise().all() || unseen).all()) {
    return fail(ErrorKind::kInvalidInput, "matches: expected finite values");
  }
  if (!information.allFinite() || !(information.array() > 0.0).all()) {
    return fail(ErrorKind::kInvalidInput, "information: expected positive values");
  }
  if (huber && (!std::isfinite(*huber) || *huber <= 0.0)) {
    return fail_setting("huber", "expected > 0", *huber);
  }
  if (auto checked = camera::check_intrinsics(model, intrinsics); !checked) {
    return std::unexpected(checked.error());
  }
  const MatchRows matches{.points1 = points1,
                          .points2 = points2,
                          .pixels1 = pixels1,
                          .pixels2 = pixels2,
                          .information = information};
  if (fixed_scale) {
    return refine(gtsam::Pose3(similarity), matches, model, intrinsics, huber, options);
  }
  return refine(to_similarity3(similarity), matches, model, intrinsics, huber, options);
}

Result<Adjustment> bundle_adjust(std::span<const Eigen::Matrix4d> poses,
                                 const Eigen::Ref<const Points>& points,
                                 const Observations& observations, camera::Model model,
                                 const Eigen::Ref<const Eigen::VectorXd>& intrinsics, double bf,
                                 std::span<const std::int64_t> fixed, double near,
                                 const Options& options) {
  if (auto checked =
          check_bundle(poses, points, observations, model, intrinsics, bf, fixed, options);
      !checked) {
    return std::unexpected(checked.error());
  }
  if (!std::isfinite(near) || near < 0.0) {
    return fail_setting("near", "expected >= 0", near);
  }
  const auto point_count = static_cast<std::size_t>(points.rows());
  // Only the kept observations take part, and only the points and free cameras they reach are
  // solved. The held cameras are constants, not variables (g2o's fixed vertices, as ORB-SLAM
  // holds them): each of their observations is a factor on its point alone.
  const Kept kept = kept_observations(poses, points, observations, near);
  const std::vector<bool> held = held_flags(fixed, poses.size());
  const Reached solving = reached(observations, kept.flags, held, point_count);
  Adjustment out{.poses = {poses.begin(), poses.end()},
                 .points = points,
                 .summary = Summary{.initial_error = 0.0,
                                    .final_error = 0.0,
                                    .iterations = 0,
                                    .termination = Termination::kFunction,
                                    .converged = true,
                                    .left_out = std::ranges::count(kept.flags, false),
                                    .ended_near = 0}};
  if (std::cmp_equal(out.summary.left_out, kept.flags.size())) {
    return out;  // nothing left to solve: the input as given, no error counted
  }
  gtsam::Values values;
  const std::size_t first_point = poses.size();
  for (std::size_t c = 0; c < poses.size(); ++c) {
    if (solving.cameras[c]) {
      values.insert(c, gtsam::Pose3(poses[c]));
    }
  }
  for (std::size_t p = 0; p < point_count; ++p) {
    if (solving.points[p]) {
      values.insert(first_point + p,
                    gtsam::Point3(points.row(static_cast<Eigen::Index>(p)).transpose()));
    }
  }
  const gtsam::NonlinearFactorGraph graph =
      bundle_graph(poses, held, observations, kept.flags,
                   Sighting{.measured = {}, .model = model, .intrinsics = intrinsics, .bf = bf});
  auto solved = solve(graph, values, options);
  if (!solved) {
    return std::unexpected(solved.error());
  }
  const std::int64_t left_out = out.summary.left_out;
  out.summary = solved->summary;
  for (std::size_t c = 0; c < poses.size(); ++c) {
    if (solving.cameras[c]) {
      out.poses[c] = solved->values.at<gtsam::Pose3>(c).matrix();
    }
  }
  for (std::size_t p = 0; p < point_count; ++p) {
    if (solving.points[p]) {
      out.points.row(static_cast<Eigen::Index>(p)) =
          solved->values.at<gtsam::Point3>(first_point + p).transpose();
    }
  }
  out.summary.left_out = left_out;
  out.summary.ended_near = ended_near(out.poses, out.points, observations, kept);
  return out;
}

}  // namespace slam_it::foundations::graphs
