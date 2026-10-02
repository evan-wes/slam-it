/// @file
/// @brief Bundle adjustment: layout, the Schur-complement system, and the NLS problem.
#include "slamlab/foundations/ba.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/nls.hpp"
#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::ba {

namespace {

using Eigen::Index;
using IndexList = std::vector<Index>;
/// @brief A camera-side Jacobian or coupling block: 2 or c rows by c or 3 columns.
using Block = Eigen::MatrixXd;

/// @brief Convert a stored index to an `Eigen::Index`.
/// @param value The index.
/// @return The same value.
Index at(std::int64_t value) { return static_cast<Index>(value); }

/// @brief Check that every entry of an index array lies in `[0, bound)`.
/// @param indices The indices.
/// @param bound The exclusive upper bound.
/// @param name The array's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_indices(const Indices& indices, Index bound, const char* name) {
  for (const std::int64_t i : indices) {
    if (i < 0 || at(i) >= bound) {
      return fail(ErrorKind::kInvalidInput,
                  std::string(name) + ": indices must lie in [0, " + std::to_string(bound) + ")");
    }
  }
  return {};
}

/// @brief Camera centres `-R^T t` of world-to-camera poses.
/// @param t_cam_world The pose.
/// @return The centre in the world frame.
Eigen::Vector3d centre(const Eigen::Matrix4d& t_cam_world) {
  return -t_cam_world.topLeftCorner<3, 3>().transpose() * t_cam_world.topRightCorner<3, 1>();
}

/// @brief An orthonormal basis of the complement of `normal`, by a Householder reflection
///        onto its largest axis (deterministic, as in the Python reference).
/// @param normal The 6-vector to complement.
/// @return A 6 x 5 basis.
Eigen::Matrix<double, kPoseDim, kPoseDim - 1> householder_complement(
    const Eigen::Matrix<double, kPoseDim, 1>& normal) {
  const Eigen::Matrix<double, kPoseDim, 1> unit = normal.normalized();
  Index k = 0;
  unit.cwiseAbs().maxCoeff(&k);
  Eigen::Matrix<double, kPoseDim, 1> v = unit;
  v(k) += unit(k) >= 0.0 ? 1.0 : -1.0;
  const Eigen::Matrix<double, kPoseDim, kPoseDim> reflection =
      Eigen::Matrix<double, kPoseDim, kPoseDim>::Identity() - (2.0 * v * v.transpose() / v.dot(v));
  Eigen::Matrix<double, kPoseDim, kPoseDim - 1> basis;
  Index column = 0;
  for (Index j = 0; j < kPoseDim; ++j) {
    if (j != k) {
      basis.col(column++) = reflection.col(j);
    }
  }
  return basis;
}

/// @brief Which observations count and which variables move, decided once from the start.
struct Layout {
  /// @brief The observations kept.
  IndexList obs;
  /// @brief Poses with all six directions free.
  Mask pose_free;
  /// @brief The pose whose step is confined, or -1.
  std::int64_t scale_pose = -1;
  /// @brief The fixed pose the scale pose's baseline is measured from, or -1.
  std::int64_t scale_reference = -1;
  /// @brief Intrinsics groups that move.
  Mask intrinsics_free;
  /// @brief Each point's index among the free points, or -1.
  IndexList point_slot;
  /// @brief The number of free points.
  Index free_points = 0;
  /// @brief The layout's counts (the solve summary is filled in later).
  Summary counts;
};

/// @brief Each point's largest angle between its first kept ray and any other (radians).
/// @param problem The problem.
/// @param obs The kept observations.
/// @return The parallax per point.
Eigen::VectorXd ray_parallax(const Problem& problem, const IndexList& obs) {
  const Index n_points = problem.points.rows();
  std::vector<Eigen::Vector3d> reference(static_cast<std::size_t>(n_points));
  std::vector<bool> seen(static_cast<std::size_t>(n_points), false);
  Eigen::VectorXd parallax = Eigen::VectorXd::Zero(n_points);
  for (const Index m : obs) {
    const auto p = static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)]);
    const Eigen::Vector3d ray =
        (problem.poses[static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)])]
             .topRightCorner<3, 1>() -
         problem.points.row(static_cast<Index>(p)).transpose())
            .normalized();
    if (!seen[p]) {
      seen[p] = true;
      reference[p] = ray;
      continue;
    }
    const double angle = std::atan2(ray.cross(reference[p]).norm(), ray.dot(reference[p]));
    parallax(static_cast<Index>(p)) = std::max(parallax(static_cast<Index>(p)), angle);
  }
  return parallax;
}

/// @brief Fix the gauge: update the free poses and record the gauge and scale poses.
/// @param problem The problem.
/// @param observed Which poses are observed.
/// @param gauge The gauge choice.
/// @param layout The layout to update.
void fix_gauge(const Problem& problem, const Mask& observed, Gauge gauge, Layout& layout) {
  if (gauge == Gauge::kNone) {
    return;
  }
  const std::size_t n_cams = problem.poses.size();
  IndexList fixed_observed;
  std::int64_t first_free = -1;
  for (std::size_t c = 0; c < n_cams; ++c) {
    if (observed[c] && problem.fixed_poses[c]) {
      fixed_observed.push_back(static_cast<Index>(c));
    } else if (observed[c] && first_free < 0) {
      first_free = static_cast<std::int64_t>(c);
    }
  }
  if (fixed_observed.empty() && first_free >= 0) {
    layout.counts.gauge_pose = first_free;
    layout.pose_free[static_cast<std::size_t>(first_free)] = false;
    fixed_observed.push_back(at(first_free));
  }
  if (gauge == Gauge::kFirstPose || fixed_observed.empty()) {
    return;
  }
  const auto pose_centre = [&](Index c) -> Eigen::Vector3d {
    return problem.poses[static_cast<std::size_t>(c)].topRightCorner<3, 1>();
  };
  const Eigen::Vector3d origin = pose_centre(fixed_observed.front());
  const double tolerance = 1e-12 * (1.0 + origin.norm());
  for (const Index c : fixed_observed) {
    if ((pose_centre(c) - origin).norm() > tolerance) {
      return;  // two fixed poses apart already pin the scale
    }
  }
  double farthest = tolerance;
  for (std::size_t c = 0; c < n_cams; ++c) {
    const double distance = (pose_centre(static_cast<Index>(c)) - origin).norm();
    if (layout.pose_free[c] && distance > farthest) {
      farthest = distance;
      layout.scale_pose = static_cast<std::int64_t>(c);
    }
  }
  if (layout.scale_pose >= 0) {
    layout.scale_reference = fixed_observed.front();
    layout.counts.scale_pose = layout.scale_pose;
    layout.pose_free[static_cast<std::size_t>(layout.scale_pose)] = false;
  }
}

/// @brief The weighted observations whose point is in front of its camera at the start.
/// @param problem The problem.
/// @param counts Incremented for each observation behind its camera.
/// @return The kept observations.
IndexList in_front(const Problem& problem, Summary& counts) {
  IndexList kept;
  for (Index m = 0; m < problem.observations.rows(); ++m) {
    if (problem.weights(m) <= 0.0) {
      continue;
    }
    const Eigen::Matrix4d& t_world_cam =
        problem.poses[static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)])];
    const Eigen::Vector3d point =
        problem.points.row(at(problem.pt_idx[static_cast<std::size_t>(m)])).transpose();
    const double depth =
        t_world_cam.topLeftCorner<3, 3>().col(2).dot(point - t_world_cam.topRightCorner<3, 1>());
    if (depth > camera::kMinDepth) {
      kept.push_back(m);
    } else {
      ++counts.dropped_observations;
    }
  }
  return kept;
}

/// @brief Free points too poorly observed to place: fewer than two views, or too little
///        parallax.
/// @param problem The problem.
/// @param kept The kept observations.
/// @param min_parallax_deg The parallax threshold (degrees).
/// @param views Each point's number of kept observations.
/// @return Which points are degenerate.
std::vector<bool> degenerate_points(const Problem& problem, const IndexList& kept,
                                    double min_parallax_deg, const std::vector<Index>& views) {
  const Eigen::VectorXd parallax = ray_parallax(problem, kept);
  const double min_parallax = min_parallax_deg * std::numbers::pi / 180.0;
  std::vector<bool> degenerate(views.size(), false);
  for (std::size_t p = 0; p < views.size(); ++p) {
    degenerate[p] = !problem.fixed_points[p] && views[p] > 0 &&
                    (views[p] < 2 || parallax(static_cast<Index>(p)) < min_parallax);
  }
  return degenerate;
}

/// @brief Drop degenerate observations, hold unobserved variables fixed, and fix the gauge.
/// @param problem The problem.
/// @param options The settings.
/// @return The layout.
Layout make_layout(const Problem& problem, const Options& options) {
  const std::size_t n_cams = problem.poses.size();
  const auto n_points = static_cast<std::size_t>(problem.points.rows());
  const auto n_groups = static_cast<std::size_t>(problem.intrinsics.rows());
  Layout layout;
  const IndexList kept = in_front(problem, layout.counts);
  std::vector<Index> views(n_points, 0);
  for (const Index m : kept) {
    ++views[static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)])];
  }
  const std::vector<bool> degenerate =
      degenerate_points(problem, kept, options.min_parallax_deg, views);
  layout.counts.dropped_points = std::count(degenerate.begin(), degenerate.end(), true);
  Mask observed(n_cams, false);
  Mask group_observed(n_groups, false);
  for (const Index m : kept) {
    const auto p = static_cast<std::size_t>(problem.pt_idx[static_cast<std::size_t>(m)]);
    if (degenerate[p]) {
      ++layout.counts.dropped_observations;
      continue;
    }
    const auto c = static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)]);
    layout.obs.push_back(m);
    observed[c] = true;
    group_observed[static_cast<std::size_t>(problem.intrinsics_idx[c])] = true;
  }
  layout.pose_free.assign(n_cams, false);
  for (std::size_t c = 0; c < n_cams; ++c) {
    layout.pose_free[c] = !problem.fixed_poses[c] && observed[c];
    layout.counts.unobserved_poses += (!problem.fixed_poses[c] && !observed[c]) ? 1 : 0;
  }
  layout.intrinsics_free.assign(n_groups, false);
  for (std::size_t g = 0; g < n_groups; ++g) {
    layout.intrinsics_free[g] = !problem.fixed_intrinsics[g] && group_observed[g];
    layout.counts.unobserved_intrinsics +=
        (!problem.fixed_intrinsics[g] && !group_observed[g]) ? 1 : 0;
  }
  fix_gauge(problem, observed, options.gauge, layout);
  layout.point_slot.assign(n_points, -1);
  for (std::size_t p = 0; p < n_points; ++p) {
    if (!problem.fixed_points[p] && !degenerate[p] && views[p] > 0) {
      layout.point_slot[p] = layout.free_points++;
    }
  }
  return layout;
}

/// @brief Maps between the full camera-side vector (6 per pose, P per group) and the free
///        coordinates: the free full entries, in order, then the scale pose's five.
class Reduction {
 public:
  /// @brief Build the reduction.
  /// @param free The free full entries.
  /// @param rows The scale pose's six full entries (or none).
  /// @param basis The scale pose's 6 x 5 basis (or 0 x 0).
  /// @param full_size The full vector's length.
  Reduction(IndexList free, IndexList rows, Eigen::MatrixXd basis, Index full_size)
      : free_(std::move(free)),
        rows_(std::move(rows)),
        basis_(std::move(basis)),
        full_size_(full_size) {}

  /// @brief The number of free coordinates.
  /// @return The size.
  [[nodiscard]] Index size() const { return static_cast<Index>(free_.size()) + basis_.cols(); }

  /// @brief Project a full vector onto the free coordinates.
  /// @param vector The full vector.
  /// @return The reduced vector.
  [[nodiscard]] Eigen::VectorXd reduce_vector(const Eigen::VectorXd& vector) const {
    Eigen::VectorXd out(size());
    const auto n_free = static_cast<Index>(free_.size());
    out.head(n_free) = vector(free_);
    out.tail(basis_.cols()) = basis_.transpose() * vector(rows_);
    return out;
  }

  /// @brief Project a full symmetric matrix onto the free coordinates, `B^T M B`.
  /// @param matrix The full matrix.
  /// @return The reduced matrix.
  [[nodiscard]] Eigen::MatrixXd reduce_matrix(const Eigen::MatrixXd& matrix) const {
    const auto n_free = static_cast<Index>(free_.size());
    Eigen::MatrixXd out(size(), size());
    out.topLeftCorner(n_free, n_free) = matrix(free_, free_);
    if (basis_.cols() > 0) {
      const Eigen::MatrixXd side = matrix(free_, rows_) * basis_;
      out.topRightCorner(n_free, basis_.cols()) = side;
      out.bottomLeftCorner(basis_.cols(), n_free) = side.transpose();
      out.bottomRightCorner(basis_.cols(), basis_.cols()) =
          basis_.transpose() * matrix(rows_, rows_) * basis_;
    }
    return out;
  }

  /// @brief The diagonal of `reduce_matrix(matrix)`.
  /// @param matrix The full matrix.
  /// @return The reduced diagonal.
  [[nodiscard]] Eigen::VectorXd reduce_diagonal(const Eigen::MatrixXd& matrix) const {
    Eigen::VectorXd out(size());
    const auto n_free = static_cast<Index>(free_.size());
    out.head(n_free) = matrix.diagonal()(free_);
    out.tail(basis_.cols()) = (basis_.transpose() * matrix(rows_, rows_) * basis_).diagonal();
    return out;
  }

  /// @brief Map free coordinates (a step) to the full vector.
  /// @param reduced The free coordinates.
  /// @return The full vector.
  [[nodiscard]] Eigen::VectorXd expand(const Eigen::VectorXd& reduced) const {
    Eigen::VectorXd full = Eigen::VectorXd::Zero(full_size_);
    const auto n_free = static_cast<Index>(free_.size());
    full(free_) = reduced.head(n_free);
    full(rows_) += basis_ * reduced.tail(basis_.cols());
    return full;
  }

 private:
  IndexList free_;
  IndexList rows_;
  Eigen::MatrixXd basis_;
  Index full_size_;
};

/// @brief Scatter-add a block into a matrix at the given rows and columns.
/// @param target The matrix.
/// @param rows The block's rows in `target`.
/// @param cols The block's columns in `target`.
/// @param block The block.
void scatter_add(Eigen::MatrixXd& target, const IndexList& rows, const IndexList& cols,
                 const Eigen::MatrixXd& block) {
  target(rows, cols) += block;
}

/// @brief The normal equations with the points eliminated: an `nls::LinearSystem`.
/// @tparam Solver The reduced system solver.
template <ReducedSystemSolver Solver>
class SchurSystem {
 public:
  /// @brief The blocks of the normal equations, formed once per linearisation.
  struct Blocks {
    /// @brief The full camera-side Hessian.
    Eigen::MatrixXd hessian_cc;
    /// @brief The full camera-side gradient.
    Eigen::VectorXd gradient_c;
    /// @brief Each free point's 3x3 Hessian block.
    std::vector<Eigen::Matrix3d> hessian_pp;
    /// @brief Each free point's gradient.
    std::vector<Eigen::Vector3d> gradient_p;
    /// @brief Each free-point observation's coupling block `J_c^T W J_p` (c x 3).
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> coupling;
    /// @brief Those observations' camera-side columns.
    std::vector<IndexList> columns;
    /// @brief Those observations' free-point slots.
    IndexList slots;
  };

  /// @brief Hold the blocks.
  /// @param reduction The free camera-side coordinates.
  /// @param blocks The normal equations' blocks.
  /// @param solver Factors the reduced camera system.
  SchurSystem(Reduction reduction, Blocks blocks, Solver solver)
      : reduction_(std::move(reduction)), blocks_(std::move(blocks)), solver_(std::move(solver)) {
    by_point_.resize(blocks_.hessian_pp.size());
    for (std::size_t m = 0; m < blocks_.slots.size(); ++m) {
      by_point_[static_cast<std::size_t>(blocks_.slots[m])].push_back(m);
    }
  }

  /// @brief The gradient in the free coordinates: cameras, then points.
  /// @return The gradient.
  [[nodiscard]] Eigen::VectorXd gradient() const {
    const Index n_cam = reduction_.size();
    Eigen::VectorXd out(n_cam + (3 * static_cast<Index>(blocks_.gradient_p.size())));
    out.head(n_cam) = reduction_.reduce_vector(blocks_.gradient_c);
    for (std::size_t p = 0; p < blocks_.gradient_p.size(); ++p) {
      out.segment<3>(n_cam + (3 * static_cast<Index>(p))) = blocks_.gradient_p[p];
    }
    return out;
  }

  /// @brief The Hessian's diagonal in the free coordinates.
  /// @return The diagonal.
  [[nodiscard]] Eigen::VectorXd hessian_diagonal() const {
    const Index n_cam = reduction_.size();
    Eigen::VectorXd out(n_cam + (3 * static_cast<Index>(blocks_.hessian_pp.size())));
    out.head(n_cam) = reduction_.reduce_diagonal(blocks_.hessian_cc);
    for (std::size_t p = 0; p < blocks_.hessian_pp.size(); ++p) {
      out.segment<3>(n_cam + (3 * static_cast<Index>(p))) = blocks_.hessian_pp[p].diagonal();
    }
    return out;
  }

  /// @brief Solve the damped normal equations by eliminating the points.
  /// @param damping Added to the Hessian's diagonal, in the free coordinates.
  /// @return The step, or nothing if a point block or the reduced system is indefinite.
  [[nodiscard]] std::optional<Eigen::VectorXd> solve(const Eigen::VectorXd& damping) const {
    const Index n_cam = reduction_.size();
    const std::size_t n_points = blocks_.hessian_pp.size();
    std::vector<Eigen::Matrix3d> inverse_pp(n_points);
    for (std::size_t p = 0; p < n_points; ++p) {
      Eigen::Matrix3d damped = blocks_.hessian_pp[p];
      damped.diagonal() += damping.segment<3>(n_cam + (3 * static_cast<Index>(p)));
      const Eigen::LLT<Eigen::Matrix3d> factor(damped);
      if (factor.info() != Eigen::Success) {
        return std::nullopt;
      }
      inverse_pp[p] = factor.solve(Eigen::Matrix3d::Identity());
    }
    const std::size_t n_obs = blocks_.slots.size();
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> weighted(n_obs);
    for (std::size_t m = 0; m < n_obs; ++m) {
      weighted[m] = blocks_.coupling[m] * inverse_pp[static_cast<std::size_t>(blocks_.slots[m])];
    }
    Eigen::MatrixXd schur = blocks_.hessian_cc;
    Eigen::VectorXd rhs = blocks_.gradient_c;
    for (const std::vector<std::size_t>& observations : by_point_) {
      for (const std::size_t a : observations) {
        for (const std::size_t b : observations) {
          scatter_add(schur, blocks_.columns[a], blocks_.columns[b],
                      -weighted[a] * blocks_.coupling[b].transpose());
        }
      }
    }
    for (std::size_t m = 0; m < n_obs; ++m) {
      rhs(blocks_.columns[m]) -=
          weighted[m] * blocks_.gradient_p[static_cast<std::size_t>(blocks_.slots[m])];
    }
    Eigen::MatrixXd reduced = reduction_.reduce_matrix(schur);
    reduced.diagonal() += damping.head(n_cam);
    const std::optional<Eigen::VectorXd> step_c =
        solver_.solve(reduced, -reduction_.reduce_vector(rhs));
    if (!step_c) {
      return std::nullopt;
    }
    const Eigen::VectorXd full_c = reduction_.expand(*step_c);
    std::vector<Eigen::Vector3d> back = blocks_.gradient_p;
    for (std::size_t m = 0; m < n_obs; ++m) {
      back[static_cast<std::size_t>(blocks_.slots[m])] +=
          blocks_.coupling[m].transpose() * full_c(blocks_.columns[m]);
    }
    Eigen::VectorXd step(n_cam + (3 * static_cast<Index>(n_points)));
    step.head(n_cam) = *step_c;
    for (std::size_t p = 0; p < n_points; ++p) {
      step.segment<3>(n_cam + (3 * static_cast<Index>(p))) = -inverse_pp[p] * back[p];
    }
    return step;
  }

 private:
  Reduction reduction_;
  Blocks blocks_;
  Solver solver_;
  std::vector<std::vector<std::size_t>> by_point_;
};

/// @brief The variables: world-to-camera poses, intrinsics groups, points.
struct State {
  /// @brief World-to-camera poses.
  Poses poses;
  /// @brief Intrinsics groups.
  Intrinsics intrinsics;
  /// @brief Points.
  Points points;
};

/// @brief The bundle-adjustment problem as an `nls::Problem` over `State`.
class Bundle {
 public:
  /// @brief The NLS state type.
  using State = ba::State;

  /// @brief Precompute each kept observation's columns.
  /// @param problem The problem (must outlive the bundle).
  /// @param layout The layout (must outlive the bundle).
  Bundle(const Problem& problem, const Layout& layout) : problem_(&problem), layout_(&layout) {
    const auto n_cams = static_cast<Index>(problem.poses.size());
    const Index n_params = problem.intrinsics.cols();
    full_size_ = (kPoseDim * n_cams) + (n_params * problem.intrinsics.rows());
    for (const Index m : layout.obs) {
      const std::int64_t c = problem.cam_idx[static_cast<std::size_t>(m)];
      const std::int64_t g = problem.intrinsics_idx[static_cast<std::size_t>(c)];
      IndexList cols;
      for (Index k = 0; k < kPoseDim; ++k) {
        cols.push_back((kPoseDim * at(c)) + k);
      }
      for (Index k = 0; k < n_params; ++k) {
        cols.push_back((kPoseDim * n_cams) + (n_params * at(g)) + k);
      }
      columns_.push_back(std::move(cols));
    }
    for (Index c = 0; c < n_cams; ++c) {
      if (layout.pose_free[static_cast<std::size_t>(c)]) {
        for (Index k = 0; k < kPoseDim; ++k) {
          free_.push_back((kPoseDim * c) + k);
        }
      }
    }
    for (Index g = 0; g < problem.intrinsics.rows(); ++g) {
      if (layout.intrinsics_free[static_cast<std::size_t>(g)]) {
        for (Index k = 0; k < n_params; ++k) {
          free_.push_back((kPoseDim * n_cams) + (n_params * g) + k);
        }
      }
    }
    if (layout.scale_pose >= 0) {
      for (Index k = 0; k < kPoseDim; ++k) {
        scale_rows_.push_back((kPoseDim * at(layout.scale_pose)) + k);
      }
    }
  }

  /// @brief The free camera-side coordinates at `state` (the scale basis moves with it).
  /// @param state The state.
  /// @return The reduction.
  [[nodiscard]] Reduction reduction(const State& state) const {
    Eigen::MatrixXd basis(0, 0);
    if (layout_->scale_pose >= 0) {
      const Eigen::Vector3d moving =
          centre(state.poses[static_cast<std::size_t>(layout_->scale_pose)]);
      const Eigen::Vector3d fixed =
          centre(state.poses[static_cast<std::size_t>(layout_->scale_reference)]);
      const Eigen::Vector3d baseline = (moving - fixed).normalized();
      Eigen::Matrix<double, kPoseDim, 1> normal;
      normal << -baseline, baseline.cross(moving);
      basis = householder_complement(normal);
    }
    return {free_, scale_rows_, basis, full_size_};
  }

  /// @brief Each kept observation's squared weighted reprojection error (inf if behind).
  /// @param state The state.
  /// @return The squared norms.
  [[nodiscard]] Eigen::VectorXd squared_norms(const State& state) const {
    Eigen::VectorXd out(static_cast<Index>(layout_->obs.size()));
    for (std::size_t i = 0; i < layout_->obs.size(); ++i) {
      const auto r = residual(state, layout_->obs[i]);
      out(static_cast<Index>(i)) = r ? r->squaredNorm() : std::numeric_limits<double>::infinity();
    }
    return out;
  }

  /// @brief Form the normal equations at `state` and eliminate the points.
  /// @param state The state.
  /// @param weights The IRLS weight of each kept observation.
  /// @return The Schur-complement system.
  [[nodiscard]] SchurSystem<DenseCholesky> linearize(const State& state,
                                                     const Eigen::VectorXd& weights) const {
    const Index n_params = problem_->intrinsics.cols();
    const Index c_dim = kPoseDim + n_params;
    SchurSystem<DenseCholesky>::Blocks blocks{
        .hessian_cc = Eigen::MatrixXd::Zero(full_size_, full_size_),
        .gradient_c = Eigen::VectorXd::Zero(full_size_),
        .hessian_pp = std::vector<Eigen::Matrix3d>(static_cast<std::size_t>(layout_->free_points),
                                                   Eigen::Matrix3d::Zero()),
        .gradient_p = std::vector<Eigen::Vector3d>(static_cast<std::size_t>(layout_->free_points),
                                                   Eigen::Vector3d::Zero()),
        .coupling = {},
        .columns = {},
        .slots = {}};
    for (std::size_t i = 0; i < layout_->obs.size(); ++i) {
      const Index m = layout_->obs[i];
      const auto c = static_cast<std::size_t>(problem_->cam_idx[static_cast<std::size_t>(m)]);
      const std::int64_t p = problem_->pt_idx[static_cast<std::size_t>(m)];
      const Eigen::Vector3d point = state.points.row(at(p)).transpose();
      const Eigen::Matrix3d rotation = state.poses[c].topLeftCorner<3, 3>();
      const Eigen::Vector3d in_camera = (rotation * point) + state.poses[c].topRightCorner<3, 1>();
      const std::int64_t g = problem_->intrinsics_idx[c];
      const Eigen::VectorXd params = state.intrinsics.row(at(g)).transpose();
      const auto jacobian = camera::project_jacobian(problem_->model, params, in_camera);
      const auto pixel = camera::project(problem_->model, params, in_camera);
      if (!jacobian || !pixel) {
        continue;  // unreachable: accepted states keep every point in front
      }
      const double scale = std::sqrt(problem_->weights(m));
      const Eigen::Vector2d r = scale * (*pixel - problem_->observations.row(m).transpose());
      const Eigen::Matrix<double, 2, 3> d_point = scale * jacobian->leftCols<3>() * rotation;
      Block jc(2, c_dim);
      jc << d_point, -d_point * so3::hat(point), scale * jacobian->rightCols(n_params);
      const double w = weights(static_cast<Index>(i));
      scatter_add(blocks.hessian_cc, columns_[i], columns_[i], w * jc.transpose() * jc);
      blocks.gradient_c(columns_[i]) += w * jc.transpose() * r;
      const Index slot = layout_->point_slot[static_cast<std::size_t>(p)];
      if (slot >= 0) {
        const auto s = static_cast<std::size_t>(slot);
        blocks.hessian_pp[s] += w * d_point.transpose() * d_point;
        blocks.gradient_p[s] += w * d_point.transpose() * r;
        blocks.coupling.emplace_back(w * jc.transpose() * d_point);
        blocks.columns.push_back(columns_[i]);
        blocks.slots.push_back(slot);
      }
    }
    return {reduction(state), std::move(blocks), DenseCholesky{}};
  }

  /// @brief Apply a step in the free coordinates.
  /// @param state The state.
  /// @param delta The step.
  /// @return The moved state.
  [[nodiscard]] State retract(const State& state, const Eigen::VectorXd& delta) const {
    const Reduction red = reduction(state);
    const Eigen::VectorXd full = red.expand(delta.head(red.size()));
    State out = state;
    const auto n_cams = static_cast<Index>(state.poses.size());
    for (Index c = 0; c < n_cams; ++c) {
      const se3::Vector6d twist = full.segment<kPoseDim>(kPoseDim * c);
      if (!twist.isZero(0.0)) {
        const auto moved = se3::compose(state.poses[static_cast<std::size_t>(c)], se3::exp(twist));
        out.poses[static_cast<std::size_t>(c)] =
            moved ? *moved : state.poses[static_cast<std::size_t>(c)];
      }
    }
    const Eigen::VectorXd intrinsics_step = full.tail(full_size_ - (kPoseDim * n_cams));
    out.intrinsics += Eigen::Map<const Intrinsics>(intrinsics_step.data(), state.intrinsics.rows(),
                                                   state.intrinsics.cols());
    for (std::size_t p = 0; p < layout_->point_slot.size(); ++p) {
      const Index slot = layout_->point_slot[p];
      if (slot >= 0) {
        out.points.row(static_cast<Index>(p)) +=
            delta.segment<3>(red.size() + (3 * slot)).transpose();
      }
    }
    return out;
  }

 private:
  /// @brief One observation's weighted reprojection error.
  /// @param state The state.
  /// @param m The observation.
  /// @return The residual, or nothing if the point is behind the camera.
  [[nodiscard]] std::optional<Eigen::Vector2d> residual(const State& state, Index m) const {
    const auto c = static_cast<std::size_t>(problem_->cam_idx[static_cast<std::size_t>(m)]);
    const Eigen::Vector3d point =
        state.points.row(at(problem_->pt_idx[static_cast<std::size_t>(m)])).transpose();
    const Eigen::Vector3d in_camera =
        (state.poses[c].topLeftCorner<3, 3>() * point) + state.poses[c].topRightCorner<3, 1>();
    const Eigen::VectorXd params =
        state.intrinsics.row(at(problem_->intrinsics_idx[c])).transpose();
    const auto pixel = camera::project(problem_->model, params, in_camera);
    if (!pixel) {
      return std::nullopt;
    }
    return std::sqrt(problem_->weights(m)) * (*pixel - problem_->observations.row(m).transpose());
  }

  const Problem* problem_;
  const Layout* layout_;
  Index full_size_ = 0;
  std::vector<IndexList> columns_;
  IndexList free_;
  IndexList scale_rows_;
};

/// @brief Invert every pose (world-to-camera and back).
/// @param poses The poses (already checked).
/// @return The inverses.
Poses invert(const Poses& poses) {
  Poses out;
  out.reserve(poses.size());
  for (const Eigen::Matrix4d& pose : poses) {
    Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
    inv.topLeftCorner<3, 3>() = pose.topLeftCorner<3, 3>().transpose();
    inv.topRightCorner<3, 1>() = -inv.topLeftCorner<3, 3>() * pose.topRightCorner<3, 1>();
    out.push_back(inv);
  }
  return out;
}

}  // namespace

std::optional<Eigen::VectorXd> DenseCholesky::solve(const Eigen::MatrixXd& matrix,
                                                    const Eigen::VectorXd& rhs) {
  const Eigen::LLT<Eigen::MatrixXd> factor(matrix);
  if (factor.info() != Eigen::Success) {
    return std::nullopt;
  }
  return Eigen::VectorXd(factor.solve(rhs));
}

Result<void> check_problem(const Problem& problem) {
  const std::size_t n_cams = problem.poses.size();
  const auto n_obs = static_cast<std::size_t>(problem.observations.rows());
  const Index n_groups = problem.intrinsics.rows();
  if (n_groups == 0) {
    return fail(ErrorKind::kInvalidInput, "intrinsics: expected at least one group");
  }
  for (Index g = 0; g < n_groups; ++g) {
    if (auto checked =
            camera::check_intrinsics(problem.model, problem.intrinsics.row(g).transpose());
        !checked) {
      return checked;
    }
  }
  for (const Eigen::Matrix4d& pose : problem.poses) {
    if (auto checked = se3::check_transform(pose); !checked) {
      return checked;
    }
  }
  if (!problem.points.allFinite() || !problem.observations.allFinite()) {
    return fail(ErrorKind::kInvalidInput, "points, observations: expected finite values");
  }
  if (std::cmp_not_equal(problem.weights.size(), n_obs) || !problem.weights.allFinite() ||
      (problem.weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "weights: expected M non-negative values");
  }
  if (problem.fixed_poses.size() != n_cams ||
      problem.fixed_intrinsics.size() != static_cast<std::size_t>(n_groups) ||
      problem.fixed_points.size() != static_cast<std::size_t>(problem.points.rows())) {
    return fail(ErrorKind::kInvalidInput, "fixed masks: wrong length");
  }
  if (problem.cam_idx.size() != n_obs || problem.pt_idx.size() != n_obs ||
      problem.intrinsics_idx.size() != n_cams) {
    return fail(ErrorKind::kInvalidInput, "cam_idx, pt_idx, intrinsics_idx: wrong length");
  }
  if (auto checked = check_indices(problem.cam_idx, static_cast<Index>(n_cams), "cam_idx");
      !checked) {
    return checked;
  }
  if (auto checked = check_indices(problem.pt_idx, problem.points.rows(), "pt_idx"); !checked) {
    return checked;
  }
  return check_indices(problem.intrinsics_idx, n_groups, "intrinsics_idx");
}

Result<Solution> solve(const Problem& problem, const Options& options) {
  if (auto checked = check_problem(problem); !checked) {
    return std::unexpected(checked.error());
  }
  if (!std::isfinite(options.min_parallax_deg) || options.min_parallax_deg < 0.0) {
    return fail(ErrorKind::kInvalidInput, "min_parallax_deg: must be non-negative and finite");
  }
  const Layout layout = make_layout(problem, options);
  const Bundle bundle(problem, layout);
  State start{
      .poses = invert(problem.poses), .intrinsics = problem.intrinsics, .points = problem.points};
  auto solved = nls::solve(bundle, std::move(start), options.solver);
  if (!solved) {
    return std::unexpected(solved.error());
  }
  Summary summary = layout.counts;
  summary.solve = std::move(solved->summary);
  Poses poses = invert(solved->state.poses);
  const Poses start_poses = invert(problem.poses);
  for (std::size_t c = 0; c < poses.size(); ++c) {
    if (solved->state.poses[c] == start_poses[c]) {
      poses[c] = problem.poses[c];  // bit-exact, not a round trip through the inverse
    }
  }
  return Solution{.poses = std::move(poses),
                  .intrinsics = std::move(solved->state.intrinsics),
                  .points = std::move(solved->state.points),
                  .summary = std::move(summary)};
}

}  // namespace slamlab::foundations::ba
