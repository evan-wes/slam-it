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
  /// @brief Positions in `obs` with a reprojection block: all but the anchors, whose error
  ///        is identically zero.
  IndexList reprojection;
  /// @brief Points parameterised as an inverse depth along their anchor's ray.
  Mask anchored;
  /// @brief Positions in `obs` of the kept observations with a depth prior.
  IndexList depth;
  /// @brief Cameras with a pose prior.
  IndexList prior_cams;
  /// @brief Depth affines that move.
  Mask affine_free;
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

/// @brief Points that cannot be placed: free points with fewer than two views or too little
///        parallax, and anchored points whose anchor was dropped.
/// @param problem The problem.
/// @param kept The kept observations.
/// @param min_parallax_deg The parallax threshold (degrees).
/// @param views Each point's number of kept observations.
/// @return Which points are degenerate.
std::vector<bool> degenerate_points(const Problem& problem, const IndexList& kept,
                                    double min_parallax_deg, const std::vector<Index>& views) {
  const Eigen::VectorXd parallax = ray_parallax(problem, kept);
  const double min_parallax = min_parallax_deg * std::numbers::pi / 180.0;
  std::vector<bool> is_kept(static_cast<std::size_t>(problem.observations.rows()), false);
  for (const Index m : kept) {
    is_kept[static_cast<std::size_t>(m)] = true;
  }
  std::vector<bool> degenerate(views.size(), false);
  for (std::size_t p = 0; p < views.size(); ++p) {
    degenerate[p] = !problem.fixed_points[p] && views[p] > 0 &&
                    (views[p] < 2 || parallax(static_cast<Index>(p)) < min_parallax);
    // An anchored point whose anchor was dropped has no ray to live on.
    const std::int64_t anchor = problem.point_anchor[p];
    degenerate[p] = degenerate[p] ||
                    (anchor >= 0 && views[p] > 0 && !is_kept[static_cast<std::size_t>(anchor)]);
  }
  return degenerate;
}

/// @brief Attach the depth and pose priors, free the observed affines, and resolve the gauge.
/// @param problem The problem.
/// @param options The settings.
/// @param layout The layout to update (its kept observations already chosen).
void attach_priors(const Problem& problem, const Options& options, Layout& layout) {
  const std::size_t n_cams = problem.poses.size();
  Mask affine_observed(n_cams, false);
  bool metric_depth = false;
  for (std::size_t i = 0; i < layout.obs.size(); ++i) {
    const Index m = layout.obs[i];
    if (std::isnan(problem.depth_prior(m)) || problem.depth_prior_weights(m) <= 0.0) {
      continue;
    }
    const auto c = static_cast<std::size_t>(problem.cam_idx[static_cast<std::size_t>(m)]);
    layout.depth.push_back(static_cast<Index>(i));
    affine_observed[c] = true;
    metric_depth = metric_depth || problem.fixed_depth_affine[c];
  }
  layout.affine_free.assign(n_cams, false);
  for (std::size_t c = 0; c < n_cams; ++c) {
    layout.affine_free[c] = !problem.fixed_depth_affine[c] && affine_observed[c];
    layout.counts.unobserved_depth_affines +=
        (!problem.fixed_depth_affine[c] && !affine_observed[c]) ? 1 : 0;
    if ((problem.pose_prior_weights.row(static_cast<Index>(c)).array() > 0.0).any()) {
      layout.prior_cams.push_back(static_cast<Index>(c));
    }
  }
  layout.counts.gauge = options.gauge;
  if (options.gauge == Gauge::kAuto) {
    if (!layout.prior_cams.empty()) {
      layout.counts.gauge = Gauge::kNone;
    } else if (metric_depth) {
      layout.counts.gauge = Gauge::kFirstPose;
    } else {
      layout.counts.gauge = Gauge::kSimilarity;
    }
  }
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
  attach_priors(problem, options, layout);
  fix_gauge(problem, observed, layout.counts.gauge, layout);
  layout.point_slot.assign(n_points, -1);
  layout.anchored.assign(n_points, false);
  std::vector<bool> is_anchor(static_cast<std::size_t>(problem.observations.rows()), false);
  for (std::size_t p = 0; p < n_points; ++p) {
    const bool used = !degenerate[p] && views[p] > 0;
    if (used && !problem.fixed_points[p]) {
      layout.point_slot[p] = layout.free_points++;
    }
    if (used && problem.point_anchor[p] >= 0) {
      layout.anchored[p] = true;
      is_anchor[static_cast<std::size_t>(problem.point_anchor[p])] = true;
    }
  }
  for (std::size_t i = 0; i < layout.obs.size(); ++i) {
    if (!is_anchor[static_cast<std::size_t>(layout.obs[i])]) {
      layout.reprojection.push_back(static_cast<Index>(i));
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

/// @brief Scatter-add a block into a matrix, accumulating repeated indices (a block's
///        columns repeat when an anchored point's host is the observing camera).
/// @param target The matrix.
/// @param rows The block's rows in `target`.
/// @param cols The block's columns in `target`.
/// @param block The block.
void scatter_add(Eigen::MatrixXd& target, const IndexList& rows, const IndexList& cols,
                 const Eigen::MatrixXd& block) {
  for (std::size_t i = 0; i < rows.size(); ++i) {
    for (std::size_t j = 0; j < cols.size(); ++j) {
      target(rows[i], cols[j]) += block(static_cast<Index>(i), static_cast<Index>(j));
    }
  }
}

/// @brief Scatter-add a vector, accumulating repeated indices.
/// @param target The vector.
/// @param rows The entries' positions in `target`.
/// @param values The values.
void scatter_add(Eigen::VectorXd& target, const IndexList& rows, const Eigen::VectorXd& values) {
  for (std::size_t i = 0; i < rows.size(); ++i) {
    target(rows[i]) += values(static_cast<Index>(i));
  }
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
    /// @brief Each free point's 3x3 Hessian block (inert identity on unused dimensions).
    std::vector<Eigen::Matrix3d> hessian_pp;
    /// @brief Each free point's gradient.
    std::vector<Eigen::Vector3d> gradient_p;
    /// @brief Each point-coupled block's coupling `J_c^T W J_p` (w x 3).
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> coupling;
    /// @brief Those blocks' camera-side columns.
    std::vector<IndexList> columns;
    /// @brief Those blocks' free-point slots.
    IndexList slots;
    /// @brief Each free point's number of variables: 3 (xyz), or 1 (an inverse depth).
    std::vector<Index> point_dims;
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
    offsets_.push_back(0);
    for (const Index dims : blocks_.point_dims) {
      offsets_.push_back(offsets_.back() + dims);
    }
  }

  /// @brief The gradient in the free coordinates: cameras, then points.
  /// @return The gradient.
  [[nodiscard]] Eigen::VectorXd gradient() const {
    const Index n_cam = reduction_.size();
    Eigen::VectorXd out(n_cam + offsets_.back());
    out.head(n_cam) = reduction_.reduce_vector(blocks_.gradient_c);
    for (std::size_t p = 0; p < blocks_.gradient_p.size(); ++p) {
      out.segment(n_cam + offsets_[p], blocks_.point_dims[p]) =
          blocks_.gradient_p[p].head(blocks_.point_dims[p]);
    }
    return out;
  }

  /// @brief The Hessian's diagonal in the free coordinates.
  /// @return The diagonal.
  [[nodiscard]] Eigen::VectorXd hessian_diagonal() const {
    const Index n_cam = reduction_.size();
    Eigen::VectorXd out(n_cam + offsets_.back());
    out.head(n_cam) = reduction_.reduce_diagonal(blocks_.hessian_cc);
    for (std::size_t p = 0; p < blocks_.hessian_pp.size(); ++p) {
      out.segment(n_cam + offsets_[p], blocks_.point_dims[p]) =
          blocks_.hessian_pp[p].diagonal().head(blocks_.point_dims[p]);
    }
    return out;
  }

  /// @brief Solve the damped normal equations by eliminating the points.
  /// @param damping Added to the Hessian's diagonal, in the free coordinates.
  /// @return The step, or nothing if a point block or the reduced system is indefinite.
  [[nodiscard]] std::optional<Eigen::VectorXd> solve(const Eigen::VectorXd& damping) const {
    const std::optional<Eliminated> eliminated = eliminate(damping);
    if (!eliminated) {
      return std::nullopt;
    }
    const std::optional<Eigen::VectorXd> step_c =
        solver_.solve(eliminated->reduced, -eliminated->rhs);
    if (!step_c) {
      return std::nullopt;
    }
    const Eigen::VectorXd full_c = reduction_.expand(*step_c);
    std::vector<Eigen::Vector3d> back = blocks_.gradient_p;
    for (std::size_t m = 0; m < blocks_.slots.size(); ++m) {
      back[static_cast<std::size_t>(blocks_.slots[m])] +=
          blocks_.coupling[m].transpose() * full_c(blocks_.columns[m]);
    }
    const Index n_cam = reduction_.size();
    Eigen::VectorXd step(n_cam + offsets_.back());
    step.head(n_cam) = *step_c;
    for (std::size_t p = 0; p < blocks_.hessian_pp.size(); ++p) {
      const Eigen::Vector3d point_step = -eliminated->inverse_pp[p] * back[p];
      step.segment(n_cam + offsets_[p], blocks_.point_dims[p]) =
          point_step.head(blocks_.point_dims[p]);
    }
    return step;
  }

  /// @brief Marginal variances of camera-side free coordinates, `diag(S^-1)`.
  /// @param positions The free coordinates wanted.
  /// @return Their variances, or infinity for all if the reduced system is singular.
  [[nodiscard]] Eigen::VectorXd camera_variances(const IndexList& positions) const {
    const auto n = static_cast<Index>(positions.size());
    const std::optional<Eliminated> undamped =
        eliminate(Eigen::VectorXd::Zero(reduction_.size() + offsets_.back()));
    if (!undamped) {
      return Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
    }
    const Eigen::LLT<Eigen::MatrixXd> factor(undamped->reduced);
    if (factor.info() != Eigen::Success) {
      return Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
    }
    const Eigen::MatrixXd inverse =
        factor.solve(Eigen::MatrixXd::Identity(undamped->reduced.rows(), undamped->reduced.cols()));
    return inverse.diagonal()(positions);
  }

 private:
  /// @brief The reduced system after eliminating the points.
  struct Eliminated {
    /// @brief The reduced camera matrix, damped.
    Eigen::MatrixXd reduced;
    /// @brief The reduced right-hand side (the reduced gradient).
    Eigen::VectorXd rhs;
    /// @brief The damped point blocks' inverses.
    std::vector<Eigen::Matrix3d> inverse_pp;
  };

  /// @brief Eliminate the points from the damped system.
  /// @param damping Added to the Hessian's diagonal, in the free coordinates.
  /// @return The reduced system, or nothing if a point block is indefinite.
  [[nodiscard]] std::optional<Eliminated> eliminate(const Eigen::VectorXd& damping) const {
    const Index n_cam = reduction_.size();
    const std::size_t n_points = blocks_.hessian_pp.size();
    std::vector<Eigen::Matrix3d> inverse_pp(n_points);
    for (std::size_t p = 0; p < n_points; ++p) {
      Eigen::Matrix3d damped = blocks_.hessian_pp[p];
      damped.diagonal().head(blocks_.point_dims[p]) +=
          damping.segment(n_cam + offsets_[p], blocks_.point_dims[p]);
      const Eigen::LLT<Eigen::Matrix3d> factor(damped);
      if (factor.info() != Eigen::Success) {
        return std::nullopt;
      }
      inverse_pp[p] = factor.solve(Eigen::Matrix3d::Identity());
    }
    const std::size_t n_blocks = blocks_.slots.size();
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> weighted(n_blocks);
    for (std::size_t m = 0; m < n_blocks; ++m) {
      weighted[m] = blocks_.coupling[m] * inverse_pp[static_cast<std::size_t>(blocks_.slots[m])];
    }
    Eigen::MatrixXd schur = blocks_.hessian_cc;
    Eigen::VectorXd rhs = blocks_.gradient_c;
    for (const std::vector<std::size_t>& on_point : by_point_) {
      for (const std::size_t a : on_point) {
        for (const std::size_t b : on_point) {
          scatter_add(schur, blocks_.columns[a], blocks_.columns[b],
                      -weighted[a] * blocks_.coupling[b].transpose());
        }
      }
    }
    for (std::size_t m = 0; m < n_blocks; ++m) {
      scatter_add(rhs, blocks_.columns[m],
                  -weighted[m] * blocks_.gradient_p[static_cast<std::size_t>(blocks_.slots[m])]);
    }
    Eigen::MatrixXd reduced = reduction_.reduce_matrix(schur);
    reduced.diagonal() += damping.head(n_cam);
    return Eliminated{.reduced = std::move(reduced),
                      .rhs = reduction_.reduce_vector(rhs),
                      .inverse_pp = std::move(inverse_pp)};
  }

  Reduction reduction_;
  Blocks blocks_;
  Solver solver_;
  std::vector<std::vector<std::size_t>> by_point_;
  IndexList offsets_;
};

/// @brief The variables: world-to-camera poses, intrinsics, points, affines, inverse depths.
struct State {
  /// @brief World-to-camera poses.
  Poses poses;
  /// @brief Intrinsics groups.
  Intrinsics intrinsics;
  /// @brief Plain 3D points (anchored rows unused).
  Points points;
  /// @brief Depth affines `(scale, shift)`.
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> depth_affine;
  /// @brief Anchored points' inverse depths (NaN for the others).
  Eigen::VectorXd inverse_depth;
};

/// @brief One residual block linearised at a state (rows already scaled by sqrt(w)).
struct LinearBlock {
  /// @brief The weighted residual.
  Eigen::VectorXd residual;
  /// @brief d residual / d(camera-side variables).
  Block camera_side;
  /// @brief The camera-side columns in the full vector.
  IndexList columns;
  /// @brief d residual / d(point variables): xyz, or (rho, 0, 0) for an anchored point (no
  ///        rows for blocks that touch no point).
  Eigen::Matrix<double, Eigen::Dynamic, 3> point;
  /// @brief The point's free slot, or -1.
  Index slot = -1;
};

/// @brief An anchored point's world position and its Jacobians at a state.
struct AnchorJacobian {
  /// @brief The point in the world frame.
  Eigen::Vector3d world;
  /// @brief d world / d rho.
  Eigen::Vector3d d_rho;
  /// @brief d world / d(host pose).
  Eigen::Matrix<double, 3, kPoseDim> d_pose;
  /// @brief d world / d(host intrinsics).
  Eigen::Matrix<double, 3, Eigen::Dynamic> d_intrinsics;
};

/// @brief Every anchored point's Jacobians at a state (empty where the point is invalid).
using Anchors = std::vector<std::optional<AnchorJacobian>>;

/// @brief Every point's world position at a state (empty where an anchored one is invalid).
using WorldPoints = std::vector<std::optional<Eigen::Vector3d>>;

/// @brief The bundle-adjustment problem as an `nls::Problem` over `State`.
class Bundle {
 public:
  /// @brief The NLS state type.
  using State = ba::State;

  /// @brief Index every block's columns, the anchored points and the free coordinates.
  /// @param problem The problem (must outlive the bundle).
  /// @param layout The layout (must outlive the bundle).
  Bundle(const Problem& problem, const Layout& layout) : problem_(&problem), layout_(&layout) {
    const auto n_cams = static_cast<Index>(problem.poses.size());
    const Index n_params = problem.intrinsics.cols();
    intrinsics_offset_ = kPoseDim * n_cams;
    affine_offset_ = intrinsics_offset_ + (n_params * problem.intrinsics.rows());
    full_size_ = affine_offset_ + (2 * n_cams);
    for (Index c = 0; c < n_cams; ++c) {
      append_range(free_, kPoseDim * c, kPoseDim, layout.pose_free[static_cast<std::size_t>(c)]);
    }
    for (Index g = 0; g < problem.intrinsics.rows(); ++g) {
      append_range(free_, intrinsics_offset_ + (n_params * g), n_params,
                   layout.intrinsics_free[static_cast<std::size_t>(g)]);
    }
    for (Index c = 0; c < n_cams; ++c) {
      append_range(free_, affine_offset_ + (2 * c), 2,
                   layout.affine_free[static_cast<std::size_t>(c)]);
    }
    if (layout.scale_pose >= 0) {
      append_range(scale_rows_, kPoseDim * at(layout.scale_pose), kPoseDim, true);
    }
    anchor_index_.assign(layout.anchored.size(), -1);
    point_dims_.assign(static_cast<std::size_t>(layout.free_points), 3);
    for (std::size_t p = 0; p < layout.anchored.size(); ++p) {
      if (!layout.anchored[p]) {
        continue;
      }
      anchor_index_[p] = static_cast<Index>(anchored_points_.size());
      anchored_points_.push_back(static_cast<Index>(p));
      if (layout.point_slot[p] >= 0) {
        point_dims_[static_cast<std::size_t>(layout.point_slot[p])] = 1;
      }
    }
  }

  /// @brief Each block's loss: 0 reprojection, 1 depth prior, 2 pose prior.
  /// @return The loss index of every block, in `squared_norms` order.
  [[nodiscard]] std::vector<std::size_t> loss_index() const {
    std::vector<std::size_t> index(layout_->reprojection.size(), 0);
    index.resize(index.size() + layout_->depth.size(), 1);
    index.resize(index.size() + layout_->prior_cams.size(), 2);
    return index;
  }

  /// @brief The starting state: anchored points placed on their rays at their depth.
  /// @return The state.
  [[nodiscard]] State start() const {
    State state{.poses = invert(problem_->poses),
                .intrinsics = problem_->intrinsics,
                .points = problem_->points,
                .depth_affine = problem_->depth_affine,
                .inverse_depth = Eigen::VectorXd::Constant(
                    problem_->points.rows(), std::numeric_limits<double>::quiet_NaN())};
    for (const Index p : anchored_points_) {
      const Eigen::Matrix4d& host = state.poses[static_cast<std::size_t>(host_of(p))];
      const Eigen::Vector3d in_host =
          (host.topLeftCorner<3, 3>() * problem_->points.row(p).transpose()) +
          host.topRightCorner<3, 1>();
      state.inverse_depth(p) = 1.0 / in_host.z();
    }
    return state;
  }

  /// @brief The free intrinsics' groups and their free coordinates.
  /// @return The free groups, and each of their parameters' position.
  [[nodiscard]] std::pair<IndexList, IndexList> intrinsics_positions() const {
    IndexList groups;
    IndexList positions;
    const Index n_params = problem_->intrinsics.cols();
    for (Index g = 0; g < problem_->intrinsics.rows(); ++g) {
      if (!layout_->intrinsics_free[static_cast<std::size_t>(g)]) {
        continue;
      }
      groups.push_back(g);
      for (Index k = 0; k < n_params; ++k) {
        const Index full = intrinsics_offset_ + (n_params * g) + k;
        positions.push_back(std::ranges::lower_bound(free_, full) - free_.begin());
      }
    }
    return {groups, positions};
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

  /// @brief Every point in the world frame (empty where an anchored point is invalid).
  /// @param state The state.
  /// @return The points.
  [[nodiscard]] WorldPoints world_points(const State& state) const {
    WorldPoints out(static_cast<std::size_t>(state.points.rows()));
    for (Index p = 0; p < state.points.rows(); ++p) {
      out[static_cast<std::size_t>(p)] = Eigen::Vector3d(state.points.row(p).transpose());
    }
    for (const Index p : anchored_points_) {
      const auto ray = anchor_ray(state, p);
      const double rho = state.inverse_depth(p);
      if (!ray || rho <= 0.0) {
        out[static_cast<std::size_t>(p)] = std::nullopt;
        continue;
      }
      const Eigen::Matrix4d& host = state.poses[static_cast<std::size_t>(host_of(p))];
      out[static_cast<std::size_t>(p)] =
          host.topLeftCorner<3, 3>().transpose() * ((*ray / rho) - host.topRightCorner<3, 1>());
    }
    return out;
  }

  /// @brief Every block's squared weighted error (inf if its point is behind or invalid).
  /// @param state The state.
  /// @return The squared norms: reprojection, then depth priors, then pose priors.
  [[nodiscard]] Eigen::VectorXd squared_norms(const State& state) const {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const WorldPoints world = world_points(state);
    const std::size_t n_obs = layout_->reprojection.size();
    const std::size_t n_depth = layout_->depth.size();
    Eigen::VectorXd out(static_cast<Index>(n_obs + n_depth + layout_->prior_cams.size()));
    Index row = 0;
    for (std::size_t i = 0; i < n_obs; ++i) {
      const auto r = reprojection(state, world, i);
      out(row++) = r ? r->squaredNorm() : kInf;
    }
    for (std::size_t d = 0; d < n_depth; ++d) {
      const auto r = depth(state, world, d);
      out(row++) = r ? *r * *r : kInf;
    }
    for (std::size_t k = 0; k < layout_->prior_cams.size(); ++k) {
      const auto r = pose_residual(state, k);
      out(row++) = r ? r->squaredNorm() : kInf;
    }
    return out;
  }

  /// @brief Form the normal equations at `state` and eliminate the points.
  /// @param state The state.
  /// @param weights The IRLS weight of every block, in `squared_norms` order.
  /// @return The Schur-complement system.
  [[nodiscard]] SchurSystem<DenseCholesky> linearize(const State& state,
                                                     const Eigen::VectorXd& weights) const {
    const auto n_free = static_cast<std::size_t>(layout_->free_points);
    SchurSystem<DenseCholesky>::Blocks blocks{
        .hessian_cc = Eigen::MatrixXd::Zero(full_size_, full_size_),
        .gradient_c = Eigen::VectorXd::Zero(full_size_),
        .hessian_pp = std::vector<Eigen::Matrix3d>(n_free, Eigen::Matrix3d::Zero()),
        .gradient_p = std::vector<Eigen::Vector3d>(n_free, Eigen::Vector3d::Zero()),
        .coupling = {},
        .columns = {},
        .slots = {},
        .point_dims = point_dims_};
    const WorldPoints world = world_points(state);
    const Anchors anchor_jacobians = anchors(state, world);
    Index row = 0;
    for (std::size_t i = 0; i < layout_->reprojection.size(); ++i) {
      accumulate(reprojection_block(state, world, anchor_jacobians, i), weights(row++), blocks);
    }
    for (std::size_t d = 0; d < layout_->depth.size(); ++d) {
      accumulate(depth_block(state, world, anchor_jacobians, d), weights(row++), blocks);
    }
    for (std::size_t k = 0; k < layout_->prior_cams.size(); ++k) {
      accumulate(pose_prior_block(state, k), weights(row++), blocks);
    }
    // An anchored point's two unused dimensions: inert, so their step is exactly zero.
    for (std::size_t s = 0; s < n_free; ++s) {
      if (point_dims_[s] == 1) {
        blocks.hessian_pp[s](1, 1) = 1.0;
        blocks.hessian_pp[s](2, 2) = 1.0;
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
        const auto& pose = state.poses[static_cast<std::size_t>(c)];
        const auto moved = se3::compose(pose, se3::exp(twist));
        out.poses[static_cast<std::size_t>(c)] = moved ? *moved : pose;
      }
    }
    const Eigen::VectorXd intrinsics_step =
        full.segment(intrinsics_offset_, affine_offset_ - intrinsics_offset_);
    out.intrinsics += Eigen::Map<const Intrinsics>(intrinsics_step.data(), state.intrinsics.rows(),
                                                   state.intrinsics.cols());
    const Eigen::VectorXd affine_step = full.tail(full_size_ - affine_offset_);
    out.depth_affine += Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>>(
        affine_step.data(), n_cams, 2);
    Index offset = red.size();
    for (std::size_t p = 0; p < layout_->point_slot.size(); ++p) {
      if (layout_->point_slot[p] < 0) {
        continue;
      }
      if (layout_->anchored[p]) {
        out.inverse_depth(static_cast<Index>(p)) += delta(offset++);
      } else {
        out.points.row(static_cast<Index>(p)) += delta.segment<3>(offset).transpose();
        offset += 3;
      }
    }
    return out;
  }

 private:
  /// @brief Append `count` consecutive indices from `start` if `when`.
  /// @param list The list.
  /// @param start The first index.
  /// @param count How many.
  /// @param when Whether to append.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (start, count), as in std::span
  static void append_range(IndexList& list, Index start, Index count, bool when) {
    for (Index k = 0; when && k < count; ++k) {
      list.push_back(start + k);
    }
  }

  /// @brief A camera's six pose columns.
  /// @param c The camera.
  /// @return The columns.
  static IndexList pose_columns(std::int64_t c) {
    IndexList cols;
    append_range(cols, kPoseDim * at(c), kPoseDim, true);
    return cols;
  }

  /// @brief Add one block's weighted contribution to the normal equations.
  /// @param block The linearised block.
  /// @param w Its IRLS weight.
  /// @param blocks The normal equations' blocks.
  static void accumulate(const LinearBlock& block, double w,
                         SchurSystem<DenseCholesky>::Blocks& blocks) {
    if (block.columns.empty()) {
      return;  // unreachable: accepted states keep every point valid
    }
    scatter_add(blocks.hessian_cc, block.columns, block.columns,
                w * block.camera_side.transpose() * block.camera_side);
    scatter_add(blocks.gradient_c, block.columns,
                w * block.camera_side.transpose() * block.residual);
    if (block.slot < 0) {
      return;
    }
    const auto s = static_cast<std::size_t>(block.slot);
    blocks.hessian_pp[s] += w * block.point.transpose() * block.point;
    blocks.gradient_p[s] += w * block.point.transpose() * block.residual;
    blocks.coupling.emplace_back(w * block.camera_side.transpose() * block.point);
    blocks.columns.push_back(block.columns);
    blocks.slots.push_back(block.slot);
  }

  /// @brief An anchored point's host camera.
  /// @param p The point.
  /// @return The camera of its anchor observation.
  [[nodiscard]] std::int64_t host_of(Index p) const {
    return problem_
        ->cam_idx[static_cast<std::size_t>(problem_->point_anchor[static_cast<std::size_t>(p)])];
  }

  /// @brief An anchored point's host intrinsics at a state.
  /// @param state The state.
  /// @param p The point.
  /// @return The intrinsics vector.
  [[nodiscard]] Eigen::VectorXd host_intrinsics(const State& state, Index p) const {
    const auto host = static_cast<std::size_t>(host_of(p));
    return state.intrinsics.row(at(problem_->intrinsics_idx[host])).transpose();
  }

  /// @brief An anchored point's ray on its host's normalised plane, `(x, y, 1)`.
  /// @param state The state.
  /// @param p The point.
  /// @return The ray, or nothing if the anchor pixel cannot be undistorted.
  [[nodiscard]] std::optional<Eigen::Vector3d> anchor_ray(const State& state, Index p) const {
    const Index a = at(problem_->point_anchor[static_cast<std::size_t>(p)]);
    const auto bearing = camera::unproject(problem_->model, host_intrinsics(state, p),
                                           problem_->observations.row(a).transpose());
    if (!bearing) {
      return std::nullopt;
    }
    return Eigen::Vector3d(*bearing / bearing->z());
  }

  /// @brief Every anchored point's Jacobians at a state.
  /// @param state The state.
  /// @param world The world points at `state`.
  /// @return One entry per anchored point (empty where invalid).
  [[nodiscard]] Anchors anchors(const State& state, const WorldPoints& world) const {
    Anchors out;
    for (const Index p : anchored_points_) {
      const auto ray = anchor_ray(state, p);
      const auto& point = world[static_cast<std::size_t>(p)];
      if (!ray || !point) {
        out.emplace_back(std::nullopt);
        continue;
      }
      const auto jacobian =
          camera::project_jacobian(problem_->model, host_intrinsics(state, p), *ray);
      if (!jacobian) {
        out.emplace_back(std::nullopt);
        continue;
      }
      const double rho = state.inverse_depth(p);
      const Eigen::Matrix3d rotation_t =
          state.poses[static_cast<std::size_t>(host_of(p))].topLeftCorner<3, 3>().transpose();
      const Index n_params = problem_->intrinsics.cols();
      const Eigen::Matrix2d d_pixel_d_ray = jacobian->leftCols(2);
      Eigen::Matrix<double, 3, Eigen::Dynamic> d_ray = Eigen::MatrixXd::Zero(3, n_params);
      d_ray.topRows(2) = -d_pixel_d_ray.inverse() * jacobian->rightCols(n_params);
      AnchorJacobian anchor{.world = *point,
                            .d_rho = -rotation_t * *ray / (rho * rho),
                            .d_pose = {},
                            .d_intrinsics = rotation_t * d_ray / rho};
      anchor.d_pose << -Eigen::Matrix3d::Identity(), so3::hat(*point);
      out.emplace_back(std::move(anchor));
    }
    return out;
  }

  /// @brief Chain a block's world-point Jacobian through an anchored point: the point's
  ///        Jacobian becomes `(d/d rho, 0, 0)` and the host's pose and intrinsics columns
  ///        are appended.
  /// @param block The block, whose `point` holds d residual / d world.
  /// @param p The block's point.
  /// @param anchor_jacobians Every anchored point's Jacobians.
  /// @return False if the anchored point is invalid.
  [[nodiscard]] bool couple(LinearBlock& block, std::int64_t p,
                            const Anchors& anchor_jacobians) const {
    const Index a = anchor_index_[static_cast<std::size_t>(p)];
    if (a < 0) {
      return true;
    }
    const auto& anchor = anchor_jacobians[static_cast<std::size_t>(a)];
    if (!anchor) {
      return false;
    }
    const Eigen::Matrix<double, Eigen::Dynamic, 3> j_world = block.point;
    const Index rows = j_world.rows();
    const Index n_params = problem_->intrinsics.cols();
    const Index width = block.camera_side.cols();
    block.camera_side.conservativeResize(rows, width + kPoseDim + n_params);
    block.camera_side.middleCols(width, kPoseDim) = j_world * anchor->d_pose;
    block.camera_side.rightCols(n_params) = j_world * anchor->d_intrinsics;
    const std::int64_t host = host_of(p);
    append_range(block.columns, kPoseDim * at(host), kPoseDim, true);
    append_range(block.columns,
                 intrinsics_offset_ +
                     (n_params * at(problem_->intrinsics_idx[static_cast<std::size_t>(host)])),
                 n_params, true);
    block.point = Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(rows, 3);
    block.point.col(0) = j_world * anchor->d_rho;
    return true;
  }

  /// @brief An observation's point in its camera's frame.
  /// @param state The state.
  /// @param point The point in the world frame.
  /// @param m The observation.
  /// @return The point in the camera frame.
  [[nodiscard]] Eigen::Vector3d in_camera(const State& state, const Eigen::Vector3d& point,
                                          Index m) const {
    const auto c = static_cast<std::size_t>(problem_->cam_idx[static_cast<std::size_t>(m)]);
    return (state.poses[c].topLeftCorner<3, 3>() * point) + state.poses[c].topRightCorner<3, 1>();
  }

  /// @brief An observation's world point.
  /// @param world The world points.
  /// @param m The observation.
  /// @return The point, or nothing if it is invalid.
  [[nodiscard]] const std::optional<Eigen::Vector3d>& point_of(const WorldPoints& world,
                                                               Index m) const {
    return world[static_cast<std::size_t>(problem_->pt_idx[static_cast<std::size_t>(m)])];
  }

  /// @brief A reprojection block's weighted error.
  /// @param state The state.
  /// @param world The world points.
  /// @param i The block.
  /// @return The residual, or nothing if the point is behind the camera or invalid.
  [[nodiscard]] std::optional<Eigen::Vector2d> reprojection(const State& state,
                                                            const WorldPoints& world,
                                                            std::size_t i) const {
    const Index m = layout_->obs[static_cast<std::size_t>(layout_->reprojection[i])];
    const auto& point = point_of(world, m);
    if (!point) {
      return std::nullopt;
    }
    const auto c = static_cast<std::size_t>(problem_->cam_idx[static_cast<std::size_t>(m)]);
    const Eigen::VectorXd params =
        state.intrinsics.row(at(problem_->intrinsics_idx[c])).transpose();
    const auto pixel = camera::project(problem_->model, params, in_camera(state, *point, m));
    if (!pixel) {
      return std::nullopt;
    }
    return std::sqrt(problem_->weights(m)) * (*pixel - problem_->observations.row(m).transpose());
  }

  /// @brief A depth prior's weighted inverse-depth error.
  /// @param state The state.
  /// @param world The world points.
  /// @param d The depth block.
  /// @return The residual, or nothing if the point is behind the camera or invalid.
  [[nodiscard]] std::optional<double> depth(const State& state, const WorldPoints& world,
                                            std::size_t d) const {
    const Index m = layout_->obs[static_cast<std::size_t>(layout_->depth[d])];
    const auto& point = point_of(world, m);
    if (!point) {
      return std::nullopt;
    }
    const double z = in_camera(state, *point, m).z();
    if (z <= camera::kMinDepth) {
      return std::nullopt;
    }
    const auto c = at(problem_->cam_idx[static_cast<std::size_t>(m)]);
    const double predicted =
        (state.depth_affine(c, 0) * problem_->depth_prior(m)) + state.depth_affine(c, 1);
    return std::sqrt(problem_->depth_prior_weights(m)) * ((1.0 / z) - predicted);
  }

  /// @brief A pose prior's unweighted error `log(prior_cam_world^-1 T_cam_world)`.
  /// @param state The state.
  /// @param k The prior.
  /// @return The error.
  [[nodiscard]] std::optional<se3::Vector6d> pose_error(const State& state, std::size_t k) const {
    const auto c = static_cast<std::size_t>(layout_->prior_cams[k]);
    const auto relative = se3::compose(problem_->pose_prior[c], state.poses[c]);
    if (!relative) {
      return std::nullopt;
    }
    const auto error = se3::log(*relative);
    return error ? std::optional<se3::Vector6d>(*error) : std::nullopt;
  }

  /// @brief A pose prior's weights, square-rooted, per tangent component.
  /// @param k The prior.
  /// @return `(sqrt w_t) x3, (sqrt w_r) x3`.
  [[nodiscard]] se3::Vector6d pose_sqrt_weights(std::size_t k) const {
    const auto c = static_cast<Index>(layout_->prior_cams[k]);
    se3::Vector6d out;
    out << se3::Vector6d::Zero();
    out.head<3>().setConstant(std::sqrt(problem_->pose_prior_weights(c, 0)));
    out.tail<3>().setConstant(std::sqrt(problem_->pose_prior_weights(c, 1)));
    return out;
  }

  /// @brief A pose prior's weighted error.
  /// @param state The state.
  /// @param k The prior.
  /// @return The residual.
  [[nodiscard]] std::optional<se3::Vector6d> pose_residual(const State& state,
                                                           std::size_t k) const {
    const auto error = pose_error(state, k);
    if (!error) {
      return std::nullopt;
    }
    return se3::Vector6d(pose_sqrt_weights(k).cwiseProduct(*error));
  }

  /// @brief Linearise a reprojection block.
  /// @param state The state.
  /// @param world The world points.
  /// @param anchor_jacobians Every anchored point's Jacobians.
  /// @param i The block.
  /// @return The block (with no columns if its point is behind or invalid).
  [[nodiscard]] LinearBlock reprojection_block(const State& state, const WorldPoints& world,
                                               const Anchors& anchor_jacobians,
                                               std::size_t i) const {
    const Index m = layout_->obs[static_cast<std::size_t>(layout_->reprojection[i])];
    const auto c = static_cast<std::size_t>(problem_->cam_idx[static_cast<std::size_t>(m)]);
    const std::int64_t p = problem_->pt_idx[static_cast<std::size_t>(m)];
    const std::int64_t g = problem_->intrinsics_idx[c];
    const Index n_params = problem_->intrinsics.cols();
    const auto& point = point_of(world, m);
    const auto residual = reprojection(state, world, i);
    if (!point || !residual) {
      return {};
    }
    const Eigen::Matrix3d rotation = state.poses[c].topLeftCorner<3, 3>();
    const Eigen::VectorXd params = state.intrinsics.row(at(g)).transpose();
    const auto jacobian =
        camera::project_jacobian(problem_->model, params, in_camera(state, *point, m));
    if (!jacobian) {
      return {};
    }
    const double scale = std::sqrt(problem_->weights(m));
    const Eigen::Matrix<double, 2, 3> j_world = scale * jacobian->leftCols<3>() * rotation;
    LinearBlock block{.residual = *residual,
                      .camera_side = Block(2, kPoseDim + n_params),
                      .columns = pose_columns(static_cast<std::int64_t>(c)),
                      .point = j_world,
                      .slot = layout_->point_slot[static_cast<std::size_t>(p)]};
    block.camera_side << j_world, -j_world * so3::hat(*point),
        scale * jacobian->rightCols(n_params);
    append_range(block.columns, intrinsics_offset_ + (n_params * at(g)), n_params, true);
    return couple(block, p, anchor_jacobians) ? block : LinearBlock{};
  }

  /// @brief Linearise a depth-prior block: `d(1/z) = -dz / z^2`.
  /// @param state The state.
  /// @param world The world points.
  /// @param anchor_jacobians Every anchored point's Jacobians.
  /// @param d The depth block.
  /// @return The block (with no columns if its point is behind or invalid).
  [[nodiscard]] LinearBlock depth_block(const State& state, const WorldPoints& world,
                                        const Anchors& anchor_jacobians, std::size_t d) const {
    const Index m = layout_->obs[static_cast<std::size_t>(layout_->depth[d])];
    const std::int64_t c = problem_->cam_idx[static_cast<std::size_t>(m)];
    const std::int64_t p = problem_->pt_idx[static_cast<std::size_t>(m)];
    const auto residual = depth(state, world, d);
    const auto& point = point_of(world, m);
    if (!residual || !point) {
      return {};
    }
    const double z = in_camera(state, *point, m).z();
    const double scale = std::sqrt(problem_->depth_prior_weights(m));
    const Eigen::RowVector3d j_world =
        (-scale / (z * z)) * state.poses[static_cast<std::size_t>(c)].row(2).head<3>();
    LinearBlock block{.residual = Eigen::VectorXd::Constant(1, *residual),
                      .camera_side = Block(1, kPoseDim + 2),
                      .columns = pose_columns(c),
                      .point = j_world,
                      .slot = layout_->point_slot[static_cast<std::size_t>(p)]};
    block.camera_side << j_world, -j_world * so3::hat(*point), -scale * problem_->depth_prior(m),
        -scale;
    append_range(block.columns, affine_offset_ + (2 * at(c)), 2, true);
    return couple(block, p, anchor_jacobians) ? block : LinearBlock{};
  }

  /// @brief Linearise a pose-prior block: `d log(E exp(delta)) = Jr^-1(e) d delta`.
  /// @param state The state.
  /// @param k The prior.
  /// @return The block (touching no point).
  [[nodiscard]] LinearBlock pose_prior_block(const State& state, std::size_t k) const {
    const auto error = pose_error(state, k);
    if (!error) {
      return {};
    }
    const auto inverse = se3::right_jacobian_inverse(*error);
    if (!inverse) {
      return {};
    }
    const se3::Vector6d weights = pose_sqrt_weights(k);
    LinearBlock block{.residual = weights.cwiseProduct(*error),
                      .camera_side = Block(weights.asDiagonal() * *inverse),
                      .columns = pose_columns(layout_->prior_cams[k]),
                      .point = Eigen::Matrix<double, Eigen::Dynamic, 3>(0, 3),
                      .slot = -1};
    return block;
  }

  const Problem* problem_;
  const Layout* layout_;
  Index intrinsics_offset_ = 0;
  Index affine_offset_ = 0;
  Index full_size_ = 0;
  IndexList free_;
  IndexList scale_rows_;
  IndexList anchored_points_;
  IndexList anchor_index_;
  std::vector<Index> point_dims_;
};

/// @brief Check the depth and pose priors' sizes and values.
/// @param problem The problem.
/// @return Nothing, or `InvalidInput` naming the offending array.
Result<void> check_priors(const Problem& problem) {
  const Index n_obs = problem.observations.rows();
  const auto n_cams = static_cast<Index>(problem.poses.size());
  const Eigen::VectorXd& prior = problem.depth_prior;
  if (prior.size() != n_obs || ((prior.array() <= 0.0) || prior.array().isInf()).any()) {
    return fail(ErrorKind::kInvalidInput, "depth_prior: expected M positive values (NaN for none)");
  }
  const Eigen::VectorXd& prior_weights = problem.depth_prior_weights;
  if (prior_weights.size() != n_obs || !prior_weights.allFinite() ||
      (prior_weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "depth_prior_weights: expected M non-negative values");
  }
  if (problem.depth_affine.rows() != n_cams || !problem.depth_affine.allFinite() ||
      std::cmp_not_equal(problem.fixed_depth_affine.size(), n_cams)) {
    return fail(ErrorKind::kInvalidInput, "depth_affine: expected C finite (scale, shift) rows");
  }
  if (std::cmp_not_equal(problem.pose_prior.size(), n_cams) ||
      problem.pose_prior_weights.rows() != n_cams || !problem.pose_prior_weights.allFinite() ||
      (problem.pose_prior_weights.array() < 0.0).any()) {
    return fail(ErrorKind::kInvalidInput, "pose_prior: expected C poses and C weight pairs");
  }
  for (const Eigen::Matrix4d& pose : problem.pose_prior) {
    if (auto checked = se3::check_transform(pose); !checked) {
      return checked;
    }
  }
  return {};
}

/// @brief Check the point anchors: -1, or one of the point's own observations.
/// @param problem The problem.
/// @return Nothing, or `InvalidInput`.
Result<void> check_anchors(const Problem& problem) {
  if (std::cmp_not_equal(problem.point_anchor.size(), problem.points.rows())) {
    return fail(ErrorKind::kInvalidInput, "point_anchor: expected N entries");
  }
  for (std::size_t p = 0; p < problem.point_anchor.size(); ++p) {
    const std::int64_t a = problem.point_anchor[p];
    if (a < -1 || a >= problem.observations.rows() ||
        (a >= 0 && std::cmp_not_equal(problem.pt_idx[static_cast<std::size_t>(a)], p))) {
      return fail(ErrorKind::kInvalidInput,
                  "point_anchor: a point's anchor must be one of its observations");
    }
  }
  return {};
}

/// @brief The free intrinsics' marginal standard deviations at a state (NaN where fixed).
/// @param bundle The problem.
/// @param state The state.
/// @param solver The settings whose per-block loss weights the blocks.
/// @param shape A matrix of the intrinsics' shape.
/// @return The deviations.
Intrinsics intrinsics_std(const Bundle& bundle, const State& state, const nls::Options& solver,
                          const Intrinsics& shape) {
  Intrinsics out =
      Intrinsics::Constant(shape.rows(), shape.cols(), std::numeric_limits<double>::quiet_NaN());
  const auto [groups, positions] = bundle.intrinsics_positions();
  if (groups.empty()) {
    return out;
  }
  const Eigen::VectorXd weights =
      nls::detail::evaluate_blocks(solver, bundle.squared_norms(state)).weights;
  const Eigen::VectorXd variances = bundle.linearize(state, weights).camera_variances(positions);
  for (std::size_t i = 0; i < groups.size(); ++i) {
    out.row(groups[i]) =
        variances.segment(static_cast<Index>(i) * shape.cols(), shape.cols()).cwiseSqrt();
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
  if (auto checked = check_priors(problem); !checked) {
    return checked;
  }
  if (auto checked = check_anchors(problem); !checked) {
    return checked;
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
  if (!std::isfinite(options.min_parallax_deg) || options.min_parallax_deg < 0.0 ||
      !std::isfinite(options.focal_std_ratio) || options.focal_std_ratio < 0.0) {
    return fail(ErrorKind::kInvalidInput,
                "min_parallax_deg, focal_std_ratio: must be non-negative and finite");
  }
  if (!options.solver.block_loss.losses.empty()) {
    return fail(ErrorKind::kInvalidInput,
                "solver.block_loss: set by bundle adjustment, not the caller");
  }
  const Layout layout = make_layout(problem, options);
  const Bundle bundle(problem, layout);
  nls::Options solver = options.solver;
  solver.block_loss = {.losses = {options.solver.loss, options.depth_loss, nls::Loss{}},
                       .index = bundle.loss_index()};
  auto solved = nls::solve(bundle, bundle.start(), solver);
  if (!solved) {
    return std::unexpected(solved.error());
  }
  const State& final_state = solved->state;
  Summary summary = layout.counts;
  summary.solve = std::move(solved->summary);
  Poses poses = invert(final_state.poses);
  const Poses start_poses = invert(problem.poses);
  for (std::size_t c = 0; c < poses.size(); ++c) {
    if (final_state.poses[c] == start_poses[c]) {
      poses[c] = problem.poses[c];  // bit-exact, not a round trip through the inverse
    }
  }
  Intrinsics std_dev = intrinsics_std(bundle, final_state, solver, final_state.intrinsics);
  for (Index g = 0; g < std_dev.rows(); ++g) {
    const bool loose = std_dev(g, 0) > options.focal_std_ratio * final_state.intrinsics(g, 0) ||
                       std_dev(g, 1) > options.focal_std_ratio * final_state.intrinsics(g, 1);
    summary.unobservable_focal += loose ? 1 : 0;
  }
  Points points = final_state.points;
  const WorldPoints world = bundle.world_points(final_state);
  for (Index p = 0; p < points.rows(); ++p) {
    const auto& point = world[static_cast<std::size_t>(p)];
    if (layout.anchored[static_cast<std::size_t>(p)] && point) {
      points.row(p) = point->transpose();
    }
  }
  return Solution{.poses = std::move(poses),
                  .intrinsics = final_state.intrinsics,
                  .points = std::move(points),
                  .inverse_depth = final_state.inverse_depth,
                  .depth_affine = final_state.depth_affine,
                  .intrinsics_std = std::move(std_dev),
                  .summary = std::move(summary)};
}

}  // namespace slamlab::foundations::ba
