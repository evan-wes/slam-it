/// @file
/// @brief Bundle adjustment internals: the blocks, Jacobians, retraction and solve.
#include "bundle.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "gauge.hpp"
#include "schur.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/nls.hpp"
#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::ba::detail {

namespace {

/// @brief Camera centres `-R^T t` of world-to-camera poses.
/// @param t_cam_world The pose.
/// @return The centre in the world frame.
Eigen::Vector3d centre(const Eigen::Matrix4d& t_cam_world) {
  return -t_cam_world.topLeftCorner<3, 3>().transpose() * t_cam_world.topRightCorner<3, 1>();
}

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

/// @brief The variables: world-to-camera poses, intrinsics, points, affines, inverse depths.
struct BundleState {
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
  using State = BundleState;

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
    for (Index k = 0; std::cmp_less(k, layout.intrinsics_free.size()); ++k) {
      append_range(free_, intrinsics_offset_ + k, 1,
                   layout.intrinsics_free[static_cast<std::size_t>(k)]);
    }
    for (Index c = 0; c < n_cams; ++c) {
      append_range(free_, affine_offset_ + (2 * c), 2,
                   layout.affine_free[static_cast<std::size_t>(c)]);
    }
    for (const Index c : layout.gauge_plan.confined) {
      append_range(confined_rows_, kPoseDim * c, kPoseDim, true);
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

  /// @brief The number of weighted residual rows: 2 per reprojection, 1 per depth prior,
  ///        and 3 per weighted half of a pose prior.
  /// @return The row count.
  [[nodiscard]] Index residual_rows() const {
    auto rows = static_cast<Index>((2 * layout_->reprojection.size()) + layout_->depth.size());
    for (const Index c : layout_->prior_cams) {
      // 3 per weighted half: a translation-only prior's rotation rows carry no information.
      rows += (problem_->pose_prior_weights(c, 0) > 0.0 ? 3 : 0) +
              (problem_->pose_prior_weights(c, 1) > 0.0 ? 3 : 0);
    }
    return rows;
  }

  /// @brief The number of free parameters (the solver's step length) at `state`.
  /// @param state The state.
  /// @return The count.
  [[nodiscard]] Index free_size(const State& state) const {
    return reduction(state).size() +
           std::accumulate(point_dims_.begin(), point_dims_.end(), Index{0});
  }

  /// @brief The free intrinsics parameters and their free coordinates.
  /// @return Each free parameter's row-major index in the intrinsics, and its position.
  [[nodiscard]] std::pair<IndexList, IndexList> intrinsics_positions() const {
    IndexList entries;
    IndexList positions;
    for (Index k = 0; std::cmp_less(k, layout_->intrinsics_free.size()); ++k) {
      if (layout_->intrinsics_free[static_cast<std::size_t>(k)]) {
        entries.push_back(k);
        const Index full = intrinsics_offset_ + k;
        positions.push_back(std::ranges::lower_bound(free_, full) - free_.begin());
      }
    }
    return {entries, positions};
  }

  /// @brief The free camera-side coordinates at `state`: each confined pose's rows map to a
  ///        basis of the steps that leave the gauge alone, rebuilt as the poses move.
  /// @param state The state.
  /// @return The reduction.
  [[nodiscard]] Reduction reduction(const State& state) const {
    Positions centres;
    for (const Eigen::Matrix4d& pose : state.poses) {
      centres.push_back(centre(pose));
    }
    const std::vector<Eigen::MatrixXd> bases =
        confinement(layout_->gauge_anchors, centres, layout_->gauge_plan);
    Index columns = 0;
    for (const Eigen::MatrixXd& block : bases) {
      columns += block.cols();
    }
    Eigen::MatrixXd basis =
        Eigen::MatrixXd::Zero(kPoseDim * static_cast<Index>(bases.size()), columns);
    Index column = 0;
    for (std::size_t i = 0; i < bases.size(); ++i) {
      basis.block(kPoseDim * static_cast<Index>(i), column, kPoseDim, bases[i].cols()) = bases[i];
      column += bases[i].cols();
    }
    return {free_, confined_rows_, basis, full_size_};
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

  /// @brief A pose prior's unweighted error `(C - C_prior, log(R_prior^T R_world_cam))`:
  ///        decoupled, so the translation half pins the world centre whatever rotation the
  ///        prior stores (a GNSS-style prior).
  /// @param state The state.
  /// @param k The prior.
  /// @return The error.
  [[nodiscard]] std::optional<se3::Vector6d> pose_error(const State& state, std::size_t k) const {
    const auto c = static_cast<std::size_t>(layout_->prior_cams[k]);
    const Eigen::Matrix4d& prior = problem_->pose_prior[c];
    const Eigen::Matrix3d r_world_cam = state.poses[c].topLeftCorner<3, 3>().transpose();
    const auto rotation_error = so3::log(prior.topLeftCorner<3, 3>().transpose() * r_world_cam);
    if (!rotation_error) {
      return std::nullopt;
    }
    se3::Vector6d error;
    error << centre(state.poses[c]) - prior.topRightCorner<3, 1>(), *rotation_error;
    return error;
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

  /// @brief Linearise a pose-prior block. Under `T_cw exp(delta)`, `delta = (rho, theta)`, the
  ///        centre moves by `-rho + hat(C) theta` and the rotation error by
  ///        `-Jr^-1(e_r) R_cw theta`.
  /// @param state The state.
  /// @param k The prior.
  /// @return The block (touching no point).
  [[nodiscard]] LinearBlock pose_prior_block(const State& state, std::size_t k) const {
    const auto error = pose_error(state, k);
    if (!error) {
      return {};
    }
    const auto inverse = so3::right_jacobian_inverse(Eigen::Vector3d(error->tail<3>()));
    if (!inverse) {
      return {};
    }
    const Eigen::Matrix4d& t_cam_world =
        state.poses[static_cast<std::size_t>(layout_->prior_cams[k])];
    Eigen::Matrix<double, kPoseDim, kPoseDim> jacobian =
        Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
    jacobian.topLeftCorner<3, 3>() = -Eigen::Matrix3d::Identity();
    jacobian.topRightCorner<3, 3>() = so3::hat(centre(t_cam_world));
    jacobian.bottomRightCorner<3, 3>() = -(*inverse) * t_cam_world.topLeftCorner<3, 3>();
    const se3::Vector6d weights = pose_sqrt_weights(k);
    LinearBlock block{.residual = weights.cwiseProduct(*error),
                      .camera_side = Block(weights.asDiagonal() * jacobian),
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
  IndexList confined_rows_;
  IndexList anchored_points_;
  IndexList anchor_index_;
  std::vector<Index> point_dims_;
};

/// @brief The a-posteriori variance factor `sum(w s) / (m - n)` at a state: the blocks'
///        squared norms under their final IRLS weights, the numerator that pairs with the
///        information `J^T W J` the deviations come from (`2 cost` under the trivial loss).
/// @param bundle The problem.
/// @param state The state.
/// @param solver The settings whose per-block loss gives the weights.
/// @return The factor, or NaN without redundancy (m <= n).
double variance_factor(const Bundle& bundle, const BundleState& state, const nls::Options& solver) {
  const Index redundancy = bundle.residual_rows() - bundle.free_size(state);
  if (redundancy <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::VectorXd squared = bundle.squared_norms(state);
  const Eigen::VectorXd weights = nls::detail::evaluate_blocks(solver, squared).weights;
  return weights.dot(squared) / static_cast<double>(redundancy);
}

/// @brief The free intrinsics' marginal standard deviations at a state (NaN where fixed).
/// @param bundle The problem.
/// @param state The state.
/// @param solver The settings whose per-block loss weights the blocks.
/// @param scaling The factor the variances are multiplied by (the variance factor, or 1).
/// @return The deviations.
Intrinsics intrinsics_std(const Bundle& bundle, const BundleState& state,
                          const nls::Options& solver, double scaling) {
  const Intrinsics& shape = state.intrinsics;
  Intrinsics out =
      Intrinsics::Constant(shape.rows(), shape.cols(), std::numeric_limits<double>::quiet_NaN());
  const auto [entries, positions] = bundle.intrinsics_positions();
  if (entries.empty()) {
    return out;
  }
  const Eigen::VectorXd weights =
      nls::detail::evaluate_blocks(solver, bundle.squared_norms(state)).weights;
  const Eigen::VectorXd variances = bundle.linearize(state, weights).camera_variances(positions);
  for (std::size_t i = 0; i < entries.size(); ++i) {
    out(entries[i] / shape.cols(), entries[i] % shape.cols()) =
        std::sqrt(scaling * variances(static_cast<Index>(i)));
  }
  return out;
}

}  // namespace

Result<Solution> adjust(const Problem& problem, const Options& options, const Layout& layout) {
  const Bundle bundle(problem, layout);
  nls::Options solver = options.solver;
  solver.block_loss = {.losses = {options.solver.loss, options.depth_loss, nls::Loss{}},
                       .index = bundle.loss_index()};
  auto solved = nls::solve(bundle, bundle.start(), solver);
  if (!solved) {
    return std::unexpected(solved.error());
  }
  const BundleState& final_state = solved->state;
  Summary summary = layout.counts;
  summary.solve = std::move(solved->summary);
  Poses poses = invert(final_state.poses);
  const Poses start_poses = invert(problem.poses);
  for (std::size_t c = 0; c < poses.size(); ++c) {
    if (final_state.poses[c] == start_poses[c]) {
      poses[c] = problem.poses[c];  // bit-exact, not a round trip through the inverse
    }
  }
  summary.variance_factor = variance_factor(bundle, final_state, solver);
  const double scaling = options.scale_deviations && std::isfinite(summary.variance_factor)
                             ? summary.variance_factor
                             : 1.0;
  Intrinsics std_dev = intrinsics_std(bundle, final_state, solver, scaling);
  for (Index g = 0; g < std_dev.rows(); ++g) {
    // Radial has one focal length; pinhole and radtan have two.
    const Index n_focal = problem.model == camera::Model::kRadial ? 1 : 2;
    bool loose = false;
    for (Index k = 0; k < n_focal; ++k) {
      loose = loose || std_dev(g, k) > options.focal_std_ratio * final_state.intrinsics(g, k);
    }
    summary.unobservable_focal += loose ? 1 : 0;
  }
  Points points = final_state.points;
  const WorldPoints world = bundle.world_points(final_state);
  for (Index p = 0; p < points.rows(); ++p) {
    const auto& point = world[static_cast<std::size_t>(p)];
    if (layout.anchored[static_cast<std::size_t>(p)]) {
      // An anchored point with no world position (rho <= 0, or a ray the lens cannot
      // invert) is a NaN row, as in the Python reference.
      points.row(p) = point
                          ? Eigen::RowVector3d(point->transpose())
                          : Eigen::RowVector3d::Constant(std::numeric_limits<double>::quiet_NaN());
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

}  // namespace slamlab::foundations::ba::detail
