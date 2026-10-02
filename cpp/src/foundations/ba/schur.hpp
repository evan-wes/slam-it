/// @file
/// @brief Bundle adjustment internals: the reduced camera-side coordinates and the
///        Schur-complement normal equations.
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "common.hpp"
#include "slamlab/foundations/ba.hpp"

namespace slamlab::foundations::ba::detail {

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

/// @brief Scatter-add a block into a matrix, accumulating repeated indices (a block's
///        columns repeat when an anchored point's host is the observing camera).
/// @param target The matrix.
/// @param rows The block's rows in `target`.
/// @param cols The block's columns in `target`.
/// @param block The block.
void scatter_add(Eigen::MatrixXd& target, const IndexList& rows, const IndexList& cols,
                 const Eigen::MatrixXd& block);

/// @brief Scatter-add a vector, accumulating repeated indices.
/// @param target The vector.
/// @param rows The entries' positions in `target`.
/// @param values The values.
void scatter_add(Eigen::VectorXd& target, const IndexList& rows, const Eigen::VectorXd& values);

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
    merge_couplings();
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
    for (std::size_t p = 0; p < back.size(); ++p) {
      back[p] += point_coupling_[p].transpose() * full_c(point_columns_[p]);
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
    Eigen::MatrixXd schur = blocks_.hessian_cc;
    Eigen::VectorXd rhs = blocks_.gradient_c;
    for (std::size_t p = 0; p < n_points; ++p) {
      // One product per point: S -= E_p H_p^-1 E_p^T, b -= E_p H_p^-1 g_p.
      const Eigen::Matrix<double, Eigen::Dynamic, 3> weighted = point_coupling_[p] * inverse_pp[p];
      subtract_upper(schur, point_columns_[p], weighted * point_coupling_[p].transpose());
      scatter_add(rhs, point_columns_[p], -weighted * blocks_.gradient_p[p]);
    }
    schur.triangularView<Eigen::StrictlyLower>() = schur.transpose();
    Eigen::MatrixXd reduced = reduction_.reduce_matrix(schur);
    reduced.diagonal() += damping.head(n_cam);
    return Eliminated{.reduced = std::move(reduced),
                      .rhs = reduction_.reduce_vector(rhs),
                      .inverse_pp = std::move(inverse_pp)};
  }

  Reduction reduction_;
  Blocks blocks_;
  Solver solver_;
  std::vector<IndexList> point_columns_;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> point_coupling_;

  /// @brief Subtract a symmetric block from the upper triangle of a column-major matrix.
  /// @param target The matrix (only its upper triangle is updated).
  /// @param columns The block's rows and columns in `target`, ascending and distinct.
  /// @param block The symmetric block.
  static void subtract_upper(Eigen::MatrixXd& target, const IndexList& columns,
                             const Eigen::MatrixXd& block) {
    for (std::size_t j = 0; j < columns.size(); ++j) {
      for (std::size_t i = 0; i <= j; ++i) {
        target(columns[i], columns[j]) -= block(static_cast<Index>(i), static_cast<Index>(j));
      }
    }
  }

  /// @brief Merge each point's blocks into one coupling matrix over its distinct camera-side
  ///        columns (a point's blocks share, for one, their camera's intrinsics columns), so
  ///        elimination does one product per point rather than one per pair of its blocks.
  void merge_couplings() {
    const std::size_t n_points = blocks_.hessian_pp.size();
    std::vector<std::vector<std::size_t>> by_point(n_points);
    for (std::size_t m = 0; m < blocks_.slots.size(); ++m) {
      by_point[static_cast<std::size_t>(blocks_.slots[m])].push_back(m);
    }
    point_columns_.resize(n_points);
    point_coupling_.resize(n_points);
    IndexList position(static_cast<std::size_t>(blocks_.hessian_cc.rows()), -1);
    for (std::size_t p = 0; p < n_points; ++p) {
      IndexList& columns = point_columns_[p];
      for (const std::size_t m : by_point[p]) {
        for (const Index c : blocks_.columns[m]) {
          if (position[static_cast<std::size_t>(c)] < 0) {
            position[static_cast<std::size_t>(c)] = static_cast<Index>(columns.size());
            columns.push_back(c);
          }
        }
      }
      Eigen::Matrix<double, Eigen::Dynamic, 3> coupling =
          Eigen::Matrix<double, Eigen::Dynamic, 3>::Zero(static_cast<Index>(columns.size()), 3);
      for (const std::size_t m : by_point[p]) {
        for (std::size_t i = 0; i < blocks_.columns[m].size(); ++i) {
          coupling.row(position[static_cast<std::size_t>(blocks_.columns[m][i])]) +=
              blocks_.coupling[m].row(static_cast<Index>(i));
        }
      }
      for (const Index c : columns) {
        position[static_cast<std::size_t>(c)] = -1;
      }
      // Ascending columns: the elimination's scatter then walks memory in order.
      std::vector<std::size_t> order(columns.size());
      std::ranges::iota(order, std::size_t{0});
      std::ranges::sort(order,
                        [&](std::size_t a, std::size_t b) { return columns[a] < columns[b]; });
      IndexList sorted_columns(columns.size());
      Eigen::Matrix<double, Eigen::Dynamic, 3> sorted_coupling(coupling.rows(), 3);
      for (std::size_t k = 0; k < order.size(); ++k) {
        sorted_columns[k] = columns[order[k]];
        sorted_coupling.row(static_cast<Index>(k)) = coupling.row(static_cast<Index>(order[k]));
      }
      columns = std::move(sorted_columns);
      point_coupling_[p] = std::move(sorted_coupling);
    }
    blocks_.coupling = {};  // merged; the per-block copies are no longer needed
    blocks_.columns = {};
  }
  IndexList offsets_;
};

}  // namespace slamlab::foundations::ba::detail
