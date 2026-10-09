/// @file
/// @brief Bundle adjustment internals: the reduced camera-side coordinates and the
///        Schur-complement normal equations.
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "common.hpp"
#include "slam_it_all/foundations/ba.hpp"

namespace slam_it_all::foundations::ba::detail {

/// @brief Maps between the full camera-side vector (6 per pose, P per group, 2 per affine) and
///        the free coordinates: the free full entries, in order, then each confined pose's
///        basis coordinates.
class Reduction {
 public:
  /// @brief Build the reduction.
  /// @param free The free full entries.
  /// @param rows The confined poses' six full entries each (or none).
  /// @param basis Their block-diagonal basis (or 0 x 0).
  /// @param full_size The full vector's length.
  Reduction(IndexList free, IndexList rows, Eigen::MatrixXd basis, Index full_size)
      : free_(std::move(free)),
        rows_(std::move(rows)),
        basis_(std::move(basis)),
        full_size_(full_size) {}

  /// @brief The full coordinates the reduced system reads, ascending: the free entries and
  ///        the confined rows. The rest (fixed poses, intrinsics and affines) it never reads.
  /// @return The coordinates.
  [[nodiscard]] IndexList kept() const {
    IndexList out = free_;
    out.insert(out.end(), rows_.begin(), rows_.end());
    std::ranges::sort(out);
    out.erase(std::ranges::unique(out).begin(), out.end());
    return out;
  }

  /// @brief This reduction with another basis for its confined rows (a state's).
  /// @param basis The basis, block-diagonal, as the constructor takes it.
  /// @return The reduction.
  [[nodiscard]] Reduction rebased(Eigen::MatrixXd basis) const {
    return {free_, rows_, std::move(basis), full_size_};
  }

  /// @brief This reduction over the positions of `kept()`: for a vector or matrix that holds
  ///        only those coordinates, in their order.
  /// @return The compacted reduction.
  [[nodiscard]] Reduction compacted() const {
    const IndexList coordinates = kept();
    const auto position = [&coordinates](Index full) {
      return static_cast<Index>(std::ranges::lower_bound(coordinates, full) - coordinates.begin());
    };
    IndexList free;
    IndexList rows;
    free.reserve(free_.size());
    rows.reserve(rows_.size());
    for (const Index c : free_) {
      free.push_back(position(c));
    }
    for (const Index c : rows_) {
      rows.push_back(position(c));
    }
    return {std::move(free), std::move(rows), basis_, static_cast<Index>(coordinates.size())};
  }

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

/// @brief Where each free point's coupling rows lie, fixed for a problem: each point's rows
///        are its distinct kept camera-side coordinates, ascending (a point's blocks share,
///        for one, their camera's intrinsics coordinates), so elimination does one product per
///        point rather than one per pair of its blocks.
struct CouplingLayout {
  /// @brief Each free point's first coupling row, and one past the last point's last row.
  IndexList offset;
  /// @brief Each coupling row's camera-side coordinate, as a position among the kept ones.
  IndexList column;
  /// @brief The most rows any point has: what elimination's scratch must hold.
  Index max_rows = 0;
};

/// @brief The normal equations with the points eliminated: an `nls::LinearSystem`. Every
///        camera-side quantity is over the kept coordinates (`Reduction::kept`) only: the fixed
///        poses', intrinsics' and affines' entries the reduced system would drop are never
///        formed.
/// @tparam Solver The reduced system solver.
template <ReducedSystemSolver Solver>
class SchurSystem {
 public:
  /// @brief The blocks of the normal equations, formed once per linearisation.
  struct Blocks {
    /// @brief The camera-side Hessian over the kept coordinates, in their order.
    Eigen::MatrixXd hessian_cc;
    /// @brief The camera-side gradient over the kept coordinates.
    Eigen::VectorXd gradient_c;
    /// @brief Each free point's 3x3 Hessian block (inert identity on unused dimensions).
    std::vector<Eigen::Matrix3d> hessian_pp;
    /// @brief Each free point's gradient.
    std::vector<Eigen::Vector3d> gradient_p;
    /// @brief Every free point's coupling `J_c^T W J_p`, its rows where `layout` puts them.
    Eigen::Matrix<double, Eigen::Dynamic, 3> coupling;
    /// @brief The coupling's rows, shared by every linearisation of the problem.
    std::shared_ptr<const CouplingLayout> layout;
    /// @brief Each free point's number of variables: 3 (xyz), or 1 (an inverse depth).
    std::vector<Index> point_dims;
  };

  /// @brief Hold the blocks.
  /// @param reduction The free camera-side coordinates, over the kept ones
  ///        (`Reduction::compacted`).
  /// @param blocks The normal equations' blocks.
  /// @param solver Factors the reduced camera system.
  SchurSystem(Reduction reduction, Blocks blocks, Solver solver)
      : reduction_(std::move(reduction)), blocks_(std::move(blocks)), solver_(std::move(solver)) {
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
    // Back-substitute: each point's step from the camera steps of its coupled coordinates.
    const Eigen::VectorXd kept_step = reduction_.expand(*step_c);
    const CouplingLayout& layout = *blocks_.layout;
    const Index n_cam = reduction_.size();
    Eigen::VectorXd step(n_cam + offsets_.back());
    step.head(n_cam) = *step_c;
    for (std::size_t p = 0; p < blocks_.hessian_pp.size(); ++p) {
      Eigen::Vector3d back = blocks_.gradient_p[p];
      for (Index r = layout.offset[p]; r < layout.offset[p + 1]; ++r) {
        back += blocks_.coupling.row(r).transpose() *
                kept_step(layout.column[static_cast<std::size_t>(r)]);
      }
      const Eigen::Vector3d point_step = -eliminated->inverse_pp[p] * back;
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
    // S = H_cc - sum_p E_p H_p^-1 E_p^T and b = g_c - sum_p E_p H_p^-1 g_p, one product per
    // point over its coupled coordinates; the upper triangle, then mirrored.
    const CouplingLayout& layout = *blocks_.layout;
    Eigen::MatrixXd schur = blocks_.hessian_cc;
    Eigen::VectorXd rhs = blocks_.gradient_c;
    Eigen::Matrix<double, Eigen::Dynamic, 3> weighted(layout.max_rows, 3);
    for (std::size_t p = 0; p < n_points; ++p) {
      const Index first = layout.offset[p];
      const Index count = layout.offset[p + 1] - first;
      if (count == 0) {
        continue;
      }
      const auto coupling = blocks_.coupling.middleRows(first, count);
      auto rows = weighted.topRows(count);
      rows.noalias() = coupling * inverse_pp[p];
      for (Index j = 0; j < count; ++j) {
        const Index cj = layout.column[static_cast<std::size_t>(first + j)];
        for (Index i = 0; i <= j; ++i) {
          schur(layout.column[static_cast<std::size_t>(first + i)], cj) -=
              rows.row(i).dot(coupling.row(j));
        }
        rhs(cj) -= rows.row(j).dot(blocks_.gradient_p[p].transpose());
      }
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
  IndexList offsets_;
};

}  // namespace slam_it_all::foundations::ba::detail
