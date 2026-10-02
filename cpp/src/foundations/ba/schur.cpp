/// @file
/// @brief Bundle adjustment internals: scatter-adds that accumulate repeated indices.
#include "schur.hpp"

#include <cstddef>

namespace slamlab::foundations::ba::detail {

/// @brief Scatter-add a block into a matrix, accumulating repeated indices (a block's
///        columns repeat when an anchored point's host is the observing camera).
/// @param target The matrix.
/// @param rows The block's rows in `target`.
/// @param cols The block's columns in `target`.
/// @param block The block.
void scatter_add(Eigen::MatrixXd& target, const IndexList& rows, const IndexList& cols,
                 const Eigen::MatrixXd& block) {
  for (std::size_t j = 0; j < cols.size(); ++j) {  // column-outer: the matrices are column-major
    for (std::size_t i = 0; i < rows.size(); ++i) {
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

}  // namespace slamlab::foundations::ba::detail
