/// @file
/// @brief Bundle adjustment internals: the gauge's null space and its removal.
#include "gauge.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <utility>

#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations::ba::detail {

namespace {

/// @brief The anchors' rows of `d(pinned) / d(t, omega, s)`.
/// @param anchors What is pinned.
/// @param centres Every camera's world centre.
/// @return A q x 7 matrix.
Eigen::MatrixXd rows(const Anchors& anchors, const Positions& centres) {
  Positions positions = anchors.points;
  for (const Index c : anchors.centre_cams) {
    positions.push_back(centres[static_cast<std::size_t>(c)]);
  }
  const auto n_positions = static_cast<Index>(positions.size());
  const Index n_rows =
      (3 * n_positions) + (anchors.orientation_cams.empty() ? 0 : 3) + (anchors.metric ? 1 : 0);
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(n_rows, kGenerators);
  for (Index i = 0; i < n_positions; ++i) {
    const Eigen::Vector3d& x = positions[static_cast<std::size_t>(i)];
    out.block<3, 3>(3 * i, 0).setIdentity();
    out.block<3, 3>(3 * i, 3) = -so3::hat(x);
    out.block<3, 1>(3 * i, 6) = x;
  }
  Index row = 3 * n_positions;
  if (!anchors.orientation_cams.empty()) {
    out.block<3, 3>(row, 3).setIdentity();
    row += 3;
  }
  if (anchors.metric) {
    out(row, kGenerators - 1) = 1.0;
  }
  return out;
}

/// @brief An orthonormal basis of the null space of a rank-`rank` matrix's rows.
/// @param matrix The matrix.
/// @param rank Its rank.
/// @return A cols x (cols - rank) basis.
Eigen::MatrixXd null_space(const Eigen::MatrixXd& matrix, Index rank) {
  if (matrix.rows() == 0) {
    return Eigen::MatrixXd::Identity(matrix.cols(), matrix.cols());
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullV);
  return svd.matrixV().rightCols(matrix.cols() - rank);
}

/// @brief The number of singular values above `tolerance`.
/// @param matrix The matrix.
/// @param tolerance The threshold.
/// @return The rank.
Index rank_of(const Eigen::MatrixXd& matrix, double tolerance) {
  if (matrix.size() == 0) {
    return 0;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  return (svd.singularValues().array() > tolerance).count();
}

/// @brief How N moves a camera's world centre and orientation.
/// @param centre The camera's world centre.
/// @param null The remaining gauge N.
/// @return A 6 x k matrix.
Eigen::MatrixXd pose_image(const Eigen::Vector3d& centre, const Eigen::MatrixXd& null) {
  Eigen::Matrix<double, kPoseDim, kGenerators> generator =
      Eigen::Matrix<double, kPoseDim, kGenerators>::Zero();
  generator.block<3, 3>(0, 0).setIdentity();
  generator.block<3, 3>(0, 3) = -so3::hat(centre);
  generator.block<3, 1>(0, 6) = centre;
  generator.block<3, 3>(3, 3).setIdentity();
  return generator * null;
}

/// @brief `d(centre, orientation) / d(step)` for a right perturbation of `T_cam_world`.
/// @param centre The camera's world centre.
/// @return The 6 x 6 Jacobian.
Eigen::Matrix<double, kPoseDim, kPoseDim> pose_jacobian(const Eigen::Vector3d& centre) {
  Eigen::Matrix<double, kPoseDim, kPoseDim> out = Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
  out.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
  out.block<3, 3>(0, 3) = so3::hat(centre);
  out.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();
  return out;
}

/// @brief The first index within a relative 1e-12 of the largest value, so that near-ties
///        resolve as in the Python reference despite last-bit rounding.
/// @param values The values.
/// @return The index.
Index first_largest(const Eigen::VectorXd& values) {
  const double threshold = values.maxCoeff() * (1.0 - 1e-12);
  for (Index i = 0; i < values.size(); ++i) {
    if (values(i) >= threshold) {
      return i;
    }
  }
  return 0;
}

/// @brief An orthonormal basis of a matrix's column span that depends on the span alone:
///        from its projector, repeatedly the normalised column of the largest diagonal entry,
///        deflated.
/// @param matrix The matrix.
/// @param rank Its rank.
/// @return The basis vectors.
std::vector<Eigen::VectorXd> canonical_span(const Eigen::MatrixXd& matrix, Index rank) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU);
  const Eigen::MatrixXd u = svd.matrixU().leftCols(rank);
  Eigen::MatrixXd projector = u * u.transpose();
  std::vector<Eigen::VectorXd> out;
  for (Index i = 0; i < rank; ++i) {
    const Index k = first_largest(projector.diagonal());
    const Eigen::VectorXd vector = projector.col(k) / std::sqrt(projector(k, k));
    out.push_back(vector);
    projector -= vector * vector.transpose();
  }
  return out;
}

/// @brief An orthonormal basis of the complement of `normals`: one Householder reflection
///        per normal onto its largest remaining axis, which is then dropped.
/// @param normals The vectors to complement.
/// @param size Their length.
/// @return A size x (size - normals) basis.
Eigen::MatrixXd complement(const std::vector<Eigen::VectorXd>& normals, Index size) {
  Eigen::MatrixXd basis = Eigen::MatrixXd::Identity(size, size);
  for (const Eigen::VectorXd& normal : normals) {
    const Eigen::VectorXd unit = (basis.transpose() * normal).normalized();
    const Index k = first_largest(unit.cwiseAbs());
    Eigen::VectorXd v = unit;
    v(k) += unit(k) >= 0.0 ? 1.0 : -1.0;
    const Eigen::MatrixXd reflection =
        Eigen::MatrixXd::Identity(unit.size(), unit.size()) - (2.0 * v * v.transpose() / v.dot(v));
    const Eigen::MatrixXd turned = basis * reflection;
    basis.resize(size, turned.cols() - 1);
    basis << turned.leftCols(k), turned.rightCols(turned.cols() - k - 1);
  }
  return basis;
}

}  // namespace

Eigen::MatrixXd remaining(const Anchors& anchors, const Positions& centres, double tolerance) {
  const Eigen::MatrixXd pinned = rows(anchors, centres);
  return null_space(pinned, rank_of(pinned, tolerance));
}

Gauge name(const Eigen::MatrixXd& null) {
  const Index k = null.cols();
  if (k == 0) {
    return Gauge::kNone;
  }
  if (k == kGenerators) {
    return Gauge::kSimilarity;
  }
  if (k == kPoseDim && null.row(kGenerators - 1).cwiseAbs().maxCoeff() <= kGaugeTolerance) {
    return Gauge::kFirstPose;
  }
  if (k == 1 && null.middleRows(3, 3).cwiseAbs().maxCoeff() <= kGaugeTolerance) {
    return Gauge::kScale;
  }
  return Gauge::kPartial;
}

Plan plan(const Anchors& anchors, const Positions& centres, const IndexList& candidates,
          double tolerance) {
  Eigen::MatrixXd null = remaining(anchors, centres, tolerance);
  Plan out;
  IndexList left = candidates;
  while (null.cols() > 0 && !left.empty()) {
    Index best = -1;
    Index best_rank = 0;
    double best_second = 0.0;
    for (const Index c : left) {
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
          pose_image(centres[static_cast<std::size_t>(c)], null));
      const Eigen::VectorXd& singular = svd.singularValues();
      const Index rank = (singular.array() > tolerance).count();
      double second = 0.0;
      if (rank == kPoseDim) {
        second = -static_cast<double>(c);
      } else if (rank > 0) {
        second = singular(rank - 1);
      }
      // Beat the best by a relative 1e-12, so ties go to the lowest index in both cores.
      if (rank > best_rank ||
          (rank == best_rank && second > best_second + (1e-12 * std::abs(best_second)))) {
        best = c;
        best_rank = rank;
        best_second = second;
      }
    }
    if (best_rank == 0) {
      break;
    }
    if (best_rank == kPoseDim) {
      out.fixed.push_back(best);
    } else {
      out.confined.push_back(best);
      out.ranks.push_back(best_rank);
    }
    null = null * null_space(pose_image(centres[static_cast<std::size_t>(best)], null), best_rank);
    left.erase(std::ranges::find(left, best));
  }
  return out;
}

std::vector<Eigen::MatrixXd> confinement(const Anchors& anchors, const Positions& centres,
                                         const Plan& chosen) {
  const IndexList& confined = chosen.confined;
  const IndexList& ranks = chosen.ranks;
  const Index total = std::accumulate(ranks.begin(), ranks.end(), Index{0});
  Eigen::MatrixXd null = null_space(rows(anchors, centres), kGenerators - total);
  std::vector<Eigen::MatrixXd> bases;
  for (std::size_t i = 0; i < confined.size(); ++i) {
    const Eigen::Vector3d& centre = centres[static_cast<std::size_t>(confined[i])];
    const Eigen::MatrixXd image = pose_image(centre, null);
    bases.push_back(
        complement(canonical_span(pose_jacobian(centre).transpose() * image, ranks[i]), kPoseDim));
    null = null * null_space(image, ranks[i]);
  }
  return bases;
}

}  // namespace slamlab::foundations::ba::detail
