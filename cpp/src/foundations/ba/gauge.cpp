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

/// @brief The anchors' rows of `d(pinned) / d(t, omega, s)`, in the anchors' frame.
/// @param anchors What is pinned.
/// @param centres Every camera's world centre.
/// @return A q x 7 matrix.
Eigen::MatrixXd rows(const Anchors& anchors, const Positions& centres) {
  Positions positions;
  for (const Eigen::Vector3d& point : anchors.points) {
    positions.push_back(in_frame(anchors.frame, point));
  }
  for (const Index c : anchors.centre_cams) {
    positions.push_back(in_frame(anchors.frame, centres[static_cast<std::size_t>(c)]));
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

/// @brief A matrix's singular values, largest first (none for an empty matrix).
/// @param matrix The matrix.
/// @return The values.
Eigen::VectorXd singular_values(const Eigen::MatrixXd& matrix) {
  if (matrix.size() == 0) {
    return Eigen::VectorXd(0);
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  return svd.singularValues();
}

/// @brief The number of singular values above `kGaugeTolerance`.
/// @param singular The singular values.
/// @return The rank.
Index rank_of(const Eigen::VectorXd& singular) {
  return (singular.array() > kGaugeTolerance).count();
}

/// @brief How N moves a camera's centre and orientation, in the frame.
/// @param centre The camera's centre in the frame.
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

/// @brief `d(centre, orientation) / d(step)` for a right perturbation of `T_cam_world`, in
///        world units.
/// @param centre The camera's world centre.
/// @return The 6 x 6 Jacobian.
Eigen::Matrix<double, kPoseDim, kPoseDim> pose_jacobian(const Eigen::Vector3d& centre) {
  Eigen::Matrix<double, kPoseDim, kPoseDim> out = Eigen::Matrix<double, kPoseDim, kPoseDim>::Zero();
  out.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
  out.block<3, 3>(0, 3) = so3::hat(centre);
  out.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();
  return out;
}

/// @brief The first index within `kTieRtol` of the largest value.
/// @param values The values.
/// @return The index.
Index first_largest(const Eigen::VectorXd& values) {
  const double threshold = values.maxCoeff() * (1.0 - kTieRtol);
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

/// @brief Score candidates seeing the same number of directions (higher wins): the lowest
///        index for a pose that would be fixed, the smallest retained singular value for one
///        that would be confined.
/// @param cam The candidate.
/// @param singular Its image's singular values.
/// @param rank How many directions it sees.
/// @return The score.
double tie_break(Index cam, const Eigen::VectorXd& singular, Index rank) {
  if (rank == kPoseDim) {
    return -static_cast<double>(cam);
  }
  return rank > 0 ? singular(rank - 1) : 0.0;
}

/// @brief The candidate that sees the most of N (ties by `tie_break`), and how many.
/// @param anchors What is pinned (for the frame).
/// @param centres Every camera's world centre.
/// @param left The candidates.
/// @param null The remaining gauge.
/// @return The candidate (-1 if none sees anything) and its rank.
std::pair<Index, Index> best_candidate(const Anchors& anchors, const Positions& centres,
                                       const IndexList& left, const Eigen::MatrixXd& null) {
  Index best = -1;
  Index best_rank = 0;
  double best_score = 0.0;
  for (const Index c : left) {
    const Eigen::VectorXd singular = singular_values(
        pose_image(in_frame(anchors.frame, centres[static_cast<std::size_t>(c)]), null));
    const Index rank = rank_of(singular);
    const double score = tie_break(c, singular, rank);
    if (rank > best_rank ||
        (rank == best_rank && score > best_score + (kTieRtol * std::abs(best_score)))) {
      best = c;
      best_rank = rank;
      best_score = score;
    }
  }
  return {best, best_rank};
}

}  // namespace

Frame frame_of(const Positions& positions) {
  if (positions.empty()) {
    return {};
  }
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& position : positions) {
    origin += position;
  }
  origin /= static_cast<double>(positions.size());
  double squared = 0.0;
  for (const Eigen::Vector3d& position : positions) {
    squared += (position - origin).squaredNorm();
  }
  const double spread = std::sqrt(squared / static_cast<double>(positions.size()));
  return {.origin = origin, .scale = spread > 0.0 ? spread : 1.0};
}

Eigen::MatrixXd remaining(const Anchors& anchors, const Positions& centres) {
  const Eigen::MatrixXd pinned = rows(anchors, centres);
  return null_space(pinned, rank_of(singular_values(pinned)));
}

Gauge classify(const Eigen::MatrixXd& null) {
  const Index k = null.cols();
  if (k == 0) {
    return Gauge::kNone;
  }
  if (k == kGenerators) {
    return Gauge::kSimilarity;
  }
  if (k == kRigidDim && null.row(kGenerators - 1).cwiseAbs().maxCoeff() <= kGaugeTolerance) {
    return Gauge::kFirstPose;
  }
  if (k == 1 && null.middleRows(3, 3).cwiseAbs().maxCoeff() <= kGaugeTolerance) {
    return Gauge::kScale;
  }
  return Gauge::kPartial;
}

Plan plan(const Anchors& anchors, const Positions& centres, const IndexList& candidates) {
  Eigen::MatrixXd null = remaining(anchors, centres);
  Plan out;
  IndexList left = candidates;
  while (null.cols() > 0 && !left.empty()) {
    const auto [best, rank] = best_candidate(anchors, centres, left, null);
    if (rank == 0) {
      break;
    }
    if (rank == kPoseDim) {
      out.fixed.push_back(best);
    } else {
      out.confined.push_back(best);
      out.ranks.push_back(rank);
    }
    null = null *
           null_space(
               pose_image(in_frame(anchors.frame, centres[static_cast<std::size_t>(best)]), null),
               rank);
    left.erase(std::ranges::find(left, best));
  }
  out.free = null.cols();
  return out;
}

std::vector<Eigen::MatrixXd> confinement(const Anchors& anchors, const Positions& centres,
                                         const Plan& chosen) {
  const Index dimension =
      std::accumulate(chosen.ranks.begin(), chosen.ranks.end(), Index{0}) + chosen.free;
  Eigen::MatrixXd null = null_space(rows(anchors, centres), kGenerators - dimension);
  Eigen::Matrix<double, kPoseDim, kPoseDim> to_world =
      Eigen::Matrix<double, kPoseDim, kPoseDim>::Identity();
  to_world.topLeftCorner<3, 3>() *= anchors.frame.scale;
  std::vector<Eigen::MatrixXd> bases;
  for (std::size_t i = 0; i < chosen.confined.size(); ++i) {
    const Eigen::Vector3d& centre = centres[static_cast<std::size_t>(chosen.confined[i])];
    const Eigen::MatrixXd image = pose_image(in_frame(anchors.frame, centre), null);
    bases.push_back(complement(
        canonical_span(pose_jacobian(centre).transpose() * to_world * image, chosen.ranks[i]),
        kPoseDim));
    null = null * null_space(image, chosen.ranks[i]);
  }
  return bases;
}

}  // namespace slamlab::foundations::ba::detail
