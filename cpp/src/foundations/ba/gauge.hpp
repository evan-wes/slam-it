/// @file
/// @brief Bundle adjustment internals: the gauge -- which similarity motions of the world the
///        data leaves free, and their removal (see the Python reference, `ba/gauge.py`).
///
/// Each anchor contributes rows of the derivative of what it pins with respect to the
/// similarity's generators `g = (t, omega, s)`: a known position X gives `[I, -hat(X), X]`, a
/// known orientation `[0, I, 0]`, metric depth `[0, 0, 1]`. The remaining gauge is their null
/// space N. It is removed pose by pose: at pose c, N moves the camera's world centre and
/// orientation by `M = [[I, -hat(C), C], [0, I, 0]] N`, and the pose's step is confined to the
/// complement of `J_c^T M` (`J_c` maps a step to the change in centre and orientation). A pose
/// that sees six directions is fixed outright.
#pragma once

#include <Eigen/Core>
#include <vector>

#include "common.hpp"
#include "slamlab/foundations/ba.hpp"

namespace slamlab::foundations::ba::detail {

/// @brief The similarity's directions: translation (3), rotation (3), scale (1).
inline constexpr int kGenerators = 7;
/// @brief Singular values below this, relative to the scene's extent, count as zero.
inline constexpr double kGaugeTolerance = 1e-9;

/// @brief World positions, one per camera or point.
using Positions = std::vector<Eigen::Vector3d>;

/// @brief What pins the world's similarity, before any gauge is applied.
struct Anchors {
  /// @brief Fixed known positions in the world.
  Positions points;
  /// @brief Poses whose current centre is pinned (fixed poses, translation priors).
  IndexList centre_cams;
  /// @brief Poses whose current orientation is pinned (fixed poses, rotation priors).
  IndexList orientation_cams;
  /// @brief Whether metric depth pins the scale.
  bool metric = false;
};

/// @brief Which poses remove the gauge, decided once from the start.
struct Plan {
  /// @brief Poses fixed outright (each saw all six of its directions).
  IndexList fixed;
  /// @brief Poses whose step is confined, in the order they take their share.
  IndexList confined;
  /// @brief How many directions each confined pose removes.
  IndexList ranks;
};

/// @brief The remaining gauge N.
/// @param anchors What is pinned.
/// @param centres Every camera's world centre.
/// @param tolerance The absolute rank tolerance.
/// @return An orthonormal 7 x k basis.
[[nodiscard]] Eigen::MatrixXd remaining(const Anchors& anchors, const Positions& centres,
                                        double tolerance);

/// @brief Name a remaining gauge: none, scale (about a point), first pose (the rigid motions),
///        similarity, or partial.
/// @param null The remaining gauge N.
/// @return The gauge it is.
[[nodiscard]] Gauge name(const Eigen::MatrixXd& null);

/// @brief Choose the poses that remove the remaining gauge, greedily: at each turn the
///        candidate seeing the most remaining directions takes them; among poses seeing all
///        six the lowest index, among the others the one seeing them best.
/// @param anchors What is pinned.
/// @param centres Every camera's world centre at the start.
/// @param candidates The free observed poses, in index order.
/// @param tolerance The absolute rank tolerance.
/// @return The plan.
[[nodiscard]] Plan plan(const Anchors& anchors, const Positions& centres,
                        const IndexList& candidates, double tolerance);

/// @brief Each confined pose's step basis at the current centres (6 x (6 - rank)).
/// @param anchors What is pinned, the poses the plan fixed included.
/// @param centres Every camera's current world centre.
/// @param chosen The plan: its confined poses, in order, and their ranks.
/// @return The bases.
[[nodiscard]] std::vector<Eigen::MatrixXd> confinement(const Anchors& anchors,
                                                       const Positions& centres,
                                                       const Plan& chosen);

}  // namespace slamlab::foundations::ba::detail
