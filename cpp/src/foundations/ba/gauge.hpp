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
#include "slam_it_all/foundations/ba.hpp"

namespace slam_it_all::foundations::ba::detail {

/// @brief The similarity's directions: translation (3), rotation (3), scale (1).
inline constexpr int kGenerators = 7;
/// @brief The rigid motions' directions: translation (3), rotation (3).
inline constexpr int kRigidDim = 6;
/// @brief Singular values below this count as zero. Rows and images are in scene-normalised
///        coordinates (see `Frame`), of order 1, so it is in effect relative.
inline constexpr double kGaugeTolerance = 1e-9;
/// @brief Two scores within this relative margin are a tie, broken by the lower index: both
///        cores then choose alike despite last-bit rounding between their SVDs.
inline constexpr double kTieRtol = 1e-12;

/// @brief World positions, one per camera or point.
using Positions = std::vector<Eigen::Vector3d>;

/// @brief Scene-normalised coordinates for the gauge: `X' = (X - origin) / scale`. The
///        similarity's generators act about the world origin; far from it the rotation and
///        scale directions shrink like `1 / |X|`, and a rank test fails for geo-referenced
///        scenes. A change of origin and unit maps the generators invertibly, so spans --
///        and the gauge -- are unchanged.
struct Frame {
  /// @brief The mean of the anchors' and observed cameras' world positions.
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  /// @brief Their RMS distance from `origin` (1 if they coincide).
  double scale = 1.0;
};

/// @brief The frame centred on `positions` and scaled by their spread.
/// @param positions World positions.
/// @return The frame.
[[nodiscard]] Frame frame_of(const Positions& positions);

/// @brief A world position in a frame.
/// @param frame The frame.
/// @param position The position.
/// @return `(position - origin) / scale`.
[[nodiscard]] inline Eigen::Vector3d in_frame(const Frame& frame, const Eigen::Vector3d& position) {
  return (position - frame.origin) / frame.scale;
}

/// @brief What pins the world's similarity, before any gauge is applied.
struct Anchors {
  /// @brief Fixed known positions in the world.
  Positions points;
  /// @brief Poses whose current world centre is pinned (fixed poses, translation priors).
  IndexList centre_cams;
  /// @brief Poses whose current world orientation is pinned (fixed poses, rotation priors).
  IndexList orientation_cams;
  /// @brief Whether metric depth pins the scale.
  bool metric = false;
  /// @brief The scene-normalised coordinates the rows are built in.
  Frame frame;
};

/// @brief Which poses remove the gauge, decided once from the start.
struct Plan {
  /// @brief Poses fixed outright (each saw all six of its directions).
  IndexList fixed;
  /// @brief Poses whose step is confined, in the order they take their share.
  IndexList confined;
  /// @brief How many directions each confined pose removes.
  IndexList ranks;
  /// @brief Directions no free pose could take, left in the problem (0 unless degenerate).
  Index free = 0;
};

/// @brief The remaining gauge N.
/// @param anchors What is pinned.
/// @param centres Every camera's world centre.
/// @return An orthonormal 7 x k basis over `(t, omega, s)` in the anchors' frame.
[[nodiscard]] Eigen::MatrixXd remaining(const Anchors& anchors, const Positions& centres);

/// @brief Name a remaining gauge: none, scale (about a point), first pose (the rigid motions),
///        similarity, or partial. Reads only N's dimension and its omega and s rows, which a
///        change of origin and unit leaves alone.
/// @param null The remaining gauge N.
/// @return The gauge it is.
[[nodiscard]] Gauge classify(const Eigen::MatrixXd& null);

/// @brief Choose the poses that remove the remaining gauge, greedily: at each turn the
///        candidate seeing the most remaining directions takes them (fixed if it sees six,
///        confined otherwise); among equals, the lowest index for a fixed pose, the one seeing
///        its directions best for a confined one.
/// @param anchors What is pinned.
/// @param centres Every camera's world centre at the start.
/// @param candidates The free observed poses, in index order.
/// @return The plan, with what no pose could take.
[[nodiscard]] Plan plan(const Anchors& anchors, const Positions& centres,
                        const IndexList& candidates);

/// @brief Each confined pose's step basis at the current centres: one 6 x (6 - rank) basis
///        per confined pose, in the plan's order, over its right-perturbation step.
/// @param anchors What is pinned, the poses the plan fixed included.
/// @param centres Every camera's current world centre.
/// @param chosen The plan.
/// @return The bases.
[[nodiscard]] std::vector<Eigen::MatrixXd> confinement(const Anchors& anchors,
                                                       const Positions& centres,
                                                       const Plan& chosen);

}  // namespace slam_it_all::foundations::ba::detail
