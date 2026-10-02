/// @file
/// @brief Bundle adjustment internals: what is optimised -- kept blocks, free variables, gauge.
#pragma once

#include <cstdint>

#include "common.hpp"
#include "gauge.hpp"
#include "slamlab/foundations/ba.hpp"

namespace slamlab::foundations::ba::detail {

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
  /// @brief Cameras with a pose prior that can move (a prior on a pose that cannot is
  ///        constant).
  IndexList prior_cams;
  /// @brief Depth affines that move.
  Mask affine_free;
  /// @brief Poses with all six directions free.
  Mask pose_free;
  /// @brief What pins the similarity, the poses the gauge fixed included.
  Anchors gauge_anchors;
  /// @brief The poses that remove the gauge: fixed ones (already out of `pose_free`), and
  ///        confined ones in order with how many directions each removes.
  Plan gauge_plan;
  /// @brief Intrinsics parameters that move, per group and parameter (K x P, row-major).
  Mask intrinsics_free;
  /// @brief Each point's index among the free points, or -1.
  IndexList point_slot;
  /// @brief The number of free points.
  Index free_points = 0;
  /// @brief The layout's counts (the solve summary is filled in later).
  Summary counts;
};

/// @brief Drop degenerate observations, hold unobserved variables fixed, and fix the gauge.
/// @param problem The problem.
/// @param options The settings.
/// @return The layout, or `InvalidInput` for a gauge the problem cannot support.
[[nodiscard]] Result<Layout> make_layout(const Problem& problem, const Options& options);

}  // namespace slamlab::foundations::ba::detail
