/// @file
/// @brief Bundle adjustment internals: what is optimised -- kept blocks, free variables, gauge.
#pragma once

#include <cstdint>

#include "common.hpp"
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
