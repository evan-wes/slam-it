/// @file
/// @brief Bundle adjustment internals: the NLS problem over poses, intrinsics, points,
///        affines and inverse depths, and the solve that drives it.
#pragma once

#include "layout.hpp"
#include "slam_it/foundations/ba.hpp"
#include "slam_it/foundations/error.hpp"

namespace slam_it::foundations::ba::detail {

/// @brief Solve a checked problem over its layout.
/// @param problem The problem (already checked).
/// @param options The settings (already checked).
/// @param layout The problem's layout.
/// @return The solution, or `NotConverged` in strict mode.
[[nodiscard]] Result<Solution> adjust(const Problem& problem, const Options& options,
                                      const Layout& layout);

}  // namespace slam_it::foundations::ba::detail
