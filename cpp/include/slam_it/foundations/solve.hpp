/// @file
/// @brief What every iterative solver returns: its best estimate, and a summary of how it
///        went.
///
/// Mirrors `slam_it.foundations.solve`. Each solver (non-linear least squares, RANSAC) owns
/// its summary, because what is worth reporting differs; the `Summary` concept names the
/// fields they all have, and `Solution` pairs the estimate with its summary. A solve that
/// stops short of its convergence test returns a `Solution`, not an error.
#pragma once

#include <concepts>

/// @brief The contract shared by rung 0's iterative solvers.
namespace slam_it::foundations::solve {

/// @brief The fields every solver's summary has.
/// @tparam S The summary type.
template <typename S>
concept Summary = requires(const S& summary) {
  { summary.iterations } -> std::convertible_to<int>;
  { summary.converged } -> std::convertible_to<bool>;
  summary.termination;
};

/// @brief A solver's best estimate, and how the solve went.
/// @tparam State The estimate's type.
/// @tparam S The solver's summary type.
template <typename State, Summary S>
struct Solution {
  /// @brief The estimate.
  State state;
  /// @brief The solver's summary.
  S summary;
};

}  // namespace slam_it::foundations::solve
