/// @file
/// @brief Shared by every binding that runs `nls::solve`: its summary out.
///
/// A summary crosses as a dict: `initial_cost`, `final_cost`, `termination` (its index in
/// `nls_terminations()`), `trace` (one row per step) and `trace_columns` (the
/// `nls::Iteration` field each column holds, which the Python side reads by name).
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <array>
#include <cstddef>
#include <vector>

#include "common.hpp"
#include "slamlab/foundations/nls.hpp"

namespace slamlab::bindings {

/// @brief The columns of a returned trace row.
inline constexpr std::size_t kTraceColumns = 7;
/// @brief The trace's column names (`nls::Iteration`'s fields), in `trace_array` order.
inline constexpr std::array<const char*, kTraceColumns> kTraceNames = {
    "iteration", "cost", "gradient_norm", "step_norm", "damping", "relative_decrease", "accepted"};

/// @brief The summary's trace as a new (I, 7) array.
/// @param trace The trace.
/// @return The array.
inline ArrayOut trace_array(const std::vector<foundations::nls::Iteration>& trace) {
  auto [out, data] = new_array({trace.size(), kTraceColumns});
  for (std::size_t i = 0; i < trace.size(); ++i) {
    const foundations::nls::Iteration& it = trace[i];
    const std::array<double, kTraceColumns> row = {static_cast<double>(it.iteration),
                                                   it.cost,
                                                   it.gradient_norm,
                                                   it.step_norm,
                                                   it.damping,
                                                   it.relative_decrease,
                                                   it.accepted ? 1.0 : 0.0};
    for (std::size_t k = 0; k < row.size(); ++k) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): row offset
      data[(kTraceColumns * i) + k] = row.at(k);
    }
  }
  return out;
}

/// @brief `nls::Termination`'s names in code order.
/// @return The names.
inline nanobind::list nls_terminations() {
  nanobind::list out;
  for (int k = 0; k < static_cast<int>(foundations::nls::Termination::kCount); ++k) {
    out.append(foundations::nls::name(static_cast<foundations::nls::Termination>(k)));
  }
  return out;
}

/// @brief A solve's summary as a dict.
/// @param summary The summary.
/// @return `{"initial_cost", "final_cost", "termination", "trace", "trace_columns"}`.
inline nanobind::dict nls_summary(const foundations::nls::Summary& summary) {
  nanobind::list columns;
  for (const char* name : kTraceNames) {
    columns.append(name);
  }
  nanobind::dict out;
  out["initial_cost"] = summary.initial_cost;
  out["final_cost"] = summary.final_cost;
  out["termination"] = static_cast<int>(summary.termination);
  out["trace"] = trace_array(summary.trace);
  out["trace_columns"] = columns;
  return out;
}

}  // namespace slamlab::bindings
