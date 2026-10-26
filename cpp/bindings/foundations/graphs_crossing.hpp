/// @file
/// @brief Shared by every binding that returns a graph solve's summary: `graphs.Summary` out.
///
/// A summary crosses as a dict's fields: `initial_error`, `final_error`, `iterations`,
/// `termination` (its code in `codes()["graphs_termination"]`), and a bundle adjustment's
/// counts, `left_out` and `ended_near` (0 for another solve), which
/// `slam_it.foundations.graphs.summary_from_backend` reads.
#pragma once

#include <nanobind/nanobind.h>

#include "slam_it/foundations/graphs.hpp"

namespace slam_it::bindings {

/// @brief Write a graph solve's summary's fields into a dict.
/// @param summary The summary.
/// @param out The dict.
inline void write_graph_summary(const foundations::graphs::Summary& summary, nanobind::dict& out) {
  out["initial_error"] = summary.initial_error;
  out["final_error"] = summary.final_error;
  out["iterations"] = summary.iterations;
  out["termination"] = static_cast<int>(summary.termination);
  out["left_out"] = summary.left_out;
  out["ended_near"] = summary.ended_near;
}

}  // namespace slam_it::bindings
