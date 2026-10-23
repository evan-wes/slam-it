/// @file
/// @brief The pipeline's internals its tests reach (the build's rule for a rung's internal
///        headers, `cpp/src/<rung>/...`): how a loop being confirmed is carried through a merge,
///        defined in pipeline.cpp. Not part of the edge rung's interface (B4's review panel,
///        run 3, A16).
#pragma once

#include <Eigen/Core>

#include "slam_it/edge/loops.hpp"

/// @brief Implementation details of the pipeline, declared for its tests.
namespace slam_it::edge::detail {

/// @brief A loop being confirmed in the active map, as it stands in the merged map
///        (`slam_it.edge.pipeline._carried`): its ids moved on by the merge's offsets, -1 kept,
///        its similarity carried into the matched map's world and units,
///        `diag(1/s, 1/s, 1/s, 1) S G^-1` with `s = scale_of(G^-1)`, so it keeps its own scale.
/// @param pending The loop, changed in place.
/// @param merged The merge: its offsets.
/// @param to_world `G`.
void carry_through(loops::Pending& pending, const loops::Merged& merged,
                   const Eigen::Matrix4d& to_world);

}  // namespace slam_it::edge::detail
