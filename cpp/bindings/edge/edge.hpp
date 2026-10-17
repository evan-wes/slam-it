/// @file
/// @brief The `slam_it._cpp.edge` submodule: one function per bound edge module.
#pragma once

#include <nanobind/nanobind.h>

namespace slam_it::bindings {

/// @brief Add the `edge` submodule and its modules.
/// @param parent The extension module.
void bind_edge(nanobind::module_& parent);

/// @brief Bind `slam_it.edge.frames`' compiled functions.
/// @param m The submodule.
void bind_frames(nanobind::module_& m);

/// @brief Bind `slam_it.edge.map`'s compiled functions.
/// @param m The submodule.
void bind_map(nanobind::module_& m);

/// @brief Bind `slam_it.edge.residuals`' compiled functions.
/// @param m The submodule.
void bind_residuals(nanobind::module_& m);

/// @brief Bind `slam_it.edge.adjust`'s compiled functions.
/// @param m The submodule.
void bind_adjust(nanobind::module_& m);

/// @brief Bind `slam_it.edge.initialization`'s compiled functions.
/// @param m The submodule.
void bind_initialization(nanobind::module_& m);

/// @brief Bind `slam_it.edge.tracking`'s compiled functions.
/// @param m The submodule.
void bind_tracking(nanobind::module_& m);

/// @brief Bind `slam_it.edge.mapping`'s compiled functions.
/// @param m The submodule.
void bind_mapping(nanobind::module_& m);

/// @brief Bind `slam_it.edge.covisibility`'s compiled functions.
/// @param m The submodule.
void bind_covisibility(nanobind::module_& m);

/// @brief Bind `slam_it.edge.database`'s compiled functions.
/// @param m The submodule.
void bind_database(nanobind::module_& m);

/// @brief Bind `slam_it.edge.relocalization`'s compiled functions.
/// @param m The submodule.
void bind_relocalization(nanobind::module_& m);

}  // namespace slam_it::bindings
