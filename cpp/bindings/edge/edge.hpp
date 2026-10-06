/// @file
/// @brief The `slamlab._cpp.edge` submodule: one function per bound edge module.
#pragma once

#include <nanobind/nanobind.h>

namespace slamlab::bindings {

/// @brief Add the `edge` submodule and its modules.
/// @param parent The extension module.
void bind_edge(nanobind::module_& parent);

/// @brief Bind `slamlab.edge.frames`' compiled functions.
/// @param m The submodule.
void bind_frames(nanobind::module_& m);

/// @brief Bind `slamlab.edge.map`'s compiled functions.
/// @param m The submodule.
void bind_map(nanobind::module_& m);

/// @brief Bind `slamlab.edge.residuals`' compiled functions.
/// @param m The submodule.
void bind_residuals(nanobind::module_& m);

/// @brief Bind `slamlab.edge.adjust`'s compiled functions.
/// @param m The submodule.
void bind_adjust(nanobind::module_& m);

/// @brief Bind `slamlab.edge.initialization`'s compiled functions.
/// @param m The submodule.
void bind_initialization(nanobind::module_& m);

/// @brief Bind `slamlab.edge.tracking`'s compiled functions.
/// @param m The submodule.
void bind_tracking(nanobind::module_& m);

}  // namespace slamlab::bindings
