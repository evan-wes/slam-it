/// @file
/// @brief Bindings for rung 0, foundations: one function per bound module.
#pragma once

#include <nanobind/nanobind.h>

/// @brief Python bindings for the C++ core.
namespace slamlab::bindings {

/// @brief Add the `foundations` submodule and its bound modules to the extension.
/// @param parent The extension's root module.
void bind_foundations(nanobind::module_& parent);

/// @brief Add the `so3` functions to the `foundations.so3` submodule.
/// @param m The `foundations.so3` submodule.
void bind_so3(nanobind::module_& m);

/// @brief Add the `se3` functions to the `foundations.se3` submodule.
/// @param m The `foundations.se3` submodule.
void bind_se3(nanobind::module_& m);

/// @brief Add the `alignment` functions to the `foundations.alignment` submodule.
/// @param m The `foundations.alignment` submodule.
void bind_alignment(nanobind::module_& m);

}  // namespace slamlab::bindings
