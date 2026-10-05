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

/// @brief Add the `camera` functions to the `foundations.camera` submodule.
/// @param m The `foundations.camera` submodule.
void bind_camera(nanobind::module_& m);

/// @brief Add the `image` functions to the `foundations.image` submodule.
/// @param m The `foundations.image` submodule.
void bind_image(nanobind::module_& m);

/// @brief Add the `alignment` functions to the `foundations.alignment` submodule.
/// @param m The `foundations.alignment` submodule.
void bind_alignment(nanobind::module_& m);

/// @brief Add the `triangulation` functions to the `foundations.triangulation` submodule.
/// @param m The `foundations.triangulation` submodule.
void bind_triangulation(nanobind::module_& m);

/// @brief Add the `epipolar` functions to the `foundations.epipolar` submodule.
/// @param m The `foundations.epipolar` submodule.
void bind_epipolar(nanobind::module_& m);

/// @brief Add the `fivepoint` functions to the `foundations.fivepoint` submodule.
/// @param m The `foundations.fivepoint` submodule.
void bind_fivepoint(nanobind::module_& m);

/// @brief Add the `p3p` functions to the `foundations.p3p` submodule.
/// @param m The `foundations.p3p` submodule.
void bind_p3p(nanobind::module_& m);
/// @brief Add the `pnp` functions to the `foundations.pnp` submodule.
/// @param m The `foundations.pnp` submodule.
void bind_pnp(nanobind::module_& m);
/// @brief Add the `homography` functions to the `foundations.homography` submodule.
/// @param m The `foundations.homography` submodule.
void bind_homography(nanobind::module_& m);

/// @brief Add the `twoview` functions to the `foundations.twoview` submodule.
/// @param m The `foundations.twoview` submodule.
void bind_twoview(nanobind::module_& m);

/// @brief Add the `ba` functions to the `foundations.ba` submodule.
/// @param m The `foundations.ba` submodule.
void bind_ba(nanobind::module_& m);

}  // namespace slamlab::bindings
