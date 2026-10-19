/// @file
/// @brief Bindings for `slam_it.foundations.triangulation`
///        (`slam_it._cpp.foundations.triangulation`).
#include "slam_it/foundations/triangulation.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace triangulation = foundations::triangulation;

/// @brief One read-only (4, 4) pose.
using PoseIn = nb::ndarray<const double, nb::shape<4, 4>, nb::c_contig, nb::device::cpu>;
/// @brief A read-only (P, V, 3) bearing array.
using RaysIn = nb::ndarray<const double, nb::shape<-1, -1, 3>, nb::c_contig, nb::device::cpu>;
/// @brief A read-only (P, V) boolean mask.
using MaskIn = nb::ndarray<const bool, nb::shape<-1, -1>, nb::c_contig, nb::device::cpu>;
/// @brief A row-major boolean matrix viewed in place.
using MaskView =
    Eigen::Map<const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

/// @brief Pack a triangulation as a new (P, 5) array: point, `in_front` (0 or 1), parallax.
/// @param result The triangulation.
/// @return The array.
ArrayOut pack(const triangulation::Triangulation& result) {
  const auto n = static_cast<std::size_t>(result.points.rows());
  auto [out, data] = new_array({n, 5});
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 5, Eigen::RowMajor>> view(
      data, static_cast<Eigen::Index>(n), 5);
  view.leftCols<3>() = result.points;
  view.col(3) = result.in_front.cast<double>().matrix();
  view.col(4) = result.parallax;
  return out;
}

/// @brief Triangulate bearing pairs through `T_c2_c1` (Lindstrom's niter2).
/// @param relative_pose The (4, 4) relative pose.
/// @param bearings1 (P, 3) unit bearings in camera 1.
/// @param bearings2 (P, 3) unit bearings in camera 2.
/// @return A new (P, 5) array.
/// @throws CoreError for invalid input.
ArrayOut two_view(const PoseIn& relative_pose, const BatchIn<3>& bearings1,
                  const BatchIn<3>& bearings2) {
  check_aligned(relative_pose.data());
  const Eigen::Matrix4d pose = Eigen::Map<const Item<4, 4>>(relative_pose.data());
  const auto b1 = copy_matrix<triangulation::Rows3>(bearings1.data(), bearings1.shape(0), 3);
  const auto b2 = copy_matrix<triangulation::Rows3>(bearings2.data(), bearings2.shape(0), 3);
  return pack(unwrap(triangulation::two_view(pose, b1, b2)));
}

/// @brief Triangulate points seen from several views (DLT on bearings).
/// @param poses (V, 4, 4) `T_cam_world`.
/// @param bearings (P, V, 3) unit bearings.
/// @param observed (P, V) observation mask.
/// @return A new (P, 5) array.
/// @throws CoreError for invalid input.
ArrayOut n_view(const BatchIn<4, 4>& poses, const RaysIn& bearings, const MaskIn& observed) {
  check_aligned(bearings.data());
  const auto rays =
      copy_matrix<triangulation::Rows3>(bearings.data(), bearings.shape(0) * bearings.shape(1), 3);
  const MaskView mask(observed.data(), static_cast<Eigen::Index>(observed.shape(0)),
                      static_cast<Eigen::Index>(observed.shape(1)));
  return pack(unwrap(triangulation::n_view(copy_poses(poses), rays, mask)));
}

}  // namespace

void bind_triangulation(nb::module_& m) {
  m.def("two_view", &two_view, nb::arg("relative_pose").noconvert(),
        nb::arg("bearings1").noconvert(), nb::arg("bearings2").noconvert(),
        "Triangulate (P, 3) bearing pairs through T_c2_c1; return (P, 5).");
  m.def("n_view", &n_view, nb::arg("poses").noconvert(), nb::arg("bearings").noconvert(),
        nb::arg("observed").noconvert(),
        "Triangulate (P, V, 3) bearings from (V, 4, 4) poses where observed; return (P, 5).");
}

}  // namespace slam_it::bindings
