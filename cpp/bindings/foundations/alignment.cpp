/// @file
/// @brief Bindings for `slam_it.foundations.alignment`
/// (`slam_it._cpp.foundations.alignment`).
#include "slam_it/foundations/alignment.hpp"

#include <nanobind/nanobind.h>

#include <cstddef>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief Read-only, contiguous float64 weights.
using WeightsIn = nb::ndarray<const double, nb::shape<-1>, nb::c_contig, nb::device::cpu>;

/// @brief A row-major (N, 3) point matrix viewed in place.
using PointsView = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>;

/// @brief Align (N, 3) point sets; return the (4, 4) matrix `[[s R, t], [0, 1]]`.
/// @param source Source points.
/// @param target Target points.
/// @param weights Non-negative weights, one per point.
/// @param with_scale Estimate a scale too.
/// @return A new (4, 4) array.
/// @throws CoreError for invalid or degenerate input.
ArrayOut umeyama(const BatchIn<3>& source, const BatchIn<3>& target, const WeightsIn& weights,
                 bool with_scale) {
  check_aligned(source.data());
  check_aligned(target.data());
  check_aligned(weights.data());
  const auto rows = static_cast<Eigen::Index>(source.shape(0));
  const PointsView src(source.data(), rows, 3);
  const PointsView tgt(target.data(), static_cast<Eigen::Index>(target.shape(0)), 3);
  const Eigen::Map<const Eigen::VectorXd> w(weights.data(),
                                            static_cast<Eigen::Index>(weights.shape(0)));
  const Eigen::Matrix4d result = unwrap(foundations::alignment::umeyama(src, tgt, w, with_scale));
  auto [out, data] = new_array({4, 4});
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> view(data);
  view = result;
  return out;
}

}  // namespace

void bind_alignment(nb::module_& m) {
  m.def("umeyama", &umeyama, nb::arg("source").noconvert(), nb::arg("target").noconvert(),
        nb::arg("weights").noconvert(), nb::arg("with_scale"),
        "Align (N, 3) point sets; return the (4, 4) matrix [[s R, t], [0, 1]].");
}

}  // namespace slam_it::bindings
