/// @file
/// @brief Bindings for `slam_it.foundations.stereo` (`slam_it._cpp.foundations.stereo`).
///
/// Camera models cross as their integer codes; a rectification comes back as a dict of its
/// rotations, rectified intrinsics and baseline. Features cross as dicts (orb_crossing.hpp);
/// settings as dicts of their dataclasses' fields.
#include "slam_it/foundations/stereo.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/image_crossing.hpp"
#include "foundations/orb_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

/// @brief A read-only, contiguous float64 vector.
using Vector = VectorIn<double>;

/// @brief Map a float64 vector in place.
/// @param in The array.
/// @return An Eigen view of it.
Eigen::Map<const Eigen::VectorXd> vector_view(const Vector& in) {
  check_aligned(in.data());
  return {in.data(), static_cast<Eigen::Index>(in.shape(0))};
}

/// @brief Rectify a pair.
/// @param model_left The left camera's model code.
/// @param intrinsics_left Its intrinsics.
/// @param model_right The right camera's model code.
/// @param intrinsics_right Its intrinsics.
/// @param T_right_left The (4, 4) transform from left to right camera coordinates.
/// @param rows The images' rows.
/// @param cols The images' columns.
/// @return A dict: `rotation_left`, `rotation_right` (3, 3), `intrinsics` (4,), `baseline`.
/// @throws CoreError for invalid or degenerate input; std::invalid_argument for a bad code.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the Python signature's order
nb::dict rectify(int model_left, const Vector& intrinsics_left, int model_right,
                 const Vector& intrinsics_right, const Matrix4In& T_right_left, std::int64_t rows,
                 std::int64_t cols) {
  const foundations::stereo::Rectification r = unwrap(foundations::stereo::rectify(
      enum_code<foundations::camera::Model>(model_left, "model_left"), vector_view(intrinsics_left),
      enum_code<foundations::camera::Model>(model_right, "model_right"),
      vector_view(intrinsics_right), matrix4(T_right_left), rows, cols));
  nb::dict out;
  out["rotation_left"] = matrix3_array(r.rotation_left);
  out["rotation_right"] = matrix3_array(r.rotation_right);
  out["intrinsics"] = vector_array(r.intrinsics);
  out["baseline"] = r.baseline;
  return out;
}

/// @brief One side's rectification map.
/// @param model The camera's model code.
/// @param intrinsics Its intrinsics.
/// @param rotation Its rectifying rotation.
/// @param rectified The rectified pinhole intrinsics.
/// @param rows The images' rows.
/// @param cols The images' columns.
/// @return A new (rows, cols, 2) float64 array.
/// @throws CoreError for invalid input; std::invalid_argument for a bad code.
ArrayOut rectification_map(int model, const Vector& intrinsics, const Matrix3In& rotation,
                           const Vector& rectified, std::int64_t rows, std::int64_t cols) {
  const Eigen::Map<const Eigen::VectorXd> pinhole = vector_view(rectified);
  if (pinhole.size() != 4) {
    throw std::invalid_argument("rectified: expected 4 values");
  }
  const foundations::image::Coordinates map = unwrap(foundations::stereo::rectification_map(
      enum_code<foundations::camera::Model>(model, "model"), vector_view(intrinsics),
      matrix3(rotation), Eigen::Vector4d(pinhole), rows, cols));
  auto [out, data] = new_array({static_cast<std::size_t>(rows), static_cast<std::size_t>(cols), 2});
  Eigen::Map<foundations::image::Coordinates>(data, map.rows(), 2) = map;
  return out;
}

/// @brief `stereo::MatchOptions`' fields, as the dict's keys.
constexpr std::array<const char*, 5> kStereoOptionKeys = {"max_distance", "band", "window",
                                                          "search", "outlier_factor"};

/// @brief Match features along the rows of a rectified pair.
/// @param left_levels The rectified left image's pyramid.
/// @param right_levels The rectified right image's.
/// @param left The left features, as a dict.
/// @param right The right features, as a dict.
/// @param focal The rectified focal length.
/// @param baseline The baseline.
/// @param options The matching settings, as a dict.
/// @return A dict of `StereoMatches`' fields.
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the Python signature's order
nb::dict match(const std::vector<ImageIn>& left_levels, const std::vector<ImageIn>& right_levels,
               const nb::dict& left, const nb::dict& right, double focal, double baseline,
               const nb::dict& options) {
  check_keys(options, kStereoOptionKeys, "stereo.Options");
  const foundations::stereo::MatchOptions settings{
      .max_distance = nb::cast<std::int64_t>(options["max_distance"]),
      .band = nb::cast<double>(options["band"]),
      .window = nb::cast<std::int64_t>(options["window"]),
      .search = nb::cast<std::int64_t>(options["search"]),
      .outlier_factor = nb::cast<double>(options["outlier_factor"])};
  const std::vector<foundations::stereo::StereoMatch> matches = unwrap(
      foundations::stereo::match(levels_in(left_levels), levels_in(right_levels), features_in(left),
                                 features_in(right), focal, baseline, settings));
  const std::size_t k = matches.size();
  auto [left_index, left_data] = new_array<std::int64_t>({k});
  auto [right_index, right_data] = new_array<std::int64_t>({k});
  auto [right_x, right_x_data] = new_array({k});
  auto [disparity, disparity_data] = new_array({k});
  auto [depth, depth_data] = new_array({k});
  for (std::size_t i = 0; i < k; ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    left_data[i] = matches[i].left;
    right_data[i] = matches[i].right;
    right_x_data[i] = matches[i].right_x;
    disparity_data[i] = matches[i].disparity;
    depth_data[i] = matches[i].depth;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  nb::dict out;
  out["left"] = left_index;
  out["right"] = right_index;
  out["right_x"] = right_x;
  out["disparity"] = disparity;
  out["depth"] = depth;
  return out;
}

}  // namespace

void bind_stereo(nb::module_& m) {
  m.def("rectify", &rectify, nb::arg("model_left"), nb::arg("intrinsics_left").noconvert(),
        nb::arg("model_right"), nb::arg("intrinsics_right").noconvert(),
        nb::arg("T_right_left").noconvert(), nb::arg("rows"), nb::arg("cols"),
        "Rectify a pair; return the rotations, the rectified intrinsics and the baseline.");
  m.def("rectification_map", &rectification_map, nb::arg("model"),
        nb::arg("intrinsics").noconvert(), nb::arg("rotation").noconvert(),
        nb::arg("rectified").noconvert(), nb::arg("rows"), nb::arg("cols"),
        "Return the (rows, cols, 2) map from rectified pixels to the original image.");
  m.def("match", &match, nb::arg("left_levels").noconvert(), nb::arg("right_levels").noconvert(),
        nb::arg("left"), nb::arg("right"), nb::arg("focal"), nb::arg("baseline"),
        nb::arg("options"), "Match features along rows; return a dict of StereoMatches' fields.");
}

}  // namespace slam_it::bindings
