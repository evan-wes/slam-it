/// @file
/// @brief Bindings for `slamlab.foundations.stereo` (`slamlab._cpp.foundations.stereo`).
///
/// Camera models cross as their integer codes; a rectification comes back as a dict of its
/// rotations, rectified intrinsics and baseline. Features cross as dicts of `xy`, `level`,
/// `level_xy` and `descriptors`; settings as dicts of their dataclasses' fields.
#include "slamlab/foundations/stereo.hpp"

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

namespace nb = nanobind;

namespace slamlab::bindings {

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

/// @brief `orb::Options`' fields, as the dict's keys.
constexpr std::array<const char*, 5> kOrbKeys = {"n_features", "levels", "scale_factor",
                                                 "fast_threshold", "min_fast_threshold"};

/// @brief A read-only, contiguous array of the given element type and shape.
/// @tparam T The element type.
/// @tparam Shape The nanobind shape.
template <typename T, typename Shape>
using In = nb::ndarray<const T, Shape, nb::c_contig, nb::device::cpu>;

/// @brief Read one camera's features from a dict of arrays.
/// @param features The dict: `xy` (N, 2) float64, `level` (N,) and `level_xy` (N, 2) int64,
///        `descriptors` (N, 32) uint8.
/// @return The features (responses and angles, unused by matching, are zero).
/// @throws std::invalid_argument for a missing key or a malformed array.
foundations::orb::Features features_in(const nb::dict& features) {
  check_keys(features, std::array<const char*, 4>{"xy", "level", "level_xy", "descriptors"},
             "the features");
  const auto xy = nb::cast<In<double, nb::shape<-1, 2>>>(features["xy"]);
  const auto level = nb::cast<In<std::int64_t, nb::shape<-1>>>(features["level"]);
  const auto level_xy = nb::cast<In<std::int64_t, nb::shape<-1, 2>>>(features["level_xy"]);
  const auto descriptors =
      nb::cast<In<std::uint8_t, nb::shape<-1, foundations::orb::kDescriptorBytes>>>(
          features["descriptors"]);
  const std::size_t n = xy.shape(0);
  if (level.shape(0) != n || level_xy.shape(0) != n || descriptors.shape(0) != n) {
    throw std::invalid_argument("features: expected one row per keypoint in every array");
  }
  foundations::orb::Features out;
  out.keypoints.resize(n);
  out.descriptors.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    out.keypoints[i] = {.xy = {xy(i, 0), xy(i, 1)},
                        .level = level(i),
                        .level_xy = {level_xy(i, 0), level_xy(i, 1)},
                        .response = 0,
                        .angle = 0};
    for (std::size_t b = 0; b < foundations::orb::kDescriptorBytes; ++b) {
      out.descriptors[i].at(b) = descriptors(i, b);
    }
  }
  return out;
}

/// @brief Match features along the rows of a rectified pair.
/// @param left_image The rectified left image.
/// @param right_image The rectified right image.
/// @param left The left features, as a dict.
/// @param right The right features, as a dict.
/// @param focal The rectified focal length.
/// @param baseline The baseline.
/// @param orb_options The extraction settings, as a dict.
/// @param options The matching settings, as a dict.
/// @return A dict of `StereoMatches`' fields.
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the Python signature's order
nb::dict match(const ImageIn& left_image, const ImageIn& right_image, const nb::dict& left,
               const nb::dict& right, double focal, double baseline, const nb::dict& orb_options,
               const nb::dict& options) {
  check_keys(orb_options, kOrbKeys, "orb.Options");
  check_keys(options, kStereoOptionKeys, "stereo.Options");
  const foundations::orb::Options extraction{
      .n_features = nb::cast<std::int64_t>(orb_options["n_features"]),
      .levels = nb::cast<std::int64_t>(orb_options["levels"]),
      .scale_factor = nb::cast<double>(orb_options["scale_factor"]),
      .fast_threshold = nb::cast<std::int64_t>(orb_options["fast_threshold"]),
      .min_fast_threshold = nb::cast<std::int64_t>(orb_options["min_fast_threshold"])};
  const foundations::stereo::MatchOptions settings{
      .max_distance = nb::cast<std::int64_t>(options["max_distance"]),
      .band = nb::cast<double>(options["band"]),
      .window = nb::cast<std::int64_t>(options["window"]),
      .search = nb::cast<std::int64_t>(options["search"]),
      .outlier_factor = nb::cast<double>(options["outlier_factor"])};
  const std::vector<foundations::stereo::StereoMatch> matches = unwrap(
      foundations::stereo::match(image_in(left_image), image_in(right_image), features_in(left),
                                 features_in(right), focal, baseline, extraction, settings));
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
  m.def("match", &match, nb::arg("left_image").noconvert(), nb::arg("right_image").noconvert(),
        nb::arg("left"), nb::arg("right"), nb::arg("focal"), nb::arg("baseline"),
        nb::arg("orb_options"), nb::arg("options"),
        "Match features along rows; return a dict of StereoMatches' fields.");
}

}  // namespace slamlab::bindings
