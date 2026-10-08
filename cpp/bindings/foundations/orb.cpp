/// @file
/// @brief Bindings for `slamlab.foundations.orb` (`slamlab._cpp.foundations.orb`).
///
/// Options cross as one dict of `slamlab.foundations.orb.Options`' fields; keypoints and
/// features come back as dicts (orb_crossing.hpp).
#include "slamlab/foundations/orb.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/image_crossing.hpp"
#include "foundations/orb_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

/// @brief A read-only, contiguous (N, 2) int64 array of pixels.
using PixelsIn = nb::ndarray<const std::int64_t, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;

/// @brief `orb::Options`' fields, as the dict's keys.
constexpr std::array<const char*, 5> kOrbOptionKeys = {"n_features", "levels", "scale_factor",
                                                       "fast_threshold", "min_fast_threshold"};

/// @brief Read the options dict.
/// @param options The dict.
/// @return The settings.
/// @throws std::invalid_argument (Python `ValueError`) for a missing or unknown key.
foundations::orb::Options orb_options(const nb::dict& options) {
  check_keys(options, kOrbOptionKeys, "orb.Options");
  return foundations::orb::Options{
      .n_features = nb::cast<std::int64_t>(options["n_features"]),
      .levels = nb::cast<std::int64_t>(options["levels"]),
      .scale_factor = nb::cast<double>(options["scale_factor"]),
      .fast_threshold = nb::cast<std::int64_t>(options["fast_threshold"]),
      .min_fast_threshold = nb::cast<std::int64_t>(options["min_fast_threshold"])};
}

/// @brief Copy int64 values into a new 1D array.
/// @param values The values.
/// @return The array.
nb::ndarray<nb::numpy, std::int64_t> int64_array(const std::vector<std::int64_t>& values) {
  auto [out, data] = new_array<std::int64_t>({values.size()});
  std::ranges::copy(values, data);
  return out;
}

/// @brief Every pixel's FAST-9 score.
/// @param image The image.
/// @return A new (H, W) int64 array.
/// @throws CoreError for an invalid image.
nb::ndarray<nb::numpy, std::int64_t> fast_scores(const ImageIn& image) {
  const std::vector<std::int64_t> scores = unwrap(foundations::orb::fast_scores(image_in(image)));
  auto [out, data] = new_array<std::int64_t>({image.shape(0), image.shape(1)});
  std::ranges::copy(scores, data);
  return out;
}

/// @brief Spread corners with the quadtree.
/// @param xy The corners' pixels.
/// @param response Their strengths.
/// @param bounds `(x_min, x_max, y_min, y_max)`.
/// @param target How many to keep.
/// @return The kept indices, a new int64 array.
/// @throws CoreError for invalid input.
nb::ndarray<nb::numpy, std::int64_t> distribute(const PixelsIn& xy,
                                                const VectorIn<std::int64_t>& response,
                                                const std::array<std::int64_t, 4>& bounds,
                                                std::int64_t target) {
  std::vector<std::array<std::int64_t, 2>> pixels(xy.shape(0));
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = {xy(i, 0), xy(i, 1)};
  }
  const std::span<const std::int64_t> strengths(response.data(), response.shape(0));
  const foundations::orb::Bounds box{
      .x_min = bounds[0], .x_max = bounds[1], .y_min = bounds[2], .y_max = bounds[3]};
  return int64_array(unwrap(foundations::orb::distribute(pixels, strengths, box, target)));
}

/// @brief Detect oriented keypoints.
/// @param image The image.
/// @param options The settings, as a dict.
/// @return A dict of `Keypoints`' fields.
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
nb::dict detect(const ImageIn& image, const nb::dict& options) {
  return keypoints_dict(unwrap(foundations::orb::detect(image_in(image), orb_options(options))));
}

/// @brief Extract features.
/// @param image The image.
/// @param options The settings, as a dict.
/// @return A dict of `Features`' fields (orb_crossing.hpp).
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
nb::dict extract(const ImageIn& image, const nb::dict& options) {
  return features_dict(unwrap(foundations::orb::extract(image_in(image), orb_options(options))));
}

/// @brief Extract features from a pyramid already built.
/// @param levels The pyramid's levels.
/// @param options The settings, as a dict.
/// @return A dict of `Features`' fields (orb_crossing.hpp).
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
nb::dict extract_levels(const std::vector<ImageIn>& levels, const nb::dict& options) {
  return features_dict(
      unwrap(foundations::orb::extract_levels(levels_in(levels), orb_options(options))));
}

}  // namespace

void bind_orb(nb::module_& m) {
  m.def("fast_scores", &fast_scores, nb::arg("image").noconvert(),
        "Return every pixel's FAST-9 score.");
  m.def("distribute", &distribute, nb::arg("xy").noconvert(), nb::arg("response").noconvert(),
        nb::arg("bounds"), nb::arg("target"),
        "Return the indices of the corners a quadtree keeps.");
  m.def("detect", &detect, nb::arg("image").noconvert(), nb::arg("options"),
        "Detect oriented keypoints; return a dict of Keypoints' fields.");
  m.def("extract", &extract, nb::arg("image").noconvert(), nb::arg("options"),
        "Extract features: Keypoints' fields, and descriptors.");
  m.def("extract_levels", &extract_levels, nb::arg("levels"), nb::arg("options"),
        "Extract features from a pyramid already built: Keypoints' fields, and descriptors.");
}

}  // namespace slamlab::bindings
