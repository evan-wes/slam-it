/// @file
/// @brief Bindings for `slam_it_all.foundations.matching`
/// (`slam_it_all._cpp.foundations.matching`).
///
/// Descriptor sets cross as C-contiguous (N, 32) uint8 arrays, orientation bins as int64
/// arrays, and the filters as one dict of `slam_it_all.foundations.matching.Options`' (or
/// `SearchOptions`') fields; matches come back as a dict of `Matches`' fields. A search by
/// projection takes its projections as a dict of `Projections`' fields and the frame's
/// keypoints as a dict of arrays (see `slam_it_all.foundations.matching.Implementation`); the grid
/// is built per call.
#include "slam_it_all/foundations/matching.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slam_it_all::bindings {

namespace {

/// @brief A read-only, contiguous (N, 32) uint8 descriptor set.
using DescriptorsIn =
    nb::ndarray<const std::uint8_t, nb::shape<-1, foundations::orb::kDescriptorBytes>, nb::c_contig,
                nb::device::cpu>;

/// @brief `matching::Options`' fields, as the dict's keys (`SearchOptions` has the same).
constexpr std::array<const char*, 3> kMatchingOptionKeys = {"max_distance", "ratio",
                                                            "check_orientation"};

/// @brief `Projections`' fields, as the dict's keys.
constexpr std::array<const char*, 7> kProjectionKeys = {
    "xy", "radius", "min_level", "max_level", "descriptors", "angle", "right"};

/// @brief The frame's fields a search reads, as the dict's keys.
constexpr std::array<const char*, 7> kFrameKeys = {"xy",    "level", "angle", "descriptors",
                                                   "right", "taken", "levels"};

/// @brief A read-only, contiguous (N, 2) float64 array.
using PositionsIn = nb::ndarray<const double, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu>;

/// @brief Copy a descriptor set in.
/// @param in The array.
/// @return The descriptors.
std::vector<foundations::orb::Descriptor> descriptors_in(const DescriptorsIn& in) {
  std::vector<foundations::orb::Descriptor> out(in.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    for (std::size_t b = 0; b < foundations::orb::kDescriptorBytes; ++b) {
      out[i].at(b) = in(i, b);
    }
  }
  return out;
}

/// @brief Wrap matches as a dict of `Matches`' fields.
/// @param matches The matches.
/// @return A dict with `pairs` (K, 2) and `distances` (K,), both int64.
nb::dict matches_dict(const std::vector<foundations::matching::Match>& matches) {
  auto [pairs, pair_data] = new_array<std::int64_t>({matches.size(), 2});
  auto [distance, distance_data] = new_array<std::int64_t>({matches.size()});
  for (std::size_t k = 0; k < matches.size(); ++k) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    pair_data[2 * k] = matches[k].first;
    pair_data[(2 * k) + 1] = matches[k].second;
    distance_data[k] = matches[k].distance;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  nb::dict out;
  out["pairs"] = pairs;
  out["distances"] = distance;
  return out;
}

/// @brief Every pair's Hamming distance.
/// @param descriptors1 The first set.
/// @param descriptors2 The second set.
/// @return A new (N, M) int64 array.
/// @throws CoreError for a set too large.
nb::ndarray<nb::numpy, std::int64_t> distances(const DescriptorsIn& descriptors1,
                                               const DescriptorsIn& descriptors2) {
  const std::vector<std::int64_t> values = unwrap(
      foundations::matching::distances(descriptors_in(descriptors1), descriptors_in(descriptors2)));
  auto [out, data] = new_array<std::int64_t>({descriptors1.shape(0), descriptors2.shape(0)});
  std::ranges::copy(values, data);
  return out;
}

/// @brief Match two descriptor sets.
/// @param descriptors1 The first set.
/// @param descriptors2 The second set.
/// @param angles1 The first set's orientation bins.
/// @param angles2 The second set's.
/// @param options The filters, as a dict.
/// @return A dict with `pairs` (K, 2) and `distances` (K,), both int64.
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
nb::dict match(const DescriptorsIn& descriptors1, const DescriptorsIn& descriptors2,
               const VectorIn<std::int64_t>& angles1, const VectorIn<std::int64_t>& angles2,
               const nb::dict& options) {
  check_keys(options, kMatchingOptionKeys, "matching.Options");
  const foundations::matching::Options settings{
      .max_distance = nb::cast<std::int64_t>(options["max_distance"]),
      .ratio = nb::cast<double>(options["ratio"]),
      .check_orientation = nb::cast<bool>(options["check_orientation"])};
  const std::vector<foundations::matching::Match> matches = unwrap(foundations::matching::match(
      descriptors_in(descriptors1), descriptors_in(descriptors2),
      std::span<const std::int64_t>(angles1.data(), angles1.shape(0)),
      std::span<const std::int64_t>(angles2.data(), angles2.shape(0)), settings));
  return matches_dict(matches);
}

/// @brief Read keypoints' positions and levels (the rest zero).
/// @param xy The positions.
/// @param level The levels.
/// @return The keypoints.
/// @throws std::invalid_argument if the arrays disagree in length.
std::vector<foundations::orb::Keypoint> keypoints_in(const PositionsIn& xy,
                                                     const VectorIn<std::int64_t>& level) {
  if (level.shape(0) != xy.shape(0)) {
    throw std::invalid_argument("keypoints: expected one level per position");
  }
  std::vector<foundations::orb::Keypoint> out(xy.shape(0));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i].xy = {xy(i, 0), xy(i, 1)};
    out[i].level = level(i);
  }
  return out;
}

/// @brief The keypoints in a square window, on a range of levels.
/// @param xy The keypoints' positions.
/// @param level Their levels.
/// @param center The window's centre.
/// @param radius Its half-width.
/// @param min_level The lowest level it covers.
/// @param max_level The highest.
/// @return A new (K,) int64 array of indices, ascending.
/// @throws CoreError for invalid input; std::invalid_argument for mismatched arrays.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): the levels' range, as in Python
nb::ndarray<nb::numpy, std::int64_t> features_in_area(const PositionsIn& xy,
                                                      const VectorIn<std::int64_t>& level,
                                                      const std::array<double, 2>& center,
                                                      double radius, std::int64_t min_level,
                                                      std::int64_t max_level) {
  const std::vector<std::int64_t> found = unwrap(foundations::matching::features_in_area(
      keypoints_in(xy, level), center, radius, min_level, max_level));
  auto [out, data] = new_array<std::int64_t>({found.size()});
  std::ranges::copy(found, data);
  return out;
}

/// @brief Read one array item of a dict, with its expected type and shape.
/// @tparam Array The nanobind array type.
/// @param values The dict.
/// @param key The item's key.
/// @return The array.
/// @throws nb::cast_error if the item is not such an array.
template <typename Array>
Array item(const nb::dict& values, const char* key) {
  return nb::cast<Array>(values[key]);
}

/// @brief Read projections from a dict of `Projections`' fields.
/// @param values The dict.
/// @return The projections.
/// @throws std::invalid_argument for a missing or unknown key or mismatched arrays.
std::vector<foundations::matching::Projection> projections_in(const nb::dict& values) {
  check_keys(values, kProjectionKeys, "matching.Projections");
  const auto xy = item<PositionsIn>(values, "xy");
  const auto radius = item<VectorIn<double>>(values, "radius");
  const auto min_level = item<VectorIn<std::int64_t>>(values, "min_level");
  const auto max_level = item<VectorIn<std::int64_t>>(values, "max_level");
  const auto descriptors = item<DescriptorsIn>(values, "descriptors");
  const auto angle = item<VectorIn<std::int64_t>>(values, "angle");
  const auto right = item<VectorIn<double>>(values, "right");
  const std::size_t m = xy.shape(0);
  if (radius.shape(0) != m || min_level.shape(0) != m || max_level.shape(0) != m ||
      descriptors.shape(0) != m || angle.shape(0) != m || right.shape(0) != m) {
    throw std::invalid_argument("projections: expected one row per projection in every array");
  }
  std::vector<foundations::matching::Projection> out(m);
  for (std::size_t i = 0; i < m; ++i) {
    foundations::matching::Projection& p = out[i];
    p.xy = {xy(i, 0), xy(i, 1)};
    p.radius = radius(i);
    p.min_level = min_level(i);
    p.max_level = max_level(i);
    for (std::size_t b = 0; b < foundations::orb::kDescriptorBytes; ++b) {
      p.descriptor.at(b) = descriptors(i, b);
    }
    p.angle = angle(i);
    p.right = right(i);
  }
  return out;
}

/// @brief A frame as a search reads it.
struct FrameIn {
  /// @brief The features (responses and level pixels zero; the scale factor unused).
  foundations::orb::Features features;
  /// @brief The right columns.
  std::vector<double> right;
  /// @brief The taken flags, 0 or 1.
  std::vector<std::uint8_t> taken;
};

/// @brief Read a frame from a dict.
/// @param values The dict: `xy`, `level`, `angle`, `descriptors`, `right`, `taken`, `levels`.
/// @return The frame.
/// @throws std::invalid_argument for a missing or unknown key or mismatched arrays.
FrameIn frame_in(const nb::dict& values) {
  check_keys(values, kFrameKeys, "the frame's keypoints");
  const auto xy = item<PositionsIn>(values, "xy");
  const auto level = item<VectorIn<std::int64_t>>(values, "level");
  const auto angle = item<VectorIn<std::int64_t>>(values, "angle");
  const auto right = item<VectorIn<double>>(values, "right");
  const auto taken = item<VectorIn<bool>>(values, "taken");
  const std::size_t n = xy.shape(0);
  if (angle.shape(0) != n || right.shape(0) != n || taken.shape(0) != n) {
    throw std::invalid_argument("keypoints: expected one row per keypoint in every array");
  }
  FrameIn out{
      .features = {.keypoints = keypoints_in(xy, level),
                   .descriptors = descriptors_in(item<DescriptorsIn>(values, "descriptors")),
                   .scale_factor = 0.0,
                   .levels = nb::cast<std::int64_t>(values["levels"])},
      .right = std::vector<double>(n),
      .taken = std::vector<std::uint8_t>(n)};
  for (std::size_t i = 0; i < n; ++i) {
    out.features.keypoints[i].angle = angle(i);
    out.right[i] = right(i);
    out.taken[i] = taken(i) ? 1 : 0;
  }
  return out;
}

/// @brief Match projected map points to a frame's keypoints.
/// @param projections A dict of `Projections`' fields.
/// @param keypoints The frame's keypoints, as a dict.
/// @param options The filters, as a dict of `SearchOptions`' fields.
/// @return A dict with `pairs` (K, 2) and `distances` (K,), both int64.
/// @throws CoreError for invalid input; std::invalid_argument for a malformed dict.
nb::dict search_by_projection(const nb::dict& projections, const nb::dict& keypoints,
                              const nb::dict& options) {
  check_keys(options, kMatchingOptionKeys, "matching.SearchOptions");
  const foundations::matching::SearchOptions settings{
      .max_distance = nb::cast<std::int64_t>(options["max_distance"]),
      .ratio = nb::cast<double>(options["ratio"]),
      .check_orientation = nb::cast<bool>(options["check_orientation"])};
  const std::vector<foundations::matching::Projection> projected = projections_in(projections);
  const FrameIn frame = frame_in(keypoints);
  const auto grid = unwrap(foundations::matching::FeatureGrid::build(frame.features.keypoints));
  return matches_dict(unwrap(foundations::matching::search_by_projection(
      projected, frame.features, grid, frame.right, frame.taken, settings)));
}

}  // namespace

void bind_matching(nb::module_& m) {
  m.def("distances", &distances, nb::arg("descriptors1").noconvert(),
        nb::arg("descriptors2").noconvert(), "Return every pair's Hamming distance.");
  m.def("match", &match, nb::arg("descriptors1").noconvert(), nb::arg("descriptors2").noconvert(),
        nb::arg("angles1").noconvert(), nb::arg("angles2").noconvert(), nb::arg("options"),
        "Match two descriptor sets; return a dict of Matches' fields.");
  m.def("features_in_area", &features_in_area, nb::arg("xy").noconvert(),
        nb::arg("level").noconvert(), nb::arg("center"), nb::arg("radius"), nb::arg("min_level"),
        nb::arg("max_level"), "Return the indices of the keypoints in a window, ascending.");
  m.def("search_by_projection", &search_by_projection, nb::arg("projections"), nb::arg("keypoints"),
        nb::arg("options"),
        "Match projected map points to a frame's keypoints; a dict of Matches' fields.");
}

}  // namespace slam_it_all::bindings
