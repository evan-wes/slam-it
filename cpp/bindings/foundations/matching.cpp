/// @file
/// @brief Bindings for `slamlab.foundations.matching` (`slamlab._cpp.foundations.matching`).
///
/// Descriptor sets cross as C-contiguous (N, 32) uint8 arrays, orientation bins as int64
/// arrays, and the filters as one dict of `slamlab.foundations.matching.Options`' fields;
/// matches come back as a dict of `Matches`' fields.
#include "slamlab/foundations/matching.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

/// @brief A read-only, contiguous (N, 32) uint8 descriptor set.
using DescriptorsIn =
    nb::ndarray<const std::uint8_t, nb::shape<-1, foundations::orb::kDescriptorBytes>, nb::c_contig,
                nb::device::cpu>;

/// @brief `matching::Options`' fields, as the dict's keys.
constexpr std::array<const char*, 3> kMatchingOptionKeys = {"max_distance", "ratio",
                                                            "check_orientation"};

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

}  // namespace

void bind_matching(nb::module_& m) {
  m.def("distances", &distances, nb::arg("descriptors1").noconvert(),
        nb::arg("descriptors2").noconvert(), "Return every pair's Hamming distance.");
  m.def("match", &match, nb::arg("descriptors1").noconvert(), nb::arg("descriptors2").noconvert(),
        nb::arg("angles1").noconvert(), nb::arg("angles2").noconvert(), nb::arg("options"),
        "Match two descriptor sets; return a dict of Matches' fields.");
}

}  // namespace slamlab::bindings
