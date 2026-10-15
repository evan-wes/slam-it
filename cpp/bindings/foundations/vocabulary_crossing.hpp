/// @file
/// @brief Shared by every binding that takes a vocabulary: it crosses as a dict of
///        `slam_it.foundations.vocabulary.Vocabulary`'s fields (`k`, `levels`, and its five
///        arrays: `descriptors` (N, 32) uint8, `parents` (N,) int32, `children` (N, 2) int32,
///        `words` (N,) int32, `weights` (W,) float64), copied in and checked.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "common.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/vocabulary.hpp"

namespace slam_it::bindings::vocabulary_crossing {

namespace nb = nanobind;
namespace vocabulary = foundations::vocabulary;
namespace orb = foundations::orb;

/// @brief A read-only, contiguous (N, 32) uint8 descriptor set.
using DescriptorsIn = nb::ndarray<const std::uint8_t, nb::shape<-1, orb::kDescriptorBytes>,
                                  nb::c_contig, nb::device::cpu>;
/// @brief A read-only, contiguous int32 array of `Dims` (one free leading dimension).
template <int... Dims>
using Int32In =
    nb::ndarray<const std::int32_t, nb::shape<-1, Dims...>, nb::c_contig, nb::device::cpu>;
/// @brief A read-only, contiguous int64 vector.
using Int64In = nb::ndarray<const std::int64_t, nb::shape<-1>, nb::c_contig, nb::device::cpu>;
/// @brief A read-only, contiguous float64 vector.
using DoubleIn = nb::ndarray<const double, nb::shape<-1>, nb::c_contig, nb::device::cpu>;

/// @brief Copy a descriptor set in.
/// @param in The array.
/// @return The descriptors.
inline std::vector<orb::Descriptor> descriptors_in(const DescriptorsIn& in) {
  std::vector<orb::Descriptor> out(in.shape(0));
  if (!out.empty()) {
    std::memcpy(out.data(), in.data(), out.size() * orb::kDescriptorBytes);
  }
  return out;
}

/// @brief Copy a vocabulary's dict in, checked.
/// @param fields The dict of `Vocabulary`'s fields.
/// @return The vocabulary.
inline vocabulary::Vocabulary vocabulary_in(const nb::dict& fields) {
  vocabulary::Vocabulary out;
  out.k = nb::cast<std::int64_t>(fields["k"]);
  out.levels = nb::cast<std::int64_t>(fields["levels"]);
  out.descriptors = descriptors_in(nb::cast<DescriptorsIn>(fields["descriptors"]));
  const auto parents = nb::cast<Int32In<>>(fields["parents"]);
  out.parents.assign(parents.data(), parents.data() + parents.shape(0));
  const auto children = nb::cast<Int32In<2>>(fields["children"]);
  out.children.resize(children.shape(0));
  for (std::size_t i = 0; i < out.children.size(); ++i) {
    out.children[i] = {children(i, 0), children(i, 1)};
  }
  const auto words = nb::cast<Int32In<>>(fields["words"]);
  out.words.assign(words.data(), words.data() + words.shape(0));
  const auto weights = nb::cast<DoubleIn>(fields["weights"]);
  out.weights.assign(weights.data(), weights.data() + weights.shape(0));
  unwrap(vocabulary::check(out));
  return out;
}

}  // namespace slam_it::bindings::vocabulary_crossing
