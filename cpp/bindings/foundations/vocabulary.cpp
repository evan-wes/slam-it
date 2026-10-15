/// @file
/// @brief Bindings for `slam_it.foundations.vocabulary` (`slam_it._cpp.foundations.vocabulary`).
///
/// A vocabulary crosses as a dict of `Vocabulary`'s fields (`k`, `levels`, and its five arrays:
/// `descriptors` (N, 32) uint8, `parents` (N,) int32, `children` (N, 2) int32, `words` (N,)
/// int32, `weights` (W,) float64), copied in per call; descriptors as a C-contiguous (n, 32)
/// uint8 array, words as int64. `read` returns the dict the core read from a folder.
#include "slam_it/foundations/vocabulary.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/vocabulary_crossing.hpp"

namespace nb = nanobind;

namespace slam_it::bindings {

namespace {

namespace vocabulary = foundations::vocabulary;
namespace orb = foundations::orb;

using vocabulary_crossing::descriptors_in;
using vocabulary_crossing::DescriptorsIn;
using vocabulary_crossing::DoubleIn;
using vocabulary_crossing::Int64In;
using vocabulary_crossing::vocabulary_in;

/// @brief An int64 vector out.
/// @param values The values.
/// @return A new (n,) int64 array.
nb::ndarray<nb::numpy, std::int64_t> int64_out(const std::vector<std::int64_t>& values) {
  auto [array, data] = new_array<std::int64_t>({values.size()});
  if (!values.empty()) {
    std::memcpy(data, values.data(), values.size() * sizeof(std::int64_t));
  }
  return array;
}

/// @brief Read a vocabulary's folder.
/// @param folder The folder.
/// @return The dict of `Vocabulary`'s fields.
nb::dict read(const std::string& folder) {
  const vocabulary::Vocabulary v = unwrap(vocabulary::read(folder));
  const std::size_t n = v.parents.size();
  auto [descriptors, descriptor_data] = new_array<std::uint8_t>({n, orb::kDescriptorBytes});
  auto [parents, parent_data] = new_array<std::int32_t>({n});
  auto [children, child_data] = new_array<std::int32_t>({n, 2});
  auto [words, word_data] = new_array<std::int32_t>({n});
  auto [weights, weight_data] = new_array<double>({v.weights.size()});
  for (std::size_t i = 0; i < n; ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    std::memcpy(descriptor_data + (i * orb::kDescriptorBytes), v.descriptors[i].data(),
                orb::kDescriptorBytes);
    parent_data[i] = v.parents[i];
    child_data[2 * i] = v.children[i][0];
    child_data[(2 * i) + 1] = v.children[i][1];
    word_data[i] = v.words[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  if (!v.weights.empty()) {
    std::memcpy(weight_data, v.weights.data(), v.weights.size() * sizeof(double));
  }
  nb::dict out;
  out["k"] = v.k;
  out["levels"] = v.levels;
  out["descriptors"] = descriptors;
  out["parents"] = parents;
  out["children"] = children;
  out["words"] = words;
  out["weights"] = weights;
  return out;
}

/// @brief Quantise descriptors.
/// @param fields The vocabulary's dict.
/// @param descriptors The descriptors.
/// @return A dict with `words` and `nodes`, int64.
nb::dict quantize(const nb::dict& fields, const DescriptorsIn& descriptors) {
  const vocabulary::Vocabulary v = vocabulary_in(fields);
  const std::vector<orb::Descriptor> in = descriptors_in(descriptors);
  const vocabulary::Quantized q = vocabulary::quantize(v, in);
  nb::dict out;
  out["words"] = int64_out(q.words);
  out["nodes"] = int64_out(q.nodes);
  return out;
}

/// @brief A frame's word vector.
/// @param fields The vocabulary's dict.
/// @param words The frame's words.
/// @return A dict with `ids` (int64) and `values` (float64).
nb::dict bow(const nb::dict& fields, const Int64In& words) {
  const vocabulary::Vocabulary v = vocabulary_in(fields);
  const vocabulary::BowVector vector =
      unwrap(vocabulary::bow(v, std::span<const std::int64_t>(words.data(), words.shape(0))));
  auto [values, value_data] = new_array<double>({vector.values.size()});
  if (!vector.values.empty()) {
    std::memcpy(value_data, vector.values.data(), vector.values.size() * sizeof(double));
  }
  nb::dict out;
  out["ids"] = int64_out(vector.ids);
  out["values"] = values;
  return out;
}

/// @brief Two word vectors' L1 score.
/// @param ids1 The first's words.
/// @param values1 Its values.
/// @param ids2 The second's words.
/// @param values2 Its values.
/// @return The score.
double score(const Int64In& ids1, const DoubleIn& values1, const Int64In& ids2,
             const DoubleIn& values2) {
  const vocabulary::BowVector first{
      .ids = std::vector<std::int64_t>(ids1.data(), ids1.data() + ids1.shape(0)),
      .values = std::vector<double>(values1.data(), values1.data() + values1.shape(0))};
  const vocabulary::BowVector second{
      .ids = std::vector<std::int64_t>(ids2.data(), ids2.data() + ids2.shape(0)),
      .values = std::vector<double>(values2.data(), values2.data() + values2.shape(0))};
  return unwrap(vocabulary::score(first, second));
}

}  // namespace

void bind_vocabulary(nb::module_& m) {
  m.def("read", &read, nb::arg("folder"), "Read a vocabulary's folder.");
  m.def("quantize", &quantize, nb::arg("vocabulary"), nb::arg("descriptors").noconvert(),
        "Return each descriptor's word and its node at the direct index's level.");
  m.def("bow", &bow, nb::arg("vocabulary"), nb::arg("words").noconvert(),
        "Return a frame's word vector.");
  m.def("score", &score, nb::arg("ids1").noconvert(), nb::arg("values1").noconvert(),
        nb::arg("ids2").noconvert(), nb::arg("values2").noconvert(),
        "Return two word vectors' L1 score.");
}

}  // namespace slam_it::bindings
