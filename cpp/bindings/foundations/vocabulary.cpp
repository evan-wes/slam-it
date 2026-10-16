/// @file
/// @brief Bindings for `slam_it.foundations.vocabulary` (`slam_it._cpp.foundations.vocabulary`).
///
/// A vocabulary crosses as a dict of `Vocabulary`'s fields (`k`, `levels`, and its five arrays:
/// `descriptors` (N, 32) uint8, `parents` (N,) int32, `children` (N, 2) int32, `words` (N,)
/// int32, `weights` (W,) float64), copied in per call; descriptors as a C-contiguous (n, 32)
/// uint8 array, words as int64.
#include "slam_it/foundations/vocabulary.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
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
  m.def("quantize", &quantize, nb::arg("vocabulary"), nb::arg("descriptors").noconvert(),
        "Return each descriptor's word and its node at the direct index's level.");
  m.def("bow", &bow, nb::arg("vocabulary"), nb::arg("words").noconvert(),
        "Return a frame's word vector.");
  m.def("score", &score, nb::arg("ids1").noconvert(), nb::arg("values1").noconvert(),
        nb::arg("ids2").noconvert(), nb::arg("values2").noconvert(),
        "Return two word vectors' L1 score.");
}

}  // namespace slam_it::bindings
