/// @file
/// @brief Place recognition's vocabulary (see vocabulary.hpp), and a `.npy` reader for its
///        arrays.

#include "slam_it/foundations/vocabulary.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace slam_it::foundations::vocabulary {

namespace {

/// @brief A `.npy` array: its type, shape and raw little-endian values.
struct Npy {
  /// @brief numpy's `descr`, e.g. `|u1`, `<i4`, `<f8`.
  std::string descr;
  /// @brief The shape.
  std::vector<std::size_t> shape;
  /// @brief The values' bytes, C order.
  std::string data;
};

/// @brief The value of a key in a `.npy` header's dict literal, up to its comma or brace.
/// @param header The header.
/// @param key The key, e.g. `'descr'`.
/// @return The value's text, or empty if the key is absent.
std::string header_value(const std::string& header, const std::string& key) {
  const std::size_t at = header.find(key);
  if (at == std::string::npos) {
    return {};
  }
  std::size_t start = header.find(':', at);
  if (start == std::string::npos) {
    return {};
  }
  ++start;
  while (start < header.size() && header[start] == ' ') {
    ++start;
  }
  if (start < header.size() && header[start] == '(') {
    const std::size_t end = header.find(')', start);
    return end == std::string::npos ? std::string{} : header.substr(start, end - start + 1);
  }
  const std::size_t end = header.find_first_of(",}", start);
  return header.substr(start, end - start);
}

/// @brief Where a `.npy` file's header lies, from its magic and version.
/// @param bytes The file.
/// @return The header's offset and length, or nothing for another file or version.
std::optional<std::pair<std::size_t, std::size_t>> header_span(const std::string& bytes) {
  constexpr std::string_view kMagic{"\x93NUMPY", 6};
  if (bytes.size() < 10 || std::string_view(bytes).substr(0, 6) != kMagic) {
    return std::nullopt;
  }
  const auto byte = [&bytes](std::size_t i) {
    return static_cast<std::size_t>(static_cast<unsigned char>(bytes[i]));
  };
  if (byte(6) == 1) {
    return std::pair{std::size_t{10}, byte(8) | (byte(9) << 8U)};
  }
  if (byte(6) == 2 && bytes.size() >= 12) {
    return std::pair{std::size_t{12},
                     byte(8) | (byte(9) << 8U) | (byte(10) << 16U) | (byte(11) << 24U)};
  }
  return std::nullopt;
}

/// @brief A `.npy` header's shape tuple, e.g. `(111070, 32)` or `(99961,)`.
/// @param text The tuple's text.
/// @return The shape, or nothing if a part is not a whole number.
std::optional<std::vector<std::size_t>> parse_shape(const std::string& text) {
  std::vector<std::size_t> out;
  std::string part;
  const std::string inner = text.size() >= 2 ? text.substr(1, text.size() - 2) : std::string{};
  std::istringstream parts(inner);
  while (std::getline(parts, part, ',')) {
    std::erase(part, ' ');
    if (part.empty()) {
      continue;
    }
    if (!std::ranges::all_of(part, [](char c) { return c >= '0' && c <= '9'; })) {
      return std::nullopt;
    }
    out.push_back(std::stoull(part));
  }
  return out;
}

/// @brief The bytes one value of a `.npy` type takes, for the types a vocabulary holds.
/// @param descr numpy's `descr`.
/// @return 1, 4 or 8; or 0 for another type.
std::size_t item_size(const std::string& descr) {
  if (descr == "|u1") {
    return 1;
  }
  if (descr == "<i4") {
    return 4;
  }
  return descr == "<f8" ? 8 : 0;
}

/// @brief Read a `.npy` file of version 1 or 2, C order.
/// @param path The file.
/// @return The array, or `InvalidInput` for a missing or malformed file.
Result<Npy> read_npy(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return fail(ErrorKind::kInvalidInput, path.string() + ": cannot be read");
  }
  const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const auto span = header_span(bytes);
  if (!span || span->first + span->second > bytes.size()) {
    return fail(ErrorKind::kInvalidInput, path.string() + ": not a .npy file of version 1 or 2");
  }
  const std::string header = bytes.substr(span->first, span->second);
  const std::string descr = header_value(header, "'descr'");
  const auto shape = parse_shape(header_value(header, "'shape'"));
  if (descr.size() < 2 || !shape || header_value(header, "'fortran_order'") != "False") {
    return fail(ErrorKind::kInvalidInput, path.string() + ": a malformed or Fortran-order .npy");
  }
  Npy out{.descr = descr.substr(1, descr.size() - 2), .shape = *shape, .data = {}};
  const std::size_t item = item_size(out.descr);
  if (item == 0) {
    return fail(ErrorKind::kInvalidInput, path.string() + ": unexpected .npy type " + out.descr);
  }
  out.data = bytes.substr(span->first + span->second);
  std::size_t count = 1;
  for (const std::size_t extent : out.shape) {
    count *= extent;
  }
  if (out.data.size() != count * item) {
    return fail(ErrorKind::kInvalidInput, path.string() + ": its data do not fill its shape");
  }
  return out;
}

/// @brief An array's values as `T`, checked against its type and shape.
/// @tparam T The element type.
/// @param array The array.
/// @param descr The type it must have.
/// @param shape The shape it must have; a 0 accepts any length on that axis.
/// @param name Its name, for messages.
/// @return The values, or `InvalidInput`.
template <typename T>
Result<std::vector<T>> values_of(const Npy& array, const char* descr,
                                 const std::vector<std::size_t>& shape, const char* name) {
  bool ok = array.descr == descr && array.shape.size() == shape.size();
  for (std::size_t i = 0; ok && i < shape.size(); ++i) {
    ok = shape[i] == 0 || array.shape[i] == shape[i];
  }
  if (!ok) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": unexpected type or shape");
  }
  std::vector<T> out(array.data.size() / sizeof(T));
  std::memcpy(out.data(), array.data.data(), array.data.size());
  return out;
}

/// @brief Each node's level, the root's 0 (each parent before its children).
/// @param parents The parents.
/// @return The levels.
std::vector<std::int64_t> depths(const std::vector<std::int32_t>& parents) {
  std::vector<std::int64_t> out(parents.size(), 0);
  for (std::size_t node = 1; node < parents.size(); ++node) {
    out[node] = out[static_cast<std::size_t>(parents[node])] + 1;
  }
  return out;
}

/// @brief The Hamming distance between two descriptors.
/// @param a A descriptor.
/// @param b Another.
/// @return The bits in which they differ.
std::int64_t hamming(const orb::Descriptor& a, const orb::Descriptor& b) {
  std::int64_t out = 0;
  for (std::size_t i = 0; i < orb::kDescriptorBytes; ++i) {
    out += std::popcount(static_cast<unsigned>(a.at(i) ^ b.at(i)));
  }
  return out;
}

/// @brief Check a word vector: as many values as words, the words ascending, values finite.
/// @param vector The vector.
/// @return Nothing, or `InvalidInput`.
Result<void> check_vector(const BowVector& vector) {
  if (vector.ids.size() != vector.values.size()) {
    return fail(ErrorKind::kInvalidInput, "vector: expected (m,) words and (m,) values");
  }
  if (std::ranges::adjacent_find(vector.ids, std::greater_equal<>()) != vector.ids.end() ||
      !std::ranges::all_of(vector.values, [](double v) { return std::isfinite(v); })) {
    return fail(ErrorKind::kInvalidInput, "vector: expected ascending words and finite values");
  }
  return {};
}

/// @brief Check that every node's parent comes before it, the root's -1.
/// @param parents The parents.
/// @return Nothing, or `InvalidInput`.
Result<void> check_parents(const std::vector<std::int32_t>& parents) {
  if (parents.empty() || parents[0] != -1) {
    return fail(ErrorKind::kInvalidInput, "parents: the root must be node 0, its parent -1");
  }
  for (std::size_t node = 1; node < parents.size(); ++node) {
    if (parents[node] < 0 || std::cmp_greater_equal(parents[node], node)) {
      return fail(ErrorKind::kInvalidInput, "parents: each node's parent must come before it");
    }
  }
  return {};
}

/// @brief Check that each node's children are consecutive, name it, and number at most `k`, and
///        that every node but the root is one node's child.
/// @param vocabulary The tree, its parents checked.
/// @return Nothing, or `InvalidInput`.
Result<void> check_children(const Vocabulary& vocabulary) {
  const std::size_t n = vocabulary.parents.size();
  std::int64_t total = 0;
  for (std::size_t node = 0; node < n; ++node) {
    const auto [first, count] = vocabulary.children[node];
    if (count < 0 || count > vocabulary.k) {
      return fail(ErrorKind::kInvalidInput, "children: a node has from 0 to k children");
    }
    if (count > 0 && (first < 1 || std::cmp_greater(first + count, n))) {
      return fail(ErrorKind::kInvalidInput,
                  "children: expected every node but the root once, in range");
    }
    const auto span =
        std::span(vocabulary.parents)
            .subspan(static_cast<std::size_t>(std::max(first, 0)), static_cast<std::size_t>(count));
    if (!std::ranges::all_of(span, [node](std::int32_t p) { return std::cmp_equal(p, node); })) {
      return fail(ErrorKind::kInvalidInput,
                  "children: a node's children must be consecutive and name it");
    }
    total += count;
  }
  if (std::cmp_not_equal(total, n - 1)) {
    return fail(ErrorKind::kInvalidInput,
                "children: expected every node but the root once, in range");
  }
  return {};
}

/// @brief Check the leaves' depth, their words in node order and a weight each.
/// @param vocabulary The tree, its parents and children checked.
/// @return Nothing, or `InvalidInput`.
Result<void> check_leaves(const Vocabulary& vocabulary) {
  const std::vector<std::int64_t> level = depths(vocabulary.parents);
  std::int32_t next = 0;
  for (std::size_t node = 0; node < level.size(); ++node) {
    const bool leaf = vocabulary.children[node][1] == 0;
    if (leaf && level[node] > vocabulary.levels) {
      return fail(ErrorKind::kInvalidInput, "levels: a leaf lies deeper than the tree's levels");
    }
    if (vocabulary.words[node] != (leaf ? next : -1)) {
      return fail(ErrorKind::kInvalidInput,
                  "words: expected the leaves numbered in node order, one weight each");
    }
    next += leaf ? 1 : 0;
  }
  if (std::cmp_not_equal(vocabulary.weights.size(), next)) {
    return fail(ErrorKind::kInvalidInput,
                "words: expected the leaves numbered in node order, one weight each");
  }
  return {};
}

}  // namespace

Result<void> check(const Vocabulary& vocabulary) {
  if (auto ok = check_range("k", vocabulary.k, 2, kMaxK); !ok) {
    return ok;
  }
  if (auto ok = check_range("levels", vocabulary.levels, 1, kMaxLevels); !ok) {
    return ok;
  }
  const std::size_t n = vocabulary.parents.size();
  if (vocabulary.descriptors.size() != n || vocabulary.children.size() != n ||
      vocabulary.words.size() != n) {
    return fail(ErrorKind::kInvalidInput, "vocabulary: expected one entry per node in each array");
  }
  if (auto ok = check_parents(vocabulary.parents); !ok) {
    return ok;
  }
  if (auto ok = check_children(vocabulary); !ok) {
    return ok;
  }
  if (auto ok = check_leaves(vocabulary); !ok) {
    return ok;
  }
  if (!std::ranges::all_of(vocabulary.weights,
                           [](double w) { return std::isfinite(w) && w >= 0.0; })) {
    return fail(ErrorKind::kInvalidInput, "weights: expected non-negative finite numbers");
  }
  return {};
}

Result<Vocabulary> read(const std::string& folder) {
  const std::filesystem::path root(folder);
  std::map<std::string, Npy> arrays;
  for (const char* name : {"descriptors", "parents", "children", "words", "weights"}) {
    auto array = read_npy(root / (std::string(name) + ".npy"));
    if (!array) {
      return std::unexpected(std::move(array.error()));
    }
    arrays.emplace(name, std::move(*array));
  }
  const std::size_t n = arrays["parents"].shape.empty() ? 0 : arrays["parents"].shape[0];
  auto descriptors = values_of<std::uint8_t>(arrays["descriptors"], "|u1",
                                             {n, orb::kDescriptorBytes}, "descriptors");
  auto parents = values_of<std::int32_t>(arrays["parents"], "<i4", {n}, "parents");
  auto children = values_of<std::int32_t>(arrays["children"], "<i4", {n, 2}, "children");
  auto words = values_of<std::int32_t>(arrays["words"], "<i4", {n}, "words");
  auto weights = values_of<double>(arrays["weights"], "<f8", {0}, "weights");
  if (!descriptors) {
    return std::unexpected(descriptors.error());
  }
  if (!parents) {
    return std::unexpected(parents.error());
  }
  if (!children) {
    return std::unexpected(children.error());
  }
  if (!words) {
    return std::unexpected(words.error());
  }
  if (!weights) {
    return std::unexpected(weights.error());
  }
  Vocabulary out;
  out.descriptors.resize(n);
  const std::span<const std::uint8_t> raw(*descriptors);
  for (std::size_t i = 0; i < n; ++i) {
    std::ranges::copy(raw.subspan(i * orb::kDescriptorBytes, orb::kDescriptorBytes),
                      out.descriptors[i].begin());
  }
  out.parents = std::move(*parents);
  out.children.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    out.children[i] = {(*children)[2 * i], (*children)[(2 * i) + 1]};
  }
  out.words = std::move(*words);
  out.weights = std::move(*weights);
  std::int64_t k = 2;
  for (const auto& [first, count] : out.children) {
    k = std::max<std::int64_t>(k, count);
  }
  out.k = k;
  const std::vector<std::int64_t> level = out.parents.empty() || out.parents[0] != -1
                                              ? std::vector<std::int64_t>{}
                                              : depths(out.parents);
  out.levels = level.empty() ? 1 : std::max<std::int64_t>(1, *std::ranges::max_element(level));
  if (auto ok = check(out); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return out;
}

Quantized quantize(const Vocabulary& vocabulary, std::span<const orb::Descriptor> descriptors) {
  Quantized out;
  out.words.reserve(descriptors.size());
  out.nodes.reserve(descriptors.size());
  for (const orb::Descriptor& descriptor : descriptors) {
    std::size_t node = 0;
    std::size_t direct = 0;
    for (std::int64_t level = 1; level <= vocabulary.levels; ++level) {
      const auto [first, count] = vocabulary.children[node];
      if (count > 0) {
        std::int64_t best = std::numeric_limits<std::int64_t>::max();
        auto chosen = static_cast<std::size_t>(first);
        for (std::int32_t c = first; c < first + count; ++c) {
          const std::int64_t d =
              hamming(descriptor, vocabulary.descriptors[static_cast<std::size_t>(c)]);
          if (d < best) {  // the first of equals, the lowest index
            best = d;
            chosen = static_cast<std::size_t>(c);
          }
        }
        node = chosen;
      }
      if (level == kDirectLevel) {
        direct = node;
      }
    }
    if (vocabulary.levels < kDirectLevel) {
      direct = node;
    }
    out.words.push_back(vocabulary.words[node]);
    out.nodes.push_back(static_cast<std::int64_t>(direct));
  }
  return out;
}

Result<BowVector> bow(const Vocabulary& vocabulary, std::span<const std::int64_t> words) {
  std::map<std::int64_t, std::int64_t> counts;
  for (const std::int64_t word : words) {
    if (word < 0 || std::cmp_greater_equal(word, vocabulary.weights.size())) {
      return fail(ErrorKind::kInvalidInput, "words: expected words of the vocabulary, 0 to " +
                                                std::to_string(vocabulary.weights.size() - 1));
    }
    ++counts[word];
  }
  BowVector out;
  for (const auto& [word, count] : counts) {  // ascending
    const double value =
        static_cast<double>(count) * vocabulary.weights[static_cast<std::size_t>(word)];
    if (value > 0.0) {
      out.ids.push_back(word);
      out.values.push_back(value);
    }
  }
  double total = 0.0;
  for (const double value : out.values) {  // in word order, as Python adds them
    total += value;
  }
  if (total > 0.0) {
    for (double& value : out.values) {
      value /= total;
    }
  }
  return out;
}

Result<double> score(const BowVector& first, const BowVector& second) {
  if (auto ok = check_vector(first); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_vector(second); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  double total = 0.0;
  std::size_t i = 0;
  std::size_t j = 0;
  while (i < first.ids.size() && j < second.ids.size()) {
    if (first.ids[i] < second.ids[j]) {
      ++i;
    } else if (second.ids[j] < first.ids[i]) {
      ++j;
    } else {
      const double v = first.values[i];
      const double w = second.values[j];
      total += std::abs(v) + std::abs(w) - std::abs(v - w);
      ++i;
      ++j;
    }
  }
  return total / 2.0;
}

}  // namespace slam_it::foundations::vocabulary
