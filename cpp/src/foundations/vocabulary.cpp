/// @file
/// @brief Place recognition's vocabulary (see vocabulary.hpp).

#include "slam_it/foundations/vocabulary.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace slam_it::foundations::vocabulary {

namespace {

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
    if (count == 0) {  // a leaf: its first child means nothing
      continue;
    }
    if (first < 1 || std::cmp_greater(std::int64_t{first} + count, n)) {  // no int32 overflow
      return fail(ErrorKind::kInvalidInput,
                  "children: expected every node but the root once, in range");
    }
    const auto span =
        std::span(vocabulary.parents)
            .subspan(static_cast<std::size_t>(first), static_cast<std::size_t>(count));
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
