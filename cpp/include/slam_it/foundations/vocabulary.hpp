/// @file
/// @brief Place recognition's vocabulary: the C++ core's mirror of
///        `slam_it.foundations.vocabulary`.
///
/// A tree of binary words over ORB descriptors (DBoW2's): a descriptor's word is the leaf
/// reached from the root by the nearest child at each level; a frame's vector is each word's
/// count times its weight, L1-normalised; two vectors score DBoW2's L1 score. The Python module's
/// docstring defines each step and the files. Training is Python's alone; this core reads a
/// vocabulary's five `.npy` arrays (Python checks `vocabulary.json`'s versions and checksums
/// before a native run is given the folder), and quantises, weighs and scores as Python does,
/// adding the same terms in the same order.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/orb.hpp"

/// @brief Place recognition's vocabulary.
namespace slam_it::foundations::vocabulary {

/// @brief The tree level whose nodes group a frame's features for matching (the direct index).
inline constexpr std::int64_t kDirectLevel = 2;
/// @brief The most children a node may have.
inline constexpr std::int64_t kMaxK = 64;
/// @brief The deepest a tree may be.
inline constexpr std::int64_t kMaxLevels = 8;

/// @brief A tree of binary words (`slam_it.foundations.vocabulary.Vocabulary`).
struct Vocabulary {
  /// @brief The most children a node has.
  std::int64_t k{0};
  /// @brief The deepest a leaf lies (the root is level 0).
  std::int64_t levels{0};
  /// @brief Each node's centre.
  std::vector<orb::Descriptor> descriptors;
  /// @brief Each node's parent; -1 for the root, node 0.
  std::vector<std::int32_t> parents;
  /// @brief Each node's first child and its count; a leaf's count is 0.
  std::vector<std::array<std::int32_t, 2>> children;
  /// @brief Each leaf's word, numbered in node order; -1 for an inner node.
  std::vector<std::int32_t> words;
  /// @brief Each word's weight.
  std::vector<double> weights;
};

/// @brief Each descriptor's word and its node at `kDirectLevel` (or its leaf, if higher).
struct Quantized {
  /// @brief The words.
  std::vector<std::int64_t> words;
  /// @brief The direct index's nodes.
  std::vector<std::int64_t> nodes;
};

/// @brief A frame's word vector: its weighted words, ascending, their values summing to 1.
struct BowVector {
  /// @brief The words.
  std::vector<std::int64_t> ids;
  /// @brief Their values.
  std::vector<double> values;
};

/// @brief Check a tree as `Vocabulary.__post_init__` does.
/// @param vocabulary The tree.
/// @return Nothing, or `InvalidInput` naming what is malformed.
[[nodiscard]] Result<void> check(const Vocabulary& vocabulary);

/// @brief Read a vocabulary's arrays from its folder; `k` and `levels` from the tree itself.
/// @param folder The folder `slam_it.foundations.vocabulary.write` wrote.
/// @return The vocabulary, checked; or `InvalidInput` for a missing or malformed array.
[[nodiscard]] Result<Vocabulary> read(const std::string& folder);

/// @brief Each descriptor's word, and its node at `kDirectLevel`.
/// @param vocabulary The vocabulary, checked.
/// @param descriptors The descriptors.
/// @return The words and nodes: at each level the nearest child by Hamming distance, the
///         lowest index among equals.
[[nodiscard]] Quantized quantize(const Vocabulary& vocabulary,
                                 std::span<const orb::Descriptor> descriptors);

/// @brief A frame's word vector: each word's count times its weight, summed in word order and
///        normalised; words of weight 0 left out.
/// @param vocabulary The vocabulary.
/// @param words The frame's features' words.
/// @return The vector, or `InvalidInput` for a word outside the vocabulary.
[[nodiscard]] Result<BowVector> bow(const Vocabulary& vocabulary,
                                    std::span<const std::int64_t> words);

/// @brief DBoW2's L1 score of two vectors: `(|v| + |w| - |v - w|) / 2` summed over their shared
///        words in ascending order.
/// @param first A vector.
/// @param second Another.
/// @return From 0 to 1; or `InvalidInput` for words and values of different lengths, or words
///         that do not ascend.
[[nodiscard]] Result<double> score(const BowVector& first, const BowVector& second);

}  // namespace slam_it::foundations::vocabulary
