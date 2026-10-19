/// @file
/// @brief A vocabulary's folder, read: the C++ mirror of `slam_it.eval.vocabularies.read`
///        (the module's docstring states the folder's files).
///
/// The tree is rung 0's (`slam_it::foundations::vocabulary`), which takes arrays, never files;
/// its folder is read here, for the native programs. The header is checked against this
/// library's format version and this ORB's descriptor version, and the tree as rung 0 checks
/// it. The arrays' SHA-256 are read but not checked: Python checks them (the module docstring's
/// one exception), and they are what a relocalising run records.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/vocabulary.hpp"

/// @brief A vocabulary's folder.
namespace slam_it::eval::vocabularies {

/// @brief The folder's format version (`slam_it.eval.vocabularies.VERSION`).
inline constexpr std::int64_t kVersion = 2;

/// @brief A SHA-256 digest's bytes.
inline constexpr std::size_t kSha256Bytes = 32;

/// @brief The tree's arrays, each a `<name>.npy` (`slam_it.foundations.vocabulary.ARRAYS`).
inline constexpr std::array<const char*, 5> kArrays = {"descriptors", "parents", "children",
                                                       "words", "weights"};

/// @brief A folder's header and checksums: what a relocalising run records of its vocabulary.
struct Header {
  /// @brief The folder's format version.
  std::int64_t version{0};
  /// @brief The version of the descriptor its words were trained on.
  std::int64_t descriptor_version{0};
  /// @brief The most children a node has.
  std::int64_t k{0};
  /// @brief The deepest a leaf lies.
  std::int64_t levels{0};
  /// @brief The number of words.
  std::int64_t words{0};
  /// @brief Each array's SHA-256, in `kArrays`' order.
  std::array<std::array<std::uint8_t, kSha256Bytes>, kArrays.size()> sha256{};
};

/// @brief A folder as read: the vocabulary, and its header.
struct Folder {
  /// @brief The vocabulary, checked.
  foundations::vocabulary::Vocabulary vocabulary;
  /// @brief Its header and checksums.
  Header header;
};

/// @brief Read a vocabulary's folder.
/// @param folder The folder `slam_it.eval.vocabularies.write` wrote.
/// @return The vocabulary and its header; or `InvalidInput` for a missing or malformed file, a
///         header of another format version or descriptor, a malformed tree, or a header whose
///         words are not the tree's.
[[nodiscard]] foundations::Result<Folder> read(const std::string& folder);

}  // namespace slam_it::eval::vocabularies
