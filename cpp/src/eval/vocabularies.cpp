/// @file
/// @brief A vocabulary's folder, read (see vocabularies.hpp), with a `.npy` reader for its
///        arrays.

#include "slam_it/eval/vocabularies.hpp"

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "slam_it/foundations/orb.hpp"

namespace slam_it::eval::vocabularies {

namespace {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
namespace orb = foundations::orb;
namespace vocabulary = foundations::vocabulary;

/// @brief A `.npy` array: its type, shape and raw little-endian values.
struct Npy {
  /// @brief numpy's `descr`, e.g. `|u1`, `<i4`, `<i8`, `<f8`.
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
/// @return The shape, or nothing if it is not a tuple of whole numbers, each one a `size_t`
///         holds.
std::optional<std::vector<std::size_t>> parse_shape(const std::string& text) {
  if (text.size() < 2 || text.front() != '(' || text.back() != ')') {
    return std::nullopt;
  }
  std::vector<std::size_t> out;
  std::string part;
  std::istringstream parts(text.substr(1, text.size() - 2));
  while (std::getline(parts, part, ',')) {
    std::erase(part, ' ');
    if (part.empty()) {
      continue;
    }
    std::size_t value = 0;
    const char* end = std::to_address(part.end());
    const auto [stop, error] = std::from_chars(part.data(), end, value);
    if (error != std::errc{} || stop != end) {  // a sign, another character, or too large
      return std::nullopt;
    }
    out.push_back(value);
  }
  return out;
}

/// @brief The bytes one value of a `.npy` type takes, for the types a vocabulary's folder holds.
/// @param descr numpy's `descr`.
/// @return 1, 4 or 8; or 0 for another type.
std::size_t item_size(const std::string& descr) {
  if (descr == "|u1") {
    return 1;
  }
  if (descr == "<i4") {
    return 4;
  }
  return descr == "<i8" || descr == "<f8" ? 8 : 0;
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
  std::size_t count = item;
  for (const std::size_t extent : out.shape) {
    if (extent != 0 && count > std::numeric_limits<std::size_t>::max() / extent) {
      return fail(ErrorKind::kInvalidInput, path.string() + ": its data do not fill its shape");
    }
    count *= extent;
  }
  if (out.data.size() != count) {
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
  if (!out.empty()) {
    std::memcpy(out.data(), array.data.data(), array.data.size());
  }
  return out;
}

/// @brief Read one `.npy` file of a folder, of a type and shape.
/// @tparam T The element type.
/// @param root The folder.
/// @param name The array's name: the file is `<name>.npy`.
/// @param descr The type it must have.
/// @param shape The shape it must have; a 0 accepts any length on that axis.
/// @return Its values, and its shape; or `InvalidInput`.
template <typename T>
Result<std::pair<std::vector<T>, std::vector<std::size_t>>> read_array(
    const std::filesystem::path& root, const std::string& name, const char* descr,
    const std::vector<std::size_t>& shape) {
  auto array = read_npy(root / (name + ".npy"));
  if (!array) {
    return std::unexpected(std::move(array.error()));
  }
  auto values = values_of<T>(*array, descr, shape, name.c_str());
  if (!values) {
    return std::unexpected(std::move(values.error()));
  }
  return std::pair{std::move(*values), std::move(array->shape)};
}

/// @brief Read a folder's header and checksums: this format's version, this ORB's descriptor.
/// @param root The folder.
/// @return The header; or `InvalidInput`.
Result<Header> read_header(const std::filesystem::path& root) {
  auto numbers = read_array<std::int64_t>(root, "header", "<i8", {5});
  if (!numbers) {
    return std::unexpected(std::move(numbers.error()));
  }
  const std::vector<std::int64_t>& n = numbers->first;
  Header out{.version = n[0], .descriptor_version = n[1], .k = n[2], .levels = n[3], .words = n[4]};
  if (out.version != kVersion) {
    return fail(ErrorKind::kInvalidInput, root.string() + ": a vocabulary of format version " +
                                              std::to_string(out.version) + ", expected " +
                                              std::to_string(kVersion));
  }
  if (out.descriptor_version != orb::kDescriptorVersion) {
    return fail(ErrorKind::kInvalidInput,
                root.string() + ": trained on descriptor version " +
                    std::to_string(out.descriptor_version) + ", this ORB's is " +
                    std::to_string(orb::kDescriptorVersion) + ": its words mean nothing here");
  }
  auto sums = read_array<std::uint8_t>(root, "sha256", "|u1", {kArrays.size(), kSha256Bytes});
  if (!sums) {
    return std::unexpected(std::move(sums.error()));
  }
  const std::span<const std::uint8_t> raw(sums->first);
  for (std::size_t i = 0; i < kArrays.size(); ++i) {
    std::ranges::copy(raw.subspan(i * kSha256Bytes, kSha256Bytes), out.sha256.at(i).begin());
  }
  return out;
}

}  // namespace

Result<Folder> read(const std::string& folder) {
  const std::filesystem::path root(folder);
  auto header = read_header(root);
  if (!header) {
    return std::unexpected(std::move(header.error()));
  }
  auto parents = read_array<std::int32_t>(root, "parents", "<i4", {0});
  if (!parents) {
    return std::unexpected(std::move(parents.error()));
  }
  const std::size_t n = parents->first.size();
  auto descriptors =
      read_array<std::uint8_t>(root, "descriptors", "|u1", {n, orb::kDescriptorBytes});
  auto children = read_array<std::int32_t>(root, "children", "<i4", {n, 2});
  auto words = read_array<std::int32_t>(root, "words", "<i4", {n});
  auto weights = read_array<double>(root, "weights", "<f8", {0});
  if (!descriptors) {
    return std::unexpected(std::move(descriptors.error()));
  }
  if (!children) {
    return std::unexpected(std::move(children.error()));
  }
  if (!words) {
    return std::unexpected(std::move(words.error()));
  }
  if (!weights) {
    return std::unexpected(std::move(weights.error()));
  }
  Folder out{.vocabulary = {}, .header = *header};
  vocabulary::Vocabulary& v = out.vocabulary;
  v.k = header->k;
  v.levels = header->levels;
  v.descriptors.resize(n);
  const std::span<const std::uint8_t> raw(descriptors->first);
  for (std::size_t i = 0; i < n; ++i) {
    std::ranges::copy(raw.subspan(i * orb::kDescriptorBytes, orb::kDescriptorBytes),
                      v.descriptors[i].begin());
  }
  v.parents = std::move(parents->first);
  v.children.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    v.children[i] = {children->first[2 * i], children->first[(2 * i) + 1]};
  }
  v.words = std::move(words->first);
  v.weights = std::move(weights->first);
  if (auto ok = vocabulary::check(v); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (std::cmp_not_equal(v.weights.size(), header->words)) {
    return fail(ErrorKind::kInvalidInput, root.string() + ": the header counts " +
                                              std::to_string(header->words) + " words, the tree " +
                                              std::to_string(v.weights.size()));
  }
  return out;
}

}  // namespace slam_it::eval::vocabularies
