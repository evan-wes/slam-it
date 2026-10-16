/// @file
/// @brief Native tests for the vocabulary reader (slam_it/eval/vocabularies.hpp), against the
///        shared fixture `tests/fixtures/eval/vocabulary/` and its cases
///        `vocabulary_cases.json`, which the Python and Rust tests run too: the tree and its
///        header as read, and every case the C++ reader reads or refuses.
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/eval/vocabularies.hpp"

namespace {

namespace vocabularies = slam_it::eval::vocabularies;
namespace fs = std::filesystem;
using slam_it::testing::expect;
using slam_it::testing::is_invalid_input;

/// @brief A hex string's bytes.
/// @param hex The digits, two a byte.
/// @return The bytes.
std::string from_hex(const std::string& hex) {
  std::string out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out += static_cast<char>(std::stoul(hex.substr(i, 2), nullptr, 16));
  }
  return out;
}

/// @brief A digest in hex.
/// @param digest The bytes.
/// @return Two digits a byte.
std::string to_hex(const std::array<std::uint8_t, vocabularies::kSha256Bytes>& digest) {
  constexpr std::string_view kDigits = "0123456789abcdef";
  std::string out;
  for (const std::uint8_t byte : digest) {
    out += kDigits[byte >> 4U];
    out += kDigits[byte & 0xFU];
  }
  return out;
}

/// @brief Check that the fixture folder reads as its tree, with its record's header.
/// @param fixture The fixture.
/// @param failures Running failure count.
void check_folder(const nlohmann::json& fixture, int& failures) {
  const auto read = vocabularies::read(slam_it::testing::fixture_path("eval/vocabulary"));
  expect(read.has_value(), "the fixture folder reads", failures);
  if (!read) {
    return;
  }
  const auto& tree = fixture["tree"];
  const auto& v = read->vocabulary;
  std::vector<std::uint8_t> descriptors;
  for (const auto& d : v.descriptors) {
    descriptors.insert(descriptors.end(), d.begin(), d.end());
  }
  std::vector<std::int32_t> children;
  for (const auto& c : v.children) {
    children.insert(children.end(), c.begin(), c.end());
  }
  expect(v.k == tree["k"] && v.levels == tree["levels"], "k and the levels are the header's",
         failures);
  expect(descriptors == slam_it::testing::flatten_rows<std::uint8_t>(tree["descriptors"]),
         "the descriptors are the tree's", failures);
  expect(v.parents == tree["parents"].get<std::vector<std::int32_t>>(),
         "the parents are the tree's", failures);
  expect(children == slam_it::testing::flatten_rows<std::int32_t>(tree["children"]),
         "the children are the tree's", failures);
  expect(v.words == tree["words"].get<std::vector<std::int32_t>>(), "the words are the tree's",
         failures);
  expect(v.weights == tree["weights"].get<std::vector<double>>(), "the weights are the tree's",
         failures);
  const auto& record = fixture["record"];
  const auto& h = read->header;
  expect(h.version == record["version"] && h.descriptor_version == record["descriptor_version"] &&
             h.k == record["k"] && h.levels == record["levels"] && h.words == record["words"],
         "the header is the record's", failures);
  for (std::size_t i = 0; i < vocabularies::kArrays.size(); ++i) {
    expect(
        to_hex(h.sha256.at(i)) == record["sha256"][vocabularies::kArrays.at(i)].get<std::string>(),
        (std::string("the checksum is the record's: ") + vocabularies::kArrays.at(i)).c_str(),
        failures);
  }
}

/// @brief Check every case: read, or refused with its message.
/// @param fixture The fixture.
/// @param failures Running failure count.
void check_cases(const nlohmann::json& fixture, int& failures) {
  const slam_it::testing::ScratchDir scratch("slam_it_vocabularies");
  for (const auto& entry : fixture["cases"]) {
    const std::string name = entry["name"];
    fs::remove_all(scratch.path());
    fs::copy(slam_it::testing::fixture_path("eval/vocabulary"), scratch.path());
    for (const auto& [file, hex] : entry["files"].items()) {
      std::ofstream out(scratch.path() / file, std::ios::binary | std::ios::trunc);
      const std::string bytes = from_hex(hex.get<std::string>());
      out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    const auto read = vocabularies::read(scratch.path().string());
    bool refused = false;
    for (const auto& language : entry["refused_by"]) {
      refused = refused || language == "cpp";
    }
    if (refused) {
      expect(is_invalid_input(read) &&
                 read.error().message.contains(entry["match"].get<std::string>()),
             ("refused, with its message: " + name).c_str(), failures);
    } else {
      expect(read.has_value(), ("read: " + name).c_str(), failures);
    }
  }
}

/// @brief Check that a missing folder is refused, as `InvalidInput`.
/// @param failures Running failure count.
void check_missing(int& failures) {
  expect(is_invalid_input(vocabularies::read(slam_it::testing::fixture_path("eval/missing"))),
         "a missing folder: InvalidInput", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slam_it::testing::load_fixture("eval/vocabulary_cases.json");
  check_folder(fixture, failures);
  check_cases(fixture, failures);
  check_missing(failures);
  return slam_it::testing::finish("eval/vocabularies", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("eval/vocabularies", run); }
