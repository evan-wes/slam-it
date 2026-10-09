/// @file
/// @brief Implementation of slamlab/eval/budgets.hpp.
#include "slamlab/eval/budgets.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "slamlab/eval/text.hpp"

namespace slamlab::eval::budgets {

namespace {

/// @brief The words after a key on a status file's line, as `slamlab.eval.budgets` splits it.
/// @param status The status file.
/// @param key The line's first word, e.g. `VmHWM:`.
/// @return The line's other words, or nothing if no line starts with the key (or no file).
std::optional<std::vector<std::string>> status_words(const std::string& status,
                                                     std::string_view key) {
  std::ifstream file(status);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream words(line);
    std::string first;
    if (!(words >> first) || first != key) {
      continue;
    }
    std::vector<std::string> rest;
    for (std::string word; words >> word;) {
      rest.push_back(word);
    }
    return rest;
  }
  return std::nullopt;
}

/// @brief Read a Linux CPU list, e.g. `0-7,16`.
/// @param text The list.
/// @return The CPUs, ascending, or nothing if it is not one.
std::optional<std::vector<std::int64_t>> cpu_list(std::string_view text) {
  std::set<std::int64_t> out;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = std::min(text.find(',', start), text.size());
    const std::string_view part = text.substr(start, comma - start);
    const std::size_t dash = part.find('-');
    const std::optional<std::int64_t> first = text::integer(part.substr(0, dash));
    const std::optional<std::int64_t> last =
        dash == std::string_view::npos ? first : text::integer(part.substr(dash + 1));
    if (!first || !last || *first < 0 || *last < *first) {
      return std::nullopt;
    }
    for (std::int64_t cpu = *first; cpu <= *last; ++cpu) {
      out.insert(cpu);
    }
    start = comma + 1;
  }
  return std::vector<std::int64_t>(out.begin(), out.end());
}

}  // namespace

std::optional<std::int64_t> peak_memory(const std::string& status) {
  constexpr std::int64_t kKilobyte = 1024;
  const auto words = status_words(status, "VmHWM:");
  if (!words || words->size() != 2 || (*words)[1] != "kB") {
    return std::nullopt;
  }
  const std::optional<std::int64_t> kilobytes = text::integer((*words)[0]);
  if (!kilobytes || *kilobytes < 0) {
    return std::nullopt;
  }
  return *kilobytes * kKilobyte;
}

std::optional<std::vector<std::int64_t>> allowed_cpus(const std::string& status) {
  const auto words = status_words(status, "Cpus_allowed_list:");
  if (!words || words->size() != 1) {
    return std::nullopt;
  }
  return cpu_list(words->front());
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): two paths, as Python's (cgroup, root)
std::optional<std::int64_t> memory_cap(const std::string& cgroup, const std::string& root) {
  std::ifstream groups(cgroup);
  std::string line;
  while (std::getline(groups, line)) {
    if (!line.starts_with("0::")) {
      continue;
    }
    std::ifstream file(root + line.substr(3) + "/memory.max");
    std::string value;
    if (!(file >> value)) {
      return std::nullopt;
    }
    const std::optional<std::int64_t> bytes = text::integer(value);
    return bytes && *bytes > 0 ? bytes : std::nullopt;
  }
  return std::nullopt;
}

}  // namespace slamlab::eval::budgets
