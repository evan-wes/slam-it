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

std::optional<std::int64_t> peak_memory(const std::string& status) {
  constexpr std::int64_t kKilobyte = 1024;
  std::ifstream file(status);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream words(line);
    std::string key;
    std::string value;
    std::string unit;
    std::string extra;
    if (!(words >> key >> value >> unit) || (words >> extra) || key != "VmHWM:" || unit != "kB") {
      continue;
    }
    const std::optional<std::int64_t> kilobytes = text::integer(value);
    if (kilobytes.has_value() && *kilobytes >= 0) {
      return *kilobytes * kKilobyte;
    }
  }
  return std::nullopt;
}

namespace {

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

std::optional<std::vector<std::int64_t>> allowed_cpus(const std::string& status) {
  std::ifstream file(status);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream words(line);
    std::string key;
    std::string value;
    std::string extra;
    if ((words >> key >> value) && !(words >> extra) && key == "Cpus_allowed_list:") {
      return cpu_list(value);
    }
  }
  return std::nullopt;
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
