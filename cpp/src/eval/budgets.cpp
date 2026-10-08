/// @file
/// @brief Implementation of slamlab/eval/budgets.hpp.
#include "slamlab/eval/budgets.hpp"

#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

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

}  // namespace slamlab::eval::budgets
