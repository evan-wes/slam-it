/// @file
/// @brief Implementation of the text helpers declared in slamlab/eval/text.hpp.
#include "slamlab/eval/text.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <memory>
#include <system_error>

namespace slamlab::eval::text {

namespace {

/// @brief Whether a character is ASCII whitespace.
/// @param c The character.
/// @return Whether it is.
bool space(char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; }

/// @brief Parse a whole token as a value (`std::from_chars`).
/// @tparam T The value's type.
/// @param token The token, trimmed.
/// @return The value, or nothing if the token is not exactly one.
template <typename T>
std::optional<T> parsed(std::string_view token) {
  T out{};
  const auto [end, error] =
      std::from_chars(std::to_address(token.begin()), std::to_address(token.end()), out);
  if (error != std::errc{} || end != std::to_address(token.end()) || token.empty()) {
    return std::nullopt;
  }
  return out;
}

}  // namespace

foundations::Result<std::vector<std::string>> read_uncommented(const std::string& path) {
  auto lines = read_lines(path);
  if (lines) {
    for (std::string& line : *lines) {
      line.resize(std::min(line.size(), line.find('#')));
    }
  }
  return lines;
}

foundations::Result<std::string> read_uncommented_text(const std::string& path) {
  return read_uncommented(path).transform([](const std::vector<std::string>& lines) {
    std::string out;
    for (const std::string& line : lines) {
      out += line;
      out += '\n';
    }
    return out;
  });
}

foundations::Result<std::vector<std::string>> read_lines(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             path + ": no such file, or not readable");
  }
  std::vector<std::string> out;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    out.push_back(line);
  }
  if (file.bad()) {
    return foundations::fail(foundations::ErrorKind::kInvalidInput,
                             path + ": could not be read whole");
  }
  return out;
}

std::vector<std::string_view> fields(std::string_view line) {
  std::vector<std::string_view> out;
  std::size_t at = 0;
  while (at < line.size()) {
    while (at < line.size() && space(line[at])) {
      ++at;
    }
    const std::size_t start = at;
    while (at < line.size() && !space(line[at])) {
      ++at;
    }
    if (at > start) {
      out.push_back(line.substr(start, at - start));
    }
  }
  return out;
}

std::string_view trim(std::string_view text) {
  while (!text.empty() && space(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && space(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

std::optional<double> number(std::string_view token) {
  token = trim(token);
  if (token.starts_with('+')) {
    token.remove_prefix(1);
    if (token.starts_with('-')) {
      return std::nullopt;  // one sign, as Rust's and Python's readers take
    }
  }
  return parsed<double>(token);
}

std::optional<std::int64_t> integer(std::string_view token) {
  return parsed<std::int64_t>(trim(token));
}

bool is_data(std::string_view line) {
  const std::string_view stripped = trim(line);
  return !stripped.empty() && !stripped.starts_with('#');
}

std::string where(const std::string& path, std::size_t number) {
  return path + ":" + std::to_string(number) + ": ";
}

}  // namespace slamlab::eval::text
