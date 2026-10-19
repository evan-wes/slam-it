/// @file
/// @brief Reading the datasets' text files: lines, whitespace fields, and numbers parsed
///        exactly (`std::from_chars`), shared by the readers.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "slam_it/foundations/error.hpp"

/// @brief Reading the datasets' text files.
namespace slam_it::eval::text {

/// @brief Read a text file's lines, a trailing `\r` dropped from each.
/// @param path The file.
/// @return The lines; `InvalidInput` naming the file if it cannot be read.
[[nodiscard]] foundations::Result<std::vector<std::string>> read_lines(const std::string& path);

/// @brief Read a text file's lines, each cut at its first `#` (a comment to the line's end).
/// @param path The file.
/// @return The lines; `InvalidInput` naming the file if it cannot be read.
[[nodiscard]] foundations::Result<std::vector<std::string>> read_uncommented(
    const std::string& path);

/// @brief Read a text file whole, each line cut at its first `#`.
/// @param path The file.
/// @return The text, each line ending in `\n`; `InvalidInput` naming the file if it cannot be
///         read.
[[nodiscard]] foundations::Result<std::string> read_uncommented_text(const std::string& path);

/// @brief Split a line into its whitespace-separated fields.
/// @param line The line.
/// @return The fields, views into `line`.
[[nodiscard]] std::vector<std::string_view> fields(std::string_view line);

/// @brief Trim whitespace from both ends.
/// @param text The text.
/// @return A view of the rest.
[[nodiscard]] std::string_view trim(std::string_view text);

/// @brief Parse a whole token as a number (a leading `+` allowed, as Python's `float`).
/// @param token The token.
/// @return The number, or nothing if the token is not exactly one.
[[nodiscard]] std::optional<double> number(std::string_view token);

/// @brief Parse a whole token as an integer.
/// @param token The token.
/// @return The integer, or nothing if the token is not exactly one.
[[nodiscard]] std::optional<std::int64_t> integer(std::string_view token);

/// @brief Whether a TUM-style line carries data: not blank, not a `#` comment.
/// @param line The line.
/// @return Whether it does.
[[nodiscard]] bool is_data(std::string_view line);

/// @brief The message prefix naming a file and a line.
/// @param path The file.
/// @param number The 1-based line number.
/// @return `path:number: `.
[[nodiscard]] std::string where(const std::string& path, std::size_t number);

}  // namespace slam_it::eval::text
