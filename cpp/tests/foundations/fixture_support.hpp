/// @file
/// @brief Reading the shared parity fixtures (tests/fixtures/) in the native tests.
///
/// The fixtures are JSON, read with nlohmann_json (a test-only dependency). Parsing throws on
/// a missing or malformed file, so a test that reads one runs its body through `guarded`,
/// which reports the exception as a failure instead of letting it escape `main`.
#pragma once

#include <unistd.h>

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace slamlab::testing {

/// @brief A path under `tests/fixtures/`.
/// @param relative The path under it, e.g. `"eval/sequences/euroc"`.
/// @return The full path.
inline std::string fixture_path(const std::string& relative) {
  return std::string(SLAMLAB_FIXTURES_DIR) + "/" + relative;
}

/// @brief A scratch folder under the temp directory, named for this process so concurrent
///        test runs never share one, and removed when it goes out of scope (a failed check
///        leaves nothing behind).
class ScratchDir {
 public:
  /// @brief Name the folder; anything left there by an earlier run is removed.
  /// @param name Its name's prefix.
  explicit ScratchDir(const std::string& name)
      : path_(std::filesystem::temp_directory_path() / (name + "_" + std::to_string(::getpid()))) {
    std::filesystem::remove_all(path_);
  }
  ScratchDir(const ScratchDir&) = delete;
  ScratchDir& operator=(const ScratchDir&) = delete;
  ScratchDir(ScratchDir&&) = delete;
  ScratchDir& operator=(ScratchDir&&) = delete;
  /// @brief Remove the folder, quietly.
  ~ScratchDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  /// @brief The folder.
  /// @return Its path (not created).
  [[nodiscard]] const std::filesystem::path& path() const { return path_; }

 private:
  /// @brief The folder.
  std::filesystem::path path_;
};

/// @brief Load a parity fixture.
/// @param relative Path under `tests/fixtures/`, e.g. `"foundations/rng.json"`.
/// @return The parsed fixture.
/// @throws nlohmann::json::exception if the file is missing or malformed.
inline nlohmann::json load_fixture(const std::string& relative) {
  std::ifstream file(fixture_path(relative));
  return nlohmann::json::parse(file);
}

/// @brief Read a fixture's list of rows as one row-major vector.
/// @tparam T The element type.
/// @param rows The list of rows.
/// @return The rows' values, concatenated.
template <typename T>
std::vector<T> flatten_rows(const nlohmann::json& rows) {
  std::vector<T> out;
  for (const auto& row : rows) {
    for (const auto& value : row) {
      out.push_back(value.get<T>());
    }
  }
  return out;
}

/// @brief Run a test body, reporting an escaping exception as a failure.
/// @tparam Body Callable as `int()`, returning the test's exit code.
/// @param name The test's name, for the message.
/// @param body The test body.
/// @return The body's exit code, or 1 if it threw.
template <typename Body>
int guarded(const char* name, Body&& body) noexcept {
  try {
    return body();
  } catch (const std::exception& e) {
    std::cerr << "FAILED: " << name << " threw: " << e.what() << "\n";
    return 1;
  }
}

}  // namespace slamlab::testing
