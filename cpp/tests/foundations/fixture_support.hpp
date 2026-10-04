/// @file
/// @brief Reading the shared parity fixtures (tests/fixtures/) in the native tests.
///
/// The fixtures are JSON, read with nlohmann_json (a test-only dependency). Parsing throws on
/// a missing or malformed file, so a test that reads one runs its body through `guarded`,
/// which reports the exception as a failure instead of letting it escape `main`.
#pragma once

#include <exception>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

namespace slamlab::testing {

/// @brief Load a parity fixture.
/// @param relative Path under `tests/fixtures/`, e.g. `"foundations/rng.json"`.
/// @return The parsed fixture.
/// @throws nlohmann::json::exception if the file is missing or malformed.
inline nlohmann::json load_fixture(const std::string& relative) {
  std::ifstream file(std::string(SLAMLAB_FIXTURES_DIR) + "/" + relative);
  return nlohmann::json::parse(file);
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
