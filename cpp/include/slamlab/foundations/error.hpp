/// @file
/// @brief The error contract shared by every rung's C++ core.
///
/// Fallible functions return `slamlab::foundations::Result<T>`, a C++23 `std::expected`. It
/// mirrors the Rust core's `slamlab_foundations::error::Result` and the Python exceptions in
/// `slamlab.foundations.errors`. The bindings translate an `Error` into the Python exception
/// of the same kind, so all three languages fail the same way. No exceptions are thrown on
/// the cores' hot paths.
#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <utility>

namespace slamlab::foundations {

/// @brief What went wrong, independent of where.
enum class ErrorKind : std::uint8_t {
  /// @brief The input violates the function's contract (shape, range, finiteness).
  kInvalidInput,
  /// @brief The input is valid but geometrically degenerate (pure rotation, collinear points,
  ///        too few correspondences), so the quantity asked for is not determined.
  kDegenerate,
  /// @brief An iterative method failed to converge where that is an error: strict mode was
  ///        requested and the convergence criteria were not met, or the iteration diverged
  ///        with no usable estimate. Reaching the iteration cap is otherwise not an error:
  ///        iterative solvers return their best estimate with a solve summary.
  kNotConverged,
};

/// @brief An error from a slamlab core function: its kind and a message naming the cause.
struct Error {
  /// @brief What went wrong.
  ErrorKind kind;
  /// @brief A human-readable description naming the offending value or condition.
  std::string message;
};

/// @brief The result type of every fallible slamlab core function.
/// @tparam T The value type on success.
template <typename T>
using Result = std::expected<T, Error>;

/// @brief Build the failed result of a fallible function.
/// @param kind What went wrong.
/// @param message A description naming the offending value or condition.
/// @return An `std::unexpected` holding the error, convertible to any `Result<T>`.
[[nodiscard]] inline std::unexpected<Error> fail(ErrorKind kind, std::string message) {
  return std::unexpected<Error>(Error{.kind = kind, .message = std::move(message)});
}

}  // namespace slamlab::foundations
