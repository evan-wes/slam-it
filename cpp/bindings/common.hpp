/// @file
/// @brief Shared helpers for every binding: array types, owned outputs, error translation.
///
/// Bindings take float64 C-contiguous numpy arrays with a leading batch dimension and return
/// new arrays (design doc, "Conventions"). Arguments are `noconvert`: anything else (another
/// dtype, a strided view, a misaligned buffer) is refused rather than silently copied, the
/// same as the Rust binding. A core `Error` crosses as the Python exception of its kind, from
/// `slamlab.foundations.errors`.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>

#include "slamlab/foundations/error.hpp"

namespace slamlab::bindings {

/// @brief A read-only, contiguous float64 batch of items of shape `Dims...`.
/// @tparam Dims The shape of one item; the leading dimension N is free.
template <std::size_t... Dims>
using BatchIn = nanobind::ndarray<const double, nanobind::shape<-1, Dims...>, nanobind::c_contig,
                                  nanobind::device::cpu>;

/// @brief A new float64 numpy array, owning its buffer.
using ArrayOut = nanobind::ndarray<nanobind::numpy, double>;

/// @brief A core error carried through C++ to the binding's exception translator.
class CoreError : public std::runtime_error {
 public:
  /// @brief Wrap a core error.
  /// @param error The error returned by a core function.
  explicit CoreError(foundations::Error error)
      : std::runtime_error(error.message), kind_(error.kind) {}

  /// @brief What went wrong.
  /// @return The error's kind.
  [[nodiscard]] foundations::ErrorKind kind() const noexcept { return kind_; }

 private:
  /// @brief What went wrong.
  foundations::ErrorKind kind_;
};

/// @brief Return a result's value, or throw its error as a `CoreError`.
/// @tparam T The value type.
/// @param result A core function's result.
/// @return The value on success.
/// @throws CoreError (translated to the matching Python exception) on failure.
template <typename T>
T unwrap(foundations::Result<T> result) {
  if (!result) {
    throw CoreError(std::move(result.error()));
  }
  return std::move(*result);
}

/// @brief Register the translator that raises a `CoreError` as the Python exception of its
///        kind, from `slamlab.foundations.errors`.
void register_error_translation();

/// @brief Refuse a buffer that is not aligned for `double` (reading it would be undefined
///        behaviour; nanobind does not check alignment).
/// @param data The buffer's first element.
/// @throws std::invalid_argument (Python `ValueError`) if it is misaligned.
inline void check_aligned(const double* data) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): inspecting an address only
  if (reinterpret_cast<std::uintptr_t>(data) % alignof(double) != 0) {
    throw std::invalid_argument("input array is misaligned for float64");
  }
}

/// @brief Allocate a new float64 numpy array of the given shape, owning its buffer.
/// @param shape The array's shape.
/// @return The array and a pointer to its (row-major) data.
inline std::pair<ArrayOut, double*> new_array(std::initializer_list<std::size_t> shape) {
  std::size_t size = 1;
  for (const std::size_t d : shape) {
    size *= d;
  }
  // The capsule owns the buffer and frees it when numpy releases the array.
  auto* data = new double[size == 0 ? 1 : size]();  // NOLINT(cppcoreguidelines-owning-memory)
  const nanobind::capsule owner(data, [](void* p) noexcept {
    delete[] static_cast<double*>(p);  // NOLINT(cppcoreguidelines-owning-memory)
  });
  return {ArrayOut(data, shape.size(), shape.begin(), owner), data};
}

/// @brief A fixed-size item as stored in a row-major numpy batch (vectors are columns).
/// @tparam Rows The item's rows.
/// @tparam Cols The item's columns (1 for a vector).
template <int Rows, int Cols>
using Item = Eigen::Matrix<double, Rows, Cols, (Cols == 1 ? Eigen::ColMajor : Eigen::RowMajor)>;

/// @brief Pass a plain value through; the counterpart of `value_of(Result<T>)`.
/// @tparam T The value type.
/// @param value A core function's (infallible) result.
/// @return `value`.
template <typename T>
T value_of(T value) {
  return value;
}

/// @brief Return a result's value, or throw its error (see `unwrap`).
/// @tparam T The value type.
/// @param result A core function's fallible result.
/// @return The value on success.
/// @throws CoreError on failure.
template <typename T>
T value_of(foundations::Result<T> result) {
  return unwrap(std::move(result));
}

/// @brief Apply a per-item core function to every item of a batch.
/// @tparam InRows Rows of one input item.
/// @tparam InCols Columns of one input item (1 for a vector).
/// @tparam OutRows Rows of one output item.
/// @tparam OutCols Columns of one output item (1 for a vector).
/// @tparam Input The input array type.
/// @tparam F The core function: item -> item, or item -> `Result<item>`.
/// @param input A C-contiguous float64 array of shape (N, InRows[, InCols]).
/// @param f The function to apply.
/// @return A new array of shape (N, OutRows[, OutCols]).
/// @throws CoreError if `f` fails on any item; std::invalid_argument if misaligned.
template <int InRows, int InCols, int OutRows, int OutCols, typename Input, typename F>
ArrayOut map_batch(const Input& input, F&& f) {
  check_aligned(input.data());
  const std::size_t n = input.shape(0);
  constexpr std::size_t kIn = static_cast<std::size_t>(InRows) * InCols;
  constexpr std::size_t kOut = static_cast<std::size_t>(OutRows) * OutCols;
  auto [out, data] =
      OutCols == 1
          ? new_array({n, static_cast<std::size_t>(OutRows)})
          : new_array({n, static_cast<std::size_t>(OutRows), static_cast<std::size_t>(OutCols)});
  for (std::size_t i = 0; i < n; ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offsets
    const Eigen::Map<const Item<InRows, InCols>> item(input.data() + (i * kIn));
    Eigen::Map<Item<OutRows, OutCols>> result(data + (i * kOut));
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    result = value_of(f(Item<InRows, InCols>(item)));
  }
  return out;
}

}  // namespace slamlab::bindings
