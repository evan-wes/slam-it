/// @file
/// @brief Shared helpers for every binding: array types, owned outputs, error translation.
///
/// Bindings take float64 C-contiguous numpy arrays with a leading batch dimension and return
/// new arrays (design doc, "Conventions"); images cross as uint8 (`image_crossing.hpp`).
/// Arguments are `noconvert`: anything else (another dtype, a strided view, a misaligned
/// buffer) is refused rather than silently copied, the same as the Rust binding. A core
/// `Error` crosses as the Python exception of its kind, from `slam_it.foundations.errors`.
#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "slam_it/foundations/error.hpp"

namespace slam_it::bindings {

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
  if constexpr (!std::is_void_v<T>) {
    return std::move(*result);
  }
}

/// @brief Register the translator that raises a `CoreError` as the Python exception of its
///        kind, from `slam_it.foundations.errors`.
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

/// @brief Allocate a new numpy array of the given shape, owning its buffer.
/// @tparam T The element type (float64 unless stated).
/// @param shape The array's shape.
/// @return The array and a pointer to its (row-major) data.
template <typename T = double>
std::pair<nanobind::ndarray<nanobind::numpy, T>, T*> new_array(
    std::initializer_list<std::size_t> shape) {
  std::size_t size = 1;
  for (const std::size_t d : shape) {
    size *= d;
  }
  // The capsule owns the buffer and frees it when numpy releases the array.
  auto* data = new T[size == 0 ? 1 : size]();  // NOLINT(cppcoreguidelines-owning-memory)
  const nanobind::capsule owner(data, [](void* p) noexcept {
    delete[] static_cast<T*>(p);  // NOLINT(cppcoreguidelines-owning-memory)
  });
  return {nanobind::ndarray<nanobind::numpy, T>(data, shape.size(), shape.begin(), owner), data};
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

/// @brief Apply a per-item core function to every pair of items of two equal-length batches.
/// @tparam ARows Rows of one item of the first input.
/// @tparam ACols Columns of one item of the first input (1 for a vector).
/// @tparam BRows Rows of one item of the second input.
/// @tparam BCols Columns of one item of the second input (1 for a vector).
/// @tparam OutRows Rows of one output item.
/// @tparam OutCols Columns of one output item (1 for a vector).
/// @tparam InputA The first input array type.
/// @tparam InputB The second input array type.
/// @tparam F The core function: (item, item) -> item, or -> `Result<item>`.
/// @param a A C-contiguous float64 array of shape (N, ARows[, ACols]).
/// @param b A C-contiguous float64 array of shape (N, BRows[, BCols]).
/// @param f The function to apply.
/// @return A new array of shape (N, OutRows[, OutCols]).
/// @throws CoreError if `f` fails on any pair; std::invalid_argument if the batches differ
///         in length or either is misaligned.
template <int ARows, int ACols, int BRows, int BCols, int OutRows, int OutCols, typename InputA,
          typename InputB, typename F>
ArrayOut map_batch2(const InputA& a, const InputB& b, F&& f) {
  check_aligned(a.data());
  check_aligned(b.data());
  const std::size_t n = a.shape(0);
  if (b.shape(0) != n) {
    throw std::invalid_argument("the two batches differ in length");
  }
  constexpr std::size_t kA = static_cast<std::size_t>(ARows) * ACols;
  constexpr std::size_t kB = static_cast<std::size_t>(BRows) * BCols;
  constexpr std::size_t kOut = static_cast<std::size_t>(OutRows) * OutCols;
  auto [out, data] =
      OutCols == 1
          ? new_array({n, static_cast<std::size_t>(OutRows)})
          : new_array({n, static_cast<std::size_t>(OutRows), static_cast<std::size_t>(OutCols)});
  for (std::size_t i = 0; i < n; ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offsets
    const Eigen::Map<const Item<ARows, ACols>> item_a(a.data() + (i * kA));
    const Eigen::Map<const Item<BRows, BCols>> item_b(b.data() + (i * kB));
    Eigen::Map<Item<OutRows, OutCols>> result(data + (i * kOut));
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    result = value_of(f(Item<ARows, ACols>(item_a), Item<BRows, BCols>(item_b)));
  }
  return out;
}

/// @brief A read-only, contiguous 1D array.
/// @tparam T The element type.
template <typename T>
using VectorIn =
    nanobind::ndarray<const T, nanobind::shape<-1>, nanobind::c_contig, nanobind::device::cpu>;

/// @brief A read-only, contiguous float64 matrix.
using MatrixIn = nanobind::ndarray<const double, nanobind::shape<-1, -1>, nanobind::c_contig,
                                   nanobind::device::cpu>;

/// @brief Copy a 1D array into a vector.
/// @tparam Out The vector type.
/// @tparam T The element type.
/// @param in The array.
/// @return The copy.
template <typename Out, typename T>
Out copy_vector(const VectorIn<T>& in) {
  Out out(in.shape(0));
  for (std::size_t i = 0; i < in.shape(0); ++i) {
    out[i] = in(i);
  }
  return out;
}

/// @brief Copy a row-major float64 matrix of known column count into an Eigen matrix.
/// @tparam M The Eigen matrix type.
/// @param data The buffer.
/// @param rows Rows.
/// @param cols Columns.
/// @return The copy.
template <typename M>
M copy_matrix(const double* data, std::size_t rows, std::size_t cols) {
  check_aligned(data);
  return Eigen::Map<const M>(data, static_cast<Eigen::Index>(rows),
                             static_cast<Eigen::Index>(cols));
}

/// @brief Copy a (C, 4, 4) batch into poses.
/// @param in The batch.
/// @return The poses.
inline std::vector<Eigen::Matrix4d> copy_poses(const BatchIn<4, 4>& in) {
  check_aligned(in.data());
  std::vector<Eigen::Matrix4d> out;
  for (std::size_t c = 0; c < in.shape(0); ++c) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    out.emplace_back(Eigen::Map<const Item<4, 4>>(in.data() + (16 * c)));
  }
  return out;
}

/// @brief Copy poses into a new (C, 4, 4) array.
/// @param poses The poses.
/// @return The array.
inline ArrayOut poses_array(const std::vector<Eigen::Matrix4d>& poses) {
  auto [out, data] = new_array({poses.size(), 4, 4});
  for (std::size_t c = 0; c < poses.size(); ++c) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): batch offset
    Eigen::Map<Item<4, 4>>(data + (16 * c)) = poses[c];
  }
  return out;
}

/// @brief Copy a row-major matrix into a new array of its shape.
/// @tparam M The matrix type.
/// @param matrix The matrix.
/// @return The array.
template <typename M>
ArrayOut matrix_array(const M& matrix) {
  auto [out, data] =
      new_array({static_cast<std::size_t>(matrix.rows()), static_cast<std::size_t>(matrix.cols())});
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      data, matrix.rows(), matrix.cols()) = matrix;
  return out;
}

/// @brief Copy a vector into a new 1D array.
/// @param vector The vector.
/// @return The array.
inline ArrayOut vector_array(const Eigen::VectorXd& vector) {
  auto [out, data] = new_array({static_cast<std::size_t>(vector.size())});
  Eigen::Map<Eigen::VectorXd>(data, vector.size()) = vector;
  return out;
}

/// @brief Copy an (N, 3) batch in, as the cores' row-major rows.
/// @param in The array.
/// @return The rows.
inline Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> rows3(const BatchIn<3>& in) {
  return copy_matrix<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(in.data(),
                                                                                in.shape(0), 3);
}

/// @brief Copy an (N, 2) batch in, as the cores' row-major rows.
/// @param in The array.
/// @return The rows.
inline Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> rows2(const BatchIn<2>& in) {
  return copy_matrix<Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>>(in.data(),
                                                                                in.shape(0), 2);
}

/// @brief Decode an enum crossing as its integer code, for an enum with a `kCount` sentinel.
/// @tparam E The enum.
/// @param code The code.
/// @param what The argument's name, for the message.
/// @return The enum value.
/// @throws std::invalid_argument (Python `ValueError`) for an unknown code.
template <typename E>
E enum_code(int code, const char* what) {
  if (code < 0 || code >= static_cast<int>(E::kCount)) {
    throw std::invalid_argument(std::string(what) + ": unknown code " + std::to_string(code));
  }
  return static_cast<E>(code);
}

/// @brief An enum's names in code order, for a `codes()` table.
/// @tparam E The enum, with a `kCount` sentinel.
/// @tparam Name Callable as `name(E)`, returning the name.
/// @param name The enum's name function.
/// @return The names.
template <typename E, typename Name>
nanobind::list enum_names(const Name& name) {
  nanobind::list out;
  for (int k = 0; k < static_cast<int>(E::kCount); ++k) {
    out.append(name(static_cast<E>(k)));
  }
  return out;
}

/// @brief Look an enum up by its name, as its core's `name` function spells it.
/// @tparam E The enum, with a `kCount` sentinel.
/// @tparam Name Callable as `name(E)`, returning the name.
/// @param value The name to find.
/// @param name The enum's name function.
/// @param what The field's name, for the message.
/// @return The enum value.
/// @throws std::invalid_argument (Python `ValueError`) for an unknown name.
template <typename E, typename Name>
E enum_by_name(const std::string& value, const Name& name, const char* what) {
  for (int k = 0; k < static_cast<int>(E::kCount); ++k) {
    if (value == name(static_cast<E>(k))) {
      return static_cast<E>(k);
    }
  }
  throw std::invalid_argument(std::string(what) + ": unknown name " + value);
}

/// @brief One read-only (3, 3) matrix.
using Matrix3In = nanobind::ndarray<const double, nanobind::shape<3, 3>, nanobind::c_contig,
                                    nanobind::device::cpu>;

/// @brief Copy one (3, 3) matrix in.
/// @param in The array.
/// @return The matrix.
inline Eigen::Matrix3d matrix3(const Matrix3In& in) {
  check_aligned(in.data());
  return Eigen::Map<const Item<3, 3>>(in.data());
}

/// @brief One read-only (4, 4) matrix.
using Matrix4In = nanobind::ndarray<const double, nanobind::shape<4, 4>, nanobind::c_contig,
                                    nanobind::device::cpu>;

/// @brief Copy one (4, 4) matrix in.
/// @param in The array.
/// @return The matrix.
inline Eigen::Matrix4d matrix4(const Matrix4In& in) {
  check_aligned(in.data());
  return Eigen::Map<const Item<4, 4>>(in.data());
}

/// @brief Return a 3x3 matrix as a new (3, 3) array.
/// @param matrix The matrix.
/// @return The array.
inline ArrayOut matrix3_array(const Eigen::Matrix3d& matrix) {
  auto [out, data] = new_array({3, 3});
  Eigen::Map<Item<3, 3>> view(data);
  view = matrix;
  return out;
}

/// @brief Copy a boolean array out.
/// @param mask The mask.
/// @return A new (N,) bool array.
inline nanobind::ndarray<nanobind::numpy, bool> mask_array(
    const Eigen::Array<bool, Eigen::Dynamic, 1>& mask) {
  auto [out, data] = new_array<bool>({static_cast<std::size_t>(mask.size())});
  for (Eigen::Index i = 0; i < mask.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    data[i] = mask(i);
  }
  return out;
}

/// @brief Refuse a dict whose keys are not exactly the expected ones (an options dict crossing
///        from a Python dataclass), so the two sides cannot drift apart. The messages are
///        serde's, which the Rust binding reads its options with, so both fail alike.
/// @param values The dict.
/// @param keys The expected keys.
/// @param what The dataclass's name, for the message.
/// @throws std::invalid_argument (Python `ValueError`) for a missing or unknown key.
inline void check_keys(const nanobind::dict& values, std::span<const char* const> keys,
                       const char* what) {
  for (const char* key : keys) {
    if (!values.contains(key)) {
      throw std::invalid_argument(std::string("options: missing field `") + key + "` of " + what);
    }
  }
  for (const auto& [key, value] : values) {
    const std::string name = nanobind::str(key).c_str();
    if (std::ranges::find(keys, name) == keys.end()) {
      throw std::invalid_argument("options: unknown field `" + name + "`, not a field of " + what);
    }
  }
}

}  // namespace slam_it::bindings
