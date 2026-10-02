/// @file
/// @brief Bindings for `slamlab.foundations.so3` (`slamlab._cpp.foundations.so3`).
#include "slamlab/foundations/so3.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>

#include "foundations/foundations.hpp"

namespace nb = nanobind;

namespace {

/// @brief A read-only, contiguous float64 3-vector from numpy.
using Vec3In = nb::ndarray<const double, nb::shape<3>, nb::c_contig, nb::device::cpu>;
/// @brief A new float64 3x3 numpy array.
using Mat3Out = nb::ndarray<nb::numpy, double, nb::shape<3, 3>>;

/// @brief Map a 3-vector to its skew-symmetric matrix, as numpy arrays.
/// @param v A contiguous float64 array of shape (3,). The argument is `noconvert`: nanobind
///          raises `TypeError` for anything else (another dtype, a strided view) rather than
///          silently copying, matching the Rust binding. The Python entry point validates
///          and converts first.
/// @return A new float64 array of shape (3, 3) that owns its memory.
Mat3Out hat(const Vec3In& v) {
  const slamlab::foundations::Mat3 h = slamlab::foundations::hat({v(0), v(1), v(2)});
  // The capsule owns the buffer and frees it when numpy releases the array.
  auto* data = new double[9];  // NOLINT(cppcoreguidelines-owning-memory)
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      data[(3 * i) + j] = h[i][j];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
  }
  const nb::capsule owner(data, [](void* p) noexcept {
    delete[] static_cast<double*>(p);  // NOLINT(cppcoreguidelines-owning-memory)
  });
  return Mat3Out(data, {3, 3}, owner);
}

}  // namespace

namespace slamlab::bindings {

void bind_so3(nb::module_& m) {
  m.def("hat", &hat, nb::arg("v").noconvert(),
        "Map a float64 3-vector to its skew-symmetric matrix, as a new (3, 3) array.");
}

}  // namespace slamlab::bindings
