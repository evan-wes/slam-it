/// @file
/// @brief Native tests for slamlab::foundations::hat, runnable without Python.
///
/// The basis vectors pin every entry of the matrix; skew-symmetry and the cross-product
/// identity are checked on a general vector. hat only copies and negates, so every check is
/// exact (no tolerance).
#include <cstddef>
#include <iostream>

#include "slamlab/foundations/so3.hpp"

namespace {

using slamlab::foundations::hat;
using slamlab::foundations::Mat3;
using slamlab::foundations::Vec3;

/// @brief Count and report a failed check.
/// @param ok Whether the check passed.
/// @param what Description of the check, printed on failure.
/// @param failures Running failure count, incremented on failure.
void expect(bool ok, const char* what, int& failures) {
  if (!ok) {
    std::cerr << "FAILED: " << what << "\n";
    ++failures;
  }
}

}  // namespace

int main() {
  int failures = 0;

  // Basis vectors fix all nine entries of the matrix.
  const Mat3 hx = hat({1.0, 0.0, 0.0});
  const Mat3 hy = hat({0.0, 1.0, 0.0});
  const Mat3 hz = hat({0.0, 0.0, 1.0});
  expect(hx == Mat3{{{0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}}}, "hat(e_x)", failures);
  expect(hy == Mat3{{{0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}}}, "hat(e_y)", failures);
  expect(hz == Mat3{{{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}}, "hat(e_z)", failures);

  // Skew-symmetry and the cross product on a general vector (exact for these values).
  const Vec3 v{1.0, -2.0, 3.0};
  const Vec3 u{0.5, 4.0, -1.5};
  const Mat3 h = hat(v);
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      expect(h[i][j] == -h[j][i], "hat(v) is skew-symmetric", failures);
    }
  }
  const Vec3 cross{v[1] * u[2] - v[2] * u[1], v[2] * u[0] - v[0] * u[2], v[0] * u[1] - v[1] * u[0]};
  for (std::size_t i = 0; i < 3; ++i) {
    expect(h[i][0] * u[0] + h[i][1] * u[1] + h[i][2] * u[2] == cross[i], "hat(v) u == v x u",
           failures);
  }

  if (failures == 0) {
    std::cout << "foundations/so3: ok\n";
  }
  return failures == 0 ? 0 : 1;
}
