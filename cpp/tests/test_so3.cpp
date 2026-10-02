/// @file
/// @brief Native test, no framework yet: hat(v) * u must equal cross(v, u).
#include <cmath>
#include <cstddef>
#include <iostream>

#include "slamlab/so3.hpp"

int main() {
  const slamlab::Vec3 v{1.0, -2.0, 3.0};
  const slamlab::Vec3 u{0.5, 4.0, -1.5};
  const slamlab::Vec3 cross{v[1] * u[2] - v[2] * u[1], v[2] * u[0] - v[0] * u[2],
                            v[0] * u[1] - v[1] * u[0]};
  const auto h = slamlab::hat(v);
  for (std::size_t i = 0; i < 3; ++i) {
    const double hu = h[i][0] * u[0] + h[i][1] * u[1] + h[i][2] * u[2];
    if (std::abs(hu - cross[i]) > 1e-12) {
      std::cerr << "row " << i << ": hat(v)u = " << hu << ", cross = " << cross[i] << "\n";
      return 1;
    }
  }
  std::cout << "so3 ok\n";
  return 0;
}
