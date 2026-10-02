// Native test, no framework yet: hat(v) * u must equal cross(v, u).
#include <cmath>
#include <cstdio>

#include "slamlab/so3.hpp"

int main() {
  const slamlab::Vec3 v{1.0, -2.0, 3.0};
  const slamlab::Vec3 u{0.5, 4.0, -1.5};
  const slamlab::Vec3 cross{v[1] * u[2] - v[2] * u[1], v[2] * u[0] - v[0] * u[2],
                            v[0] * u[1] - v[1] * u[0]};
  const auto H = slamlab::hat(v);
  for (int i = 0; i < 3; ++i) {
    const double hu = H[i][0] * u[0] + H[i][1] * u[1] + H[i][2] * u[2];
    if (std::abs(hu - cross[i]) > 1e-12) {
      std::fprintf(stderr, "row %d: hat(v)u = %g, cross = %g\n", i, hu, cross[i]);
      return 1;
    }
  }
  std::puts("so3 ok");
  return 0;
}
