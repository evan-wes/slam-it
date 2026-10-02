/// @file
/// @brief Implementation of the SO(3) functions declared in slamlab/foundations/so3.hpp.
#include "slamlab/foundations/so3.hpp"

namespace slamlab::foundations {

Mat3 hat(const Vec3& v) noexcept {
  return {{
      {0.0, -v[2], v[1]},
      {v[2], 0.0, -v[0]},
      {-v[1], v[0], 0.0},
  }};
}

}  // namespace slamlab::foundations
