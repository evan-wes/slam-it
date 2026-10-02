/// @file
/// @brief Implementation of the SO(3) helpers declared in slamlab/so3.hpp.
#include "slamlab/so3.hpp"

namespace slamlab {

Mat3 hat(const Vec3& v) noexcept {
  return {{
      {0.0, -v[2], v[1]},
      {v[2], 0.0, -v[0]},
      {-v[1], v[0], 0.0},
  }};
}

}  // namespace slamlab
