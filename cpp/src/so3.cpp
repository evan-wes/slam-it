#include "slamlab/so3.hpp"

namespace slamlab {

Mat3 hat(const Vec3& v) {
  return {{
      {0.0, -v[2], v[1]},
      {v[2], 0.0, -v[0]},
      {-v[1], v[0], 0.0},
  }};
}

}  // namespace slamlab
