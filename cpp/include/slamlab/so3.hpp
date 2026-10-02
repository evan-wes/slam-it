#pragma once

#include <array>

namespace slamlab {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

// hat(v): the skew-symmetric matrix [v]x, so that hat(v) * u == cross(v, u).
// Maps a 3-vector into the Lie algebra so(3). A placeholder proving the build
// pipeline; rung 0's real SO(3) work arrives with milestone 1.
Mat3 hat(const Vec3& v);

}  // namespace slamlab
