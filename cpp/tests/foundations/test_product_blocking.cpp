/// @file
/// @brief Native tests for Eigen's product blocking: every rung compiles with `EIGEN_NO_CPUID`
///        (CMakeLists.txt), so products are blocked by Eigen's own default cache sizes, the
///        same on every core, never by the caches of the core that runs the first product.
#include <Eigen/Core>

#include "test_support.hpp"

namespace {

using slam_it_all::testing::expect;

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
#ifndef EIGEN_NO_CPUID
  expect(false, "a rung's code is compiled with EIGEN_NO_CPUID", failures);
#endif
  expect(Eigen::l1CacheSize() == Eigen::internal::defaultL1CacheSize &&
             Eigen::l2CacheSize() == Eigen::internal::defaultL2CacheSize &&
             Eigen::l3CacheSize() == Eigen::internal::defaultL3CacheSize,
         "products are blocked by Eigen's default cache sizes", failures);
  return slam_it_all::testing::finish("foundations/product_blocking", failures);
}

}  // namespace

int main() { return run(); }
