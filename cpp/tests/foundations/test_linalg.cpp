/// @file
/// @brief Native tests for the shared linear-algebra setting
///        (slam_it_all/foundations/linalg.hpp): products are blocked by fixed cache sizes,
///        whatever another setting or the core's own caches were.
#include <Eigen/Core>
#include <cstddef>

#include "slam_it_all/foundations/linalg.hpp"
#include "test_support.hpp"

namespace {

namespace linalg = slam_it_all::foundations::linalg;
using slam_it_all::testing::expect;

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  constexpr std::ptrdiff_t kKiB = 1024;
  Eigen::setCpuCacheSizes(48 * kKiB, 3 * kKiB * 1024, 36 * kKiB * 1024);  // a core's own
  linalg::fix_product_blocking();
  expect(Eigen::l1CacheSize() == linalg::kL1CacheBytes &&
             Eigen::l2CacheSize() == linalg::kL2CacheBytes &&
             Eigen::l3CacheSize() == linalg::kL3CacheBytes,
         "products are blocked by the fixed cache sizes", failures);
  return slam_it_all::testing::finish("foundations/linalg", failures);
}

}  // namespace

int main() { return run(); }
