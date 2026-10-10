/// @file
/// @brief The linear-algebra setting the C++ modules share: Eigen's product blocking, fixed.
///
/// Eigen splits a large matrix product into blocks sized by the CPU's caches, and the blocks
/// set the order its sums run in. It reads the cache sizes from the core that runs the first
/// product, and on a hybrid CPU the core types report different caches: the same product then
/// rounded differently by where the process happened to start (an Intel Core Ultra's
/// performance and efficiency cores gave runs differing in their last bits, found in PR 10).
/// Fixed sizes make every product round the same on every core, as Rust's products already do.
/// The C++ entry points (the native program, the Python extension) fix them before any product.
#pragma once

#include <Eigen/Core>
#include <cstddef>

namespace slam_it_all::foundations::linalg {

/// @brief The L1 cache size products are blocked by, in bytes: Eigen's own x86 default.
inline constexpr std::ptrdiff_t kL1CacheBytes = std::ptrdiff_t{32} * 1024;

/// @brief The L2 cache size products are blocked by, in bytes: Eigen's own x86 default.
inline constexpr std::ptrdiff_t kL2CacheBytes = std::ptrdiff_t{256} * 1024;

/// @brief The L3 cache size products are blocked by, in bytes: Eigen's own x86 default.
inline constexpr std::ptrdiff_t kL3CacheBytes = std::ptrdiff_t{2} * 1024 * 1024;

/// @brief Fix Eigen's product blocking to `kL1CacheBytes`, `kL2CacheBytes` and `kL3CacheBytes`,
///        whatever core runs: call it before the first product.
inline void fix_product_blocking() {
  Eigen::setCpuCacheSizes(kL1CacheBytes, kL2CacheBytes, kL3CacheBytes);
}

}  // namespace slam_it_all::foundations::linalg
