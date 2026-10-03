/// @file
/// @brief PCG32, the random generator every backend shares, so a seed draws the same samples
///        everywhere.
///
/// Mirrors `slamlab.foundations.rng`, whose docstring states the algorithm: O'Neill's PCG32
/// (XSH RR), its unbiased bounded draw, and Floyd's algorithm for distinct indices. RANSAC
/// draws its samples from it, so the C++ and Python backends test the same hypotheses. It is
/// not bound to Python itself.
#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "slamlab/foundations/error.hpp"

/// @brief Random numbers shared by every backend.
namespace slamlab::foundations::rng {

/// @brief O'Neill's PCG32 (XSH RR, 64-bit state, 32-bit output).
class Pcg32 {
 public:
  /// @brief Seed a generator, as the reference library's `pcg32_srandom_r` does.
  /// @param seed The initial state.
  /// @param stream The stream (sequence) selector, below 2^63.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (seed, stream), as in pcg32_srandom_r
  Pcg32(std::uint64_t seed, std::uint64_t stream) : increment_((stream << 1U) | 1U) {
    next_u32();
    state_ += seed;
    next_u32();
  }

  /// @brief Draw the next 32-bit output.
  /// @return A uniform integer in [0, 2^32).
  std::uint32_t next_u32() {
    const std::uint64_t old = state_;
    state_ = (old * kMultiplier) + increment_;
    const auto xorshifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
    const auto rotation = static_cast<std::uint32_t>(old >> 59U);
    return (xorshifted >> rotation) | (xorshifted << ((0U - rotation) & 31U));
  }

  /// @brief Draw a uniform integer below `bound`, without modulo bias: outputs below
  ///        `2^32 mod bound` are rejected.
  /// @param bound The exclusive upper bound, at least 1.
  /// @return A uniform integer in [0, bound), or `InvalidInput` if `bound` is 0.
  Result<std::uint32_t> bounded(std::uint32_t bound) {
    if (bound == 0) {
      return fail(ErrorKind::kInvalidInput, "bound: expected 1 <= bound < 2**32, got 0");
    }
    return draw_below(bound);
  }

  /// @brief Draw `count` distinct indices from [0, population) (Floyd's algorithm), with
  ///        exactly `count` bounded draws.
  /// @param population The number of indices to choose from.
  /// @param count How many to draw.
  /// @return The indices in the order Floyd's algorithm adds them, or `InvalidInput` if
  ///         `count` exceeds `population`.
  Result<std::vector<std::uint32_t>> sample(std::uint32_t population, std::uint32_t count) {
    if (count > population) {
      return fail(ErrorKind::kInvalidInput, "sample: expected 0 <= count <= population, got " +
                                                std::to_string(count) + " of " +
                                                std::to_string(population));
    }
    std::vector<std::uint32_t> chosen;
    chosen.reserve(count);
    for (std::uint32_t j = population - count; j < population; ++j) {
      const std::uint32_t pick = draw_below(j + 1);
      const bool taken = std::ranges::find(chosen, pick) != chosen.end();
      chosen.push_back(taken ? j : pick);
    }
    return chosen;
  }

 private:
  /// @brief The bounded draw, for a bound already known to be at least 1.
  /// @param bound The exclusive upper bound, at least 1.
  /// @return A uniform integer in [0, bound).
  std::uint32_t draw_below(std::uint32_t bound) {
    const std::uint32_t threshold = (0U - bound) % bound;
    while (true) {
      const std::uint32_t value = next_u32();
      if (value >= threshold) {
        return value % bound;
      }
    }
  }

  /// @brief The LCG multiplier of the reference library.
  static constexpr std::uint64_t kMultiplier = 6364136223846793005ULL;
  /// @brief The 64-bit state.
  std::uint64_t state_ = 0;
  /// @brief The increment, odd: `2 * stream + 1`.
  std::uint64_t increment_;
};

}  // namespace slamlab::foundations::rng
