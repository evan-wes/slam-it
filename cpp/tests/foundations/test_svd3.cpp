/// @file
/// @brief The 3x3 SVD every C++ module uses, `Eigen::JacobiSVD<Matrix3d>`, against the golden
///        fixture `tests/fixtures/foundations/svd3.json`, which Rust's `linalg::svd3` (a port of
///        the same algorithm) is held to bit for bit. Eigen is the oracle: this program also
///        writes the fixture, from its own hard matrices, with `--regenerate`:
///
///            build/native/foundations_test_svd3 --regenerate >
///            tests/fixtures/foundations/svd3.json
///
///        Run as a test, it fails if Eigen's factors moved (an Eigen upgrade, say), so the two
///        languages cannot drift apart unnoticed.
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "fixture_support.hpp"
#include "test_support.hpp"

namespace {

using slamlab::testing::expect;

/// @brief A deterministic generator for the matrices: an LCG, uniform in [-1, 1).
class Draws {
 public:
  /// @brief The next draw.
  /// @return A value in [-1, 1).
  double next() {
    constexpr std::uint64_t kMultiplier = 6364136223846793005ULL;
    constexpr std::uint64_t kIncrement = 1442695040888963407ULL;
    constexpr unsigned kMantissa = 53U;
    state_ = (state_ * kMultiplier) + kIncrement;
    return (static_cast<double>(state_ >> (64U - kMantissa)) /
            static_cast<double>(std::uint64_t{1} << kMantissa) * 2.0) -
           1.0;
  }

  /// @brief A matrix of draws.
  /// @return The matrix.
  Eigen::Matrix3d matrix() {
    Eigen::Matrix3d out;
    for (Eigen::Index k = 0; k < out.size(); ++k) {
      out(k) = next();
    }
    return out;
  }

  /// @brief A rotation about a drawn axis by a drawn angle.
  /// @return The rotation.
  Eigen::Matrix3d rotation() {
    const Eigen::Vector3d axis(next(), next(), next());
    return Eigen::AngleAxisd(2.0 * next(), axis.normalized()).toRotationMatrix();
  }

 private:
  std::uint64_t state_ = 12345;
};

/// @brief The hard matrices, named: random, graded, rank-deficient, repeated and near-repeated
///        singular values, signs, permutations, extreme scales.
/// @return The cases.
std::vector<std::pair<std::string, Eigen::Matrix3d>> matrices() {
  Draws draws;
  constexpr std::size_t kCases = 21;
  std::vector<std::pair<std::string, Eigen::Matrix3d>> out;
  out.reserve(kCases);
  for (int k = 0; k < 4; ++k) {
    out.emplace_back("random " + std::to_string(k), draws.matrix());
  }
  const auto between = [&draws](double a, double b, double c) {
    const Eigen::Matrix3d left = draws.rotation();
    return Eigen::Matrix3d(left * Eigen::Vector3d(a, b, c).asDiagonal() * draws.rotation());
  };
  out.emplace_back("graded to 1e-8", between(1.0, 1e-4, 1e-8));
  out.emplace_back("graded to 1e-15", between(1.0, 1e-7, 1e-15));
  out.emplace_back("rank 2", between(1.0, 1e-8, 0.0));
  out.emplace_back("essential: two equal, one zero", between(1.0, 1.0, 0.0));
  out.emplace_back("near-repeated", between(1.0, 1.0 + 1e-12, 0.5));
  out.emplace_back("three equal: a rotation", draws.rotation());
  out.emplace_back("a reflection", Eigen::Matrix3d(-draws.rotation()));
  const Eigen::Vector3d a(draws.next(), draws.next(), draws.next());
  const Eigen::Vector3d b(draws.next(), draws.next(), draws.next());
  out.emplace_back("rank 1", Eigen::Matrix3d(a * b.transpose()));
  out.emplace_back("diagonal, one negative", Eigen::Vector3d(1.0, -3.0, 2.0).asDiagonal());
  Eigen::Matrix3d permutation;
  permutation << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0;
  out.emplace_back("a permutation", permutation);
  Eigen::Matrix3d skew;
  skew << 0.0, -0.3, 0.2, 0.3, 0.0, -0.1, -0.2, 0.1, 0.0;
  out.emplace_back("skew plus 1e-12 I",
                   Eigen::Matrix3d(skew + (1e-12 * Eigen::Matrix3d::Identity())));
  out.emplace_back("zero", Eigen::Matrix3d::Zero());
  out.emplace_back("identity", Eigen::Matrix3d::Identity());
  out.emplace_back("minus identity", Eigen::Matrix3d(-Eigen::Matrix3d::Identity()));
  out.emplace_back("scaled by 1e-300", Eigen::Matrix3d(draws.matrix() * 1e-300));
  out.emplace_back("scaled by 1e300", Eigen::Matrix3d(draws.matrix() * 1e300));
  // Symmetric 2x2 blocks take the symmetrising rotation's identity branch (Umeyama's
  // covariance of a set aligned to itself is symmetric).
  const Eigen::Matrix3d spread = draws.matrix();
  out.emplace_back("symmetric: a covariance", Eigen::Matrix3d(spread.transpose() * spread));
  return out;
}

/// @brief A matrix's entries, row by row.
/// @param m The matrix.
/// @return Its nine entries.
std::vector<double> rows_of(const Eigen::Matrix3d& m) {
  std::vector<double> out;
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      out.push_back(m(r, c));
    }
  }
  return out;
}

/// @brief A matrix from its entries, row by row.
/// @param values Nine entries.
/// @return The matrix.
Eigen::Matrix3d from_rows(const std::vector<double>& values) {
  Eigen::Matrix3d out;
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      out(r, c) = values[static_cast<std::size_t>((3 * r) + c)];
    }
  }
  return out;
}

/// @brief Whether two matrices hold the same bits, signs of zero included (`==` equates 0.0
///        and -0.0).
/// @param a A matrix.
/// @param b Another, of the same size.
/// @return Whether every entry's bits agree.
template <typename A, typename B>
bool same_bits(const A& a, const B& b) {
  for (Eigen::Index k = 0; k < a.size(); ++k) {
    if (std::bit_cast<std::uint64_t>(a(k)) != std::bit_cast<std::uint64_t>(b(k))) {
      return false;
    }
  }
  return true;
}

/// @brief Eigen's factors of a matrix.
/// @param m The matrix.
/// @return `(U, singular values, V)`.
std::tuple<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix3d> factors(const Eigen::Matrix3d& m) {
  const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(m);
  return {svd.matrixU(), svd.singularValues(), svd.matrixV()};
}

/// @brief Write the fixture to stdout.
/// @return The exit code.
int regenerate() {
  nlohmann::json cases = nlohmann::json::array();
  for (const auto& [name, m] : matrices()) {
    const auto [u, s, v] = factors(m);
    cases.push_back({{"name", name},
                     {"matrix", rows_of(m)},
                     {"u", rows_of(u)},
                     {"singular_values", std::vector<double>{s(0), s(1), s(2)}},
                     {"v", rows_of(v)}});
  }
  const nlohmann::json fixture = {
      {"description",
       "Eigen::JacobiSVD<Matrix3d>'s factors (U, singular values, V; matrices row by row), the "
       "oracle Rust's linalg::svd3 is held to bit for bit. Written by "
       "build/native/foundations_test_svd3 --regenerate (cpp/tests/foundations/test_svd3.cpp)."},
      {"cases", cases}};
  std::cout << fixture.dump(1) << "\n";
  return 0;
}

/// @brief The test body: Eigen still gives the fixture's factors, exactly.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slamlab::testing::load_fixture("foundations/svd3.json");
  expect(fixture["cases"].size() == matrices().size(), "the fixture holds every case", failures);
  for (const auto& entry : fixture["cases"]) {
    const Eigen::Matrix3d m = from_rows(entry["matrix"].get<std::vector<double>>());
    const auto [u, s, v] = factors(m);
    const auto expected_s = entry["singular_values"].get<std::vector<double>>();
    const bool same = same_bits(u, from_rows(entry["u"].get<std::vector<double>>())) &&
                      same_bits(s, Eigen::Vector3d(expected_s[0], expected_s[1], expected_s[2])) &&
                      same_bits(v, from_rows(entry["v"].get<std::vector<double>>()));
    const std::string message =
        entry["name"].get<std::string>() + ": Eigen's factors are the fixture's";
    expect(same, message.c_str(), failures);
  }
  return slamlab::testing::finish("foundations/svd3", failures);
}

}  // namespace

int main(int argc, char** argv) {
  const std::span<char*> arguments(argv, static_cast<std::size_t>(argc));
  if (arguments.size() == 2 && std::string_view(arguments[1]) == "--regenerate") {
    return slamlab::testing::guarded("foundations/svd3 --regenerate", regenerate);
  }
  return slamlab::testing::guarded("foundations/svd3", run);
}
