/// @file
/// @brief Implementation of the 5-point solvers declared in slam_it_all/foundations/fivepoint.hpp.
#include "slam_it_all/foundations/fivepoint.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace slam_it_all::foundations::fivepoint {

namespace {

/// @brief A polynomial in (x, y, z) of degree up to three: coefficient of x^a y^b z^c at
///        `a * 16 + b * 4 + c`.
using Cubic = std::array<double, 64>;
/// @brief A monomial's exponents of (x, y, z).
using Monomial = std::array<int, 3>;
/// @brief Twenty monomials in a chosen order.
using Order = std::array<Monomial, 20>;
/// @brief A 3x3 matrix of polynomials of degree up to three.
using CubicMatrix = std::array<std::array<Cubic, 3>, 3>;
/// @brief A polynomial in one variable, ascending.
using Poly = std::vector<double>;

/// @brief Nistér's monomial order.
constexpr Order kNisterOrder = {{{3, 0, 0}, {0, 3, 0}, {2, 1, 0}, {1, 2, 0}, {2, 0, 1},
                                 {2, 0, 0}, {0, 2, 1}, {0, 2, 0}, {1, 1, 1}, {1, 1, 0},
                                 {1, 0, 2}, {1, 0, 1}, {1, 0, 0}, {0, 1, 2}, {0, 1, 1},
                                 {0, 1, 0}, {0, 0, 3}, {0, 0, 2}, {0, 0, 1}, {0, 0, 0}}};
/// @brief Graded reverse lexicographic order.
constexpr Order kGrevlexOrder = {{{3, 0, 0}, {2, 1, 0}, {2, 0, 1}, {1, 2, 0}, {1, 1, 1},
                                  {1, 0, 2}, {0, 3, 0}, {0, 2, 1}, {0, 1, 2}, {0, 0, 3},
                                  {2, 0, 0}, {1, 1, 0}, {1, 0, 1}, {0, 2, 0}, {0, 1, 1},
                                  {0, 0, 2}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}};

/// @brief The first of Nistér's row pairs (x^2 z, x^2), (y^2 z, y^2), (xyz, xy): rows 4, 6 and 8
///        of the reduced matrix, each followed by its partner (see `_NISTER_ROW_PAIRS`).
constexpr std::array<Eigen::Index, 3> kNisterRowPairs = {4, 6, 8};

/// @brief A 3x3 determinant's six terms: the column taken from each row, and the permutation's
///        sign (see `_DETERMINANT_TERMS`).
constexpr std::array<std::tuple<std::size_t, std::size_t, std::size_t, double>, 6>
    kDeterminantTerms = {{{0, 1, 2, 1.0},
                          {1, 2, 0, 1.0},
                          {2, 0, 1, 1.0},
                          {0, 2, 1, -1.0},
                          {1, 0, 2, -1.0},
                          {2, 1, 0, -1.0}}};

/// @brief The flat index of a monomial.
/// @param a The exponent of x.
/// @param b The exponent of y.
/// @param c The exponent of z.
/// @return `a * 16 + b * 4 + c`.
constexpr std::size_t at(int a, int b, int c) {
  return (static_cast<std::size_t>(a) * 16) + (static_cast<std::size_t>(b) * 4) +
         static_cast<std::size_t>(c);
}

/// @brief Multiply two polynomials (the product must stay within degree three).
/// @param p A polynomial.
/// @param q A polynomial.
/// @return The product.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): multiplication commutes
Cubic multiply(const Cubic& p, const Cubic& q) {
  Cubic out{};
  for (int a = 0; a < 4; ++a) {
    for (int b = 0; a + b < 4; ++b) {
      for (int c = 0; a + b + c < 4; ++c) {
        const double left = p[at(a, b, c)];
        if (left == 0.0) {
          continue;
        }
        for (int d = 0; a + b + c + d < 4; ++d) {
          for (int e = 0; a + b + c + d + e < 4; ++e) {
            for (int f = 0; a + b + c + d + e + f < 4; ++f) {
              out[at(a + d, b + e, c + f)] += left * q[at(d, e, f)];
            }
          }
        }
      }
    }
  }
  return out;
}

/// @brief Add `scale * q` to `p`.
/// @param p The polynomial to accumulate into.
/// @param q The polynomial to add.
/// @param scale Its factor.
void accumulate(Cubic& p, const Cubic& q, double scale) {
  for (std::size_t k = 0; k < p.size(); ++k) {
    p[k] += scale * q[k];
  }
}

/// @brief The null-space basis X, Y, Z, W of five pairs' epipolar equations.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @return The basis, or nothing if the system's rank is below five.
std::optional<std::array<Eigen::Matrix3d, 4>> null_space(
    const Eigen::Ref<const correspondences::Rows3>& bearings1,
    const Eigen::Ref<const correspondences::Rows3>& bearings2) {
  Eigen::Matrix<double, 9, 9> system = Eigen::Matrix<double, 9, 9>::Zero();
  for (Eigen::Index i = 0; i < 5; ++i) {
    for (Eigen::Index r = 0; r < 3; ++r) {
      system.block<1, 3>(i, 3 * r) = bearings2(i, r) * bearings1.row(i);
    }
  }
  const Eigen::JacobiSVD<Eigen::Matrix<double, 9, 9>, Eigen::ComputeFullV> svd(system);
  const auto& singular = svd.singularValues();
  if (!(singular(4) > correspondences::kRankTolerance * singular(0))) {
    return std::nullopt;
  }
  std::array<Eigen::Matrix3d, 4> basis{};
  for (std::size_t k = 0; k < 4; ++k) {
    const Eigen::Matrix<double, 9, 1> column = svd.matrixV().col(5 + static_cast<Eigen::Index>(k));
    basis.at(k) = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(column.data());
  }
  return basis;
}

/// @brief The ten cubic constraints on `E = x X + y Y + z Z + W`, in a monomial order.
/// @param basis X, Y, Z, W.
/// @param order The monomial order of the columns.
/// @return The 10x20 coefficient matrix (det E first, then the trace constraint row-major).
Eigen::Matrix<double, 10, 20> constraints(const std::array<Eigen::Matrix3d, 4>& basis,
                                          const Order& order) {
  CubicMatrix e{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Cubic& entry = e.at(i).at(j);
      entry[at(1, 0, 0)] = basis[0](i, j);
      entry[at(0, 1, 0)] = basis[1](i, j);
      entry[at(0, 0, 1)] = basis[2](i, j);
      entry[at(0, 0, 0)] = basis[3](i, j);
    }
  }
  CubicMatrix eet{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        accumulate(eet.at(i).at(j), multiply(e.at(i).at(k), e.at(j).at(k)), 1.0);
      }
    }
  }
  Cubic trace = eet[0][0];
  accumulate(trace, eet[1][1], 1.0);
  accumulate(trace, eet[2][2], 1.0);
  std::array<Cubic, 10> rows{};
  Cubic& determinant = rows[0];
  for (int k = 0; k < 3; ++k) {
    const int k1 = (k + 1) % 3;
    const int k2 = (k + 2) % 3;
    Cubic cofactor = multiply(e[1].at(k1), e[2].at(k2));
    accumulate(cofactor, multiply(e[1].at(k2), e[2].at(k1)), -1.0);
    accumulate(determinant, multiply(e[0].at(k), cofactor), 1.0);
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Cubic& cubic = rows.at(1 + (3 * static_cast<std::size_t>(i)) + static_cast<std::size_t>(j));
      accumulate(cubic, multiply(trace, e.at(i).at(j)), -1.0);
      for (int k = 0; k < 3; ++k) {
        accumulate(cubic, multiply(eet.at(i).at(k), e.at(k).at(j)), 2.0);
      }
    }
  }
  Eigen::Matrix<double, 10, 20> out;
  for (std::size_t r = 0; r < 10; ++r) {
    for (std::size_t c = 0; c < 20; ++c) {
      const Monomial& m = order.at(c);
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
          rows.at(r)[at(m[0], m[1], m[2])];
    }
  }
  return out;
}

/// @brief Reduce a 10x20 matrix to `[I | B]` by partial pivoting.
/// @param matrix The matrix.
/// @return B, or nothing if the left block is singular.
std::optional<Eigen::Matrix<double, 10, 10>> gauss_jordan(Eigen::Matrix<double, 10, 20> matrix) {
  for (Eigen::Index col = 0; col < 10; ++col) {
    Eigen::Index pivot = 0;
    matrix.col(col).tail(10 - col).cwiseAbs().maxCoeff(&pivot);
    pivot += col;
    if (!(std::abs(matrix(pivot, col)) > 0.0)) {
      return std::nullopt;
    }
    matrix.row(col).swap(matrix.row(pivot));
    matrix.row(col) /= matrix(col, col);
    for (Eigen::Index row = 0; row < 10; ++row) {
      if (row != col) {
        matrix.row(row) -= matrix(row, col) * matrix.row(col);
      }
    }
  }
  return matrix.rightCols<10>();
}

/// @brief Compose E from its coefficients in the null-space basis, scale it to singular values
///        (1, 1, 0), and keep it if it is essential (see `_keep` in the Python reference).
/// @param solutions The solutions so far.
/// @param basis X, Y, Z, W.
/// @param xyz x, y, z.
void keep(std::vector<Eigen::Matrix3d>& solutions, const std::array<Eigen::Matrix3d, 4>& basis,
          const Eigen::Vector3d& xyz) {
  const Eigen::Matrix3d essential =
      (xyz(0) * basis[0]) + (xyz(1) * basis[1]) + (xyz(2) * basis[2]) + basis[3];
  const Eigen::Matrix3d scaled = essential * (std::numbers::sqrt2 / essential.norm());
  const Eigen::Vector3d singular = Eigen::JacobiSVD<Eigen::Matrix3d>(scaled).singularValues();
  if (!(std::max(singular(0) - singular(1), singular(2)) <= kEssentialTolerance)) {
    return;
  }
  for (const Eigen::Matrix3d& kept : solutions) {
    if (std::min((kept - scaled).cwiseAbs().maxCoeff(), (kept + scaled).cwiseAbs().maxCoeff()) <=
        kDuplicateTolerance) {
      return;
    }
  }
  solutions.push_back(scaled);
}

// --- polynomials in one variable ------------------------------------------------------------

/// @brief Evaluate a polynomial (Horner).
/// @param p Coefficients, ascending.
/// @param z The point.
/// @return p(z).
double evaluate(const Poly& p, double z) {
  double value = 0.0;
  for (const double c : std::views::reverse(p)) {
    value = (value * z) + c;
  }
  return value;
}

/// @brief Drop trailing zero coefficients (keeping at least one).
/// @param p The polynomial.
/// @return The trimmed polynomial.
Poly trim(Poly p) {
  while (p.size() > 1 && p.back() == 0.0) {
    p.pop_back();
  }
  return p;
}

/// @brief The product of two polynomials.
/// @param p A polynomial.
/// @param q A polynomial.
/// @return p q.
Poly product(const Poly& p, const Poly& q) {
  Poly out(p.size() + q.size() - 1, 0.0);
  for (std::size_t i = 0; i < p.size(); ++i) {
    for (std::size_t j = 0; j < q.size(); ++j) {
      out[i + j] += p[i] * q[j];
    }
  }
  return out;
}

/// @brief `p + scale * q`.
/// @param p A polynomial.
/// @param q A polynomial.
/// @param scale The factor.
/// @return The sum.
Poly sum(const Poly& p, const Poly& q, double scale) {
  Poly out(std::max(p.size(), q.size()), 0.0);
  for (std::size_t i = 0; i < p.size(); ++i) {
    out[i] += p[i];
  }
  for (std::size_t i = 0; i < q.size(); ++i) {
    out[i] += scale * q[i];
  }
  return out;
}

/// @brief The remainder of polynomial division `numerator / denominator`.
/// @param numerator The numerator.
/// @param denominator The denominator (non-zero leading coefficient).
/// @return The remainder.
Poly remainder(Poly numerator, const Poly& denominator) {
  const std::size_t d = denominator.size() - 1;
  while (numerator.size() > d && numerator.size() > 1) {
    const double factor = numerator.back() / denominator.back();
    const std::size_t shift = numerator.size() - 1 - d;
    for (std::size_t i = 0; i <= d; ++i) {
      numerator[shift + i] -= factor * denominator[i];
    }
    numerator.pop_back();
  }
  return numerator.empty() ? Poly{0.0} : numerator;
}

/// @brief The derivative of a polynomial.
/// @param p The polynomial.
/// @return p'.
Poly derivative(const Poly& p) {
  if (p.size() < 2) {
    return {0.0};
  }
  Poly out(p.size() - 1);
  for (std::size_t i = 1; i < p.size(); ++i) {
    out[i - 1] = static_cast<double>(i) * p[i];
  }
  return out;
}

/// @brief The Sturm sequence of a polynomial.
/// @param p The polynomial.
/// @return p, p', -rem(p, p'), ...
std::vector<Poly> sturm_chain(const Poly& p) {
  std::vector<Poly> chain = {p, derivative(p)};
  while (chain.back().size() > 1 || chain.back()[0] != 0.0) {
    Poly next = remainder(chain[chain.size() - 2], chain.back());
    for (double& c : next) {
      c = -c;
    }
    next = trim(next);
    if (next.size() == 1 && next[0] == 0.0) {
      break;
    }
    chain.push_back(next);
    if (next.size() == 1) {
      break;
    }
  }
  return chain;
}

/// @brief The sign of a number (0 for zero).
/// @param value The number.
/// @return -1, 0 or 1.
double sign_of(double value) {
  if (value > 0.0) {
    return 1.0;
  }
  return value < 0.0 ? -1.0 : 0.0;
}

/// @brief The sign changes of a Sturm sequence at a point.
/// @param chain The sequence.
/// @param z The point.
/// @return The count.
int sign_changes(const std::vector<Poly>& chain, double z) {
  int changes = 0;
  double last = 0.0;
  for (const Poly& p : chain) {
    const double value = evaluate(p, z);
    const double sign = sign_of(value);
    if (sign == 0.0) {
      continue;
    }
    if (last != 0.0 && sign != last) {
      ++changes;
    }
    last = sign;
  }
  return changes;
}

/// @brief Refine a root bracketed by a sign change.
/// @param p The polynomial.
/// @param low The bracket's low end.
/// @param high The bracket's high end.
/// @return The root.
double bisect(const Poly& p, double low, double high) {
  double f_low = evaluate(p, low);
  while (high - low > kRootTolerance * std::max({1.0, std::abs(low), std::abs(high)})) {
    const double middle = 0.5 * (low + high);
    const double f_middle = evaluate(p, middle);
    if (f_middle == 0.0) {
      return middle;
    }
    if ((f_middle < 0.0) == (f_low < 0.0)) {
      low = middle;
      f_low = f_middle;
    } else {
      high = middle;
    }
  }
  return 0.5 * (low + high);
}

/// @brief A row's tail split into the polynomials in z multiplying x, y and 1.
/// @param tail Coefficients over `xz^2 xz x yz^2 yz y z^3 z^2 z 1`.
/// @return The three polynomials, ascending in z.
std::array<Poly, 3> z_polynomials(const Eigen::Matrix<double, 1, 10>& tail) {
  return {Poly{tail(2), tail(1), tail(0)}, Poly{tail(5), tail(4), tail(3)},
          Poly{tail(9), tail(8), tail(7), tail(6)}};
}

/// @brief The monomials of an order evaluated at (x, y, z), and their derivatives.
/// @param order The monomial order.
/// @param xyz The point.
/// @return The 20 values, and their 20x3 gradient.
std::pair<Eigen::Matrix<double, 20, 1>, Eigen::Matrix<double, 20, 3>> monomials(
    const Order& order, const Eigen::Vector3d& xyz) {
  const auto power = [&xyz](const Monomial& e) {
    return std::pow(xyz(0), e[0]) * std::pow(xyz(1), e[1]) * std::pow(xyz(2), e[2]);
  };
  Eigen::Matrix<double, 20, 1> values;
  Eigen::Matrix<double, 20, 3> slope = Eigen::Matrix<double, 20, 3>::Zero();
  for (std::size_t k = 0; k < order.size(); ++k) {
    const Monomial& e = order.at(k);
    const auto row = static_cast<Eigen::Index>(k);
    values(row) = power(e);
    for (std::size_t v = 0; v < 3; ++v) {
      if (e.at(v) == 0) {
        continue;
      }
      Monomial lowered = e;
      lowered.at(v) -= 1;
      slope(row, static_cast<Eigen::Index>(v)) = e.at(v) * power(lowered);
    }
  }
  return {values, slope};
}

/// @brief Refine (x, y, z) by Gauss-Newton on the ten cubic constraints, to convergence (see
///        `_polish` in the Python reference).
/// @param m The 10x20 constraint matrix.
/// @param order Its monomial order.
/// @param xyz The starting point.
/// @return The polished point.
Eigen::Vector3d polish(const Eigen::Matrix<double, 10, 20>& m, const Order& order,
                       Eigen::Vector3d xyz) {
  auto evaluated = monomials(order, xyz);
  Eigen::Matrix<double, 10, 1> residual = m * evaluated.first;
  for (int step = 0; step < kPolishMaxSteps; ++step) {
    const Eigen::Matrix<double, 10, 3> jacobian = m * evaluated.second;
    const Eigen::Vector3d delta =
        Eigen::JacobiSVD<Eigen::Matrix<double, 10, 3>, Eigen::ComputeThinU | Eigen::ComputeThinV>(
            jacobian)
            .solve(-residual);
    if (!delta.allFinite()) {
      break;
    }
    const Eigen::Vector3d moved = xyz + delta;
    auto moved_evaluated = monomials(order, moved);
    const Eigen::Matrix<double, 10, 1> moved_residual = m * moved_evaluated.first;
    if (!(moved_residual.norm() < residual.norm())) {
      break;
    }
    xyz = moved;
    evaluated = std::move(moved_evaluated);
    residual = moved_residual;
    if (delta.norm() <= kPolishTolerance * std::max(1.0, xyz.norm())) {
      break;
    }
  }
  return xyz;
}

/// @brief The system both methods start from.
struct System {
  /// @brief X, Y, Z, W: E's null-space basis.
  std::array<Eigen::Matrix3d, 4> basis;
  /// @brief The ten cubic constraints over the twenty monomials.
  Eigen::Matrix<double, 10, 20> constraints;
  /// @brief B, the constraints' Gauss-Jordan reduction.
  Eigen::Matrix<double, 10, 10> reduced;
};

/// @brief The null space, its constraints in a monomial order, and their reduction.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @param order The monomial order.
/// @return The system, or nothing for a degenerate sample.
std::optional<System> set_up(const Eigen::Ref<const correspondences::Rows3>& bearings1,
                             const Eigen::Ref<const correspondences::Rows3>& bearings2,
                             const Order& order) {
  const auto basis = null_space(bearings1, bearings2);
  if (!basis) {
    return std::nullopt;
  }
  const Eigen::Matrix<double, 10, 20> m = constraints(*basis, order);
  const auto reduced = gauss_jordan(m);
  if (!reduced) {
    return std::nullopt;
  }
  return System{.basis = *basis, .constraints = m, .reduced = *reduced};
}

/// @brief Nistér's method.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @return The real solutions.
std::vector<Eigen::Matrix3d> nister(const Eigen::Ref<const correspondences::Rows3>& bearings1,
                                    const Eigen::Ref<const correspondences::Rows3>& bearings2) {
  const auto system = set_up(bearings1, bearings2, kNisterOrder);
  if (!system) {
    return {};
  }
  const auto& reduced = system->reduced;
  // Each pair's first row minus z times its second: three rows of polynomials in z.
  std::array<std::array<Poly, 3>, 3> rows;
  for (std::size_t r = 0; r < 3; ++r) {
    const Eigen::Index upper = kNisterRowPairs.at(r);
    const auto a = z_polynomials(reduced.row(upper));
    const auto b = z_polynomials(reduced.row(upper + 1));
    for (std::size_t c = 0; c < 3; ++c) {
      Poly shifted(b.at(c).size() + 1, 0.0);
      std::ranges::copy(b.at(c), shifted.begin() + 1);
      rows.at(r).at(c) = sum(a.at(c), shifted, -1.0);
    }
  }
  Poly determinant = {0.0};
  for (const auto& [i, j, k, sign] : kDeterminantTerms) {
    const Poly term = product(product(rows[0].at(i), rows[1].at(j)), rows[2].at(k));
    determinant = sum(determinant, term, sign);
  }
  std::vector<Eigen::Matrix3d> solutions;
  for (const double root : real_roots(determinant)) {
    Eigen::Matrix3d b;
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        b(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
            evaluate(rows.at(r).at(c), root);
      }
    }
    const Eigen::Vector3d v = b.row(0).transpose().cross(b.row(1).transpose());
    if (!(std::abs(v(2)) > 0.0)) {
      continue;
    }
    const Eigen::Vector3d xyz =
        polish(system->constraints, kNisterOrder, Eigen::Vector3d(v(0) / v(2), v(1) / v(2), root));
    keep(solutions, system->basis, xyz);
  }
  return solutions;
}

/// @brief Stewénius' action-matrix method.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @return The real solutions.
std::vector<Eigen::Matrix3d> stewenius(const Eigen::Ref<const correspondences::Rows3>& bearings1,
                                       const Eigen::Ref<const correspondences::Rows3>& bearings2) {
  const auto system = set_up(bearings1, bearings2, kGrevlexOrder);
  if (!system) {
    return {};
  }
  // x b for b = (x^2 xy xz y^2 yz z^2 x y z 1): six cubics from B, four shifts within b.
  Eigen::Matrix<double, 10, 10> action = Eigen::Matrix<double, 10, 10>::Zero();
  action.topRows<6>() = -system->reduced.topRows<6>();
  for (const auto& [row, index] :
       {std::pair{6, 0}, std::pair{7, 1}, std::pair{8, 2}, std::pair{9, 6}}) {
    action(row, index) = 1.0;
  }
  const Eigen::EigenSolver<Eigen::Matrix<double, 10, 10>> eigen(action);
  const Eigen::Matrix<std::complex<double>, 10, 10> vectors = eigen.eigenvectors();
  std::vector<Eigen::Matrix3d> solutions;
  for (Eigen::Index k = 0; k < 10; ++k) {
    const std::complex<double> value = eigen.eigenvalues()(k);
    const Eigen::Matrix<std::complex<double>, 10, 1> vector = vectors.col(k);
    if (std::abs(value.imag()) > kRealTolerance * std::max(1.0, std::abs(value)) ||
        !(std::abs(vector(9)) > 0.0)) {
      continue;
    }
    const Eigen::Vector3d xyz =
        polish(system->constraints, kGrevlexOrder,
               Eigen::Vector3d((vector(6) / vector(9)).real(), (vector(7) / vector(9)).real(),
                               (vector(8) / vector(9)).real()));
    keep(solutions, system->basis, xyz);
  }
  return solutions;
}

/// @brief A matrix's entries in row-major order, the canonical order's key.
/// @param m The matrix.
/// @return Its nine entries, row by row.
std::array<double, 9> row_major(const Eigen::Matrix3d& m) {
  std::array<double, 9> out{};
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      out.at(static_cast<std::size_t>((3 * r) + c)) = m(r, c);
    }
  }
  return out;
}

/// @brief Put the solutions in canonical order (`slam_it_all.foundations.fivepoint`'s "Canonical
///        order"): each signed so that its first entry, row by row, within `kSignTieRtol` of
///        the largest magnitude is positive, then all sorted lexicographically by their
///        row-major entries.
/// @param solutions The solutions.
/// @return The same solutions, signed and sorted.
std::vector<Eigen::Matrix3d> canonical(std::vector<Eigen::Matrix3d> solutions) {
  for (Eigen::Matrix3d& essential : solutions) {
    const std::array<double, 9> entries = row_major(essential);
    double largest = 0.0;
    for (const double entry : entries) {
      largest = std::max(largest, std::abs(entry));
    }
    std::size_t first = 0;
    while (std::abs(entries.at(first)) < largest * (1.0 - kSignTieRtol)) {
      ++first;
    }
    if (!(entries.at(first) > 0.0)) {
      essential = -essential;
    }
  }
  std::ranges::sort(solutions, [](const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    return std::ranges::lexicographical_compare(row_major(a), row_major(b));
  });
  return solutions;
}

}  // namespace

const char* name(Solver solver) {
  switch (solver) {
    case Solver::kNister:
      return "nister";
    case Solver::kStewenius:
      return "stewenius";
    case Solver::kCount:
      break;
  }
  return "unknown";
}

std::vector<double> real_roots(std::vector<double> poly) {
  Poly p = trim(std::move(poly));
  if (p.size() < 2) {
    return {};
  }
  const double lead = p.back();
  for (double& c : p) {
    c /= lead;
  }
  double bound = 0.0;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    bound = std::max(bound, std::abs(p[i]));
  }
  bound += 1.0;
  const std::vector<Poly> chain = sturm_chain(p);
  struct Interval {
    double low;
    double high;
    int s_low;
    int s_high;
  };
  std::vector<Interval> stack = {{.low = -bound,
                                  .high = bound,
                                  .s_low = sign_changes(chain, -bound),
                                  .s_high = sign_changes(chain, bound)}};
  std::vector<double> roots;
  while (!stack.empty()) {
    const Interval interval = stack.back();
    stack.pop_back();
    const int count = interval.s_low - interval.s_high;
    if (count <= 0) {
      continue;
    }
    if (count == 1 && sign_of(evaluate(p, interval.low)) != sign_of(evaluate(p, interval.high))) {
      roots.push_back(bisect(p, interval.low, interval.high));
      continue;
    }
    if (interval.high - interval.low < kRootTolerance * std::max(1.0, std::abs(interval.low))) {
      roots.push_back(0.5 * (interval.low + interval.high));
      continue;
    }
    const double middle = 0.5 * (interval.low + interval.high);
    const int s_middle = sign_changes(chain, middle);
    stack.push_back(
        {.low = middle, .high = interval.high, .s_low = s_middle, .s_high = interval.s_high});
    stack.push_back(
        {.low = interval.low, .high = middle, .s_low = interval.s_low, .s_high = s_middle});
  }
  std::ranges::sort(roots);
  return roots;
}

std::vector<Eigen::Matrix3d> solve(const Eigen::Ref<const correspondences::Rows3>& bearings1,
                                   const Eigen::Ref<const correspondences::Rows3>& bearings2,
                                   Solver solver) {
  switch (solver) {
    case Solver::kNister:
      return canonical(nister(bearings1, bearings2));
    case Solver::kStewenius:
      return canonical(stewenius(bearings1, bearings2));
    case Solver::kCount:
      break;
  }
  return {};
}

Result<std::vector<Eigen::Matrix3d>> essential_5pt(
    const Eigen::Ref<const correspondences::Rows3>& bearings1,
    const Eigen::Ref<const correspondences::Rows3>& bearings2, Solver solver) {
  if (solver != Solver::kNister && solver != Solver::kStewenius) {
    return fail(ErrorKind::kInvalidInput, "solver: unknown code");
  }
  if (auto ok = correspondences::check_bearing_pairs(bearings1, bearings2, kSampleSize, false);
      !ok) {
    return std::unexpected(ok.error());
  }
  if (bearings1.rows() != kSampleSize) {
    return fail(ErrorKind::kInvalidInput,
                "bearings: expected exactly 5 pairs, got " + std::to_string(bearings1.rows()));
  }
  return solve(bearings1, bearings2, solver);
}

}  // namespace slam_it_all::foundations::fivepoint
