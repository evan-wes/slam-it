//! The 5-point essential matrix: the Rust core's Nistér and Stewénius solvers.
//!
//! Mirrors `slamlab.foundations.fivepoint`, whose docstring states the maths: the null space
//! of five bearing pairs' epipolar equations, Demazure's ten cubic constraints, then either
//! Nistér's degree-10 polynomial (real roots by Sturm sequences and bisection) or Stewénius'
//! 10x10 action matrix (real eigenvectors), each solution then polished by Gauss-Newton on the
//! constraints. Every real solution that is essential is returned, scaled to singular values
//! (1, 1, 0).
//!
//! nalgebra has no eigenvectors for a general real matrix, so Stewénius' method takes the
//! action matrix's eigenvalues from a (capped) real Schur decomposition and each real
//! eigenvalue's eigenvector as the null vector of `A - λ I`, by SVD: the same vector Eigen and
//! numpy return, up to scale, which the method divides out.

use std::f64::consts::SQRT_2;

use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, Schur, Vector3};

use crate::correspondences::{RANK_TOLERANCE, check_bearing_pairs};
use crate::error::{ErrorKind, Result, fail};
use crate::linalg::{SVD_MAX_ITERATIONS, least_squares, svd};

/// Correspondences a 5-point solver takes.
pub const SAMPLE_SIZE: usize = 5;
/// At most this many Gauss-Newton steps polish each solution on the ten cubics.
pub const POLISH_MAX_STEPS: usize = 10;
/// Polishing stops when a step is below this, relative to the solution (at least 1).
pub const POLISH_TOLERANCE: f64 = 1e-14;
/// An eigenvalue counts as real when its imaginary part is below this, relative to its
/// modulus (at least 1).
pub const REAL_TOLERANCE: f64 = 1e-10;
/// Bisection stops when a root's bracket is below this, relative to the root (at least 1).
pub const ROOT_TOLERANCE: f64 = 1e-14;
/// A solution is kept only if, scaled to norm √2, `s1 - s2` and `s3` are both below this.
pub const ESSENTIAL_TOLERANCE: f64 = 1e-6;
/// A solution within this (max-abs, up to sign) of one already kept is the same solution.
pub const DUPLICATE_TOLERANCE: f64 = 1e-8;

/// The solvers, with the codes they cross the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Solver {
    /// Nistér's polynomial and Sturm sequence.
    Nister = 0,
    /// Stewénius' action matrix.
    Stewenius = 1,
}

impl Solver {
    /// Every solver, in code order.
    pub const ALL: [Self; 2] = [Self::Nister, Self::Stewenius];

    /// The solver's name, as `slamlab.foundations.fivepoint.SOLVERS` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Nister => "nister",
            Self::Stewenius => "stewenius",
        }
    }
}

/// A polynomial in (x, y, z) of degree up to three: coefficient of x^a y^b z^c at
/// `a * 16 + b * 4 + c`.
type Cubic = [f64; 64];
/// A monomial's exponents of (x, y, z).
type Monomial = [usize; 3];
/// Twenty monomials in a chosen order.
type Order = [Monomial; 20];
/// A 3x3 matrix of polynomials of degree up to three.
type CubicMatrix = [[Cubic; 3]; 3];
/// The ten cubic constraints over twenty monomials.
type Constraints = SMatrix<f64, 10, 20>;

/// Nistér's monomial order.
const NISTER_ORDER: Order = [
    [3, 0, 0],
    [0, 3, 0],
    [2, 1, 0],
    [1, 2, 0],
    [2, 0, 1],
    [2, 0, 0],
    [0, 2, 1],
    [0, 2, 0],
    [1, 1, 1],
    [1, 1, 0],
    [1, 0, 2],
    [1, 0, 1],
    [1, 0, 0],
    [0, 1, 2],
    [0, 1, 1],
    [0, 1, 0],
    [0, 0, 3],
    [0, 0, 2],
    [0, 0, 1],
    [0, 0, 0],
];
/// Graded reverse lexicographic order.
const GREVLEX_ORDER: Order = [
    [3, 0, 0],
    [2, 1, 0],
    [2, 0, 1],
    [1, 2, 0],
    [1, 1, 1],
    [1, 0, 2],
    [0, 3, 0],
    [0, 2, 1],
    [0, 1, 2],
    [0, 0, 3],
    [2, 0, 0],
    [1, 1, 0],
    [1, 0, 1],
    [0, 2, 0],
    [0, 1, 1],
    [0, 0, 2],
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    [0, 0, 0],
];
/// The first of Nistér's row pairs (x²z, x²), (y²z, y²), (xyz, xy): rows 4, 6 and 8 of the
/// reduced matrix, each followed by its partner.
const NISTER_ROW_PAIRS: [usize; 3] = [4, 6, 8];
/// A 3x3 determinant's six terms: the column taken from each row, and the permutation's sign.
const DETERMINANT_TERMS: [(usize, usize, usize, f64); 6] = [
    (0, 1, 2, 1.0),
    (1, 2, 0, 1.0),
    (2, 0, 1, 1.0),
    (0, 2, 1, -1.0),
    (1, 0, 2, -1.0),
    (2, 1, 0, -1.0),
];

/// The flat index of a monomial.
///
/// # Arguments
///
/// * `a`, `b`, `c` - The exponents of x, y and z.
///
/// # Returns
///
/// `a * 16 + b * 4 + c`.
const fn at(a: usize, b: usize, c: usize) -> usize {
    a * 16 + b * 4 + c
}

/// Multiplies two polynomials (the product must stay within degree three).
///
/// # Arguments
///
/// * `p`, `q` - The polynomials.
///
/// # Returns
///
/// The product.
fn multiply(p: &Cubic, q: &Cubic) -> Cubic {
    let mut out = [0.0; 64];
    for a in 0..4 {
        for b in 0..4 - a {
            for c in 0..4 - a - b {
                let left = p[at(a, b, c)];
                if left == 0.0 {
                    continue;
                }
                for d in 0..4 - a - b - c {
                    for e in 0..4 - a - b - c - d {
                        for f in 0..4 - a - b - c - d - e {
                            out[at(a + d, b + e, c + f)] += left * q[at(d, e, f)];
                        }
                    }
                }
            }
        }
    }
    out
}

/// Adds `scale * q` to `p`.
///
/// # Arguments
///
/// * `p` - The polynomial to accumulate into.
/// * `q` - The polynomial to add.
/// * `scale` - Its factor.
fn accumulate(p: &mut Cubic, q: &Cubic, scale: f64) {
    for (pk, qk) in p.iter_mut().zip(q) {
        *pk += scale * qk;
    }
}

/// The null-space basis X, Y, Z, W of five pairs' epipolar equations.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - The pairs.
///
/// # Returns
///
/// The basis, or `None` if the system's rank is below five.
fn null_space(bearings1: &[Vector3<f64>], bearings2: &[Vector3<f64>]) -> Option<[Matrix3<f64>; 4]> {
    let mut system = DMatrix::zeros(9, 9);
    for (i, (b1, b2)) in bearings1.iter().zip(bearings2).enumerate() {
        for r in 0..3 {
            for c in 0..3 {
                system[(i, 3 * r + c)] = b2[r] * b1[c];
            }
        }
    }
    let decomposition = svd(system, false, true)?;
    if !(decomposition.singular_values[4] > RANK_TOLERANCE * decomposition.singular_values[0]) {
        return None;
    }
    let v_t = decomposition.v_t?;
    Some(std::array::from_fn(|k| {
        Matrix3::from_row_slice(v_t.row(5 + k).transpose().as_slice())
    }))
}

/// The ten cubic constraints on `E = x X + y Y + z Z + W`, in a monomial order.
///
/// # Arguments
///
/// * `basis` - X, Y, Z, W.
/// * `order` - The monomial order of the columns.
///
/// # Returns
///
/// The 10x20 coefficient matrix (det E first, then the trace constraint row-major).
fn constraints(basis: &[Matrix3<f64>; 4], order: &Order) -> Constraints {
    let mut e: CubicMatrix = [[[0.0; 64]; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let entry = &mut e[i][j];
            entry[at(1, 0, 0)] = basis[0][(i, j)];
            entry[at(0, 1, 0)] = basis[1][(i, j)];
            entry[at(0, 0, 1)] = basis[2][(i, j)];
            entry[at(0, 0, 0)] = basis[3][(i, j)];
        }
    }
    let mut eet: CubicMatrix = [[[0.0; 64]; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for (eik, ejk) in e[i].iter().zip(&e[j]) {
                accumulate(&mut eet[i][j], &multiply(eik, ejk), 1.0);
            }
        }
    }
    let mut trace = eet[0][0];
    accumulate(&mut trace, &eet[1][1], 1.0);
    accumulate(&mut trace, &eet[2][2], 1.0);
    let mut rows = [[0.0; 64]; 10];
    for k in 0..3 {
        let (k1, k2) = ((k + 1) % 3, (k + 2) % 3);
        let mut cofactor = multiply(&e[1][k1], &e[2][k2]);
        accumulate(&mut cofactor, &multiply(&e[1][k2], &e[2][k1]), -1.0);
        accumulate(&mut rows[0], &multiply(&e[0][k], &cofactor), 1.0);
    }
    for i in 0..3 {
        for j in 0..3 {
            let cubic = &mut rows[1 + 3 * i + j];
            accumulate(cubic, &multiply(&trace, &e[i][j]), -1.0);
            for k in 0..3 {
                accumulate(cubic, &multiply(&eet[i][k], &e[k][j]), 2.0);
            }
        }
    }
    Constraints::from_fn(|r, c| {
        let [a, b, cc] = order[c];
        rows[r][at(a, b, cc)]
    })
}

/// Reduces a 10x20 matrix to `[I | B]` by partial pivoting.
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// B, or `None` if the left block is singular.
fn gauss_jordan(mut matrix: Constraints) -> Option<SMatrix<f64, 10, 10>> {
    for col in 0..10 {
        // The first largest entry, as Eigen's maxCoeff picks it.
        let mut pivot = col;
        for row in col + 1..10 {
            if matrix[(row, col)].abs() > matrix[(pivot, col)].abs() {
                pivot = row;
            }
        }
        if !(matrix[(pivot, col)].abs() > 0.0) {
            return None;
        }
        matrix.swap_rows(col, pivot);
        let lead = matrix[(col, col)];
        let normalised = matrix.row(col) / lead;
        matrix.set_row(col, &normalised);
        for row in 0..10 {
            if row != col {
                let factor = matrix[(row, col)];
                let updated = matrix.row(row) - normalised * factor;
                matrix.set_row(row, &updated);
            }
        }
    }
    Some(matrix.fixed_columns::<10>(10).into_owned())
}

/// Composes E from its coefficients in the null-space basis, scales it to singular values
/// (1, 1, 0), and keeps it if it is essential and new.
///
/// # Arguments
///
/// * `solutions` - The solutions so far.
/// * `basis` - X, Y, Z, W.
/// * `xyz` - x, y, z.
fn keep(solutions: &mut Vec<Matrix3<f64>>, basis: &[Matrix3<f64>; 4], xyz: &Vector3<f64>) {
    let essential = basis[0] * xyz.x + basis[1] * xyz.y + basis[2] * xyz.z + basis[3];
    let scaled = essential * (SQRT_2 / essential.norm());
    let singular = scaled.singular_values();
    if !((singular[0] - singular[1]).max(singular[2]) <= ESSENTIAL_TOLERANCE) {
        return;
    }
    let duplicate = solutions
        .iter()
        .any(|kept| (kept - scaled).amax().min((kept + scaled).amax()) <= DUPLICATE_TOLERANCE);
    if !duplicate {
        solutions.push(scaled);
    }
}

/// Evaluates a polynomial (Horner).
///
/// # Arguments
///
/// * `p` - Coefficients, ascending.
/// * `z` - The point.
///
/// # Returns
///
/// p(z).
fn evaluate(p: &[f64], z: f64) -> f64 {
    p.iter().rev().fold(0.0, |value, c| value * z + c)
}

/// Drops trailing zero coefficients (keeping at least one).
///
/// # Arguments
///
/// * `p` - The polynomial.
///
/// # Returns
///
/// The trimmed polynomial.
fn trim(mut p: Vec<f64>) -> Vec<f64> {
    while p.len() > 1 && p.last() == Some(&0.0) {
        p.pop();
    }
    p
}

/// The product of two polynomials.
///
/// # Arguments
///
/// * `p`, `q` - The polynomials.
///
/// # Returns
///
/// p q.
fn product(p: &[f64], q: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; p.len() + q.len() - 1];
    for (i, pi) in p.iter().enumerate() {
        for (j, qj) in q.iter().enumerate() {
            out[i + j] += pi * qj;
        }
    }
    out
}

/// `p + scale * q`.
///
/// # Arguments
///
/// * `p`, `q` - The polynomials.
/// * `scale` - The factor.
///
/// # Returns
///
/// The sum.
fn sum(p: &[f64], q: &[f64], scale: f64) -> Vec<f64> {
    let mut out = vec![0.0; p.len().max(q.len())];
    for (o, pi) in out.iter_mut().zip(p) {
        *o += pi;
    }
    for (o, qi) in out.iter_mut().zip(q) {
        *o += scale * qi;
    }
    out
}

/// The remainder of polynomial division `numerator / denominator`.
///
/// # Arguments
///
/// * `numerator` - The numerator.
/// * `denominator` - The denominator (non-zero leading coefficient).
///
/// # Returns
///
/// The remainder.
fn remainder(mut numerator: Vec<f64>, denominator: &[f64]) -> Vec<f64> {
    let d = denominator.len() - 1;
    while numerator.len() > d && numerator.len() > 1 {
        let factor = numerator[numerator.len() - 1] / denominator[d];
        let shift = numerator.len() - 1 - d;
        for (i, di) in denominator.iter().enumerate() {
            numerator[shift + i] -= factor * di;
        }
        numerator.pop();
    }
    if numerator.is_empty() {
        vec![0.0]
    } else {
        numerator
    }
}

/// The derivative of a polynomial.
///
/// # Arguments
///
/// * `p` - The polynomial.
///
/// # Returns
///
/// p'.
#[allow(
    clippy::cast_precision_loss,
    reason = "a polynomial degree is far below 2^52"
)]
fn derivative(p: &[f64]) -> Vec<f64> {
    if p.len() < 2 {
        return vec![0.0];
    }
    p.iter()
        .enumerate()
        .skip(1)
        .map(|(i, c)| i as f64 * c)
        .collect()
}

/// The Sturm sequence of a polynomial.
///
/// # Arguments
///
/// * `p` - The polynomial.
///
/// # Returns
///
/// p, p', -rem(p, p'), ...
fn sturm_chain(p: &[f64]) -> Vec<Vec<f64>> {
    let mut chain = vec![p.to_vec(), derivative(p)];
    loop {
        let last = &chain[chain.len() - 1];
        if last.len() <= 1 && last[0] == 0.0 {
            break;
        }
        let next: Vec<f64> = remainder(chain[chain.len() - 2].clone(), last)
            .iter()
            .map(|c| -c)
            .collect();
        let next = trim(next);
        if next.len() == 1 && next[0] == 0.0 {
            break;
        }
        let constant = next.len() == 1;
        chain.push(next);
        if constant {
            break;
        }
    }
    chain
}

/// The sign of a number (0 for zero, and for NaN).
///
/// # Arguments
///
/// * `value` - The number.
///
/// # Returns
///
/// -1, 0 or 1.
fn sign_of(value: f64) -> i8 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

/// The sign changes of a Sturm sequence at a point.
///
/// # Arguments
///
/// * `chain` - The sequence.
/// * `z` - The point.
///
/// # Returns
///
/// The count.
fn sign_changes(chain: &[Vec<f64>], z: f64) -> i64 {
    let mut changes = 0;
    let mut last = 0;
    for p in chain {
        let sign = sign_of(evaluate(p, z));
        if sign == 0 {
            continue;
        }
        if last != 0 && sign != last {
            changes += 1;
        }
        last = sign;
    }
    changes
}

/// Refines a root bracketed by a sign change.
///
/// # Arguments
///
/// * `p` - The polynomial.
/// * `low`, `high` - The bracket.
///
/// # Returns
///
/// The root.
fn bisect(p: &[f64], mut low: f64, mut high: f64) -> f64 {
    let mut f_low = evaluate(p, low);
    while high - low > ROOT_TOLERANCE * 1.0_f64.max(low.abs()).max(high.abs()) {
        let middle = f64::midpoint(low, high);
        let f_middle = evaluate(p, middle);
        if f_middle == 0.0 {
            return middle;
        }
        if (f_middle < 0.0) == (f_low < 0.0) {
            low = middle;
            f_low = f_middle;
        } else {
            high = middle;
        }
    }
    f64::midpoint(low, high)
}

/// A polynomial's distinct real roots, ascending, by Sturm sequences and bisection.
///
/// # Arguments
///
/// * `poly` - Coefficients, ascending (`poly[k]` multiplies `z^k`).
///
/// # Returns
///
/// The real roots.
#[must_use]
pub fn real_roots(poly: Vec<f64>) -> Vec<f64> {
    let mut p = trim(poly);
    if p.len() < 2 {
        return Vec::new();
    }
    let lead = p[p.len() - 1];
    for c in &mut p {
        *c /= lead;
    }
    let bound = p[..p.len() - 1].iter().fold(0.0_f64, |m, c| m.max(c.abs())) + 1.0;
    let chain = sturm_chain(&p);
    let mut stack = vec![(
        -bound,
        bound,
        sign_changes(&chain, -bound),
        sign_changes(&chain, bound),
    )];
    let mut roots = Vec::new();
    while let Some((low, high, s_low, s_high)) = stack.pop() {
        let count = s_low - s_high;
        if count <= 0 {
            continue;
        }
        if count == 1 && sign_of(evaluate(&p, low)) != sign_of(evaluate(&p, high)) {
            roots.push(bisect(&p, low, high));
            continue;
        }
        if high - low < ROOT_TOLERANCE * 1.0_f64.max(low.abs()) {
            roots.push(f64::midpoint(low, high));
            continue;
        }
        let middle = f64::midpoint(low, high);
        let s_middle = sign_changes(&chain, middle);
        stack.push((middle, high, s_middle, s_high));
        stack.push((low, middle, s_low, s_middle));
    }
    roots.sort_by(f64::total_cmp);
    roots
}

/// A row's tail split into the polynomials in z multiplying x, y and 1.
///
/// # Arguments
///
/// * `tail` - Coefficients over `xz² xz x yz² yz y z³ z² z 1`.
///
/// # Returns
///
/// The three polynomials, ascending in z.
fn z_polynomials(tail: &[f64]) -> [Vec<f64>; 3] {
    [
        vec![tail[2], tail[1], tail[0]],
        vec![tail[5], tail[4], tail[3]],
        vec![tail[9], tail[8], tail[7], tail[6]],
    ]
}

/// `x^a`, with `x^0 = 1` for any x (as `pow` does).
///
/// # Arguments
///
/// * `x` - The base.
/// * `a` - The exponent, at most three.
///
/// # Returns
///
/// The power.
fn power(x: f64, a: usize) -> f64 {
    (0..a).fold(1.0, |acc, _| acc * x)
}

/// The monomials of an order evaluated at (x, y, z), and their derivatives.
///
/// # Arguments
///
/// * `order` - The monomial order.
/// * `xyz` - The point.
///
/// # Returns
///
/// The 20 values, and their 20x3 gradient.
fn monomials(order: &Order, xyz: &Vector3<f64>) -> (SMatrix<f64, 20, 1>, SMatrix<f64, 20, 3>) {
    let value = |e: &Monomial| power(xyz.x, e[0]) * power(xyz.y, e[1]) * power(xyz.z, e[2]);
    let mut values = SMatrix::<f64, 20, 1>::zeros();
    let mut slope = SMatrix::<f64, 20, 3>::zeros();
    for (k, e) in order.iter().enumerate() {
        values[k] = value(e);
        for v in 0..3 {
            if e[v] == 0 {
                continue;
            }
            let mut lowered = *e;
            lowered[v] -= 1;
            #[allow(clippy::cast_precision_loss, reason = "an exponent is at most three")]
            let exponent = e[v] as f64;
            slope[(k, v)] = exponent * value(&lowered);
        }
    }
    (values, slope)
}

/// The least-squares step `argmin |J δ + r|` by SVD, singular values below Eigen's default
/// relative threshold treated as zero (as `JacobiSVD::solve` does).
///
/// # Arguments
///
/// * `jacobian` - J, 10x3.
/// * `residual` - r.
///
/// # Returns
///
/// δ, or `None` if the SVD fails.
fn least_squares_step(
    jacobian: &SMatrix<f64, 10, 3>,
    residual: &SMatrix<f64, 10, 1>,
) -> Option<Vector3<f64>> {
    let step = least_squares(
        DMatrix::from_column_slice(10, 3, jacobian.as_slice()),
        &DVector::from_column_slice((-residual).as_slice()),
        3.0 * f64::EPSILON,
    )?;
    Some(Vector3::new(step[0], step[1], step[2]))
}

/// Refines (x, y, z) by Gauss-Newton on the ten cubic constraints, to convergence.
///
/// # Arguments
///
/// * `m` - The 10x20 constraint matrix.
/// * `order` - Its monomial order.
/// * `start` - The starting point.
///
/// # Returns
///
/// The polished point.
fn polish(m: &Constraints, order: &Order, start: Vector3<f64>) -> Vector3<f64> {
    let mut xyz = start;
    let (mut values, mut slope) = monomials(order, &xyz);
    let mut residual = m * values;
    for _ in 0..POLISH_MAX_STEPS {
        let jacobian = m * slope;
        let Some(delta) = least_squares_step(&jacobian, &residual) else {
            break;
        };
        if !delta.iter().all(|d| d.is_finite()) {
            break;
        }
        let moved = xyz + delta;
        let (moved_values, moved_slope) = monomials(order, &moved);
        let moved_residual = m * moved_values;
        if !(moved_residual.norm() < residual.norm()) {
            break;
        }
        xyz = moved;
        values = moved_values;
        slope = moved_slope;
        residual = moved_residual;
        if delta.norm() <= POLISH_TOLERANCE * xyz.norm().max(1.0) {
            break;
        }
    }
    let _ = values;
    xyz
}

/// The system both methods start from.
struct System {
    /// X, Y, Z, W: E's null-space basis.
    basis: [Matrix3<f64>; 4],
    /// The ten cubic constraints over the twenty monomials.
    constraints: Constraints,
    /// B, the constraints' Gauss-Jordan reduction.
    reduced: SMatrix<f64, 10, 10>,
}

/// The null space, its constraints in a monomial order, and their reduction.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - The pairs.
/// * `order` - The monomial order.
///
/// # Returns
///
/// The system, or `None` for a degenerate sample.
fn set_up(bearings1: &[Vector3<f64>], bearings2: &[Vector3<f64>], order: &Order) -> Option<System> {
    let basis = null_space(bearings1, bearings2)?;
    let constraints = constraints(&basis, order);
    let reduced = gauss_jordan(constraints)?;
    Some(System {
        basis,
        constraints,
        reduced,
    })
}

/// Nistér's method.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - The pairs.
///
/// # Returns
///
/// The real solutions.
fn nister(bearings1: &[Vector3<f64>], bearings2: &[Vector3<f64>]) -> Vec<Matrix3<f64>> {
    let Some(system) = set_up(bearings1, bearings2, &NISTER_ORDER) else {
        return Vec::new();
    };
    let reduced = &system.reduced;
    let row = |r: usize| -> Vec<f64> { reduced.row(r).iter().copied().collect() };
    // Each pair's first row minus z times its second: three rows of polynomials in z.
    let rows: [[Vec<f64>; 3]; 3] = std::array::from_fn(|r| {
        let upper = NISTER_ROW_PAIRS[r];
        let a = z_polynomials(&row(upper));
        let b = z_polynomials(&row(upper + 1));
        std::array::from_fn(|c| {
            let mut shifted = vec![0.0];
            shifted.extend(&b[c]);
            sum(&a[c], &shifted, -1.0)
        })
    });
    let mut determinant = vec![0.0];
    for (i, j, k, sign) in DETERMINANT_TERMS {
        let term = product(&product(&rows[0][i], &rows[1][j]), &rows[2][k]);
        determinant = sum(&determinant, &term, sign);
    }
    let mut solutions = Vec::new();
    for root in real_roots(determinant) {
        let b = Matrix3::from_fn(|r, c| evaluate(&rows[r][c], root));
        let v = b.row(0).transpose().cross(&b.row(1).transpose());
        if !(v.z.abs() > 0.0) {
            continue;
        }
        let xyz = polish(
            &system.constraints,
            &NISTER_ORDER,
            Vector3::new(v.x / v.z, v.y / v.z, root),
        );
        keep(&mut solutions, &system.basis, &xyz);
    }
    solutions
}

/// A real eigenvector of `matrix` for a real eigenvalue: the null vector of `A - λ I`.
///
/// # Arguments
///
/// * `matrix` - A.
/// * `value` - λ.
///
/// # Returns
///
/// The unit eigenvector, or `None` if the SVD fails.
fn eigenvector(matrix: &SMatrix<f64, 10, 10>, value: f64) -> Option<DVector<f64>> {
    let shifted = matrix - SMatrix::<f64, 10, 10>::identity() * value;
    let decomposition = svd(
        DMatrix::from_column_slice(10, 10, shifted.as_slice()),
        false,
        true,
    )?;
    Some(decomposition.v_t?.row(9).transpose())
}

/// Stewénius' action-matrix method.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - The pairs.
///
/// # Returns
///
/// The real solutions.
fn stewenius(bearings1: &[Vector3<f64>], bearings2: &[Vector3<f64>]) -> Vec<Matrix3<f64>> {
    let Some(system) = set_up(bearings1, bearings2, &GREVLEX_ORDER) else {
        return Vec::new();
    };
    // x b for b = (x² xy xz y² yz z² x y z 1): six cubics from B, four shifts within b.
    let mut action = SMatrix::<f64, 10, 10>::zeros();
    action
        .fixed_rows_mut::<6>(0)
        .copy_from(&-system.reduced.fixed_rows::<6>(0));
    for (r, index) in [(6, 0), (7, 1), (8, 2), (9, 6)] {
        action[(r, index)] = 1.0;
    }
    let Some(schur) = Schur::try_new(action, f64::EPSILON, SVD_MAX_ITERATIONS) else {
        return Vec::new();
    };
    let mut solutions = Vec::new();
    for value in schur.complex_eigenvalues().iter() {
        if value.im.abs() > REAL_TOLERANCE * value.norm().max(1.0) {
            continue;
        }
        let Some(vector) = eigenvector(&action, value.re) else {
            continue;
        };
        if !(vector[9].abs() > 0.0) {
            continue;
        }
        let xyz = polish(
            &system.constraints,
            &GREVLEX_ORDER,
            Vector3::new(
                vector[6] / vector[9],
                vector[7] / vector[9],
                vector[8] / vector[9],
            ),
        );
        keep(&mut solutions, &system.basis, &xyz);
    }
    solutions
}

/// Every essential matrix five bearing pairs allow (unchecked).
///
/// # Arguments
///
/// * `bearings1` - Five unit bearings in camera 1.
/// * `bearings2` - Five unit bearings in camera 2.
/// * `solver` - The solver.
///
/// # Returns
///
/// The real solutions (none for a degenerate sample).
#[must_use]
pub fn solve(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    solver: Solver,
) -> Vec<Matrix3<f64>> {
    match solver {
        Solver::Nister => nister(bearings1, bearings2),
        Solver::Stewenius => stewenius(bearings1, bearings2),
    }
}

/// Every essential matrix five bearing pairs allow.
///
/// # Arguments
///
/// * `bearings1` - Five unit bearings in camera 1.
/// * `bearings2` - Five unit bearings in camera 2.
/// * `solver` - The solver.
///
/// # Returns
///
/// The solutions.
///
/// # Errors
///
/// `InvalidInput` unless there are five unit pairs.
pub fn essential_5pt(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    solver: Solver,
) -> Result<Vec<Matrix3<f64>>> {
    check_bearing_pairs(bearings1, bearings2, SAMPLE_SIZE, false)?;
    if bearings1.len() != SAMPLE_SIZE {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "bearings: expected exactly 5 pairs, got {}",
                bearings1.len()
            ),
        );
    }
    Ok(solve(bearings1, bearings2, solver))
}

#[cfg(test)]
mod tests {
    //! Native tests: both solvers find the true essential matrix among their solutions, and
    //! the real roots of a known polynomial.

    use nalgebra::Vector6;

    use super::*;
    use crate::{se3, so3};

    /// Five bearing pairs through a known relative pose, and its essential matrix.
    fn sample() -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>, Matrix3<f64>) {
        let pose = se3::exp(&Vector6::new(0.8, -0.1, 0.2, 0.05, -0.12, 0.03));
        let points: Vec<Vector3<f64>> = (0..5)
            .map(|i| {
                let t = f64::from(i);
                Vector3::new((1.7 * t).sin(), (0.9 * t).cos(), 4.0 + 0.5 * t)
            })
            .collect();
        let b1 = points.iter().map(Vector3::normalize).collect();
        let b2 = points
            .iter()
            .map(|p| se3::act(&pose, p).unwrap().normalize())
            .collect();
        let essential = so3::hat(&se3::translation(&pose)) * se3::rotation(&pose);
        (b1, b2, essential * (SQRT_2 / essential.norm()))
    }

    /// Both solvers return the true E (up to sign) among their solutions.
    #[test]
    fn both_solvers_find_the_true_essential_matrix() {
        let (b1, b2, truth) = sample();
        for solver in Solver::ALL {
            let found = essential_5pt(&b1, &b2, solver).unwrap();
            assert!(!found.is_empty() && found.len() <= 10, "{solver:?}");
            let best = found
                .iter()
                .map(|e| (e - truth).amax().min((e + truth).amax()))
                .fold(f64::INFINITY, f64::min);
            assert!(best < 1e-9, "{solver:?}: {best}");
        }
    }

    /// The real roots of (z - 1)(z + 2)(z² + 1), ascending.
    #[test]
    fn real_roots_of_a_known_polynomial() {
        // (z - 1)(z + 2) = z² + z - 2; times (z² + 1): z⁴ + z³ - z² + z - 2.
        let roots = real_roots(vec![-2.0, 1.0, -1.0, 1.0, 1.0]);
        assert_eq!(roots.len(), 2);
        assert!((roots[0] + 2.0).abs() < 1e-12 && (roots[1] - 1.0).abs() < 1e-12);
    }
}
