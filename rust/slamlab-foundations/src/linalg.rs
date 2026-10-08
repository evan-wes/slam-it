//! Dense linear algebra shared by rung 0's Rust modules: the SVD every module uses, and the
//! minimum-norm least squares built on it.
//!
//! Every SVD in the crate goes through [`svd`], which calls faer, or, for a 3x3, [`svd3`] and
//! [`singular_values3`], which run Eigen's `JacobiSVD` algorithm on the stack; faer also does
//! the one general eigendecomposition ([`eigen_real`], Stewénius' action matrix). nalgebra
//! 0.35's SVDs are not backward stable: its 3x3 SVD works on `MᵀM`, squaring the condition
//! number (thin P3P triangles gave rotations off by 0.7), and its dynamic SVD leaves errors
//! near 1e-4 in some graded matrices' trailing singular pairs (8-point E and F off by up to
//! 6e-3). faer's matches LAPACK to roundoff on both (PR 6 panel, L1 and L2), once the matrix is
//! scaled into a safe range as LAPACK's `dlascl` does (faer does not; PR 6 panel, run 2, L8).
//! faer allocates, which the per-frame 3x3s of the edge VO (`so3::project_to_so3`, Umeyama)
//! should not (PR 6 panel, A11): two-sided Jacobi is backward stable and needs no workspace,
//! and is what C++'s `Eigen::JacobiSVD<Matrix3d>` computes. nalgebra stays the crate's matrix
//! type; faer's factors are copied across at each call. Only the crate uses this module.
//! There is no Python or C++ counterpart: numpy (LAPACK) and Eigen (`JacobiSVD`) need none.

use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::evd::{self as faer_evd, ComputeEigenvectors};
use faer::linalg::svd::{self as faer_svd, ComputeSvdVectors};
use faer::{Mat, Par};
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

/// The largest absolute entries (2^-400, 2^400) [`svd`] decomposes without scaling: well inside
/// the range where faer's intermediate squares neither underflow nor overflow (about 2^±511).
const SAFE_RANGE: (f64, f64) = (3.872_591_914_849_318_3e-121, 2.582_249_878_086_908_6e120);

/// A matrix's thin SVD: `M = U diag(σ) Vᵀ` with k = min(m, n) singular triplets, as
/// nalgebra's and LAPACK's thin SVDs return them. A caller that needs all n right singular
/// vectors pads the matrix to at least n rows first (which changes neither its row space nor
/// its right singular vectors).
#[derive(Debug, Clone)]
pub struct Svd {
    /// U (m x k), if asked for.
    pub u: Option<DMatrix<f64>>,
    /// The singular values, largest first (k of them).
    pub singular_values: DVector<f64>,
    /// Vᵀ (k x n), if asked for.
    pub v_t: Option<DMatrix<f64>>,
}

/// A dynamic matrix's SVD, singular values largest first, or `None` for a matrix with a
/// non-finite entry or one faer cannot decompose.
///
/// # Arguments
///
/// * `matrix` - The matrix (not empty).
/// * `compute_u` - Whether to return `U`.
/// * `compute_v` - Whether to return `Vᵀ`.
///
/// # Returns
///
/// The thin decomposition.
#[must_use]
pub fn svd(matrix: &DMatrix<f64>, compute_u: bool, compute_v: bool) -> Option<Svd> {
    if !matrix.iter().all(|x| x.is_finite()) {
        return None;
    }
    let (m, n) = matrix.shape();
    let k = m.min(n);
    // Outside a safe range, scale the largest entry to about 1 by a power of two (exact), as
    // LAPACK's dlascl does: faer works on the matrix as given, and its squares underflow below
    // about 1e-154 (silently wrong factors) and overflow above about 1e154. Inside the range
    // (every matrix real data produces) nothing is scaled. U and V are unchanged by scaling;
    // σ scales back. The factor is applied in two halves so that neither overflows.
    let largest = matrix.amax();
    #[allow(
        clippy::cast_possible_truncation,
        reason = "a finite f64's binary exponent is within ±1100"
    )]
    let exponent = if largest > 0.0 && !(SAFE_RANGE.0..=SAFE_RANGE.1).contains(&largest) {
        -(largest.log2().round() as i32)
    } else {
        0
    };
    let (half, rest) = (
        f64::from(exponent / 2).exp2(),
        f64::from(exponent - exponent / 2).exp2(),
    );
    let a = Mat::<f64>::from_fn(m, n, |i, j| matrix[(i, j)] * half * rest);
    let vectors = |wanted: bool| {
        if wanted {
            ComputeSvdVectors::Thin
        } else {
            ComputeSvdVectors::No
        }
    };
    // faer's low-level SVD, sequential, with its workspace sized for this call. Its
    // convenience `thin_svd` took 18 times as long on a tall LO refit (350 x 9).
    let mut workspace = MemBuffer::new(faer_svd::svd_scratch::<f64>(
        m,
        n,
        vectors(compute_u),
        vectors(compute_v),
        Par::Seq,
        faer::Spec::default(),
    ));
    let mut s = faer::diag::Diag::<f64>::zeros(k);
    let mut u = compute_u.then(|| Mat::<f64>::zeros(m, k));
    let mut v = compute_v.then(|| Mat::<f64>::zeros(n, k));
    faer_svd::svd(
        a.as_ref(),
        s.as_mut(),
        u.as_mut().map(Mat::as_mut),
        v.as_mut().map(Mat::as_mut),
        Par::Seq,
        MemStack::new(&mut workspace),
        faer::Spec::default(),
    )
    .ok()?;
    Some(Svd {
        u: u.map(|u| DMatrix::from_fn(m, k, |i, j| u[(i, j)])),
        singular_values: DVector::from_fn(k, |i, _| s[i] / half / rest),
        v_t: v.map(|v| DMatrix::from_fn(k, n, |i, j| v[(j, i)])),
    })
}

/// A real square matrix's eigenvalues and right eigenvectors, as LAPACK's `dgeev` returns
/// them.
///
/// # Arguments
///
/// * `matrix` - The square matrix.
///
/// # Returns
///
/// The eigenvalues' real and imaginary parts, and the eigenvectors as columns: for a real
/// eigenvalue, column j is its eigenvector; a complex pair's columns hold the real and
/// imaginary parts of the first one's. `None` for a non-finite matrix. faer does not report a
/// Schur iteration that fails to converge (numpy raises there); on the small matrices the
/// crate decomposes this is not observed, and the callers filter what they find.
#[must_use]
pub fn eigen_real(matrix: &DMatrix<f64>) -> Option<(DVector<f64>, DVector<f64>, DMatrix<f64>)> {
    if !matrix.is_square() || !matrix.iter().all(|x| x.is_finite()) {
        return None;
    }
    let n = matrix.nrows();
    let a = Mat::<f64>::from_fn(n, n, |i, j| matrix[(i, j)]);
    let mut workspace = MemBuffer::new(faer_evd::evd_scratch::<f64>(
        n,
        ComputeEigenvectors::No,
        ComputeEigenvectors::Yes,
        Par::Seq,
        faer::Spec::default(),
    ));
    let mut re = faer::diag::Diag::<f64>::zeros(n);
    let mut im = faer::diag::Diag::<f64>::zeros(n);
    let mut vectors = Mat::<f64>::zeros(n, n);
    faer_evd::evd_real(
        a.as_ref(),
        re.as_mut(),
        im.as_mut(),
        None,
        Some(vectors.as_mut()),
        Par::Seq,
        MemStack::new(&mut workspace),
        faer::Spec::default(),
    )
    .ok()?;
    Some((
        DVector::from_fn(n, |i, _| re[i]),
        DVector::from_fn(n, |i, _| im[i]),
        DMatrix::from_fn(n, n, |i, j| vectors[(i, j)]),
    ))
}

/// The most sweeps [`svd3`] makes before it gives up. Eigen's `JacobiSVD` has no cap; cyclic
/// Jacobi converges quadratically, in about six sweeps on the hardest 3x3s, so a cap this
/// generous never changes a result where Eigen ends, and turns a non-termination into `None`.
const MAX_SWEEPS: usize = 64;

/// A plane rotation `[c s; -s c]`, as Eigen's real `JacobiRotation`.
#[derive(Debug, Clone, Copy)]
struct Rotation {
    /// The cosine.
    c: f64,
    /// The sine.
    s: f64,
}

impl Rotation {
    /// No rotation.
    const IDENTITY: Self = Self { c: 1.0, s: 0.0 };

    /// The transposed rotation.
    ///
    /// # Returns
    ///
    /// `[c -s; s c]`.
    fn transpose(self) -> Self {
        Self {
            c: self.c,
            s: -self.s,
        }
    }

    /// The product of two rotations.
    ///
    /// # Arguments
    ///
    /// * `other` - The rotation on the right.
    ///
    /// # Returns
    ///
    /// `self * other`.
    fn times(self, other: Self) -> Self {
        Self {
            c: self.c * other.c - self.s * other.s,
            s: self.c * other.s + self.s * other.c,
        }
    }

    /// The rotation that diagonalises the symmetric `[x y; y z]` from both sides, as Eigen's
    /// `JacobiRotation::makeJacobi`.
    ///
    /// # Arguments
    ///
    /// * `x` - The upper-left entry.
    /// * `y` - The off-diagonal entry.
    /// * `z` - The lower-right entry.
    ///
    /// # Returns
    ///
    /// The rotation (none when `y` is zero or subnormal).
    fn jacobi(x: f64, y: f64, z: f64) -> Self {
        let deno = 2.0 * y.abs();
        if deno < f64::MIN_POSITIVE {
            return Self::IDENTITY;
        }
        let tau = (x - z) / deno;
        let w = (tau * tau + 1.0).sqrt();
        let t = if tau > 0.0 {
            1.0 / (tau + w)
        } else {
            1.0 / (tau - w)
        };
        let sign_t = if t > 0.0 { 1.0 } else { -1.0 };
        let n = 1.0 / (t * t + 1.0).sqrt();
        Self {
            c: n,
            s: -sign_t * (y / y.abs()) * t.abs() * n,
        }
    }

    /// Whether this is no rotation (applying it is then skipped, as Eigen skips it).
    ///
    /// # Returns
    ///
    /// Whether `c` is 1 and `s` is 0.
    #[allow(clippy::float_cmp, reason = "exactly no rotation, as Eigen tests it")]
    fn is_identity(self) -> bool {
        self.c == 1.0 && self.s == 0.0
    }

    /// Applies the rotation to rows `p` and `q` from the left: `B = J B`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The matrix.
    /// * `p` - The first row.
    /// * `q` - The second row.
    fn rotate_rows(self, matrix: &mut Matrix3<f64>, p: usize, q: usize) {
        if self.is_identity() {
            return;
        }
        for k in 0..3 {
            let (x, y) = (matrix[(p, k)], matrix[(q, k)]);
            matrix[(p, k)] = self.c * x + self.s * y;
            matrix[(q, k)] = -self.s * x + self.c * y;
        }
    }

    /// Applies the rotation to columns `p` and `q` from the right: `B = B J`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The matrix.
    /// * `p` - The first column.
    /// * `q` - The second column.
    fn rotate_columns(self, matrix: &mut Matrix3<f64>, p: usize, q: usize) {
        let t = self.transpose();
        if t.is_identity() {
            return;
        }
        for k in 0..3 {
            let (x, y) = (matrix[(k, p)], matrix[(k, q)]);
            matrix[(k, p)] = t.c * x + t.s * y;
            matrix[(k, q)] = -t.s * x + t.c * y;
        }
    }
}

/// The SVD of a 2x2 block `(p, q)` of a matrix, as Eigen's `real_2x2_jacobi_svd`: a rotation
/// symmetrises the block, then a Jacobi rotation diagonalises it.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `p` - The block's second index.
/// * `q` - Its first.
///
/// # Returns
///
/// The left and right rotations `(J_l, J_r)`: `J_l B J_r` is diagonal.
fn jacobi_2x2(matrix: &Matrix3<f64>, p: usize, q: usize) -> (Rotation, Rotation) {
    let (a, b) = (matrix[(p, p)], matrix[(p, q)]);
    let (c, d) = (matrix[(q, p)], matrix[(q, q)]);
    let t = a + d;
    let skew = c - b;
    let symmetrise = if skew.abs() < f64::MIN_POSITIVE {
        Rotation::IDENTITY
    } else {
        let u = t / skew;
        let tmp = (1.0 + u * u).sqrt();
        Rotation {
            c: u / tmp,
            s: 1.0 / tmp,
        }
    };
    // The block's rows rotated: `[x y; y z]`, symmetric.
    let (x, y, z) = if symmetrise.is_identity() {
        (a, b, d)
    } else {
        (
            symmetrise.c * a + symmetrise.s * c,
            symmetrise.c * b + symmetrise.s * d,
            -symmetrise.s * b + symmetrise.c * d,
        )
    };
    let right = Rotation::jacobi(x, y, z);
    (symmetrise.times(right.transpose()), right)
}

/// A 3x3 matrix's SVD by two-sided Jacobi, Eigen's `JacobiSVD` for a square matrix: the
/// matrix scaled by its largest entry, sweeps of 2x2 SVDs over each off-diagonal pair until
/// every pair is below `2ε` times the largest diagonal entry, the diagonal made positive (its
/// signs moved into `U`), then sorted.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `vectors` - Whether to accumulate `U` and `V` (identity otherwise).
///
/// # Returns
///
/// `(U, σ, V)`, or `None` for a non-finite matrix (or one not diagonalised within
/// [`MAX_SWEEPS`], never observed).
fn jacobi_svd3(
    matrix: &Matrix3<f64>,
    vectors: bool,
) -> Option<(Matrix3<f64>, Vector3<f64>, Matrix3<f64>)> {
    if !matrix.iter().all(|x| x.is_finite()) {
        return None;
    }
    let largest = matrix.amax();
    let scale = if largest == 0.0 { 1.0 } else { largest };
    let mut work = matrix / scale;
    let mut u = Matrix3::identity();
    let mut v = Matrix3::identity();
    let precision = 2.0 * f64::EPSILON;
    let mut max_diagonal = work.diagonal().amax();
    let mut finished = false;
    let mut sweeps = 0;
    while !finished {
        if sweeps == MAX_SWEEPS {
            return None;
        }
        sweeps += 1;
        finished = true;
        for p in 1..3 {
            for q in 0..p {
                // Subnormal entries count as zero, so the sweeps end.
                let threshold = f64::MIN_POSITIVE.max(precision * max_diagonal);
                if work[(p, q)].abs() > threshold || work[(q, p)].abs() > threshold {
                    finished = false;
                    let (left, right) = jacobi_2x2(&work, p, q);
                    left.rotate_rows(&mut work, p, q);
                    right.rotate_columns(&mut work, p, q);
                    if vectors {
                        left.transpose().rotate_columns(&mut u, p, q);
                        right.rotate_columns(&mut v, p, q);
                    }
                    max_diagonal = max_diagonal.max(work[(p, p)].abs().max(work[(q, q)].abs()));
                }
            }
        }
    }
    let mut singular = Vector3::zeros();
    for i in 0..3 {
        let a = work[(i, i)];
        singular[i] = a.abs();
        if vectors && a < 0.0 {
            u.set_column(i, &-u.column(i));
        }
    }
    singular *= scale;
    // Largest first; the first of equal values stays first, as Eigen's `maxCoeff`.
    for i in 0..3 {
        let mut pos = i;
        for k in i + 1..3 {
            if singular[k] > singular[pos] {
                pos = k;
            }
        }
        if singular[pos] == 0.0 {
            break;
        }
        if pos != i {
            singular.swap_rows(i, pos);
            u.swap_columns(i, pos);
            v.swap_columns(i, pos);
        }
    }
    Some((u, singular, v))
}

/// A 3x3 matrix's SVD, singular values largest first: two-sided Jacobi, on the stack, as
/// C++'s `Eigen::JacobiSVD<Matrix3d>`.
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// `(U, σ, Vᵀ)`, or `None` for a non-finite matrix.
#[must_use]
pub fn svd3(matrix: &Matrix3<f64>) -> Option<(Matrix3<f64>, Vector3<f64>, Matrix3<f64>)> {
    let (u, singular, v) = jacobi_svd3(matrix, true)?;
    Some((u, singular, v.transpose()))
}

/// A 3x3 matrix's singular values, largest first (as [`svd3`]).
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// σ, or NaN for a non-finite matrix (which every rank test, written so NaN fails it, then
/// rejects).
#[must_use]
pub fn singular_values3(matrix: &Matrix3<f64>) -> Vector3<f64> {
    jacobi_svd3(matrix, false).map_or_else(|| Vector3::repeat(f64::NAN), |(_, s, _)| s)
}

/// The minimum-norm least-squares solution of `A x = b` by SVD, singular values at or below
/// `relative` times the largest treated as zero.
///
/// `relative` picks whose convention the caller follows: `max(m, n) ε` is numpy's `lstsq`
/// default, `min(m, n) ε` Eigen's `JacobiSVD::solve`.
///
/// # Arguments
///
/// * `matrix` - A (not empty).
/// * `rhs` - b.
/// * `relative` - The cutoff, relative to the largest singular value.
///
/// # Returns
///
/// x, or `None` if the SVD fails (a non-finite entry).
#[must_use]
pub fn least_squares(
    matrix: &DMatrix<f64>,
    rhs: &DVector<f64>,
    relative: f64,
) -> Option<DVector<f64>> {
    let decomposition = svd(matrix, true, true)?;
    let (u, v_t) = (decomposition.u?, decomposition.v_t?);
    let singular = &decomposition.singular_values;
    let threshold = relative * singular.max();
    let mut x = DVector::zeros(v_t.ncols());
    for (k, &sigma) in singular.iter().enumerate() {
        if sigma > threshold {
            x += v_t.row(k).transpose() * (u.column(k).dot(rhs) / sigma);
        }
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    //! Native tests of the SVD: factors, failure, and the graded and thin cases nalgebra's
    //! SVDs got wrong (PR 6 panel, L1 and L2).

    use nalgebra::Rotation3;

    use super::*;

    /// A test generator: the probe's LCG, uniform in [-1, 1).
    fn lcg(seed: u64) -> impl FnMut() -> f64 {
        let mut state = seed;
        move || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            #[allow(clippy::cast_precision_loss, reason = "53 bits of a 64-bit draw")]
            let unit = (state >> 11) as f64 / (1_u64 << 53) as f64;
            unit * 2.0 - 1.0
        }
    }

    /// The thin factors are orthonormal and multiply back to the matrix; a non-finite matrix
    /// fails at once.
    #[test]
    fn factors_reconstruct_and_non_finite_fails() {
        let m = DMatrix::from_row_slice(3, 2, &[1.0, 2.0, 3.0, 4.0, 5.0, 7.0]);
        let d = svd(&m, true, true).unwrap();
        let (u, v_t) = (d.u.unwrap(), d.v_t.unwrap());
        assert_eq!((u.shape(), v_t.shape()), ((3, 2), (2, 2)));
        let sigma = DMatrix::from_diagonal(&d.singular_values);
        assert!((&u * sigma * &v_t - &m).amax() < 1e-14);
        assert!((u.transpose() * &u - DMatrix::identity(2, 2)).amax() < 1e-14);
        let mut bad = m;
        bad[(1, 1)] = f64::NAN;
        assert!(svd(&bad, true, true).is_none());
        assert!(
            singular_values3(&Matrix3::repeat(f64::INFINITY))
                .iter()
                .all(|x| x.is_nan())
        );
    }

    /// Graded matrices (σ from 1 down to 1e-8) decompose to roundoff, where nalgebra 0.35's
    /// dynamic SVD left errors up to 3e-4 in about 1% of them.
    #[test]
    fn graded_matrices_decompose_to_roundoff() {
        let mut rnd = lcg(5);
        for (rows, cols) in [(3, 3), (10, 4), (9, 9), (12, 9)] {
            for _ in 0..300 {
                let qa = DMatrix::from_fn(rows, rows, |_, _| rnd()).qr().q();
                let qb = DMatrix::from_fn(cols, cols, |_, _| rnd()).qr().q();
                let mut sigma = DMatrix::zeros(rows, cols);
                for i in 0..rows.min(cols) {
                    sigma[(i, i)] = if i == 0 {
                        1.0
                    } else {
                        10_f64.powf(-4.0 * (rnd() + 1.0))
                    };
                }
                let m = &qa * &sigma * qb.transpose();
                let d = svd(&m, true, true).unwrap();
                let s = DMatrix::from_diagonal(&d.singular_values);
                assert!((d.u.unwrap() * s * d.v_t.unwrap() - m).amax() < 1e-13);
            }
        }
    }

    /// Scaled far from 1, a matrix keeps its singular values (relative) and vectors, where an
    /// unscaled faer SVD underflows below about 1e-154 and overflows above 1e154 (L8).
    #[test]
    fn extreme_scales_decompose_exactly() {
        #[rustfmt::skip] // keep one matrix row per line
        let base = DMatrix::from_row_slice(3, 3, &[
            4.0, 1.0, 0.0,
            1.0, 3.0, 1.0,
            0.0, 1.0, 2.0,
        ]);
        let reference = svd(&base, true, true).unwrap();
        for scale in [1e-300, 1e-200, 1e-160, 1e160, 1e200, 1e300] {
            let d = svd(&(&base * scale), true, true).unwrap();
            let relative = (&d.singular_values / scale - &reference.singular_values).amax();
            assert!(relative < 1e-14, "scale {scale:e}");
            let (u, v_t) = (d.u.unwrap(), d.v_t.unwrap());
            let rebuilt = &u * DMatrix::from_diagonal(&reference.singular_values) * &v_t;
            assert!((rebuilt - &base).amax() < 1e-14, "scale {scale:e}");
        }
        let zero = svd(&DMatrix::zeros(3, 3), true, true).unwrap();
        assert!(zero.singular_values.iter().all(|&s| s == 0.0));
    }

    /// The safe range's bounds are exactly 2^-400 and 2^400.
    #[test]
    fn safe_range_is_powers_of_two() {
        #[allow(clippy::float_cmp, reason = "exact powers of two")]
        let exact = SAFE_RANGE == ((-400.0_f64).exp2(), 400.0_f64.exp2());
        assert!(exact);
    }

    /// A rank-2 3x3 with σ = (1, 1e-8, 0) keeps σ₂ to 1e-6 relative, where nalgebra 0.35's
    /// 3x3 SVD (through `MᵀM`) lost up to 30%; the factors rebuild it to a few ulp (Jacobi's
    /// accumulated rotations, as Eigen's).
    #[test]
    fn small_singular_values_of_a_3x3_are_accurate() {
        let mut rnd = lcg(11);
        for _ in 0..200 {
            let a = Rotation3::new(Vector3::new(rnd(), rnd(), rnd()) * 2.0).into_inner();
            let b = Rotation3::new(Vector3::new(rnd(), rnd(), rnd()) * 2.0).into_inner();
            let m = a * Matrix3::from_diagonal(&Vector3::new(1.0, 1e-8, 0.0)) * b;
            assert!((singular_values3(&m)[1] / 1e-8 - 1.0).abs() < 1e-6);
            let (u, s, v_t) = svd3(&m).unwrap();
            assert!((u * Matrix3::from_diagonal(&s) * v_t - m).amax() < 1e-14);
            assert!((u.transpose() * u - Matrix3::identity()).amax() < 1e-14);
        }
    }

    /// The 3x3 SVD gives Eigen's factors bit for bit, on the golden fixture C++'s
    /// `foundations_test_svd3` writes from `Eigen::JacobiSVD` and checks Eigen against
    /// (random, graded, rank-deficient, repeated and near-repeated singular values, signs, a
    /// permutation, extreme scales).
    #[test]
    fn svd3_gives_eigens_factors_bit_for_bit() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/svd3.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        let fixture: serde_json::Value = serde_json::from_str(&text).expect("valid JSON");
        let numbers = |value: &serde_json::Value| -> Vec<f64> {
            value
                .as_array()
                .expect("a list")
                .iter()
                .map(|x| x.as_f64().expect("a number"))
                .collect()
        };
        let cases = fixture["cases"].as_array().expect("cases");
        assert!(cases.len() >= 20);
        for case in cases {
            let name = case["name"].as_str().expect("a name");
            let m = Matrix3::from_row_slice(&numbers(&case["matrix"]));
            let (u, s, v_t) = svd3(&m).expect("a finite matrix");
            assert_eq!(
                u,
                Matrix3::from_row_slice(&numbers(&case["u"])),
                "{name}: U"
            );
            let singular = Vector3::from_column_slice(&numbers(&case["singular_values"]));
            assert_eq!(s, singular, "{name}: σ");
            assert_eq!(singular_values3(&m), singular, "{name}: σ alone");
            let v = Matrix3::from_row_slice(&numbers(&case["v"]));
            assert_eq!(v_t, v.transpose(), "{name}: V");
        }
    }

    /// The 3x3 SVD is Eigen's `JacobiSVD`: values largest first and non-negative, a negative
    /// diagonal's sign moved into `U`, equal values left in place, and no rotation at all for a
    /// matrix already diagonal.
    #[test]
    fn svd3_orders_and_signs_as_eigen() {
        let (u, s, v_t) = svd3(&Matrix3::from_diagonal(&Vector3::new(1.0, -3.0, 2.0))).unwrap();
        assert_eq!(s, Vector3::new(3.0, 2.0, 1.0));
        #[rustfmt::skip] // keep one matrix row per line
        let expected_u = Matrix3::new(
            0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
        );
        assert_eq!(u, expected_u);
        assert_eq!(v_t, expected_u.abs().transpose());
        let (u, s, v_t) = svd3(&Matrix3::identity()).unwrap();
        assert_eq!(
            (u, s, v_t),
            (
                Matrix3::identity(),
                Vector3::repeat(1.0),
                Matrix3::identity()
            )
        );
        let (_, s, _) = svd3(&Matrix3::zeros()).unwrap();
        assert_eq!(s, Vector3::zeros());
        assert!(svd3(&Matrix3::repeat(f64::NAN)).is_none());
    }

    /// A real eigenvector satisfies `A v = λ v`; a rotation block's pair is complex.
    #[test]
    fn eigenpairs_of_a_real_matrix() {
        #[rustfmt::skip] // keep one matrix row per line
        let a = DMatrix::from_row_slice(3, 3, &[
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 2.0,
        ]);
        let (re, im, vectors) = eigen_real(&a).unwrap();
        let real: Vec<usize> = (0..3).filter(|&j| im[j] == 0.0).collect();
        assert_eq!(real.len(), 1);
        let (j, v) = (real[0], vectors.column(real[0]));
        assert!((re[j] - 2.0).abs() < 1e-14);
        assert!((&a * v - v * re[j]).amax() < 1e-14);
        assert!(eigen_real(&DMatrix::zeros(2, 3)).is_none());
    }

    /// The minimum-norm solution of a rank-deficient system ignores the null direction.
    #[test]
    fn least_squares_is_minimum_norm() {
        // Columns 0 and 1 are equal: x0 + x1 is determined, x0 - x1 is not, and is zero.
        let a = DMatrix::from_row_slice(3, 2, &[1.0, 1.0, 2.0, 2.0, 3.0, 3.0]);
        let x = least_squares(&a, &DVector::from_vec(vec![2.0, 4.0, 6.0]), 1e-12).unwrap();
        assert!((x - DVector::from_vec(vec![1.0, 1.0])).amax() < 1e-14);
    }
}
