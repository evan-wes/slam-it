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
//! The 3x3 SVD is translated from Eigen, so it has a module and file of its own,
//! [`jacobi_svd3`], under Eigen's licence (the Mozilla Public License 2.0).
//!
//! [`svd3`]: jacobi_svd3::svd3
//! [`singular_values3`]: jacobi_svd3::singular_values3

use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::evd::{self as faer_evd, ComputeEigenvectors};
use faer::linalg::svd::{self as faer_svd, ComputeSvdVectors};
use faer::{Mat, Par};
use nalgebra::{DMatrix, DVector};

pub mod jacobi_svd3;

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

    use nalgebra::{Matrix3, Rotation3, Vector3};

    use super::jacobi_svd3::{singular_values3, svd3};
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
        // Bits, not values: `==` equates 0.0 and -0.0, whose sign Eigen moves into U.
        let bits = |values: &[f64]| values.iter().map(|x| x.to_bits()).collect::<Vec<u64>>();
        let cases = fixture["cases"].as_array().expect("cases");
        assert!(cases.len() >= 21);
        for case in cases {
            let name = case["name"].as_str().expect("a name");
            let m = Matrix3::from_row_slice(&numbers(&case["matrix"]));
            let (u, s, v_t) = svd3(&m).expect("a finite matrix");
            let expected_u = Matrix3::from_row_slice(&numbers(&case["u"]));
            assert_eq!(bits(u.as_slice()), bits(expected_u.as_slice()), "{name}: U");
            let singular = numbers(&case["singular_values"]);
            assert_eq!(bits(s.as_slice()), bits(&singular), "{name}: σ");
            let alone = singular_values3(&m);
            assert_eq!(bits(alone.as_slice()), bits(&singular), "{name}: σ alone");
            let v = Matrix3::from_row_slice(&numbers(&case["v"]));
            assert_eq!(
                bits(v_t.as_slice()),
                bits(v.transpose().as_slice()),
                "{name}: V"
            );
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
