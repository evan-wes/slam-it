//! Dense linear algebra shared by rung 0's Rust modules: an SVD that always returns, and the
//! minimum-norm least squares built on it.
//!
//! nalgebra's general `svd` has no iteration cap, so a matrix with a NaN or an infinity never
//! returns, where LAPACK (numpy) and Eigen stop. Every dynamic SVD in the crate goes through
//! [`svd`], which checks finiteness first and caps the iterations. Fixed 2x2 and 3x3
//! matrices take nalgebra's closed-form routines, which always return, and need no helper.
//! There is no Python or C++ counterpart: numpy and Eigen need none.

use nalgebra::{DMatrix, DVector, Dyn, SVD};

/// Golub-Kahan sweeps allowed in a dynamic SVD before it is abandoned.
pub const SVD_MAX_ITERATIONS: usize = 10_000;

/// A dynamic matrix's SVD, singular values largest first, or `None` for a matrix with a
/// non-finite entry or one that does not converge within [`SVD_MAX_ITERATIONS`].
///
/// The convergence tolerance is nalgebra's own (`5 ε`), so finite results are those of its
/// `svd`.
///
/// # Arguments
///
/// * `matrix` - The matrix (not empty).
/// * `compute_u` - Whether to compute `U`.
/// * `compute_v` - Whether to compute `Vᵀ`.
///
/// # Returns
///
/// The decomposition.
#[must_use]
pub fn svd(matrix: DMatrix<f64>, compute_u: bool, compute_v: bool) -> Option<SVD<f64, Dyn, Dyn>> {
    if !matrix.iter().all(|x| x.is_finite()) {
        return None;
    }
    matrix.try_svd(compute_u, compute_v, f64::EPSILON * 5.0, SVD_MAX_ITERATIONS)
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
    matrix: DMatrix<f64>,
    rhs: &DVector<f64>,
    relative: f64,
) -> Option<DVector<f64>> {
    let decomposition = svd(matrix, true, true)?;
    let threshold = relative * decomposition.singular_values.max();
    decomposition.solve(rhs, threshold).ok()
}

#[cfg(test)]
mod tests {
    //! Native tests of the capped SVD.

    use super::*;

    /// A finite matrix decomposes as nalgebra's own SVD does; a non-finite one fails at once.
    #[test]
    fn finite_matrices_decompose_and_others_fail() {
        let m = DMatrix::from_row_slice(3, 2, &[1.0, 2.0, 3.0, 4.0, 5.0, 7.0]);
        let ours = svd(m.clone(), false, false).unwrap().singular_values;
        let theirs = m.clone().svd(false, false).singular_values;
        assert!((ours - theirs).amax() < 1e-15);
        let mut bad = m;
        bad[(1, 1)] = f64::NAN;
        assert!(svd(bad, true, true).is_none());
    }
}
