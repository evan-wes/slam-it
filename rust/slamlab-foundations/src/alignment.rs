//! Point-set alignment (Umeyama): the Rust core's implementation.
//!
//! Mirrors `slamlab.foundations.alignment`: the rotation, translation and optional scale
//! minimising `Σ w_i |y_i - (s R x_i + t)|²` (Umeyama, TPAMI 1991).

use nalgebra::{Matrix3, Matrix4, Vector3};

use crate::error::{ErrorKind, Result, fail};
use crate::se3;

/// The cross-covariance's second singular value must exceed this fraction of its first;
/// below it the points are collinear and the rotation is undetermined.
pub const DEGENERACY_RATIO: f64 = 1e-10;

/// Aligns corresponding point sets.
///
/// # Arguments
///
/// * `source` - Source points, N >= 3.
/// * `target` - Target points, N.
/// * `weights` - Non-negative weights, N, at least three positive.
/// * `with_scale` - Estimate a scale too (Sim(3)).
///
/// # Returns
///
/// The 4x4 matrix `[[s R, t], [0, 1]]`.
///
/// # Errors
///
/// `InvalidInput` for mismatched lengths, a non-finite value or negative weights;
/// `Degenerate` for fewer than three weighted points, or collinear points.
///
/// # Panics
///
/// Never: the SVD is asked for both `U` and `Vᵀ`, so both are present.
pub fn umeyama(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    weights: &[f64],
    with_scale: bool,
) -> Result<Matrix4<f64>> {
    let n = source.len();
    if target.len() != n || weights.len() != n {
        return fail(
            ErrorKind::InvalidInput,
            "umeyama: source, target and weights differ in length",
        );
    }
    let finite = |points: &[Vector3<f64>]| points.iter().all(|p| p.iter().all(|x| x.is_finite()));
    if !finite(source) || !finite(target) || !weights.iter().all(|w| w.is_finite()) {
        return fail(
            ErrorKind::InvalidInput,
            "source, target, weights: expected finite values",
        );
    }
    if weights.iter().any(|&w| w < 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            "weights: expected N non-negative values",
        );
    }
    if weights.iter().filter(|&&w| w > 0.0).count() < 3 {
        return fail(
            ErrorKind::Degenerate,
            "umeyama: fewer than three points with positive weight",
        );
    }
    let total: f64 = weights.iter().sum();
    let w: Vec<f64> = weights.iter().map(|x| x / total).collect();
    let mean = |points: &[Vector3<f64>]| -> Vector3<f64> {
        points.iter().zip(&w).map(|(p, wi)| p * *wi).sum()
    };
    let (mu_x, mu_y) = (mean(source), mean(target));
    let mut cov = Matrix3::zeros();
    let mut spread = 0.0;
    for ((x, y), wi) in source.iter().zip(target).zip(&w) {
        let (xc, yc) = (x - mu_x, y - mu_y);
        cov += (yc * *wi) * xc.transpose();
        spread += wi * xc.norm_squared();
    }
    let svd = cov.svd(true, true);
    let u = svd.u.expect("U was requested");
    let v_t = svd.v_t.expect("V^T was requested");
    let d = svd.singular_values;
    if d[1] <= DEGENERACY_RATIO * d[0] {
        return fail(
            ErrorKind::Degenerate,
            "umeyama: points are collinear or coincident",
        );
    }
    let mut sign = Vector3::new(1.0, 1.0, 1.0);
    if u.determinant() * v_t.determinant() < 0.0 {
        sign[2] = -1.0;
    }
    let rotation = u * Matrix3::from_diagonal(&sign) * v_t;
    let scale = if with_scale {
        d.dot(&sign) / spread
    } else {
        1.0
    };
    Ok(se3::from_parts(
        &(rotation * scale),
        &(mu_y - rotation * mu_x * scale),
    ))
}

#[cfg(test)]
mod tests {
    //! Native tests, runnable without Python: recovery of a known similarity, and the
    //! degenerate inputs.

    use super::*;
    use crate::so3;

    /// Corners of a non-planar point set.
    fn cloud() -> Vec<Vector3<f64>> {
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.2),
            Vector3::new(0.0, 2.0, -0.4),
            Vector3::new(0.5, 0.3, 1.5),
            Vector3::new(-1.0, 0.7, 0.1),
        ]
    }

    /// A known similarity is recovered exactly from noise-free points.
    #[test]
    fn recovers_a_similarity() {
        let rotation = so3::exp(&Vector3::new(0.3, -1.0, 0.5));
        let (scale, t) = (2.5, Vector3::new(1.0, -2.0, 3.0));
        let source = cloud();
        let target: Vec<_> = source.iter().map(|p| rotation * p * scale + t).collect();
        let weights = vec![1.0; source.len()];
        let found = umeyama(&source, &target, &weights, true).unwrap();
        let expected = se3::from_parts(&(rotation * scale), &t);
        assert!((found - expected).amax() < 1e-12);
    }

    /// Collinear points and too few weighted points are `Degenerate`.
    #[test]
    fn degenerate_input_is_rejected() {
        let line: Vec<_> = (0..5)
            .map(|i| Vector3::new(f64::from(i), 0.0, 0.0))
            .collect();
        let err = umeyama(&line, &line, &[1.0; 5], false).unwrap_err();
        assert_eq!(err.kind, ErrorKind::Degenerate);
        let err = umeyama(&cloud(), &cloud(), &[1.0, 1.0, 0.0, 0.0, 0.0], false).unwrap_err();
        assert_eq!(err.kind, ErrorKind::Degenerate);
    }

    /// A NaN weight or point is `InvalidInput`, not a NaN result returned as `Ok`.
    #[test]
    fn non_finite_input_is_rejected() {
        let nan_weights = [1.0, f64::NAN, 1.0, 1.0, 1.0];
        let err = umeyama(&cloud(), &cloud(), &nan_weights, false).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
        let mut target = cloud();
        target[2].x = f64::INFINITY;
        let err = umeyama(&cloud(), &target, &[1.0; 5], true).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
    }
}
