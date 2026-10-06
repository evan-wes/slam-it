//! Observation errors: the Rust core's mirror of `slamlab.edge.residuals`.
//!
//! A keypoint on pyramid level `l` has the variance `s^(2 l)`; an observation is an outlier
//! when its squared reprojection error over that variance passes the 95% chi-square threshold,
//! with the predicted right column `u - f b / z` compared too where it has one.

use nalgebra::Vector3;
use slamlab_foundations::error::{ErrorKind, Result, fail};

use crate::frames::Camera;
use crate::map::Map;
use crate::options::Options;

/// A level's variance, `s^(2 l)` (`slamlab.edge.residuals.sigma2`).
///
/// # Arguments
///
/// * `m` - The map (its scale factor).
/// * `level` - The level.
///
/// # Returns
///
/// The variance.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "levels are at most MAX_LEVELS, exact in f64"
)]
pub fn sigma2(m: &Map, level: usize) -> f64 {
    m.scale_factor.powf(2.0 * level as f64)
}

/// An observation's chi-square threshold (`slamlab.edge.residuals.thresholds`).
///
/// # Arguments
///
/// * `right` - Its right column, or NaN.
/// * `options` - The thresholds.
///
/// # Returns
///
/// `chi2_stereo` with a right column, else `chi2_mono`.
#[must_use]
pub fn threshold(right: f64, options: &Options) -> f64 {
    if right.is_nan() {
        options.chi2_mono
    } else {
        options.chi2_stereo
    }
}

/// Each observation's chi-square error (`slamlab.edge.residuals.chi2`).
///
/// # Arguments
///
/// * `camera` - The camera.
/// * `local` - The points, in the camera's frame.
/// * `xy` - Their observed pixels.
/// * `right` - Their observed right columns (NaN for none).
/// * `variance` - Their levels' variances.
///
/// # Returns
///
/// `(|pi(X) - x|^2 + (pi_u(X) - f b / Z - u_r)^2) / sigma^2`, infinite for a point not in
/// front.
///
/// # Errors
///
/// `InvalidInput` for values that are not finite (NaN allowed in `right`), slices of
/// different lengths, or a variance that is not positive.
pub fn chi2(
    camera: &Camera,
    local: &[Vector3<f64>],
    xy: &[[f64; 2]],
    right: &[f64],
    variance: &[f64],
) -> Result<Vec<f64>> {
    let n = local.len();
    if xy.len() != n || right.len() != n || variance.len() != n {
        return fail(
            ErrorKind::InvalidInput,
            "chi2: expected one row per observation in every array",
        );
    }
    for i in 0..n {
        if !(local[i].iter().all(|v| v.is_finite())
            && xy[i][0].is_finite()
            && xy[i][1].is_finite()
            && !right[i].is_infinite()
            && variance[i].is_finite())
        {
            return fail(
                ErrorKind::InvalidInput,
                "chi2: expected finite values (NaN allowed in right)",
            );
        }
        if variance[i] <= 0.0 {
            return fail(ErrorKind::InvalidInput, "variance: must be positive");
        }
    }
    let bf = camera.focal() * camera.baseline;
    Ok((0..n)
        .map(|i| {
            if !(local[i].z > 0.0) {
                return f64::INFINITY;
            }
            let pixel = camera.project(&local[i]);
            let dx = pixel.x - xy[i][0];
            let dy = pixel.y - xy[i][1];
            let mut squared = dx * dx + dy * dy;
            if !right[i].is_nan() {
                let dr = pixel.x - bf / local[i].z - right[i];
                squared += dr * dr;
            }
            squared / variance[i]
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use slamlab_foundations::camera::Model;

    /// The tests' camera: 250 px focal, a 0.1 m baseline.
    fn camera() -> Camera {
        Camera {
            model: Model::Pinhole,
            intrinsics: vec![250.0, 250.0, 160.0, 120.0],
            width: 320,
            height: 240,
            baseline: 0.1,
        }
    }

    #[test]
    fn chi2_weighs_both_terms_and_refuses_points_behind() {
        let local = [
            Vector3::new(0.0, 0.0, 2.0),
            Vector3::new(0.0, 0.0, 2.0),
            Vector3::new(0.0, 0.0, -1.0),
        ];
        let xy = [[163.0, 124.0], [160.0, 120.0], [160.0, 120.0]];
        // The second's predicted right column is 160 - 250 * 0.1 / 2 = 147.5.
        let right = [f64::NAN, 145.5, 147.5];
        let errors = chi2(&camera(), &local, &xy, &right, &[1.0, 4.0, 1.0]).unwrap();
        assert!((errors[0] - 25.0).abs() < 1e-12);
        assert!((errors[1] - 1.0).abs() < 1e-12);
        assert!(errors[2].is_infinite());
    }

    #[test]
    fn chi2_refuses_bad_input() {
        let local = [Vector3::new(0.0, 0.0, 2.0)];
        let err = chi2(&camera(), &local, &[[0.0, 0.0]], &[f64::INFINITY], &[1.0]).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
        let err = chi2(&camera(), &local, &[[0.0, 0.0]], &[f64::NAN], &[0.0]).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
        let err = chi2(&camera(), &local, &[], &[], &[]).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
    }

    #[test]
    fn thresholds_follow_the_right_column() {
        let options = Options::default();
        assert!((threshold(f64::NAN, &options) - options.chi2_mono).abs() < f64::EPSILON);
        assert!((threshold(10.0, &options) - options.chi2_stereo).abs() < f64::EPSILON);
        let m = Map::empty(8, 1.2);
        assert!((sigma2(&m, 2) - 1.2_f64.powi(4)).abs() < 1e-12);
    }
}
