//! Building blocks for estimating geometry from correspondences between two views: the Rust
//! core's mirror of `slamlab.foundations.correspondences`.
//!
//! Unchecked helpers shared by the two-view estimators (`epipolar`, `homography`) and
//! absolute pose: homogeneous and normalised image points, Hartley's normalisation, the null
//! vector of a linear system, row gathering. The checks the estimators run on their input are
//! here too, so each core validates once with the same rules.

use std::f64::consts::SQRT_2;

use nalgebra::{DMatrix, Matrix3, SVector, Vector2, Vector3};

use crate::error::{ErrorKind, Result, fail};
use crate::linalg::svd;

/// The 8-point system's second-smallest singular value must exceed this fraction of its
/// largest; below it the solution is not unique.
pub const RANK_TOLERANCE: f64 = 1e-10;
/// Unit bearings must have norm 1 within this (`slamlab.foundations.arrays`' value).
pub const BEARING_TOLERANCE: f64 = 1e-9;

/// Appends a 1 to each 2D point.
///
/// # Arguments
///
/// * `points` - The points.
///
/// # Returns
///
/// `(x, y, 1)` per point.
#[must_use]
pub fn homogeneous(points: &[Vector2<f64>]) -> Vec<Vector3<f64>> {
    points.iter().map(|p| p.push(1.0)).collect()
}

/// Maps bearings with positive z to homogeneous normalised image points `f / f_z`.
///
/// # Arguments
///
/// * `bearings` - The bearings.
///
/// # Returns
///
/// `(x, y, 1)` per bearing.
#[must_use]
pub fn normalised_plane(bearings: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    bearings.iter().map(|f| f / f.z).collect()
}

/// Maps bearings with positive z to normalised image points `(f_x / f_z, f_y / f_z)`.
///
/// # Arguments
///
/// * `bearings` - The bearings.
///
/// # Returns
///
/// `(x, y)` per bearing: [`normalised_plane`] without its ones.
#[must_use]
pub fn normalised_xy(bearings: &[Vector3<f64>]) -> Vec<Vector2<f64>> {
    bearings.iter().map(|f| f.xy() / f.z).collect()
}

/// Hartley's similarity: centroid to the origin, mean distance √2.
///
/// # Arguments
///
/// * `points` - The points (not empty).
///
/// # Returns
///
/// `T`, or `None` if every point coincides.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "a point count is far below 2^52"
)]
pub fn hartley_normalisation(points: &[Vector2<f64>]) -> Option<Matrix3<f64>> {
    let n = points.len() as f64;
    let centroid = points.iter().sum::<Vector2<f64>>() / n;
    let spread = points.iter().map(|p| (p - centroid).norm()).sum::<f64>() / n;
    if !(spread > 0.0) {
        return None;
    }
    let scale = SQRT_2 / spread;
    #[rustfmt::skip] // keep one matrix row per line
    let t = Matrix3::new(
        scale, 0.0, -scale * centroid.x,
        0.0, scale, -scale * centroid.y,
        0.0, 0.0, 1.0,
    );
    Some(t)
}

/// Solves `A v = 0` for a unit `v` by SVD, if the solution is unique.
///
/// # Arguments
///
/// * `system` - `A`, M x 9 with M >= 8.
///
/// # Returns
///
/// The right singular vector of the smallest singular value, or `None` if the
/// second-smallest is below [`RANK_TOLERANCE`] of the largest (or the SVD fails).
#[must_use]
pub fn null_vector(system: &DMatrix<f64>) -> Option<SVector<f64, 9>> {
    // Pad an eight-row system to nine so V is the full 9x9 basis.
    let padded = if system.nrows() < 9 {
        system.clone().resize_vertically(9, 0.0)
    } else {
        system.clone()
    };
    let decomposition = svd(padded, false, true)?;
    let singular = &decomposition.singular_values;
    if !(singular[7] > RANK_TOLERANCE * singular[0]) {
        return None;
    }
    let v_t = decomposition.v_t?;
    Some(SVector::<f64, 9>::from_iterator(v_t.row(8).iter().copied()))
}

/// Checks one bearing: finite, with unit norm within [`BEARING_TOLERANCE`].
///
/// # Arguments
///
/// * `bearing` - The bearing.
/// * `name` - The argument's name, for the message.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming the argument.
pub fn check_bearing(bearing: &Vector3<f64>, name: &str) -> Result<()> {
    if !bearing.iter().all(|x| x.is_finite()) || (bearing.norm() - 1.0).abs() > BEARING_TOLERANCE {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected unit vectors within 1e-09"),
        );
    }
    Ok(())
}

/// The failure for a bearing that is not in front of a pinhole camera.
///
/// # Returns
///
/// `InvalidInput`.
fn behind<T>() -> Result<T> {
    fail(
        ErrorKind::InvalidInput,
        "bearings: expected positive z (a pinhole view)",
    )
}

/// Checks bearing pairs: equal sizes, at least `minimum`, unit vectors, and (if asked)
/// positive z, which methods working in the normalised image plane need.
///
/// # Arguments
///
/// * `bearings1` - Bearings in camera 1.
/// * `bearings2` - Bearings in camera 2.
/// * `minimum` - The fewest pairs accepted.
/// * `positive_z` - Require positive z.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming what failed.
pub fn check_bearing_pairs(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    minimum: usize,
    positive_z: bool,
) -> Result<()> {
    if bearings1.len() != bearings2.len() {
        return fail(
            ErrorKind::InvalidInput,
            "bearings: expected equal (N, 3) shapes",
        );
    }
    if bearings1.len() < minimum {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "bearings: need at least {minimum} pairs, got {}",
                bearings1.len()
            ),
        );
    }
    for (b1, b2) in bearings1.iter().zip(bearings2) {
        for (bearing, name) in [(b1, "bearings1"), (b2, "bearings2")] {
            check_bearing(bearing, name)?;
            if positive_z && !(bearing.z > 0.0) {
                return behind();
            }
        }
    }
    Ok(())
}

/// Checks world points and their bearings: equal sizes, at least `minimum`, finite points,
/// unit bearings, and (if asked) positive z.
///
/// # Arguments
///
/// * `points` - World points.
/// * `bearings` - Their bearings in the camera.
/// * `minimum` - The fewest correspondences accepted.
/// * `positive_z` - Require positive z.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming what failed.
pub fn check_correspondences(
    points: &[Vector3<f64>],
    bearings: &[Vector3<f64>],
    minimum: usize,
    positive_z: bool,
) -> Result<()> {
    if points.len() != bearings.len() {
        return fail(
            ErrorKind::InvalidInput,
            "points, bearings: expected equal (N, 3) shapes",
        );
    }
    if points.len() < minimum {
        return fail(
            ErrorKind::InvalidInput,
            format!("points: need at least {minimum}, got {}", points.len()),
        );
    }
    if !points.iter().all(|p| p.iter().all(|x| x.is_finite())) {
        return fail(ErrorKind::InvalidInput, "points: expected finite values");
    }
    for bearing in bearings {
        check_bearing(bearing, "bearings")?;
        if positive_z && !(bearing.z > 0.0) {
            return behind();
        }
    }
    Ok(())
}

/// Checks point pairs: equal sizes, at least `minimum`, finite.
///
/// # Arguments
///
/// * `points1` - Points in image 1.
/// * `points2` - Points in image 2.
/// * `minimum` - The fewest pairs accepted.
/// * `name` - The arguments' name, for messages.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming what failed.
pub fn check_point_pairs(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    minimum: usize,
    name: &str,
) -> Result<()> {
    if points1.len() != points2.len() {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected equal (N, 2) shapes"),
        );
    }
    if points1.len() < minimum {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{name}: need at least {minimum} pairs, got {}",
                points1.len()
            ),
        );
    }
    let finite = |p: &Vector2<f64>| p.iter().all(|x| x.is_finite());
    if !points1.iter().all(finite) || !points2.iter().all(finite) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected finite values"),
        );
    }
    Ok(())
}

/// Gathers the rows at the given indices.
///
/// # Arguments
///
/// * `rows` - The rows.
/// * `indices` - The rows to take.
///
/// # Returns
///
/// The gathered rows.
#[must_use]
pub fn gather<T: Copy>(rows: &[T], indices: &[usize]) -> Vec<T> {
    indices.iter().map(|&i| rows[i]).collect()
}

/// Fits a 3x3 model on a sample of the pairs: an estimator's non-minimal fit.
///
/// # Arguments
///
/// * `fit` - The fit, on the gathered pairs.
/// * `first`, `second` - Every pair's two sides.
/// * `indices` - The sample.
///
/// # Returns
///
/// The model, or none if the sample does not determine it.
pub fn fit_sample<T: Copy>(
    fit: fn(&[T], &[T]) -> Option<Matrix3<f64>>,
    first: &[T],
    second: &[T],
    indices: &[usize],
) -> Vec<Matrix3<f64>> {
    fit(&gather(first, indices), &gather(second, indices))
        .into_iter()
        .collect()
}

#[cfg(test)]
mod tests {
    //! Native tests of the helpers.

    use super::*;

    /// Hartley's similarity centres the points at mean distance √2; coincident points have
    /// none.
    #[test]
    fn hartley_normalises() {
        let points = [
            Vector2::new(1.0, 2.0),
            Vector2::new(5.0, -1.0),
            Vector2::new(-2.0, 4.0),
        ];
        let t = hartley_normalisation(&points).unwrap();
        let moved: Vec<Vector2<f64>> = points.iter().map(|p| (t * p.push(1.0)).xy()).collect();
        let centroid = moved.iter().sum::<Vector2<f64>>() / 3.0;
        assert!(centroid.norm() < 1e-14);
        let spread = moved.iter().map(Vector2::norm).sum::<f64>() / 3.0;
        assert!((spread - SQRT_2).abs() < 1e-14);
        assert!(hartley_normalisation(&[Vector2::new(1.0, 1.0); 3]).is_none());
    }

    /// The null vector of a rank-8 system spans its kernel; a rank-deficient one has none.
    #[test]
    fn null_vector_of_a_rank_eight_system() {
        let rows: Vec<f64> = (0..72).map(|k| (f64::from(k * k) * 0.137).sin()).collect();
        let a = DMatrix::from_row_slice(8, 9, &rows);
        let v = null_vector(&a).unwrap();
        assert!((&a * DMatrix::from_column_slice(9, 1, v.as_slice())).amax() < 1e-13);
        assert!((v.norm() - 1.0).abs() < 1e-14);
        let deficient = DMatrix::from_fn(8, 9, |i, j| f64::from(u8::try_from(i + j).unwrap()));
        assert!(null_vector(&deficient).is_none());
    }

    /// The input checks name what is wrong.
    #[test]
    fn checks_reject_bad_input() {
        let unit = Vector3::new(0.0, 0.0, 1.0);
        let kind = |r: Result<()>| r.unwrap_err().kind;
        assert_eq!(
            kind(check_bearing_pairs(&[unit], &[], 0, true)),
            ErrorKind::InvalidInput
        );
        assert_eq!(
            kind(check_bearing_pairs(&[unit], &[-unit], 0, true)),
            ErrorKind::InvalidInput
        );
        assert_eq!(
            kind(check_correspondences(
                &[Vector3::repeat(f64::NAN)],
                &[unit],
                1,
                true
            )),
            ErrorKind::InvalidInput
        );
        assert!(check_bearing_pairs(&[unit], &[-unit], 1, false).is_ok());
    }
}
