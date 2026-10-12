//! Homographies: the Rust core's normalised DLT, its Sampson error, the decomposition of a
//! calibrated H, and its RANSAC estimator.
//!
//! Mirrors `slam_it.foundations.homography`, whose docstring states the maths: the DLT on
//! Hartley-normalised points, from the first two rows of `x2 x H x1 = 0`, and the Sampson
//! error `eps^T (J J^T)^-1 eps`. H is returned with unit Frobenius norm, up to sign.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, Vector2, Vector3, Vector4};

use crate::correspondences::{
    RANK_TOLERANCE, check_point_pairs, fit_sample, hartley_normalisation, null_vector,
};
use crate::error::{ErrorKind, Result, fail};
use crate::linalg::singular_values3;
use crate::ransac::{self, Estimator, Options};
use crate::{se3, so3};

/// Correspondences in a minimal sample.
pub const SAMPLE_SIZE: usize = 4;
/// The DLT refuses an H whose smallest singular value is below this fraction of its largest,
/// in the Hartley-normalised frame it is fitted in
/// (`slam_it.foundations.homography.CONDITION_TOLERANCE`, which states the evidence).
pub const CONDITION_TOLERANCE: f64 = 1e-4;
/// Below this largest entry of `HᵀH - I` (H scaled to a middle singular value of 1), H is a
/// rotation (OpenCV's `HomographyDecompInria` value).
pub const PURE_ROTATION_TOLERANCE: f64 = 1e-3;

/// The normalised DLT (unchecked).
///
/// # Arguments
///
/// * `points1`, `points2` - Points in images 1 and 2, N >= 4.
///
/// # Returns
///
/// H with unit norm, or `None` if the pairs do not determine it or fit one of rank below 3
/// (smallest singular value under [`CONDITION_TOLERANCE`] of the largest, in the normalised
/// frame).
#[must_use]
pub fn fit(points1: &[Vector2<f64>], points2: &[Vector2<f64>]) -> Option<Matrix3<f64>> {
    let t1 = hartley_normalisation(points1)?;
    let t2 = hartley_normalisation(points2)?;
    let n = points1.len();
    let mut system = DMatrix::zeros(2 * n, 9);
    // Rows of x2 x H x1 = 0 (with w2 = 1): -(h2 . x1) + y2 (h3 . x1), (h1 . x1) - x2 (h3 . x1).
    for (i, (p1, p2)) in points1.iter().zip(points2).enumerate() {
        let x1 = t1 * p1.push(1.0);
        let x2 = t2 * p2.push(1.0);
        for c in 0..3 {
            system[(2 * i, 3 + c)] = -x1[c];
            system[(2 * i, 6 + c)] = x2[1] * x1[c];
            system[(2 * i + 1, c)] = x1[c];
            system[(2 * i + 1, 6 + c)] = -x2[0] * x1[c];
        }
    }
    let normalised = Matrix3::from_row_slice(null_vector(&system)?.as_slice());
    let singular = singular_values3(&normalised);
    if !(singular[2] > CONDITION_TOLERANCE * singular[0]) {
        return None; // numerically of rank below 3: no homography
    }
    let homography = t2.try_inverse()? * normalised * t1;
    Some(homography / homography.norm())
}

/// The squared Sampson error of point pairs under H (unchecked).
///
/// # Arguments
///
/// * `h` - H.
/// * `points1`, `points2` - Points in images 1 and 2.
///
/// # Returns
///
/// One error per pair; infinity where `J Jᵀ` is singular.
#[must_use]
pub fn sampson(
    h: &Matrix3<f64>,
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
) -> DVector<f64> {
    DVector::from_iterator(
        points1.len(),
        points1.iter().zip(points2).map(|(p1, p2)| {
            let mapped = h * p1.push(1.0);
            let (x2, y2, w) = (p2[0], p2[1], mapped[2]);
            let eps = Vector2::new(y2 * w - mapped[1], mapped[0] - x2 * w);
            // J = d(eps1, eps2) / d(x1, y1, x2, y2).
            let j1 = Vector4::new(
                y2 * h[(2, 0)] - h[(1, 0)],
                y2 * h[(2, 1)] - h[(1, 1)],
                0.0,
                w,
            );
            let j2 = Vector4::new(
                h[(0, 0)] - x2 * h[(2, 0)],
                h[(0, 1)] - x2 * h[(2, 1)],
                -w,
                0.0,
            );
            let (a, b, c) = (j1.norm_squared(), j1.dot(&j2), j2.norm_squared());
            let determinant = a * c - b * b;
            if determinant > 0.0 {
                (c * eps[0] * eps[0] - 2.0 * b * eps[0] * eps[1] + a * eps[1] * eps[1])
                    / determinant
            } else {
                f64::INFINITY
            }
        }),
    )
}

/// Estimates H from four or more point pairs (normalised DLT).
///
/// # Arguments
///
/// * `points1`, `points2` - Points in images 1 and 2.
///
/// # Returns
///
/// H.
///
/// # Errors
///
/// `InvalidInput` for mismatched sizes or N < 4; `Degenerate` if not unique or of rank below 3.
pub fn homography_dlt(points1: &[Vector2<f64>], points2: &[Vector2<f64>]) -> Result<Matrix3<f64>> {
    check_point_pairs(points1, points2, SAMPLE_SIZE, "points")?;
    match fit(points1, points2) {
        Some(homography) => Ok(homography),
        None => fail(
            ErrorKind::Degenerate,
            "H: the correspondences do not determine a unique solution",
        ),
    }
}

/// Squared Sampson errors of point pairs under H.
///
/// # Arguments
///
/// * `homography` - H.
/// * `points1`, `points2` - Points in images 1 and 2.
///
/// # Returns
///
/// One error per pair.
///
/// # Errors
///
/// `InvalidInput` for mismatched sizes or non-finite values.
pub fn sampson_homography(
    homography: &Matrix3<f64>,
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
) -> Result<DVector<f64>> {
    check_point_pairs(points1, points2, 0, "points")?;
    check_finite(homography)?;
    Ok(sampson(homography, points1, points2))
}

/// Checks a homography finite.
///
/// # Arguments
///
/// * `homography` - H.
///
/// # Errors
///
/// `InvalidInput` if any entry is not finite.
fn check_finite(homography: &Matrix3<f64>) -> Result<()> {
    if homography.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        fail(
            ErrorKind::InvalidInput,
            "homography: expected a finite (3, 3) matrix",
        )
    }
}

/// The normalised DLT and its Sampson error, as a [`ransac::Estimator`].
pub struct HomographyEstimator {
    /// The points in image 1.
    points1: Vec<Vector2<f64>>,
    /// The points in image 2.
    points2: Vec<Vector2<f64>>,
}

impl HomographyEstimator {
    /// Holds checked point pairs.
    ///
    /// # Arguments
    ///
    /// * `points1`, `points2` - Points in images 1 and 2.
    ///
    /// # Returns
    ///
    /// The estimator.
    #[must_use]
    pub fn new(points1: &[Vector2<f64>], points2: &[Vector2<f64>]) -> Self {
        Self {
            points1: points1.to_vec(),
            points2: points2.to_vec(),
        }
    }
}

impl Estimator for HomographyEstimator {
    type Model = Matrix3<f64>;

    fn sample_size(&self) -> usize {
        SAMPLE_SIZE
    }

    fn size(&self) -> usize {
        self.points1.len()
    }

    fn fit_nonminimal(&self, indices: &[usize]) -> Vec<Matrix3<f64>> {
        fit_sample(fit, &self.points1, &self.points2, indices)
    }

    fn residuals(&self, model: &Matrix3<f64>) -> DVector<f64> {
        sampson(model, &self.points1, &self.points2)
    }
}

/// The motions a calibrated homography allows: `H ~ R + t nᵀ`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarMotions {
    /// Each candidate `T_c2_c1`, translation over the plane's distance.
    pub poses: Vec<Matrix4<f64>>,
    /// Each candidate's unit plane normal in camera 1 (zero for a pure rotation).
    pub normals: Vec<Vector3<f64>>,
}

/// The negated (row, col) minor of a symmetric 3x3 matrix.
///
/// # Arguments
///
/// * `m` - The matrix.
/// * `row`, `col` - The row and column removed.
///
/// # Returns
///
/// `-M_ij`.
fn opposite_of_minor(m: &Matrix3<f64>, row: usize, col: usize) -> f64 {
    let a = usize::from(row == 0);
    let b = if row == 2 { 1 } else { 2 };
    let c = usize::from(col == 0);
    let d = if col == 2 { 1 } else { 2 };
    m[(a, d)] * m[(b, c)] - m[(a, c)] * m[(b, d)]
}

/// A rotation made proper (a determinant of +1 before projecting) and projected onto SO(3).
///
/// # Arguments
///
/// * `matrix` - Near a rotation, up to sign.
///
/// # Returns
///
/// The rotation.
fn proper_rotation(matrix: Matrix3<f64>) -> Matrix3<f64> {
    so3::project_to_so3(&if matrix.determinant() > 0.0 {
        matrix
    } else {
        -matrix
    })
}

/// A non-negative square root (rounding can push a zero just below).
///
/// # Arguments
///
/// * `x` - The value.
///
/// # Returns
///
/// `sqrt(max(x, 0))`.
fn root(x: f64) -> f64 {
    x.max(0.0).sqrt()
}

/// +1 or -1 by sign, with +1 for zero.
///
/// # Arguments
///
/// * `x` - The value.
///
/// # Returns
///
/// The sign.
fn sign_of(x: f64) -> f64 {
    if x >= 0.0 { 1.0 } else { -1.0 }
}

/// Malis and Vargas' two normal candidates `n_a`, `n_b` (unnormalised).
///
/// # Arguments
///
/// * `s` - `HᵀH - I`.
/// * `k` - The index of the largest `|s_kk|`.
///
/// # Returns
///
/// The pair.
fn normal_candidates(s: &Matrix3<f64>, k: usize) -> (Vector3<f64>, Vector3<f64>) {
    let r00 = root(opposite_of_minor(s, 0, 0));
    let r11 = root(opposite_of_minor(s, 1, 1));
    let r22 = root(opposite_of_minor(s, 2, 2));
    let e01 = sign_of(opposite_of_minor(s, 0, 1));
    let e02 = sign_of(opposite_of_minor(s, 0, 2));
    let e12 = sign_of(opposite_of_minor(s, 1, 2));
    match k {
        0 => (
            Vector3::new(s[(0, 0)], s[(0, 1)] + r22, s[(0, 2)] + e12 * r11),
            Vector3::new(s[(0, 0)], s[(0, 1)] - r22, s[(0, 2)] - e12 * r11),
        ),
        1 => (
            Vector3::new(s[(0, 1)] + r22, s[(1, 1)], s[(1, 2)] - e02 * r00),
            Vector3::new(s[(0, 1)] - r22, s[(1, 1)], s[(1, 2)] + e02 * r00),
        ),
        _ => (
            Vector3::new(s[(0, 2)] + e01 * r11, s[(1, 2)] + r00, s[(2, 2)]),
            Vector3::new(s[(0, 2)] - e01 * r11, s[(1, 2)] - r00, s[(2, 2)]),
        ),
    }
}

/// Decomposes a calibrated H by Malis and Vargas' closed form.
///
/// # Arguments
///
/// * `homography` - H on normalised image points (any scale and sign).
///
/// # Returns
///
/// Four candidates, or one for a pure rotation.
///
/// # Errors
///
/// `InvalidInput` if not finite; `Degenerate` if its rank is below 2.
pub fn decompose_homography(homography: &Matrix3<f64>) -> Result<PlanarMotions> {
    check_finite(homography)?;
    let singular = singular_values3(homography);
    if !(singular[1] > RANK_TOLERANCE * singular[0]) {
        return fail(ErrorKind::Degenerate, "homography: rank below 2");
    }
    let h = homography / singular[1];
    let s = h.transpose() * h - Matrix3::identity();
    if s.amax() < PURE_ROTATION_TOLERANCE {
        // An estimated H is only near a rotation: project it, as for the four candidates.
        return Ok(PlanarMotions {
            poses: vec![se3::from_parts(&proper_rotation(h), &Vector3::zeros())],
            normals: vec![Vector3::zeros()],
        });
    }
    // Names follow Malis and Vargas: S, the plane normals n'_a and n'_b, epsilon = sign(s_kk),
    // and t* the translations before rotation.
    let k = s.diagonal().iamax();
    let (na, nb) = normal_candidates(&s, k);
    let (na, nb) = (na.normalize(), nb.normalize());
    let trace = s.trace();
    // nu = 2 sqrt(1 + tr S - M_11 - M_22 - M_33), rho = sqrt(2 + tr S + nu),
    // |t_e| = sqrt(2 + tr S - nu); each radicand clamped at zero.
    let nu = 2.0
        * root(
            1.0 + trace
                - opposite_of_minor(&s, 0, 0)
                - opposite_of_minor(&s, 1, 1)
                - opposite_of_minor(&s, 2, 2),
        );
    let rho = root(2.0 + trace + nu);
    let t_norm = root(2.0 + trace - nu);
    let epsilon = sign_of(s[(k, k)]);
    let mut out = PlanarMotions {
        poses: Vec::with_capacity(4),
        normals: Vec::with_capacity(4),
    };
    // Each normal's t* = |t_e| / 2 (epsilon rho n_other - |t_e| n): t*_a from n_a, t*_b from n_b.
    for (n, other) in [(na, nb), (nb, na)] {
        let t_star = 0.5 * t_norm * (epsilon * rho * other - t_norm * n);
        // R = H (I - (2 / nu) t* nᵀ); for an estimated H only near a rotation, so projected.
        let rotation =
            proper_rotation(h * (Matrix3::identity() - (2.0 / nu) * t_star * n.transpose()));
        let t = rotation * t_star;
        for sign in [1.0, -1.0] {
            out.poses.push(se3::from_parts(&rotation, &(sign * t)));
            out.normals.push(sign * n);
        }
    }
    Ok(out)
}

/// Estimates H robustly: LO-MSAC with the normalised DLT.
///
/// # Arguments
///
/// * `points1`, `points2` - Points in images 1 and 2, N >= 4.
/// * `options` - The settings (threshold: a Sampson distance in the points' units).
///
/// # Returns
///
/// H, its inliers and the run's summary.
///
/// # Errors
///
/// `InvalidInput` or `Degenerate`.
pub fn homography_ransac(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    options: &Options,
) -> Result<ransac::Solution<Matrix3<f64>>> {
    check_point_pairs(points1, points2, SAMPLE_SIZE, "points")?;
    ransac::estimate(&HomographyEstimator::new(points1, points2), options)
}

#[cfg(test)]
mod tests {
    //! Native tests: the DLT on noise-free pairs, and the true motion among H's candidates.

    use nalgebra::Vector6;

    use super::*;

    /// The DLT recovers a known H from points of a plane seen by two cameras, and the true
    /// motion and normal are among its decomposition's candidates.
    #[test]
    fn dlt_and_decomposition() {
        let pose = se3::exp(&Vector6::new(0.3, -0.1, 0.05, 0.05, 0.1, -0.02));
        let normal = Vector3::new(0.1, -0.2, 1.0).normalize();
        let distance = 3.0;
        let (rotation, t) = (se3::rotation(&pose), se3::translation(&pose));
        let truth = rotation + t * normal.transpose() / distance;
        let mut points1 = Vec::new();
        let mut points2 = Vec::new();
        for i in 0..8 {
            let a = f64::from(i);
            let p = Vector2::new((1.3 * a).sin() * 0.4, (0.7 * a).cos() * 0.3);
            let q = truth * p.push(1.0);
            points1.push(p);
            points2.push(q.xy() / q[2]);
        }
        let h = homography_dlt(&points1, &points2).unwrap();
        let unit = truth / truth.norm();
        assert!((h - unit).amax().min((h + unit).amax()) < 1e-10);
        let motions = decompose_homography(&h).unwrap();
        let expected = se3::from_parts(&rotation, &(t / distance));
        assert!(
            motions
                .poses
                .iter()
                .zip(&motions.normals)
                .any(|(p, n)| (p - expected).amax() < 1e-8 && (n - normal).amax() < 1e-8)
        );
    }
}
