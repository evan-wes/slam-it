//! Sim(3) RANSAC: the Rust core's implementation (`slam_it.foundations.sim3`).
//!
//! The Python module's docstring states the method (ORB-SLAM's `Sim3Solver`): Umeyama's
//! alignment of three matches, a match an inlier when each point, carried into the other
//! camera, projects within the chi-square bound of its partner's projection, and rung 0's
//! LO-MSAC run to its end.

use nalgebra::{DVector, Matrix4, Vector2, Vector3};

use crate::alignment::umeyama;
use crate::camera::{self, MIN_DEPTH, Model};
use crate::error::{ErrorKind, Result, fail};
use crate::ransac::{self, Estimator, Options};

/// A minimal sample: three matches.
pub const SAMPLE_SIZE: usize = 3;

/// The inverse of `[[s R, t], [0, 1]]`.
///
/// # Arguments
///
/// * `similarity` - The similarity, its scale positive.
///
/// # Returns
///
/// `[[Rᵀ / s, -Rᵀ t / s], [0, 1]]`.
#[must_use]
pub fn inverse(similarity: &Matrix4<f64>) -> Matrix4<f64> {
    let block = similarity.fixed_view::<3, 3>(0, 0);
    let scale2 = block.column(0).norm_squared(); // a column of s R has length s
    let back = block.transpose() / scale2;
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(&back);
    out.fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&(-back * similarity.fixed_view::<3, 1>(0, 3)));
    out
}

/// Carries a point by a similarity: `s R p + t`.
///
/// # Arguments
///
/// * `similarity` - The similarity.
/// * `point` - The point.
///
/// # Returns
///
/// The point carried.
fn carried(similarity: &Matrix4<f64>, point: &Vector3<f64>) -> Vector3<f64> {
    similarity.fixed_view::<3, 3>(0, 0) * point + similarity.fixed_view::<3, 1>(0, 3)
}

/// The matches, checked, and where each point projects in its own image.
#[derive(Debug, Clone)]
pub struct Matches {
    /// Each point in camera 1's frame.
    pub points1: Vec<Vector3<f64>>,
    /// Each point in camera 2's frame.
    pub points2: Vec<Vector3<f64>>,
    /// `points1` projected into image 1.
    pub image1: Vec<Vector2<f64>>,
    /// `points2` projected into image 2.
    pub image2: Vec<Vector2<f64>>,
    /// Each match's variance in image 1, px².
    pub sigma2_1: Vec<f64>,
    /// Each match's variance in image 2, px².
    pub sigma2_2: Vec<f64>,
}

/// The camera both images share.
#[derive(Debug, Clone)]
pub struct Lens {
    /// The camera model.
    pub model: Model,
    /// Its intrinsics, checked.
    pub intrinsics: Vec<f64>,
}

/// Umeyama's alignment with the reprojection test in both images, as a [`ransac::Estimator`].
#[derive(Debug, Clone)]
pub struct SimilarityEstimator {
    /// The matches.
    matches: Matches,
    /// The camera.
    lens: Lens,
    /// Whether the scale is held at 1.
    fixed_scale: bool,
}

impl SimilarityEstimator {
    /// Holds checked matches.
    ///
    /// # Arguments
    ///
    /// * `matches` - The matches.
    /// * `lens` - The camera.
    /// * `fixed_scale` - Fit rigid motions (`s = 1`).
    ///
    /// # Returns
    ///
    /// The estimator.
    #[must_use]
    pub fn new(matches: Matches, lens: Lens, fixed_scale: bool) -> Self {
        Self {
            matches,
            lens,
            fixed_scale,
        }
    }

    /// The least-squares similarity carrying the rows' `points2` onto their `points1`.
    ///
    /// # Arguments
    ///
    /// * `rows` - The matches.
    ///
    /// # Returns
    ///
    /// `S_12`, or none if degenerate.
    fn fit(&self, rows: &[usize]) -> Vec<Matrix4<f64>> {
        let source: Vec<Vector3<f64>> = rows.iter().map(|&r| self.matches.points2[r]).collect();
        let target: Vec<Vector3<f64>> = rows.iter().map(|&r| self.matches.points1[r]).collect();
        umeyama(&source, &target, &vec![1.0; rows.len()], !self.fixed_scale)
            .map_or_else(|_| Vec::new(), |found| vec![found])
    }

    /// A point's squared reprojection error over its variance, if it is in front.
    ///
    /// # Arguments
    ///
    /// * `point` - The point carried into the camera.
    /// * `observed` - Where its partner projects.
    /// * `sigma2` - The variance.
    ///
    /// # Returns
    ///
    /// The ratio, or none behind the camera.
    fn scaled_error(
        &self,
        point: &Vector3<f64>,
        observed: &Vector2<f64>,
        sigma2: f64,
    ) -> Option<f64> {
        if point.z <= MIN_DEPTH {
            return None;
        }
        camera::project(self.lens.model, &self.lens.intrinsics, point)
            .ok()
            .map(|pixel| (pixel - observed).norm_squared() / sigma2)
    }
}

impl Estimator for SimilarityEstimator {
    type Model = Matrix4<f64>;

    fn sample_size(&self) -> usize {
        SAMPLE_SIZE
    }

    fn size(&self) -> usize {
        self.matches.points1.len()
    }

    fn fit_minimal(&self, indices: &[usize]) -> Vec<Matrix4<f64>> {
        self.fit(indices)
    }

    fn fit_nonminimal(&self, indices: &[usize]) -> Vec<Matrix4<f64>> {
        self.fit(indices)
    }

    fn residuals(&self, model: &Matrix4<f64>) -> DVector<f64> {
        let back = inverse(model);
        let m = &self.matches;
        DVector::from_iterator(
            m.points1.len(),
            (0..m.points1.len()).map(|i| {
                let error1 =
                    self.scaled_error(&carried(model, &m.points2[i]), &m.image1[i], m.sigma2_1[i]);
                let error2 =
                    self.scaled_error(&carried(&back, &m.points1[i]), &m.image2[i], m.sigma2_2[i]);
                match (error1, error2) {
                    (Some(a), Some(b)) => a.max(b),
                    _ => f64::INFINITY,
                }
            }),
        )
    }
}

/// Projects points into an image, each in front of the camera.
///
/// # Arguments
///
/// * `points` - The points, checked to be in front.
/// * `lens` - The camera, checked.
///
/// # Returns
///
/// The pixels.
///
/// # Errors
///
/// The projection's error.
fn project_all(points: &[Vector3<f64>], lens: &Lens) -> Result<Vec<Vector2<f64>>> {
    points
        .iter()
        .map(|point| camera::project(lens.model, &lens.intrinsics, point))
        .collect()
}

/// Checks one variance per match: finite and positive.
///
/// # Arguments
///
/// * `values` - The variances.
/// * `count` - The matches.
/// * `name` - Their name, for the message.
///
/// # Errors
///
/// `InvalidInput` if any is not.
fn check_variances(values: &[f64], count: usize, name: &str) -> Result<()> {
    if values.len() != count || !values.iter().all(|v| v.is_finite() && *v > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected {count} positive variances"),
        );
    }
    Ok(())
}

/// Estimates `S_12` from two cameras' matched points robustly (`sim3.sim3_ransac`).
///
/// # Arguments
///
/// * `points1`, `points2` - Each match's point in camera 1's frame and in camera 2's, in front
///   of each, at least three.
/// * `sigma2_1`, `sigma2_2` - Each match's variance in image 1 and in image 2, px², positive.
/// * `lens` - The camera, the same for both images.
/// * `fixed_scale` - Estimate a rigid motion, the scale held at 1.
/// * `options` - RANSAC's settings; the threshold is the chi-square bound's square root.
///
/// # Returns
///
/// `S_12`, its inliers and the run's summary.
///
/// # Errors
///
/// `InvalidInput` for invalid matches, camera or settings; `Degenerate` if no sample gave a
/// similarity.
pub fn sim3_ransac(
    points1: &[Vector3<f64>],
    points2: &[Vector3<f64>],
    (sigma2_1, sigma2_2): (&[f64], &[f64]),
    lens: &Lens,
    fixed_scale: bool,
    options: &Options,
) -> Result<ransac::Solution<Matrix4<f64>>> {
    let n = points1.len();
    let finite = |points: &[Vector3<f64>]| points.iter().all(|p| p.iter().all(|v| v.is_finite()));
    if points2.len() != n || n < SAMPLE_SIZE || !finite(points1) || !finite(points2) {
        return fail(
            ErrorKind::InvalidInput,
            "points1, points2: expected equal (N, 3) arrays, N >= 3, finite",
        );
    }
    if points1.iter().chain(points2).any(|p| p.z <= MIN_DEPTH) {
        return fail(
            ErrorKind::InvalidInput,
            "points1, points2: every point must be in front of its camera",
        );
    }
    check_variances(sigma2_1, n, "sigma2_1")?;
    check_variances(sigma2_2, n, "sigma2_2")?;
    camera::check_intrinsics(lens.model, &lens.intrinsics)?;
    let matches = Matches {
        points1: points1.to_vec(),
        points2: points2.to_vec(),
        image1: project_all(points1, lens)?,
        image2: project_all(points2, lens)?,
        sigma2_1: sigma2_1.to_vec(),
        sigma2_2: sigma2_2.to_vec(),
    };
    ransac::estimate(
        &SimilarityEstimator::new(matches, lens.clone(), fixed_scale),
        options,
    )
}

#[cfg(test)]
mod tests {
    //! Native tests: the similarity and the clean inliers through outliers, the inverse, and a
    //! point carried behind the other camera.

    use nalgebra::Vector6;

    use super::*;
    use crate::se3;
    use crate::solve::Summary as _;

    /// A pinhole camera.
    fn lens() -> Lens {
        Lens {
            model: Model::Pinhole,
            intrinsics: vec![500.0, 500.0, 320.0, 240.0],
        }
    }

    /// A scene: matches with every fourth an outlier, and the similarity of the rest.
    struct Scene {
        /// The true `S_12`.
        truth: Matrix4<f64>,
        /// Points in camera 1's frame.
        points1: Vec<Vector3<f64>>,
        /// Points in camera 2's frame.
        points2: Vec<Vector3<f64>>,
        /// Which matches are true.
        clean: Vec<bool>,
    }

    /// Builds a scene: 80 points 3-8 m before camera 2, carried by `S_12`, a quarter replaced.
    fn scene(scale: f64) -> Scene {
        let mut truth = se3::exp(&Vector6::new(0.3, -0.1, 0.2, 0.1, -0.2, 0.05));
        truth.fixed_view_mut::<3, 3>(0, 0).scale_mut(scale);
        let mut out = Scene {
            truth,
            points1: Vec::new(),
            points2: Vec::new(),
            clean: Vec::new(),
        };
        for i in 0..80_u32 {
            let a = f64::from(i);
            let (u, v) = ((1.7 * a).sin(), (2.3 * a).cos());
            let p2 = Vector3::new(2.0 * u, 1.5 * v, 5.5 + 2.5 * (0.9 * a).sin());
            let good = i % 4 != 0;
            out.points2.push(p2);
            out.points1.push(if good {
                carried(&truth, &p2)
            } else {
                Vector3::new(-1.5 * v, 1.2 * u, 3.0 + u.abs())
            });
            out.clean.push(good);
        }
        out
    }

    /// Through a quarter of outliers the similarity is exact, its inliers the clean matches.
    #[test]
    fn recovers_the_similarity() {
        for fixed_scale in [false, true] {
            let Scene {
                truth,
                points1,
                points2,
                clean,
            } = scene(if fixed_scale { 1.0 } else { 0.7 });
            let ones = vec![1.0; points1.len()];
            let options = Options {
                threshold: 9.21_f64.sqrt(),
                confidence: 0.99,
                max_iterations: 300,
                ..Options::default()
            };
            let found = sim3_ransac(
                &points1,
                &points2,
                (&ones, &ones),
                &lens(),
                fixed_scale,
                &options,
            )
            .unwrap();
            assert!((found.state.model - truth).amax() < 1e-10);
            assert_eq!(found.state.inliers, clean);
            assert!(found.summary.converged());
        }
    }

    /// The inverse undoes the similarity, scale and all.
    #[test]
    fn inverse_undoes_it() {
        let truth = scene(1.3).truth;
        assert!((inverse(&truth) * truth - Matrix4::identity()).amax() < 1e-14);
    }

    /// A match whose point is carried behind the other camera has an infinite residual.
    #[test]
    fn behind_the_camera_is_never_an_inlier() {
        let points = vec![
            Vector3::new(0.0, 0.0, 4.0),
            Vector3::new(1.0, 0.0, 5.0),
            Vector3::new(0.0, 1.0, 6.0),
        ];
        let matches = Matches {
            points1: points.clone(),
            points2: points,
            image1: vec![Vector2::zeros(); 3],
            image2: vec![Vector2::zeros(); 3],
            sigma2_1: vec![1.0; 3],
            sigma2_2: vec![1.0; 3],
        };
        let estimator = SimilarityEstimator::new(matches, lens(), false);
        let mut flip = Matrix4::identity();
        flip[(2, 3)] = -10.0;
        assert!(estimator.residuals(&flip).iter().all(|r| r.is_infinite()));
    }

    /// Malformed matches are refused.
    #[test]
    fn bad_input_is_refused() {
        let Scene {
            points1,
            mut points2,
            ..
        } = scene(1.0);
        let ones = vec![1.0; points1.len()];
        let options = Options::default();
        let run = |p1: &[Vector3<f64>], p2: &[Vector3<f64>], s: &[f64]| {
            sim3_ransac(p1, p2, (s, &ones), &lens(), false, &options)
        };
        assert!(run(&points1[..2], &points2[..2], &ones[..2]).is_err());
        let mut zero = ones.clone();
        zero[3] = 0.0;
        assert!(run(&points1, &points2, &zero).is_err());
        points2[1].z = -1.0;
        assert!(run(&points1, &points2, &ones).is_err());
    }
}
