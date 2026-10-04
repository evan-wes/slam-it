//! Triangulation: the Rust core's two-view optimal (Lindstrom's niter2) and N-view linear
//! (DLT on bearings) methods.
//!
//! Mirrors `slamlab.foundations.triangulation`, whose docstring states the maths. Rays are
//! unit bearing vectors; poses are `T_cam_world`. A point whose rays are parallel is NaN, with
//! `in_front` false and parallax 0: counted by the caller, never raised.

use nalgebra::{DMatrix, Matrix2, Matrix4, Vector2, Vector3};

use crate::correspondences::{check_bearing, check_bearing_pairs};
use crate::error::{ErrorKind, Result, fail};
use crate::linalg::svd;
use crate::{se3, so3};

/// Rays meeting at an angle whose sine is below this are parallel; the DLT uses the same
/// ratio between the homogeneous point's last coordinate and the rest.
pub const PARALLEL_SINE: f64 = 1e-12;

/// Triangulated points, with their cheirality and parallax.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Triangulation {
    /// The points; NaN where the rays are parallel.
    pub points: Vec<Vector3<f64>>,
    /// Whether each point lies ahead along every ray that observes it.
    pub in_front: Vec<bool>,
    /// The largest angle at each point between two observing camera centres (radians).
    pub parallax: Vec<f64>,
}

impl Triangulation {
    /// Records one point, or a point at infinity (NaN, behind, no parallax).
    ///
    /// # Arguments
    ///
    /// * `point` - The point.
    /// * `in_front` - Its cheirality.
    /// * `parallax` - Its parallax.
    /// * `bad` - Whether it is at infinity or undetermined.
    fn push(&mut self, point: Vector3<f64>, in_front: bool, parallax: f64, bad: bool) {
        if bad {
            self.points.push(Vector3::repeat(f64::NAN));
            self.in_front.push(false);
            self.parallax.push(0.0);
        } else {
            self.points.push(point);
            self.in_front.push(in_front);
            self.parallax.push(parallax);
        }
    }
}

/// The angle between two vectors, from `atan2` (accurate at 0 and π).
///
/// # Arguments
///
/// * `a` - The first vector.
/// * `b` - The second vector.
///
/// # Returns
///
/// The angle in [0, π].
fn angle(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a.cross(b).norm().atan2(a.dot(b))
}

/// Triangulates bearing pairs optimally through a known relative pose (Lindstrom's niter2,
/// then the exact intersection of the corrected rays).
///
/// # Arguments
///
/// * `relative_pose` - `T_c2_c1`, with a non-zero translation.
/// * `bearings1` - Unit bearings in camera 1, positive z.
/// * `bearings2` - Unit bearings in camera 2, positive z.
///
/// # Returns
///
/// The points in camera 1's frame.
///
/// # Errors
///
/// `InvalidInput` for a bad pose, a zero translation, mismatched sizes, or bearings that are
/// not unit vectors with positive z.
pub fn two_view(
    relative_pose: &Matrix4<f64>,
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
) -> Result<Triangulation> {
    se3::check_transform(relative_pose)?;
    let rotation = se3::rotation(relative_pose);
    let t = se3::translation(relative_pose);
    if !(t.norm() > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            "relative_pose: zero translation; the depth is undetermined",
        );
    }
    check_bearing_pairs(bearings1, bearings2, 0, true)?;
    let essential = so3::hat(&t) * rotation;
    let e2: Matrix2<f64> = essential.fixed_view::<2, 2>(0, 0).into_owned();
    let centre2 = -rotation.transpose() * t;
    let mut out = Triangulation::default();
    for (b1, b2) in bearings1.iter().zip(bearings2) {
        let x1 = b1 / b1.z;
        let x2 = b2 / b2.z;
        // niter2, with the paper's (x, x') = (x2, x1): the constraint is x2ᵀ E x1 = 0.
        let mut n2: Vector2<f64> = (essential * x1).xy();
        let mut n1: Vector2<f64> = (essential.transpose() * x2).xy();
        let a = n2.dot(&(e2 * n1));
        let b = f64::midpoint(n2.norm_squared(), n1.norm_squared());
        let c = x2.dot(&(essential * x1));
        let d = (b * b - a * c).max(0.0).sqrt();
        let mut lambda = c / (b + d);
        let step2 = n2 * lambda;
        let step1 = n1 * lambda;
        n2 -= e2 * step1;
        n1 -= e2.transpose() * step2;
        lambda *= 2.0 * d / (n2.norm_squared() + n1.norm_squared());
        let x2c = Vector3::new(x2.x - lambda * n2.x, x2.y - lambda * n2.y, x2.z);
        let x1c = Vector3::new(x1.x - lambda * n1.x, x1.y - lambda * n1.y, x1.z);
        // Intersect the corrected rays: d2 x2c = d1 R x1c + t.
        let rotated = rotation * x1c;
        let across = x2c.cross(&rotated);
        let parallel = !(across.norm() > PARALLEL_SINE * x2c.norm() * rotated.norm());
        let depth1 = -x2c.cross(&t).dot(&across) / across.norm_squared();
        let point = x1c * depth1;
        let in_front = point.dot(b1) > 0.0 && (rotation * point + t).dot(b2) > 0.0;
        let bad = parallel || !point.iter().all(|v| v.is_finite());
        out.push(point, in_front, angle(&point, &(point - centre2)), bad);
    }
    Ok(out)
}

/// One point's observations, for the DLT.
struct Track<'a> {
    /// Every view's `T_cam_world`.
    poses: &'a [Matrix4<f64>],
    /// Every view's centre, `-Rᵀ t`.
    centres: &'a [Vector3<f64>],
    /// The point's bearing in every view.
    bearings: &'a [Vector3<f64>],
    /// The views that observe it.
    seen: &'a [usize],
}

/// The normalised DLT on one track's bearings.
///
/// # Arguments
///
/// * `track` - The point's observations.
///
/// # Returns
///
/// The point, and whether it is undetermined (at infinity, the system leaves a line of
/// solutions, or the SVD fails).
#[allow(clippy::cast_precision_loss, reason = "a view count is far below 2^52")]
fn dlt(track: &Track<'_>) -> (Vector3<f64>, bool) {
    let k = track.seen.len();
    let mean = track
        .seen
        .iter()
        .map(|&v| track.centres[v])
        .sum::<Vector3<f64>>()
        / k as f64;
    let spread = track
        .seen
        .iter()
        .map(|&v| (track.centres[v] - mean).norm())
        .sum::<f64>()
        / k as f64;
    let scale = if spread > 0.0 { spread } else { 1.0 };
    let mut system = DMatrix::zeros(3 * k, 4);
    for (j, &v) in track.seen.iter().enumerate() {
        let r = se3::rotation(&track.poses[v]);
        // In the normalised world X = mean + scale X', so R X + t = scale (R X' + t').
        let t_norm = (se3::translation(&track.poses[v]) + r * mean) / scale;
        let skew = so3::hat(&track.bearings[v]);
        system
            .fixed_view_mut::<3, 3>(3 * j, 0)
            .copy_from(&(skew * r));
        system
            .fixed_view_mut::<3, 1>(3 * j, 3)
            .copy_from(&(skew * t_norm));
    }
    let Some(decomposition) = svd(system, false, true) else {
        return (Vector3::repeat(f64::NAN), true);
    };
    let Some(v_t) = decomposition.v_t else {
        return (Vector3::repeat(f64::NAN), true);
    };
    let singular = &decomposition.singular_values;
    let homogeneous = v_t.row(3);
    let w = homogeneous[3];
    let direction = Vector3::new(homogeneous[0], homogeneous[1], homogeneous[2]);
    // Rays along one line leave a two-dimensional null space: the point is undetermined.
    let undetermined = !(singular[2] > PARALLEL_SINE * singular[0])
        || !(w.abs() > PARALLEL_SINE * direction.norm());
    (mean + direction * (scale / w), undetermined)
}

/// A point's cheirality over the views that observe it, and its largest parallax.
///
/// # Arguments
///
/// * `track` - The point's observations.
/// * `point` - The point.
///
/// # Returns
///
/// Whether it is ahead along every ray, and the largest angle between two centres.
fn cheirality_and_parallax(track: &Track<'_>, point: &Vector3<f64>) -> (bool, f64) {
    let mut in_front = true;
    let mut parallax = 0.0_f64;
    for (j, &v) in track.seen.iter().enumerate() {
        let pose = &track.poses[v];
        let in_cam = se3::rotation(pose) * point + se3::translation(pose);
        in_front = in_front && in_cam.dot(&track.bearings[v]) > 0.0;
        for &m in &track.seen[j + 1..] {
            parallax = parallax.max(angle(
                &(point - track.centres[v]),
                &(point - track.centres[m]),
            ));
        }
    }
    (in_front, parallax)
}

/// Triangulates points seen in two or more views, by the DLT on bearing vectors.
///
/// # Arguments
///
/// * `poses` - Each view's `T_cam_world` (V).
/// * `bearings` - Each point's bearing in each view, entry `p * V + v` (P V).
/// * `observed` - Which views observe each point, entry `p * V + v` (P V); each point needs
///   two.
///
/// # Returns
///
/// The points in the world frame (NaN where undetermined: at infinity, or rays along one
/// line).
///
/// # Errors
///
/// `InvalidInput` for a bad pose, mismatched sizes, an observed bearing that is not a unit
/// vector, or a point seen in fewer than two views.
pub fn n_view(
    poses: &[Matrix4<f64>],
    bearings: &[Vector3<f64>],
    observed: &[bool],
) -> Result<Triangulation> {
    let views = poses.len();
    if views == 0 || observed.len() != bearings.len() || !bearings.len().is_multiple_of(views) {
        return fail(
            ErrorKind::InvalidInput,
            "bearings: expected (P, V, 3) with observed (P, V)",
        );
    }
    poses.iter().try_for_each(se3::check_transform)?;
    let centres: Vec<Vector3<f64>> = poses
        .iter()
        .map(|pose| -se3::rotation(pose).transpose() * se3::translation(pose))
        .collect();
    let rows: Vec<(&[Vector3<f64>], &[bool])> =
        bearings.chunks(views).zip(observed.chunks(views)).collect();
    for (rays, seen) in &rows {
        if seen.iter().filter(|&&s| s).count() < 2 {
            return fail(
                ErrorKind::InvalidInput,
                "observed: every point needs at least two observing views",
            );
        }
        for (ray, &s) in rays.iter().zip(seen.iter()) {
            if s {
                check_bearing(ray, "bearings")?;
            }
        }
    }
    let mut out = Triangulation::default();
    for (rays, mask) in rows {
        let seen: Vec<usize> = (0..views).filter(|&v| mask[v]).collect();
        let track = Track {
            poses,
            centres: &centres,
            bearings: rays,
            seen: &seen,
        };
        let (point, undetermined) = dlt(&track);
        let (in_front, parallax) = cheirality_and_parallax(&track, &point);
        let bad = undetermined || !point.iter().all(|v| v.is_finite());
        out.push(point, in_front, parallax, bad);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests: exact recovery from noise-free rays in both methods, parallel rays, and
    //! the input checks.

    use nalgebra::Vector6;

    use super::*;

    /// Three cameras looking at a point cloud, as `T_cam_world`.
    fn poses() -> Vec<Matrix4<f64>> {
        [-1.0, 0.0, 1.2]
            .iter()
            .map(|&x| se3::exp(&Vector6::new(x, 0.2 * x, 0.0, 0.05 * x, -0.1 * x, 0.0)))
            .collect()
    }

    /// The points.
    fn points() -> Vec<Vector3<f64>> {
        (0..10)
            .map(|i| {
                let t = f64::from(i);
                Vector3::new(t.sin(), (0.5 * t).cos(), 4.0 + 0.3 * t)
            })
            .collect()
    }

    /// A point's unit bearing in a camera.
    fn bearing(pose: &Matrix4<f64>, point: &Vector3<f64>) -> Vector3<f64> {
        (se3::rotation(pose) * point + se3::translation(pose)).normalize()
    }

    /// Noise-free rays meet at the points, in both methods.
    #[test]
    fn noise_free_rays_recover_the_points() {
        let poses = poses();
        let points = points();
        let n = n_view(
            &poses,
            &points
                .iter()
                .flat_map(|p| poses.iter().map(|pose| bearing(pose, p)))
                .collect::<Vec<_>>(),
            &vec![true; points.len() * poses.len()],
        )
        .unwrap();
        for (got, want) in n.points.iter().zip(&points) {
            assert!((got - want).amax() < 1e-10);
        }
        assert!(n.in_front.iter().all(|&f| f));
        // Two-view, in camera 0's frame.
        let relative = se3::compose(&poses[1], &se3::inverse(&poses[0]).unwrap()).unwrap();
        let b1: Vec<_> = points.iter().map(|p| bearing(&poses[0], p)).collect();
        let b2: Vec<_> = points.iter().map(|p| bearing(&poses[1], p)).collect();
        let two = two_view(&relative, &b1, &b2).unwrap();
        for (got, p) in two.points.iter().zip(&points) {
            let want = se3::act(&poses[0], p).unwrap();
            assert!((got - want).amax() < 1e-10);
        }
    }

    /// Parallel rays give NaN, behind, no parallax; a zero translation is rejected.
    #[test]
    fn parallel_rays_and_bad_input() {
        let relative = se3::exp(&Vector6::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        let ray = Vector3::new(0.0, 0.0, 1.0);
        let out = two_view(&relative, &[ray], &[ray]).unwrap();
        assert!(out.points[0].iter().all(|v| v.is_nan()));
        assert!(!out.in_front[0]);
        let still = Matrix4::identity();
        let err = two_view(&still, &[ray], &[ray]).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
        let err = n_view(&poses(), &[ray; 3], &[true, false, false]).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
    }
}
