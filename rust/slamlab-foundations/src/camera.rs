//! Camera models: the Rust core's projection, Jacobians and unprojection.
//!
//! Mirrors `slamlab.foundations.camera`. Points are in the camera frame (z forward); pixels
//! follow OpenCV (top-left pixel centre at (0, 0)); intrinsics are flat slices: pinhole
//! `[fx, fy, cx, cy]`, radtan `[fx, fy, cx, cy, k1, k2, p1, p2, k3]`, radial
//! `[f, cx, cy, k1, k2]`.

use nalgebra::{Matrix2, Matrix2x3, Matrix2xX, Vector2, Vector3};

use crate::error::{ErrorKind, Result, fail};

/// The camera models, with the integer codes they cross the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// `[fx, fy, cx, cy]`.
    Pinhole = 0,
    /// `[fx, fy, cx, cy, k1, k2, p1, p2, k3]`, Brown-Conrady radial-tangential.
    RadTan = 1,
    /// `[f, cx, cy, k1, k2]`, COLMAP's RADIAL and BAL's model: radtan with `fx = fy = f` and
    /// no tangential or k3 terms, computed through radtan.
    Radial = 2,
}

impl Model {
    /// Every model, in code order.
    pub const ALL: [Self; 3] = [Self::Pinhole, Self::RadTan, Self::Radial];

    /// The model's name, as the Python side and the bindings' `codes()` spell it.
    ///
    /// # Returns
    ///
    /// `"pinhole"`, `"radtan"` or `"radial"`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Pinhole => "pinhole",
            Self::RadTan => "radtan",
            Self::Radial => "radial",
        }
    }

    /// The model's focal-length entries in its intrinsics vector
    /// (`slamlab.foundations.camera.FOCAL_INDICES`).
    ///
    /// # Returns
    ///
    /// Their indices: `[0, 1]`, or `[0]` for the radial model.
    #[must_use]
    pub const fn focal_indices(self) -> &'static [usize] {
        match self {
            Self::Pinhole | Self::RadTan => &[0, 1],
            Self::Radial => &[0],
        }
    }

    /// The length of the model's intrinsics.
    ///
    /// # Returns
    ///
    /// 4, 9 or 5.
    #[must_use]
    pub const fn intrinsics_len(self) -> usize {
        match self {
            Self::Pinhole => 4,
            Self::RadTan => 9,
            Self::Radial => 5,
        }
    }
}

/// Points with `Z` at or below this are behind the camera and cannot project.
pub const MIN_DEPTH: f64 = 1e-9;
/// Newton iterations allowed when inverting the radtan distortion.
pub const UNDISTORT_MAX_ITERATIONS: usize = 30;
/// Newton stops when the distortion residual is below this (normalised units).
pub const UNDISTORT_TOLERANCE: f64 = 1e-15;

/// A 2 x (3 + K) Jacobian: d(pixel)/d(point) then d(pixel)/d(intrinsics).
pub type ProjectionJacobian = Matrix2xX<f64>;

/// Radial `[f, cx, cy, k1, k2]` as radtan `[f, f, cx, cy, k1, k2, 0, 0, 0]`.
///
/// # Arguments
///
/// * `k` - Radial intrinsics.
///
/// # Returns
///
/// The radtan intrinsics.
fn radial_to_radtan(k: &[f64]) -> [f64; 9] {
    [k[0], k[0], k[1], k[2], k[3], k[4], 0.0, 0.0, 0.0]
}

/// The terms shared by radtan distortion and its Jacobian.
struct RadialTerms {
    /// Normalised x.
    x: f64,
    /// Normalised y.
    y: f64,
    /// `x² + y²`.
    r2: f64,
    /// The radial factor `1 + k1 r² + k2 r⁴ + k3 r⁶`.
    radial: f64,
}

/// Computes the terms shared by distortion and its Jacobian.
///
/// # Arguments
///
/// * `k` - The intrinsics (radtan layout).
/// * `xy` - The normalised coordinates.
///
/// # Returns
///
/// The terms.
fn radial_terms(k: &[f64], xy: &Vector2<f64>) -> RadialTerms {
    let r2 = xy.x * xy.x + xy.y * xy.y;
    RadialTerms {
        x: xy.x,
        y: xy.y,
        r2,
        radial: 1.0 + r2 * (k[4] + r2 * (k[5] + r2 * k[8])),
    }
}

/// Applies radtan distortion to normalised coordinates.
///
/// # Arguments
///
/// * `k` - The intrinsics (radtan layout).
/// * `xy` - The normalised coordinates.
///
/// # Returns
///
/// The distorted coordinates.
fn distort(k: &[f64], xy: &Vector2<f64>) -> Vector2<f64> {
    let RadialTerms { x, y, r2, radial } = radial_terms(k, xy);
    Vector2::new(
        x * radial + 2.0 * k[6] * x * y + k[7] * (r2 + 2.0 * x * x),
        y * radial + k[6] * (r2 + 2.0 * y * y) + 2.0 * k[7] * x * y,
    )
}

/// The Jacobian of radtan distortion with respect to the normalised coordinates.
///
/// # Arguments
///
/// * `k` - The intrinsics (radtan layout).
/// * `xy` - The normalised coordinates.
///
/// # Returns
///
/// d(distorted)/d(normalised).
fn distortion_jacobian(k: &[f64], xy: &Vector2<f64>) -> Matrix2<f64> {
    let RadialTerms { x, y, r2, radial } = radial_terms(k, xy);
    let d_radial = k[4] + r2 * (2.0 * k[5] + 3.0 * k[8] * r2);
    let cross = 2.0 * x * y * d_radial + 2.0 * k[6] * x + 2.0 * k[7] * y;
    Matrix2::new(
        radial + 2.0 * x * x * d_radial + 2.0 * k[6] * y + 6.0 * k[7] * x,
        cross,
        cross,
        radial + 2.0 * y * y * d_radial + 6.0 * k[6] * y + 2.0 * k[7] * x,
    )
}

/// Whether a distortion residual is within the Newton tolerance.
///
/// # Arguments
///
/// * `residual` - The residual.
/// * `target` - The distorted coordinates being inverted.
///
/// # Returns
///
/// True when converged.
fn converged(residual: &Vector2<f64>, target: &Vector2<f64>) -> bool {
    residual
        .iter()
        .zip(target.iter())
        .all(|(r, t)| r.abs() <= UNDISTORT_TOLERANCE * t.abs().max(1.0))
}

/// Checks a model's intrinsics.
///
/// # Arguments
///
/// * `model` - The model.
/// * `intrinsics` - The intrinsics.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` for the wrong length or a non-positive focal length.
pub fn check_intrinsics(model: Model, intrinsics: &[f64]) -> Result<()> {
    if intrinsics.len() != model.intrinsics_len() {
        return fail(
            ErrorKind::InvalidInput,
            "intrinsics: wrong length for the camera model",
        );
    }
    if model.focal_indices().iter().any(|&i| intrinsics[i] <= 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            "intrinsics: focal lengths must be positive",
        );
    }
    Ok(())
}

/// A camera's intrinsics in the form the projection code computes with: radial expanded to
/// radtan, the others unchanged.
struct Expanded {
    /// Pinhole or radtan.
    model: Model,
    /// The intrinsics in that model's layout (radtan's nine, or pinhole's four then zeros).
    intrinsics: [f64; 9],
}

/// Checks a model's intrinsics and expands radial to radtan.
///
/// # Arguments
///
/// * `model` - The model.
/// * `intrinsics` - The intrinsics.
///
/// # Returns
///
/// The expanded form.
///
/// # Errors
///
/// `InvalidInput` from [`check_intrinsics`].
fn expand(model: Model, intrinsics: &[f64]) -> Result<Expanded> {
    check_intrinsics(model, intrinsics)?;
    if model == Model::Radial {
        return Ok(Expanded {
            model: Model::RadTan,
            intrinsics: radial_to_radtan(intrinsics),
        });
    }
    let mut padded = [0.0; 9];
    padded[..intrinsics.len()].copy_from_slice(intrinsics);
    Ok(Expanded {
        model,
        intrinsics: padded,
    })
}

/// The point on the normalised image plane.
///
/// # Arguments
///
/// * `point` - The point, which must be in front of the camera.
///
/// # Returns
///
/// `(X/Z, Y/Z)`.
///
/// # Errors
///
/// `InvalidInput` if `Z <= MIN_DEPTH`.
fn normalise(point: &Vector3<f64>) -> Result<Vector2<f64>> {
    if point.z <= MIN_DEPTH {
        return fail(
            ErrorKind::InvalidInput,
            "points: Z must exceed 1e-09 (in front of the camera)",
        );
    }
    Ok(point.xy() / point.z)
}

/// Projects a camera-frame point to a pixel.
///
/// # Arguments
///
/// * `model` - The model.
/// * `intrinsics` - The model's intrinsics.
/// * `point` - The point, with `Z > MIN_DEPTH`.
///
/// # Returns
///
/// The pixel `(u, v)`.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics or a point not in front of the camera.
pub fn project(model: Model, intrinsics: &[f64], point: &Vector3<f64>) -> Result<Vector2<f64>> {
    let camera = expand(model, intrinsics)?;
    let k = &camera.intrinsics;
    let xy = normalise(point)?;
    let xd = if camera.model == Model::RadTan {
        distort(k, &xy)
    } else {
        xy
    };
    Ok(Vector2::new(k[0] * xd.x + k[2], k[1] * xd.y + k[3]))
}

/// Differentiates the projection with respect to the point and the intrinsics.
///
/// # Arguments
///
/// * `model` - The model.
/// * `intrinsics` - The model's intrinsics.
/// * `point` - The point, with `Z > MIN_DEPTH`.
///
/// # Returns
///
/// The 2 x (3 + K) Jacobian, K the model's intrinsics length.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics or a point not in front of the camera.
#[rustfmt::skip] // keep one matrix row per line
pub fn project_jacobian(
    model: Model,
    intrinsics: &[f64],
    point: &Vector3<f64>,
) -> Result<ProjectionJacobian> {
    let camera = expand(model, intrinsics)?;
    let k = &camera.intrinsics;
    let xy = normalise(point)?;
    let z = point.z;
    let d_xy_d_point = Matrix2x3::new(
        1.0 / z, 0.0, -xy.x / z,
        0.0, 1.0 / z, -xy.y / z,
    );
    let focal = Matrix2::from_diagonal(&Vector2::new(k[0], k[1]));
    let radtan = camera.model == Model::RadTan;
    let (xd, d_xd_d_xy) = if radtan {
        (distort(k, &xy), distortion_jacobian(k, &xy))
    } else {
        (xy, Matrix2::identity())
    };
    let expanded_len = camera.model.intrinsics_len();
    let mut full = ProjectionJacobian::zeros(3 + expanded_len);
    full.fixed_columns_mut::<3>(0).copy_from(&(focal * d_xd_d_xy * d_xy_d_point));
    full[(0, 3)] = xd.x;
    full[(1, 4)] = xd.y;
    full[(0, 5)] = 1.0;
    full[(1, 6)] = 1.0;
    if radtan {
        let (x, y) = (xy.x, xy.y);
        let r2 = x * x + y * y;
        let d_dist = nalgebra::Matrix2x5::new(
            x * r2, x * r2 * r2, 2.0 * x * y, r2 + 2.0 * x * x, x * r2 * r2 * r2,
            y * r2, y * r2 * r2, r2 + 2.0 * y * y, 2.0 * x * y, y * r2 * r2 * r2,
        );
        full.fixed_columns_mut::<5>(7).copy_from(&(focal * d_dist));
    }
    if model != Model::Radial {
        return Ok(full);
    }
    // Chain through radial -> radtan: f feeds fx and fy; cx, cy, k1, k2 map one to one.
    let mut out = ProjectionJacobian::zeros(3 + 5);
    out.fixed_columns_mut::<3>(0).copy_from(&full.fixed_columns::<3>(0));
    out.set_column(3, &(full.column(3) + full.column(4)));
    for (radial_col, radtan_col) in [(4, 5), (5, 6), (6, 7), (7, 8)] {
        out.set_column(radial_col, &full.column(radtan_col));
    }
    Ok(out)
}

/// Maps a pixel to a unit bearing vector.
///
/// # Arguments
///
/// * `model` - The model.
/// * `intrinsics` - The model's intrinsics.
/// * `pixel` - The pixel `(u, v)`.
///
/// # Returns
///
/// The unit bearing.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics; `NotConverged` if the distortion cannot be inverted
/// there (Newton leaves the invertible region, or does not converge in time).
///
/// # Panics
///
/// Never: a Newton step is taken only where the distortion Jacobian's determinant is
/// positive, so it is invertible.
pub fn unproject(model: Model, intrinsics: &[f64], pixel: &Vector2<f64>) -> Result<Vector3<f64>> {
    let camera = expand(model, intrinsics)?;
    let k = &camera.intrinsics;
    let target = Vector2::new((pixel.x - k[2]) / k[0], (pixel.y - k[3]) / k[1]);
    let mut xy = target;
    if camera.model == Model::RadTan {
        let mut done = false;
        for it in 0..=UNDISTORT_MAX_ITERATIONS {
            let residual = distort(k, &xy) - target;
            if converged(&residual, &target) {
                done = true;
                break;
            }
            if it == UNDISTORT_MAX_ITERATIONS {
                break;
            }
            let jac = distortion_jacobian(k, &xy);
            if jac.determinant() <= 0.0 {
                return fail(
                    ErrorKind::NotConverged,
                    "undistort: pixel outside the invertible region of the distortion",
                );
            }
            let inverse = jac
                .try_inverse()
                .expect("a positive determinant makes the 2x2 invertible");
            xy -= inverse * residual;
        }
        if !done {
            return fail(
                ErrorKind::NotConverged,
                "undistort: Newton did not converge in 30 iterations",
            );
        }
    }
    Ok(Vector3::new(xy.x, xy.y, 1.0).normalize())
}

#[cfg(test)]
mod tests {
    //! Native tests, runnable without Python: round trips through each model, the Jacobian
    //! against central differences, and the error paths.

    use super::*;

    /// Intrinsics for each model, with visible distortion.
    fn cameras() -> [(Model, Vec<f64>); 3] {
        [
            (Model::Pinhole, vec![500.0, 480.0, 320.0, 240.0]),
            (
                Model::RadTan,
                vec![500.0, 480.0, 320.0, 240.0, -0.2, 0.05, 1e-3, -2e-3, 0.01],
            ),
            (Model::Radial, vec![500.0, 320.0, 240.0, -0.2, 0.05]),
        ]
    }

    /// Unprojecting a projected point returns its bearing.
    #[test]
    fn project_unproject_round_trip() {
        let point = Vector3::new(0.3, -0.2, 2.0);
        for (model, k) in cameras() {
            let pixel = project(model, &k, &point).unwrap();
            let bearing = unproject(model, &k, &pixel).unwrap();
            assert!((bearing - point.normalize()).amax() < 1e-14, "{model:?}");
        }
    }

    /// The analytic Jacobian matches central differences in the point and the intrinsics.
    #[test]
    fn jacobian_matches_finite_differences() {
        let point = Vector3::new(0.3, -0.2, 2.0);
        let h = 1e-6;
        for (model, k) in cameras() {
            let analytic = project_jacobian(model, &k, &point).unwrap();
            for c in 0..3 + k.len() {
                let shifted = |sign: f64| {
                    let (mut p, mut kk) = (point, k.clone());
                    if c < 3 {
                        p[c] += sign * h;
                    } else {
                        kk[c - 3] += sign * h;
                    }
                    project(model, &kk, &p).unwrap()
                };
                let numeric = (shifted(1.0) - shifted(-1.0)) / (2.0 * h);
                let scale = analytic.column(c).amax().max(1.0);
                assert!(
                    (analytic.column(c) - numeric).amax() < 1e-6 * scale,
                    "{model:?} {c}"
                );
            }
        }
    }

    /// Bad intrinsics and points behind the camera are `InvalidInput`.
    #[test]
    fn invalid_input_is_rejected() {
        let point = Vector3::new(0.0, 0.0, 1.0);
        let kind = |r: Result<Vector2<f64>>| r.unwrap_err().kind;
        assert_eq!(
            kind(project(Model::Pinhole, &[1.0, 1.0, 0.0], &point)),
            ErrorKind::InvalidInput
        );
        assert_eq!(
            kind(project(Model::Pinhole, &[-1.0, 1.0, 0.0, 0.0], &point)),
            ErrorKind::InvalidInput
        );
        assert_eq!(
            kind(project(Model::Pinhole, &[1.0, 1.0, 0.0, 0.0], &-point)),
            ErrorKind::InvalidInput
        );
    }

    /// A pixel far outside the image of a strongly distorting lens is `NotConverged`.
    #[test]
    fn undistortion_failure_is_not_converged() {
        let k = [500.0, 500.0, 320.0, 240.0, -0.9, 0.0, 0.0, 0.0, 0.0];
        let err = unproject(Model::RadTan, &k, &Vector2::new(2000.0, 1500.0)).unwrap_err();
        assert_eq!(err.kind, ErrorKind::NotConverged);
    }

    /// Each model's discriminant is its position in `ALL`, the code it crosses as.
    #[test]
    fn codes_are_positions() {
        for (i, model) in Model::ALL.iter().enumerate() {
            assert_eq!(*model as usize, i, "{}", model.name());
        }
    }
}
