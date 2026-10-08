//! SO(3), the group of 3D rotations: the Rust core's implementation.
//!
//! Mirrors `slamlab.foundations.so3`, the Python reference, which the parity tests hold this
//! implementation to, term for term with the C++ core: cancellation-free forms where they
//! exist, Taylor series below the named thresholds, `atan2` for the angle in `log`, and the
//! symmetric-part axis near π. Conventions (design doc, "Conventions"): right (local)
//! perturbations, `R ⊕ δ = R · exp(δ)`; Hamilton quaternions stored `(x, y, z, w)`; `log`
//! returns angles in [0, π].

use std::f64::consts::PI;

use nalgebra::{Matrix3, Vector3, Vector4};

use crate::error::{ErrorKind, Result, fail};
use crate::linalg::svd3;

/// Below this angle, `sin θ / θ` and `θ / sin θ` use their series.
pub const EXP_TAYLOR_THRESHOLD: f64 = 1e-4;
/// Below this angle, the Jacobians' coefficients use their series.
pub const JACOBIAN_TAYLOR_THRESHOLD: f64 = 0.05;
/// Near π (with `cos θ < 0`), `log` takes the axis from the symmetric part below this.
pub const NEAR_PI_SINE: f64 = 1e-3;
/// Accepted deviation of `RᵀR` from the identity, element-wise.
pub const ROTATION_TOLERANCE: f64 = 1e-6;
/// Accepted asymmetry `|W + Wᵀ|`, relative to `max(1, |W|)`.
pub const SKEW_TOLERANCE: f64 = 1e-9;
/// Accepted deviation of a quaternion's norm from 1.
pub const QUATERNION_TOLERANCE: f64 = 1e-6;
/// The inverse Jacobians are singular at `|ω| = 2π`; larger angles are rejected.
pub const INVERSE_JACOBIAN_LIMIT: f64 = 2.0 * PI - 1e-6;

/// `sin θ / θ`, with its series below [`EXP_TAYLOR_THRESHOLD`].
///
/// # Arguments
///
/// * `theta` - The angle, `>= 0`.
///
/// # Returns
///
/// The coefficient.
fn sinc(theta: f64) -> f64 {
    if theta < EXP_TAYLOR_THRESHOLD {
        1.0 - theta * theta / 6.0
    } else {
        theta.sin() / theta
    }
}

/// `(1 - cos θ) / θ²` as `½ (sin(θ/2) / (θ/2))²`: no cancellation at small θ.
///
/// # Arguments
///
/// * `theta` - The angle, `>= 0`.
///
/// # Returns
///
/// The coefficient.
fn half_versine_coefficient(theta: f64) -> f64 {
    let s = sinc(0.5 * theta);
    0.5 * s * s
}

/// `(θ - sin θ) / θ³`, with its series below [`JACOBIAN_TAYLOR_THRESHOLD`].
///
/// # Arguments
///
/// * `theta` - The angle, `>= 0`.
///
/// # Returns
///
/// The coefficient.
fn third_coefficient(theta: f64) -> f64 {
    if theta < JACOBIAN_TAYLOR_THRESHOLD {
        let t2 = theta * theta;
        1.0 / 6.0 - t2 / 120.0 + t2 * t2 / 5040.0 - t2 * t2 * t2 / 362_880.0
    } else {
        (theta - theta.sin()) / (theta * theta * theta)
    }
}

/// `1/θ² - cot(θ/2) / (2θ)`, with its series below [`JACOBIAN_TAYLOR_THRESHOLD`].
///
/// # Arguments
///
/// * `theta` - The angle, in `[0, 2π)`.
///
/// # Returns
///
/// The coefficient.
fn inverse_coefficient(theta: f64) -> f64 {
    if theta < JACOBIAN_TAYLOR_THRESHOLD {
        let t2 = theta * theta;
        1.0 / 12.0 + t2 / 720.0 + t2 * t2 / 30_240.0 + t2 * t2 * t2 / 1_209_600.0
    } else {
        1.0 / (theta * theta) - 1.0 / (2.0 * theta * (0.5 * theta).tan())
    }
}

/// `I + sign · B W + C W²`: the left (sign +1) or right (sign -1) Jacobian.
///
/// # Arguments
///
/// * `omega` - A rotation vector.
/// * `sign` - +1 for the left Jacobian, -1 for the right.
///
/// # Returns
///
/// The Jacobian.
fn jacobian(omega: &Vector3<f64>, sign: f64) -> Matrix3<f64> {
    let theta = omega.norm();
    let w = hat(omega);
    Matrix3::identity()
        + w * (sign * half_versine_coefficient(theta))
        + w * w * third_coefficient(theta)
}

/// `I - sign · ½ W + D W²`: the left (sign +1) or right (sign -1) inverse Jacobian.
///
/// # Arguments
///
/// * `omega` - A rotation vector with `|ω| <` [`INVERSE_JACOBIAN_LIMIT`].
/// * `sign` - +1 for the left inverse Jacobian, -1 for the right.
///
/// # Returns
///
/// The inverse Jacobian.
///
/// # Errors
///
/// `InvalidInput` beyond the limit.
fn jacobian_inverse(omega: &Vector3<f64>, sign: f64) -> Result<Matrix3<f64>> {
    let theta = omega.norm();
    if theta >= INVERSE_JACOBIAN_LIMIT {
        return fail(
            ErrorKind::InvalidInput,
            "omega: inverse Jacobian undefined at |omega| >= 2*pi",
        );
    }
    let w = hat(omega);
    Ok(Matrix3::identity() - w * (sign * 0.5) + w * w * inverse_coefficient(theta))
}

/// The rotation axis near θ = π, from `(R + Rᵀ)/2 - cos θ · I = (1 - cos θ) n nᵀ`.
///
/// The sign follows `v = sin θ · n`; when that vanishes the largest-magnitude component is
/// made positive, matching the Python reference.
///
/// # Arguments
///
/// * `rotation` - The rotation.
/// * `cosine` - `cos θ`, clamped to [-1, 1].
/// * `v` - The skew part `sin θ · n`.
///
/// # Returns
///
/// The unit axis.
fn axis_near_pi(rotation: &Matrix3<f64>, cosine: f64, v: &Vector3<f64>) -> Vector3<f64> {
    let outer = (rotation + rotation.transpose()) * 0.5 - Matrix3::identity() * cosine;
    let k = outer.diagonal().imax();
    let mut axis: Vector3<f64> = outer.column(k) / (outer[(k, k)] * (1.0 - cosine)).sqrt();
    axis.normalize_mut();
    let alignment = axis.dot(v);
    let largest = axis.iamax();
    if alignment < 0.0 || (alignment == 0.0 && axis[largest] < 0.0) {
        axis = -axis;
    }
    axis
}

/// Maps a vector to its skew-symmetric matrix, so that `hat(ω) u == ω × u`.
///
/// It is the map from R^3 into the Lie algebra so(3). Exact: it only copies and negates
/// entries.
///
/// # Arguments
///
/// * `omega` - A vector in R^3, the tangent space of SO(3).
///
/// # Returns
///
/// The skew-symmetric matrix `[ω]x`.
///
/// # Examples
///
/// ```
/// use nalgebra::{Matrix3, Vector3};
/// use slamlab_foundations::so3::hat;
///
/// let expected = Matrix3::new(0.0, -3.0, 2.0, 3.0, 0.0, -1.0, -2.0, 1.0, 0.0);
/// assert_eq!(hat(&Vector3::new(1.0, 2.0, 3.0)), expected);
/// ```
#[must_use]
#[rustfmt::skip] // keep one matrix row per line
pub fn hat(omega: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -omega.z, omega.y,
        omega.z, 0.0, -omega.x,
        -omega.y, omega.x, 0.0,
    )
}

/// Maps a skew-symmetric matrix back to its vector; the inverse of [`hat`].
///
/// # Arguments
///
/// * `skew` - A skew-symmetric matrix (within [`SKEW_TOLERANCE`]).
///
/// # Returns
///
/// The vector `ω` with `hat(ω) == skew`.
///
/// # Errors
///
/// `InvalidInput` if `skew` has a non-finite entry or is not skew-symmetric.
pub fn vee(skew: &Matrix3<f64>) -> Result<Vector3<f64>> {
    if !skew.iter().all(|x| x.is_finite()) {
        return fail(ErrorKind::InvalidInput, "skew: expected finite values");
    }
    let scale = skew.amax().max(1.0);
    if (skew + skew.transpose()).amax() > SKEW_TOLERANCE * scale {
        return fail(
            ErrorKind::InvalidInput,
            "skew: not skew-symmetric within 1e-09",
        );
    }
    Ok(Vector3::new(skew[(2, 1)], skew[(0, 2)], skew[(1, 0)]))
}

/// Maps a rotation vector to a rotation matrix (Rodrigues' formula).
///
/// # Arguments
///
/// * `omega` - A rotation vector: axis times angle in radians (any angle).
///
/// # Returns
///
/// The rotation matrix `exp(hat(ω))`.
#[must_use]
pub fn exp(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    let w = hat(omega);
    Matrix3::identity() + w * sinc(theta) + w * w * half_versine_coefficient(theta)
}

/// Checks that a matrix is a rotation within [`ROTATION_TOLERANCE`].
///
/// # Arguments
///
/// * `rotation` - The matrix to check.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming what failed: a non-finite entry, not orthonormal, or a reflection.
pub fn check_rotation(rotation: &Matrix3<f64>) -> Result<()> {
    if !rotation.iter().all(|x| x.is_finite()) {
        return fail(ErrorKind::InvalidInput, "rotation: expected finite values");
    }
    let gram = rotation.transpose() * rotation;
    if (gram - Matrix3::identity()).amax() > ROTATION_TOLERANCE {
        return fail(
            ErrorKind::InvalidInput,
            "rotation: not orthonormal within 1e-06; use project_to_so3 to repair it explicitly",
        );
    }
    if rotation.determinant() <= 0.0 {
        return fail(
            ErrorKind::InvalidInput,
            "rotation: determinant is not positive (a reflection)",
        );
    }
    Ok(())
}

/// Maps a rotation matrix to its rotation vector, with angle in [0, π].
///
/// At exactly θ = π the axis is ambiguous up to sign; the vector whose largest-magnitude
/// component is positive is returned, as in every backend.
///
/// # Arguments
///
/// * `rotation` - A rotation matrix (orthonormal within [`ROTATION_TOLERANCE`], det > 0).
///
/// # Returns
///
/// The rotation vector.
///
/// # Errors
///
/// `InvalidInput` if `rotation` is not a rotation.
pub fn log(rotation: &Matrix3<f64>) -> Result<Vector3<f64>> {
    check_rotation(rotation)?;
    let r = rotation;
    let v = Vector3::new(
        0.5 * (r[(2, 1)] - r[(1, 2)]),
        0.5 * (r[(0, 2)] - r[(2, 0)]),
        0.5 * (r[(1, 0)] - r[(0, 1)]),
    );
    let sine = v.norm();
    let cosine = (0.5 * (r.trace() - 1.0)).clamp(-1.0, 1.0);
    let theta = sine.atan2(cosine);
    if cosine < 0.0 && sine < NEAR_PI_SINE {
        return Ok(axis_near_pi(r, cosine, &v) * theta);
    }
    let factor = if theta < EXP_TAYLOR_THRESHOLD {
        1.0 + theta * theta / 6.0
    } else {
        theta / sine
    };
    Ok(v * factor)
}

/// The right Jacobian `Jr(ω)`, with `exp(ω + δ) ≈ exp(ω) exp(Jr(ω) δ)`.
///
/// # Arguments
///
/// * `omega` - A rotation vector.
///
/// # Returns
///
/// `I - (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²`, with `W = hat(ω)`.
#[must_use]
pub fn right_jacobian(omega: &Vector3<f64>) -> Matrix3<f64> {
    jacobian(omega, -1.0)
}

/// The left Jacobian `Jl(ω) = Jr(-ω)`.
///
/// # Arguments
///
/// * `omega` - A rotation vector.
///
/// # Returns
///
/// `I + (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²`.
#[must_use]
pub fn left_jacobian(omega: &Vector3<f64>) -> Matrix3<f64> {
    jacobian(omega, 1.0)
}

/// The inverse of the right Jacobian, defined for `|ω| < 2π`.
///
/// # Arguments
///
/// * `omega` - A rotation vector with norm below [`INVERSE_JACOBIAN_LIMIT`].
///
/// # Returns
///
/// `I + ½ W + (1/θ² - cot(θ/2)/(2θ)) W²`.
///
/// # Errors
///
/// `InvalidInput` if `|ω| >= 2π`.
pub fn right_jacobian_inverse(omega: &Vector3<f64>) -> Result<Matrix3<f64>> {
    jacobian_inverse(omega, -1.0)
}

/// The inverse of the left Jacobian, defined for `|ω| < 2π`.
///
/// # Arguments
///
/// * `omega` - A rotation vector with norm below [`INVERSE_JACOBIAN_LIMIT`].
///
/// # Returns
///
/// `I - ½ W + (1/θ² - cot(θ/2)/(2θ)) W²`.
///
/// # Errors
///
/// `InvalidInput` if `|ω| >= 2π`.
pub fn left_jacobian_inverse(omega: &Vector3<f64>) -> Result<Matrix3<f64>> {
    jacobian_inverse(omega, 1.0)
}

/// Maps a unit quaternion (Hamilton, stored xyzw) to a rotation matrix.
///
/// # Arguments
///
/// * `quaternion` - `(x, y, z, w)` with norm within [`QUATERNION_TOLERANCE`] of 1; it is
///   normalised before use.
///
/// # Returns
///
/// The rotation matrix.
///
/// # Errors
///
/// `InvalidInput` for a non-finite entry, or a norm not close to 1.
#[rustfmt::skip] // keep one matrix row per line
pub fn from_quaternion(quaternion: &Vector4<f64>) -> Result<Matrix3<f64>> {
    if !quaternion.iter().all(|x| x.is_finite()) {
        return fail(ErrorKind::InvalidInput, "quaternion: expected finite values");
    }
    let norm = quaternion.norm();
    if (norm - 1.0).abs() > QUATERNION_TOLERANCE {
        return fail(ErrorKind::InvalidInput, "quaternion: norm not within 1e-06 of 1");
    }
    let unit = quaternion / norm;
    let (x, y, z, w) = (unit[0], unit[1], unit[2], unit[3]);
    Ok(Matrix3::new(
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
        2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
        2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y),
    ))
}

/// Maps a rotation matrix to a unit quaternion (xyzw), canonically `w >= 0`.
///
/// Shepperd's method: the largest of the trace and the diagonal picks the stable branch.
///
/// # Arguments
///
/// * `rotation` - A rotation matrix (orthonormal within [`ROTATION_TOLERANCE`], det > 0).
///
/// # Returns
///
/// `(x, y, z, w)` with `w >= 0`.
///
/// # Errors
///
/// `InvalidInput` if `rotation` is not a rotation.
pub fn to_quaternion(rotation: &Matrix3<f64>) -> Result<Vector4<f64>> {
    check_rotation(rotation)?;
    let r = rotation;
    let trace = r.trace();
    let branch = Vector4::new(trace, r[(0, 0)], r[(1, 1)], r[(2, 2)]).imax();
    let (s, q) = match branch {
        0 => {
            let s = 2.0 * (1.0 + trace).sqrt();
            let q = Vector4::new(
                r[(2, 1)] - r[(1, 2)],
                r[(0, 2)] - r[(2, 0)],
                r[(1, 0)] - r[(0, 1)],
                0.25 * s * s,
            );
            (s, q)
        }
        1 => {
            let s = 2.0 * (1.0 + r[(0, 0)] - r[(1, 1)] - r[(2, 2)]).sqrt();
            let q = Vector4::new(
                0.25 * s * s,
                r[(0, 1)] + r[(1, 0)],
                r[(0, 2)] + r[(2, 0)],
                r[(2, 1)] - r[(1, 2)],
            );
            (s, q)
        }
        2 => {
            let s = 2.0 * (1.0 - r[(0, 0)] + r[(1, 1)] - r[(2, 2)]).sqrt();
            let q = Vector4::new(
                r[(0, 1)] + r[(1, 0)],
                0.25 * s * s,
                r[(1, 2)] + r[(2, 1)],
                r[(0, 2)] - r[(2, 0)],
            );
            (s, q)
        }
        _ => {
            let s = 2.0 * (1.0 - r[(0, 0)] - r[(1, 1)] + r[(2, 2)]).sqrt();
            let q = Vector4::new(
                r[(0, 2)] + r[(2, 0)],
                r[(1, 2)] + r[(2, 1)],
                0.25 * s * s,
                r[(1, 0)] - r[(0, 1)],
            );
            (s, q)
        }
    };
    let q = q / s;
    Ok(if q[3] < 0.0 { -q } else { q })
}

/// The nearest rotation to a matrix in the Frobenius norm (via an SVD).
///
/// # Arguments
///
/// * `matrix` - Any real 3x3 matrix.
///
/// # Returns
///
/// `U diag(1, 1, det(U Vᵀ)) Vᵀ` for `matrix = U S Vᵀ`; NaN for a non-finite matrix (or one
/// the 3x3 SVD gives up on, never observed).
#[must_use]
pub fn project_to_so3(matrix: &Matrix3<f64>) -> Matrix3<f64> {
    let Some((mut u, _, v_t)) = svd3(matrix) else {
        return Matrix3::repeat(f64::NAN);
    };
    if (u * v_t).determinant() < 0.0 {
        u.column_mut(2).neg_mut();
    }
    u * v_t
}

#[cfg(test)]
mod tests {
    //! Native tests, runnable without Python.
    //!
    //! The parity fixtures (`tests/fixtures/foundations/`) are the main check of every value;
    //! these cover what the standalone core must get right on its own: exact `hat` entries,
    //! round trips, the θ = π convention, and the error paths.
    #![allow(
        clippy::float_cmp,
        reason = "hat only copies and negates, so exact comparison is the correct check"
    )]

    use super::*;
    use crate::error::Error;

    /// The basis vectors fix all nine entries of `hat`, exactly.
    #[test]
    #[rustfmt::skip] // keep one matrix row per line
    fn hat_of_basis_vectors() {
        assert_eq!(hat(&Vector3::x()), Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0));
        assert_eq!(hat(&Vector3::y()), Matrix3::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0));
        assert_eq!(hat(&Vector3::z()), Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    /// `log(exp(ω)) == ω` across angles, including both sides of the Taylor switch.
    #[test]
    fn exp_log_round_trip() {
        let axis = Vector3::new(0.3, -0.5, 0.8).normalize();
        for theta in [
            0.0,
            1e-12,
            1e-4 * (1.0 - 1e-9),
            1e-4 * (1.0 + 1e-9),
            0.5,
            3.0,
        ] {
            let omega = axis * theta;
            let back = log(&exp(&omega)).unwrap();
            assert!((back - omega).amax() < 1e-14, "theta = {theta}");
        }
    }

    /// At exactly θ = π the largest-magnitude component of the result is positive.
    #[test]
    fn log_at_pi_is_canonical() {
        let r_pi = Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, -1.0));
        let omega = log(&r_pi).unwrap();
        assert!((omega - Vector3::new(PI, 0.0, 0.0)).amax() < 1e-15);
    }

    /// `Jr⁻¹ · Jr = I`.
    #[test]
    fn inverse_right_jacobian_inverts() {
        let w = Vector3::new(0.4, -1.1, 0.7);
        let product = right_jacobian_inverse(&w).unwrap() * right_jacobian(&w);
        assert!((product - Matrix3::identity()).amax() < 1e-14);
    }

    /// A quaternion round trip returns the rotation, with `w >= 0`.
    #[test]
    fn quaternion_round_trip() {
        let r = exp(&Vector3::new(-2.0, 0.4, 1.1));
        let q = to_quaternion(&r).unwrap();
        assert!(q[3] >= 0.0);
        assert!((from_quaternion(&q).unwrap() - r).amax() < 1e-15);
    }

    /// The projection repairs a perturbed rotation, and returns a rotation (not a
    /// reflection) from a reflection.
    #[test]
    fn projection_returns_a_rotation() {
        let r = exp(&Vector3::new(0.2, 0.9, -0.3));
        assert!(check_rotation(&project_to_so3(&(r + Matrix3::repeat(1e-3)))).is_ok());
        assert!(check_rotation(&project_to_so3(&-r)).is_ok());
    }

    /// The error paths return `InvalidInput` rather than garbage.
    #[test]
    fn invalid_input_is_rejected() {
        let invalid = |e: Error| e.kind == ErrorKind::InvalidInput;
        assert!(invalid(log(&Matrix3::repeat(0.5)).unwrap_err()));
        assert!(invalid(log(&-Matrix3::identity()).unwrap_err()));
        assert!(invalid(vee(&Matrix3::identity()).unwrap_err()));
        assert!(invalid(
            from_quaternion(&Vector4::new(1.0, 1.0, 0.0, 0.0)).unwrap_err()
        ));
        assert!(invalid(
            right_jacobian_inverse(&Vector3::new(7.0, 0.0, 0.0)).unwrap_err()
        ));
        // NaN passes every tolerance comparison, so finiteness is checked first.
        let mut nan = Matrix3::identity();
        nan[(0, 1)] = f64::NAN;
        assert!(invalid(log(&nan).unwrap_err()));
        assert!(invalid(vee(&nan).unwrap_err()));
        let nan_quaternion = Vector4::new(f64::NAN, 0.0, 0.0, 1.0);
        assert!(invalid(from_quaternion(&nan_quaternion).unwrap_err()));
    }
}
