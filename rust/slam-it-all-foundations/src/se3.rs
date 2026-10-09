//! SE(3), rigid-body transforms: the Rust core's implementation.
//!
//! Mirrors `slam_it_all.foundations.se3`. A transform is a 4x4 matrix `[[R, t], [0, 1]]`; the
//! tangent is `ξ = (ρ, θ)`, translation first; perturbations are right (local). The Jacobians
//! use Barfoot's block form with `Q(ρ, θ)`, whose coefficients come from their power series
//! below [`SERIES_THRESHOLD`] (design note `docs/notes/lie-groups.md`).

use nalgebra::{Matrix3, Matrix4, Matrix6, RowVector4, Vector3, Vector6};

use crate::error::{ErrorKind, Result, fail};
use crate::so3;

/// Below this rotation angle, `Q`'s coefficients come from their power series.
pub const SERIES_THRESHOLD: f64 = 1.0;
/// Terms summed for each series.
pub const SERIES_TERMS: u32 = 10;
/// A transform's last row must equal `[0, 0, 0, 1]` within this.
pub const BOTTOM_ROW_TOLERANCE: f64 = 1e-12;

/// The rotation block of a transform.
///
/// # Arguments
///
/// * `transform` - A 4x4 transform.
///
/// # Returns
///
/// `R`, its top-left 3x3 block.
#[must_use]
pub fn rotation(transform: &Matrix4<f64>) -> Matrix3<f64> {
    transform.fixed_view::<3, 3>(0, 0).into_owned()
}

/// The translation of a transform.
///
/// # Arguments
///
/// * `transform` - A 4x4 transform.
///
/// # Returns
///
/// `t`, its top-right 3x1 block.
#[must_use]
pub fn translation(transform: &Matrix4<f64>) -> Vector3<f64> {
    transform.fixed_view::<3, 1>(0, 3).into_owned()
}

/// Assembles a transform from its blocks.
///
/// # Arguments
///
/// * `rotation` - `R`.
/// * `translation` - `t`.
///
/// # Returns
///
/// `[[R, t], [0, 1]]`, with the last row exact.
#[must_use]
pub fn from_parts(rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> Matrix4<f64> {
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(rotation);
    out.fixed_view_mut::<3, 1>(0, 3).copy_from(translation);
    out
}

/// The three coefficients of `Q`: series below the threshold, closed form above.
///
/// # Arguments
///
/// * `theta` - The rotation angle, `>= 0`.
///
/// # Returns
///
/// `(θ - sin θ)/θ³`, `(θ² + 2 cos θ - 2)/(2θ⁴)`, `(2θ - 3 sin θ + θ cos θ)/(2θ⁵)`.
fn q_coefficients(theta: f64) -> [f64; 3] {
    if theta < SERIES_THRESHOLD {
        let t2 = theta * theta;
        let mut sum = [0.0; 3];
        let mut power = 1.0;
        // Each coefficient is Σ_k (-1)^k θ^{2k} c_k with c_k = 1/(2k+3)!, 1/(2k+4)!,
        // (k+1)/(2k+5)!; the factorials are built incrementally.
        let (mut f3, mut f4, mut f5) = (6.0, 24.0, 120.0);
        for k in 0..SERIES_TERMS {
            let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
            let kk = f64::from(k);
            sum[0] += sign * power / f3;
            sum[1] += sign * power / f4;
            sum[2] += sign * power * (kk + 1.0) / f5;
            power *= t2;
            f3 *= (2.0 * kk + 4.0) * (2.0 * kk + 5.0);
            f4 *= (2.0 * kk + 5.0) * (2.0 * kk + 6.0);
            f5 *= (2.0 * kk + 6.0) * (2.0 * kk + 7.0);
        }
        return sum;
    }
    let (s, c) = theta.sin_cos();
    [
        (theta - s) / (theta * theta * theta),
        (theta * theta + 2.0 * c - 2.0) / (2.0 * theta.powi(4)),
        (2.0 * theta - 3.0 * s + theta * c) / (2.0 * theta.powi(5)),
    ]
}

/// The translation part `ρ` of a twist.
///
/// # Arguments
///
/// * `xi` - The twist `(ρ, θ)`.
///
/// # Returns
///
/// `ρ`.
fn rho(xi: &Vector6<f64>) -> Vector3<f64> {
    xi.fixed_rows::<3>(0).into_owned()
}

/// The rotation part `θ` of a twist.
///
/// # Arguments
///
/// * `xi` - The twist `(ρ, θ)`.
///
/// # Returns
///
/// `θ`.
fn phi(xi: &Vector6<f64>) -> Vector3<f64> {
    xi.fixed_rows::<3>(3).into_owned()
}

/// Barfoot's `Q(ρ, θ)`, the coupling block of the left Jacobian.
///
/// # Arguments
///
/// * `xi` - The twist.
///
/// # Returns
///
/// `Q`.
fn q_matrix(xi: &Vector6<f64>) -> Matrix3<f64> {
    let r = so3::hat(&rho(xi));
    let p = so3::hat(&phi(xi));
    let [c1, c2, c3] = q_coefficients(phi(xi).norm());
    let pr = p * r;
    let rp = r * p;
    let prp = p * r * p;
    r * 0.5 + (pr + rp + prp) * c1 + (p * pr + rp * p - prp * 3.0) * c2 + (prp * p + p * prp) * c3
}

/// A 6x6 matrix with the same 3x3 block on its diagonal and `upper` above it.
///
/// # Arguments
///
/// * `diagonal` - The two diagonal blocks.
/// * `upper` - The top-right block.
///
/// # Returns
///
/// `[[diagonal, upper], [0, diagonal]]`.
fn block_upper(diagonal: &Matrix3<f64>, upper: &Matrix3<f64>) -> Matrix6<f64> {
    let mut out = Matrix6::zeros();
    out.fixed_view_mut::<3, 3>(0, 0).copy_from(diagonal);
    out.fixed_view_mut::<3, 3>(3, 3).copy_from(diagonal);
    out.fixed_view_mut::<3, 3>(0, 3).copy_from(upper);
    out
}

/// Checks that a matrix is a rigid transform.
///
/// # Arguments
///
/// * `transform` - The matrix to check.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming what failed: a non-finite entry, the last row, or the rotation block.
pub fn check_transform(transform: &Matrix4<f64>) -> Result<()> {
    if !transform.iter().all(|x| x.is_finite()) {
        return fail(ErrorKind::InvalidInput, "transform: expected finite values");
    }
    let bottom = RowVector4::new(0.0, 0.0, 0.0, 1.0);
    if (transform.row(3) - bottom).amax() > BOTTOM_ROW_TOLERANCE {
        return fail(
            ErrorKind::InvalidInput,
            "transform: last row is not [0, 0, 0, 1]",
        );
    }
    so3::check_rotation(&rotation(transform))
}

/// Maps a twist to a rigid transform: `[[exp(θ), Jl(θ) ρ], [0, 1]]`.
///
/// # Arguments
///
/// * `xi` - The twist `(ρ, θ)`.
///
/// # Returns
///
/// The transform.
#[must_use]
pub fn exp(xi: &Vector6<f64>) -> Matrix4<f64> {
    let theta = phi(xi);
    from_parts(&so3::exp(&theta), &(so3::left_jacobian(&theta) * rho(xi)))
}

/// Maps a rigid transform to its twist, rotation angle in [0, π].
///
/// # Arguments
///
/// * `transform` - A rigid transform.
///
/// # Returns
///
/// The twist `(ρ, θ)`.
///
/// # Errors
///
/// `InvalidInput` if `transform` is not a rigid transform.
pub fn log(transform: &Matrix4<f64>) -> Result<Vector6<f64>> {
    check_transform(transform)?;
    let theta = so3::log(&rotation(transform))?;
    let rho = so3::left_jacobian_inverse(&theta)? * translation(transform);
    Ok(Vector6::new(rho.x, rho.y, rho.z, theta.x, theta.y, theta.z))
}

/// Inverts a rigid transform: `[[Rᵀ, -Rᵀ t], [0, 1]]`.
///
/// # Arguments
///
/// * `transform` - A rigid transform.
///
/// # Returns
///
/// The inverse.
///
/// # Errors
///
/// `InvalidInput` if `transform` is not a rigid transform.
pub fn inverse(transform: &Matrix4<f64>) -> Result<Matrix4<f64>> {
    check_transform(transform)?;
    let rt = rotation(transform).transpose();
    Ok(from_parts(&rt, &(-rt * translation(transform))))
}

/// Composes rigid transforms, `a · b`, keeping the last row exact.
///
/// # Arguments
///
/// * `a` - The left transform.
/// * `b` - The right transform.
///
/// # Returns
///
/// The product.
///
/// # Errors
///
/// `InvalidInput` if either is not a rigid transform.
pub fn compose(a: &Matrix4<f64>, b: &Matrix4<f64>) -> Result<Matrix4<f64>> {
    check_transform(a)?;
    check_transform(b)?;
    let ra = rotation(a);
    Ok(from_parts(
        &(ra * rotation(b)),
        &(ra * translation(b) + translation(a)),
    ))
}

/// Applies a rigid transform to a point: `R p + t`.
///
/// # Arguments
///
/// * `transform` - A rigid transform.
/// * `point` - The point.
///
/// # Returns
///
/// The transformed point.
///
/// # Errors
///
/// `InvalidInput` if `transform` is not rigid.
pub fn act(transform: &Matrix4<f64>, point: &Vector3<f64>) -> Result<Vector3<f64>> {
    check_transform(transform)?;
    Ok(rotation(transform) * point + translation(transform))
}

/// The adjoint `[[R, hat(t) R], [0, R]]`, with `T exp(ξ) T⁻¹ = exp(Ad_T ξ)`.
///
/// # Arguments
///
/// * `transform` - A rigid transform.
///
/// # Returns
///
/// The adjoint.
///
/// # Errors
///
/// `InvalidInput` if `transform` is not rigid.
pub fn adjoint(transform: &Matrix4<f64>) -> Result<Matrix6<f64>> {
    check_transform(transform)?;
    let r = rotation(transform);
    Ok(block_upper(&r, &(so3::hat(&translation(transform)) * r)))
}

/// The left Jacobian `[[Jl(θ), Q(ρ, θ)], [0, Jl(θ)]]`.
///
/// # Arguments
///
/// * `xi` - The twist.
///
/// # Returns
///
/// The left Jacobian.
#[must_use]
pub fn left_jacobian(xi: &Vector6<f64>) -> Matrix6<f64> {
    block_upper(&so3::left_jacobian(&phi(xi)), &q_matrix(xi))
}

/// The right Jacobian `Jr(ξ) = Jl(-ξ)`.
///
/// # Arguments
///
/// * `xi` - The twist.
///
/// # Returns
///
/// The right Jacobian.
#[must_use]
pub fn right_jacobian(xi: &Vector6<f64>) -> Matrix6<f64> {
    left_jacobian(&-xi)
}

/// The inverse left Jacobian `[[J⁻¹, -J⁻¹ Q J⁻¹], [0, J⁻¹]]`, for `|θ| < 2π`.
///
/// # Arguments
///
/// * `xi` - The twist.
///
/// # Returns
///
/// The inverse.
///
/// # Errors
///
/// `InvalidInput` if `|θ| >= 2π`.
pub fn left_jacobian_inverse(xi: &Vector6<f64>) -> Result<Matrix6<f64>> {
    let jl_inv = so3::left_jacobian_inverse(&phi(xi))?;
    Ok(block_upper(&jl_inv, &(-jl_inv * q_matrix(xi) * jl_inv)))
}

/// The inverse right Jacobian `Jl(-ξ)⁻¹`, for `|θ| < 2π`.
///
/// # Arguments
///
/// * `xi` - The twist.
///
/// # Returns
///
/// The inverse.
///
/// # Errors
///
/// `InvalidInput` if `|θ| >= 2π`.
pub fn right_jacobian_inverse(xi: &Vector6<f64>) -> Result<Matrix6<f64>> {
    left_jacobian_inverse(&-xi)
}

#[cfg(test)]
mod tests {
    //! Native tests, runnable without Python: round trips, the group laws, the Jacobian
    //! identities on both sides of the series switch, and the error paths.

    use super::*;

    /// Twists on both sides of the series threshold.
    fn twists() -> [Vector6<f64>; 3] {
        [
            Vector6::new(0.3, -1.2, 2.0, 1e-9, 0.0, 0.0),
            Vector6::new(0.3, -1.2, 2.0, 0.2, -0.5, 0.4),
            Vector6::new(-4.0, 0.5, 1.0, 1.1, 2.0, -0.7),
        ]
    }

    /// `log(exp(ξ)) == ξ`.
    #[test]
    fn exp_log_round_trip() {
        for xi in twists() {
            assert!((log(&exp(&xi)).unwrap() - xi).amax() < 1e-13);
        }
    }

    /// `T · T⁻¹ = I`, and `act` agrees with the matrix product.
    #[test]
    fn group_laws() {
        let t = exp(&twists()[2]);
        let product = compose(&t, &inverse(&t).unwrap()).unwrap();
        assert!((product - Matrix4::identity()).amax() < 1e-14);
        let p = Vector3::new(1.0, -2.0, 0.5);
        let expected = (t * p.push(1.0)).xyz();
        assert!((act(&t, &p).unwrap() - expected).amax() < 1e-14);
    }

    /// `Jr⁻¹ · Jr = I` and `Jl(ξ) = Ad(exp ξ) Jr(ξ)`.
    #[test]
    fn jacobian_identities() {
        for xi in twists() {
            let jr = right_jacobian(&xi);
            let inv = right_jacobian_inverse(&xi).unwrap();
            assert!((inv * jr - Matrix6::identity()).amax() < 1e-12);
            let ad = adjoint(&exp(&xi)).unwrap();
            assert!((left_jacobian(&xi) - ad * jr).amax() < 1e-12);
        }
    }

    /// A non-rigid matrix and an angle at 2π are `InvalidInput`.
    #[test]
    fn invalid_input_is_rejected() {
        let mut skewed = Matrix4::identity();
        skewed[(3, 0)] = 1e-3;
        assert_eq!(log(&skewed).unwrap_err().kind, ErrorKind::InvalidInput);
        let mut nan = Matrix4::identity();
        nan[(1, 3)] = f64::NAN;
        assert_eq!(inverse(&nan).unwrap_err().kind, ErrorKind::InvalidInput);
        let full_turn = Vector6::new(0.0, 0.0, 0.0, 7.0, 0.0, 0.0);
        assert_eq!(
            left_jacobian_inverse(&full_turn).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
    }
}
