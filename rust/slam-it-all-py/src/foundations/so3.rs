//! Bindings for `slam_it_all.foundations.so3` (`slam_it_all_rs.foundations.so3`).
//!
//! Every function is batched: float64 arrays of shape (N, ...) in, new arrays out, each item
//! computed by the core in `slam_it_all_foundations::so3`. Core errors become the matching
//! `slam_it_all.foundations.errors` exception.

use nalgebra::{Matrix3, Vector3, Vector4};
use pyo3::prelude::*;
use slam_it_all_foundations::so3 as core;

use crate::common::{add_functions, batched};

batched! {
    /// Maps (N, 3) vectors to (N, 3, 3) skew-symmetric matrices.
    hat(omega: Vector3<f64>) -> Matrix3<f64> = |w| Ok(core::hat(w))
}
batched! {
    /// Maps (N, 3, 3) skew-symmetric matrices to (N, 3) vectors.
    vee(skew: Matrix3<f64>) -> Vector3<f64> = core::vee
}
batched! {
    /// Maps (N, 3) rotation vectors to (N, 3, 3) rotations.
    exp(omega: Vector3<f64>) -> Matrix3<f64> = |w| Ok(core::exp(w))
}
batched! {
    /// Maps (N, 3, 3) rotations to (N, 3) rotation vectors with angle in [0, pi].
    log(rotation: Matrix3<f64>) -> Vector3<f64> = core::log
}
batched! {
    /// Maps (N, 3) vectors to their (N, 3, 3) right Jacobians.
    right_jacobian(omega: Vector3<f64>) -> Matrix3<f64> = |w| Ok(core::right_jacobian(w))
}
batched! {
    /// Maps (N, 3) vectors to their (N, 3, 3) left Jacobians.
    left_jacobian(omega: Vector3<f64>) -> Matrix3<f64> = |w| Ok(core::left_jacobian(w))
}
batched! {
    /// Maps (N, 3) vectors to their inverse right Jacobians.
    right_jacobian_inverse(omega: Vector3<f64>) -> Matrix3<f64> = core::right_jacobian_inverse
}
batched! {
    /// Maps (N, 3) vectors to their inverse left Jacobians.
    left_jacobian_inverse(omega: Vector3<f64>) -> Matrix3<f64> = core::left_jacobian_inverse
}
batched! {
    /// Maps (N, 4) xyzw quaternions to (N, 3, 3) rotations.
    from_quaternion(quaternion: Vector4<f64>) -> Matrix3<f64> = core::from_quaternion
}
batched! {
    /// Maps (N, 3, 3) rotations to (N, 4) quaternions, w >= 0.
    to_quaternion(rotation: Matrix3<f64>) -> Vector4<f64> = core::to_quaternion
}
batched! {
    /// Maps (N, 3, 3) matrices to their nearest rotations.
    project_to_so3(matrix: Matrix3<f64>) -> Matrix3<f64> = |m| Ok(core::project_to_so3(m))
}

/// Registers the `so3` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn so3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m;
        hat, vee, exp, log, right_jacobian, left_jacobian, right_jacobian_inverse,
        left_jacobian_inverse, from_quaternion, to_quaternion, project_to_so3
    );
    Ok(())
}
