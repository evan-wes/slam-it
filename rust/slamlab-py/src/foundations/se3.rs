//! Bindings for `slamlab.foundations.se3` (`slamlab_rs.foundations.se3`).
//!
//! Every function is batched: float64 arrays of shape (N, ...) in, new arrays out, each item
//! computed by the core in `slamlab_foundations::se3`.

use nalgebra::{Matrix4, Matrix6, Vector3, Vector6};
use pyo3::prelude::*;
use slamlab_foundations::se3 as core;

use crate::common::{add_functions, batched};

batched! {
    /// Maps (N, 6) twists (rho, theta) to (N, 4, 4) transforms.
    exp(xi: Vector6<f64>) -> Matrix4<f64> = |x| Ok(core::exp(x))
}
batched! {
    /// Maps (N, 4, 4) transforms to (N, 6) twists.
    log(transform: Matrix4<f64>) -> Vector6<f64> = core::log
}
batched! {
    /// Inverts (N, 4, 4) transforms.
    inverse(transform: Matrix4<f64>) -> Matrix4<f64> = core::inverse
}
batched! {
    /// Composes (N, 4, 4) transforms.
    compose(a: Matrix4<f64>, b: Matrix4<f64>) -> Matrix4<f64> = core::compose
}
batched! {
    /// Applies (N, 4, 4) transforms to (N, 3) points.
    act(transform: Matrix4<f64>, points: Vector3<f64>) -> Vector3<f64> = core::act
}
batched! {
    /// Maps (N, 4, 4) transforms to (N, 6, 6) adjoints.
    adjoint(transform: Matrix4<f64>) -> Matrix6<f64> = core::adjoint
}
batched! {
    /// Maps (N, 6) twists to their (N, 6, 6) right Jacobians.
    right_jacobian(xi: Vector6<f64>) -> Matrix6<f64> = |x| Ok(core::right_jacobian(x))
}
batched! {
    /// Maps (N, 6) twists to their (N, 6, 6) left Jacobians.
    left_jacobian(xi: Vector6<f64>) -> Matrix6<f64> = |x| Ok(core::left_jacobian(x))
}
batched! {
    /// Maps (N, 6) twists to their inverse right Jacobians.
    right_jacobian_inverse(xi: Vector6<f64>) -> Matrix6<f64> = core::right_jacobian_inverse
}
batched! {
    /// Maps (N, 6) twists to their inverse left Jacobians.
    left_jacobian_inverse(xi: Vector6<f64>) -> Matrix6<f64> = core::left_jacobian_inverse
}

/// Registers the `se3` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn se3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m;
        exp, log, inverse, compose, act, adjoint, right_jacobian, left_jacobian,
        right_jacobian_inverse, left_jacobian_inverse
    );
    Ok(())
}
