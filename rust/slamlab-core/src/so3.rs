//! SO(3) helpers.
//!
//! A placeholder that proves the three-language pipeline. Rung 0's real SO(3) work arrives
//! with milestone 1.

/// A 3-vector.
pub type Vec3 = [f64; 3];

/// A 3x3 matrix, stored row-major as an array of rows.
pub type Mat3 = [[f64; 3]; 3];

/// Maps a 3-vector to its skew-symmetric matrix.
///
/// The result `[v]x` satisfies `hat(v) * u == cross(v, u)` for every 3-vector `u`. It is
/// the map from R^3 into the Lie algebra so(3).
///
/// # Arguments
///
/// * `v` - The vector to map.
///
/// # Returns
///
/// The skew-symmetric matrix `[v]x`.
///
/// # Examples
///
/// ```
/// use slamlab_core::so3::hat;
///
/// assert_eq!(hat([1.0, 2.0, 3.0]), [[0.0, -3.0, 2.0], [3.0, 0.0, -1.0], [-2.0, 1.0, 0.0]]);
/// ```
#[must_use]
#[rustfmt::skip] // keep one matrix row per line
pub fn hat(v: Vec3) -> Mat3 {
    [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]
}

#[cfg(test)]
mod tests {
    //! Unit tests for the SO(3) helpers.

    use super::*;

    /// `hat(v) * u` equals the cross product `v x u`.
    #[test]
    fn hat_is_cross_product() {
        let (v, u) = ([1.0, -2.0, 3.0], [0.5, 4.0, -1.5]);
        let cross = [
            v[1] * u[2] - v[2] * u[1],
            v[2] * u[0] - v[0] * u[2],
            v[0] * u[1] - v[1] * u[0],
        ];
        let h = hat(v);
        for (row, expected) in h.iter().zip(cross) {
            let hu: f64 = row.iter().zip(u).map(|(a, b)| a * b).sum();
            assert!(
                (hu - expected).abs() < 1e-12,
                "hat(v)u = {hu}, cross = {expected}"
            );
        }
    }
}
