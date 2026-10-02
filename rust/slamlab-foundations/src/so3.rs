//! SO(3), the group of 3D rotations: the Rust core's implementation.
//!
//! A placeholder that proves the three-language pipeline: only `hat` exists so far. Rung 0's
//! SO(3) work (`vee`, `exp`, `log`, Jacobians) arrives with milestone 1. The Python reference
//! is `slamlab.foundations.so3`; parity tests hold this implementation to it.

/// A 3-vector.
pub type Vec3 = [f64; 3];

/// A 3x3 matrix, stored row-major as an array of rows.
pub type Mat3 = [[f64; 3]; 3];

/// Maps a 3-vector to its skew-symmetric matrix.
///
/// The result `[v]x` satisfies `hat(v) * u == cross(v, u)` for every 3-vector `u`. It is
/// the map from R^3 into the Lie algebra so(3). Exact: it only copies and negates entries.
///
/// # Arguments
///
/// * `v` - The vector to map. Input validation (finiteness) is the caller's job; the Python
///   entry point validates before calling.
///
/// # Returns
///
/// The skew-symmetric matrix `[v]x`.
///
/// # Examples
///
/// ```
/// use slamlab_foundations::so3::hat;
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
    //! Unit tests for the SO(3) functions. `hat` only copies and negates, so every check is
    //! exact (no tolerance).
    #![allow(
        clippy::float_cmp,
        reason = "hat only copies and negates, so exact comparison is the correct check"
    )]

    use super::*;

    /// The basis vectors fix all nine entries of the matrix.
    #[test]
    #[rustfmt::skip] // keep one matrix row per line
    fn hat_of_basis_vectors() {
        assert_eq!(hat([1.0, 0.0, 0.0]), [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]);
        assert_eq!(hat([0.0, 1.0, 0.0]), [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]);
        assert_eq!(hat([0.0, 0.0, 1.0]), [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]);
    }

    /// `hat(v)` is skew-symmetric: its transpose is its negation.
    #[test]
    fn hat_is_skew_symmetric() {
        let h = hat([1.0, -2.0, 3.0]);
        for (i, row) in h.iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                assert_eq!(*value, -h[j][i], "entry ({i}, {j})");
            }
        }
    }

    /// `hat(v) * u` equals the cross product `v x u` (exact for these values).
    #[test]
    fn hat_is_cross_product() {
        let (v, u) = ([1.0, -2.0, 3.0], [0.5, 4.0, -1.5]);
        let cross = [
            v[1] * u[2] - v[2] * u[1],
            v[2] * u[0] - v[0] * u[2],
            v[0] * u[1] - v[1] * u[0],
        ];
        for (row, expected) in hat(v).iter().zip(cross) {
            let hu: f64 = row.iter().zip(u).map(|(a, b)| a * b).sum();
            assert_eq!(hu, expected);
        }
    }
}
