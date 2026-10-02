//! SO(3) helpers. A placeholder proving the build pipeline; rung 0's real
//! SO(3) work arrives with milestone 1.

pub type Vec3 = [f64; 3];
pub type Mat3 = [[f64; 3]; 3];

/// The skew-symmetric matrix `[v]x`, so that `hat(v) * u == cross(v, u)`.
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
    use super::*;

    #[test]
    fn hat_is_cross_product() {
        let (v, u) = ([1.0, -2.0, 3.0], [0.5, 4.0, -1.5]);
        let cross = [
            v[1] * u[2] - v[2] * u[1],
            v[2] * u[0] - v[0] * u[2],
            v[0] * u[1] - v[1] * u[0],
        ];
        let h = hat(v);
        for i in 0..3 {
            let hu: f64 = (0..3).map(|j| h[i][j] * u[j]).sum();
            assert!(
                (hu - cross[i]).abs() < 1e-12,
                "row {i}: {hu} != {}",
                cross[i]
            );
        }
    }
}
