// A translation to Rust of parts of Eigen 5.0.1 (https://gitlab.com/libeigen/eigen): the
// two-sided Jacobi SVD of `JacobiSVD::compute` in `Eigen/src/SVD/JacobiSVD.h`,
// `JacobiRotation::makeJacobi` in `Eigen/src/Jacobi/Jacobi.h`, and `real_2x2_jacobi_svd` in
// `Eigen/src/misc/RealSvd2x2.h`. Their copyright lines, as those files carry them:
//
// Eigen/src/SVD/JacobiSVD.h:
// Copyright (C) 2009-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2013-2014 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen/src/Jacobi/Jacobi.h:
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen/src/misc/RealSvd2x2.h:
// Copyright (C) 2009-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2013-2016 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A 3x3 matrix's SVD by Eigen's `JacobiSVD` algorithm, on the stack: [`svd3`] and
//! [`singular_values3`], which callers import from this module. It is translated from Eigen,
//! so this file is under Eigen's licence, the Mozilla Public License 2.0, which applies file by
//! file: the rest of `linalg` is not. It gives `Eigen::JacobiSVD<Matrix3d>`'s factors bit for
//! bit (`linalg`'s tests hold it to the fixture C++ writes from Eigen).

use nalgebra::{Matrix3, Vector3};

/// The most sweeps [`svd3`] makes before it gives up. Eigen's `JacobiSVD` has no cap; cyclic
/// Jacobi converges quadratically, in about six sweeps on the hardest 3x3s, so a cap this
/// generous never changes a result where Eigen ends, and turns a non-termination into `None`.
const MAX_SWEEPS: usize = 64;

/// A plane rotation `[c s; -s c]`, as Eigen's real `JacobiRotation`.
#[derive(Debug, Clone, Copy)]
struct Rotation {
    /// The cosine.
    c: f64,
    /// The sine.
    s: f64,
}

impl Rotation {
    /// No rotation.
    const IDENTITY: Self = Self { c: 1.0, s: 0.0 };

    /// The transposed rotation.
    ///
    /// # Returns
    ///
    /// `[c -s; s c]`.
    fn transpose(self) -> Self {
        Self {
            c: self.c,
            s: -self.s,
        }
    }

    /// The product of two rotations.
    ///
    /// # Arguments
    ///
    /// * `other` - The rotation on the right.
    ///
    /// # Returns
    ///
    /// `self * other`.
    fn times(self, other: Self) -> Self {
        Self {
            c: self.c * other.c - self.s * other.s,
            s: self.c * other.s + self.s * other.c,
        }
    }

    /// The rotation that diagonalises the symmetric `[x y; y z]` from both sides, as Eigen's
    /// `JacobiRotation::makeJacobi`.
    ///
    /// # Arguments
    ///
    /// * `x` - The upper-left entry.
    /// * `y` - The off-diagonal entry.
    /// * `z` - The lower-right entry.
    ///
    /// # Returns
    ///
    /// The rotation (none when `y` is zero or subnormal).
    fn jacobi(x: f64, y: f64, z: f64) -> Self {
        let deno = 2.0 * y.abs();
        if deno < f64::MIN_POSITIVE {
            return Self::IDENTITY;
        }
        let tau = (x - z) / deno;
        let w = (tau * tau + 1.0).sqrt();
        let t = if tau > 0.0 {
            1.0 / (tau + w)
        } else {
            1.0 / (tau - w)
        };
        let sign_t = if t > 0.0 { 1.0 } else { -1.0 };
        let n = 1.0 / (t * t + 1.0).sqrt();
        Self {
            c: n,
            s: -sign_t * (y / y.abs()) * t.abs() * n,
        }
    }

    /// Whether this is no rotation (applying it is then skipped, as Eigen skips it).
    ///
    /// # Returns
    ///
    /// Whether `c` is 1 and `s` is 0.
    #[allow(clippy::float_cmp, reason = "exactly no rotation, as Eigen tests it")]
    fn is_identity(self) -> bool {
        self.c == 1.0 && self.s == 0.0
    }

    /// Applies the rotation to rows `p` and `q` from the left: `B = J B`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The matrix.
    /// * `p` - The first row.
    /// * `q` - The second row.
    fn rotate_rows(self, matrix: &mut Matrix3<f64>, p: usize, q: usize) {
        if self.is_identity() {
            return;
        }
        for k in 0..3 {
            let (x, y) = (matrix[(p, k)], matrix[(q, k)]);
            matrix[(p, k)] = self.c * x + self.s * y;
            matrix[(q, k)] = -self.s * x + self.c * y;
        }
    }

    /// Applies the rotation to columns `p` and `q` from the right: `B = B J`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The matrix.
    /// * `p` - The first column.
    /// * `q` - The second column.
    fn rotate_columns(self, matrix: &mut Matrix3<f64>, p: usize, q: usize) {
        let t = self.transpose();
        if t.is_identity() {
            return;
        }
        for k in 0..3 {
            let (x, y) = (matrix[(k, p)], matrix[(k, q)]);
            matrix[(k, p)] = t.c * x + t.s * y;
            matrix[(k, q)] = -t.s * x + t.c * y;
        }
    }
}

/// The SVD of a 2x2 block `(p, q)` of a matrix, as Eigen's `real_2x2_jacobi_svd`: a rotation
/// symmetrises the block, then a Jacobi rotation diagonalises it.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `p` - The block's second index.
/// * `q` - Its first.
///
/// # Returns
///
/// The left and right rotations `(J_l, J_r)`: `J_l B J_r` is diagonal.
fn jacobi_2x2(matrix: &Matrix3<f64>, p: usize, q: usize) -> (Rotation, Rotation) {
    let (a, b) = (matrix[(p, p)], matrix[(p, q)]);
    let (c, d) = (matrix[(q, p)], matrix[(q, q)]);
    let t = a + d;
    let skew = c - b;
    let symmetrise = if skew.abs() < f64::MIN_POSITIVE {
        Rotation::IDENTITY
    } else {
        let u = t / skew;
        let tmp = (1.0 + u * u).sqrt();
        Rotation {
            c: u / tmp,
            s: 1.0 / tmp,
        }
    };
    // The block's rows rotated: `[x y; y z]`, symmetric.
    let (x, y, z) = if symmetrise.is_identity() {
        (a, b, d)
    } else {
        (
            symmetrise.c * a + symmetrise.s * c,
            symmetrise.c * b + symmetrise.s * d,
            -symmetrise.s * b + symmetrise.c * d,
        )
    };
    let right = Rotation::jacobi(x, y, z);
    (symmetrise.times(right.transpose()), right)
}

/// A 3x3 matrix's SVD by two-sided Jacobi, Eigen's `JacobiSVD` for a square matrix: the
/// matrix scaled by its largest entry, sweeps of 2x2 SVDs over each off-diagonal pair until
/// every pair is below `2ε` times the largest diagonal entry, the diagonal made positive (its
/// signs moved into `U`), then sorted.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `vectors` - Whether to accumulate `U` and `V` (identity otherwise).
///
/// # Returns
///
/// `(U, σ, V)`, or `None` for a non-finite matrix (or one not diagonalised within
/// [`MAX_SWEEPS`], never observed).
fn jacobi_svd3(
    matrix: &Matrix3<f64>,
    vectors: bool,
) -> Option<(Matrix3<f64>, Vector3<f64>, Matrix3<f64>)> {
    if !matrix.iter().all(|x| x.is_finite()) {
        return None;
    }
    let largest = matrix.amax();
    let scale = if largest == 0.0 { 1.0 } else { largest };
    let mut work = matrix / scale;
    let mut u = Matrix3::identity();
    let mut v = Matrix3::identity();
    let precision = 2.0 * f64::EPSILON;
    let mut max_diagonal = work.diagonal().amax();
    let mut finished = false;
    let mut sweeps = 0;
    while !finished {
        if sweeps == MAX_SWEEPS {
            return None;
        }
        sweeps += 1;
        finished = true;
        for p in 1..3 {
            for q in 0..p {
                // Subnormal entries count as zero, so the sweeps end.
                let threshold = f64::MIN_POSITIVE.max(precision * max_diagonal);
                if work[(p, q)].abs() > threshold || work[(q, p)].abs() > threshold {
                    finished = false;
                    let (left, right) = jacobi_2x2(&work, p, q);
                    left.rotate_rows(&mut work, p, q);
                    right.rotate_columns(&mut work, p, q);
                    if vectors {
                        left.transpose().rotate_columns(&mut u, p, q);
                        right.rotate_columns(&mut v, p, q);
                    }
                    max_diagonal = max_diagonal.max(work[(p, p)].abs().max(work[(q, q)].abs()));
                }
            }
        }
    }
    let mut singular = Vector3::zeros();
    for i in 0..3 {
        let a = work[(i, i)];
        singular[i] = a.abs();
        if vectors && a < 0.0 {
            u.set_column(i, &-u.column(i));
        }
    }
    singular *= scale;
    // Largest first; the first of equal values stays first, as Eigen's `maxCoeff`.
    for i in 0..3 {
        let mut pos = i;
        for k in i + 1..3 {
            if singular[k] > singular[pos] {
                pos = k;
            }
        }
        if singular[pos] == 0.0 {
            break;
        }
        if pos != i {
            singular.swap_rows(i, pos);
            u.swap_columns(i, pos);
            v.swap_columns(i, pos);
        }
    }
    Some((u, singular, v))
}

/// A 3x3 matrix's SVD, singular values largest first: two-sided Jacobi, on the stack, as
/// C++'s `Eigen::JacobiSVD<Matrix3d>`.
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// `(U, σ, Vᵀ)`, or `None` for a non-finite matrix (or one not diagonalised within
/// [`MAX_SWEEPS`] sweeps, never observed).
#[must_use]
pub fn svd3(matrix: &Matrix3<f64>) -> Option<(Matrix3<f64>, Vector3<f64>, Matrix3<f64>)> {
    let (u, singular, v) = jacobi_svd3(matrix, true)?;
    Some((u, singular, v.transpose()))
}

/// A 3x3 matrix's singular values, largest first (as [`svd3`]).
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// σ, or NaN for a non-finite matrix or one [`svd3`] gives up on (which every rank test,
/// written so NaN fails it, then
/// rejects).
#[must_use]
pub fn singular_values3(matrix: &Matrix3<f64>) -> Vector3<f64> {
    jacobi_svd3(matrix, false).map_or_else(|| Vector3::repeat(f64::NAN), |(_, s, _)| s)
}
