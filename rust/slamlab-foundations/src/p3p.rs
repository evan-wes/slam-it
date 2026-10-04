//! P3P: the Rust core's camera pose from three world points and their bearings.
//!
//! Mirrors `slamlab.foundations.p3p`, whose docstring states the maths: the three distance
//! equations in the depths, two conics whose pencil holds a degenerate member (a root of a
//! cubic), split into two lines and intersected with a conic, the depths polished by Newton's
//! method, and the pose as the rigid alignment of the two triangles. Lambda Twist (Persson and
//! Nordberg, ECCV 2018) splits the line pair by its eigen-decomposition; Ding et al. (CVPR
//! 2023) by its adjugate, on depth ratios referred to the longest side. Every pose with all
//! three depths positive is returned, as `T_cam_world`.

use nalgebra::{Matrix3, Matrix3x2, Matrix4, Vector3};

use crate::alignment;
use crate::correspondences::check_correspondences;
use crate::error::{ErrorKind, Result, fail};
use crate::so3;

/// Correspondences a P3P solver takes.
pub const SAMPLE_SIZE: usize = 3;
/// At most this many Newton steps polish the cubic's root, and the depths.
pub const POLISH_MAX_STEPS: usize = 10;
/// Polishing stops when a step is below this, relative to the value polished.
pub const POLISH_TOLERANCE: f64 = 1e-14;
/// A negative discriminant above this times `-(b^2 + |4ac|)` counts as zero (a double root);
/// a positive one, however small, gives two roots.
pub const REAL_TOLERANCE: f64 = 1e-12;
/// A solution is kept only if its polished depths satisfy the distance equations to within
/// this times the largest squared distance.
pub const DISTANCE_TOLERANCE: f64 = 1e-6;
/// The world points are collinear when `|X12 x X13|` is below this times `|X12| |X13|`.
pub const COLLINEAR_TOLERANCE: f64 = 1e-10;

/// The solvers, with the codes they cross the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Solver {
    /// Ding et al.'s solver.
    Ding = 0,
    /// Lambda Twist.
    LambdaTwist = 1,
}

impl Solver {
    /// Every solver, in code order.
    pub const ALL: [Self; 2] = [Self::Ding, Self::LambdaTwist];

    /// The solver's name, as `slamlab.foundations.p3p.SOLVERS` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Ding => "ding",
            Self::LambdaTwist => "lambda_twist",
        }
    }
}

/// The distance equations' data: `b_ij` (bearing cosines) and `a_ij` (squared world
/// distances), for `ij = 12, 13, 23`.
struct Geometry {
    /// The cosines `b_ij`.
    cosines: Vector3<f64>,
    /// The squared distances `a_ij`.
    squared: Vector3<f64>,
}

impl Geometry {
    /// The distance equations' data as named scalars.
    ///
    /// # Returns
    ///
    /// `[b_12, b_13, b_23, a_12, a_13, a_23]`.
    fn terms(&self) -> [f64; 6] {
        [
            self.cosines[0],
            self.cosines[1],
            self.cosines[2],
            self.squared[0],
            self.squared[1],
            self.squared[2],
        ]
    }
}

/// The point pairs of the three distance equations, in [`Geometry`]'s order.
const PAIRS: [(usize, usize); 3] = [(0, 1), (0, 2), (1, 2)];

/// Ding's reordering: for the longest of the pairs 01, 02, 12, the order of the points that
/// makes it the pair of the second and third.
const LONGEST_LAST: [[usize; 3]; 3] = [[2, 1, 0], [1, 0, 2], [0, 1, 2]];

/// The index of the first largest entry, as Eigen's `maxCoeff` and numpy's `argmax` pick it.
///
/// # Arguments
///
/// * `values` - The entries.
///
/// # Returns
///
/// The index.
fn first_max(values: impl IntoIterator<Item = f64>) -> usize {
    let mut best = (0, f64::NEG_INFINITY);
    for (i, value) in values.into_iter().enumerate() {
        if i == 0 || value > best.1 {
            best = (i, value);
        }
    }
    best.0
}

/// The monic cubic's value at `z`.
///
/// # Arguments
///
/// * `z` - The argument.
/// * `c` - The coefficients `(c2, c1, c0)`.
///
/// # Returns
///
/// `z^3 + c2 z^2 + c1 z + c0`.
fn cubic(z: f64, c: [f64; 3]) -> f64 {
    ((z + c[0]) * z + c[1]) * z + c[2]
}

/// Polishes a root of the monic cubic by Newton's method.
///
/// # Arguments
///
/// * `root` - The root.
/// * `c` - The coefficients `(c2, c1, c0)`.
///
/// # Returns
///
/// The polished root.
fn polish_cubic(mut root: f64, c: [f64; 3]) -> f64 {
    let mut value = cubic(root, c);
    for _ in 0..POLISH_MAX_STEPS {
        let slope = (3.0 * root + 2.0 * c[0]) * root + c[1];
        if !(slope != 0.0) {
            break;
        }
        let step = value / slope;
        let moved = root - step;
        let moved_value = cubic(moved, c);
        if !(moved_value.abs() < value.abs()) {
            break;
        }
        root = moved;
        value = moved_value;
        if step.abs() <= POLISH_TOLERANCE * root.abs().max(1.0) {
            break;
        }
    }
    root
}

/// The largest real root of the monic cubic `z^3 + c2 z^2 + c1 z + c0`, in closed form and
/// polished by Newton's method (see `slamlab.foundations.p3p.cubic_root`).
///
/// # Arguments
///
/// * `c2`, `c1`, `c0` - The coefficients, highest first.
///
/// # Returns
///
/// The root.
#[must_use]
pub fn cubic_root(c2: f64, c1: f64, c0: f64) -> f64 {
    let p = c1 - c2 * c2 / 3.0;
    let q = (2.0 * c2 * c2 * c2 - 9.0 * c2 * c1) / 27.0 + c0;
    let discriminant = q * q / 4.0 + p * p * p / 27.0;
    let u = if discriminant >= 0.0 {
        let root = discriminant.sqrt();
        (-0.5 * q + root).cbrt() + (-0.5 * q - root).cbrt()
    } else {
        let cosine = (1.5 * q / p * (-3.0 / p).sqrt()).clamp(-1.0, 1.0);
        2.0 * (-p / 3.0).sqrt() * (cosine.acos() / 3.0).cos()
    };
    polish_cubic(u - c2 / 3.0, [c2, c1, c0])
}

/// A symmetric 3x3 matrix's cofactor matrix, column k the cross product of the other two.
///
/// # Arguments
///
/// * `m` - The matrix.
///
/// # Returns
///
/// The cofactor matrix.
fn cofactor(m: &Matrix3<f64>) -> Matrix3<f64> {
    Matrix3::from_columns(&[
        m.column(1).cross(&m.column(2)),
        m.column(2).cross(&m.column(0)),
        m.column(0).cross(&m.column(1)),
    ])
}

/// `det(first + s second)` as a cubic in `s`, highest power first.
///
/// # Arguments
///
/// * `first`, `second` - The conics.
///
/// # Returns
///
/// `(det B, <A, cof B>, <B, cof A>, det A)`.
fn pencil_cubic(first: &Matrix3<f64>, second: &Matrix3<f64>) -> [f64; 4] {
    let cof_first = cofactor(first);
    let cof_second = cofactor(second);
    [
        second.column(0).dot(&cof_second.column(0)),
        first.component_mul(&cof_second).sum(),
        second.component_mul(&cof_first).sum(),
        first.column(0).dot(&cof_first.column(0)),
    ]
}

/// The real roots of `a t^2 + b t + c`, without cancellation.
///
/// # Arguments
///
/// * `a`, `b`, `c` - The coefficients, highest first.
///
/// # Returns
///
/// Zero, one or two roots.
fn quadratic_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    let discriminant = b * b - 4.0 * a * c;
    let scale = b * b + (4.0 * a * c).abs();
    if discriminant < -REAL_TOLERANCE * scale {
        return Vec::new();
    }
    if a == 0.0 {
        return if b == 0.0 { Vec::new() } else { vec![-c / b] };
    }
    if discriminant < 0.0 {
        return vec![-0.5 * b / a];
    }
    let half = -0.5 * (b + discriminant.sqrt().copysign(b));
    if half == 0.0 {
        vec![half / a]
    } else {
        vec![half / a, c / half]
    }
}

/// The depth `sqrt(a / (t^2 - 2 b t + 1))` one distance equation fixes.
///
/// # Arguments
///
/// * `squared` - The squared distance `a`.
/// * `tau` - The depth ratio `t`, positive.
/// * `cosine` - The bearings' cosine `b`.
///
/// # Returns
///
/// The depth, or `None` for a non-positive ratio or where rounding pushed the denominator to
/// zero or below (two coinciding bearings).
fn scaled(squared: f64, tau: f64, cosine: f64) -> Option<f64> {
    let denominator = tau * (tau - 2.0 * cosine) + 1.0;
    (tau > 0.0 && denominator > 0.0).then(|| (squared / denominator).sqrt())
}

/// The distance equations' residuals at the depths.
///
/// # Arguments
///
/// * `depths` - The depths.
/// * `geometry` - The cosines and squared distances.
///
/// # Returns
///
/// `r_ij`, for `ij = 12, 13, 23`.
fn distance_residuals(depths: &Vector3<f64>, geometry: &Geometry) -> Vector3<f64> {
    Vector3::from_fn(|k, _| {
        let (i, j) = PAIRS[k];
        depths[i] * depths[i] - 2.0 * geometry.cosines[k] * depths[i] * depths[j]
            + depths[j] * depths[j]
            - geometry.squared[k]
    })
}

/// Polishes the depths by Newton's method on the distance equations.
///
/// # Arguments
///
/// * `depths` - The depths.
/// * `geometry` - The cosines and squared distances.
///
/// # Returns
///
/// The polished depths and the final residual's norm.
fn polish_depths(mut depths: Vector3<f64>, geometry: &Geometry) -> (Vector3<f64>, f64) {
    let mut current = distance_residuals(&depths, geometry);
    for _ in 0..POLISH_MAX_STEPS {
        let mut jacobian = Matrix3::zeros();
        for (k, &(i, j)) in PAIRS.iter().enumerate() {
            jacobian[(k, i)] = 2.0 * (depths[i] - geometry.cosines[k] * depths[j]);
            jacobian[(k, j)] = 2.0 * (depths[j] - geometry.cosines[k] * depths[i]);
        }
        // A singular Jacobian stops the polish, as numpy's LinAlgError does.
        let Some(step) = jacobian.full_piv_lu().solve(&current) else {
            break;
        };
        let moved = depths - step;
        let moved_residuals = distance_residuals(&moved, geometry);
        if !(moved_residuals.norm() < current.norm()) {
            break;
        }
        depths = moved;
        current = moved_residuals;
        if step.norm() <= POLISH_TOLERANCE * depths.norm() {
            break;
        }
    }
    (depths, current.norm())
}

/// Whether three world points are (nearly) collinear.
///
/// # Arguments
///
/// * `points` - The points.
///
/// # Returns
///
/// True if they have no unique pose.
fn collinear(points: &[Vector3<f64>]) -> bool {
    let d1 = points[0] - points[1];
    let d2 = points[0] - points[2];
    !(d1.cross(&d2).norm() > COLLINEAR_TOLERANCE * d1.norm() * d2.norm())
}

/// Polishes candidate depths and adds the pose, if all three stay positive and satisfy the
/// distance equations.
///
/// # Arguments
///
/// * `depths` - The candidate depths.
/// * `points`, `bearings` - The correspondences.
/// * `geometry` - Their cosines and squared distances.
/// * `poses` - The poses found so far.
fn finish(
    depths: Vector3<f64>,
    points: &[Vector3<f64>],
    bearings: &[Vector3<f64>],
    geometry: &Geometry,
    poses: &mut Vec<Matrix4<f64>>,
) {
    let (polished, residual) = polish_depths(depths, geometry);
    if !polished.iter().all(|&d| d > 0.0) || residual > DISTANCE_TOLERANCE * geometry.squared.max()
    {
        return;
    }
    let camera: Vec<Vector3<f64>> = bearings
        .iter()
        .zip(polished.iter())
        .map(|(b, &d)| b * d)
        .collect();
    if let Ok(pose) = alignment::umeyama(points, &camera, &[1.0; SAMPLE_SIZE], false)
        && pose.iter().all(|x| x.is_finite())
    {
        poses.push(pose);
    }
}

/// The squared distances and cosines of three correspondences.
///
/// # Arguments
///
/// * `points`, `bearings` - The correspondences.
///
/// # Returns
///
/// `a_ij` and `b_ij`, for `ij = 12, 13, 23`.
fn geometry_of(points: &[Vector3<f64>], bearings: &[Vector3<f64>]) -> Geometry {
    Geometry {
        cosines: Vector3::from_fn(|k, _| bearings[PAIRS[k].0].dot(&bearings[PAIRS[k].1])),
        squared: Vector3::from_fn(|k, _| (points[PAIRS[k].0] - points[PAIRS[k].1]).norm_squared()),
    }
}

/// Keeps the candidate depths with every entry positive.
///
/// # Arguments
///
/// * `candidates` - The candidates.
///
/// # Returns
///
/// The positive ones.
fn positive(candidates: Vec<Vector3<f64>>) -> Vec<Vector3<f64>> {
    candidates
        .into_iter()
        .filter(|d| d.iter().all(|&x| x > 0.0))
        .collect()
}

/// A symmetric matrix with a zero eigenvalue, eigen-decomposed in closed form.
struct KnownZeroEigen {
    /// The eigenvectors of the two nonzero eigenvalues, as columns.
    vectors: Matrix3x2<f64>,
    /// The larger eigenvalue in magnitude.
    sigma1: f64,
    /// The smaller.
    sigma2: f64,
}

/// Eigen-decomposes a symmetric 3x3 matrix with a zero eigenvalue, in closed form.
///
/// # Arguments
///
/// * `m` - The matrix.
///
/// # Returns
///
/// The decomposition, or `None` if the eigenvalues are not real or an eigenvector's last
/// entry vanishes.
fn eigen_known_zero(m: &Matrix3<f64>) -> Option<KnownZeroEigen> {
    let p1 = -m.trace();
    let p0 = -(m[(0, 1)] * m[(0, 1)]) - m[(0, 2)] * m[(0, 2)] - m[(1, 2)] * m[(1, 2)]
        + m[(0, 0)] * (m[(1, 1)] + m[(2, 2)])
        + m[(1, 1)] * m[(2, 2)];
    let inside = p1 * p1 / 4.0 - p0;
    if !(inside >= 0.0) {
        return None;
    }
    let half = inside.sqrt();
    let (mut sigma1, mut sigma2) = (-p1 / 2.0 + half, -p1 / 2.0 - half);
    if sigma1.abs() < sigma2.abs() {
        std::mem::swap(&mut sigma1, &mut sigma2);
    }
    let mut vectors = Matrix3x2::zeros();
    for (column, sigma) in [sigma1, sigma2].into_iter().enumerate() {
        let c = sigma * sigma + m[(0, 0)] * m[(1, 1)]
            - sigma * (m[(0, 0)] + m[(1, 1)])
            - m[(0, 1)] * m[(0, 1)];
        if !(c != 0.0) {
            return None;
        }
        let a1 = (sigma * m[(0, 2)] + m[(0, 1)] * m[(1, 2)] - m[(0, 2)] * m[(1, 1)]) / c;
        let a2 = (sigma * m[(1, 2)] + m[(0, 1)] * m[(0, 2)] - m[(0, 0)] * m[(1, 2)]) / c;
        vectors.set_column(
            column,
            &(Vector3::new(a1, a2, 1.0) / (1.0 + a1 * a1 + a2 * a2).sqrt()),
        );
    }
    Some(KnownZeroEigen {
        vectors,
        sigma1,
        sigma2,
    })
}

/// Intersects one of Lambda Twist's lines, `u . L = 0`, with the distance conics.
///
/// # Arguments
///
/// * `u` - The line.
/// * `geometry` - The cosines and squared distances.
///
/// # Returns
///
/// The candidate depths, every one positive.
fn twist_depths(u: &Vector3<f64>, geometry: &Geometry) -> Vec<Vector3<f64>> {
    let [b12, b13, b23, a12, a13, a23] = geometry.terms();
    let mut candidates = Vec::new();
    if u[0].abs() < u[1].abs() {
        // l2 = w0 l1 + w1 l3; t = l3 / l1.
        let (w0, w1) = (-u[0] / u[1], -u[2] / u[1]);
        for tau in quadratic_roots(
            a13 * w1 * w1 - 2.0 * a13 * b23 * w1 + a13 - a23,
            2.0 * a13 * w0 * w1 - 2.0 * a13 * b23 * w0 + 2.0 * a23 * b13,
            a13 * w0 * w0 - a23,
        ) {
            if let Some(lam1) = scaled(a13, tau, b13) {
                let lam3 = tau * lam1;
                candidates.push(Vector3::new(lam1, w0 * lam1 + w1 * lam3, lam3));
            }
        }
    } else {
        // l1 = w0 l2 + w1 l3; t = l3 / l2.
        let (w0, w1) = (-u[1] / u[0], -u[2] / u[0]);
        for tau in quadratic_roots(
            (a13 - a12) * w1 * w1 + 2.0 * a12 * b13 * w1 - a12,
            2.0 * (a13 - a12) * w0 * w1 - 2.0 * a13 * b12 * w1 + 2.0 * a12 * b13 * w0,
            (a13 - a12) * w0 * w0 - 2.0 * a13 * b12 * w0 + a13,
        ) {
            if let Some(lam2) = scaled(a23, tau, b23) {
                let lam3 = tau * lam2;
                candidates.push(Vector3::new(w0 * lam2 + w1 * lam3, lam2, lam3));
            }
        }
    }
    positive(candidates)
}

/// Lambda Twist.
///
/// # Arguments
///
/// * `points`, `bearings` - The correspondences.
///
/// # Returns
///
/// The poses.
fn lambda_twist(points: &[Vector3<f64>], bearings: &[Vector3<f64>]) -> Vec<Matrix4<f64>> {
    let geometry = geometry_of(points, bearings);
    let [b12, b13, b23, a12, a13, a23] = geometry.terms();
    #[rustfmt::skip] // keep one matrix row per line
    let d1 = Matrix3::new(
        a23, -a23 * b12, 0.0,
        -a23 * b12, a23 - a12, a12 * b23,
        0.0, a12 * b23, -a12,
    );
    #[rustfmt::skip] // keep one matrix row per line
    let d2 = Matrix3::new(
        a23, 0.0, -a23 * b13,
        0.0, -a13, a13 * b23,
        -a23 * b13, a13 * b23, a23 - a13,
    );
    let [c3, c2, c1, c0] = pencil_cubic(&d1, &d2);
    if !(c3 != 0.0) {
        return Vec::new();
    }
    let gamma = cubic_root(c2 / c3, c1 / c3, c0 / c3);
    let Some(decomposed) = eigen_known_zero(&(d1 + gamma * d2)) else {
        return Vec::new();
    };
    let ratio = -decomposed.sigma2 / decomposed.sigma1;
    if !(ratio >= 0.0) {
        return Vec::new();
    }
    let s = ratio.sqrt();
    let mut poses = Vec::new();
    for sign in [1.0, -1.0] {
        let u = decomposed.vectors.column(0) - sign * s * decomposed.vectors.column(1);
        for depths in twist_depths(&u, &geometry) {
            finish(depths, points, bearings, &geometry, &mut poses);
        }
    }
    poses
}

/// Splits a degenerate conic into its two lines through its adjugate.
///
/// # Arguments
///
/// * `c` - The conic, rank 2.
///
/// # Returns
///
/// The lines, or `None` if they are complex.
fn line_pair(c: &Matrix3<f64>) -> Option<[Vector3<f64>; 2]> {
    #[rustfmt::skip] // keep one matrix row per line
    let adjugate = -Matrix3::new(
        c[(1, 1)] * c[(2, 2)] - c[(1, 2)] * c[(1, 2)],
        c[(0, 2)] * c[(1, 2)] - c[(0, 1)] * c[(2, 2)],
        c[(0, 1)] * c[(1, 2)] - c[(0, 2)] * c[(1, 1)],
        c[(0, 2)] * c[(1, 2)] - c[(0, 1)] * c[(2, 2)],
        c[(0, 0)] * c[(2, 2)] - c[(0, 2)] * c[(0, 2)],
        c[(0, 1)] * c[(0, 2)] - c[(0, 0)] * c[(1, 2)],
        c[(0, 1)] * c[(1, 2)] - c[(0, 2)] * c[(1, 1)],
        c[(0, 1)] * c[(0, 2)] - c[(0, 0)] * c[(1, 2)],
        c[(0, 0)] * c[(1, 1)] - c[(0, 1)] * c[(0, 1)],
    );
    let k = first_max(adjugate.diagonal().iter().copied());
    if !(adjugate[(k, k)] > 0.0) {
        return None;
    }
    let w: Vector3<f64> = adjugate.column(k) / adjugate[(k, k)].sqrt();
    let rank_one = c + so3::hat(&w);
    // Column-major order, as Eigen's maxCoeff visits a column-major matrix.
    let largest = first_max(rank_one.iter().map(|x| x.abs()));
    let (row, column) = (largest % 3, largest / 3);
    Some([
        rank_one.column(column).into_owned(),
        rank_one.row(row).transpose(),
    ])
}

/// Ding's conic coefficients: the squared distances over the longest, `a12`.
struct Ratios {
    /// `a01 / a12`.
    a: f64,
    /// `a02 / a12`.
    b: f64,
}

/// Intersects one of Ding et al.'s lines, `p . (d2, d0, d1) = 0`, with a conic.
///
/// # Arguments
///
/// * `p` - The line.
/// * `geometry` - The reordered points' cosines and squared distances.
/// * `ratios` - The conics' `a = a01 / a12` and `b = a02 / a12`.
///
/// # Returns
///
/// The candidate depths, every one positive.
fn ding_depths(p: &Vector3<f64>, geometry: &Geometry, ratios: &Ratios) -> Vec<Vector3<f64>> {
    let [m01, m02, m12] = [
        geometry.cosines[0],
        geometry.cosines[1],
        geometry.cosines[2],
    ];
    let (a01, a12) = (geometry.squared[0], geometry.squared[2]);
    let Ratios { a, b } = *ratios;
    let mut candidates = Vec::new();
    if p[0].abs() <= p[1].abs() {
        // d0 = w0 d2 + w1 d1; t = d1 / d2.
        let (w0, w1) = (-p[0] / p[1], -p[2] / p[1]);
        for tau in quadratic_roots(
            w1 * w1 - b,
            2.0 * (b * m12 - m02 * w1 + w0 * w1),
            w0 * w0 - 2.0 * m02 * w0 - b + 1.0,
        ) {
            if let Some(d2) = scaled(a12, tau, m12) {
                let d1 = tau * d2;
                candidates.push(Vector3::new(w0 * d2 + w1 * d1, d1, d2));
            }
        }
    } else {
        // d2 = w0 d0 + w1 d1; t = d1 / d0.
        let (w0, w1) = (-p[1] / p[0], -p[2] / p[0]);
        for tau in quadratic_roots(
            -a * w1 * w1 + 2.0 * a * m12 * w1 - a + 1.0,
            2.0 * (a * m12 * w0 - m01 - a * w0 * w1),
            1.0 - a * w0 * w0,
        ) {
            if let Some(d0) = scaled(a01, tau, m01) {
                let d1 = tau * d0;
                candidates.push(Vector3::new(d0, d1, w0 * d0 + w1 * d1));
            }
        }
    }
    positive(candidates)
}

/// Ding et al.'s solver.
///
/// # Arguments
///
/// * `points`, `bearings` - The correspondences.
///
/// # Returns
///
/// The poses.
fn ding(points: &[Vector3<f64>], bearings: &[Vector3<f64>]) -> Vec<Matrix4<f64>> {
    let longest = first_max(geometry_of(points, bearings).squared.iter().copied());
    let order = LONGEST_LAST[longest];
    let x: Vec<Vector3<f64>> = order.iter().map(|&i| bearings[i]).collect();
    let world: Vec<Vector3<f64>> = order.iter().map(|&i| points[i]).collect();
    let geometry = geometry_of(&world, &x);
    let [m01, m02, m12] = [
        geometry.cosines[0],
        geometry.cosines[1],
        geometry.cosines[2],
    ];
    let ratios = Ratios {
        a: geometry.squared[0] / geometry.squared[2],
        b: geometry.squared[1] / geometry.squared[2],
    };
    let Ratios { a, b } = ratios;
    #[rustfmt::skip] // keep one matrix row per line
    let first = Matrix3::new(
        -a, 0.0, a * m12,
        0.0, 1.0, -m01,
        a * m12, -m01, 1.0 - a,
    );
    #[rustfmt::skip] // keep one matrix row per line
    let second = Matrix3::new(
        1.0 - b, -m02, b * m12,
        -m02, 1.0, 0.0,
        b * m12, 0.0, -b,
    );
    let [c3, c2, c1, c0] = pencil_cubic(&first, &second);
    if !(c3 != 0.0) {
        return Vec::new();
    }
    let s = cubic_root(c2 / c3, c1 / c3, c0 / c3);
    let Some(lines) = line_pair(&(first + s * second)) else {
        return Vec::new();
    };
    let mut poses = Vec::new();
    for line in &lines {
        for depths in ding_depths(line, &geometry, &ratios) {
            finish(depths, &world, &x, &geometry, &mut poses);
        }
    }
    poses
}

/// Every camera pose three world points and their bearings allow (unchecked).
///
/// # Arguments
///
/// * `points` - Three world points.
/// * `bearings` - Their unit bearings in the camera.
/// * `solver` - The solver.
///
/// # Returns
///
/// The poses `T_cam_world` with every depth positive (none for collinear points).
#[must_use]
pub fn solve(
    points: &[Vector3<f64>],
    bearings: &[Vector3<f64>],
    solver: Solver,
) -> Vec<Matrix4<f64>> {
    if collinear(points) {
        return Vec::new();
    }
    match solver {
        Solver::Ding => ding(points, bearings),
        Solver::LambdaTwist => lambda_twist(points, bearings),
    }
}

/// Every camera pose three world points and their bearings allow.
///
/// # Arguments
///
/// * `points` - Three world points.
/// * `bearings` - Their unit bearings in the camera (any sign of z).
/// * `solver` - The solver.
///
/// # Returns
///
/// The poses.
///
/// # Errors
///
/// `InvalidInput` unless there are three finite points and unit bearings.
pub fn p3p(
    points: &[Vector3<f64>],
    bearings: &[Vector3<f64>],
    solver: Solver,
) -> Result<Vec<Matrix4<f64>>> {
    check_correspondences(points, bearings, SAMPLE_SIZE, false)?;
    if points.len() != SAMPLE_SIZE {
        return fail(
            ErrorKind::InvalidInput,
            format!("points: expected exactly 3, got {}", points.len()),
        );
    }
    Ok(solve(points, bearings, solver))
}

#[cfg(test)]
mod tests {
    //! Native tests: both solvers find the true pose among their solutions.

    use nalgebra::Vector6;

    use super::*;
    use crate::se3;

    /// Each solver recovers a known pose from three noise-free correspondences, and the
    /// cubic's root is a root.
    #[test]
    fn both_solvers_recover_the_pose() {
        let pose = se3::exp(&Vector6::new(0.2, -0.3, 0.1, 0.3, -0.2, 0.15));
        let points = [
            Vector3::new(0.5, -0.3, 4.0),
            Vector3::new(-0.7, 0.2, 5.0),
            Vector3::new(0.1, 0.8, 3.5),
        ];
        let bearings: Vec<Vector3<f64>> = points
            .iter()
            .map(|p| se3::act(&pose, p).unwrap().normalize())
            .collect();
        for solver in Solver::ALL {
            let poses = p3p(&points, &bearings, solver).unwrap();
            assert!(
                poses.iter().any(|p| (p - pose).amax() < 1e-9),
                "{}",
                solver.name()
            );
        }
        let root = cubic_root(-2.0, -5.0, 6.0);
        assert!((root - 3.0).abs() < 1e-14);
    }
}
