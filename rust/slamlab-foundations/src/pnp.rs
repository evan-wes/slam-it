//! Absolute pose (PnP): the Rust core's EPnP, pose refinement and PnP RANSAC.
//!
//! Mirrors `slamlab.foundations.pnp`, whose docstring states the maths: the reprojection
//! error in the normalised image plane (infinite behind the camera), EPnP's control points,
//! null space and three linearisations of the distance constraints (Lepetit et al., IJCV
//! 2009), the Levenberg-Marquardt refinement on SE(3) with a Cauchy loss, and LO-MSAC with a
//! P3P minimal solver and EPnP refits, its consensus refined and kept only if the MSAC cost
//! drops. Poses are `T_cam_world`.

use nalgebra::{
    DMatrix, DVector, Matrix3, Matrix4, Matrix6x4, RowVector4, SMatrix, SVector, SymmetricEigen,
    Vector2, Vector3, Vector4, Vector6,
};

use crate::correspondences::{check_correspondences, gather, normalised_xy};
use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::linalg::least_squares;
use crate::p3p::{self, Solver};
use crate::ransac::{self, Estimator, Options, RefinedSolution};
use crate::{alignment, nls, se3, so3};

/// Correspondences EPnP takes (see `slamlab.foundations.pnp.EPNP_MINIMUM`).
pub const EPNP_MINIMUM: usize = 5;
/// The points are planar for EPnP when their covariance's smallest eigenvalue is below this
/// fraction of its largest.
pub const PLANAR_TOLERANCE: f64 = 1e-12;
/// At most this many Gauss-Newton steps polish EPnP's betas.
pub const POLISH_MAX_STEPS: usize = 10;
/// Polishing stops when a step is below this, relative to the betas.
pub const POLISH_TOLERANCE: f64 = 1e-14;
/// Levenberg-Marquardt iterations of the pose refinement.
pub const REFINE_ITERATIONS: i64 = 20;

/// Iterations allowed in nalgebra's symmetric eigen-decomposition (EPnP's control points and
/// null space) before it is abandoned.
const MAX_ITERATIONS: usize = 10_000;

/// The control points' six pairs, in the rows of EPnP's distance system.
const PAIRS: [(usize, usize); 6] = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
/// The ten products `b_k b_l`, in the columns of EPnP's distance system.
const MONOMIALS: [(usize, usize); 10] = [
    (0, 0),
    (0, 1),
    (1, 1),
    (0, 2),
    (1, 2),
    (2, 2),
    (0, 3),
    (1, 3),
    (2, 3),
    (3, 3),
];
/// The columns each linearisation keeps (`_APPROXIMATIONS` in Python): N = 4, 2 and 3.
const APPROXIMATIONS: [&[usize]; 3] = [&[0, 1, 3, 6], &[0, 1, 2], &[0, 1, 2, 3, 4]];

/// EPnP's distance system: `L (b_k b_l) = rho` keeps the control-point distances.
struct DistanceSystem {
    /// `L`, 6 x 10.
    system: SMatrix<f64, 6, 10>,
    /// `rho`, the world control points' squared distances.
    rho: SVector<f64, 6>,
}

/// Squared normalised-plane reprojection errors; infinity behind the camera.
///
/// # Arguments
///
/// * `pose` - `T_cam_world`.
/// * `points` - World points.
/// * `image` - Their bearings' normalised image points.
///
/// # Returns
///
/// The errors.
#[must_use]
pub fn reprojection(
    pose: &Matrix4<f64>,
    points: &[Vector3<f64>],
    image: &[Vector2<f64>],
) -> DVector<f64> {
    let (rotation, translation) = (se3::rotation(pose), se3::translation(pose));
    DVector::from_iterator(
        points.len(),
        points.iter().zip(image).map(|(x, observed)| {
            let p = rotation * x + translation;
            if p[2] > 0.0 {
                (p.xy() / p[2] - observed).norm_squared()
            } else {
                f64::INFINITY
            }
        }),
    )
}

/// The eigen-decomposition of a symmetric matrix, eigenvalues ascending (as Eigen's
/// `SelfAdjointEigenSolver` and numpy's `eigh` return them).
///
/// # Arguments
///
/// * `matrix` - The matrix, finite.
///
/// # Returns
///
/// The eigenvalues and the eigenvectors as columns, or `None` if it does not converge.
fn ascending_eigen(matrix: DMatrix<f64>) -> Option<(DVector<f64>, DMatrix<f64>)> {
    let eigen = SymmetricEigen::try_new(matrix, f64::EPSILON, MAX_ITERATIONS)?;
    let mut order: Vec<usize> = (0..eigen.eigenvalues.len()).collect();
    order.sort_by(|&a, &b| eigen.eigenvalues[a].total_cmp(&eigen.eigenvalues[b]));
    let values = DVector::from_iterator(order.len(), order.iter().map(|&i| eigen.eigenvalues[i]));
    let vectors = eigen.eigenvectors.select_columns(&order);
    Some((values, vectors))
}

/// Flips each column so its largest-magnitude entry is positive.
///
/// # Arguments
///
/// * `vectors` - The eigenvectors, as columns.
///
/// # Returns
///
/// The signed eigenvectors.
fn deterministic_sign(mut vectors: DMatrix<f64>) -> DMatrix<f64> {
    for mut column in vectors.column_iter_mut() {
        if column[column.iamax()] < 0.0 {
            column.neg_mut();
        }
    }
    vectors
}

/// EPnP's four world control points: the centroid, then the principal axes scaled by their
/// spread.
///
/// # Arguments
///
/// * `points` - The world points.
///
/// # Returns
///
/// The control points, or `None` if the points are planar.
fn control_points(points: &[Vector3<f64>]) -> Option<[Vector3<f64>; 4]> {
    #[allow(
        clippy::cast_precision_loss,
        reason = "a point count is far below 2^52"
    )]
    let n = points.len() as f64;
    let centroid = points.iter().sum::<Vector3<f64>>() / n;
    let covariance: Matrix3<f64> = points
        .iter()
        .map(|p| (p - centroid) * (p - centroid).transpose())
        .sum();
    let (values, vectors) =
        ascending_eigen(DMatrix::from_column_slice(3, 3, covariance.as_slice()))?;
    if !(values[0] > PLANAR_TOLERANCE * values[2]) {
        return None;
    }
    // The axes largest spread first.
    let axes = deterministic_sign(vectors.select_columns(&[2, 1, 0]));
    let mut out = [centroid; 4];
    for k in 0..3 {
        let spread = (values[2 - k] / n).sqrt();
        out[k + 1] = centroid + spread * Vector3::from_column_slice(axes.column(k).as_slice());
    }
    Some(out)
}

/// EPnP's distance system for a null-space basis.
///
/// # Arguments
///
/// * `basis` - The null-space vectors as rows of 12 (4 x 12).
/// * `controls` - The world control points.
///
/// # Returns
///
/// `L` and `rho`.
fn distance_system(basis: &SMatrix<f64, 4, 12>, controls: &[Vector3<f64>; 4]) -> DistanceSystem {
    let mut out = DistanceSystem {
        system: SMatrix::zeros(),
        rho: SVector::zeros(),
    };
    for (row, &(a, b)) in PAIRS.iter().enumerate() {
        let dv: [Vector3<f64>; 4] = std::array::from_fn(|k| {
            Vector3::from_fn(|c, _| basis[(k, 3 * a + c)] - basis[(k, 3 * b + c)])
        });
        for (column, &(k, m)) in MONOMIALS.iter().enumerate() {
            let factor = if k == m { 1.0 } else { 2.0 };
            out.system[(row, column)] = factor * dv[k].dot(&dv[m]);
        }
        out.rho[row] = (controls[a] - controls[b]).norm_squared();
    }
    out
}

/// A least-squares solve with numpy's `lstsq` cutoff (`max(m, n) ε`).
///
/// # Arguments
///
/// * `matrix` - A.
/// * `rhs` - b.
///
/// # Returns
///
/// x, or zeros if the SVD fails (it cannot on the finite systems EPnP forms).
#[allow(
    clippy::cast_precision_loss,
    reason = "a matrix dimension is far below 2^52"
)]
fn lstsq(matrix: &DMatrix<f64>, rhs: &DVector<f64>) -> DVector<f64> {
    let relative = matrix.nrows().max(matrix.ncols()) as f64 * f64::EPSILON;
    let columns = matrix.ncols();
    least_squares(matrix, rhs, relative).unwrap_or_else(|| DVector::zeros(columns))
}

/// One linearisation of the distance equations, factored back into betas.
///
/// # Arguments
///
/// * `distances` - The distance system.
/// * `kind` - 0 (N = 4), 1 (N = 2) or 2 (N = 3).
///
/// # Returns
///
/// The betas.
fn initial_betas(distances: &DistanceSystem, kind: usize) -> Vector4<f64> {
    let kept = DMatrix::from_fn(6, APPROXIMATIONS[kind].len(), |r, c| {
        distances.system[(r, APPROXIMATIONS[kind][c])]
    });
    let products = lstsq(&kept, &DVector::from_column_slice(distances.rho.as_slice()));
    let mut betas = Vector4::zeros();
    if kind == 0 {
        let first = products[0].abs().sqrt();
        let sign = if products[0] < 0.0 { -1.0 } else { 1.0 };
        betas[0] = first;
        if first > 0.0 {
            for k in 1..4 {
                betas[k] = sign * products[k] / first;
            }
        }
        return betas;
    }
    if products[0] < 0.0 {
        betas[0] = (-products[0]).sqrt();
        betas[1] = if products[2] < 0.0 {
            (-products[2]).sqrt()
        } else {
            0.0
        };
    } else {
        betas[0] = products[0].sqrt();
        betas[1] = if products[2] > 0.0 {
            products[2].sqrt()
        } else {
            0.0
        };
    }
    if products[1] < 0.0 {
        betas[0] = -betas[0];
    }
    if kind == 2 && betas[0] != 0.0 {
        betas[2] = products[3] / betas[0];
    }
    betas
}

/// The ten products `b_k b_l`.
///
/// # Arguments
///
/// * `betas` - The betas.
///
/// # Returns
///
/// The products, in [`MONOMIALS`] order.
fn monomials(betas: &Vector4<f64>) -> SVector<f64, 10> {
    SVector::from_fn(|column, _| {
        let (k, m) = MONOMIALS[column];
        betas[k] * betas[m]
    })
}

/// Below this, relative to the largest, an `R` diagonal entry sends a beta step to the SVD.
/// The diagonal of an unpivoted R does not bound J's conditioning, so this is a heuristic,
/// chosen by measurement (PR 6 panel, run 2, L9): over 6000 EPnP scenes it matched `lstsq` on
/// every generic, near-planar and near-collinear one, and differed on 23 of 1500 scenes a few
/// micrometres to centimetres across, where every backend (C++ included) already disagrees
/// with Python's at roundoff.
const QR_RANK_CUTOFF: f64 = 1e-4;

/// One Gauss-Newton step of the beta polish, `argmin |J δ - r|`.
///
/// A J whose R has every diagonal entry above [`QR_RANK_CUTOFF`] of the largest is solved by
/// Householder QR, backward stable for full-rank least squares, which agrees with the SVD
/// solution to within ε κ(J); anything nearer rank deficiency takes the minimum-norm SVD
/// solve, numpy's `lstsq`. The QR path keeps faer's per-call cost off these tiny systems
/// (several hundred per PnP RANSAC run).
///
/// # Arguments
///
/// * `jacobian` - J.
/// * `residual` - r.
///
/// # Returns
///
/// δ.
fn beta_step(jacobian: &Matrix6x4<f64>, residual: &SVector<f64, 6>) -> Vector4<f64> {
    let qr = jacobian.qr();
    let r = qr.r();
    let diagonal = r.diagonal().map(f64::abs);
    if diagonal.min() > QR_RANK_CUTOFF * diagonal.max()
        && let Some(step) = r.solve_upper_triangular(&(qr.q().transpose() * residual))
    {
        return step;
    }
    let step = lstsq(
        &DMatrix::from_column_slice(6, 4, jacobian.as_slice()),
        &DVector::from_column_slice(residual.as_slice()),
    );
    Vector4::from_column_slice(step.as_slice())
}

/// Polishes the betas by Gauss-Newton on the distance equations.
///
/// # Arguments
///
/// * `betas` - The betas.
/// * `distances` - The distance system.
///
/// # Returns
///
/// The polished betas.
fn polish_betas(mut betas: Vector4<f64>, distances: &DistanceSystem) -> Vector4<f64> {
    let mut residual = distances.rho - distances.system * monomials(&betas);
    for _ in 0..POLISH_MAX_STEPS {
        let mut jacobian = Matrix6x4::zeros();
        for (column, &(k, m)) in MONOMIALS.iter().enumerate() {
            let mut col_k = jacobian.column_mut(k);
            col_k += distances.system.column(column) * betas[m];
            let mut col_m = jacobian.column_mut(m);
            col_m += distances.system.column(column) * betas[k];
        }
        let step = beta_step(&jacobian, &residual);
        let moved = betas + step;
        let moved_residual = distances.rho - distances.system * monomials(&moved);
        if !(moved_residual.norm() < residual.norm()) {
            break;
        }
        let step_norm = step.norm();
        betas = moved;
        residual = moved_residual;
        if step_norm <= POLISH_TOLERANCE * betas.norm() {
            break;
        }
    }
    betas
}

/// EPnP on checked correspondences (unchecked).
///
/// # Arguments
///
/// * `points` - World points, N >= 5.
/// * `bearings` - Their unit bearings, positive z.
///
/// # Returns
///
/// The pose, or `None` if the points are planar or degenerate.
#[must_use]
pub fn solve_epnp(points: &[Vector3<f64>], bearings: &[Vector3<f64>]) -> Option<Matrix4<f64>> {
    let controls = control_points(points)?;
    let control_homogeneous =
        Matrix4::from_fn(|r, c| if c == 0 { 1.0 } else { controls[r][c - 1] });
    let inverse = control_homogeneous.try_inverse()?;
    let alphas: Vec<RowVector4<f64>> = points
        .iter()
        .map(|p| RowVector4::new(1.0, p[0], p[1], p[2]) * inverse)
        .collect();
    let image = normalised_xy(bearings);
    let n = points.len();
    let mut system = DMatrix::zeros(2 * n, 12);
    for (i, (alpha, x)) in alphas.iter().zip(&image).enumerate() {
        for j in 0..4 {
            system[(2 * i, 3 * j)] = alpha[j];
            system[(2 * i, 3 * j + 2)] = -alpha[j] * x[0];
            system[(2 * i + 1, 3 * j + 1)] = alpha[j];
            system[(2 * i + 1, 3 * j + 2)] = -alpha[j] * x[1];
        }
    }
    let (_, vectors) = ascending_eigen(system.transpose() * &system)?;
    let null = deterministic_sign(vectors.columns(0, 4).into_owned());
    let basis = SMatrix::<f64, 4, 12>::from_fn(|r, c| null[(c, r)]);
    let distances = distance_system(&basis, &controls);
    let mut best: Option<(f64, Matrix4<f64>)> = None;
    for kind in 0..APPROXIMATIONS.len() {
        let betas = polish_betas(initial_betas(&distances, kind), &distances);
        let combined = betas.transpose() * basis;
        let camera_controls: [Vector3<f64>; 4] =
            std::array::from_fn(|j| Vector3::from_fn(|c, _| combined[3 * j + c]));
        let mut camera: Vec<Vector3<f64>> = alphas
            .iter()
            .map(|alpha| (0..4).map(|j| camera_controls[j] * alpha[j]).sum())
            .collect();
        if camera.iter().map(|p| p[2]).sum::<f64>() < 0.0 {
            for p in &mut camera {
                *p = -*p;
            }
        }
        let Ok(fitted) = alignment::umeyama(points, &camera, &vec![1.0; n], false) else {
            continue;
        };
        let error = reprojection(&fitted, points, &image).sum();
        if best.as_ref().is_none_or(|(lowest, _)| error < *lowest) {
            best = Some((error, fitted));
        }
    }
    best.map(|(_, pose)| pose)
}

/// Estimates the camera pose from five or more correspondences by EPnP.
///
/// # Arguments
///
/// * `points` - World points, spanning three dimensions.
/// * `bearings` - Their unit bearings, positive z.
///
/// # Returns
///
/// `T_cam_world`.
///
/// # Errors
///
/// `InvalidInput` for invalid or too few correspondences; `Degenerate` for planar points.
pub fn epnp(points: &[Vector3<f64>], bearings: &[Vector3<f64>]) -> Result<Matrix4<f64>> {
    check_correspondences(points, bearings, EPNP_MINIMUM, true)?;
    match solve_epnp(points, bearings) {
        Some(pose) => Ok(pose),
        None => fail(
            ErrorKind::Degenerate,
            "epnp: the points are planar, the control points undetermined",
        ),
    }
}

/// Least squares on the reprojection errors of correspondences, over SE(3) (see
/// `slamlab.foundations.pnp.ReprojectionProblem`).
struct ReprojectionProblem {
    /// The world points.
    points: Vec<Vector3<f64>>,
    /// Their normalised image points.
    image: Vec<Vector2<f64>>,
}

impl nls::Problem for ReprojectionProblem {
    type State = Matrix4<f64>;
    type System = nls::DenseSystem;

    /// Each block's squared residual; infinity behind the camera, so a step that moves a
    /// point there is rejected (cheirality).
    fn squared_norms(&self, state: &Matrix4<f64>) -> DVector<f64> {
        reprojection(state, &self.points, &self.image)
    }

    fn linearize(&self, state: &Matrix4<f64>, weights: &DVector<f64>) -> nls::DenseSystem {
        let (rotation, translation) = (se3::rotation(state), se3::translation(state));
        let n = self.points.len();
        let mut jacobian = DMatrix::zeros(2 * n, 6);
        let mut residuals = DVector::zeros(2 * n);
        for (i, (world, observed)) in self.points.iter().zip(&self.image).enumerate() {
            let p = rotation * world + translation;
            #[rustfmt::skip] // keep one matrix row per line
            let projection = SMatrix::<f64, 2, 3>::new(
                1.0 / p[2], 0.0, -p[0] / (p[2] * p[2]),
                0.0, 1.0 / p[2], -p[1] / (p[2] * p[2]),
            );
            let mut motion = SMatrix::<f64, 3, 6>::zeros();
            motion.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
            motion
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&(-rotation * so3::hat(world)));
            jacobian
                .fixed_view_mut::<2, 6>(2 * i, 0)
                .copy_from(&(projection * motion));
            residuals
                .fixed_rows_mut::<2>(2 * i)
                .copy_from(&(p.xy() / p[2] - observed));
        }
        nls::DenseSystem::build(&jacobian, &residuals, weights)
            .expect("two residuals and one weight per correspondence")
    }

    fn retract(&self, state: &Matrix4<f64>, delta: &DVector<f64>) -> Matrix4<f64> {
        state * se3::exp(&Vector6::from_column_slice(&delta.as_slice()[..6]))
    }
}

/// Refines a pose by Levenberg-Marquardt on the Cauchy-robust reprojection error
/// (unchecked).
///
/// # Arguments
///
/// * `pose` - The starting `T_cam_world`.
/// * `points` - World points.
/// * `image` - Their bearings' normalised image points.
/// * `scale` - The Cauchy loss's scale, in the normalised plane.
///
/// # Returns
///
/// The refined pose and the solve's summary.
///
/// # Errors
///
/// `InvalidInput` if a point is behind the starting camera (an infinite start).
pub fn refine(
    pose: &Matrix4<f64>,
    points: Vec<Vector3<f64>>,
    image: Vec<Vector2<f64>>,
    scale: f64,
) -> Result<nls::Solution<Matrix4<f64>>> {
    let problem = ReprojectionProblem { points, image };
    nls::solve_cauchy(&problem, *pose, scale, REFINE_ITERATIONS)
}

/// Refines a pose by minimising the inliers' robust reprojection error over SE(3).
///
/// # Arguments
///
/// * `pose` - The starting `T_cam_world`.
/// * `points` - The inliers' world points, M >= 3.
/// * `bearings` - Their unit bearings, positive z.
/// * `scale` - The Cauchy loss's scale, in the normalised plane (positive).
///
/// # Returns
///
/// The refined pose and the summary.
///
/// # Errors
///
/// `InvalidInput` for an invalid pose or correspondences, M < 3, a non-positive scale, or a
/// point behind the start.
pub fn refine_pose(
    pose: &Matrix4<f64>,
    points: &[Vector3<f64>],
    bearings: &[Vector3<f64>],
    scale: f64,
) -> Result<nls::Solution<Matrix4<f64>>> {
    if !pose.iter().all(|x| x.is_finite()) {
        return fail(
            ErrorKind::InvalidInput,
            "pose: expected one finite (4, 4) matrix",
        );
    }
    se3::check_transform(pose)?;
    check_correspondences(points, bearings, p3p::SAMPLE_SIZE, true)?;
    if !scale.is_finite() || scale <= 0.0 {
        return fail_setting("scale", "must be positive and finite", scale);
    }
    let (rotation, translation) = (se3::rotation(pose), se3::translation(pose));
    if !points.iter().all(|x| (rotation * x + translation)[2] > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            "pose: a point is behind the starting camera",
        );
    }
    refine(pose, points.to_vec(), normalised_xy(bearings), scale)
}

/// P3P and EPnP with the reprojection error, as a [`ransac::Estimator`].
pub struct PoseEstimator {
    /// The world points.
    points: Vec<Vector3<f64>>,
    /// Their bearings.
    bearings: Vec<Vector3<f64>>,
    /// Their normalised image points.
    image: Vec<Vector2<f64>>,
    /// The P3P solver.
    solver: Solver,
}

impl PoseEstimator {
    /// Holds checked correspondences.
    ///
    /// # Arguments
    ///
    /// * `points` - World points.
    /// * `bearings` - Their unit bearings, positive z.
    /// * `solver` - The P3P solver.
    ///
    /// # Returns
    ///
    /// The estimator.
    #[must_use]
    pub fn new(points: &[Vector3<f64>], bearings: &[Vector3<f64>], solver: Solver) -> Self {
        Self {
            points: points.to_vec(),
            bearings: bearings.to_vec(),
            image: normalised_xy(bearings),
            solver,
        }
    }
}

impl Estimator for PoseEstimator {
    type Model = Matrix4<f64>;

    fn sample_size(&self) -> usize {
        p3p::SAMPLE_SIZE
    }

    fn size(&self) -> usize {
        self.points.len()
    }

    fn fit_minimal(&self, indices: &[usize]) -> Vec<Matrix4<f64>> {
        p3p::solve(
            &gather(&self.points, indices),
            &gather(&self.bearings, indices),
            self.solver,
        )
    }

    fn fit_nonminimal(&self, indices: &[usize]) -> Vec<Matrix4<f64>> {
        if indices.len() < EPNP_MINIMUM {
            return Vec::new();
        }
        solve_epnp(
            &gather(&self.points, indices),
            &gather(&self.bearings, indices),
        )
        .into_iter()
        .collect()
    }

    fn residuals(&self, model: &Matrix4<f64>) -> DVector<f64> {
        reprojection(model, &self.points, &self.image)
    }
}

/// Estimates the camera pose robustly: LO-MSAC with a P3P solver and EPnP refits, then
/// (optionally) the refinement on the inliers, kept only if it lowers the MSAC cost.
///
/// # Arguments
///
/// * `points` - World points, N >= 3.
/// * `bearings` - Their unit bearings, positive z.
/// * `options` - RANSAC's settings; the threshold is in the normalised plane.
/// * `minimal` - The P3P solver.
/// * `refine` - Refine the consensus.
///
/// # Returns
///
/// The pose, its inliers and the summary.
///
/// # Errors
///
/// `InvalidInput` for invalid input or settings; `Degenerate` if no sample gave a pose.
pub fn pnp_ransac(
    points: &[Vector3<f64>],
    bearings: &[Vector3<f64>],
    options: &Options,
    minimal: Solver,
    refine: bool,
) -> Result<RefinedSolution<Matrix4<f64>>> {
    check_correspondences(points, bearings, p3p::SAMPLE_SIZE, true)?;
    let image = normalised_xy(bearings);
    let refiner = |model: &Matrix4<f64>, inliers: &[usize]| {
        self::refine(
            model,
            gather(points, inliers),
            gather(&image, inliers),
            options.threshold,
        )
    };
    ransac::estimate_refined(
        &PoseEstimator::new(points, bearings, minimal),
        options,
        if refine { Some(&refiner) } else { None },
        p3p::SAMPLE_SIZE,
    )
}

#[cfg(test)]
mod tests {
    //! Native tests: EPnP on noise-free correspondences, and PnP RANSAC through outliers.

    use super::*;

    /// Correspondences through a known pose.
    fn scene(n: u32) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>, Matrix4<f64>) {
        let pose = se3::exp(&Vector6::new(0.1, -0.2, 0.3, 0.2, -0.1, 0.05));
        let (mut points, mut bearings) = (Vec::new(), Vec::new());
        for i in 0..n {
            let t = f64::from(i);
            let point = Vector3::new(
                1.5 * (0.7 * t).sin(),
                (1.3 * t).cos(),
                4.0 + 1.5 * (0.45 * t).sin(),
            );
            points.push(point);
            bearings.push(se3::act(&pose, &point).unwrap().normalize());
        }
        (points, bearings, pose)
    }

    /// EPnP recovers the pose exactly from noise-free correspondences.
    #[test]
    fn epnp_recovers_the_pose() {
        let (points, bearings, pose) = scene(20);
        assert!((epnp(&points, &bearings).unwrap() - pose).amax() < 1e-9);
    }

    /// PnP RANSAC keeps every inlier through 30% outliers and converges.
    #[test]
    fn pnp_ransac_through_outliers() {
        let (points, mut bearings, pose) = scene(50);
        for (i, b) in bearings.iter_mut().enumerate() {
            if i % 10 < 3 {
                *b = Vector3::new(0.2, 0.1, 1.0).normalize();
            }
        }
        let options = Options {
            threshold: 1e-6,
            seed: 5,
            ..Options::default()
        };
        let found = pnp_ransac(&points, &bearings, &options, Solver::Ding, true).unwrap();
        assert_eq!(found.summary.run.inlier_count, 35);
        assert!((found.state.model - pose).amax() < 1e-9);
    }

    /// The unchecked refinement from a pose with a point behind the camera is an error (an
    /// infinite start), not a panic (panel A2).
    #[test]
    fn a_point_behind_the_start_is_an_error() {
        let (mut points, bearings, pose) = scene(10);
        let centre = -se3::rotation(&pose).transpose() * se3::translation(&pose);
        points[0] = 2.0 * centre - points[0]; // mirrored through the camera centre
        let failed = refine(&pose, points, normalised_xy(&bearings), 1e-3).unwrap_err();
        assert_eq!(failed.kind, ErrorKind::InvalidInput);
    }

    /// A rank-deficient beta step takes the SVD fallback: the minimum-norm solution, as numpy's
    /// `lstsq` gives it, not QR's (panel run 2, L9).
    #[test]
    fn a_rank_deficient_beta_step_takes_the_minimum_norm_solution() {
        // Columns 0 and 1 are equal, so R's second diagonal entry is zero.
        let column = SVector::<f64, 6>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mut jacobian = Matrix6x4::zeros();
        jacobian.set_column(0, &column);
        jacobian.set_column(1, &column);
        jacobian.set_column(2, &SVector::<f64, 6>::new(0.0, 1.0, 0.0, 1.0, 0.0, 1.0));
        jacobian.set_column(3, &SVector::<f64, 6>::new(1.0, 0.0, 1.0, 0.0, 1.0, 0.0));
        let residual = column * 2.0;
        let step = beta_step(&jacobian, &residual);
        // x0 + x1 = 2 is determined, x0 - x1 is not: the minimum norm splits it evenly.
        assert!((step - Vector4::new(1.0, 1.0, 0.0, 0.0)).amax() < 1e-12);
    }
}
