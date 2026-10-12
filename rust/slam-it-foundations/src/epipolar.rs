//! Epipolar geometry: the Rust core's 8-point essential and fundamental matrices, their
//! Sampson errors, the refinement of E on its manifold, and their RANSAC estimators.
//!
//! Mirrors `slam_it.foundations.epipolar`, whose docstring states the maths: the 8-point
//! algorithm (normalised for pixels), projection onto the essential manifold or rank 2, and
//! the Sampson error (in the normalised plane for E, in pixels for F). E is returned with
//! singular values (1, 1, 0), F with unit Frobenius norm, both up to sign.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix3x2, Matrix4, Vector2, Vector3};

use crate::correspondences::{
    RANK_TOLERANCE, check_bearing_pairs, check_point_pairs, fit_sample, gather,
    hartley_normalisation, homogeneous, normalised_plane, null_vector,
};
use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::linalg::{singular_values3, svd3};
use crate::ransac::{self, Estimator, Options, RefinedSolution};
use crate::{fivepoint, nls, se3, so3};

/// Correspondences the 8-point algorithm needs (its minimal sample).
pub const SAMPLE_SIZE: usize = 8;
/// Levenberg-Marquardt iterations of the Sampson refinement.
pub const REFINE_ITERATIONS: i64 = 20;

/// E-RANSAC's minimal solver, with the codes it crosses the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MinimalSolver {
    /// Nistér's 5-point solver.
    Nister = 0,
    /// Stewénius' 5-point solver.
    Stewenius = 1,
    /// The 8-point algorithm.
    EightPoint = 2,
}

impl MinimalSolver {
    /// Every solver, in code order.
    pub const ALL: [Self; 3] = [Self::Nister, Self::Stewenius, Self::EightPoint];

    /// The solver's name, as `slam_it.foundations.epipolar.MINIMAL_SOLVERS` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Nister => "nister",
            Self::Stewenius => "stewenius",
            Self::EightPoint => "8pt",
        }
    }

    /// The minimal sample size.
    ///
    /// # Returns
    ///
    /// Five, or eight for the 8-point algorithm.
    #[must_use]
    pub const fn sample_size(self) -> usize {
        match self {
            Self::EightPoint => SAMPLE_SIZE,
            Self::Nister | Self::Stewenius => fivepoint::SAMPLE_SIZE,
        }
    }
}

/// Stacks `x2 ⊗ x1` per correspondence: the rows of `A vec(M) = 0` (vec row-major).
///
/// # Arguments
///
/// * `x1`, `x2` - Homogeneous points in images 1 and 2.
///
/// # Returns
///
/// `A` (N x 9).
fn epipolar_system(x1: &[Vector3<f64>], x2: &[Vector3<f64>]) -> DMatrix<f64> {
    let mut system = DMatrix::zeros(x1.len(), 9);
    for (i, (p1, p2)) in x1.iter().zip(x2).enumerate() {
        for r in 0..3 {
            for c in 0..3 {
                system[(i, 3 * r + c)] = p2[r] * p1[c];
            }
        }
    }
    system
}

/// The squared Sampson error of homogeneous point pairs (z = 1) under E or F.
///
/// # Arguments
///
/// * `matrix` - E (on normalised points) or F (on pixels).
/// * `x1`, `x2` - The points in images 1 and 2.
///
/// # Returns
///
/// One error per pair; infinity where the denominator vanishes.
#[must_use]
pub fn sampson(matrix: &Matrix3<f64>, x1: &[Vector3<f64>], x2: &[Vector3<f64>]) -> DVector<f64> {
    DVector::from_iterator(
        x1.len(),
        x1.iter().zip(x2).map(|(p1, p2)| {
            let line2 = matrix * p1;
            let line1 = matrix.transpose() * p2;
            let algebraic = p2.dot(&line2);
            let gradient = line2.xy().norm_squared() + line1.xy().norm_squared();
            if gradient > 0.0 {
                algebraic * algebraic / gradient
            } else {
                f64::INFINITY
            }
        }),
    )
}

/// The 8-point system solved on Hartley-normalised points: the solution in normalised
/// coordinates, and the two similarities.
///
/// # Arguments
///
/// * `points1`, `points2` - Points in images 1 and 2.
///
/// # Returns
///
/// `(raw, T1, T2)`, or `None` if every point coincides or the solution is not unique.
fn normalised_8pt(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
) -> Option<(Matrix3<f64>, Matrix3<f64>, Matrix3<f64>)> {
    let t1 = hartley_normalisation(points1)?;
    let t2 = hartley_normalisation(points2)?;
    let x1: Vec<Vector3<f64>> = homogeneous(points1).iter().map(|p| t1 * p).collect();
    let x2: Vec<Vector3<f64>> = homogeneous(points2).iter().map(|p| t2 * p).collect();
    let solution = null_vector(&epipolar_system(&x1, &x2))?;
    Some((Matrix3::from_row_slice(solution.as_slice()), t1, t2))
}

/// The points of the normalised image plane, without their ones.
///
/// # Arguments
///
/// * `bearings` - Bearings with positive z.
///
/// # Returns
///
/// `(x, y)` per bearing.
fn plane_xy(bearings: &[Vector3<f64>]) -> Vec<Vector2<f64>> {
    normalised_plane(bearings).iter().map(Vector3::xy).collect()
}

/// The normalised 8-point E on the bearings' normalised image points, projected onto the
/// essential manifold (unchecked).
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - Bearings, N >= 8.
///
/// # Returns
///
/// E, or `None` if the pairs do not determine it.
#[must_use]
pub fn fit_essential(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
) -> Option<Matrix3<f64>> {
    let (raw, t1, t2) = normalised_8pt(&plane_xy(bearings1), &plane_xy(bearings2))?;
    // Project after mapping back: the normalisation does not preserve the essential structure.
    let (u, _, v_t) = svd3(&(t2.transpose() * raw * t1))?;
    Some(u * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 0.0)) * v_t)
}

/// The normalised 8-point F on pixel pairs, rank 2, unit norm (unchecked).
///
/// # Arguments
///
/// * `pixels1`, `pixels2` - Pixels, N >= 8.
///
/// # Returns
///
/// F, or `None` if the pairs do not determine it.
#[must_use]
pub fn fit_fundamental(pixels1: &[Vector2<f64>], pixels2: &[Vector2<f64>]) -> Option<Matrix3<f64>> {
    let (raw, t1, t2) = normalised_8pt(pixels1, pixels2)?;
    let (u, s, v_t) = svd3(&raw)?;
    let rank2 = u * Matrix3::from_diagonal(&Vector3::new(s[0], s[1], 0.0)) * v_t;
    let fundamental = t2.transpose() * rank2 * t1;
    Some(fundamental / fundamental.norm())
}

/// Estimates E from eight or more bearing pairs: `x2ᵀ E x1 = 0` for each pair's normalised
/// image points, camera 1 then camera 2, so E = [t]ₓ R for `T_c2_c1 = (R, t)`.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - The pairs' unit bearings in cameras 1 and 2, positive z.
///
/// # Returns
///
/// E.
///
/// # Errors
///
/// `InvalidInput` for bad bearings or N < 8; `Degenerate` if not unique.
pub fn essential_8pt(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
) -> Result<Matrix3<f64>> {
    check_bearing_pairs(bearings1, bearings2, SAMPLE_SIZE, true)?;
    match fit_essential(bearings1, bearings2) {
        Some(essential) => Ok(essential),
        None => fail(
            ErrorKind::Degenerate,
            "E: the correspondences do not determine a unique solution",
        ),
    }
}

/// Estimates F from eight or more pixel pairs (normalised 8-point): `x2ᵀ F x1 = 0` for each
/// pair's homogeneous pixels, image 1 then image 2.
///
/// # Arguments
///
/// * `pixels1`, `pixels2` - The pairs' pixels in images 1 and 2.
///
/// # Returns
///
/// F.
///
/// # Errors
///
/// `InvalidInput` for mismatched sizes or N < 8; `Degenerate` if not unique.
pub fn fundamental_8pt(pixels1: &[Vector2<f64>], pixels2: &[Vector2<f64>]) -> Result<Matrix3<f64>> {
    check_point_pairs(pixels1, pixels2, SAMPLE_SIZE, "pixels")?;
    match fit_fundamental(pixels1, pixels2) {
        Some(fundamental) => Ok(fundamental),
        None => fail(
            ErrorKind::Degenerate,
            "F: the correspondences do not determine a unique solution",
        ),
    }
}

/// Squared Sampson errors of bearing pairs under E (`x2ᵀ E x1 = 0`), in the normalised plane.
///
/// # Arguments
///
/// * `essential` - E.
/// * `bearings1`, `bearings2` - The pairs' unit bearings in cameras 1 and 2, positive z.
///
/// # Returns
///
/// One error per pair.
///
/// # Errors
///
/// `InvalidInput` for bad bearings.
pub fn sampson_essential(
    essential: &Matrix3<f64>,
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
) -> Result<DVector<f64>> {
    check_bearing_pairs(bearings1, bearings2, 0, true)?;
    Ok(sampson(
        essential,
        &normalised_plane(bearings1),
        &normalised_plane(bearings2),
    ))
}

/// Squared Sampson errors of pixel pairs under F, in pixels² (`x2ᵀ F x1 = 0`).
///
/// # Arguments
///
/// * `fundamental` - F.
/// * `pixels1`, `pixels2` - The pairs' pixels in images 1 and 2.
///
/// # Returns
///
/// One error per pair.
///
/// # Errors
///
/// `InvalidInput` for mismatched sizes or non-finite pixels.
pub fn sampson_fundamental(
    fundamental: &Matrix3<f64>,
    pixels1: &[Vector2<f64>],
    pixels2: &[Vector2<f64>],
) -> Result<DVector<f64>> {
    check_point_pairs(pixels1, pixels2, 0, "pixels")?;
    Ok(sampson(
        fundamental,
        &homogeneous(pixels1),
        &homogeneous(pixels2),
    ))
}

/// The 8-point or 5-point E solver and its Sampson error, as a [`ransac::Estimator`].
pub struct EssentialEstimator {
    /// The bearings in camera 1.
    bearings1: Vec<Vector3<f64>>,
    /// The bearings in camera 2.
    bearings2: Vec<Vector3<f64>>,
    /// Their normalised image points in camera 1.
    x1: Vec<Vector3<f64>>,
    /// Their normalised image points in camera 2.
    x2: Vec<Vector3<f64>>,
    /// The minimal solver.
    minimal: MinimalSolver,
}

impl EssentialEstimator {
    /// Holds checked bearing pairs.
    ///
    /// # Arguments
    ///
    /// * `bearings1`, `bearings2` - Unit bearings with positive z.
    /// * `minimal` - The minimal solver.
    ///
    /// # Returns
    ///
    /// The estimator.
    #[must_use]
    pub fn new(
        bearings1: &[Vector3<f64>],
        bearings2: &[Vector3<f64>],
        minimal: MinimalSolver,
    ) -> Self {
        Self {
            bearings1: bearings1.to_vec(),
            bearings2: bearings2.to_vec(),
            x1: normalised_plane(bearings1),
            x2: normalised_plane(bearings2),
            minimal,
        }
    }
}

impl Estimator for EssentialEstimator {
    type Model = Matrix3<f64>;

    fn sample_size(&self) -> usize {
        self.minimal.sample_size()
    }

    fn size(&self) -> usize {
        self.bearings1.len()
    }

    fn fit_minimal(&self, indices: &[usize]) -> Vec<Matrix3<f64>> {
        let solver = match self.minimal {
            MinimalSolver::EightPoint => return self.fit_nonminimal(indices),
            MinimalSolver::Nister => fivepoint::Solver::Nister,
            MinimalSolver::Stewenius => fivepoint::Solver::Stewenius,
        };
        fivepoint::solve(
            &gather(&self.bearings1, indices),
            &gather(&self.bearings2, indices),
            solver,
        )
    }

    fn fit_nonminimal(&self, indices: &[usize]) -> Vec<Matrix3<f64>> {
        fit_sample(fit_essential, &self.bearings1, &self.bearings2, indices)
    }

    fn residuals(&self, model: &Matrix3<f64>) -> DVector<f64> {
        sampson(model, &self.x1, &self.x2)
    }
}

/// The normalised 8-point F solver and its Sampson error, as a [`ransac::Estimator`].
pub struct FundamentalEstimator {
    /// The pixels in image 1.
    pixels1: Vec<Vector2<f64>>,
    /// The pixels in image 2.
    pixels2: Vec<Vector2<f64>>,
    /// Their homogeneous points in image 1.
    x1: Vec<Vector3<f64>>,
    /// Their homogeneous points in image 2.
    x2: Vec<Vector3<f64>>,
}

impl FundamentalEstimator {
    /// Holds checked pixel pairs.
    ///
    /// # Arguments
    ///
    /// * `pixels1`, `pixels2` - Pixels.
    ///
    /// # Returns
    ///
    /// The estimator.
    #[must_use]
    pub fn new(pixels1: &[Vector2<f64>], pixels2: &[Vector2<f64>]) -> Self {
        Self {
            pixels1: pixels1.to_vec(),
            pixels2: pixels2.to_vec(),
            x1: homogeneous(pixels1),
            x2: homogeneous(pixels2),
        }
    }
}

impl Estimator for FundamentalEstimator {
    type Model = Matrix3<f64>;

    fn sample_size(&self) -> usize {
        SAMPLE_SIZE
    }

    fn size(&self) -> usize {
        self.pixels1.len()
    }

    fn fit_nonminimal(&self, indices: &[usize]) -> Vec<Matrix3<f64>> {
        fit_sample(fit_fundamental, &self.pixels1, &self.pixels2, indices)
    }

    fn residuals(&self, model: &Matrix3<f64>) -> DVector<f64> {
        sampson(model, &self.x1, &self.x2)
    }
}

/// The four relative poses an essential matrix allows: `(R1, t)`, `(R1, -t)`, `(R2, t)`,
/// `(R2, -t)` with `R = U W Vᵀ` or `U Wᵀ Vᵀ` and `t = u3` (unit).
///
/// # Arguments
///
/// * `essential` - E (any scale).
///
/// # Returns
///
/// The candidates `T_c2_c1`.
///
/// # Errors
///
/// `InvalidInput` if not finite; `Degenerate` if its rank is below 2.
pub fn decompose_essential(essential: &Matrix3<f64>) -> Result<Vec<Matrix4<f64>>> {
    if !essential.iter().all(|x| x.is_finite()) {
        return fail(
            ErrorKind::InvalidInput,
            "essential: expected a finite (3, 3) matrix",
        );
    }
    let Some((mut u, singular, v_t)) = svd3(essential) else {
        return fail(ErrorKind::Degenerate, "essential: no decomposition");
    };
    if !(singular[1] > RANK_TOLERANCE * singular[0]) {
        return fail(
            ErrorKind::Degenerate,
            "essential: rank below 2, no translation direction",
        );
    }
    // E = U diag(1, 1, 0) Vᵀ holds with either sign of U or V, so make both proper rotations.
    let mut v = v_t.transpose();
    if u.determinant() < 0.0 {
        u = -u;
    }
    if v.determinant() < 0.0 {
        v = -v;
    }
    #[rustfmt::skip] // keep one matrix row per line
    let w = Matrix3::new(
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    );
    let t: Vector3<f64> = u.column(2).into_owned();
    let mut poses = Vec::with_capacity(4);
    for rotation in [u * w * v.transpose(), u * w.transpose() * v.transpose()] {
        for sign in [1.0, -1.0] {
            poses.push(se3::from_parts(&rotation, &(t * sign)));
        }
    }
    Ok(poses)
}

/// E's motion on its manifold: `E = [t]x R` with |t| = 1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motion {
    /// R.
    pub rotation: Matrix3<f64>,
    /// The unit translation t.
    pub translation: Vector3<f64>,
}

impl Motion {
    /// The motion's E.
    ///
    /// # Returns
    ///
    /// `[t]x R`: singular values (1, 1, 0).
    #[must_use]
    pub fn essential(&self) -> Matrix3<f64> {
        so3::hat(&self.translation) * self.rotation
    }
}

/// An orthonormal basis of the plane tangent to the unit sphere at a direction, built from the
/// coordinate axis least aligned with it (as in Python).
///
/// # Arguments
///
/// * `direction` - A unit vector.
///
/// # Returns
///
/// Two orthonormal columns perpendicular to it.
#[must_use]
pub fn tangent_basis(direction: &Vector3<f64>) -> Matrix3x2<f64> {
    let k = direction.iamin();
    let first = direction.cross(&Vector3::ith(k, 1.0)).normalize();
    Matrix3x2::from_columns(&[first, direction.cross(&first)])
}

/// The motion a refinement of E starts from: the candidate rotation with the largest trace,
/// and the translation's sign that makes its largest-magnitude component positive.
///
/// # Arguments
///
/// * `essential` - E, of rank 2.
///
/// # Returns
///
/// The start.
///
/// # Errors
///
/// [`decompose_essential`]'s: `InvalidInput` if E is not finite, `Degenerate` if its rank is
/// below 2.
pub fn start_motion(essential: &Matrix3<f64>) -> Result<Motion> {
    let candidates = decompose_essential(essential)?;
    // The first of the largest traces, as max_element (C++) and argmax (Python) pick it.
    let mut best = &candidates[0];
    for candidate in &candidates[1..] {
        if se3::rotation(candidate).trace() > se3::rotation(best).trace() {
            best = candidate;
        }
    }
    let mut translation = se3::translation(best);
    if translation[translation.iamax()] < 0.0 {
        translation = -translation;
    }
    Ok(Motion {
        rotation: se3::rotation(best),
        translation,
    })
}

/// One match's Sampson terms under E: the epipolar lines, `c = x2ᵀ E x1` and `g`, the sum of
/// both lines' first two coordinates squared; the residual is `c / √g`.
struct SampsonTerms {
    /// `E x1`.
    line2: Vector3<f64>,
    /// `Eᵀ x2`.
    line1: Vector3<f64>,
    /// The algebraic error.
    c: f64,
    /// The squared gradient norm.
    g: f64,
}

/// One match's Sampson terms.
///
/// # Arguments
///
/// * `e` - E.
/// * `x1`, `x2` - The points, z = 1.
///
/// # Returns
///
/// The terms.
fn sampson_terms(e: &Matrix3<f64>, x1: &Vector3<f64>, x2: &Vector3<f64>) -> SampsonTerms {
    let line2 = e * x1;
    let line1 = e.transpose() * x2;
    SampsonTerms {
        c: x2.dot(&line2),
        g: line2.xy().norm_squared() + line1.xy().norm_squared(),
        line2,
        line1,
    }
}

/// Least squares on the signed Sampson distance of E's inliers, over E's manifold (see
/// `slam_it.foundations.epipolar.SampsonProblem`).
struct SampsonProblem {
    /// Points in image 1, z = 1.
    x1: Vec<Vector3<f64>>,
    /// Points in image 2, z = 1.
    x2: Vec<Vector3<f64>>,
}

impl SampsonProblem {
    /// Each match's signed Sampson distance.
    ///
    /// # Arguments
    ///
    /// * `state` - The motion.
    ///
    /// # Returns
    ///
    /// The residuals.
    fn residuals(&self, state: &Motion) -> DVector<f64> {
        let e = state.essential();
        DVector::from_iterator(
            self.x1.len(),
            self.x1.iter().zip(&self.x2).map(|(x1, x2)| {
                let terms = sampson_terms(&e, x1, x2);
                terms.c / terms.g.sqrt()
            }),
        )
    }
}

impl nls::Problem for SampsonProblem {
    type State = Motion;
    type System = nls::DenseSystem;

    fn squared_norms(&self, state: &Motion) -> DVector<f64> {
        self.residuals(state).map(|r| r * r)
    }

    fn linearize(&self, state: &Motion, weights: &DVector<f64>) -> nls::DenseSystem {
        let e = state.essential();
        let t_hat = so3::hat(&state.translation);
        let basis = tangent_basis(&state.translation);
        // dE along each of (dθ, dt): [t]x R [e_k]x, then [b_j]x R.
        let directions: [Matrix3<f64>; 5] = std::array::from_fn(|k| {
            if k < 3 {
                t_hat * state.rotation * so3::hat(&Vector3::ith(k, 1.0))
            } else {
                so3::hat(&basis.column(k - 3).into_owned()) * state.rotation
            }
        });
        let n = self.x1.len();
        let mut jacobian = DMatrix::zeros(n, 5);
        let mut residual = DVector::zeros(n);
        for (i, (x1, x2)) in self.x1.iter().zip(&self.x2).enumerate() {
            let SampsonTerms { line2, line1, c, g } = sampson_terms(&e, x1, x2);
            let root = g.sqrt();
            // dc/dE = x2 x1ᵀ; dg/dE = 2 Σ_{k=0,1} (line2_k e_k x1ᵀ + line1_k x2 e_kᵀ).
            let mut dg = Matrix3::zeros();
            for k in 0..2 {
                let mut row = dg.row_mut(k);
                row += x1.transpose() * (2.0 * line2[k]);
                let mut column = dg.column_mut(k);
                column += x2 * (2.0 * line1[k]);
            }
            let dr = (x2 * x1.transpose()) / root - dg * (c / (2.0 * g * root));
            for (k, direction) in directions.iter().enumerate() {
                jacobian[(i, k)] = dr.component_mul(direction).sum();
            }
            residual[i] = c / root;
        }
        nls::DenseSystem::build(&jacobian, &residual, weights)
            .expect("one residual and one weight per match")
    }

    fn retract(&self, state: &Motion, delta: &DVector<f64>) -> Motion {
        let step = Vector2::new(delta[3], delta[4]);
        let moved = state.translation + tangent_basis(&state.translation) * step;
        Motion {
            rotation: state.rotation * so3::exp(&Vector3::new(delta[0], delta[1], delta[2])),
            translation: moved.normalize(),
        }
    }
}

/// Refines E by Levenberg-Marquardt on the Cauchy-robust Sampson distance (unchecked).
///
/// # Arguments
///
/// * `essential` - The starting E, of rank 2.
/// * `x1`, `x2` - The matches' normalised image points `(x, y, 1)`, M >= 5.
/// * `scale` - The Cauchy scale, a Sampson distance in the normalised plane.
///
/// # Returns
///
/// The refined E and the solve's summary.
///
/// # Errors
///
/// `InvalidInput` if a match's Sampson distance is not finite at the start, and
/// [`start_motion`]'s if E does not decompose.
pub fn refine(
    essential: &Matrix3<f64>,
    x1: Vec<Vector3<f64>>,
    x2: Vec<Vector3<f64>>,
    scale: f64,
) -> Result<nls::Solution<Matrix3<f64>>> {
    let problem = SampsonProblem { x1, x2 };
    let solved = nls::solve_cauchy(&problem, start_motion(essential)?, scale, REFINE_ITERATIONS)?;
    Ok(nls::Solution {
        state: solved.state.essential(),
        summary: solved.summary,
    })
}

/// Refines E on its inliers by minimising the robust Sampson distance over E's manifold.
///
/// # Arguments
///
/// * `essential` - The starting E.
/// * `bearings1`, `bearings2` - Unit bearings with positive z, M >= 5.
/// * `scale` - The Cauchy scale (positive).
///
/// # Returns
///
/// The refined E and the summary.
///
/// # Errors
///
/// `InvalidInput` for bad input, or a Sampson distance that is not finite at the start;
/// `Degenerate` if E's rank is below 2.
pub fn refine_essential(
    essential: &Matrix3<f64>,
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    scale: f64,
) -> Result<nls::Solution<Matrix3<f64>>> {
    if !essential.iter().all(|x| x.is_finite()) {
        return fail(
            ErrorKind::InvalidInput,
            "essential: expected a finite (3, 3) matrix",
        );
    }
    check_bearing_pairs(bearings1, bearings2, fivepoint::SAMPLE_SIZE, true)?;
    if !scale.is_finite() || scale <= 0.0 {
        return fail_setting("scale", "must be positive and finite", scale);
    }
    let singular = singular_values3(essential);
    if !(singular[1] > RANK_TOLERANCE * essential.norm()) {
        return fail(
            ErrorKind::Degenerate,
            "essential: rank below 2, no translation direction",
        );
    }
    refine(
        essential,
        normalised_plane(bearings1),
        normalised_plane(bearings2),
        scale,
    )
}

/// Estimates E robustly: LO-MSAC, then (optionally) the Sampson refinement on the inliers,
/// kept only if it lowers the MSAC cost.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - Unit bearings with positive z, N >= the minimal sample.
/// * `options` - The settings (threshold: a Sampson distance in the normalised plane).
/// * `minimal` - The minimal solver.
/// * `refine` - Refine the best model on its inliers.
///
/// # Returns
///
/// E, its inliers and the run's summary.
///
/// # Errors
///
/// `InvalidInput` or `Degenerate`.
pub fn essential_ransac(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    options: &Options,
    minimal: MinimalSolver,
    refine: bool,
) -> Result<RefinedSolution<Matrix3<f64>>> {
    check_bearing_pairs(bearings1, bearings2, minimal.sample_size(), true)?;
    let (x1, x2) = (normalised_plane(bearings1), normalised_plane(bearings2));
    let refiner = |model: &Matrix3<f64>, inliers: &[usize]| {
        self::refine(
            model,
            gather(&x1, inliers),
            gather(&x2, inliers),
            options.threshold,
        )
    };
    ransac::estimate_refined(
        &EssentialEstimator::new(bearings1, bearings2, minimal),
        options,
        if refine { Some(&refiner) } else { None },
        fivepoint::SAMPLE_SIZE,
    )
}

/// Estimates F robustly: LO-MSAC with the normalised 8-point solver.
///
/// # Arguments
///
/// * `pixels1`, `pixels2` - Pixels, N >= 8.
/// * `options` - The settings (threshold: a Sampson distance in pixels).
///
/// # Returns
///
/// F, its inliers and the run's summary.
///
/// # Errors
///
/// `InvalidInput` or `Degenerate`.
pub fn fundamental_ransac(
    pixels1: &[Vector2<f64>],
    pixels2: &[Vector2<f64>],
    options: &Options,
) -> Result<ransac::Solution<Matrix3<f64>>> {
    check_point_pairs(pixels1, pixels2, SAMPLE_SIZE, "pixels")?;
    ransac::estimate(&FundamentalEstimator::new(pixels1, pixels2), options)
}

#[cfg(test)]
mod tests {
    //! Native tests: the 8-point E on noise-free pairs, the true pose among E's four, and
    //! E-RANSAC through outliers.

    use nalgebra::Vector6;

    use super::*;

    /// Bearing pairs through a known relative pose, and the pose.
    fn scene(n: usize) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>, Matrix4<f64>) {
        let pose = se3::exp(&Vector6::new(0.9, -0.1, 0.15, 0.04, -0.1, 0.02));
        let (mut b1, mut b2) = (Vec::with_capacity(n), Vec::with_capacity(n));
        for i in 0..n {
            let t = f64::from(u32::try_from(i).unwrap());
            let point = Vector3::new((1.7 * t).sin(), (0.9 * t).cos(), 4.0 + (0.3 * t).sin());
            b1.push(point.normalize());
            b2.push(se3::act(&pose, &point).unwrap().normalize());
        }
        (b1, b2, pose)
    }

    /// The 8-point E is the true E, up to scale and sign; the true pose is among its four.
    #[test]
    fn eight_point_and_decomposition() {
        let (b1, b2, pose) = scene(12);
        let e = essential_8pt(&b1, &b2).unwrap();
        let truth = so3::hat(&se3::translation(&pose).normalize()) * se3::rotation(&pose);
        assert!((e - truth).amax().min((e + truth).amax()) < 1e-10);
        let unit = se3::from_parts(&se3::rotation(&pose), &se3::translation(&pose).normalize());
        let candidates = decompose_essential(&e).unwrap();
        assert!(candidates.iter().any(|c| (c - unit).amax() < 1e-10));
    }

    /// E-RANSAC finds every inlier through 30% outliers, and its refined summary reports the
    /// run's convergence.
    #[test]
    fn essential_ransac_through_outliers() {
        let (b1, mut b2, _) = scene(40);
        for (i, b) in b2.iter_mut().enumerate() {
            if i % 10 < 3 {
                *b = Vector3::new(0.3, -0.2, 1.0).normalize();
            }
        }
        let options = Options {
            threshold: 1e-6,
            seed: 3,
            ..Options::default()
        };
        let found = essential_ransac(&b1, &b2, &options, MinimalSolver::Stewenius, true).unwrap();
        assert_eq!(found.summary.run.inlier_count, 28);
        assert!(crate::solve::Summary::converged(&found.summary));
    }

    /// A match at both epipoles has a 0 / 0 Sampson distance: the refinement is an error (as
    /// in Python), and a rank-1 E has no start motion rather than a panic (panel H1, A2).
    #[test]
    fn a_match_at_both_epipoles_is_an_error() {
        // Forward motion, no rotation: both epipoles are exactly (0, 0, 1).
        let forward = Vector3::new(0.0, 0.0, 1.0);
        let (mut b1, mut b2) = (vec![forward], vec![forward]);
        for i in 0..8_u32 {
            let t = f64::from(i);
            let point = Vector3::new((1.7 * t).sin(), (0.9 * t).cos(), 4.0 + (0.3 * t).sin());
            b1.push(point.normalize());
            b2.push((point + forward).normalize());
        }
        let found = refine_essential(&so3::hat(&forward), &b1, &b2, 1e-3).unwrap_err();
        assert_eq!(found.kind, ErrorKind::InvalidInput);
        let rank_one = Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert_eq!(
            start_motion(&rank_one).unwrap_err().kind,
            ErrorKind::Degenerate
        );
    }
}
