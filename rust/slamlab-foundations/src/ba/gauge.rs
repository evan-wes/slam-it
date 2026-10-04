//! Bundle adjustment internals: the gauge -- which similarity motions of the world the data
//! leaves free, and their removal (see the Python reference, `ba/gauge.py`).
//!
//! Each anchor contributes rows of the derivative of what it pins with respect to the
//! similarity's generators `g = (t, ω, s)`: a known position X gives `[I, -hat(X), X]`, a
//! known orientation `[0, I, 0]`, metric depth `[0, 0, 1]`. The remaining gauge is their null
//! space N. It is removed pose by pose: at pose c, N moves the camera's world centre and
//! orientation by `M = [[I, -hat(C), C], [0, I, 0]] N`, and the pose's step is confined to the
//! complement of `J_cᵀ M` (`J_c` maps a step to the change in centre and orientation). A pose
//! that sees six directions is fixed outright.

use nalgebra::{DMatrix, DVector, Dyn, Matrix3, Matrix6, OMatrix, SMatrix, SVD, Vector3};

use super::{Gauge, POSE_DIM};
use crate::error::{ErrorKind, Result, fail};
use crate::so3;

/// The similarity's directions: translation (3), rotation (3), scale (1).
pub const GENERATORS: usize = 7;
/// The rigid motions' directions: translation (3), rotation (3).
pub const RIGID_DIM: usize = 6;
/// Singular values below this count as zero. Rows and images are in scene-normalised
/// coordinates (see [`Frame`]), of order 1, so it is in effect relative.
pub const GAUGE_TOLERANCE: f64 = 1e-9;
/// Two scores within this relative margin are a tie, broken by the lower index: every core
/// then chooses alike despite last-bit rounding between their SVDs.
pub const TIE_RTOL: f64 = 1e-12;

/// Golub-Kahan sweeps allowed in a dynamic SVD before it is abandoned. nalgebra's own `svd`
/// has no cap and never returns on a non-finite matrix; LAPACK and Eigen stop.
const SVD_MAX_ITERATIONS: usize = 10_000;

/// A dynamic matrix's SVD, singular values largest first, or `None` for a matrix with a
/// non-finite entry or one that does not converge within [`SVD_MAX_ITERATIONS`].
///
/// # Arguments
///
/// * `matrix` - The matrix (not empty).
/// * `compute_u` - Whether to compute `U`.
/// * `compute_v` - Whether to compute `Vᵀ`.
///
/// # Returns
///
/// The decomposition.
fn svd(matrix: DMatrix<f64>, compute_u: bool, compute_v: bool) -> Option<SVD<f64, Dyn, Dyn>> {
    if !matrix.iter().all(|x| x.is_finite()) {
        return None;
    }
    matrix.try_svd(compute_u, compute_v, f64::EPSILON * 5.0, SVD_MAX_ITERATIONS)
}

/// Scene-normalised coordinates for the gauge: `X' = (X - origin) / scale`.
///
/// The similarity's generators act about the world origin; far from it the rotation and
/// scale directions shrink like `1 / |X|`, and a rank test fails for geo-referenced scenes. A
/// change of origin and unit maps the generators invertibly, so spans -- and the gauge -- are
/// unchanged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// The mean of the anchors' and observed cameras' world positions.
    pub origin: Vector3<f64>,
    /// Their RMS distance from `origin` (1 if they coincide).
    pub scale: f64,
}

impl Default for Frame {
    /// The world's own frame.
    fn default() -> Self {
        Self {
            origin: Vector3::zeros(),
            scale: 1.0,
        }
    }
}

/// The frame centred on `positions` and scaled by their spread.
///
/// # Arguments
///
/// * `positions` - World positions.
///
/// # Returns
///
/// The frame.
#[allow(
    clippy::cast_precision_loss,
    reason = "a count of positions is far below 2^52"
)]
pub fn frame_of(positions: &[Vector3<f64>]) -> Frame {
    if positions.is_empty() {
        return Frame::default();
    }
    let n = positions.len() as f64;
    let origin = positions.iter().sum::<Vector3<f64>>() / n;
    let squared: f64 = positions.iter().map(|p| (p - origin).norm_squared()).sum();
    let spread = (squared / n).sqrt();
    Frame {
        origin,
        scale: if spread > 0.0 { spread } else { 1.0 },
    }
}

/// A world position in a frame.
///
/// # Arguments
///
/// * `frame` - The frame.
/// * `position` - The position.
///
/// # Returns
///
/// `(position - origin) / scale`.
pub fn in_frame(frame: &Frame, position: &Vector3<f64>) -> Vector3<f64> {
    (position - frame.origin) / frame.scale
}

/// What pins the world's similarity, before any gauge is applied.
#[derive(Debug, Clone, Default)]
pub struct Anchors {
    /// Fixed known positions in the world.
    pub points: Vec<Vector3<f64>>,
    /// Poses whose current world centre is pinned (fixed poses, translation priors).
    pub centre_cams: Vec<usize>,
    /// Poses whose current world orientation is pinned (fixed poses, rotation priors).
    pub orientation_cams: Vec<usize>,
    /// Whether metric depth pins the scale.
    pub metric: bool,
    /// The scene-normalised coordinates the rows are built in.
    pub frame: Frame,
}

/// Which poses remove the gauge, decided once from the start.
#[derive(Debug, Clone, Default)]
pub struct Plan {
    /// Poses fixed outright (each saw all six of its directions).
    pub fixed: Vec<usize>,
    /// Poses whose step is confined, in the order they take their share.
    pub confined: Vec<usize>,
    /// How many directions each confined pose removes.
    pub ranks: Vec<usize>,
    /// Directions no free pose could take, left in the problem (0 unless degenerate).
    pub free: usize,
}

/// The anchors' rows of `d(pinned) / d(t, ω, s)`, in the anchors' frame.
///
/// # Arguments
///
/// * `anchors` - What is pinned.
/// * `centres` - Every camera's world centre.
///
/// # Returns
///
/// A q x 7 matrix.
fn rows(anchors: &Anchors, centres: &[Vector3<f64>]) -> DMatrix<f64> {
    let positions: Vec<Vector3<f64>> = anchors
        .points
        .iter()
        .chain(anchors.centre_cams.iter().map(|&c| &centres[c]))
        .map(|p| in_frame(&anchors.frame, p))
        .collect();
    let orientation_rows = if anchors.orientation_cams.is_empty() {
        0
    } else {
        3
    };
    let n_rows = 3 * positions.len() + orientation_rows + usize::from(anchors.metric);
    let mut out = DMatrix::zeros(n_rows, GENERATORS);
    for (i, x) in positions.iter().enumerate() {
        out.fixed_view_mut::<3, 3>(3 * i, 0).fill_with_identity();
        out.fixed_view_mut::<3, 3>(3 * i, 3)
            .copy_from(&-so3::hat(x));
        out.fixed_view_mut::<3, 1>(3 * i, 6).copy_from(x);
    }
    let mut row = 3 * positions.len();
    if orientation_rows > 0 {
        out.fixed_view_mut::<3, 3>(row, 3).fill_with_identity();
        row += 3;
    }
    if anchors.metric {
        out[(row, GENERATORS - 1)] = 1.0;
    }
    out
}

/// An orthonormal basis of the null space of a rank-`rank` matrix's rows.
///
/// nalgebra's SVD is thin, so a wide matrix is padded with zero rows (which change neither
/// its row space nor its right singular vectors) to get every right singular vector. A
/// matrix the SVD fails on (a non-finite entry) gives a NaN basis: at the start the plan
/// reports that as an error ([`remaining`], [`plan`]); during the solve it makes every cost
/// it touches non-finite, so the step is rejected rather than taken.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `rank` - Its rank.
///
/// # Returns
///
/// A cols x (cols - rank) basis.
fn null_space(matrix: &DMatrix<f64>, rank: usize) -> DMatrix<f64> {
    let cols = matrix.ncols();
    if matrix.nrows() == 0 {
        return DMatrix::identity(cols, cols);
    }
    let padded = if matrix.nrows() < cols {
        matrix.clone().resize_vertically(cols, 0.0)
    } else {
        matrix.clone()
    };
    match svd(padded, false, true).and_then(|d| d.v_t) {
        Some(v_t) => v_t.rows(rank, cols - rank).transpose(),
        None => DMatrix::from_element(cols, cols - rank, f64::NAN),
    }
}

/// A matrix's singular values, largest first (none for an empty matrix; NaN where the SVD
/// fails, which no rank test counts).
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// The values.
fn singular_values(matrix: &DMatrix<f64>) -> DVector<f64> {
    if matrix.is_empty() {
        return DVector::zeros(0);
    }
    svd(matrix.clone(), false, false).map_or_else(
        || DVector::from_element(matrix.nrows().min(matrix.ncols()), f64::NAN),
        |d| d.singular_values,
    )
}

/// The number of singular values above [`GAUGE_TOLERANCE`].
///
/// # Arguments
///
/// * `singular` - The singular values.
///
/// # Returns
///
/// The rank.
fn rank_of(singular: &DVector<f64>) -> usize {
    singular.iter().filter(|&&s| s > GAUGE_TOLERANCE).count()
}

/// How N moves a camera's centre and orientation, in the frame.
///
/// # Arguments
///
/// * `centre` - The camera's centre in the frame.
/// * `null` - The remaining gauge N.
///
/// # Returns
///
/// A 6 x k matrix.
fn pose_image(centre: &Vector3<f64>, null: &DMatrix<f64>) -> DMatrix<f64> {
    let mut generator = SMatrix::<f64, POSE_DIM, GENERATORS>::zeros();
    generator.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
    generator
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&-so3::hat(centre));
    generator.fixed_view_mut::<3, 1>(0, 6).copy_from(centre);
    generator.fixed_view_mut::<3, 3>(3, 3).fill_with_identity();
    dynamic(&(generator * null))
}

/// A six-row matrix as a dynamically sized one.
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// The same values.
fn dynamic(matrix: &OMatrix<f64, nalgebra::U6, nalgebra::Dyn>) -> DMatrix<f64> {
    DMatrix::from_column_slice(matrix.nrows(), matrix.ncols(), matrix.as_slice())
}

/// `d(centre, orientation) / d(step)` for a right perturbation of `T_cam_world`, in world
/// units.
///
/// # Arguments
///
/// * `centre` - The camera's world centre.
///
/// # Returns
///
/// The 6 x 6 Jacobian.
fn pose_jacobian(centre: &Vector3<f64>) -> Matrix6<f64> {
    let mut out = Matrix6::zeros();
    out.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&-Matrix3::identity());
    out.fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&so3::hat(centre));
    out.fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&-Matrix3::identity());
    out
}

/// The first index within [`TIE_RTOL`] of the largest value.
///
/// # Arguments
///
/// * `values` - The values (not empty).
///
/// # Returns
///
/// The index.
fn first_largest(values: &DVector<f64>) -> usize {
    let threshold = values.max() * (1.0 - TIE_RTOL);
    values.iter().position(|&v| v >= threshold).unwrap_or(0)
}

/// An orthonormal basis of a matrix's column span that depends on the span alone: from its
/// projector, repeatedly the normalised column of the largest diagonal entry, deflated.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `rank` - Its rank.
///
/// # Returns
///
/// The basis vectors (NaN where the SVD fails).
fn canonical_span(matrix: &DMatrix<f64>, rank: usize) -> Vec<DVector<f64>> {
    let Some(u) = svd(matrix.clone(), true, false).and_then(|d| d.u) else {
        return vec![DVector::from_element(matrix.nrows(), f64::NAN); rank];
    };
    let u = u.columns(0, rank);
    let mut projector = u * u.transpose();
    let mut out = Vec::with_capacity(rank);
    for _ in 0..rank {
        let k = first_largest(&projector.diagonal());
        let vector: DVector<f64> = projector.column(k) / projector[(k, k)].sqrt();
        projector -= &vector * vector.transpose();
        out.push(vector);
    }
    out
}

/// An orthonormal basis of the complement of `normals`: one Householder reflection per
/// normal onto its largest remaining axis, which is then dropped.
///
/// # Arguments
///
/// * `normals` - The vectors to complement.
/// * `size` - Their length.
///
/// # Returns
///
/// A size x (size - normals) basis.
fn complement(normals: &[DVector<f64>], size: usize) -> DMatrix<f64> {
    let mut basis = DMatrix::identity(size, size);
    for normal in normals {
        let unit = (basis.transpose() * normal).normalize();
        let k = first_largest(&unit.abs());
        let mut v = unit.clone();
        v[k] += if unit[k] >= 0.0 { 1.0 } else { -1.0 };
        let reflection =
            DMatrix::identity(unit.len(), unit.len()) - (&v * v.transpose()) * (2.0 / v.dot(&v));
        basis = (basis * reflection).remove_column(k);
    }
    basis
}

/// Scores candidates seeing the same number of directions (higher wins): the lowest index for
/// a pose that would be fixed, the smallest retained singular value for one that would be
/// confined.
///
/// # Arguments
///
/// * `cam` - The candidate.
/// * `singular` - Its image's singular values.
/// * `rank` - How many directions it sees.
///
/// # Returns
///
/// The score.
#[allow(
    clippy::cast_precision_loss,
    reason = "a camera index is far below 2^52"
)]
fn tie_break(cam: usize, singular: &DVector<f64>, rank: usize) -> f64 {
    if rank == POSE_DIM {
        return -(cam as f64);
    }
    if rank > 0 { singular[rank - 1] } else { 0.0 }
}

/// The candidate that sees the most of N (ties by [`tie_break`]), and how many.
///
/// # Arguments
///
/// * `anchors` - What is pinned (for the frame).
/// * `centres` - Every camera's world centre.
/// * `left` - The candidates.
/// * `null` - The remaining gauge.
///
/// # Returns
///
/// The candidate (`None` if none sees anything) and its rank.
fn best_candidate(
    anchors: &Anchors,
    centres: &[Vector3<f64>],
    left: &[usize],
    null: &DMatrix<f64>,
) -> (Option<usize>, usize) {
    let mut best = None;
    let mut best_rank = 0;
    let mut best_score = 0.0;
    for &c in left {
        let singular = singular_values(&pose_image(&in_frame(&anchors.frame, &centres[c]), null));
        let rank = rank_of(&singular);
        let score = tie_break(c, &singular, rank);
        if rank > best_rank
            || (rank == best_rank && score > best_score + TIE_RTOL * f64::abs(best_score))
        {
            best = Some(c);
            best_rank = rank;
            best_score = score;
        }
    }
    (best, best_rank)
}

/// The failure of a gauge computation whose SVD failed (a non-finite anchor or centre).
///
/// # Returns
///
/// `Degenerate`, naming the gauge.
fn gauge_failed<T>() -> Result<T> {
    fail(
        ErrorKind::Degenerate,
        "gauge: the SVD of the pinned directions failed (a non-finite anchor or camera centre)",
    )
}

/// The remaining gauge N.
///
/// # Arguments
///
/// * `anchors` - What is pinned.
/// * `centres` - Every camera's world centre.
///
/// # Returns
///
/// An orthonormal 7 x k basis over `(t, ω, s)` in the anchors' frame.
///
/// # Errors
///
/// `Degenerate` if an SVD fails, rather than reading as "nothing pins anything".
pub fn remaining(anchors: &Anchors, centres: &[Vector3<f64>]) -> Result<DMatrix<f64>> {
    let pinned = rows(anchors, centres);
    let singular = singular_values(&pinned);
    let null = null_space(&pinned, rank_of(&singular));
    if singular.iter().chain(null.iter()).any(|x| !x.is_finite()) {
        return gauge_failed();
    }
    Ok(null)
}

/// Names a remaining gauge: none, scale (about a point), first pose (the rigid motions),
/// similarity, or partial. Reads only N's dimension and its ω and s rows, which a change of
/// origin and unit leaves alone.
///
/// # Arguments
///
/// * `null` - The remaining gauge N.
///
/// # Returns
///
/// The gauge it is.
pub fn classify(null: &DMatrix<f64>) -> Gauge {
    let k = null.ncols();
    if k == 0 {
        return Gauge::None;
    }
    if k == GENERATORS {
        return Gauge::Similarity;
    }
    if k == RIGID_DIM && null.row(GENERATORS - 1).amax() <= GAUGE_TOLERANCE {
        return Gauge::FirstPose;
    }
    if k == 1 && null.rows(3, 3).amax() <= GAUGE_TOLERANCE {
        return Gauge::Scale;
    }
    Gauge::Partial
}

/// Chooses the poses that remove the remaining gauge, greedily.
///
/// At each turn the candidate seeing the most remaining directions takes them (fixed if it
/// sees six, confined otherwise); among equals, the lowest index for a fixed pose, the one
/// seeing its directions best for a confined one.
///
/// # Arguments
///
/// * `anchors` - What is pinned.
/// * `centres` - Every camera's world centre at the start.
/// * `candidates` - The free observed poses, in index order.
///
/// # Returns
///
/// The plan, with what no pose could take.
///
/// # Errors
///
/// `Degenerate` if an SVD fails.
pub fn plan(anchors: &Anchors, centres: &[Vector3<f64>], candidates: &[usize]) -> Result<Plan> {
    let mut null = remaining(anchors, centres)?;
    let mut out = Plan::default();
    let mut left = candidates.to_vec();
    while null.ncols() > 0 && !left.is_empty() {
        let (Some(best), rank) = best_candidate(anchors, centres, &left, &null) else {
            break;
        };
        if rank == POSE_DIM {
            out.fixed.push(best);
        } else {
            out.confined.push(best);
            out.ranks.push(rank);
        }
        let image = pose_image(&in_frame(&anchors.frame, &centres[best]), &null);
        null = &null * null_space(&image, rank);
        if null.iter().any(|x| !x.is_finite()) {
            return gauge_failed();
        }
        left.retain(|&c| c != best);
    }
    out.free = null.ncols();
    Ok(out)
}

/// Each confined pose's step basis at the current centres: one 6 x (6 - rank) basis per
/// confined pose, in the plan's order, over its right-perturbation step.
///
/// # Arguments
///
/// * `anchors` - What is pinned, the poses the plan fixed included.
/// * `centres` - Every camera's current world centre.
/// * `chosen` - The plan.
///
/// # Returns
///
/// The bases.
pub fn confinement(
    anchors: &Anchors,
    centres: &[Vector3<f64>],
    chosen: &Plan,
) -> Vec<DMatrix<f64>> {
    let dimension = chosen.ranks.iter().sum::<usize>() + chosen.free;
    let mut null = null_space(&rows(anchors, centres), GENERATORS - dimension);
    let mut to_world = Matrix6::<f64>::identity();
    to_world
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(Matrix3::identity() * anchors.frame.scale));
    let mut bases = Vec::with_capacity(chosen.confined.len());
    for (&cam, &rank) in chosen.confined.iter().zip(&chosen.ranks) {
        let centre = &centres[cam];
        let image = pose_image(&in_frame(&anchors.frame, centre), &null);
        let seen = dynamic(&(pose_jacobian(centre).transpose() * to_world * &image));
        bases.push(complement(&canonical_span(&seen, rank), POSE_DIM));
        null = &null * null_space(&image, rank);
    }
    bases
}

#[cfg(test)]
mod tests {
    //! Native tests of the gauge's linear algebra, runnable without Python. The gauge choices
    //! themselves are held to the Python reference by the BA parity fixture.

    use super::*;

    /// nalgebra's SVD returns its singular values largest first, which every rank and null
    /// space here relies on.
    #[test]
    fn singular_values_are_sorted() {
        let m = DMatrix::from_row_slice(3, 3, &[0.1, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 2.0]);
        let s = singular_values(&m);
        assert!(s[0] >= s[1] && s[1] >= s[2]);
    }

    /// With nothing anchored the whole similarity is free; a fixed pose leaves the scale; two
    /// fixed poses leave nothing.
    #[test]
    fn remaining_gauge_by_anchors() {
        let centres = [Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)];
        let free = Anchors::default();
        assert_eq!(
            classify(&remaining(&free, &centres).unwrap()),
            Gauge::Similarity
        );
        let one = Anchors {
            centre_cams: vec![0],
            orientation_cams: vec![0],
            frame: frame_of(&centres),
            ..Anchors::default()
        };
        assert_eq!(classify(&remaining(&one, &centres).unwrap()), Gauge::Scale);
        let both = Anchors {
            centre_cams: vec![0, 1],
            orientation_cams: vec![0, 1],
            ..one
        };
        assert_eq!(classify(&remaining(&both, &centres).unwrap()), Gauge::None);
    }

    /// The null space is orthonormal and annihilated by the matrix, wide or tall.
    #[test]
    fn null_space_of_a_wide_matrix() {
        let m = DMatrix::from_row_slice(2, 4, &[1.0, 2.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0]);
        let n = null_space(&m, 2);
        assert_eq!(n.shape(), (4, 2));
        assert!((&m * &n).amax() < 1e-14);
        assert!((n.transpose() * &n - DMatrix::identity(2, 2)).amax() < 1e-14);
    }

    /// A non-finite matrix fails the SVD observably (NaN), instead of looping forever as
    /// nalgebra's uncapped `svd` does (panel L2).
    #[test]
    fn non_finite_matrices_do_not_hang() {
        for bad in [f64::NAN, f64::INFINITY] {
            let mut m = DMatrix::from_element(7, 7, 1.0);
            m[(3, 2)] = bad;
            assert!(null_space(&m, 3).iter().all(|x| x.is_nan()));
            assert!(singular_values(&m).iter().all(|x| x.is_nan()));
            assert_eq!(rank_of(&singular_values(&m)), 0);
            assert!(
                canonical_span(&m, 2)
                    .iter()
                    .all(|v| v.iter().all(|x| x.is_nan()))
            );
        }
    }

    /// At the start, a non-finite anchor makes the plan an error, not a free similarity
    /// that reads as "nothing pins anything".
    #[test]
    fn a_failed_plan_is_an_error() {
        let centres = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(f64::NAN, 0.0, 0.0),
        ];
        let anchors = Anchors {
            centre_cams: vec![1],
            orientation_cams: vec![1],
            ..Anchors::default()
        };
        let kind = remaining(&anchors, &centres).unwrap_err().kind;
        assert_eq!(kind, ErrorKind::Degenerate);
        let kind = plan(&anchors, &centres, &[0]).unwrap_err().kind;
        assert_eq!(kind, ErrorKind::Degenerate);
    }

    /// The complement of two normals is orthonormal and orthogonal to them.
    #[test]
    fn complement_is_orthogonal_to_its_normals() {
        let normals = [
            DVector::from_vec(vec![1.0, 1.0, 0.0, 0.0, 0.0, 0.0]).normalize(),
            DVector::from_vec(vec![0.0, 0.0, 1.0, 0.0, 0.0, 2.0]).normalize(),
        ];
        let basis = complement(&normals, 6);
        assert_eq!(basis.shape(), (6, 4));
        for normal in &normals {
            assert!((basis.transpose() * normal).amax() < 1e-14);
        }
        assert!((basis.transpose() * &basis - DMatrix::identity(4, 4)).amax() < 1e-14);
    }
}
