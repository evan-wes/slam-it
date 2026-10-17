//! Factor graphs on GTSAM: a pose graph, a similarity refined against matches, and BA.
//!
//! Mirrors `slam_it.foundations.graphs`, whose module docstring states the conventions: poses
//! cross camera-to-world (`T_world_cam`), a similarity as `[[s R, t], [0, 1]]` mapping `p` to
//! `s R p + t`; Jacobians are on GTSAM's right-perturbed tangents, rotation first: `(w, v)`
//! for a pose, `(w, v, lambda)` for a similarity.
//!
//! GTSAM is reached through a `cxx` bridge (`cxx/graphs_bridge.cpp`): its graph, its values,
//! its between and equality factors, and its Levenberg-Marquardt. The two factors GTSAM lacks
//! are this crate's own: a generic C++ factor calls back into Rust (`evaluate`), which
//! computes the residual and its Jacobians on this crate's camera model, as GTSAM's
//! `CustomFactor` calls back into Python. The solve is driven from here one iteration at a
//! time, by the rule every language stops by (`Termination`).
//!
//! The bridge is the one place in the crate that writes `unsafe`: `cxx` declares the C++ side
//! of a bridge in an `unsafe extern "C++"` block, the promise that its signatures are true.
//! `cxx` checks the rest; the crate allows `unsafe` in that module alone.

use cxx::UniquePtr;
use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, Vector2, Vector3};

use crate::camera::{self, Model};
use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::so3;

/// A pose's tangent dimension, `(w, v)`.
pub const POSE_DIM: usize = 6;
/// A similarity's tangent dimension, `(w, v, lambda)`.
pub const SIMILARITY_DIM: usize = 7;

/// Why a solve stopped (`slam_it.foundations.graphs.Termination`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Termination {
    /// Converged: an iteration lowered the error by no more than the tolerances, or left it at
    /// zero.
    Function = 0,
    /// Not converged: `max_iterations` iterations ran.
    MaxIterations = 1,
    /// Not converged: no step lowered the error before the damping reached its upper bound
    /// (also how a solve at its minimum ends when both tolerances are 0).
    DampingLimit = 2,
}

impl Termination {
    /// Every termination, in code order.
    pub const ALL: [Self; 3] = [Self::Function, Self::MaxIterations, Self::DampingLimit];

    /// The termination's name, as the Python side spells it.
    ///
    /// # Returns
    ///
    /// `"function"`, `"max_iterations"` or `"damping_limit"`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Function => "function",
            Self::MaxIterations => "max_iterations",
            Self::DampingLimit => "damping_limit",
        }
    }

    /// Whether the termination counts as converged.
    ///
    /// # Returns
    ///
    /// True for `Function`.
    #[must_use]
    pub const fn is_converged(self) -> bool {
        matches!(self, Self::Function)
    }
}

/// How a solve runs (`slam_it.foundations.graphs.Options`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Options {
    /// Iterations at most, at least 1: each linearises once and tries dampings until a step
    /// lowers the error.
    pub max_iterations: usize,
    /// Levenberg-Marquardt's first damping (GTSAM's `lambdaInitial`), positive.
    pub initial_damping: f64,
    /// An iteration lowering the error by at most this fraction of it ends the solve as
    /// converged; non-negative.
    pub relative_tolerance: f64,
    /// An iteration lowering the error by at most this ends the solve as converged;
    /// non-negative.
    pub absolute_tolerance: f64,
}

impl Default for Options {
    /// The Python side's defaults.
    ///
    /// # Returns
    ///
    /// Ten iterations, a first damping of 1e-5, a relative tolerance of 1e-10 and no absolute
    /// one.
    fn default() -> Self {
        Self {
            max_iterations: 10,
            initial_damping: 1e-5,
            relative_tolerance: 1e-10,
            absolute_tolerance: 0.0,
        }
    }
}

impl Options {
    /// Checks the options.
    ///
    /// # Errors
    ///
    /// `InvalidInput` naming the option out of its range.
    pub fn check(&self) -> Result<()> {
        if self.max_iterations < 1 {
            return fail_setting("max_iterations", "expected >= 1", self.max_iterations);
        }
        if !(self.initial_damping.is_finite() && self.initial_damping > 0.0) {
            return fail_setting("initial_damping", "expected > 0", self.initial_damping);
        }
        if !(self.relative_tolerance.is_finite() && self.relative_tolerance >= 0.0) {
            return fail_setting(
                "relative_tolerance",
                "expected >= 0",
                self.relative_tolerance,
            );
        }
        if !(self.absolute_tolerance.is_finite() && self.absolute_tolerance >= 0.0) {
            return fail_setting(
                "absolute_tolerance",
                "expected >= 0",
                self.absolute_tolerance,
            );
        }
        Ok(())
    }
}

/// How a solve went.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Summary {
    /// GTSAM's error, `1/2 sum rho(|r|^2_Sigma)`, at the start.
    pub initial_error: f64,
    /// GTSAM's error at the returned values.
    pub final_error: f64,
    /// The iterations run.
    pub iterations: usize,
    /// Why the solve stopped.
    pub termination: Termination,
    /// Whether `termination` counts as converged.
    pub converged: bool,
}

impl crate::solve::Summary for Summary {
    type Termination = Termination;

    fn iterations(&self) -> usize {
        self.iterations
    }

    fn converged(&self) -> bool {
        self.converged
    }

    fn termination(&self) -> Termination {
        self.termination
    }
}

/// A pose graph's solution.
#[derive(Clone, Debug)]
pub struct PoseGraph {
    /// Each pose, camera-to-world: rigid, or a similarity `[[s R, t], [0, 1]]`.
    pub poses: Vec<Matrix4<f64>>,
    /// How the solve went.
    pub summary: Summary,
}

/// A refined similarity (or rigid motion), and how far each match is from it.
#[derive(Clone, Debug)]
pub struct SimilarityRefinement {
    /// `S_12`, `[[s R, t], [0, 1]]`, carrying camera 2's points into camera 1's frame.
    pub similarity: Matrix4<f64>,
    /// Each match's whitened squared error at the result, before the robust loss: in image 1,
    /// then image 2; infinite where the carried point is behind the camera, NaN where image 2
    /// does not observe the match.
    pub chi2: Vec<[f64; 2]>,
    /// How the solve went.
    pub summary: Summary,
}

/// A bundle adjustment's solution.
#[derive(Clone, Debug)]
pub struct Adjustment {
    /// Each camera's `T_world_cam`.
    pub poses: Vec<Matrix4<f64>>,
    /// Each point, in the world.
    pub points: Vec<Vector3<f64>>,
    /// How the solve went.
    pub summary: Summary,
}

/// A refinement's matches, row for row.
#[derive(Clone, Copy, Debug)]
pub struct Matches<'a> {
    /// Each match's point in camera 1's frame.
    pub points1: &'a [Vector3<f64>],
    /// Each match's point in camera 2's frame.
    pub points2: &'a [Vector3<f64>],
    /// Where image 1 observes each.
    pub pixels1: &'a [Vector2<f64>],
    /// Where image 2 observes each; NaN where it does not (the point found in a neighbour of
    /// keyframe 2), whose match then counts its image-1 error alone.
    pub pixels2: &'a [Vector2<f64>],
    /// Each observation's information, image 1 then image 2, positive.
    pub information: &'a [[f64; 2]],
}

/// A bundle's observations, row for row.
#[derive(Clone, Copy, Debug)]
pub struct Observations<'a> {
    /// Each observation's camera.
    pub cameras: &'a [usize],
    /// Each observation's point.
    pub point_ids: &'a [usize],
    /// Each observation's `(u, v)`.
    pub pixels: &'a [Vector2<f64>],
    /// Each observation's right column, NaN for none.
    pub right: &'a [f64],
    /// Each observation's information, `1 / sigma^2` in px^-2, positive.
    pub information: &'a [f64],
}

/// The camera every factor projects through.
#[derive(Clone, Debug)]
pub struct Lens {
    /// The camera model.
    pub model: Model,
    /// Its intrinsics.
    pub intrinsics: Vec<f64>,
    /// The baseline times the focal length, for a right column `u - bf / z`.
    pub bf: f64,
}

// --- the factors' residuals ---------------------------------------------------------------

/// A point's reprojection error in one camera, and its Jacobians.
#[derive(Clone, Debug)]
pub struct ProjectionResidual {
    /// The predicted minus the measured, 2, or 3 with a right column.
    pub residual: DVector<f64>,
    /// Its Jacobian on the pose's tangent `(w, v)`, perturbed on the right (D x 6).
    pub d_pose: DMatrix<f64>,
    /// Its Jacobian on the point (D x 3).
    pub d_point: DMatrix<f64>,
    /// Whether the point is in front of the camera; one that is not contributes nothing
    /// (zero residual and Jacobians).
    pub in_front: bool,
}

/// A match's two reprojection errors through a similarity, and their Jacobians.
#[derive(Clone, Debug)]
pub struct SimilarityResiduals {
    /// `pi(S_12 X_2) - u_1`.
    pub residual1: Vector2<f64>,
    /// Its Jacobian on the motion's tangent, on the right (2 x 6 or 7).
    pub d_similarity1: DMatrix<f64>,
    /// `pi(S_12^-1 X_1) - u_2`.
    pub residual2: Vector2<f64>,
    /// Its Jacobian (2 x 6 or 7).
    pub d_similarity2: DMatrix<f64>,
    /// Whether `S_12 X_2` is in front of camera 1.
    pub in_front1: bool,
    /// Whether `S_12^-1 X_1` is in front of camera 2.
    pub in_front2: bool,
}

/// A camera-frame point's pixel and its Jacobian on the point, if it is in front.
///
/// # Arguments
///
/// * `lens` - The camera.
/// * `point` - The point.
///
/// # Returns
///
/// The pixel and `d(pixel) / d(point)` (2 x 3), or `None` if the point is not in front.
fn projected(lens: &Lens, point: &Vector3<f64>) -> Option<(Vector2<f64>, DMatrix<f64>)> {
    if !(point.z > camera::MIN_DEPTH) {
        return None;
    }
    let pixel = camera::project(lens.model, &lens.intrinsics, point).ok()?;
    let jacobian = camera::project_jacobian(lens.model, &lens.intrinsics, point).ok()?;
    Some((
        pixel,
        DMatrix::from_iterator(2, 3, jacobian.columns(0, 3).iter().copied()),
    ))
}

/// A 3x3 matrix as a dynamically sized one.
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// The copy.
fn dynamic(matrix: &Matrix3<f64>) -> DMatrix<f64> {
    DMatrix::from_column_slice(3, 3, matrix.as_slice())
}

/// Splits `[[s R, t], [0, 1]]` into `R`, `t` and `s`.
///
/// # Arguments
///
/// * `matrix` - The similarity, its scale positive.
///
/// # Returns
///
/// `(R, t, s)`.
fn split(matrix: &Matrix4<f64>) -> (Matrix3<f64>, Vector3<f64>, f64) {
    let block: Matrix3<f64> = matrix.fixed_view::<3, 3>(0, 0).into();
    let scale = block.determinant().cbrt();
    (block / scale, matrix.fixed_view::<3, 1>(0, 3).into(), scale)
}

/// Returns a point's reprojection error in one camera, and its Jacobians
/// (`graphs.projection_residual`).
///
/// # Arguments
///
/// * `pose` - The camera's `T_world_cam`.
/// * `point` - The point, in the world.
/// * `measured` - The observed `(u, v)`, and the right column for a stereo observation.
/// * `lens` - The camera.
///
/// # Returns
///
/// The residual, its Jacobians and whether the point is in front of the camera.
///
/// # Errors
///
/// `InvalidInput` if `measured` has neither 2 nor 3 values.
pub fn projection_residual(
    pose: &Matrix4<f64>,
    point: &Vector3<f64>,
    measured: &[f64],
    lens: &Lens,
) -> Result<ProjectionResidual> {
    let size = measured.len();
    if size != 2 && size != 3 {
        return fail(
            ErrorKind::InvalidInput,
            "measured: expected (u, v) or (u, v, u_r)",
        );
    }
    let (rotation, translation, _) = split(pose);
    let in_camera = rotation.transpose() * (point - translation);
    let Some((pixel, d_pixel)) = projected(lens, &in_camera) else {
        return Ok(ProjectionResidual {
            residual: DVector::zeros(size),
            d_pose: DMatrix::zeros(size, POSE_DIM),
            d_point: DMatrix::zeros(size, 3),
            in_front: false,
        });
    };
    let mut predicted = DVector::zeros(size);
    let mut d_predicted = DMatrix::zeros(size, 3);
    predicted[0] = pixel.x;
    predicted[1] = pixel.y;
    d_predicted.rows_mut(0, 2).copy_from(&d_pixel);
    if size == 3 {
        let z = in_camera.z;
        predicted[2] = pixel.x - lens.bf / z;
        for c in 0..3 {
            d_predicted[(2, c)] = d_pixel[(0, c)];
        }
        d_predicted[(2, 2)] += lens.bf / (z * z);
    }
    // (T Exp(xi))^-1 X = Exp(-xi) T^-1 X ~ X_c - w x X_c - v.
    let mut d_camera_d_pose = DMatrix::zeros(3, POSE_DIM);
    d_camera_d_pose
        .view_mut((0, 0), (3, 3))
        .copy_from(&so3::hat(&in_camera));
    d_camera_d_pose
        .view_mut((0, 3), (3, 3))
        .copy_from(&(-Matrix3::identity()));
    let residual = predicted - DVector::from_column_slice(measured);
    let d_point = &d_predicted * dynamic(&rotation.transpose());
    Ok(ProjectionResidual {
        residual,
        d_pose: &d_predicted * d_camera_d_pose,
        d_point,
        in_front: true,
    })
}

/// Returns a match's two reprojection errors through a similarity, and their Jacobians
/// (`graphs.similarity_residuals`).
///
/// # Arguments
///
/// * `similarity` - `S_12`, `[[s R, t], [0, 1]]`.
/// * `point1` - The match's point in camera 1's frame.
/// * `point2` - Its point in camera 2's frame.
/// * `pixel1` - Where image 1 observes it.
/// * `pixel2` - Where image 2 observes it.
/// * `lens` - The camera, the same for both images.
/// * `fixed_scale` - Whether the motion is rigid: its tangent is then `(w, v)`.
///
/// # Returns
///
/// The residuals, their Jacobians and whether each carried point is in front.
#[must_use]
pub fn similarity_residuals(
    similarity: &Matrix4<f64>,
    point1: &Vector3<f64>,
    point2: &Vector3<f64>,
    pixel1: &Vector2<f64>,
    pixel2: &Vector2<f64>,
    lens: &Lens,
    fixed_scale: bool,
) -> SimilarityResiduals {
    let (rotation, translation, scale) = split(similarity);
    let dimension = if fixed_scale {
        POSE_DIM
    } else {
        SIMILARITY_DIM
    };
    let forward = scale * rotation * point2 + translation;
    let backward = rotation.transpose() * (point1 - translation) / scale;
    // S Exp(xi) p ~ S (p + w x p + v + lambda p), and (S Exp(xi))^-1 q = Exp(-xi) S^-1 q.
    let mut d_forward = DMatrix::zeros(3, SIMILARITY_DIM);
    d_forward
        .view_mut((0, 0), (3, 3))
        .copy_from(&(-so3::hat(point2)));
    d_forward
        .view_mut((0, 3), (3, 3))
        .copy_from(&Matrix3::identity());
    d_forward.view_mut((0, 6), (3, 1)).copy_from(point2);
    let d_forward = dynamic(&(scale * rotation)) * d_forward;
    let mut d_backward = DMatrix::zeros(3, SIMILARITY_DIM);
    d_backward
        .view_mut((0, 0), (3, 3))
        .copy_from(&so3::hat(&backward));
    d_backward
        .view_mut((0, 3), (3, 3))
        .copy_from(&(-Matrix3::identity()));
    d_backward.view_mut((0, 6), (3, 1)).copy_from(&(-backward));
    let side = |carried: &Vector3<f64>, d_carried: &DMatrix<f64>, pixel: &Vector2<f64>| {
        projected(lens, carried).map_or_else(
            || (Vector2::zeros(), DMatrix::zeros(2, dimension), false),
            |(seen, d_pixel)| {
                (
                    seen - pixel,
                    d_pixel * d_carried.columns(0, dimension),
                    true,
                )
            },
        )
    };
    let (residual1, d_similarity1, in_front1) = side(&forward, &d_forward, pixel1);
    let (residual2, d_similarity2, in_front2) = side(&backward, &d_backward, pixel2);
    SimilarityResiduals {
        residual1,
        d_similarity1,
        residual2,
        d_similarity2,
        in_front1,
        in_front2,
    }
}

// --- the bridge ----------------------------------------------------------------------------

/// What the Rust factors of a problem are.
#[derive(Clone, Debug)]
enum Factor {
    /// A point's reprojection: variables a pose and a point.
    Projection {
        /// The observation: `(u, v)`, and the right column if it has one.
        measured: Vec<f64>,
    },
    /// One side of a match carried through the motion: variable the motion.
    Similarity {
        /// The match's point in camera 1's frame.
        point1: Vector3<f64>,
        /// Its point in camera 2's frame.
        point2: Vector3<f64>,
        /// Where image 1 observes it.
        pixel1: Vector2<f64>,
        /// Where image 2 observes it.
        pixel2: Vector2<f64>,
        /// True for the error in image 1.
        first: bool,
        /// Whether the motion is rigid.
        fixed_scale: bool,
    },
}

/// The Rust factors of a problem and their camera, which GTSAM calls back into.
#[derive(Clone, Debug)]
pub struct Evaluator {
    /// The camera.
    lens: Lens,
    /// The factors, by index.
    factors: Vec<Factor>,
}

/// Writes a matrix row-major into a slice of its size.
///
/// # Arguments
///
/// * `matrix` - The matrix.
/// * `out` - Its rows times its columns doubles.
fn write_rows(matrix: &DMatrix<f64>, out: &mut [f64]) {
    for (r, row) in out
        .chunks_mut(matrix.ncols())
        .enumerate()
        .take(matrix.nrows())
    {
        for (c, value) in row.iter_mut().enumerate() {
            *value = matrix[(r, c)];
        }
    }
}

/// GTSAM's call into a Rust factor: its residual and its Jacobian (each variable's block side
/// by side, row-major).
///
/// Never panics, as a call from C++ must not: a factor index or a value count it does not know
/// leaves the outputs at zero, which the factors built here never produce.
///
/// # Arguments
///
/// * `evaluator` - The problem's factors.
/// * `index` - The factor's index.
/// * `values` - Its variables' values: a pose or similarity 16 row-major, a point 3.
/// * `residual` - Where to write the residual.
/// * `jacobian` - Where to write the Jacobian.
fn evaluate(
    evaluator: &Evaluator,
    index: usize,
    values: &[f64],
    residual: &mut [f64],
    jacobian: &mut [f64],
) {
    residual.fill(0.0);
    jacobian.fill(0.0);
    match evaluator.factors.get(index) {
        Some(Factor::Projection { measured }) if values.len() == 19 => {
            let pose = Matrix4::from_row_slice(&values[..16]);
            let point = Vector3::from_column_slice(&values[16..]);
            if let Ok(at) = projection_residual(&pose, &point, measured, &evaluator.lens) {
                let mut both = DMatrix::zeros(at.residual.len(), POSE_DIM + 3);
                both.columns_mut(0, POSE_DIM).copy_from(&at.d_pose);
                both.columns_mut(POSE_DIM, 3).copy_from(&at.d_point);
                residual.copy_from_slice(at.residual.as_slice());
                write_rows(&both, jacobian);
            }
        }
        Some(Factor::Similarity {
            point1,
            point2,
            pixel1,
            pixel2,
            first,
            fixed_scale,
        }) if values.len() == 16 => {
            let motion = if *fixed_scale {
                Matrix4::from_row_slice(values)
            } else {
                from_similarity3(values)
            };
            let both = similarity_residuals(
                &motion,
                point1,
                point2,
                pixel1,
                pixel2,
                &evaluator.lens,
                *fixed_scale,
            );
            let (error, d_motion) = if *first {
                (both.residual1, both.d_similarity1)
            } else {
                (both.residual2, both.d_similarity2)
            };
            residual.copy_from_slice(error.as_slice());
            write_rows(&d_motion, jacobian);
        }
        _ => {}
    }
}

/// The bridge to GTSAM (`cxx/graphs_bridge.hpp`).
#[cxx::bridge(namespace = "slam_it::rust_bridge")]
#[allow(unsafe_code, clippy::missing_errors_doc, clippy::needless_lifetimes)]
mod ffi {
    extern "Rust" {
        /// The Rust factors of a problem.
        type Evaluator;

        /// GTSAM's call into a Rust factor (`super::evaluate`).
        fn evaluate(
            evaluator: &Evaluator,
            index: usize,
            values: &[f64],
            residual: &mut [f64],
            jacobian: &mut [f64],
        );
    }

    unsafe extern "C++" {
        include!("slam-it-foundations/cxx/graphs_bridge.hpp");

        /// GTSAM's graph and starting values.
        type Problem;
        /// Levenberg-Marquardt on a problem.
        type Optimizer;

        /// Makes an empty problem.
        fn new_problem() -> UniquePtr<Problem>;
        /// Adds a pose variable.
        fn add_pose(problem: Pin<&mut Problem>, key: u64, matrix: &[f64]) -> Result<()>;
        /// Adds a similarity variable.
        fn add_similarity(problem: Pin<&mut Problem>, key: u64, matrix: &[f64]) -> Result<()>;
        /// Adds a point variable.
        fn add_point(problem: Pin<&mut Problem>, key: u64, point: &[f64]) -> Result<()>;
        /// Adds GTSAM's between factor on two poses.
        fn add_between_poses(
            problem: Pin<&mut Problem>,
            first: u64,
            second: u64,
            relative: &[f64],
        ) -> Result<()>;
        /// Adds GTSAM's between factor on two similarities.
        fn add_between_similarities(
            problem: Pin<&mut Problem>,
            first: u64,
            second: u64,
            relative: &[f64],
        ) -> Result<()>;
        /// Holds a pose exactly.
        fn fix_pose(problem: Pin<&mut Problem>, key: u64, matrix: &[f64]) -> Result<()>;
        /// Holds a similarity exactly.
        fn fix_similarity(problem: Pin<&mut Problem>, key: u64, matrix: &[f64]) -> Result<()>;
        /// Adds a factor Rust evaluates.
        fn add_rust_factor(
            problem: Pin<&mut Problem>,
            keys: &[u64],
            kinds: &[u8],
            dimension: usize,
            information: f64,
            huber: f64,
            index: usize,
        ) -> Result<()>;
        /// Starts Levenberg-Marquardt, evaluating the starting error.
        fn new_optimizer(
            problem: &Problem,
            evaluator: &Evaluator,
            initial_damping: f64,
            relative_tolerance: f64,
            absolute_tolerance: f64,
        ) -> Result<UniquePtr<Optimizer>>;
        /// Runs one iteration.
        fn iterate(optimizer: Pin<&mut Optimizer>, evaluator: &Evaluator) -> Result<()>;
        /// The error at the optimizer's values.
        fn error(optimizer: &Optimizer) -> f64;
        /// The current damping.
        fn damping(optimizer: &Optimizer) -> f64;
        /// The damping at which a search gives up.
        fn damping_limit(optimizer: &Optimizer) -> f64;
        /// Reads a pose.
        fn pose_at(optimizer: &Optimizer, key: u64, out: &mut [f64]) -> Result<()>;
        /// Reads a similarity.
        fn similarity_at(optimizer: &Optimizer, key: u64, out: &mut [f64]) -> Result<()>;
        /// Reads a point.
        fn point_at(optimizer: &Optimizer, key: u64, out: &mut [f64]) -> Result<()>;
    }
}

/// A variable's kind, as the bridge takes it.
const POSE: u8 = 0;
/// A similarity variable's kind.
const SIMILARITY: u8 = 1;
/// A point variable's kind.
const POINT: u8 = 2;

/// Wraps a failure inside GTSAM as `NotConverged`.
///
/// # Arguments
///
/// * `error` - The C++ exception.
///
/// # Returns
///
/// The error.
fn gtsam_failed(error: &cxx::Exception) -> crate::error::Error {
    crate::error::Error::new(
        ErrorKind::NotConverged,
        format!("GTSAM failed: {}", error.what()),
    )
}

/// A 4x4 matrix's 16 values, row-major.
///
/// # Arguments
///
/// * `matrix` - The matrix.
///
/// # Returns
///
/// The values.
fn rows_of(matrix: &Matrix4<f64>) -> [f64; 16] {
    let mut out = [0.0; 16];
    for (k, value) in out.iter_mut().enumerate() {
        *value = matrix[(k / 4, k % 4)];
    }
    out
}

/// GTSAM's `Similarity3` matrix, `[[R, t'], [0, 1/s]]` with `t' = t / s`, for
/// `[[s R, t], [0, 1]]` (`graphs.to_similarity3`).
///
/// # Arguments
///
/// * `matrix` - The similarity, its scale positive.
///
/// # Returns
///
/// GTSAM's matrix, 16 row-major.
fn to_similarity3(matrix: &Matrix4<f64>) -> [f64; 16] {
    let (rotation, translation, scale) = split(matrix);
    let mut gtsam = Matrix4::identity();
    gtsam.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
    gtsam
        .fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&(translation / scale));
    gtsam[(3, 3)] = 1.0 / scale;
    rows_of(&gtsam)
}

/// `[[s R, t], [0, 1]]` for GTSAM's `Similarity3` matrix, `[[R, t'], [0, 1/s]]`
/// (`graphs.from_similarity3`).
///
/// # Arguments
///
/// * `values` - GTSAM's matrix, 16 row-major.
///
/// # Returns
///
/// The similarity.
fn from_similarity3(values: &[f64]) -> Matrix4<f64> {
    let gtsam = Matrix4::from_row_slice(values);
    let scale = 1.0 / gtsam[(3, 3)];
    let mut out = Matrix4::identity();
    out.fixed_view_mut::<3, 4>(0, 0)
        .copy_from(&(gtsam.fixed_view::<3, 4>(0, 0) * scale));
    out
}

/// A key for an index: the bridge's keys are GTSAM's 64-bit keys.
///
/// # Arguments
///
/// * `index` - The index.
///
/// # Returns
///
/// The key.
fn key(index: usize) -> u64 {
    u64::try_from(index).unwrap_or(u64::MAX)
}

/// Runs Levenberg-Marquardt by `Termination`'s rule (`graphs._solve`).
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `evaluator` - Its Rust factors.
/// * `options` - How the solve runs, checked.
///
/// # Returns
///
/// The optimizer at its last values, and how the solve went.
///
/// # Errors
///
/// `NotConverged` if GTSAM fails.
fn solve(
    problem: &ffi::Problem,
    evaluator: &Evaluator,
    options: &Options,
) -> Result<(UniquePtr<ffi::Optimizer>, Summary)> {
    let mut optimizer = ffi::new_optimizer(
        problem,
        evaluator,
        options.initial_damping,
        options.relative_tolerance,
        options.absolute_tolerance,
    )
    .map_err(|e| gtsam_failed(&e))?;
    let initial = ffi::error(&optimizer);
    let mut current = initial;
    let mut termination = Termination::MaxIterations;
    let mut iterations = 0;
    while current > 0.0 && iterations < options.max_iterations {
        ffi::iterate(optimizer.pin_mut(), evaluator).map_err(|e| gtsam_failed(&e))?;
        iterations += 1;
        let error = ffi::error(&optimizer);
        if error >= current && ffi::damping(&optimizer) >= ffi::damping_limit(&optimizer) {
            termination = Termination::DampingLimit;
            break;
        }
        let decrease = current - error;
        let previous = current;
        current = error;
        if decrease <= options.absolute_tolerance
            || decrease <= options.relative_tolerance * previous
        {
            termination = Termination::Function;
            break;
        }
    }
    if current <= 0.0 {
        termination = Termination::Function;
    }
    let summary = Summary {
        initial_error: initial,
        final_error: current,
        iterations,
        termination,
        converged: termination.is_converged(),
    };
    Ok((optimizer, summary))
}

/// Reads a 4x4 value (a pose or a similarity) from the optimizer.
///
/// # Arguments
///
/// * `optimizer` - The optimizer.
/// * `index` - The variable's index.
/// * `similarity` - Whether it is a similarity.
///
/// # Returns
///
/// The matrix.
///
/// # Errors
///
/// `NotConverged` if GTSAM holds no such value.
fn matrix_at(optimizer: &ffi::Optimizer, index: usize, similarity: bool) -> Result<Matrix4<f64>> {
    let mut out = [0.0; 16];
    if similarity {
        ffi::similarity_at(optimizer, key(index), &mut out)
    } else {
        ffi::pose_at(optimizer, key(index), &mut out)
    }
    .map_err(|e| gtsam_failed(&e))?;
    Ok(if similarity {
        from_similarity3(&out)
    } else {
        Matrix4::from_row_slice(&out)
    })
}

// --- input checks ----------------------------------------------------------------------------

/// Checks a 4x4 pose: a last row of `[0, 0, 0, 1]` and a rotation, scaled for a similarity.
///
/// # Arguments
///
/// * `pose` - The pose.
/// * `similarity` - Whether a positive scale is allowed.
/// * `name` - What it is, for the message.
/// * `index` - Its index, for the message.
///
/// # Errors
///
/// `InvalidInput` naming it.
fn check_pose(pose: &Matrix4<f64>, similarity: bool, name: &str, index: usize) -> Result<()> {
    let at = format!("{name}[{index}]: ");
    if !pose.iter().all(|v| v.is_finite()) {
        return fail(ErrorKind::InvalidInput, at + "expected finite values");
    }
    if pose.row(3).iter().copied().ne([0.0, 0.0, 0.0, 1.0]) {
        return fail(
            ErrorKind::InvalidInput,
            at + "expected a last row of [0, 0, 0, 1]",
        );
    }
    let block: Matrix3<f64> = pose.fixed_view::<3, 3>(0, 0).into();
    let determinant = block.determinant();
    if !(determinant > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            at + "expected a positive determinant",
        );
    }
    let scale = if similarity { determinant.cbrt() } else { 1.0 };
    so3::check_rotation(&(block / scale))
        .map_err(|e| crate::error::Error::new(ErrorKind::InvalidInput, at + &e.message))
}

/// Checks a list of poses (`check_pose`).
///
/// # Arguments
///
/// * `poses` - The poses.
/// * `similarity` - Whether a positive scale is allowed.
/// * `name` - What they are, for the message.
///
/// # Errors
///
/// `InvalidInput`.
fn check_poses(poses: &[Matrix4<f64>], similarity: bool, name: &str) -> Result<()> {
    poses
        .iter()
        .enumerate()
        .try_for_each(|(k, pose)| check_pose(pose, similarity, name, k))
}

/// Checks indices against `[0, bound)`.
///
/// # Arguments
///
/// * `indices` - The indices.
/// * `bound` - The bound.
/// * `name` - What they are, for the message.
///
/// # Errors
///
/// `InvalidInput`.
fn check_indices(indices: impl IntoIterator<Item = usize>, bound: usize, name: &str) -> Result<()> {
    if indices.into_iter().any(|index| index >= bound) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected indices in [0, {bound})"),
        );
    }
    Ok(())
}

/// The sorted, distinct indices.
///
/// # Arguments
///
/// * `indices` - The indices.
///
/// # Returns
///
/// Them, each once, ascending.
fn distinct(indices: &[usize]) -> Vec<usize> {
    let mut out = indices.to_vec();
    out.sort_unstable();
    out.dedup();
    out
}

/// Checks a refinement's inputs (`refine_similarity`'s errors).
///
/// # Arguments
///
/// * `similarity` - The starting motion.
/// * `matches` - The matches.
/// * `model` - The camera model.
/// * `intrinsics` - Its intrinsics.
/// * `fixed_scale` - Whether the motion is rigid.
/// * `huber` - The Huber loss's threshold, or none.
/// * `options` - How the solve runs.
///
/// # Errors
///
/// `InvalidInput`, as `refine_similarity` documents.
fn check_refinement(
    similarity: &Matrix4<f64>,
    matches: &Matches<'_>,
    model: Model,
    intrinsics: &[f64],
    fixed_scale: bool,
    huber: Option<f64>,
    options: &Options,
) -> Result<()> {
    options.check()?;
    check_pose(similarity, !fixed_scale, "similarity", 0)?;
    let count = matches.points1.len();
    if count == 0
        || matches.points2.len() != count
        || matches.pixels1.len() != count
        || matches.pixels2.len() != count
        || matches.information.len() != count
    {
        return fail(
            ErrorKind::InvalidInput,
            "matches: expected equal (M, 3) and (M, 2) arrays, M >= 1",
        );
    }
    let finite = |v: &[f64]| v.iter().all(|x| x.is_finite());
    if !(matches.points1.iter().all(|p| finite(p.as_slice()))
        && matches.points2.iter().all(|p| finite(p.as_slice()))
        && matches.pixels1.iter().all(|p| finite(p.as_slice()))
        && matches
            .pixels2
            .iter()
            .all(|p| finite(p.as_slice()) || p.iter().all(|v| v.is_nan())))
    {
        return fail(ErrorKind::InvalidInput, "matches: expected finite values");
    }
    if !matches
        .information
        .iter()
        .flatten()
        .all(|w| w.is_finite() && *w > 0.0)
    {
        return fail(
            ErrorKind::InvalidInput,
            "information: expected positive values",
        );
    }
    if let Some(threshold) = huber
        && !(threshold.is_finite() && threshold > 0.0)
    {
        return fail_setting("huber", "expected > 0", threshold);
    }
    camera::check_intrinsics(model, intrinsics)
}

/// Checks a bundle's inputs (`bundle_adjust`'s errors).
///
/// # Arguments
///
/// * `poses` - The cameras' starts.
/// * `points` - The points' starts.
/// * `observations` - The observations.
/// * `lens` - The camera.
/// * `fixed` - The cameras held fixed.
/// * `options` - How the solve runs.
///
/// # Errors
///
/// `InvalidInput`, as `bundle_adjust` documents.
fn check_bundle(
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    observations: &Observations<'_>,
    lens: &Lens,
    fixed: &[usize],
    options: &Options,
) -> Result<()> {
    options.check()?;
    if poses.is_empty() {
        return fail(ErrorKind::InvalidInput, "poses: expected N >= 1");
    }
    check_poses(poses, false, "poses")?;
    if points.is_empty() || !points.iter().all(|p| p.iter().all(|v| v.is_finite())) {
        return fail(
            ErrorKind::InvalidInput,
            "points: expected (P, 3) finite values, P >= 1",
        );
    }
    let count = observations.cameras.len();
    if observations.point_ids.len() != count
        || observations.pixels.len() != count
        || observations.right.len() != count
        || observations.information.len() != count
    {
        return fail(
            ErrorKind::InvalidInput,
            "observations: expected cameras, point_ids, pixels, right alike",
        );
    }
    if !observations
        .pixels
        .iter()
        .all(|p| p.iter().all(|v| v.is_finite()))
    {
        return fail(ErrorKind::InvalidInput, "pixels: expected finite values");
    }
    if observations.right.iter().any(|v| v.is_infinite()) {
        return fail(
            ErrorKind::InvalidInput,
            "right: expected finite values or NaN",
        );
    }
    check_indices(observations.cameras.iter().copied(), poses.len(), "cameras")?;
    check_indices(
        observations.point_ids.iter().copied(),
        points.len(),
        "point_ids",
    )?;
    if !observations
        .information
        .iter()
        .all(|w| w.is_finite() && *w > 0.0)
    {
        return fail(
            ErrorKind::InvalidInput,
            "information: expected positive values",
        );
    }
    if distinct(observations.point_ids).len() != points.len() {
        return fail(
            ErrorKind::InvalidInput,
            "point_ids: every point must be observed",
        );
    }
    let stereo = observations.right.iter().any(|v| !v.is_nan());
    if stereo && !(lens.bf.is_finite() && lens.bf > 0.0) {
        return fail_setting("bf", "expected > 0 with a right column", lens.bf);
    }
    check_indices(fixed.iter().copied(), poses.len(), "fixed")?;
    camera::check_intrinsics(lens.model, &lens.intrinsics)
}

// --- the solves ------------------------------------------------------------------------------

/// Optimises a pose graph, each edge's measured relative pose with unit information
/// (`graphs.optimize_pose_graph`).
///
/// ORB-SLAM's `OptimizeEssentialGraph` on GTSAM: edge `(i, j)` measures `T_i_j = T_world_i^-1
/// T_world_j` (`BetweenFactor`); the poses in `fixed` are held exactly (`NonlinearEquality`).
///
/// # Arguments
///
/// * `poses` - Each pose's start, camera-to-world: rigid, or a similarity with `similarity`.
/// * `edges` - Each edge's two poses `(i, j)`, `i != j`.
/// * `relatives` - Each edge's measured `T_i_j`, of the poses' kind.
/// * `fixed` - The poses held fixed, at least one.
/// * `similarity` - Optimise Sim(3) rather than SE(3).
/// * `options` - How the solve runs.
///
/// # Returns
///
/// The poses and how the solve went.
///
/// # Errors
///
/// `InvalidInput` for a pose or relative not of the kind, mismatched sizes, an index out of
/// range, a self-edge, no fixed pose or bad options; `NotConverged` if GTSAM fails.
pub fn optimize_pose_graph(
    poses: &[Matrix4<f64>],
    edges: &[[usize; 2]],
    relatives: &[Matrix4<f64>],
    fixed: &[usize],
    similarity: bool,
    options: &Options,
) -> Result<PoseGraph> {
    options.check()?;
    if poses.is_empty() {
        return fail(ErrorKind::InvalidInput, "poses: expected N >= 1");
    }
    check_poses(poses, similarity, "poses")?;
    if edges.len() != relatives.len() {
        return fail(ErrorKind::InvalidInput, "relatives: expected one per edge");
    }
    check_poses(relatives, similarity, "relatives")?;
    check_indices(edges.iter().flatten().copied(), poses.len(), "edges")?;
    if edges.iter().any(|[i, j]| i == j) {
        return fail(
            ErrorKind::InvalidInput,
            "edges: an edge joins a pose to itself",
        );
    }
    check_indices(fixed.iter().copied(), poses.len(), "fixed")?;
    if fixed.is_empty() {
        return fail(
            ErrorKind::InvalidInput,
            "fixed: expected at least one pose held fixed (the gauge)",
        );
    }
    let mut problem = ffi::new_problem();
    let gtsam =
        |result: std::result::Result<(), cxx::Exception>| result.map_err(|e| gtsam_failed(&e));
    for (k, pose) in poses.iter().enumerate() {
        if similarity {
            gtsam(ffi::add_similarity(
                problem.pin_mut(),
                key(k),
                &to_similarity3(pose),
            ))?;
        } else {
            gtsam(ffi::add_pose(problem.pin_mut(), key(k), &rows_of(pose)))?;
        }
    }
    for ([i, j], relative) in edges.iter().zip(relatives) {
        if similarity {
            gtsam(ffi::add_between_similarities(
                problem.pin_mut(),
                key(*i),
                key(*j),
                &to_similarity3(relative),
            ))?;
        } else {
            gtsam(ffi::add_between_poses(
                problem.pin_mut(),
                key(*i),
                key(*j),
                &rows_of(relative),
            ))?;
        }
    }
    for k in distinct(fixed) {
        if similarity {
            gtsam(ffi::fix_similarity(
                problem.pin_mut(),
                key(k),
                &to_similarity3(&poses[k]),
            ))?;
        } else {
            gtsam(ffi::fix_pose(
                problem.pin_mut(),
                key(k),
                &rows_of(&poses[k]),
            ))?;
        }
    }
    let evaluator = Evaluator {
        lens: Lens {
            model: Model::Pinhole,
            intrinsics: Vec::new(),
            bf: 0.0,
        },
        factors: Vec::new(),
    };
    let (optimizer, summary) = solve(&problem, &evaluator, options)?;
    let poses = (0..poses.len())
        .map(|k| matrix_at(&optimizer, k, similarity))
        .collect::<Result<Vec<_>>>()?;
    Ok(PoseGraph { poses, summary })
}

/// Refines the similarity carrying one keyframe's matched points into another's images
/// (`graphs.refine_similarity`).
///
/// ORB-SLAM's `OptimizeSim3` on GTSAM, one round: `S_12` is the only variable, each match two
/// errors (its camera-2 point in image 1, its camera-1 point in image 2) with a Huber loss.
///
/// # Arguments
///
/// * `similarity` - The starting `S_12`, `s > 0` (`s = 1` with `fixed_scale`).
/// * `matches` - The matches, at least one.
/// * `model` - The camera model, the same for both images.
/// * `intrinsics` - Its intrinsics.
/// * `fixed_scale` - Hold the scale at 1, refining a rigid motion.
/// * `huber` - The Huber loss's threshold on the whitened error's norm, positive; none for no
///   robust loss (`OptimizeSim3`'s second round).
/// * `options` - How the solve runs.
///
/// # Returns
///
/// The motion, each match's chi-squares and how the solve went.
///
/// # Errors
///
/// `InvalidInput` for a motion not of the kind, mismatched sizes, no match, a non-positive
/// information or threshold, a bad camera or bad options; `NotConverged` if GTSAM fails.
pub fn refine_similarity(
    similarity: &Matrix4<f64>,
    matches: &Matches<'_>,
    model: Model,
    intrinsics: &[f64],
    fixed_scale: bool,
    huber: Option<f64>,
    options: &Options,
) -> Result<SimilarityRefinement> {
    check_refinement(
        similarity,
        matches,
        model,
        intrinsics,
        fixed_scale,
        huber,
        options,
    )?;
    let count = matches.points1.len();
    let lens = Lens {
        model,
        intrinsics: intrinsics.to_vec(),
        bf: 0.0,
    };
    let mut problem = ffi::new_problem();
    let kind = if fixed_scale { POSE } else { SIMILARITY };
    let gtsam =
        |result: std::result::Result<(), cxx::Exception>| result.map_err(|e| gtsam_failed(&e));
    if fixed_scale {
        gtsam(ffi::add_pose(problem.pin_mut(), 0, &rows_of(similarity)))?;
    } else {
        gtsam(ffi::add_similarity(
            problem.pin_mut(),
            0,
            &to_similarity3(similarity),
        ))?;
    }
    let mut factors = Vec::with_capacity(2 * count);
    for k in 0..count {
        // A match image 2 does not observe (a NaN pixel) counts its image-1 error alone.
        let seen2 = !matches.pixels2[k].x.is_nan();
        for (side, first) in [(0, true), (1, false)] {
            if !first && !seen2 {
                continue;
            }
            gtsam(ffi::add_rust_factor(
                problem.pin_mut(),
                &[0],
                &[kind],
                2,
                matches.information[k][side],
                huber.unwrap_or(0.0), // the bridge's 0: no robust loss
                factors.len(),
            ))?;
            factors.push(Factor::Similarity {
                point1: matches.points1[k],
                point2: matches.points2[k],
                pixel1: matches.pixels1[k],
                pixel2: matches.pixels2[k],
                first,
                fixed_scale,
            });
        }
    }
    let evaluator = Evaluator { lens, factors };
    let (optimizer, summary) = solve(&problem, &evaluator, options)?;
    let motion = matrix_at(&optimizer, 0, !fixed_scale)?;
    let chi2 = (0..count)
        .map(|k| {
            let both = similarity_residuals(
                &motion,
                &matches.points1[k],
                &matches.points2[k],
                &matches.pixels1[k],
                &matches.pixels2[k],
                &evaluator.lens,
                fixed_scale,
            );
            let weights = matches.information[k];
            [
                if both.in_front1 {
                    weights[0] * both.residual1.norm_squared()
                } else {
                    f64::INFINITY
                },
                if matches.pixels2[k].x.is_nan() {
                    f64::NAN
                } else if both.in_front2 {
                    weights[1] * both.residual2.norm_squared()
                } else {
                    f64::INFINITY
                },
            ]
        })
        .collect();
    Ok(SimilarityRefinement {
        similarity: motion,
        chi2,
        summary,
    })
}

/// Adjusts every pose and point against every observation, with no robust loss
/// (`graphs.bundle_adjust`).
///
/// ORB-SLAM's `GlobalBundleAdjustemnt` as loop closing calls it (`bRobust` false), on GTSAM.
///
/// # Arguments
///
/// * `poses` - Each camera's starting `T_world_cam`.
/// * `points` - Each point's start, in the world, at least one.
/// * `observations` - The observations; every point must be observed.
/// * `lens` - The camera; its `bf` positive if any observation has a right column.
/// * `fixed` - The cameras held fixed.
/// * `options` - How the solve runs.
///
/// # Returns
///
/// The poses, the points and how the solve went.
///
/// # Errors
///
/// `InvalidInput` for a pose not rigid, mismatched sizes, an index out of range, an
/// unobserved point, a non-positive information, a right column without `bf`, a bad camera or
/// bad options; `NotConverged` if GTSAM fails.
pub fn bundle_adjust(
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    observations: &Observations<'_>,
    lens: &Lens,
    fixed: &[usize],
    options: &Options,
) -> Result<Adjustment> {
    check_bundle(poses, points, observations, lens, fixed, options)?;
    let count = observations.cameras.len();
    let mut problem = ffi::new_problem();
    let gtsam =
        |result: std::result::Result<(), cxx::Exception>| result.map_err(|e| gtsam_failed(&e));
    let first_point = poses.len();
    for (c, pose) in poses.iter().enumerate() {
        gtsam(ffi::add_pose(problem.pin_mut(), key(c), &rows_of(pose)))?;
    }
    for (p, point) in points.iter().enumerate() {
        gtsam(ffi::add_point(
            problem.pin_mut(),
            key(first_point + p),
            point.as_slice(),
        ))?;
    }
    for c in distinct(fixed) {
        gtsam(ffi::fix_pose(
            problem.pin_mut(),
            key(c),
            &rows_of(&poses[c]),
        ))?;
    }
    let mut factors = Vec::with_capacity(count);
    for m in 0..count {
        let pixel = observations.pixels[m];
        let measured = if observations.right[m].is_nan() {
            vec![pixel.x, pixel.y]
        } else {
            vec![pixel.x, pixel.y, observations.right[m]]
        };
        gtsam(ffi::add_rust_factor(
            problem.pin_mut(),
            &[
                key(observations.cameras[m]),
                key(first_point + observations.point_ids[m]),
            ],
            &[POSE, POINT],
            measured.len(),
            observations.information[m],
            0.0,
            factors.len(),
        ))?;
        factors.push(Factor::Projection { measured });
    }
    let evaluator = Evaluator {
        lens: lens.clone(),
        factors,
    };
    let (optimizer, summary) = solve(&problem, &evaluator, options)?;
    let poses = (0..poses.len())
        .map(|c| matrix_at(&optimizer, c, false))
        .collect::<Result<Vec<_>>>()?;
    let points = (0..points.len())
        .map(|p| {
            let mut out = [0.0; 3];
            ffi::point_at(&optimizer, key(first_point + p), &mut out)
                .map_err(|e| gtsam_failed(&e))?;
            Ok(Vector3::from(out))
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(Adjustment {
        poses,
        points,
        summary,
    })
}

#[cfg(test)]
mod tests {
    //! Native tests, runnable without Python: the factors' Jacobians against central
    //! differences, each solve recovering a known answer, how a solve stops, and bad input.

    use super::*;
    use crate::se3;
    use nalgebra::Vector6;

    /// The central differences' step.
    const STEP: f64 = 1e-6;
    /// A stereo rig's baseline times its focal length, in pixel-metres.
    const BF: f64 = 40.0;

    /// A pinhole camera.
    fn pinhole() -> Lens {
        Lens {
            model: Model::Pinhole,
            intrinsics: vec![520.0, 515.0, 320.0, 240.0],
            bf: BF,
        }
    }

    /// A radtan camera (TUM fr1's).
    fn radtan() -> Lens {
        Lens {
            model: Model::RadTan,
            intrinsics: vec![
                517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633,
            ],
            bf: BF,
        }
    }

    /// `[[s R, t], [0, 1]]`.
    fn similarity(rotation: &Matrix3<f64>, translation: &Vector3<f64>, scale: f64) -> Matrix4<f64> {
        let mut out = Matrix4::identity();
        out.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(scale * rotation));
        out.fixed_view_mut::<3, 1>(0, 3).copy_from(translation);
        out
    }

    /// A pose from rung 0's SE(3) exponential, its tangent `(rho, theta)`.
    fn pose(rho: [f64; 3], theta: [f64; 3]) -> Matrix4<f64> {
        se3::exp(&Vector6::new(
            rho[0], rho[1], rho[2], theta[0], theta[1], theta[2],
        ))
    }

    /// Retracts a motion on the right by GTSAM's tangent, to first order its retraction.
    fn retract(motion: &Matrix4<f64>, delta: &[f64]) -> Matrix4<f64> {
        let w = [delta[0], delta[1], delta[2]];
        let v = [delta[3], delta[4], delta[5]];
        if delta.len() == POSE_DIM {
            return motion * pose(v, w);
        }
        let grow = delta[6].exp();
        motion
            * similarity(
                &so3::exp(&Vector3::from(w)),
                &(grow * Vector3::from(v)),
                grow,
            )
    }

    /// Differentiates a function at 0 by central differences, a column per coordinate.
    fn numeric(function: impl Fn(&[f64]) -> DVector<f64>, dimension: usize) -> DMatrix<f64> {
        let columns: Vec<DVector<f64>> = (0..dimension)
            .map(|k| {
                let mut plus = vec![0.0; dimension];
                let mut minus = vec![0.0; dimension];
                plus[k] = STEP;
                minus[k] = -STEP;
                (function(&plus) - function(&minus)) / (2.0 * STEP)
            })
            .collect();
        DMatrix::from_columns(&columns)
    }

    /// Whether two matrices agree within `atol + rtol |expected|`, entry by entry.
    fn close(actual: &DMatrix<f64>, expected: &DMatrix<f64>, rtol: f64, atol: f64) -> bool {
        actual.shape() == expected.shape()
            && actual
                .iter()
                .zip(expected.iter())
                .all(|(a, e)| (a - e).abs() <= atol + rtol * e.abs())
    }

    #[test]
    fn the_projection_factor_s_jacobians_are_its_derivatives() {
        let camera_pose = pose([0.1, -0.2, 0.05], [0.2, -0.1, 0.3]);
        let point = camera_pose.fixed_view::<3, 3>(0, 0) * Vector3::new(0.2, -0.1, 2.5)
            + camera_pose.fixed_view::<3, 1>(0, 3);
        for lens in [pinhole(), radtan()] {
            for measured in [vec![300.0, 250.0], vec![300.0, 250.0, 290.0]] {
                let at = projection_residual(&camera_pose, &point, &measured, &lens).unwrap();
                assert!(at.in_front);
                let on_pose = |delta: &[f64]| {
                    projection_residual(&retract(&camera_pose, delta), &point, &measured, &lens)
                        .unwrap()
                        .residual
                };
                let on_point = |delta: &[f64]| {
                    let moved = point + Vector3::new(delta[0], delta[1], delta[2]);
                    projection_residual(&camera_pose, &moved, &measured, &lens)
                        .unwrap()
                        .residual
                };
                assert!(close(&at.d_pose, &numeric(on_pose, POSE_DIM), 1e-6, 1e-5));
                assert!(close(&at.d_point, &numeric(on_point, 3), 1e-6, 1e-5));
            }
        }
    }

    #[test]
    fn the_similarity_factors_jacobians_are_their_derivatives() {
        let point2 = Vector3::new(0.3, -0.2, 2.4);
        let (pixel1, pixel2) = (Vector2::new(350.0, 220.0), Vector2::new(330.0, 230.0));
        for fixed_scale in [false, true] {
            let motion = similarity(
                &so3::exp(&Vector3::new(0.05, -0.1, 0.02)),
                &Vector3::new(0.1, -0.05, 0.2),
                if fixed_scale { 1.0 } else { 1.3 },
            );
            let point1 = motion.fixed_view::<3, 3>(0, 0) * point2
                + motion.fixed_view::<3, 1>(0, 3)
                + Vector3::new(0.01, -0.02, 0.03);
            let lens = radtan();
            let at = similarity_residuals(
                &motion,
                &point1,
                &point2,
                &pixel1,
                &pixel2,
                &lens,
                fixed_scale,
            );
            assert!(at.in_front1 && at.in_front2);
            let dimension = if fixed_scale {
                POSE_DIM
            } else {
                SIMILARITY_DIM
            };
            for first in [true, false] {
                let side = |delta: &[f64]| {
                    let moved = similarity_residuals(
                        &retract(&motion, delta),
                        &point1,
                        &point2,
                        &pixel1,
                        &pixel2,
                        &lens,
                        fixed_scale,
                    );
                    let error = if first {
                        moved.residual1
                    } else {
                        moved.residual2
                    };
                    DVector::from_column_slice(error.as_slice())
                };
                let analytic = if first {
                    &at.d_similarity1
                } else {
                    &at.d_similarity2
                };
                assert!(close(analytic, &numeric(side, dimension), 1e-6, 1e-5));
            }
        }
    }

    #[test]
    fn a_point_behind_its_camera_contributes_nothing() {
        let behind = projection_residual(
            &Matrix4::identity(),
            &Vector3::new(0.0, 0.0, -1.0),
            &[1.0, 2.0, 3.0],
            &pinhole(),
        )
        .unwrap();
        assert!(!behind.in_front);
        assert!(
            behind
                .residual
                .iter()
                .chain(behind.d_pose.iter())
                .all(|v| *v == 0.0)
        );
        let both = similarity_residuals(
            &Matrix4::identity(),
            &Vector3::new(0.0, 0.0, 2.0),
            &Vector3::new(0.0, 0.0, -2.0),
            &Vector2::zeros(),
            &Vector2::zeros(),
            &pinhole(),
            false,
        );
        assert!(!both.in_front1 && both.in_front2);
        assert!(both.d_similarity1.iter().all(|v| *v == 0.0));
    }

    /// A ring of camera-to-world poses round the origin, scaled with `with_scale`.
    fn ring(count: usize, with_scale: bool) -> Vec<Matrix4<f64>> {
        (0..count)
            .map(|k| {
                #[allow(clippy::cast_precision_loss)] // a handful of poses
                let (k_f, count_f) = (k as f64, count as f64);
                let angle = 2.0 * std::f64::consts::PI * k_f / count_f;
                let rigid = pose(
                    [3.0 * angle.cos(), 3.0 * angle.sin(), 0.0],
                    [0.0, 0.0, angle],
                );
                similarity(
                    &rigid.fixed_view::<3, 3>(0, 0).into(),
                    &rigid.fixed_view::<3, 1>(0, 3).into(),
                    if with_scale { 1.0 + 0.1 * k_f } else { 1.0 },
                )
            })
            .collect()
    }

    /// The inverse of `[[s R, t], [0, 1]]`, in closed form: `[[R^T / s, -R^T t / s], [0, 1]]`.
    fn inverse(matrix: &Matrix4<f64>) -> Matrix4<f64> {
        let (rotation, translation, scale) = split(matrix);
        similarity(
            &rotation.transpose(),
            &(-(rotation.transpose() * translation) / scale),
            1.0 / scale,
        )
    }

    /// A ring's edges round it and each edge's relative.
    fn ring_edges(truth: &[Matrix4<f64>]) -> (Vec<[usize; 2]>, Vec<Matrix4<f64>>) {
        let n = truth.len();
        let edges: Vec<[usize; 2]> = (0..n).map(|k| [k, (k + 1) % n]).collect();
        let relatives = edges
            .iter()
            .map(|[i, j]| inverse(&truth[*i]) * truth[*j])
            .collect();
        (edges, relatives)
    }

    #[test]
    fn a_pose_graph_closes_its_ring() {
        for with_scale in [false, true] {
            let truth = ring(8, with_scale);
            let (edges, relatives) = ring_edges(&truth);
            let mut start = truth.clone();
            for (k, value) in start.iter_mut().enumerate().skip(1) {
                #[allow(clippy::cast_precision_loss)] // a handful of poses
                let k_f = k as f64;
                *value *= pose(
                    [0.05 * k_f, -0.03 * k_f, 0.02],
                    [0.01 * k_f, 0.0, -0.02 * k_f],
                );
                if with_scale {
                    let grown = value.fixed_view::<3, 3>(0, 0) * (1.0 + 0.02 * k_f);
                    value.fixed_view_mut::<3, 3>(0, 0).copy_from(&grown);
                }
            }
            let options = Options {
                max_iterations: 30,
                ..Options::default()
            };
            let result =
                optimize_pose_graph(&start, &edges, &relatives, &[0], with_scale, &options)
                    .unwrap();
            assert_eq!(result.poses[0], truth[0]);
            for (got, expected) in result.poses.iter().zip(&truth) {
                assert!((got - expected).abs().max() < 1e-8);
            }
            assert!(result.summary.final_error < 1e-15 && result.summary.initial_error > 1e-15);
        }
    }

    #[test]
    fn a_similarity_is_refined_its_outlier_dropped_and_refined_again() {
        const MATCHES: usize = 40;
        for fixed_scale in [false, true] {
            let truth = similarity(
                &so3::exp(&Vector3::new(0.05, 0.1, -0.03)),
                &Vector3::new(0.2, -0.1, 0.1),
                if fixed_scale { 1.0 } else { 0.8 },
            );
            let lens = pinhole();
            let (mut points1, mut points2, mut pixels1, mut pixels2) =
                (Vec::new(), Vec::new(), Vec::new(), Vec::new());
            for k in 0..MATCHES {
                #[allow(clippy::cast_precision_loss)] // forty matches
                let k_f = k as f64;
                let p2 = Vector3::new(
                    (1.3 * k_f).sin(),
                    (0.7 * k_f).cos(),
                    3.5 + 1.5 * (0.4 * k_f).sin(),
                );
                let p1 = truth.fixed_view::<3, 3>(0, 0) * p2 + truth.fixed_view::<3, 1>(0, 3);
                pixels1.push(camera::project(lens.model, &lens.intrinsics, &p1).unwrap());
                pixels2.push(camera::project(lens.model, &lens.intrinsics, &p2).unwrap());
                points1.push(p1);
                points2.push(p2);
            }
            pixels1[0] += Vector2::new(40.0, -35.0); // one wrong match
            let information = vec![[1.0, 1.0]; MATCHES];
            let options = Options {
                max_iterations: 20,
                ..Options::default()
            };
            // ORB-SLAM's two rounds: a Huber loss, then none on the inliers.
            let refine = |start: &Matrix4<f64>, from: usize, huber: Option<f64>| {
                refine_similarity(
                    start,
                    &Matches {
                        points1: &points1[from..],
                        points2: &points2[from..],
                        pixels1: &pixels1[from..],
                        pixels2: &pixels2[from..],
                        information: &information[from..],
                    },
                    lens.model,
                    &lens.intrinsics,
                    fixed_scale,
                    huber,
                    &options,
                )
                .unwrap()
            };
            let start = truth * pose([0.01, -0.01, 0.02], [0.02, -0.02, 0.01]);
            let first = refine(&start, 0, Some(10.0_f64.sqrt()));
            assert!((first.similarity - truth).abs().max() < 1e-2);
            assert!(first.chi2[0][0] > 1000.0);
            let second = refine(&first.similarity, 1, None);
            assert!((second.similarity - truth).abs().max() < 1e-12);
        }
    }

    #[test]
    fn bundle_adjustment_recovers_a_stereo_scene() {
        const CAMERAS: usize = 5;
        const POINTS: usize = 60;
        #[allow(clippy::cast_precision_loss)] // a handful of cameras and points
        let truth: Vec<Matrix4<f64>> = (0..CAMERAS)
            .map(|c| pose([0.4 * c as f64, 0.0, 0.0], [0.0, 0.05 * c as f64, 0.0]))
            .collect();
        #[allow(clippy::cast_precision_loss)]
        let points: Vec<Vector3<f64>> = (0..POINTS)
            .map(|p| {
                let p_f = p as f64;
                Vector3::new(-1.0 + 0.065 * p_f, p_f.sin() * 0.9, 3.0 + 0.05 * p_f)
            })
            .collect();
        let lens = radtan();
        let (mut cameras, mut ids, mut pixels, mut right) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for (c, camera_pose) in truth.iter().enumerate() {
            for (p, point) in points.iter().enumerate() {
                let rotation: Matrix3<f64> = camera_pose.fixed_view::<3, 3>(0, 0).into();
                let local = rotation.transpose() * (point - camera_pose.fixed_view::<3, 1>(0, 3));
                let seen = camera::project(lens.model, &lens.intrinsics, &local).unwrap();
                pixels.push(seen);
                right.push(if p % 2 == 1 {
                    seen.x - BF / local.z
                } else {
                    f64::NAN
                });
                cameras.push(c);
                ids.push(p);
            }
        }
        let mut start = truth.clone();
        for (c, value) in start.iter_mut().enumerate().skip(1) {
            #[allow(clippy::cast_precision_loss)]
            let c_f = c as f64;
            *value *= pose([0.01 * c_f, -0.02, 0.015], [0.01, -0.005 * c_f, 0.02]);
        }
        #[allow(clippy::cast_precision_loss)]
        let start_points: Vec<Vector3<f64>> = points
            .iter()
            .enumerate()
            .map(|(p, point)| point + Vector3::new(0.03, -0.02, 0.04) * (p as f64).cos())
            .collect();
        let information = vec![1.0; cameras.len()];
        let options = Options {
            max_iterations: 20,
            ..Options::default()
        };
        let result = bundle_adjust(
            &start,
            &start_points,
            &Observations {
                cameras: &cameras,
                point_ids: &ids,
                pixels: &pixels,
                right: &right,
                information: &information,
            },
            &lens,
            &[0],
            &options,
        )
        .unwrap();
        assert_eq!(result.poses[0], truth[0]);
        for (got, expected) in result.poses.iter().zip(&truth) {
            assert!((got - expected).abs().max() < 1e-7);
        }
        for (got, expected) in result.points.iter().zip(&points) {
            assert!((got - expected).abs().max() < 1e-6);
        }
    }

    #[test]
    fn a_solve_stops_by_its_iterations_its_tolerances_or_its_damping() {
        let truth = ring(6, false);
        let (edges, mut relatives) = ring_edges(&truth);
        relatives[0] *= pose([0.1, 0.0, 0.0], [0.0, 0.0, 0.05]); // inconsistent
        let mut start: Vec<Matrix4<f64>> = truth
            .iter()
            .map(|p| p * pose([0.1, 0.1, 0.0], [0.0, 0.0, 0.1]))
            .collect();
        start[0] = truth[0];
        let solve_with = |poses: &[Matrix4<f64>], options: Options| {
            optimize_pose_graph(poses, &edges, &relatives, &[0], false, &options).unwrap()
        };
        let capped = solve_with(
            &start,
            Options {
                max_iterations: 1,
                ..Options::default()
            },
        );
        assert_eq!(capped.summary.iterations, 1);
        assert_eq!(capped.summary.termination, Termination::MaxIterations);
        assert!(!capped.summary.converged);
        let tolerant = solve_with(
            &start,
            Options {
                max_iterations: 50,
                relative_tolerance: 1e-6,
                ..Options::default()
            },
        );
        assert_eq!(tolerant.summary.termination, Termination::Function);
        assert!(tolerant.summary.converged);
        let at_minimum = solve_with(
            &tolerant.poses,
            Options {
                max_iterations: 50,
                relative_tolerance: 0.0,
                ..Options::default()
            },
        );
        assert_eq!(at_minimum.summary.termination, Termination::DampingLimit);
    }

    #[test]
    fn bad_graphs_are_refused() {
        let truth = ring(4, false);
        let relatives = vec![inverse(&truth[0]) * truth[1]];
        let refused = |result: Result<PoseGraph>| matches!(result, Err(e) if e.kind == ErrorKind::InvalidInput);
        let options = Options::default();
        assert!(refused(optimize_pose_graph(
            &truth,
            &[[0, 1]],
            &relatives,
            &[],
            false,
            &options
        )));
        assert!(refused(optimize_pose_graph(
            &truth,
            &[[1, 1]],
            &relatives,
            &[0],
            false,
            &options
        )));
        assert!(refused(optimize_pose_graph(
            &truth,
            &[[0, 4]],
            &relatives,
            &[0],
            false,
            &options
        )));
        let mut scaled = truth.clone();
        let grown = scaled[1].fixed_view::<3, 3>(0, 0) * 2.0;
        scaled[1].fixed_view_mut::<3, 3>(0, 0).copy_from(&grown);
        assert!(refused(optimize_pose_graph(
            &scaled,
            &[[0, 1]],
            &relatives,
            &[0],
            false,
            &options
        )));
        assert!(
            Options {
                max_iterations: 0,
                ..Options::default()
            }
            .check()
            .is_err()
        );
        assert_eq!(Termination::DampingLimit.name(), "damping_limit");
    }
}
