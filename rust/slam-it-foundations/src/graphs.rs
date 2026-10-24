//! Factor graphs on GTSAM: a pose graph, a similarity refined against matches, and BA.
//!
//! Mirrors `slam_it.foundations.graphs`, whose module docstring states the conventions: poses
//! cross camera-to-world (`T_world_cam`), a similarity as `[[s R, t], [0, 1]]` mapping `p` to
//! `s R p + t`; Jacobians are on GTSAM's right-perturbed tangents, rotation first: `(w, v)`
//! for a pose, `(w, v, lambda)` for a similarity.
//!
//! GTSAM is reached through a `cxx` bridge (`cxx/graphs_bridge.cpp`): its graph, its values,
//! its between factors, and its Levenberg-Marquardt. The two factors GTSAM lacks are this
//! crate's own: a generic C++ factor calls back into Rust (`evaluate`), which computes the
//! residual and its Jacobians on this crate's camera model, as GTSAM's `CustomFactor` calls back
//! into Python. The solve is driven from here one iteration at a time, by the rule every
//! language stops by (`Termination`).
//!
//! A held camera or pose is a constant, never a variable, as g2o's fixed vertices hold
//! ORB-SLAM's (`slam_it.foundations.graphs` says why): a held camera's observation is a factor
//! on its point alone, a pose graph's edge with one held end a factor on its other end, and the
//! edges with both ends held, whose error is a constant, one factor with a zero Jacobian, so that
//! every error GTSAM reads is the whole graph's.
//!
//! The bridge is the one place in the crate that writes `unsafe`: `cxx` declares the C++ side
//! of a bridge in an `unsafe extern "C++"` block, the promise that its signatures are true.
//! `cxx` checks the rest; the crate allows `unsafe` in that module alone.

use std::pin::Pin;

use cxx::UniquePtr;
use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, Vector2, Vector3};

use crate::camera::{self, Model};
use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::nls::Stop;
use crate::so3;

/// A pose's tangent dimension, `(w, v)`.
pub const POSE_DIM: usize = 6;
/// A similarity's tangent dimension, `(w, v, lambda)`.
pub const SIMILARITY_DIM: usize = 7;

/// GTSAM's own damping's factor, its `lambdaFactor` default (`graphs.ABSOLUTE_DAMPING_FACTOR`):
/// the damping divided by it after a step that lowers the error, multiplied by it after one that
/// does not.
pub const ABSOLUTE_DAMPING_FACTOR: f64 = 10.0;
/// GTSAM's own damping's upper bound, its `lambdaUpperBound` default
/// (`graphs.ABSOLUTE_DAMPING_LIMIT`): a search that reaches it gives up.
pub const ABSOLUTE_DAMPING_LIMIT: f64 = 1e5;
/// Nielsen's first factor for the scaled damping (`graphs.SCALED_DAMPING_FACTOR`): a failed trial
/// multiplies the damping by 2, 4, 8, ..., as g2o's `_ni`; an accepted step by
/// `max(1/3, 1 - (2 rho - 1)^3)` (GTSAM's `useFixedLambdaFactor = false`, without g2o's cap at
/// 2/3).
pub const SCALED_DAMPING_FACTOR: f64 = 2.0;
/// The scaled damping's upper bound, GTSAM's own for its Ceres-like parameters
/// (`graphs.SCALED_DAMPING_LIMIT`): in practice none, as g2o has none.
pub const SCALED_DAMPING_LIMIT: f64 = 1e32;

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
    /// Not converged: stopped from outside, between iterations (`Options::stop`).
    Stopped = 3,
}

impl Termination {
    /// Every termination, in code order.
    pub const ALL: [Self; 4] = [
        Self::Function,
        Self::MaxIterations,
        Self::DampingLimit,
        Self::Stopped,
    ];

    /// The termination's name, as the Python side spells it.
    ///
    /// # Returns
    ///
    /// `"function"`, `"max_iterations"`, `"damping_limit"` or `"stopped"`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Function => "function",
            Self::MaxIterations => "max_iterations",
            Self::DampingLimit => "damping_limit",
            Self::Stopped => "stopped",
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
#[derive(Clone, Debug, PartialEq)]
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
    /// Follow g2o's damping (`OptimizationAlgorithmLevenberg`, which ORB-SLAM3's BAs run)
    /// rather than GTSAM's own: the first damping `initial_damping` times the largest diagonal
    /// entry of the normal equations at the start (g2o's `tau`), then Nielsen's schedule
    /// (`SCALED_DAMPING_FACTOR`) up to `SCALED_DAMPING_LIMIT`; GTSAM's own starts at
    /// `initial_damping`, divides it by 10 after a step that lowers the error and multiplies it
    /// by 10 after one that does not, up to `ABSOLUTE_DAMPING_LIMIT`.
    pub scaled_damping: bool,
    /// End the solve as converged once this many iterations in a row have each lowered the
    /// error by less than `small_decrease` of it (ORB-SLAM3's g2o: 3); 0, never.
    pub small_steps: usize,
    /// See `small_steps` (ORB-SLAM3's 1e-3); positive.
    pub small_decrease: f64,
    /// Stops the solve before its next iteration when another thread sets it (a global BA
    /// superseded, ORB-SLAM's `mbStopGBA`).
    pub stop: Stop,
}

impl Default for Options {
    /// The Python side's defaults.
    ///
    /// # Returns
    ///
    /// Ten iterations, a first damping of 1e-5, a relative tolerance of 1e-10 and no absolute
    /// one; GTSAM's own damping, and no stop by small steps.
    fn default() -> Self {
        Self {
            max_iterations: 10,
            initial_damping: 1e-5,
            relative_tolerance: 1e-10,
            absolute_tolerance: 0.0,
            scaled_damping: false,
            small_steps: 0,
            small_decrease: 1e-3,
            stop: Stop::never(),
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
        if !(self.small_decrease.is_finite() && self.small_decrease > 0.0) {
            return fail_setting("small_decrease", "expected > 0", self.small_decrease);
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
    /// Each observation's Huber threshold on its whitened error's norm, positive; empty for no
    /// robust loss.
    pub huber: &'a [f64],
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
    /// Whether the point is in front of the camera (`Z > camera::MIN_DEPTH`). One that is not
    /// cannot be projected: its residual is twice the focal length in every row, its Jacobians
    /// zero, GTSAM's own projection factors' convention (a constant error with no gradient, so
    /// a solve gains nothing by moving a point behind its camera).
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
        let fx = lens.intrinsics[lens.model.focal_indices()[0]];
        return Ok(ProjectionResidual {
            residual: DVector::from_element(size, 2.0 * fx),
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
    /// A point's reprojection in a held camera: variable the point alone.
    HeldProjection {
        /// The observation: `(u, v)`, and the right column if it has one.
        measured: Vec<f64>,
        /// The held camera's `T_world_cam`.
        pose: Matrix4<f64>,
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
    /// A pose graph's edges whose ends are both held: their residuals, stacked, a constant; its
    /// one variable a free pose (or similarity), which they do not depend on (a zero Jacobian).
    HeldEdges {
        /// The residuals.
        residuals: Vec<f64>,
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

/// A pose's or a similarity's values as they cross the bridge: its 4x4 matrix, row-major.
const MATRIX_VALUES: usize = 16;

/// A point's values as they cross the bridge.
const POINT_VALUES: usize = 3;

/// A projection factor's values: its pose's matrix, then its point's 3.
const PROJECTION_VALUES: usize = MATRIX_VALUES + POINT_VALUES;

/// GTSAM's call into a Rust factor: its residual and its Jacobian (each variable's block side
/// by side, row-major).
///
/// A call from C++ must not panic: a factor index or a value count it does not know leaves the
/// outputs at zero, which the factors built here never produce. It does rely on one invariant:
/// the bridge sizes `residual` by the factor's own dimension (2, or 3 with a right column), as
/// every factor built here declares, so the copy into it cannot fail.
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
        Some(Factor::Projection { measured }) if values.len() == PROJECTION_VALUES => {
            let pose = Matrix4::from_row_slice(&values[..MATRIX_VALUES]);
            let point = Vector3::from_column_slice(&values[MATRIX_VALUES..]);
            if let Ok(at) = projection_residual(&pose, &point, measured, &evaluator.lens) {
                let mut both = DMatrix::zeros(at.residual.len(), POSE_DIM + 3);
                both.columns_mut(0, POSE_DIM).copy_from(&at.d_pose);
                both.columns_mut(POSE_DIM, 3).copy_from(&at.d_point);
                residual.copy_from_slice(at.residual.as_slice());
                write_rows(&both, jacobian);
            }
        }
        Some(Factor::HeldProjection { measured, pose }) if values.len() == POINT_VALUES => {
            let point = Vector3::from_column_slice(values);
            if let Ok(at) = projection_residual(pose, &point, measured, &evaluator.lens) {
                residual.copy_from_slice(at.residual.as_slice());
                write_rows(&at.d_point, jacobian);
            }
        }
        Some(Factor::Similarity {
            point1,
            point2,
            pixel1,
            pixel2,
            first,
            fixed_scale,
        }) if values.len() == MATRIX_VALUES => {
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
        Some(Factor::HeldEdges { residuals })
            if values.len() == MATRIX_VALUES && residuals.len() == residual.len() =>
        {
            residual.copy_from_slice(residuals); // its Jacobian stays zero
        }
        _ => {}
    }
}

/// The bridge to GTSAM (`cxx/graphs_bridge.hpp`).
#[cxx::bridge(namespace = "slam_it::rust_bridge")]
#[allow(unsafe_code, clippy::missing_errors_doc, clippy::needless_lifetimes)]
#[allow(
    clippy::too_many_arguments,
    reason = "new_optimizer takes GTSAM's parameters one by one, as its C++ side declares them"
)]
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
        /// Adds GTSAM's between factor on two poses, one held at a value: a factor on the other.
        fn add_held_between_poses(
            problem: Pin<&mut Problem>,
            first: u64,
            second: u64,
            relative: &[f64],
            held: &[f64],
            first_held: bool,
        ) -> Result<()>;
        /// Adds GTSAM's between factor on two similarities, one held at a value.
        fn add_held_between_similarities(
            problem: Pin<&mut Problem>,
            first: u64,
            second: u64,
            relative: &[f64],
            held: &[f64],
            first_held: bool,
        ) -> Result<()>;
        /// GTSAM's residual of its between factor on two poses at given values.
        fn between_residual_poses(
            first: &[f64],
            second: &[f64],
            relative: &[f64],
            out: &mut [f64],
        ) -> Result<()>;
        /// GTSAM's residual of its between factor on two similarities at given values.
        fn between_residual_similarities(
            first: &[f64],
            second: &[f64],
            relative: &[f64],
            out: &mut [f64],
        ) -> Result<()>;
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
        /// The largest diagonal entry of the problem's normal equations at its values.
        fn largest_hessian_diagonal(problem: &Problem, evaluator: &Evaluator) -> Result<f64>;
        /// Starts Levenberg-Marquardt, evaluating the starting error: its first damping, its
        /// factor, whether Nielsen's schedule moves it (or the factor alone), its upper bound
        /// and its tolerances.
        fn new_optimizer(
            problem: &Problem,
            evaluator: &Evaluator,
            initial_damping: f64,
            damping_factor: f64,
            nielsen: bool,
            damping_limit: f64,
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
    let (initial_damping, factor, limit) = if options.scaled_damping {
        let largest =
            ffi::largest_hessian_diagonal(problem, evaluator).map_err(|e| gtsam_failed(&e))?;
        // With no gradient anywhere (every Jacobian zero), the damping cannot be scaled to it:
        // tau alone.
        let scale = if largest > 0.0 { largest } else { 1.0 };
        (
            options.initial_damping * scale,
            SCALED_DAMPING_FACTOR,
            SCALED_DAMPING_LIMIT,
        )
    } else {
        (
            options.initial_damping,
            ABSOLUTE_DAMPING_FACTOR,
            ABSOLUTE_DAMPING_LIMIT,
        )
    };
    let mut optimizer = ffi::new_optimizer(
        problem,
        evaluator,
        initial_damping,
        factor,
        options.scaled_damping,
        limit,
        options.relative_tolerance,
        options.absolute_tolerance,
    )
    .map_err(|e| gtsam_failed(&e))?;
    let initial = ffi::error(&optimizer);
    let mut current = initial;
    let mut termination = Termination::MaxIterations;
    let mut iterations = 0;
    // Iterations in a row each lowering the error by under `small_decrease` of it.
    let mut small = 0;
    while current > 0.0 && iterations < options.max_iterations {
        if options.stop.is_set() {
            termination = Termination::Stopped;
            break;
        }
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
        small = if decrease < options.small_decrease * previous {
            small + 1
        } else {
            0
        };
        if options.small_steps > 0 && small >= options.small_steps {
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
    if !observations.huber.is_empty()
        && (observations.huber.len() != count
            || !observations.huber.iter().all(|h| h.is_finite() && *h > 0.0))
    {
        return fail(
            ErrorKind::InvalidInput,
            "huber: expected one positive value per observation",
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

/// Which of `count` indices are held.
///
/// # Arguments
///
/// * `fixed` - The held indices, checked.
/// * `count` - How many there are.
///
/// # Returns
///
/// A flag per index.
fn held_flags(fixed: &[usize], count: usize) -> Vec<bool> {
    let mut out = vec![false; count];
    for &k in fixed {
        out[k] = true;
    }
    out
}

/// Optimises a pose graph, each edge's measured relative pose with unit information
/// (`graphs.optimize_pose_graph`).
///
/// ORB-SLAM's `OptimizeEssentialGraph` on GTSAM: edge `(i, j)` measures `T_i_j = T_world_i^-1
/// T_world_j` (`BetweenFactor`); the poses in `fixed` are held as constants, as g2o's fixed
/// vertices hold ORB-SLAM's: an edge with one held end is a factor on its other end, and the
/// edges with both ends held, whose error is a constant, one factor on a free pose with a zero
/// Jacobian (`Factor::HeldEdges`), so that every error GTSAM reads, its search over dampings'
/// included, is the whole graph's. With every pose held, nothing is solved: the poses come back
/// as given, after no iteration.
///
/// # Arguments
///
/// * `poses` - Each pose's start, camera-to-world: rigid, or a similarity with `similarity`.
/// * `edges` - Each edge's two poses `(i, j)`, `i != j`.
/// * `relatives` - Each edge's measured `T_i_j`, of the poses' kind.
/// * `fixed` - The poses held fixed, at least one; each comes back exactly as given.
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
    let held = held_flags(fixed, poses.len());
    let mut problem = ffi::new_problem();
    for (k, pose) in poses.iter().enumerate().filter(|(k, _)| !held[*k]) {
        if similarity {
            ffi::add_similarity(problem.pin_mut(), key(k), &to_similarity3(pose))
        } else {
            ffi::add_pose(problem.pin_mut(), key(k), &rows_of(pose))
        }
        .map_err(|e| gtsam_failed(&e))?;
    }
    let mut residuals = Vec::new();
    for (&[i, j], relative) in edges.iter().zip(relatives) {
        residuals.extend(add_edge(
            problem.pin_mut(),
            [i, j],
            relative,
            [held[i].then_some(&poses[i]), held[j].then_some(&poses[j])],
            similarity,
        )?);
    }
    let Some(free) = held.iter().position(|&h| !h) else {
        // Every pose held: nothing is solved, the error the held edges' (as their factor's).
        let error = 0.5 * residuals.iter().map(|r| r * r).sum::<f64>();
        return Ok(PoseGraph {
            poses: poses.to_vec(),
            summary: Summary {
                initial_error: error,
                final_error: error,
                iterations: 0,
                termination: Termination::Function,
                converged: true,
            },
        });
    };
    let mut factors = Vec::new();
    if !residuals.is_empty() {
        ffi::add_rust_factor(
            problem.pin_mut(),
            &[key(free)],
            &[if similarity { SIMILARITY } else { POSE }],
            residuals.len(),
            1.0, // unit information
            0.0, // the bridge's 0: no robust loss
            factors.len(),
        )
        .map_err(|e| gtsam_failed(&e))?;
        factors.push(Factor::HeldEdges { residuals });
    }
    let evaluator = Evaluator {
        lens: Lens {
            model: Model::Pinhole,
            intrinsics: Vec::new(),
            bf: 0.0,
        },
        factors,
    };
    let (optimizer, summary) = solve(&problem, &evaluator, options)?;
    let poses = poses
        .iter()
        .enumerate()
        .map(|(k, pose)| {
            if held[k] {
                Ok(*pose)
            } else {
                matrix_at(&optimizer, k, similarity)
            }
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(PoseGraph { poses, summary })
}

/// Adds a pose graph's edge to its problem: GTSAM's between factor on its two ends, or a factor
/// on its free end alone if the other is held; with both ends held, nothing (its residual is a
/// constant, which the caller gathers into one factor).
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `ends` - The edge's two poses `(i, j)`.
/// * `relative` - Its measured `T_i_j`.
/// * `held` - Each end's value if it is held, none if it is a variable.
/// * `similarity` - Whether the poses are similarities.
///
/// # Returns
///
/// The edge's residual with both ends held (GTSAM's, 6 or 7 long); empty otherwise.
///
/// # Errors
///
/// `NotConverged` if GTSAM fails.
fn add_edge(
    problem: Pin<&mut ffi::Problem>,
    ends: [usize; 2],
    relative: &Matrix4<f64>,
    held: [Option<&Matrix4<f64>>; 2],
    similarity: bool,
) -> Result<Vec<f64>> {
    // A pose or similarity as the bridge takes it: GTSAM's own matrix for a similarity.
    let crossing = |matrix: &Matrix4<f64>| {
        if similarity {
            to_similarity3(matrix)
        } else {
            rows_of(matrix)
        }
    };
    let (first, second, measured) = (key(ends[0]), key(ends[1]), crossing(relative));
    if let [Some(one), Some(two)] = held {
        let (one, two) = (crossing(one), crossing(two));
        let mut residual = vec![0.0; if similarity { SIMILARITY_DIM } else { POSE_DIM }];
        if similarity {
            ffi::between_residual_similarities(&one, &two, &measured, &mut residual)
        } else {
            ffi::between_residual_poses(&one, &two, &measured, &mut residual)
        }
        .map_err(|e| gtsam_failed(&e))?;
        return Ok(residual);
    }
    let added = match held {
        [Some(value), None] | [None, Some(value)] => {
            let (value, first_held) = (crossing(value), held[0].is_some());
            if similarity {
                ffi::add_held_between_similarities(
                    problem, first, second, &measured, &value, first_held,
                )
            } else {
                ffi::add_held_between_poses(problem, first, second, &measured, &value, first_held)
            }
        }
        _ if similarity => ffi::add_between_similarities(problem, first, second, &measured),
        _ => ffi::add_between_poses(problem, first, second, &measured),
    };
    added.map_err(|e| gtsam_failed(&e))?;
    Ok(Vec::new())
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

/// Which observations a bundle adjustment keeps (`graphs.kept_observations`).
///
/// With `near` 0, every one. Otherwise those whose point, at the start, is in front of its camera
/// and no nearer than `near` times the median of every observation's depth, the `floor(M/2)`-th
/// smallest of the M counting from 0; each depth `R(0,2) dx + R(1,2) dy + R(2,2) dz`, summed in
/// that order, `(dx, dy, dz)` the point less the camera's centre, as every language takes it.
///
/// # Arguments
///
/// * `poses` - Each camera's `T_world_cam`, checked.
/// * `points` - Each point, checked.
/// * `observations` - The observations, checked.
/// * `near` - The fraction of the median depth, non-negative.
///
/// # Returns
///
/// A flag per observation.
#[must_use]
pub fn kept_observations(
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    observations: &Observations<'_>,
    near: f64,
) -> Vec<bool> {
    let count = observations.cameras.len();
    if near == 0.0 || count == 0 {
        return vec![true; count];
    }
    let depth: Vec<f64> = observations
        .cameras
        .iter()
        .zip(observations.point_ids)
        .map(|(&c, &p)| {
            let pose = &poses[c];
            let offset = points[p] - pose.fixed_view::<3, 1>(0, 3);
            pose[(0, 2)] * offset.x + pose[(1, 2)] * offset.y + pose[(2, 2)] * offset.z
        })
        .collect();
    let mut sorted = depth.clone();
    let middle = count / 2;
    let (_, median, _) = sorted.select_nth_unstable_by(middle, f64::total_cmp);
    let limit = near * *median;
    depth.iter().map(|&z| z > 0.0 && z >= limit).collect()
}

/// Adjusts every pose and point against every observation, with no robust loss or Huber's
/// (`graphs.bundle_adjust`).
///
/// ORB-SLAM's `GlobalBundleAdjustemnt` as loop closing calls it (`bRobust` false), on GTSAM; with
/// the observations' `huber`, each whitened error through a Huber loss at its threshold, as
/// ORB-SLAM's local and welding BAs weigh theirs. The cameras in `fixed` are held as constants,
/// as g2o's fixed vertices hold ORB-SLAM's: each of their observations is a factor on its point
/// alone. With `near`, the observations near their camera's plane are left out
/// (`kept_observations`): only the points and free cameras the others reach are solved, the rest
/// coming back as given, and the summary's errors are the kept observations'; with none kept,
/// nothing is solved: 0 iterations, converged, errors of 0.
///
/// # Arguments
///
/// * `poses` - Each camera's starting `T_world_cam`.
/// * `points` - Each point's start, in the world, at least one.
/// * `observations` - The observations; every point must be observed.
/// * `lens` - The camera; its `bf` positive if any observation has a right column.
/// * `fixed` - The cameras held fixed; each comes back exactly as given.
/// * `near` - Leave out each observation whose point, at the start, is behind its camera or
///   nearer than this fraction of the median depth, non-negative; 0 leaves none out.
/// * `options` - How the solve runs.
///
/// # Returns
///
/// The poses, the points and how the solve went.
///
/// # Errors
///
/// `InvalidInput` for a pose not rigid, mismatched sizes, an index out of range, an
/// unobserved point, a non-positive information or Huber threshold, a right column without
/// `bf`, a negative `near`, a bad camera or bad options; `NotConverged` if GTSAM fails.
pub fn bundle_adjust(
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    observations: &Observations<'_>,
    lens: &Lens,
    fixed: &[usize],
    near: f64,
    options: &Options,
) -> Result<Adjustment> {
    check_bundle(poses, points, observations, lens, fixed, options)?;
    if !(near.is_finite() && near >= 0.0) {
        return fail_setting("near", "expected >= 0", near);
    }
    // Only the kept observations take part, and only the points and free cameras they reach are
    // solved. The held cameras are constants, not variables: each of their observations is a
    // factor on its point alone.
    let kept = kept_observations(poses, points, observations, near);
    let held = held_flags(fixed, poses.len());
    let (solved_camera, solved_point) = reached(observations, &kept, &held, points.len());
    if !kept.iter().any(|&k| k) {
        // Nothing left to solve: the input as given, no error counted.
        return Ok(Adjustment {
            poses: poses.to_vec(),
            points: points.to_vec(),
            summary: Summary {
                initial_error: 0.0,
                final_error: 0.0,
                iterations: 0,
                termination: Termination::Function,
                converged: true,
            },
        });
    }
    let mut problem = ffi::new_problem();
    let gtsam =
        |result: std::result::Result<(), cxx::Exception>| result.map_err(|e| gtsam_failed(&e));
    let first_point = poses.len();
    for (c, pose) in poses.iter().enumerate().filter(|(c, _)| solved_camera[*c]) {
        gtsam(ffi::add_pose(problem.pin_mut(), key(c), &rows_of(pose)))?;
    }
    for (p, point) in points.iter().enumerate().filter(|(p, _)| solved_point[*p]) {
        gtsam(ffi::add_point(
            problem.pin_mut(),
            key(first_point + p),
            point.as_slice(),
        ))?;
    }
    let factors = add_observations(problem.pin_mut(), poses, &held, observations, &kept)?;
    let evaluator = Evaluator {
        lens: lens.clone(),
        factors,
    };
    let (optimizer, summary) = solve(&problem, &evaluator, options)?;
    let poses = poses
        .iter()
        .enumerate()
        .map(|(c, pose)| {
            if solved_camera[c] {
                matrix_at(&optimizer, c, false)
            } else {
                Ok(*pose)
            }
        })
        .collect::<Result<Vec<_>>>()?;
    let points = points
        .iter()
        .enumerate()
        .map(|(p, point)| {
            if !solved_point[p] {
                return Ok(*point);
            }
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

/// The free cameras and the points a bundle adjustment's kept observations reach: those it
/// solves.
///
/// # Arguments
///
/// * `observations` - The observations, checked.
/// * `kept` - Which are kept.
/// * `held` - Which cameras are held.
/// * `point_count` - The points.
///
/// # Returns
///
/// A flag per camera, the held ones never, and a flag per point.
fn reached(
    observations: &Observations<'_>,
    kept: &[bool],
    held: &[bool],
    point_count: usize,
) -> (Vec<bool>, Vec<bool>) {
    let mut cameras = vec![false; held.len()];
    let mut points = vec![false; point_count];
    for m in (0..kept.len()).filter(|&m| kept[m]) {
        let c = observations.cameras[m];
        cameras[c] = !held[c];
        points[observations.point_ids[m]] = true;
    }
    (cameras, points)
}

/// Adds a bundle adjustment's factors to its problem: one per kept observation, on its camera
/// and point, or on its point alone through a held camera (g2o's fixed vertex, as ORB-SLAM holds
/// one), the points keyed from `poses.len()`.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `poses` - The cameras' `T_world_cam`, checked.
/// * `held` - Which cameras are held.
/// * `observations` - The observations, checked.
/// * `kept` - Which are kept.
///
/// # Returns
///
/// The Rust factors, by the index each was added with.
///
/// # Errors
///
/// `NotConverged` if GTSAM fails.
fn add_observations(
    mut problem: Pin<&mut ffi::Problem>,
    poses: &[Matrix4<f64>],
    held: &[bool],
    observations: &Observations<'_>,
    kept: &[bool],
) -> Result<Vec<Factor>> {
    let first_point = poses.len();
    let mut factors = Vec::with_capacity(kept.len());
    for m in (0..kept.len()).filter(|&m| kept[m]) {
        let pixel = observations.pixels[m];
        let measured = if observations.right[m].is_nan() {
            vec![pixel.x, pixel.y]
        } else {
            vec![pixel.x, pixel.y, observations.right[m]]
        };
        let dimension = measured.len();
        let c = observations.cameras[m];
        let pair = [key(c), key(first_point + observations.point_ids[m])];
        let (keys, kinds, factor) = if held[c] {
            let pose = poses[c];
            let factor = Factor::HeldProjection { measured, pose };
            (&pair[1..], &[POINT][..], factor)
        } else {
            (
                &pair[..],
                &[POSE, POINT][..],
                Factor::Projection { measured },
            )
        };
        ffi::add_rust_factor(
            problem.as_mut(),
            keys,
            kinds,
            dimension,
            observations.information[m],
            observations.huber.get(m).copied().unwrap_or(0.0), // the bridge's 0: no robust loss
            factors.len(),
        )
        .map_err(|e| gtsam_failed(&e))?;
        factors.push(factor);
    }
    Ok(factors)
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

    /// A point behind its camera costs GTSAM's constant, twice the focal length in every row,
    /// with no gradient; a match carried behind a camera adds nothing to the similarity.
    #[test]
    fn a_point_behind_its_camera_costs_a_constant_with_no_gradient() {
        let lens = pinhole();
        let fx = lens.intrinsics[lens.model.focal_indices()[0]];
        for measured in [&[1.0, 2.0][..], &[1.0, 2.0, 3.0][..]] {
            let behind = projection_residual(
                &Matrix4::identity(),
                &Vector3::new(0.0, 0.0, -1.0),
                measured,
                &lens,
            )
            .unwrap();
            assert!(!behind.in_front);
            assert_eq!(
                behind.residual,
                DVector::from_element(measured.len(), 2.0 * fx)
            );
            assert!(
                behind
                    .d_pose
                    .iter()
                    .chain(behind.d_point.iter())
                    .all(|v| *v == 0.0)
            );
        }
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

    /// A stereo scene seen through the radtan camera: five cameras along x looking down +z, 60
    /// points in front of them, every camera's view of every point (every second one with its
    /// right column), and a start with the cameras after the first and every point perturbed.
    struct Scene {
        /// Each camera's true `T_world_cam`.
        truth: Vec<Matrix4<f64>>,
        /// Each point's true position.
        points: Vec<Vector3<f64>>,
        /// Each observation's camera.
        cameras: Vec<usize>,
        /// Each observation's point.
        ids: Vec<usize>,
        /// Each observation's pixel.
        pixels: Vec<Vector2<f64>>,
        /// Each observation's right column, NaN for none.
        right: Vec<f64>,
        /// Each camera's starting pose.
        start: Vec<Matrix4<f64>>,
        /// Each point's start.
        start_points: Vec<Vector3<f64>>,
    }

    /// The stereo scene.
    fn stereo_scene() -> Scene {
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
        Scene {
            truth,
            points,
            cameras,
            ids,
            pixels,
            right,
            start,
            start_points,
        }
    }

    /// Adjusts the scene from its start, the cameras in `fixed` held, through the radtan camera
    /// with unit information.
    fn adjust_scene(scene: &Scene, fixed: &[usize], options: &Options) -> Adjustment {
        let information = vec![1.0; scene.cameras.len()];
        bundle_adjust(
            &scene.start,
            &scene.start_points,
            &Observations {
                cameras: &scene.cameras,
                point_ids: &scene.ids,
                pixels: &scene.pixels,
                right: &scene.right,
                information: &information,
                huber: &[],
            },
            &radtan(),
            fixed,
            0.0,
            options,
        )
        .unwrap()
    }

    /// Adjusts the scene from its start for 20 iterations, the cameras in `fixed` held, and
    /// checks that the held ones come back exactly as they started, the others and every point
    /// to the truth.
    fn adjust_and_check(scene: &Scene, fixed: &[usize]) -> Adjustment {
        let options = Options {
            max_iterations: 20,
            ..Options::default()
        };
        let result = adjust_scene(scene, fixed, &options);
        for (c, got) in result.poses.iter().enumerate() {
            if fixed.contains(&c) {
                assert_eq!(*got, scene.start[c]);
            } else {
                assert!((got - scene.truth[c]).abs().max() < 1e-7);
            }
        }
        for (got, expected) in result.points.iter().zip(&scene.points) {
            assert!((got - expected).abs().max() < 1e-6);
        }
        result
    }

    #[test]
    fn bundle_adjustment_recovers_a_stereo_scene() {
        adjust_and_check(&stereo_scene(), &[0]);
    }

    #[test]
    fn held_cameras_come_back_exactly_with_three_held_or_every_one() {
        let mut scene = stereo_scene();
        scene.start.clone_from(&scene.truth);
        scene.start[4] *= pose([0.02, -0.02, 0.01], [0.01, 0.0, -0.01]);
        adjust_and_check(&scene, &[0, 1, 2]);
        scene.start[4] = scene.truth[4];
        let every = adjust_and_check(&scene, &[0, 1, 2, 3, 4]);
        assert!(every.summary.iterations > 0);
    }

    #[test]
    fn held_poses_are_constants_and_their_edges_cost_a_constant() {
        for with_scale in [false, true] {
            let truth = ring(4, with_scale);
            // Both held; held first; held second; free.
            let edges = vec![[0, 1], [1, 2], [3, 1], [2, 3]];
            let mut relatives: Vec<Matrix4<f64>> = edges
                .iter()
                .map(|[i, j]| inverse(&truth[*i]) * truth[*j])
                .collect();
            relatives[0] *= pose([0.1, 0.0, 0.0], [0.0, 0.0, 0.05]); // costs a constant
            let mut start = truth.clone();
            start[2] *= pose([0.05, -0.03, 0.02], [0.01, 0.0, -0.02]);
            start[3] *= pose([-0.04, 0.02, 0.01], [0.0, 0.02, 0.01]);
            let options = Options {
                max_iterations: 30,
                ..Options::default()
            };
            let result =
                optimize_pose_graph(&start, &edges, &relatives, &[0, 1], with_scale, &options)
                    .unwrap();
            assert_eq!(result.poses[..2], start[..2]);
            for (got, expected) in result.poses.iter().zip(&truth).skip(2) {
                assert!((got - expected).abs().max() < 1e-8);
            }
            let held = optimize_pose_graph(
                &truth,
                &edges,
                &relatives,
                &[0, 1, 2, 3],
                with_scale,
                &Options::default(),
            )
            .unwrap();
            assert_eq!(held.poses, truth);
            assert_eq!(
                (held.summary.iterations, held.summary.termination),
                (0, Termination::Function)
            );
            assert!(held.summary.converged);
            let constant = held.summary.initial_error;
            assert!(constant > 1e-4);
            #[allow(clippy::float_cmp)] // nothing was solved: the very number
            let unchanged = held.summary.final_error == constant;
            assert!(unchanged);
            assert!((result.summary.final_error - constant).abs() <= 1e-9 * constant);
            assert!(result.summary.initial_error > result.summary.final_error);
        }
    }

    #[test]
    fn an_edge_between_held_poses_costs_its_error_inside_gtsam_too() {
        // A free pose in no edge: the one iteration lowers nothing, and the solve converges, as
        // when the held poses were variables (0.3.17.2's review panel, L1), not at the damping's
        // limit.
        let truth = vec![
            pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            pose([1.0, 0.2, 0.0], [0.0, 0.0, 2.0]),
            pose([2.0, -0.2, 0.0], [0.0, 0.0, 4.0]),
        ];
        let relatives =
            vec![inverse(&truth[0]) * truth[1] * pose([0.1, 0.0, 0.0], [0.0, 0.0, 0.05])];
        let options = Options {
            max_iterations: 10,
            ..Options::default()
        };
        let result =
            optimize_pose_graph(&truth, &[[0, 1]], &relatives, &[0, 1], false, &options).unwrap();
        assert_eq!(
            (result.summary.iterations, result.summary.termination),
            (1, Termination::Function)
        );
        assert!(result.summary.converged && result.summary.initial_error > 1e-3);
        #[allow(clippy::float_cmp)] // nothing could move: the very number
        let unchanged = result.summary.final_error == result.summary.initial_error;
        assert!(unchanged);
    }

    /// Two held cameras 0.3 m apart along x, looking down +z: a point alone is solved.
    fn held_pair() -> Vec<Matrix4<f64>> {
        vec![Matrix4::identity(), pose([0.3, 0.0, 0.0], [0.0, 0.0, 0.0])]
    }

    /// The point the held pair sees, 4 m ahead, and its start, three times as far: GTSAM's first
    /// trial steps fail there.
    const FAR_POINT: ([f64; 3], [f64; 3]) = ([0.2, -0.1, 4.0], [0.2, -0.1, 12.0]);

    /// Where the held pair's cameras see `FAR_POINT`'s truth.
    fn held_pair_pixels() -> Vec<Vector2<f64>> {
        let lens = pinhole();
        held_pair()
            .iter()
            .map(|camera_pose| {
                let rotation: Matrix3<f64> = camera_pose.fixed_view::<3, 3>(0, 0).into();
                let local = rotation.transpose()
                    * (Vector3::from(FAR_POINT.0) - camera_pose.fixed_view::<3, 1>(0, 3));
                camera::project(lens.model, &lens.intrinsics, &local).unwrap()
            })
            .collect()
    }

    /// The held pair's residuals at a point (unit information), stacked, and their Jacobian.
    fn pair_residuals(point: &Vector3<f64>) -> (DVector<f64>, DMatrix<f64>) {
        let mut residual = DVector::zeros(4);
        let mut jacobian = DMatrix::zeros(4, 3);
        for (c, (camera_pose, seen)) in held_pair().iter().zip(held_pair_pixels()).enumerate() {
            let at =
                projection_residual(camera_pose, point, &[seen.x, seen.y], &pinhole()).unwrap();
            residual.rows_mut(2 * c, 2).copy_from(&at.residual);
            jacobian.rows_mut(2 * c, 2).copy_from(&at.d_point);
        }
        (residual, jacobian)
    }

    /// One of GTSAM's Levenberg-Marquardt iterations on the held pair's point, emulated
    /// (`tests/foundations/test_graphs.py`'s `_gtsam_iteration`): trials of
    /// `(H + lambda I) delta = -g` until the gain ratio passes GTSAM's `minModelFidelity`, 1e-3; a
    /// failed one multiplies the damping by the factor, which Nielsen's schedule doubles; an
    /// accepted one divides it by the factor, or with Nielsen's schedule scales it by
    /// `max(1/3, 1 - (2 rho - 1)^3)` and resets the factor to 2. Takes and returns the point, the
    /// damping and the factor.
    fn gtsam_iteration(
        (point, mut damping, mut factor): (Vector3<f64>, f64, f64),
        nielsen: bool,
        limit: f64,
    ) -> (Vector3<f64>, f64, f64) {
        let (residual, jacobian) = pair_residuals(&point);
        let error = 0.5 * residual.norm_squared();
        let hessian = jacobian.transpose() * &jacobian;
        let gradient = jacobian.transpose() * &residual;
        while damping < limit {
            let damped = &hessian + DMatrix::identity(3, 3) * damping;
            let step = -damped.lu().solve(&gradient).unwrap();
            let predicted = error - 0.5 * (&residual + &jacobian * &step).norm_squared();
            let moved = point + Vector3::new(step[0], step[1], step[2]);
            let ratio = (error - 0.5 * pair_residuals(&moved).0.norm_squared()) / predicted;
            if ratio > 1e-3 {
                if nielsen {
                    let scale = (1.0 - (2.0 * ratio - 1.0).powi(3)).max(1.0 / 3.0);
                    return (moved, damping * scale, 2.0);
                }
                return (moved, damping / factor, factor);
            }
            damping *= factor;
            if nielsen {
                factor *= 2.0;
            }
        }
        (point, damping, factor)
    }

    #[test]
    fn the_damping_starts_and_moves_as_its_schedule_says() {
        const TAU: f64 = 1e-5;
        let start = Vector3::from(FAR_POINT.1);
        let pixels = held_pair_pixels();
        let (_, jacobian) = pair_residuals(&start);
        let largest = (jacobian.transpose() * &jacobian).diagonal().max();
        for scaled in [true, false] {
            let mut expected = if scaled {
                (start, TAU * largest, SCALED_DAMPING_FACTOR)
            } else {
                (start, TAU, ABSOLUTE_DAMPING_FACTOR)
            };
            let limit = if scaled {
                SCALED_DAMPING_LIMIT
            } else {
                ABSOLUTE_DAMPING_LIMIT
            };
            for iterations in 1..=3 {
                expected = gtsam_iteration(expected, scaled, limit);
                let result = bundle_adjust(
                    &held_pair(),
                    &[start],
                    &Observations {
                        cameras: &[0, 1],
                        point_ids: &[0, 0],
                        pixels: &pixels,
                        right: &[f64::NAN, f64::NAN],
                        information: &[1.0, 1.0],
                        huber: &[],
                    },
                    &pinhole(),
                    &[0, 1],
                    0.0,
                    &Options {
                        max_iterations: iterations,
                        initial_damping: TAU,
                        scaled_damping: scaled,
                        ..Options::default()
                    },
                )
                .unwrap();
                assert_eq!(result.summary.iterations, iterations);
                assert!((result.points[0] - expected.0).abs().max() < 1e-9);
            }
        }
    }

    #[test]
    fn small_steps_in_a_row_end_a_solve_and_a_large_one_resets_their_count() {
        const MOST: usize = 30;
        let mut scene = stereo_scene();
        #[allow(clippy::cast_precision_loss)] // a few hundred observations
        for (m, pixel) in scene.pixels.iter_mut().enumerate() {
            let k = m as f64;
            *pixel += Vector2::new(0.5 * (1.7 * k).sin(), 0.5 * (2.3 * k).cos());
        }
        let solve = |options: Options| {
            adjust_scene(
                &scene,
                &[0],
                &Options {
                    initial_damping: 3000.0,
                    scaled_damping: true,
                    ..options
                },
            )
        };
        let capped = |iterations| {
            solve(Options {
                max_iterations: iterations,
                ..Options::default()
            })
        };
        let mut errors = vec![capped(1).summary.initial_error];
        errors.extend((1..=MOST).map(|k| capped(k).summary.final_error));
        let decrease: Vec<f64> = errors.windows(2).map(|e| (e[0] - e[1]) / e[0]).collect();
        let threshold = (decrease[1] * decrease[2]).sqrt();
        let small: Vec<bool> = decrease.iter().map(|&d| d < threshold).collect();
        assert_eq!(small[..3], [true, true, false]); // two small, then one that resets them
        let ends = (3..small.len())
            .find(|&k| small[k] && small[k - 1] && small[k - 2])
            .unwrap()
            + 1;
        assert!(ends > 6);
        let stopped = solve(Options {
            max_iterations: MOST,
            small_steps: 3,
            small_decrease: threshold,
            ..Options::default()
        });
        assert_eq!(stopped.summary.iterations, ends);
        assert_eq!(stopped.summary.termination, Termination::Function);
        let three = solve(Options {
            max_iterations: MOST,
            small_steps: 3,
            small_decrease: 0.9,
            ..Options::default()
        });
        assert_eq!(three.summary.iterations, 3);
        assert!(three.summary.converged);
    }

    #[test]
    fn the_near_filter_leaves_out_exactly_the_near_and_behind_observations() {
        // The stereo scene, camera 0 held, a sixth camera 2 m ahead, point 60 1 mm in front of
        // it (seen too by cameras 0 and 4), point 61 behind cameras 0, 1 and 5 (seen by them).
        let mut scene = stereo_scene();
        let mut poses = scene.start.clone();
        poses.push(pose([0.8, 0.0, 2.0], [0.0, 0.0, 0.0]));
        let ahead = |c: usize, local: Vector3<f64>| {
            poses[c].fixed_view::<3, 3>(0, 0) * local + poses[c].fixed_view::<3, 1>(0, 3)
        };
        let count = scene.start_points.len();
        let mut points = scene.start_points.clone();
        points.push(ahead(5, Vector3::new(0.0, 0.0, 1e-3)));
        points.push(ahead(0, Vector3::new(0.1, 0.0, -1.0)));
        let extra = [
            (5, count, false),
            (0, count, true),
            (4, count, true),
            (0, count + 1, false),
            (1, count + 1, false),
            (5, count + 1, false),
        ];
        let mut expected = vec![true; scene.cameras.len()];
        for &(c, p, kept) in &extra {
            scene.cameras.push(c);
            scene.ids.push(p);
            scene.pixels.push(Vector2::new(300.0, 300.0));
            scene.right.push(f64::NAN);
            expected.push(kept);
        }
        let information = vec![1.0; scene.cameras.len()];
        let observations = Observations {
            cameras: &scene.cameras,
            point_ids: &scene.ids,
            pixels: &scene.pixels,
            right: &scene.right,
            information: &information,
            huber: &[],
        };
        let costs: Vec<f64> = (0..scene.cameras.len())
            .map(|m| {
                let pixel = scene.pixels[m];
                let measured = if scene.right[m].is_nan() {
                    vec![pixel.x, pixel.y]
                } else {
                    vec![pixel.x, pixel.y, scene.right[m]]
                };
                let at = projection_residual(
                    &poses[scene.cameras[m]],
                    &points[scene.ids[m]],
                    &measured,
                    &radtan(),
                )
                .unwrap();
                0.5 * at.residual.norm_squared()
            })
            .collect();
        let every: f64 = costs.iter().sum();
        let kept: f64 = costs
            .iter()
            .zip(&expected)
            .filter(|(_, k)| **k)
            .map(|(c, _)| c)
            .sum();
        assert_eq!(
            kept_observations(&poses, &points, &observations, 1e-3),
            expected
        );
        let adjust = |near| {
            bundle_adjust(
                &poses,
                &points,
                &observations,
                &radtan(),
                &[0],
                near,
                &Options {
                    max_iterations: 5,
                    ..Options::default()
                },
            )
        };
        let filtered = adjust(1e-3).unwrap();
        assert!((filtered.summary.initial_error - kept).abs() <= 1e-12 * kept);
        assert_eq!(filtered.points[count + 1], points[count + 1]);
        assert_eq!(filtered.poses[0], poses[0]);
        assert_eq!(filtered.poses[5], poses[5]);
        assert_ne!(filtered.points[count], points[count]); // seen by cameras 0 and 4 still
        let all = adjust(0.0).unwrap();
        assert!((all.summary.initial_error - every).abs() <= 1e-12 * every);
        for bad in [-1e-3, f64::INFINITY, f64::NAN] {
            assert_eq!(adjust(bad).unwrap_err().kind, ErrorKind::InvalidInput);
        }
    }

    #[test]
    fn a_bundle_whose_observations_are_all_left_out_comes_back_as_given() {
        let poses = vec![Matrix4::identity(), pose([0.2, 0.0, 0.0], [0.0, 0.0, 0.0])];
        let points = vec![Vector3::new(0.1, 0.2, -1.0), Vector3::new(0.0, -0.1, -2.0)];
        let result = bundle_adjust(
            &poses,
            &points,
            &Observations {
                cameras: &[0, 1, 1],
                point_ids: &[0, 0, 1],
                pixels: &[Vector2::new(300.0, 300.0); 3],
                right: &[f64::NAN; 3],
                information: &[1.0; 3],
                huber: &[],
            },
            &pinhole(),
            &[0],
            1e-3,
            &Options::default(),
        )
        .unwrap();
        assert_eq!(result.poses, poses);
        assert_eq!(result.points, points);
        assert_eq!(result.summary.iterations, 0);
        assert!(result.summary.converged);
        assert_eq!(result.summary.termination, Termination::Function);
        assert_eq!(
            (result.summary.initial_error, result.summary.final_error),
            (0.0, 0.0)
        );
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
        let stopped = solve_with(
            &start,
            Options {
                max_iterations: 50,
                stop: Stop::on(std::sync::Arc::new(std::sync::atomic::AtomicBool::new(
                    true,
                ))),
                ..Options::default()
            },
        );
        assert_eq!(
            (stopped.summary.iterations, stopped.summary.termination),
            (0, Termination::Stopped)
        );
        assert!(!stopped.summary.converged);
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
        for small_decrease in [0.0, f64::INFINITY] {
            assert!(
                Options {
                    small_decrease,
                    ..Options::default()
                }
                .check()
                .is_err()
            );
        }
        assert_eq!(Termination::DampingLimit.name(), "damping_limit");
        // A row of pixels2 is finite, or wholly NaN (unseen): anything else is refused (H1, L1).
        let lens = pinhole();
        let points = [
            Vector3::new(0.0, 0.0, 3.0),
            Vector3::new(0.1, 0.0, 3.0),
            Vector3::new(0.2, 0.1, 3.0),
        ];
        let seen: Vec<Vector2<f64>> = points
            .iter()
            .map(|p| camera::project(lens.model, &lens.intrinsics, p).unwrap())
            .collect();
        let information = [[1.0, 1.0]; 3];
        let refine = |pixels2: &[Vector2<f64>]| {
            refine_similarity(
                &Matrix4::identity(),
                &Matches {
                    points1: &points,
                    points2: &points,
                    pixels1: &seen,
                    pixels2,
                    information: &information,
                },
                lens.model,
                &lens.intrinsics,
                true,
                None,
                &options,
            )
        };
        for bad in [
            Vector2::new(f64::NAN, 210.0),
            Vector2::new(f64::INFINITY, 210.0),
            Vector2::new(300.0, f64::NEG_INFINITY),
        ] {
            let mut pixels2 = seen.clone();
            pixels2[1] = bad;
            assert!(matches!(refine(&pixels2), Err(e) if e.kind == ErrorKind::InvalidInput));
        }
        let mut unseen = seen.clone();
        unseen[1] = Vector2::new(f64::NAN, f64::NAN);
        assert!(refine(&unseen).is_ok());
    }
}
