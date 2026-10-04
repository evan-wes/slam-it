//! Bundle adjustment: the Rust core.
//!
//! Mirrors `slamlab.foundations.ba`, whose docstring states the model:
//! - robust reprojection errors over poses (right-perturbed `T_cam_world`), intrinsics groups
//!   and points (plain, or anchored inverse depths), plus inverse-depth priors through
//!   per-camera affines and pose priors, each kind of block with its own loss;
//! - points eliminated by the Schur complement, with the reduced camera system factored by a
//!   [`ReducedSystemSolver`];
//! - the gauge removed by fixing a pose and, for the similarity gauge, confining one pose's
//!   step to the Householder complement of its baseline direction;
//! - degenerate observations dropped and counted;
//! - free intrinsics reported with their marginal standard deviations, and focal lengths the
//!   data does not pin counted.
//!
//! The solve is [`nls::solve`] over the same loop as every other problem.

mod bundle;
mod gauge;
mod layout;
mod schur;

use nalgebra::{DMatrix, DVector, Matrix4, Vector2, Vector3};

use crate::camera::{self, Model};
use crate::error::{ErrorKind, Result, fail};
use crate::nls;
use crate::se3;

/// A pose's tangent dimension.
pub const POSE_DIM: usize = 6;

/// How the gauge freedom is removed, with the integer codes it crosses the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gauge {
    /// The caller's fixed variables define the gauge.
    None = 0,
    /// SE(3): fix the first free observed pose if no pose is fixed.
    FirstPose = 1,
    /// Sim(3): as `FirstPose`, then confine one pose's step to keep its baseline.
    Similarity = 2,
    /// Exactly what the fixed variables and priors leave free, whatever part of the
    /// similarity it is (reported as `None`, `FirstPose`, `Scale`, `Similarity` or
    /// `Partial`).
    Auto = 3,
    /// Scale only: confine one pose's step to keep its baseline to the pose the rigid motion
    /// is pinned to (a fixed pose or a full pose prior).
    Scale = 4,
    /// Reported only: `Auto` found a part of the similarity no other name describes.
    Partial = 5,
}

impl Gauge {
    /// Every gauge, in code order.
    pub const ALL: [Self; 6] = [
        Self::None,
        Self::FirstPose,
        Self::Similarity,
        Self::Auto,
        Self::Scale,
        Self::Partial,
    ];

    /// The gauge's name, as `slamlab.foundations.ba.BAOptions` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::FirstPose => "first_pose",
            Self::Similarity => "similarity",
            Self::Auto => "auto",
            Self::Scale => "scale",
            Self::Partial => "partial",
        }
    }
}

/// Solves the reduced camera system `S x = b`, S symmetric positive definite.
pub trait ReducedSystemSolver {
    /// Solves `matrix x = rhs`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - Symmetric positive definite.
    /// * `rhs` - The right-hand side.
    ///
    /// # Returns
    ///
    /// The solution, or `None` if `matrix` is not positive definite.
    fn solve(&self, matrix: DMatrix<f64>, rhs: &DVector<f64>) -> Option<DVector<f64>>;
}

/// The dense reduced solver: Cholesky on the full matrix.
#[derive(Debug, Clone, Copy, Default)]
pub struct DenseCholesky;

impl ReducedSystemSolver for DenseCholesky {
    fn solve(&self, matrix: DMatrix<f64>, rhs: &DVector<f64>) -> Option<DVector<f64>> {
        Some(matrix.cholesky()?.solve(rhs))
    }
}

/// Intrinsics groups, one row of P parameters each, stored row-major.
#[derive(Debug, Clone, PartialEq)]
pub struct Intrinsics {
    /// The values, group after group.
    pub values: Vec<f64>,
    /// The parameters per group, P.
    pub params: usize,
}

impl Intrinsics {
    /// The number of groups, K.
    ///
    /// # Returns
    ///
    /// K.
    #[must_use]
    pub fn groups(&self) -> usize {
        self.values.len().checked_div(self.params).unwrap_or(0)
    }

    /// One group's parameters.
    ///
    /// # Arguments
    ///
    /// * `g` - The group, below [`Intrinsics::groups`].
    ///
    /// # Returns
    ///
    /// Its P parameters.
    #[must_use]
    pub fn group(&self, g: usize) -> &[f64] {
        &self.values[g * self.params..(g + 1) * self.params]
    }
}

/// A bundle-adjustment problem, as a struct of arrays.
#[derive(Debug, Clone, PartialEq)]
pub struct Problem {
    /// The camera model shared by every intrinsics group.
    pub model: Model,
    /// The intrinsics groups (K x P).
    pub intrinsics: Intrinsics,
    /// Camera-to-world poses `T_world_cam` (C).
    pub poses: Vec<Matrix4<f64>>,
    /// World points (N).
    pub points: Vec<Vector3<f64>>,
    /// Observed pixels (M).
    pub observations: Vec<Vector2<f64>>,
    /// Each observation's camera (M).
    pub cam_idx: Vec<i64>,
    /// Each observation's point (M).
    pub pt_idx: Vec<i64>,
    /// Each camera's intrinsics group (C).
    pub intrinsics_idx: Vec<i64>,
    /// Each observation's weight, an information in px⁻² (M); 0 excludes it.
    pub weights: Vec<f64>,
    /// Poses held fixed (C).
    pub fixed_poses: Vec<bool>,
    /// Intrinsics held fixed, per group and parameter (K x P, row-major).
    pub fixed_intrinsics: Vec<bool>,
    /// Points held fixed (N).
    pub fixed_points: Vec<bool>,
    /// Each observation's prior inverse depth (M); NaN for none.
    pub depth_prior: Vec<f64>,
    /// Each depth prior's weight, an information in inverse-depth units (M).
    pub depth_prior_weights: Vec<f64>,
    /// Each camera's prior-to-model map `(scale, shift)` (C).
    pub depth_affine: Vec<Vector2<f64>>,
    /// Affines held fixed (C); a metric prior's stays at `(1, 0)`.
    pub fixed_depth_affine: Vec<bool>,
    /// Each camera's prior camera-to-world pose (C), used where its weights are positive.
    pub pose_prior: Vec<Matrix4<f64>>,
    /// Each pose prior's `(translation, rotation)` information (C).
    pub pose_prior_weights: Vec<Vector2<f64>>,
    /// Each point's anchor observation (N), or -1: an anchored point is one inverse depth
    /// along its anchor pixel's ray, in the anchor's camera.
    pub point_anchor: Vec<i64>,
}

impl Problem {
    /// A problem with every optional array at its default, as `BAProblem.create` fills them.
    ///
    /// The defaults: every camera in intrinsics group 0, every observation weighted 1, no
    /// pose or point fixed, every intrinsic fixed, no depth priors (affines fixed at
    /// `(1, 0)`), no pose priors, and no anchored points.
    ///
    /// # Arguments
    ///
    /// * `model` - The camera model.
    /// * `intrinsics` - The intrinsics groups.
    /// * `poses` - Camera-to-world poses `T_world_cam`.
    /// * `points` - World points.
    /// * `observations` - Observed pixels.
    /// * `cam_idx` - Each observation's camera.
    /// * `pt_idx` - Each observation's point.
    ///
    /// # Returns
    ///
    /// The problem (unchecked: [`solve`] checks it).
    #[must_use]
    pub fn with_defaults(
        model: Model,
        intrinsics: Intrinsics,
        poses: Vec<Matrix4<f64>>,
        points: Vec<Vector3<f64>>,
        observations: Vec<Vector2<f64>>,
        cam_idx: Vec<i64>,
        pt_idx: Vec<i64>,
    ) -> Self {
        let (n_cams, n_points, n_obs) = (poses.len(), points.len(), observations.len());
        Self {
            model,
            fixed_intrinsics: vec![true; intrinsics.values.len()],
            intrinsics,
            pose_prior: poses.clone(),
            poses,
            points,
            observations,
            cam_idx,
            pt_idx,
            intrinsics_idx: vec![0; n_cams],
            weights: vec![1.0; n_obs],
            fixed_poses: vec![false; n_cams],
            fixed_points: vec![false; n_points],
            depth_prior: vec![f64::NAN; n_obs],
            depth_prior_weights: vec![1.0; n_obs],
            depth_affine: vec![Vector2::new(1.0, 0.0); n_cams],
            fixed_depth_affine: vec![true; n_cams],
            pose_prior_weights: vec![Vector2::zeros(); n_cams],
            point_anchor: vec![-1; n_points],
        }
    }
}

/// Settings for [`solve`].
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    /// The NLS method, tolerances and limits; its `loss` is the reprojection loss and its
    /// `block_loss` must be empty (bundle adjustment assigns it).
    pub solver: nls::Options,
    /// The depth priors' robust loss.
    pub depth_loss: nls::Loss,
    /// How the gauge freedom is removed.
    pub gauge: Gauge,
    /// A free point whose rays span less than this angle (degrees) is dropped (default 0:
    /// only points seen fewer than twice are).
    pub min_parallax_deg: f64,
    /// A free focal length whose deviation exceeds this fraction of it is unobservable.
    pub focal_std_ratio: f64,
    /// Scale the intrinsics' deviations by the a-posteriori variance factor (see
    /// [`Summary::variance_factor`]); off, the weights are taken as exact informations.
    pub scale_deviations: bool,
}

impl Default for Options {
    /// The defaults of `slamlab.foundations.ba.BAOptions`.
    fn default() -> Self {
        Self {
            solver: nls::Options::default(),
            depth_loss: nls::Loss::default(),
            gauge: Gauge::Auto,
            min_parallax_deg: 0.0,
            focal_std_ratio: 0.01,
            scale_deviations: true,
        }
    }
}

/// How a bundle adjustment went.
#[derive(Debug, Clone, PartialEq)]
pub struct Summary {
    /// The NLS solve's summary.
    pub solve: nls::Summary,
    /// The gauge applied (`Auto` resolved).
    pub gauge: Gauge,
    /// The pose fixed to remove the rigid gauge.
    pub gauge_pose: Option<usize>,
    /// The first pose whose step was confined to remove the rest of the gauge.
    pub confined_pose: Option<usize>,
    /// Gauge directions left in the problem: under `Auto`, `Similarity` and `Scale` 0 unless
    /// the geometry is degenerate; under `None` and `FirstPose` whatever they leave.
    pub free_gauge: usize,
    /// Free points dropped as degenerate.
    pub dropped_points: usize,
    /// Observations left out (of dropped points, or behind their camera).
    pub dropped_observations: usize,
    /// Free poses held fixed because nothing observes them.
    pub unobserved_poses: usize,
    /// Free intrinsics groups held fixed because nothing observes them.
    pub unobserved_intrinsics: usize,
    /// Free depth affines held fixed because no depth prior observes them.
    pub unobserved_depth_affines: usize,
    /// Free intrinsics groups whose focal length the data does not pin.
    pub unobservable_focal: usize,
    /// The a-posteriori variance factor `Σ(w s) / (m - n)` at the solution (NaN when
    /// m <= n).
    pub variance_factor: f64,
}

/// The refined variables and the summary.
#[derive(Debug, Clone, PartialEq)]
pub struct Solution {
    /// Refined camera-to-world poses.
    pub poses: Vec<Matrix4<f64>>,
    /// Refined intrinsics groups.
    pub intrinsics: Intrinsics,
    /// Refined points (dropped points unchanged; NaN for an anchored point with no position).
    pub points: Vec<Vector3<f64>>,
    /// Anchored points' refined inverse depths (N); NaN for the others.
    pub inverse_depth: Vec<f64>,
    /// Refined depth affines (C).
    pub depth_affine: Vec<Vector2<f64>>,
    /// Free intrinsics' standard deviations (K x P); NaN where fixed, inf if singular.
    pub intrinsics_std: Intrinsics,
    /// How the solve went.
    pub summary: Summary,
}

/// Converts a checked index to `usize`.
///
/// # Arguments
///
/// * `index` - An index already checked to be non-negative.
///
/// # Returns
///
/// The same value.
fn at(index: i64) -> usize {
    usize::try_from(index).unwrap_or(usize::MAX)
}

/// Checks that every entry of an index array lies in `[0, bound)`.
///
/// # Arguments
///
/// * `indices` - The indices.
/// * `bound` - The exclusive upper bound.
/// * `name` - The array's name, for the message.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming the array.
fn check_indices(indices: &[i64], bound: usize, name: &str) -> Result<()> {
    if indices
        .iter()
        .any(|&i| usize::try_from(i).map_or(true, |i| i >= bound))
    {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: indices must lie in [0, {bound})"),
        );
    }
    Ok(())
}

/// Checks the depth and pose priors' sizes and values.
///
/// # Arguments
///
/// * `problem` - The problem.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming the offending array.
fn check_priors(problem: &Problem) -> Result<()> {
    let n_obs = problem.observations.len();
    let n_cams = problem.poses.len();
    if problem.depth_prior.len() != n_obs
        || problem
            .depth_prior
            .iter()
            .any(|&d| d <= 0.0 || d.is_infinite())
    {
        return fail(
            ErrorKind::InvalidInput,
            "depth_prior: expected M positive values (NaN for none)",
        );
    }
    if problem.depth_prior_weights.len() != n_obs
        || problem
            .depth_prior_weights
            .iter()
            .any(|&w| !w.is_finite() || w < 0.0)
    {
        return fail(
            ErrorKind::InvalidInput,
            "depth_prior_weights: expected M non-negative values",
        );
    }
    if problem.depth_affine.len() != n_cams
        || problem
            .depth_affine
            .iter()
            .any(|a| !a.iter().all(|v| v.is_finite()))
        || problem.fixed_depth_affine.len() != n_cams
    {
        return fail(
            ErrorKind::InvalidInput,
            "depth_affine: expected C finite (scale, shift) rows",
        );
    }
    if problem.pose_prior.len() != n_cams
        || problem.pose_prior_weights.len() != n_cams
        || problem
            .pose_prior_weights
            .iter()
            .any(|w| w.iter().any(|&v| !v.is_finite() || v < 0.0))
    {
        return fail(
            ErrorKind::InvalidInput,
            "pose_prior: expected C poses and C weight pairs",
        );
    }
    problem.pose_prior.iter().try_for_each(se3::check_transform)
}

/// Checks the point anchors: -1, or one of the point's own observations.
///
/// # Arguments
///
/// * `problem` - The problem.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` for a bad anchor.
fn check_anchors(problem: &Problem) -> Result<()> {
    if problem.point_anchor.len() != problem.points.len() {
        return fail(ErrorKind::InvalidInput, "point_anchor: expected N entries");
    }
    for (p, &a) in problem.point_anchor.iter().enumerate() {
        let observation = usize::try_from(a).ok();
        let foreign = observation
            .is_some_and(|m| m >= problem.observations.len() || at(problem.pt_idx[m]) != p);
        if a < -1 || foreign {
            return fail(
                ErrorKind::InvalidInput,
                "point_anchor: a point's anchor must be one of its observations",
            );
        }
        if a >= 0 && problem.fixed_points[p] {
            return fail(
                ErrorKind::InvalidInput,
                "point_anchor: a fixed point cannot be anchored (it would move with its host)",
            );
        }
    }
    Ok(())
}

/// Checks a problem's sizes, indices and values.
///
/// # Arguments
///
/// * `problem` - The problem.
///
/// # Returns
///
/// Nothing on success.
///
/// # Errors
///
/// `InvalidInput` naming the offending array.
pub fn check_problem(problem: &Problem) -> Result<()> {
    let n_cams = problem.poses.len();
    let n_obs = problem.observations.len();
    let n_groups = problem.intrinsics.groups();
    if n_groups == 0 {
        return fail(
            ErrorKind::InvalidInput,
            "intrinsics: expected at least one group",
        );
    }
    for g in 0..n_groups {
        camera::check_intrinsics(problem.model, problem.intrinsics.group(g))?;
    }
    problem.poses.iter().try_for_each(se3::check_transform)?;
    let finite = |v: &[f64]| v.iter().all(|x| x.is_finite());
    if !problem.points.iter().all(|p| finite(p.as_slice()))
        || !problem.observations.iter().all(|o| finite(o.as_slice()))
    {
        return fail(
            ErrorKind::InvalidInput,
            "points, observations: expected finite values",
        );
    }
    if problem.weights.len() != n_obs || problem.weights.iter().any(|&w| !w.is_finite() || w < 0.0)
    {
        return fail(
            ErrorKind::InvalidInput,
            "weights: expected M non-negative values",
        );
    }
    if problem.fixed_poses.len() != n_cams
        || problem.fixed_intrinsics.len() != problem.intrinsics.values.len()
        || problem.fixed_points.len() != problem.points.len()
    {
        return fail(ErrorKind::InvalidInput, "fixed masks: wrong length");
    }
    if problem.cam_idx.len() != n_obs
        || problem.pt_idx.len() != n_obs
        || problem.intrinsics_idx.len() != n_cams
    {
        return fail(
            ErrorKind::InvalidInput,
            "cam_idx, pt_idx, intrinsics_idx: wrong length",
        );
    }
    check_indices(&problem.cam_idx, n_cams, "cam_idx")?;
    check_indices(&problem.pt_idx, problem.points.len(), "pt_idx")?;
    check_indices(&problem.intrinsics_idx, n_groups, "intrinsics_idx")?;
    // The priors and anchors index the arrays checked above, so they come last.
    check_priors(problem)?;
    check_anchors(problem)
}

/// The point a solve is centred on.
///
/// The mean of the centres of the cameras with a positive-weight observation (all of them if
/// none is), summed in index order. The solve shifts the world by its negative and back.
///
/// # Arguments
///
/// * `problem` - The problem (checked).
///
/// # Returns
///
/// The origin.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "a camera count is far below 2^52"
)]
pub fn centring_origin(problem: &Problem) -> Vector3<f64> {
    // The cameras with a positive-weight observation: an unobserved placeholder (an
    // unregistered camera left at the identity) must not pull the origin off the scene.
    let mut seen = vec![false; problem.poses.len()];
    for (m, &w) in problem.weights.iter().enumerate() {
        if w > 0.0 {
            seen[at(problem.cam_idx[m])] = true;
        }
    }
    let any_seen = seen.contains(&true);
    let mut sum = Vector3::zeros();
    let mut count = 0;
    for (pose, &observed) in problem.poses.iter().zip(&seen) {
        if observed || !any_seen {
            sum += se3::translation(pose); // in index order, as Python sums
            count += 1;
        }
    }
    if count > 0 {
        sum / f64::from(count)
    } else {
        sum
    }
}

/// The problem shifted by `-origin`: camera centres, points and prior centres.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `origin` - The shift.
///
/// # Returns
///
/// The shifted problem.
fn shifted(problem: &Problem, origin: &Vector3<f64>) -> Problem {
    let shift = |pose: &Matrix4<f64>| {
        let mut out = *pose;
        out.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(se3::translation(pose) - origin));
        out
    };
    Problem {
        poses: problem.poses.iter().map(shift).collect(),
        pose_prior: problem.pose_prior.iter().map(shift).collect(),
        points: problem.points.iter().map(|p| p - origin).collect(),
        ..problem.clone()
    }
}

/// Refines a problem's poses, intrinsics and points.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `options` - Solver settings, gauge and degeneracy threshold.
///
/// # Returns
///
/// The solution.
///
/// # Errors
///
/// `InvalidInput` for an invalid problem or options; `NotConverged` if
/// `options.solver.strict` and the solve stopped without converging.
pub fn solve(problem: &Problem, options: &Options) -> Result<Solution> {
    check_problem(problem)?;
    if !options.min_parallax_deg.is_finite()
        || options.min_parallax_deg < 0.0
        || !options.focal_std_ratio.is_finite()
        || options.focal_std_ratio < 0.0
    {
        return fail(
            ErrorKind::InvalidInput,
            "min_parallax_deg, focal_std_ratio: must be non-negative and finite",
        );
    }
    if options.gauge == Gauge::Partial {
        return fail(
            ErrorKind::InvalidInput,
            "gauge: 'partial' is reported, not requested",
        );
    }
    if !options.solver.block_loss.losses.is_empty() {
        return fail(
            ErrorKind::InvalidInput,
            "solver.block_loss: set by bundle adjustment, not the caller",
        );
    }
    // Solve in a world centred on the cameras: exact (every residual is unchanged by a
    // shift), and it keeps the steps well conditioned far from the origin, where a step's
    // rotation about the world origin would swing a camera by about |θ|² |C| / 2.
    let origin = centring_origin(problem);
    let centred = shifted(problem, &origin);
    let layout = layout::make_layout(&centred, options)?;
    let mut solution = bundle::adjust(&centred, options, &layout)?;
    // Shift back; what did not move (fixed or unobserved poses, fixed or dropped points) is
    // returned as given, bit for bit.
    for ((pose, centred_pose), original) in solution
        .poses
        .iter_mut()
        .zip(&centred.poses)
        .zip(&problem.poses)
    {
        if pose == centred_pose {
            *pose = *original;
        } else {
            let moved = se3::translation(pose) + origin;
            pose.fixed_view_mut::<3, 1>(0, 3).copy_from(&moved);
        }
    }
    for ((point, centred_point), original) in solution
        .points
        .iter_mut()
        .zip(&centred.points)
        .zip(&problem.points)
    {
        if point == centred_point {
            *point = *original;
        } else {
            *point += origin;
        }
    }
    Ok(solution)
}

#[cfg(test)]
mod tests {
    //! Native tests of bundle adjustment, runnable without Python: the gauge's codes, the
    //! problem checks, and small synthetic scenes solved back to their truth. The Python
    //! reference holds every value to its parity fixture.

    use nalgebra::Vector6;

    use super::*;
    use crate::camera::Model;
    use crate::error::ErrorKind;

    /// Each gauge's discriminant is its position in `ALL`, the code it crosses as.
    #[test]
    fn codes_are_positions() {
        for (i, gauge) in Gauge::ALL.iter().enumerate() {
            assert_eq!(*gauge as usize, i, "{}", gauge.name());
        }
    }

    /// Pinhole intrinsics.
    const PINHOLE: [f64; 4] = [500.0, 500.0, 320.0, 240.0];

    /// A noise-free scene: three cameras along x looking down z at a cloud of points, every
    /// point seen by every camera, with the first two poses fixed.
    fn scene() -> Problem {
        let poses: Vec<Matrix4<f64>> = [-0.6, 0.0, 0.7]
            .iter()
            .enumerate()
            .map(|(c, &x)| {
                let angle = 0.05 * f64::from(u8::try_from(c).unwrap());
                se3::exp(&Vector6::new(x, 0.1 * angle, 0.0, 0.0, angle, -angle))
            })
            .collect();
        let points: Vec<Vector3<f64>> = (0..20)
            .map(|i| {
                let t = f64::from(i);
                Vector3::new((1.3 * t).sin(), (0.7 * t).cos(), 5.0 + (0.9 * t).sin())
            })
            .collect();
        let mut observations = Vec::new();
        let (mut cam_idx, mut pt_idx) = (Vec::new(), Vec::new());
        for (c, pose) in poses.iter().enumerate() {
            let t_cam_world = se3::inverse(pose).unwrap();
            for (p, point) in points.iter().enumerate() {
                let in_camera = se3::act(&t_cam_world, point).unwrap();
                observations.push(camera::project(Model::Pinhole, &PINHOLE, &in_camera).unwrap());
                cam_idx.push(i64::try_from(c).unwrap());
                pt_idx.push(i64::try_from(p).unwrap());
            }
        }
        let intrinsics = Intrinsics {
            values: PINHOLE.to_vec(),
            params: 4,
        };
        let mut problem = Problem::with_defaults(
            Model::Pinhole,
            intrinsics,
            poses,
            points,
            observations,
            cam_idx,
            pt_idx,
        );
        problem.fixed_poses = vec![true, true, false];
        problem
    }

    /// Tight settings, so a noise-free solve reaches the truth.
    fn tight() -> Options {
        Options {
            solver: nls::Options {
                max_iterations: 100,
                function_tolerance: 0.0,
                gradient_tolerance: 1e-12,
                step_tolerance: 1e-14,
                ..nls::Options::default()
            },
            ..Options::default()
        }
    }

    /// From a perturbed start, the free pose and the points return to the truth; the fixed
    /// poses come back bit for bit. This runs the layout, the Schur system and the bundle.
    #[test]
    fn noise_free_scene_recovers_truth() {
        let truth = scene();
        let mut start = truth.clone();
        start.poses[2] = se3::compose(
            &truth.poses[2],
            &se3::exp(&Vector6::new(0.02, -0.01, 0.03, 0.01, -0.02, 0.01)),
        )
        .unwrap();
        for (p, point) in start.points.iter_mut().enumerate() {
            let t = f64::from(u8::try_from(p).unwrap());
            *point += Vector3::new(0.01 * t.sin(), 0.02 * t.cos(), -0.03 * (2.0 * t).sin());
        }
        let solved = solve(&start, &tight()).unwrap();
        assert!(solved.summary.solve.termination.is_converged());
        assert_eq!(solved.summary.gauge, Gauge::None);
        assert_eq!(solved.poses[0], truth.poses[0]);
        assert!((solved.poses[2] - truth.poses[2]).amax() < 1e-9);
        for (got, want) in solved.points.iter().zip(&truth.points) {
            assert!((got - want).amax() < 1e-8);
        }
    }

    /// A point seen by one camera only is dropped and counted, with its observation.
    #[test]
    fn a_point_seen_once_is_dropped() {
        let mut problem = scene();
        problem.points.push(Vector3::new(0.2, -0.1, 4.0));
        problem.fixed_points.push(false);
        problem.point_anchor.push(-1);
        problem.observations.push(Vector2::new(330.0, 230.0));
        problem.cam_idx.push(0);
        problem.pt_idx.push(20);
        problem.weights.push(1.0);
        problem.depth_prior.push(f64::NAN);
        problem.depth_prior_weights.push(1.0);
        let solved = solve(&problem, &tight()).unwrap();
        assert_eq!(solved.summary.dropped_points, 1);
        assert_eq!(solved.summary.dropped_observations, 1);
        assert_eq!(solved.points[20], problem.points[20]);
    }

    /// Each malformed problem is `InvalidInput`, naming what is wrong.
    #[test]
    fn invalid_problems_are_rejected() {
        type Change = Box<dyn Fn(&mut Problem)>;
        let cases: Vec<(&str, Change)> = vec![
            ("cam_idx", Box::new(|p| p.cam_idx[0] = 9)),
            ("pt_idx", Box::new(|p| p.pt_idx[0] = -1)),
            ("weights", Box::new(|p| p.weights[1] = -1.0)),
            (
                "points, observations",
                Box::new(|p| p.points[0].x = f64::NAN),
            ),
            (
                "fixed masks",
                Box::new(|p| {
                    p.fixed_points.pop();
                }),
            ),
            ("depth_prior", Box::new(|p| p.depth_prior[0] = -0.5)),
            ("transform", Box::new(|p| p.poses[1][(3, 0)] = 1.0)),
            ("intrinsics", Box::new(|p| p.intrinsics.values[0] = 0.0)),
            (
                "fixed point cannot be anchored",
                Box::new(|p| {
                    p.point_anchor[0] = 0;
                    p.fixed_points[0] = true;
                }),
            ),
        ];
        for (message, change) in cases {
            let mut problem = scene();
            change(&mut problem);
            let err = solve(&problem, &tight()).unwrap_err();
            assert_eq!(err.kind, ErrorKind::InvalidInput, "{message}");
            assert!(err.message.contains(message), "{message}: {}", err.message);
        }
        let partial = Options {
            gauge: Gauge::Partial,
            ..tight()
        };
        assert!(
            solve(&scene(), &partial)
                .unwrap_err()
                .message
                .contains("partial")
        );
    }
}
