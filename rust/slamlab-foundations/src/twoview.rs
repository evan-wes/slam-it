//! Two-view initialisation: the Rust core's E and H RANSAC, GRIC model selection, and the
//! choice of motion by triangulation.
//!
//! Mirrors `slamlab.foundations.twoview`, whose docstring states the method: E and H by
//! LO-MSAC on independent random streams, Torr's GRIC over the matches either model explains,
//! the winner's decomposition, each candidate tried by triangulating the inliers, and E
//! deciding when H's candidates are ambiguous. The outcome is a status, not an error.

use nalgebra::{Matrix3, Matrix4, Vector2, Vector3};

use crate::correspondences::{check_bearing_pairs, gather, normalised_plane};
use crate::epipolar::{self, MinimalSolver};
use crate::error::{ErrorKind, Result, fail, fail_setting};
use crate::ransac::{self, Consensus, RefinedSolution, RefinedSummary};
use crate::{homography, se3, triangulation};

/// Torr's `lambda_3`: a residual's contribution is capped at `lambda_3 (r - d)`.
pub const GRIC_LAMBDA3: f64 = 2.0;
/// r: a correspondence is two image points.
pub const DATA_DIMENSION: usize = 4;
/// E's variety dimension in the data space.
pub const ESSENTIAL_DIMENSION: usize = 3;
/// E's degrees of freedom.
pub const ESSENTIAL_PARAMETERS: usize = 5;
/// H's variety dimension in the data space.
pub const HOMOGRAPHY_DIMENSION: usize = 2;
/// H's degrees of freedom.
pub const HOMOGRAPHY_PARAMETERS: usize = 8;

/// H's RANSAC threshold over E's: `sqrt(5.991 / 3.841)`, the ratio of the 95% quantiles of
/// chi-squared with two degrees of freedom and with one (H's residual has two).
///
/// # Returns
///
/// The scale.
#[must_use]
pub fn homography_threshold_scale() -> f64 {
    (5.991_f64 / 3.841).sqrt()
}

/// Which model GRIC chose, with the codes it crosses the bindings as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// The essential matrix: a general scene.
    Essential = 0,
    /// A homography: a plane, or a rotation.
    Homography = 1,
}

impl Model {
    /// Every model, in code order.
    pub const ALL: [Self; 2] = [Self::Essential, Self::Homography];

    /// The model's name, as `slamlab.foundations.twoview.Model` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Essential => "essential",
            Self::Homography => "homography",
        }
    }
}

/// Whether the two views initialise, and if not, why.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// One motion is clearly best, with enough well-triangulated points.
    Initialized = 0,
    /// The camera only rotated: no translation, nothing to triangulate.
    PureRotation = 1,
    /// Two candidate motions put similar numbers of points in front.
    Ambiguous = 2,
    /// Too few points have enough parallax for a reliable depth.
    LowParallax = 3,
}

impl Status {
    /// Every status, in code order.
    pub const ALL: [Self; 4] = [
        Self::Initialized,
        Self::PureRotation,
        Self::Ambiguous,
        Self::LowParallax,
    ];

    /// The status's name, as `slamlab.foundations.twoview.Status` spells it.
    ///
    /// # Returns
    ///
    /// The name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Initialized => "initialized",
            Self::PureRotation => "pure_rotation",
            Self::Ambiguous => "ambiguous",
            Self::LowParallax => "low_parallax",
        }
    }
}

/// The initialiser's settings (see `slamlab.foundations.twoview.Options`).
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    /// The focal length in pixels.
    pub focal: f64,
    /// E-RANSAC's inlier threshold (a Sampson distance), in pixels; H's is scaled by
    /// [`homography_threshold_scale`].
    pub threshold_px: f64,
    /// The image noise's standard deviation for GRIC, in pixels.
    pub sigma_px: f64,
    /// RANSAC's target confidence.
    pub confidence: f64,
    /// RANSAC's sample limit, per model.
    pub max_iterations: i64,
    /// The random seed; E uses stream 0 and H stream 1.
    pub seed: u64,
    /// The parallax a triangulated point needs to count, in degrees.
    pub min_parallax_deg: f64,
    /// The well-triangulated points initialisation needs.
    pub min_points: i64,
    /// The runner-up must have fewer than this fraction of the best's points in front.
    pub ambiguity_ratio: f64,
    /// E-RANSAC's minimal solver.
    pub essential_solver: MinimalSolver,
    /// Whether E-RANSAC's model is refined on its inliers' Sampson distances.
    pub refine: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            focal: 1.0,
            threshold_px: 2.0,
            sigma_px: 1.0,
            confidence: 0.999,
            max_iterations: 10_000,
            seed: 0,
            min_parallax_deg: 1.0,
            min_points: 50,
            ambiguity_ratio: 0.75,
            essential_solver: MinimalSolver::Stewenius,
            refine: true,
        }
    }
}

/// Checks the settings.
///
/// # Arguments
///
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` naming the setting out of range.
pub fn check_options(options: &Options) -> Result<()> {
    for (name, value) in [
        ("focal", options.focal),
        ("threshold_px", options.threshold_px),
        ("sigma_px", options.sigma_px),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return fail_setting(name, "must be positive and finite", value);
        }
    }
    if !options.min_parallax_deg.is_finite() || options.min_parallax_deg < 0.0 {
        return fail_setting(
            "min_parallax_deg",
            "must be non-negative",
            options.min_parallax_deg,
        );
    }
    if options.min_points < 1 {
        return fail_setting("min_points", "must be at least 1", options.min_points);
    }
    if !(options.ambiguity_ratio > 0.0 && options.ambiguity_ratio <= 1.0) {
        return fail_setting(
            "ambiguity_ratio",
            "must be in (0, 1]",
            options.ambiguity_ratio,
        );
    }
    Ok(())
}

/// Torr's GRIC score of a model's residuals.
///
/// # Arguments
///
/// * `squared_errors` - Each datum's squared (Sampson) distance to the model.
/// * `sigma` - The noise's standard deviation, in the same units.
/// * `dimension` - The model's dimension in the 4D data space.
/// * `parameters` - The model's degrees of freedom.
///
/// # Returns
///
/// `sum min(e^2 / sigma^2, lambda_3 (r - d)) + ln(r) d n + ln(r n) k`.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "counts and dimensions are far below 2^52"
)]
pub fn gric(squared_errors: &[f64], sigma: f64, dimension: usize, parameters: usize) -> f64 {
    let n = squared_errors.len() as f64;
    let r = DATA_DIMENSION as f64;
    let cap = GRIC_LAMBDA3 * (DATA_DIMENSION - dimension) as f64;
    let fit: f64 = squared_errors
        .iter()
        .map(|e| (e / (sigma * sigma)).min(cap))
        .sum();
    fit + r.ln() * dimension as f64 * n + (r * n).ln() * parameters as f64
}

/// The initialiser's result (see `slamlab.foundations.twoview.Initialization`).
#[derive(Debug, Clone, PartialEq)]
pub struct Initialization {
    /// Whether it initialised, and if not, why.
    pub status: Status,
    /// The model that decided: GRIC's choice, or E when it fell back.
    pub model: Model,
    /// Whether H won GRIC but its candidates were ambiguous, and E decided instead.
    pub fallback: bool,
    /// The best candidate `T_c2_c1`, unit translation (zero for a pure rotation).
    pub relative_pose: Matrix4<f64>,
    /// The chosen model's RANSAC inliers.
    pub inliers: Vec<bool>,
    /// The well-triangulated points in camera 1's frame; NaN elsewhere.
    pub points: Vec<Vector3<f64>>,
    /// Which matches have a point.
    pub triangulated: Vec<bool>,
    /// E's GRIC score (infinity if its RANSAC found no model).
    pub gric_essential: f64,
    /// H's GRIC score (infinity if its RANSAC found no model).
    pub gric_homography: f64,
    /// Each candidate's inliers in front of both cameras, best first.
    pub in_front: Vec<usize>,
    /// E-RANSAC's summary (with the refinement's), if it found a model.
    pub essential: Option<RefinedSummary>,
    /// H's RANSAC summary, if it found a model.
    pub homography: Option<ransac::Summary>,
}

/// One candidate motion, triangulated.
#[derive(Debug, Clone)]
struct Tried {
    /// The candidate, unit translation.
    pose: Matrix4<f64>,
    /// Its inliers in front of both cameras.
    in_front: usize,
    /// The inliers in front with enough parallax.
    good: Vec<bool>,
    /// Their points; NaN elsewhere.
    points: Vec<Vector3<f64>>,
}

/// A point that is not there.
const NO_POINT: Vector3<f64> = Vector3::new(f64::NAN, f64::NAN, f64::NAN);

/// One model's RANSAC options in the normalised plane, on its own stream.
///
/// # Arguments
///
/// * `options` - The initialiser's settings.
/// * `model` - E (stream 0, `threshold_px`) or H (stream 1, scaled).
///
/// # Returns
///
/// The options.
fn ransac_options(options: &Options, model: Model) -> ransac::Options {
    let planar = model == Model::Homography;
    let scale = if planar {
        homography_threshold_scale()
    } else {
        1.0
    };
    ransac::Options {
        threshold: scale * options.threshold_px / options.focal,
        confidence: options.confidence,
        max_iterations: options.max_iterations,
        seed: options.seed,
        stream: u64::from(planar),
        ..ransac::Options::default()
    }
}

/// Keeps a RANSAC's "no model" as an empty result.
///
/// # Arguments
///
/// * `run` - The RANSAC's result.
///
/// # Returns
///
/// The solution, or `None` if no sample gave a model.
///
/// # Errors
///
/// Any other error.
fn found<S>(run: Result<S>) -> Result<Option<S>> {
    match run {
        Ok(solution) => Ok(Some(solution)),
        Err(error) if error.kind == ErrorKind::Degenerate => Ok(None),
        Err(error) => Err(error),
    }
}

/// The indices of the set entries of a mask.
///
/// # Arguments
///
/// * `mask` - The mask.
///
/// # Returns
///
/// The indices, ascending.
fn indices(mask: &[bool]) -> Vec<usize> {
    mask.iter()
        .enumerate()
        .filter_map(|(i, &set)| set.then_some(i))
        .collect()
}

/// Points of the normalised plane without their ones.
///
/// # Arguments
///
/// * `points` - Points `(x, y, 1)`.
///
/// # Returns
///
/// `(x, y)` per point.
fn xy(points: &[Vector3<f64>]) -> Vec<Vector2<f64>> {
    points.iter().map(Vector3::xy).collect()
}

/// Triangulates a candidate's inliers; counts those in front, keeps those with parallax.
///
/// # Arguments
///
/// * `pose` - The candidate.
/// * `rays` - Both cameras' bearings.
/// * `inliers` - The chosen model's inliers.
/// * `min_parallax` - The parallax a point needs, in radians.
///
/// # Returns
///
/// The tried candidate.
///
/// # Errors
///
/// A triangulation error.
fn try_candidate(
    pose: &Matrix4<f64>,
    rays: (&[Vector3<f64>], &[Vector3<f64>]),
    inliers: &[bool],
    min_parallax: f64,
) -> Result<Tried> {
    let unit = se3::from_parts(&se3::rotation(pose), &se3::translation(pose).normalize());
    let index = indices(inliers);
    let result = triangulation::two_view(&unit, &gather(rays.0, &index), &gather(rays.1, &index))?;
    let mut out = Tried {
        pose: unit,
        in_front: 0,
        good: vec![false; inliers.len()],
        points: vec![NO_POINT; inliers.len()],
    };
    for (j, &i) in index.iter().enumerate() {
        if result.in_front[j] {
            out.in_front += 1;
            if result.parallax[j] >= min_parallax {
                out.good[i] = true;
                out.points[i] = result.points[j];
            }
        }
    }
    Ok(out)
}

/// The chosen model's candidate motions.
///
/// # Arguments
///
/// * `model` - The model.
/// * `matrix` - Its matrix.
///
/// # Returns
///
/// The candidates `T_c2_c1`.
///
/// # Errors
///
/// A decomposition error.
fn candidates(model: Model, matrix: &Matrix3<f64>) -> Result<Vec<Matrix4<f64>>> {
    match model {
        Model::Essential => epipolar::decompose_essential(matrix),
        Model::Homography => Ok(homography::decompose_homography(matrix)?.poses),
    }
}

/// The matches in the normalised image plane.
struct Planes {
    /// Camera 1's points `(x, y, 1)`.
    x1: Vec<Vector3<f64>>,
    /// Camera 2's points `(x, y, 1)`.
    x2: Vec<Vector3<f64>>,
}

/// Both RANSAC runs; at least one found a model.
struct Runs {
    /// E's run, if it found a model.
    essential: Option<RefinedSolution<Matrix3<f64>>>,
    /// H's run, if it found a model.
    planar: Option<ransac::Solution<Matrix3<f64>>>,
}

/// Runs E and H RANSAC on independent streams.
///
/// # Arguments
///
/// * `rays` - Both cameras' bearings.
/// * `planes` - The matches in the normalised plane.
/// * `options` - The settings.
///
/// # Returns
///
/// The runs.
///
/// # Errors
///
/// `Degenerate` if neither found a model; any other RANSAC error.
fn run_both(
    rays: (&[Vector3<f64>], &[Vector3<f64>]),
    planes: &Planes,
    options: &Options,
) -> Result<Runs> {
    let essential = found(epipolar::essential_ransac(
        rays.0,
        rays.1,
        &ransac_options(options, Model::Essential),
        options.essential_solver,
        options.refine,
    ))?;
    let planar = found(homography::homography_ransac(
        &xy(&planes.x1),
        &xy(&planes.x2),
        &ransac_options(options, Model::Homography),
    ))?;
    if essential.is_none() && planar.is_none() {
        return fail(
            ErrorKind::Degenerate,
            "initialize: neither RANSAC found a model",
        );
    }
    Ok(Runs { essential, planar })
}

/// Scores both models by GRIC over the matches at least one explains, and chooses.
///
/// # Arguments
///
/// * `runs` - Both runs.
/// * `planes` - The matches in the normalised plane.
/// * `sigma` - The noise in the normalised plane.
///
/// # Returns
///
/// A result with the scores, summaries and chosen model filled in.
fn scored(runs: &Runs, planes: &Planes, sigma: f64) -> Initialization {
    let n = planes.x1.len();
    let support: Vec<bool> = (0..n)
        .map(|i| {
            runs.essential.as_ref().is_some_and(|r| r.state.inliers[i])
                || runs.planar.as_ref().is_some_and(|r| r.state.inliers[i])
        })
        .collect();
    let explained = indices(&support);
    let x1 = gather(&planes.x1, &explained);
    let x2 = gather(&planes.x2, &explained);
    let gric_essential = runs.essential.as_ref().map_or(f64::INFINITY, |run| {
        let errors = epipolar::sampson(&run.state.model, &x1, &x2);
        gric(
            errors.as_slice(),
            sigma,
            ESSENTIAL_DIMENSION,
            ESSENTIAL_PARAMETERS,
        )
    });
    let gric_homography = runs.planar.as_ref().map_or(f64::INFINITY, |run| {
        let errors = homography::sampson(&run.state.model, &xy(&x1), &xy(&x2));
        gric(
            errors.as_slice(),
            sigma,
            HOMOGRAPHY_DIMENSION,
            HOMOGRAPHY_PARAMETERS,
        )
    });
    Initialization {
        status: Status::LowParallax,
        // The lower score is finite, so the chosen run exists.
        model: if gric_homography < gric_essential {
            Model::Homography
        } else {
            Model::Essential
        },
        fallback: false,
        relative_pose: Matrix4::identity(),
        inliers: Vec::new(),
        points: vec![NO_POINT; n],
        triangulated: vec![false; n],
        gric_essential,
        gric_homography,
        in_front: Vec::new(),
        essential: runs.essential.as_ref().map(|run| run.summary.clone()),
        homography: runs.planar.as_ref().map(|run| run.summary),
    }
}

/// A model's verdict: its status, best candidate and per-candidate counts.
struct Decision {
    /// The status this model reaches.
    status: Status,
    /// The best candidate (the first, for a pure rotation).
    pose: Matrix4<f64>,
    /// The best candidate, triangulated (none for a pure rotation).
    best: Option<Tried>,
    /// Each candidate's inliers in front, best first.
    in_front: Vec<usize>,
}

/// Tries a model's candidate motions on its inliers and judges the best.
///
/// # Arguments
///
/// * `model` - The model.
/// * `run` - Its RANSAC consensus.
/// * `rays` - Both cameras' bearings.
/// * `options` - The settings.
///
/// # Returns
///
/// The verdict.
///
/// # Errors
///
/// A decomposition or triangulation error.
#[allow(
    clippy::cast_precision_loss,
    reason = "inlier counts are far below 2^52"
)]
fn decide(
    model: Model,
    run: &Consensus<Matrix3<f64>>,
    rays: (&[Vector3<f64>], &[Vector3<f64>]),
    options: &Options,
) -> Result<Decision> {
    let poses = candidates(model, &run.model)?;
    if !(se3::translation(&poses[0]).norm() > 0.0) {
        return Ok(Decision {
            status: Status::PureRotation,
            pose: poses[0],
            best: None,
            in_front: Vec::new(),
        });
    }
    let min_parallax = options.min_parallax_deg.to_radians();
    let mut tried = poses
        .iter()
        .map(|pose| try_candidate(pose, rays, &run.inliers, min_parallax))
        .collect::<Result<Vec<Tried>>>()?;
    // Stable, as std::ranges::stable_sort (C++) and a stable argsort (Python) are.
    tried.sort_by_key(|t| std::cmp::Reverse(t.in_front));
    let best = &tried[0];
    let good = best.good.iter().filter(|&&g| g).count();
    let status = if i64::try_from(good).unwrap_or(i64::MAX) < options.min_points {
        Status::LowParallax
    } else if tried.len() > 1
        && tried[1].in_front as f64 > options.ambiguity_ratio * best.in_front as f64
    {
        Status::Ambiguous
    } else {
        Status::Initialized
    };
    Ok(Decision {
        status,
        pose: best.pose,
        in_front: tried.iter().map(|t| t.in_front).collect(),
        best: Some(best.clone()),
    })
}

/// Finds the relative pose and first points of two calibrated views, or says why not.
///
/// # Arguments
///
/// * `bearings1`, `bearings2` - Unit bearings with positive z, N >= 8.
/// * `options` - The settings.
///
/// # Returns
///
/// The result.
///
/// # Errors
///
/// `InvalidInput` for bad bearings or settings; `Degenerate` if neither RANSAC found a model.
pub fn initialize(
    bearings1: &[Vector3<f64>],
    bearings2: &[Vector3<f64>],
    options: &Options,
) -> Result<Initialization> {
    check_options(options)?;
    check_bearing_pairs(bearings1, bearings2, epipolar::SAMPLE_SIZE, true)?;
    let rays = (bearings1, bearings2);
    let planes = Planes {
        x1: normalised_plane(bearings1),
        x2: normalised_plane(bearings2),
    };
    let runs = run_both(rays, &planes, options)?;
    let mut out = scored(&runs, &planes, options.sigma_px / options.focal);
    let run = match out.model {
        Model::Homography => runs.planar.as_ref().map(|r| &r.state),
        Model::Essential => runs.essential.as_ref().map(|r| &r.state),
    };
    let Some(run) = run else {
        // Unreachable: the chosen model's score is finite only when its run found a model.
        return fail(
            ErrorKind::Degenerate,
            "initialize: the chosen model has no run",
        );
    };
    let mut decision = decide(out.model, run, rays, options)?;
    out.inliers.clone_from(&run.inliers);
    if decision.status == Status::Ambiguous
        && out.model == Model::Homography
        && let Some(essential) = &runs.essential
    {
        let alternative = decide(Model::Essential, &essential.state, rays, options)?;
        if alternative.status == Status::Initialized {
            out.model = Model::Essential;
            out.fallback = true;
            out.inliers.clone_from(&essential.state.inliers);
            decision = alternative;
        }
    }
    out.status = decision.status;
    out.relative_pose = decision.pose;
    out.in_front = decision.in_front;
    if let Some(best) = decision.best {
        out.points = best.points;
        out.triangulated = best.good;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests: a general scene initialises with E and recovers the motion.

    use nalgebra::Vector6;

    use super::*;

    /// A general scene through a known motion initialises with E, its unit translation
    /// recovered, and triangulates most matches.
    #[test]
    fn general_scene_initializes() {
        let pose = se3::exp(&Vector6::new(0.5, 0.05, -0.1, 0.02, -0.05, 0.03));
        let (mut b1, mut b2) = (Vec::new(), Vec::new());
        for i in 0..150_u32 {
            let t = f64::from(i);
            let point = Vector3::new(
                2.0 * (0.37 * t).sin(),
                1.5 * (0.53 * t).cos(),
                4.0 + 2.0 * (0.11 * t).sin(),
            );
            b1.push(point.normalize());
            b2.push(se3::act(&pose, &point).unwrap().normalize());
        }
        let options = Options {
            focal: 500.0,
            threshold_px: 1.0,
            ..Options::default()
        };
        let result = initialize(&b1, &b2, &options).unwrap();
        assert_eq!(result.status, Status::Initialized);
        assert_eq!(result.model, Model::Essential);
        let truth = se3::translation(&pose).normalize();
        assert!((se3::translation(&result.relative_pose) - truth).amax() < 1e-6);
        assert!(result.triangulated.iter().filter(|&&g| g).count() > 100);
    }
}
