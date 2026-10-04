//! Reading a Bundle Adjustment in the Large (BAL) problem into a [`Problem`], natively, and
//! the BAL demo's solver settings; shared by the `bal` example and the benchmarks.
//!
//! The same conversion as the Python reader (`slamlab.eval.readers.read_bal`): BAL's camera
//! frame flips `y` and `z` relative to this package's, its cameras are world-to-camera
//! `(ω, t, f, k1, k2)`, and its observations are centred pixels with `y` up. The result has
//! one radial intrinsics group per camera, `f, k1, k2` free and the principal point fixed.
//! Plain text only: decompress a downloaded `.bz2` first (`bunzip2 -k`).

use std::path::Path;

use nalgebra::{Matrix3, Vector2, Vector3};
use slamlab_foundations::ba::{self, Gauge, Intrinsics, Problem};
use slamlab_foundations::camera::Model;
use slamlab_foundations::{nls, se3, so3};

/// BAL's camera frame to this package's: flip `y` and `z`.
fn flip() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, -1.0))
}

/// Reads a BAL problem from a plain-text file.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The problem, or a message naming what is wrong with the file.
pub fn read_bal(path: &Path) -> Result<Problem, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut words = text.split_whitespace();
    let mut count = || -> Result<usize, String> {
        words
            .next()
            .and_then(|w| w.parse().ok())
            .ok_or_else(|| "not a BAL header".to_string())
    };
    let (n_cams, n_points, n_obs) = (count()?, count()?, count()?);
    let numbers: Vec<f64> = text
        .split_whitespace()
        .skip(3)
        .map(str::parse)
        .collect::<Result<_, _>>()
        .map_err(|e| format!("not a BAL problem: {e}"))?;
    let expected = 4 * n_obs + 9 * n_cams + 3 * n_points;
    if numbers.len() != expected {
        return Err(format!(
            "expected {expected} numbers, found {}",
            numbers.len()
        ));
    }
    let (obs, rest) = numbers.split_at(4 * n_obs);
    let (cams, points) = rest.split_at(9 * n_cams);
    let index = |v: f64| format!("{v:.0}").parse::<i64>().map_err(|e| e.to_string());
    let mut cam_idx = Vec::with_capacity(n_obs);
    let mut pt_idx = Vec::with_capacity(n_obs);
    let mut observations = Vec::with_capacity(n_obs);
    for o in obs.as_chunks::<4>().0 {
        cam_idx.push(index(o[0])?);
        pt_idx.push(index(o[1])?);
        observations.push(Vector2::new(o[2], -o[3]));
    }
    let mut poses = Vec::with_capacity(n_cams);
    let mut values = Vec::with_capacity(5 * n_cams);
    for c in cams.as_chunks::<9>().0 {
        let rotation = flip() * so3::exp(&Vector3::new(c[0], c[1], c[2]));
        let translation = flip() * Vector3::new(c[3], c[4], c[5]);
        let t_cam_world = se3::from_parts(&rotation, &translation);
        poses.push(se3::inverse(&t_cam_world).map_err(|e| e.to_string())?);
        values.extend([c[6], 0.0, 0.0, c[7], c[8]]);
    }
    let fixed_intrinsics = (0..n_cams)
        .flat_map(|_| [false, true, true, false, false])
        .collect();
    Ok(Problem {
        model: Model::Radial,
        intrinsics: Intrinsics { values, params: 5 },
        pose_prior: poses.clone(),
        poses,
        points: points
            .as_chunks::<3>()
            .0
            .iter()
            .map(|&p| Vector3::from(p))
            .collect(),
        observations,
        cam_idx,
        pt_idx,
        intrinsics_idx: (0..n_cams)
            .map(|c| i64::try_from(c).map_err(|e| e.to_string()))
            .collect::<Result<_, _>>()?,
        weights: vec![1.0; n_obs],
        fixed_poses: vec![false; n_cams],
        fixed_intrinsics,
        fixed_points: vec![false; n_points],
        depth_prior: vec![f64::NAN; n_obs],
        depth_prior_weights: vec![1.0; n_obs],
        depth_affine: vec![Vector2::new(1.0, 0.0); n_cams],
        fixed_depth_affine: vec![true; n_cams],
        pose_prior_weights: vec![Vector2::zeros(); n_cams],
        point_anchor: vec![-1; n_points],
    })
}

/// The BAL demo's settings (`scripts/demo_bal.py`): Levenberg-Marquardt, at most 100
/// iterations, and no gauge removal, as BAL's own reference solves have none.
///
/// # Returns
///
/// The options.
pub fn demo_options() -> ba::Options {
    ba::Options {
        solver: nls::Options {
            max_iterations: 100,
            ..nls::Options::default()
        },
        gauge: Gauge::None,
        ..ba::Options::default()
    }
}
