//! Bundle adjustment on a real BAL problem, in Rust alone: no Python anywhere.
//!
//! The standalone counterpart of `scripts/demo_bal.py`: it reads a Bundle Adjustment in the
//! Large problem, solves it with the demo's settings (Levenberg-Marquardt, at most 100
//! iterations, the gauge left to the data), and prints the RMS reprojection error over the
//! observations in front of their camera at the start, and the time. It shows the core
//! building and running without Python, as the edge budgets need.
//!
//! Run from `rust/`, with a decompressed problem:
//!
//! ```text
//! bunzip2 -k ~/datasets/bal/ladybug/problem-49-7776-pre.txt.bz2
//! cargo run --release --example bal -- ~/datasets/bal/ladybug/problem-49-7776-pre.txt
//! ```

mod reader;

use std::path::PathBuf;
use std::time::Instant;

use nalgebra::{Matrix4, Vector2, Vector3};
use slam_it_foundations::ba::{self, Intrinsics, Problem};
use slam_it_foundations::{camera, se3};

/// One observation's projection at some variables, or `None` if its point is behind.
///
/// # Arguments
///
/// * `problem` - The problem (for its indices and model).
/// * `m` - The observation.
/// * `poses`, `points`, `intrinsics` - The variables.
///
/// # Returns
///
/// The pixel.
fn project(
    problem: &Problem,
    m: usize,
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    intrinsics: &Intrinsics,
) -> Option<Vector2<f64>> {
    let c = usize::try_from(problem.cam_idx[m]).ok()?;
    let p = usize::try_from(problem.pt_idx[m]).ok()?;
    let in_camera = se3::act(&se3::inverse(&poses[c]).ok()?, &points[p]).ok()?;
    camera::project(problem.model, intrinsics.group(c), &in_camera).ok()
}

/// The RMS reprojection error (px) over the `used` observations.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `used` - The observations to include.
/// * `poses`, `points`, `intrinsics` - The variables to measure.
///
/// # Returns
///
/// `sqrt(mean |π(X) - u|²)`, or a message if a used point went behind its camera.
fn rms(
    problem: &Problem,
    used: &[bool],
    poses: &[Matrix4<f64>],
    points: &[Vector3<f64>],
    intrinsics: &Intrinsics,
) -> Result<f64, String> {
    let mut sum = 0.0;
    let mut count = 0_u32;
    for (m, observed) in problem.observations.iter().enumerate() {
        if !used[m] {
            continue;
        }
        let pixel = project(problem, m, poses, points, intrinsics)
            .ok_or("a used observation's point is behind its camera")?;
        sum += (pixel - observed).norm_squared();
        count += 1;
    }
    Ok((sum / f64::from(count.max(1))).sqrt())
}

/// Reads, solves and reports.
///
/// # Errors
///
/// A message if the file cannot be read or the solve fails.
fn main() -> Result<(), String> {
    let path: PathBuf = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .ok_or("usage: bal <problem.txt>")?;
    let problem = reader::read_bal(&path)?;
    let (poses, points, intrinsics) = (&problem.poses, &problem.points, &problem.intrinsics);
    let used: Vec<bool> = (0..problem.observations.len())
        .map(|m| project(&problem, m, poses, points, intrinsics).is_some())
        .collect();
    let behind = used.iter().filter(|&&u| !u).count();
    println!(
        "{}: {} cameras, {} points, {} observations ({behind} behind their camera, left out \
         of every RMS); start RMS {:.4} px",
        path.display(),
        problem.poses.len(),
        problem.points.len(),
        problem.observations.len(),
        rms(&problem, &used, poses, points, intrinsics)?,
    );
    let options = reader::demo_options();
    let began = Instant::now();
    let solution = ba::solve::solve(&problem, &options).map_err(|e| e.to_string())?;
    let seconds = began.elapsed().as_secs_f64();
    let summary = &solution.summary.solve;
    println!(
        "SLAM It!   rust: RMS {:.4} px, {} iterations ({}), {seconds:.2} s",
        rms(
            &problem,
            &used,
            &solution.poses,
            &solution.points,
            &solution.intrinsics
        )?,
        summary.trace.len(),
        summary.termination.name(),
    );
    Ok(())
}
