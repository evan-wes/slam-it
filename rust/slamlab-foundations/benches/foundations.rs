//! Native benchmarks of rung 0's Rust core, the counterparts of `cpp/bench/foundations/`.
//!
//! Small operations (SO(3), SE(3), a radtan projection) time the per-item cost the binding
//! batches over. Two robust estimators run on one fixed synthetic scene, built from the same
//! closed-form expressions as the C++ benches' (no random numbers, so both cores see identical
//! data): E-RANSAC with Stewénius' 5-point solver and PnP RANSAC with Ding's P3P, each refined.
//! The headline is a whole bundle adjustment on a BAL problem, with the BAL demo's settings.
//! The BAL benchmark runs only when `SLAMLAB_BAL` names a decompressed problem file:
//!
//! ```text
//! SLAMLAB_BAL=~/datasets/bal/ladybug/problem-49-7776-pre.txt cargo bench --bench foundations
//! ```

#[path = "../examples/bal/reader.rs"]
#[allow(
    dead_code,
    unused_imports,
    reason = "the reader's own tests compile here too, but only the example's run them"
)]
mod reader;

use std::hint::black_box;
use std::path::PathBuf;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};
use nalgebra::{Vector3, Vector6};
use slamlab_foundations::camera::{self, Model};
use slamlab_foundations::epipolar::{self, MinimalSolver};
use slamlab_foundations::p3p::Solver;
use slamlab_foundations::{ba, pnp, ransac, se3, so3};

/// SO(3)'s exponential and logarithm, SE(3)'s logarithm, and a radtan projection.
fn small_operations(c: &mut Criterion) {
    let omega = Vector3::new(0.3, -1.1, 0.7);
    let rotation = so3::exp(&omega);
    let transform = se3::exp(&Vector6::new(0.4, -0.2, 1.5, 0.3, -1.1, 0.7));
    let radtan = [500.0, 480.0, 320.0, 240.0, -0.2, 0.05, 1e-3, -2e-3, 0.01];
    let point = Vector3::new(0.3, -0.2, 2.0);
    c.bench_function("so3_exp", |b| b.iter(|| so3::exp(black_box(&omega))));
    c.bench_function("so3_log", |b| b.iter(|| so3::log(black_box(&rotation))));
    c.bench_function("se3_log", |b| b.iter(|| se3::log(black_box(&transform))));
    c.bench_function("project_radtan", |b| {
        b.iter(|| camera::project(Model::RadTan, black_box(&radtan), black_box(&point)));
    });
}

/// Matches in the RANSAC scene.
const MATCHES: u32 = 500;
/// The scene's focal length in pixels: the noise and the threshold are set in pixels.
const FOCAL: f64 = 500.0;

/// The RANSAC benchmarks' scene: world points, their bearings in camera 1 (at the world
/// origin) and camera 2, 30% of camera 2's replaced by outliers, half a pixel of noise on the
/// rest. The same expressions as `ransac_scene` in the C++ benches.
struct RansacScene {
    /// The world points.
    points: Vec<Vector3<f64>>,
    /// Their bearings in camera 1.
    bearings1: Vec<Vector3<f64>>,
    /// Their bearings in camera 2, outliers included.
    bearings2: Vec<Vector3<f64>>,
}

/// Builds the RANSAC scene.
///
/// # Panics
///
/// Never: the pose is a valid transform.
fn ransac_scene() -> RansacScene {
    let pose = se3::exp(&Vector6::new(0.6, 0.05, -0.1, 0.03, -0.08, 0.04));
    let mut out = RansacScene {
        points: Vec::new(),
        bearings1: Vec::new(),
        bearings2: Vec::new(),
    };
    for i in 0..MATCHES {
        let t = f64::from(i);
        let point = Vector3::new(
            2.0 * (0.37 * t + 0.1).sin(),
            1.5 * (0.53 * t).cos(),
            5.0 + 2.0 * (0.11 * t + 0.3).sin(),
        );
        let seen = se3::act(&pose, &point).expect("a valid pose");
        let mut image = Vector3::new(
            seen.x / seen.z + 0.5 / FOCAL * (1.3 * t).sin(),
            seen.y / seen.z + 0.5 / FOCAL * (1.7 * t).cos(),
            1.0,
        );
        if i % 10 < 3 {
            image = Vector3::new(0.4 * (2.9 * t).sin(), 0.3 * (3.1 * t).cos(), 1.0);
        }
        out.points.push(point);
        out.bearings1.push(point.normalize());
        out.bearings2.push(image.normalize());
    }
    out
}

/// The RANSAC benchmarks' settings: a one-pixel threshold, a fixed seed.
fn ransac_options() -> ransac::Options {
    ransac::Options {
        threshold: 1.0 / FOCAL,
        seed: 7,
        ..ransac::Options::default()
    }
}

/// E-RANSAC (Stewénius' 5-point solver) and PnP RANSAC (Ding's P3P, EPnP refits), each
/// refined. Each run's inliers and iterations are printed once first, so the two cores' runs
/// can be seen to take the same decisions before their times are compared.
///
/// # Panics
///
/// If either estimator fails on the scene.
fn robust_estimators(c: &mut Criterion) {
    let scene = ransac_scene();
    let options = ransac_options();
    let essential = |s: &RansacScene| {
        epipolar::essential_ransac(
            &s.bearings1,
            &s.bearings2,
            &options,
            MinimalSolver::Stewenius,
            true,
        )
        .expect("E-RANSAC finds a model")
    };
    let pose = |s: &RansacScene| {
        pnp::pnp_ransac(&s.points, &s.bearings2, &options, Solver::Ding, true)
            .expect("PnP RANSAC finds a pose")
    };
    let (e, p) = (essential(&scene), pose(&scene));
    eprintln!(
        "essential_ransac: {} inliers, {} iterations",
        e.summary.run.inlier_count, e.summary.run.iterations
    );
    eprintln!(
        "pnp_ransac: {} inliers, {} iterations",
        p.summary.run.inlier_count, p.summary.run.iterations
    );
    c.bench_function("essential_ransac", |b| {
        b.iter(|| essential(black_box(&scene)));
    });
    c.bench_function("pnp_ransac", |b| b.iter(|| pose(black_box(&scene))));
}

/// A whole bundle adjustment on the BAL problem `SLAMLAB_BAL` names, if it does.
///
/// # Panics
///
/// If the named file is not a BAL problem, or the solve fails.
fn bal(c: &mut Criterion) {
    let Some(path) = std::env::var_os("SLAMLAB_BAL").map(PathBuf::from) else {
        eprintln!("SLAMLAB_BAL not set: skipping the BAL benchmark");
        return;
    };
    let problem = reader::read_bal(&path).expect("SLAMLAB_BAL names a BAL problem");
    let options = reader::demo_options();
    let mut group = c.benchmark_group("ba");
    group
        .sample_size(10)
        .measurement_time(Duration::from_secs(60));
    group.bench_function("bal", |b| {
        b.iter(|| ba::solve::solve(black_box(&problem), &options).expect("the solve succeeds"));
    });
    group.finish();
}

criterion_group!(benches, small_operations, robust_estimators, bal);
criterion_main!(benches);
