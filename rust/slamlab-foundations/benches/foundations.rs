//! Native benchmarks of rung 0's Rust core, the counterparts of `cpp/bench/foundations/`.
//!
//! Small operations (SO(3), SE(3), a radtan projection) time the per-item cost the binding
//! batches over; the headline is a whole bundle adjustment on a BAL problem, with the BAL
//! demo's settings. The BAL benchmark runs only when `SLAMLAB_BAL` names a decompressed
//! problem file:
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
use slamlab_foundations::ba;
use slamlab_foundations::camera::{self, Model};
use slamlab_foundations::{se3, so3};

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
        b.iter(|| ba::solve(black_box(&problem), &options).expect("the solve succeeds"));
    });
    group.finish();
}

criterion_group!(benches, small_operations, bal);
criterion_main!(benches);
