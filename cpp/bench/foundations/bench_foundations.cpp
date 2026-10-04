/// @file
/// @brief Native benchmarks of rung 0's C++ core, the counterparts of the Rust core's
///        `rust/slamlab-foundations/benches/foundations.rs`.
///
/// Small operations (SO(3), SE(3), a radtan projection) time the per-item cost the binding
/// batches over; the headline is a whole bundle adjustment on a BAL problem, with the BAL
/// demo's settings, registered only when `SLAMLAB_BAL` names a decompressed problem file:
/// `SLAMLAB_BAL=<problem.txt> build/native/bench/foundations_bench_foundations`.
#include <benchmark/benchmark.h>

#include <Eigen/Core>
#include <cstdlib>
#include <iostream>
#include <string>

#include "bal_reader.hpp"
#include "slamlab/foundations/ba.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/se3.hpp"
#include "slamlab/foundations/so3.hpp"

namespace {

namespace foundations = slamlab::foundations;

/// @brief SO(3)'s exponential.
/// @param state The benchmark state.
void so3_exp(benchmark::State& state) {
  const Eigen::Vector3d omega(0.3, -1.1, 0.7);
  for (const auto _ : state) {
    benchmark::DoNotOptimize(foundations::so3::exp(omega));
  }
}

/// @brief SO(3)'s logarithm.
/// @param state The benchmark state.
void so3_log(benchmark::State& state) {
  const Eigen::Matrix3d rotation = foundations::so3::exp({0.3, -1.1, 0.7});
  for (const auto _ : state) {
    benchmark::DoNotOptimize(foundations::so3::log(rotation));
  }
}

/// @brief SE(3)'s logarithm.
/// @param state The benchmark state.
void se3_log(benchmark::State& state) {
  foundations::se3::Vector6d xi;
  xi << 0.4, -0.2, 1.5, 0.3, -1.1, 0.7;
  const Eigen::Matrix4d transform = foundations::se3::exp(xi);
  for (const auto _ : state) {
    benchmark::DoNotOptimize(foundations::se3::log(transform));
  }
}

/// @brief A radtan projection.
/// @param state The benchmark state.
void project_radtan(benchmark::State& state) {
  Eigen::VectorXd radtan(9);
  radtan << 500.0, 480.0, 320.0, 240.0, -0.2, 0.05, 1e-3, -2e-3, 0.01;
  const Eigen::Vector3d point(0.3, -0.2, 2.0);
  for (const auto _ : state) {
    benchmark::DoNotOptimize(
        foundations::camera::project(foundations::camera::Model::kRadTan, radtan, point));
  }
}

BENCHMARK(so3_exp);
BENCHMARK(so3_log);
BENCHMARK(se3_log);
BENCHMARK(project_radtan);

}  // namespace

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  // NOLINTNEXTLINE(concurrency-mt-unsafe): read once, before any benchmark thread exists
  const char* bal = std::getenv("SLAMLAB_BAL");
  if (bal == nullptr) {
    std::cerr << "SLAMLAB_BAL not set: skipping the BAL benchmark\n";
  } else {
    const auto problem = slamlab::bench::read_bal(bal);
    if (!problem) {
      std::cerr << "SLAMLAB_BAL: " << bal << " is not a BAL problem\n";
      return 1;
    }
    foundations::ba::Options options;
    options.solver.max_iterations = 100;
    options.gauge = foundations::ba::Gauge::kNone;
    benchmark::RegisterBenchmark(
        "ba/bal",
        [problem = *problem, options](benchmark::State& state) {
          for (const auto _ : state) {
            benchmark::DoNotOptimize(foundations::ba::solve(problem, options));
          }
        })
        ->Unit(benchmark::kMillisecond)
        ->Iterations(10);
  }
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
