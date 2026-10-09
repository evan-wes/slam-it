/// @file
/// @brief Native benchmarks of rung 0's C++ core, the counterparts of the Rust core's
///        `rust/slam-it-all-foundations/benches/foundations.rs`.
///
/// Small operations (SO(3), SE(3), a radtan projection) time the per-item cost the binding
/// batches over. Two robust estimators run on one fixed synthetic scene, built from the same
/// closed-form expressions as the Rust benches' (no random numbers, so both cores see identical
/// data): E-RANSAC with Stewénius' 5-point solver and PnP RANSAC with Ding's P3P, each refined.
/// The front end extracts ORB features (ORB-SLAM's settings) from a 640x480 synthetic frame
/// built from one integer expression, the Rust benches' too.
/// The headline is a whole bundle adjustment on a BAL problem, with the BAL demo's settings,
/// registered only when `SLAM_IT_ALL_BAL` names a decompressed problem file:
/// `SLAM_IT_ALL_BAL=<problem.txt> build/native/bench/foundations_bench_foundations`.
#include <benchmark/benchmark.h>

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "bal_reader.hpp"
#include "slam_it_all/foundations/ba.hpp"
#include "slam_it_all/foundations/camera.hpp"
#include "slam_it_all/foundations/correspondences.hpp"
#include "slam_it_all/foundations/epipolar.hpp"
#include "slam_it_all/foundations/image.hpp"
#include "slam_it_all/foundations/orb.hpp"
#include "slam_it_all/foundations/p3p.hpp"
#include "slam_it_all/foundations/pnp.hpp"
#include "slam_it_all/foundations/ransac.hpp"
#include "slam_it_all/foundations/se3.hpp"
#include "slam_it_all/foundations/so3.hpp"

namespace {

namespace foundations = slam_it_all::foundations;

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

/// @brief Matches in the RANSAC scene.
constexpr int kMatches = 500;
/// @brief The scene's focal length in pixels: the noise and the threshold are set in pixels.
constexpr double kFocal = 500.0;

/// @brief The RANSAC benchmarks' scene: world points, their bearings in camera 1 (at the
///        world origin) and camera 2, 30% of camera 2's replaced by outliers, half a pixel of
///        noise on the rest. The same expressions as `ransac_scene` in the Rust benches.
struct RansacScene {
  /// @brief The world points.
  slam_it_all::foundations::correspondences::Rows3 points;
  /// @brief Their bearings in camera 1.
  slam_it_all::foundations::correspondences::Rows3 bearings1;
  /// @brief Their bearings in camera 2, outliers included.
  slam_it_all::foundations::correspondences::Rows3 bearings2;
};

/// @brief Build the RANSAC scene.
/// @return The scene.
RansacScene ransac_scene() {
  foundations::se3::Vector6d xi;
  xi << 0.6, 0.05, -0.1, 0.03, -0.08, 0.04;
  const Eigen::Matrix4d pose = foundations::se3::exp(xi);
  RansacScene out{.points = {kMatches, 3}, .bearings1 = {kMatches, 3}, .bearings2 = {kMatches, 3}};
  for (int i = 0; i < kMatches; ++i) {
    const double t = i;
    const Eigen::Vector3d point(2.0 * std::sin((0.37 * t) + 0.1), 1.5 * std::cos(0.53 * t),
                                5.0 + (2.0 * std::sin((0.11 * t) + 0.3)));
    const Eigen::Vector3d seen = (pose.topLeftCorner<3, 3>() * point) + pose.topRightCorner<3, 1>();
    Eigen::Vector3d image((seen.x() / seen.z()) + (0.5 / kFocal * std::sin(1.3 * t)),
                          (seen.y() / seen.z()) + (0.5 / kFocal * std::cos(1.7 * t)), 1.0);
    if (i % 10 < 3) {
      image = Eigen::Vector3d(0.4 * std::sin(2.9 * t), 0.3 * std::cos(3.1 * t), 1.0);
    }
    out.points.row(i) = point.transpose();
    out.bearings1.row(i) = point.normalized().transpose();
    out.bearings2.row(i) = image.normalized().transpose();
  }
  return out;
}

/// @brief The RANSAC benchmarks' settings: a one-pixel threshold, a fixed seed.
/// @return The options.
slam_it_all::foundations::ransac::Options ransac_options() {
  slam_it_all::foundations::ransac::Options options;
  options.threshold = 1.0 / kFocal;
  options.seed = 7;
  return options;
}

/// @brief E-RANSAC (Stewénius' 5-point solver) with the Sampson refinement.
/// @param state The benchmark state.
void essential_ransac(benchmark::State& state) {
  const RansacScene scene = ransac_scene();
  const auto options = ransac_options();
  for (const auto _ : state) {
    benchmark::DoNotOptimize(foundations::epipolar::essential_ransac(
        scene.bearings1, scene.bearings2, options, foundations::epipolar::MinimalSolver::kStewenius,
        true));
  }
}

/// @brief PnP RANSAC (Ding's P3P, EPnP refits) with the pose refinement.
/// @param state The benchmark state.
void pnp_ransac(benchmark::State& state) {
  const RansacScene scene = ransac_scene();
  const auto options = ransac_options();
  for (const auto _ : state) {
    benchmark::DoNotOptimize(foundations::pnp::pnp_ransac(scene.points, scene.bearings2, options,
                                                          foundations::p3p::Solver::kDing, true));
  }
}

/// @brief A 640x480 synthetic frame: 16-pixel blocks of varied intensity with a fine texture,
///        from the same integer expression as the Rust benches'.
/// @return The frame.
foundations::image::Image bench_frame() {
  constexpr std::int64_t kRows = 480;
  constexpr std::int64_t kCols = 640;
  foundations::image::Image frame{.rows = kRows, .cols = kCols, .pixels = {}};
  frame.pixels.reserve(static_cast<std::size_t>(kRows * kCols));
  for (std::int64_t r = 0; r < kRows; ++r) {
    for (std::int64_t c = 0; c < kCols; ++c) {
      const std::int64_t value = (((c / 16) * 37 + (r / 16) * 61) % 200) + 28 + ((r * c) % 7);
      frame.pixels.push_back(static_cast<std::uint8_t>(value));
    }
  }
  return frame;
}

/// @brief ORB extraction with ORB-SLAM's settings (1000 features, 8 levels at 1.2).
/// @param state The benchmark state.
void orb_extract(benchmark::State& state) {
  const foundations::image::Image frame = bench_frame();
  const foundations::orb::Options options{};
  for (const auto _ : state) {
    benchmark::DoNotOptimize(foundations::orb::extract(frame, options));
  }
}

/// @brief Print each RANSAC's inliers and iterations once, so the two cores' runs can be
///        seen to take the same decisions before their times are compared.
void report_ransac_decisions() {
  const RansacScene scene = ransac_scene();
  const auto options = ransac_options();
  const auto essential = foundations::epipolar::essential_ransac(
      scene.bearings1, scene.bearings2, options, foundations::epipolar::MinimalSolver::kStewenius,
      true);
  const auto pose = foundations::pnp::pnp_ransac(scene.points, scene.bearings2, options,
                                                 foundations::p3p::Solver::kDing, true);
  if (essential && pose) {
    std::cerr << "essential_ransac: " << essential->summary.run.inlier_count << " inliers, "
              << essential->summary.run.iterations << " iterations\n"
              << "pnp_ransac: " << pose->summary.run.inlier_count << " inliers, "
              << pose->summary.run.iterations << " iterations\n";
  }
}

BENCHMARK(essential_ransac)->Unit(benchmark::kMillisecond);
BENCHMARK(pnp_ransac)->Unit(benchmark::kMillisecond);
BENCHMARK(orb_extract)->Unit(benchmark::kMillisecond);
BENCHMARK(so3_exp);
BENCHMARK(so3_log);
BENCHMARK(se3_log);
BENCHMARK(project_radtan);

}  // namespace

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  report_ransac_decisions();
  // NOLINTNEXTLINE(concurrency-mt-unsafe): read once, before any benchmark thread exists
  const char* bal = std::getenv("SLAM_IT_ALL_BAL");
  if (bal == nullptr) {
    std::cerr << "SLAM_IT_ALL_BAL not set: skipping the BAL benchmark\n";
  } else {
    const auto problem = slam_it_all::bench::read_bal(bal);
    if (!problem) {
      std::cerr << "SLAM_IT_ALL_BAL: " << bal << " is not a BAL problem\n";
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
