/// @file
/// @brief Implementation of the bundle adjustment declared in slam_it/edge/adjust.hpp.
#include "slam_it/edge/adjust.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "slam_it/edge/residuals.hpp"
#include "slam_it/foundations/ba.hpp"
#include "slam_it/foundations/nls.hpp"
#include "slam_it/foundations/se3.hpp"

namespace slam_it::edge {

namespace {

namespace ba = foundations::ba;
namespace nls = foundations::nls;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

/// @brief The BA problem built from the map, and where its rows came from.
struct Built {
  /// @brief The problem.
  ba::Problem problem;
  /// @brief Each observation's keypoint row, ascending.
  std::vector<std::int64_t> rows;
  /// @brief Each camera's keyframe row, ascending.
  std::vector<std::int64_t> observing;
};

/// @brief Build the BA problem (`slam_it.edge.adjust._problem`).
/// @param m The map.
/// @param camera The camera.
/// @param keyframes The keyframes to adjust.
/// @param fixed The keyframes held fixed.
/// @param points The points the keyframes observe, by row, ascending.
/// @return The problem; an error if a pose cannot be inverted.
Result<Built> build(const Map& m, const Camera& camera, std::span<const std::int64_t> keyframes,
                    std::span<const std::uint8_t> fixed, const std::vector<std::int64_t>& points) {
  std::vector<bool> free(keyframe_count(m), false);
  for (const std::int64_t k : keyframes) {
    free[static_cast<std::size_t>(k)] = fixed[static_cast<std::size_t>(k)] == 0;
  }
  // (keypoint row, point index): the observations in keypoint order, as Python's scan finds them.
  std::vector<std::pair<std::int64_t, std::int64_t>> seen;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (const std::int64_t r : m.pt_observations[static_cast<std::size_t>(points[i])]) {
      seen.emplace_back(r, static_cast<std::int64_t>(i));
    }
  }
  std::ranges::sort(seen);
  Built out;
  std::vector<std::int64_t> point_of;
  std::vector<std::size_t> keyframe_rows;
  std::set<std::int64_t> observing;
  for (const auto& [r, i] : seen) {
    out.rows.push_back(r);
    point_of.push_back(i);
    keyframe_rows.push_back(keyframe_of(m, r));
    observing.insert(static_cast<std::int64_t>(keyframe_rows.back()));
  }
  out.observing.assign(observing.begin(), observing.end());
  std::vector<std::int64_t> camera_index(keyframe_count(m), -1);
  ba::Poses poses;
  for (std::size_t c = 0; c < out.observing.size(); ++c) {
    const auto k = static_cast<std::size_t>(out.observing[c]);
    camera_index[k] = static_cast<std::int64_t>(c);
    auto inverse = foundations::se3::inverse(m.kf_pose[k]);
    if (!inverse) {
      return std::unexpected(std::move(inverse.error()));
    }
    poses.push_back(*inverse);
  }
  const std::size_t n = out.rows.size();
  ba::Points xyz(static_cast<Eigen::Index>(points.size()), 3);
  for (std::size_t i = 0; i < points.size(); ++i) {
    xyz.row(static_cast<Eigen::Index>(i)) = m.pt_xyz[static_cast<std::size_t>(points[i])];
  }
  ba::Pixels observations(static_cast<Eigen::Index>(n), 2);
  ba::Indices cam_idx(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto r = static_cast<std::size_t>(out.rows[i]);
    observations.row(static_cast<Eigen::Index>(i)) << m.kp_xy[r][0], m.kp_xy[r][1];
    cam_idx[i] = camera_index[keyframe_rows[i]];
  }
  ba::Intrinsics intrinsics = camera.intrinsics.transpose();
  out.problem = ba::Problem::with_defaults(camera.model, std::move(intrinsics), std::move(poses),
                                           std::move(xyz), std::move(observations),
                                           std::move(cam_idx), std::move(point_of));
  ba::Problem& problem = out.problem;
  const double bf = focal(camera) * camera.baseline;
  for (std::size_t i = 0; i < n; ++i) {
    const auto r = static_cast<std::size_t>(out.rows[i]);
    const double variance = sigma2(m, m.kp_level[r]);
    const auto e = static_cast<Eigen::Index>(i);
    problem.weights(e) = 1.0 / variance;
    if (std::isnan(m.kp_depth[r])) {
      problem.depth_prior_weights(e) = 0.0;
    } else {
      problem.depth_prior(e) = 1.0 / m.kp_depth[r];
      problem.depth_prior_weights(e) = (bf * bf) / variance;
    }
  }
  for (std::size_t c = 0; c < out.observing.size(); ++c) {
    problem.fixed_poses[c] = !free[static_cast<std::size_t>(out.observing[c])];
  }
  return out;
}

/// @brief The BA settings: Levenberg-Marquardt with a Huber loss, capped iterations.
/// @param options The edge settings (the loss's scale).
/// @param iterations The cap.
/// @return The settings.
ba::Options solver_options(const Options& options, std::int64_t iterations) {
  ba::Options out;
  out.solver.loss =
      nls::Loss{.kind = nls::LossKind::kHuber, .scale = std::sqrt(options.adjustment.chi2_mono)};
  out.solver.max_iterations = static_cast<int>(iterations);
  return out;
}

/// @brief Which observations fail the chi-square test, or are behind their camera
///        (`slam_it.edge.adjust._outliers`).
/// @param m The map (the keypoints).
/// @param camera The camera.
/// @param problem The problem, at the estimate to test.
/// @param rows Each observation's keypoint row.
/// @param options The thresholds.
/// @return One flag per observation; an error if a pose cannot be inverted.
Result<std::vector<bool>> outliers(const Map& m, const Camera& camera, const ba::Problem& problem,
                                   const std::vector<std::int64_t>& rows, const Options& options) {
  std::vector<Eigen::Matrix4d> t_cam_world;
  for (const Eigen::Matrix4d& pose : problem.poses) {
    auto inverse = foundations::se3::inverse(pose);
    if (!inverse) {
      return std::unexpected(std::move(inverse.error()));
    }
    t_cam_world.push_back(*inverse);
  }
  const std::size_t n = rows.size();
  std::vector<Eigen::Vector3d> local(n);
  std::vector<std::array<double, 2>> xy(n);
  std::vector<double> right(n);
  std::vector<double> variance(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Matrix4d& t = t_cam_world[static_cast<std::size_t>(problem.cam_idx[i])];
    const Eigen::Vector3d point =
        problem.points.row(static_cast<Eigen::Index>(problem.pt_idx[i])).transpose();
    local[i] = (t.topLeftCorner<3, 3>() * point) + t.topRightCorner<3, 1>();
    const auto r = static_cast<std::size_t>(rows[i]);
    xy[i] = m.kp_xy[r];
    right[i] = m.kp_right[r];
    variance[i] = sigma2(m, m.kp_level[r]);
  }
  auto error = chi2(camera, local, xy, right, variance);
  if (!error) {
    return std::unexpected(std::move(error.error()));
  }
  std::vector<bool> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = !((*error)[i] < threshold(right[i], options));
  }
  return out;
}

/// @brief Check the arguments.
/// @param m The map.
/// @param camera The camera.
/// @param keyframes The keyframe rows.
/// @param fixed The flags.
/// @param options The settings.
/// @return Nothing, or `InvalidInput`.
Result<void> check(const Map& m, const Camera& camera, std::span<const std::int64_t> keyframes,
                   std::span<const std::uint8_t> fixed, const Options& options) {
  if (auto ok = check_setup(camera, options); !ok) {
    return ok;
  }
  if (std::ranges::any_of(keyframes, [&m](std::int64_t k) {
        return k < 0 || std::cmp_greater_equal(k, keyframe_count(m));
      })) {
    return fail(ErrorKind::kInvalidInput,
                "keyframes: indices must lie in [0, " + std::to_string(keyframe_count(m)) + ")");
  }
  if (fixed.size() != keyframe_count(m)) {
    return fail(ErrorKind::kInvalidInput, "fixed: expected one flag per keyframe");
  }
  return {};
}

/// @brief The second solve's problem: the outliers' weights zeroed, and a point left with
///        fewer than two inlier views back where it started (left out of the solve, it would
///        otherwise keep where its outliers pulled it).
/// @param problem The problem at the first solve's estimate.
/// @param start The points before the first solve.
/// @param outlier Each observation's flag.
/// @return The problem.
ba::Problem refined_problem(const ba::Problem& problem, const ba::Points& start,
                            const std::vector<bool>& outlier) {
  ba::Problem out = problem;
  std::vector<int> inlier_views(static_cast<std::size_t>(start.rows()), 0);
  for (std::size_t i = 0; i < outlier.size(); ++i) {
    if (outlier[i]) {
      out.weights(static_cast<Eigen::Index>(i)) = 0.0;
      out.depth_prior_weights(static_cast<Eigen::Index>(i)) = 0.0;
    } else {
      ++inlier_views[static_cast<std::size_t>(problem.pt_idx[i])];
    }
  }
  for (std::size_t p = 0; p < inlier_views.size(); ++p) {
    if (inlier_views[p] < 2) {
      out.points.row(static_cast<Eigen::Index>(p)) = start.row(static_cast<Eigen::Index>(p));
    }
  }
  return out;
}

/// @brief Write a solution into the map: the free keyframes' poses and the points.
/// @param m The map.
/// @param built The problem and its rows.
/// @param solution The solution.
/// @param points The points' rows.
/// @return Nothing, or an error if a pose cannot be inverted.
Result<void> write_back(Map& m, const Built& built, const ba::Solution& solution,
                        const std::vector<std::int64_t>& points) {
  for (std::size_t c = 0; c < built.observing.size(); ++c) {
    if (!built.problem.fixed_poses[c]) {
      auto pose = foundations::se3::inverse(solution.poses[c]);
      if (!pose) {
        return std::unexpected(std::move(pose.error()));
      }
      m.kf_pose[static_cast<std::size_t>(built.observing[c])] = *pose;
    }
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    m.pt_xyz[static_cast<std::size_t>(points[i])] =
        solution.points.row(static_cast<Eigen::Index>(i)).transpose();
  }
  return {};
}

/// @brief Forget the observations still failing the test, then refresh the adjusted points
///        that remain.
/// @param m The map.
/// @param rows Each observation's keypoint row.
/// @param still Each observation's flag.
/// @param ids The adjusted points' ids.
/// @return Nothing, or the map's error.
Result<std::int64_t> forget_and_refresh(Map& m, const std::vector<std::int64_t>& rows,
                                        const std::vector<bool>& still,
                                        const std::vector<std::int64_t>& ids) {
  std::vector<std::int64_t> dropped;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (still[i]) {
      dropped.push_back(rows[i]);
    }
  }
  if (auto ok = forget_observations(m, dropped); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<std::int64_t> kept;
  for (const std::int64_t id : ids) {
    if (const std::int64_t p = point_row(m, id); p >= 0) {
      kept.push_back(p);
    }
  }
  if (auto ok = refresh_points(m, kept); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return static_cast<std::int64_t>(dropped.size());
}

/// @brief Write a solve back to the map, forget the observations it failed, and report it.
/// @param m The map.
/// @param built The problem and its rows.
/// @param solution The solve kept.
/// @param points The points' rows.
/// @param failing Each observation's flag: dropped as an outlier.
/// @param solves The solves run.
/// @param out The report so far: the keyframes free and fixed, and whether it was aborted.
/// @return The report with the solves and the outliers dropped; an error if a pose cannot be
///         inverted, or the map's.
Result<Adjustment> written_back(Map& m, const Built& built, const ba::Solution& solution,
                                const std::vector<std::int64_t>& points,
                                const std::vector<bool>& failing, std::vector<Solve> solves,
                                Adjustment out) {
  if (auto ok = write_back(m, built, solution, points); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<std::int64_t> ids;
  ids.reserve(points.size());
  for (const std::int64_t p : points) {
    ids.push_back(m.pt_id[static_cast<std::size_t>(p)]);
  }
  auto dropped = forget_and_refresh(m, built.rows, failing, ids);
  if (!dropped) {
    return std::unexpected(std::move(dropped.error()));
  }
  out.solves = std::move(solves);
  out.outliers = *dropped;
  return out;
}

}  // namespace

Solve solve_of(const foundations::nls::Summary& summary) {
  return {.initial_cost = summary.initial_cost,
          .final_cost = summary.final_cost,
          .iterations = summary.iterations,
          .termination = summary.termination};
}

namespace {

/// @brief Holds a local BA outside the map's lock while it lives (`Around`): left on making,
///        entered again before the result is written, or on the way out.
class Outside {
 public:
  /// @brief Leave the map's lock.
  /// @param around What to do around the solves; null for nothing.
  explicit Outside(const Around* around) : around_(around) {
    if (around_ != nullptr) {
      around_->leave();
    }
  }

  /// @brief Enter it again, if not yet.
  ~Outside() { enter(); }

  Outside(const Outside&) = delete;
  Outside& operator=(const Outside&) = delete;
  Outside(Outside&&) = delete;
  Outside& operator=(Outside&&) = delete;

  /// @brief Enter the map's lock again, before writing the result back.
  void enter() {
    if (around_ != nullptr && !entered_) {
      entered_ = true;
      around_->enter();
    }
  }

 private:
  /// @brief What to do around the solves.
  const Around* around_;
  /// @brief Whether the lock was entered again.
  bool entered_{false};
};

}  // namespace

Result<Adjustment> bundle_adjust(Map& m, const Camera& camera,
                                 std::span<const std::int64_t> keyframes,
                                 std::span<const std::uint8_t> fixed, const Options& options,
                                 const std::atomic<bool>* abort, const Around* around) {
  if (auto ok = check(m, camera, keyframes, fixed, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::vector<std::int64_t> points = observed_by(m, keyframes);
  if (points.empty()) {
    return Adjustment{};
  }
  auto built = build(m, camera, keyframes, fixed, points);
  if (!built) {
    return std::unexpected(std::move(built.error()));
  }
  ba::Problem& problem = built->problem;
  const auto held = static_cast<std::int64_t>(std::ranges::count(problem.fixed_poses, true));
  const auto posed = static_cast<std::int64_t>(problem.fixed_poses.size());
  if (held == 0) {
    return Adjustment{};  // no gauge: ORB-SLAM3 skips it
  }
  const auto stopped = [abort] { return abort != nullptr && abort->load(); };
  if (stopped()) {
    return Adjustment{.solves = {}, .outliers = 0, .free = 0, .fixed = 0, .aborted = true};
  }
  Outside outside(around);
  ba::Options first_options = solver_options(options, options.adjustment.iterations);
  first_options.solver.stop = abort;
  auto first = ba::solve(problem, first_options);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  const ba::Points start = problem.points;
  problem.poses = std::move(first->poses);
  problem.points = std::move(first->points);
  auto outlier = outliers(m, camera, problem, built->rows, options);
  if (!outlier) {
    return std::unexpected(std::move(outlier.error()));
  }
  if (stopped()) {  // ORB-SLAM2's bDoMore: the first solve kept, its outliers dropped
    ba::Solution kept = std::move(*first);
    kept.poses = problem.poses;
    kept.points = problem.points;
    outside.enter();
    return written_back(
        m, *built, kept, points, *outlier, {solve_of(kept.summary.solve)},
        Adjustment{
            .solves = {}, .outliers = 0, .free = posed - held, .fixed = held, .aborted = true});
  }
  ba::Options second_options = solver_options(options, options.adjustment.refine_iterations);
  second_options.solver.stop = abort;
  auto second = ba::solve(refined_problem(problem, start, *outlier), second_options);
  if (!second) {
    return std::unexpected(std::move(second.error()));
  }
  problem.poses = second->poses;
  problem.points = second->points;
  auto still = outliers(m, camera, problem, built->rows, options);
  if (!still) {
    return std::unexpected(std::move(still.error()));
  }
  const std::vector<Solve> solves{solve_of(first->summary.solve), solve_of(second->summary.solve)};
  const bool interrupted = std::ranges::any_of(solves, [](const Solve& solve) {
    return solve.termination == foundations::nls::Termination::kStopped;
  });
  outside.enter();
  return written_back(m, *built, *second, points, *still, solves,
                      Adjustment{.solves = {},
                                 .outliers = 0,
                                 .free = posed - held,
                                 .fixed = held,
                                 .aborted = interrupted});
}

}  // namespace slam_it::edge
