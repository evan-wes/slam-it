/// @file
/// @brief Implementation of the tracking declared in slam_it/edge/tracking.hpp.
#include "slam_it/edge/tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "slam_it/edge/residuals.hpp"
#include "slam_it/foundations/correspondences.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/p3p.hpp"
#include "slam_it/foundations/pnp.hpp"
#include "slam_it/foundations/ransac.hpp"
#include "slam_it/foundations/so3.hpp"

namespace slam_it::edge {

namespace {

namespace matching = foundations::matching;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;
using foundations::correspondences::Rows3;

/// @brief NaN, for an unknown right column.
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief Check the camera, the settings, and a frame's matches: one point id (or -1) per
///        keypoint.
/// @param camera The camera.
/// @param options The settings.
/// @param points The matches.
/// @param frame The frame.
/// @param name The matches' name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_tracking(const Camera& camera, const Options& options,
                            std::span<const std::int64_t> points, const Frame& frame,
                            const char* name) {
  if (auto ok = check_setup(camera, options); !ok) {
    return ok;
  }
  if (points.size() != keypoint_count(frame) ||
      std::ranges::any_of(points, [](std::int64_t id) { return id < -1; })) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": expected one point id (or -1) per keypoint");
  }
  return {};
}

/// @brief The point id each keypoint matched, from a search's matches.
/// @param m The map.
/// @param frame The frame.
/// @param found The `(projection, keypoint)` matches.
/// @param rows Each projection's point row.
/// @return One id (or -1) per keypoint.
std::vector<std::int64_t> matched_ids(const Map& m, const Frame& frame,
                                      const std::vector<matching::Match>& found,
                                      const std::vector<std::int64_t>& rows) {
  std::vector<std::int64_t> out(keypoint_count(frame), -1);
  for (const matching::Match& match : found) {
    out[static_cast<std::size_t>(match.second)] =
        m.pt_id[static_cast<std::size_t>(rows[static_cast<std::size_t>(match.first)])];
  }
  return out;
}

/// @brief How many keypoints matched.
/// @param points One id (or -1) per keypoint.
/// @return The count.
std::int64_t count_matched(const std::vector<std::int64_t>& points) {
  return std::ranges::count_if(points, [](std::int64_t id) { return id >= 0; });
}

/// @brief A refinement's observations: each matched keypoint with a bearing in front, its
///        point, and what the chi-square test reads.
struct Observations {
  /// @brief The keypoints.
  std::vector<std::size_t> keypoints;
  /// @brief Their points.
  std::vector<Eigen::Vector3d> xyz;
  /// @brief Their pixels.
  std::vector<std::array<double, 2>> xy;
  /// @brief Their right columns.
  std::vector<double> right;
  /// @brief Their levels' variances.
  std::vector<double> variance;
};

/// @brief Gather a frame's matched keypoints (`_optimize_pose`'s `matched`).
/// @param m The map.
/// @param frame The frame.
/// @param points One point id (or -1) per keypoint.
/// @return The observations.
Observations gather(const Map& m, const Frame& frame, std::span<const std::int64_t> points) {
  Observations out;
  for (std::size_t i = 0; i < keypoint_count(frame); ++i) {
    const std::int64_t p = point_row(m, points[i]);
    if (p >= 0 && frame.bearings[i].z() > 0.0) {
      out.keypoints.push_back(i);
      out.xyz.push_back(m.pt_xyz[static_cast<std::size_t>(p)]);
      out.xy.push_back(frame.features.keypoints[i].xy);
      out.right.push_back(frame.right[i]);
      out.variance.push_back(sigma2(m, frame.features.keypoints[i].level));
    }
  }
  return out;
}

/// @brief Refine a pose on the observations a round uses, each weighted by its level's
///        information (`PoseOptimization`'s `invSigma2`).
/// @param pose The starting pose.
/// @param observations The observations.
/// @param chosen Which of them, by index.
/// @param frame The frame (the bearings).
/// @param scale The Cauchy loss's scale, in whitened units.
/// @return The refined pose, or the refinement's error.
Result<Eigen::Matrix4d> refine_round(const Eigen::Matrix4d& pose, const Observations& observations,
                                     const std::vector<std::size_t>& chosen, const Frame& frame,
                                     double scale) {
  Rows3 world(static_cast<Eigen::Index>(chosen.size()), 3);
  Rows3 rays(static_cast<Eigen::Index>(chosen.size()), 3);
  Eigen::VectorXd weights(static_cast<Eigen::Index>(chosen.size()));
  for (std::size_t j = 0; j < chosen.size(); ++j) {
    const auto row = static_cast<Eigen::Index>(j);
    world.row(row) = observations.xyz[chosen[j]].transpose();
    rays.row(row) = frame.bearings[observations.keypoints[chosen[j]]].transpose();
    weights(row) = 1.0 / observations.variance[chosen[j]];
  }
  auto solved = foundations::pnp::refine_pose(pose, world, rays, scale, weights);
  if (!solved) {
    return std::unexpected(std::move(solved.error()));
  }
  return solved->state;
}

/// @brief Classify every observation by the chi-square test at a pose.
/// @param camera The camera.
/// @param observations The observations.
/// @param pose The pose.
/// @param options The thresholds.
/// @return One flag per observation: true for an inlier.
Result<std::vector<bool>> classify(const Camera& camera, const Observations& observations,
                                   const Eigen::Matrix4d& pose, const Options& options) {
  std::vector<Eigen::Vector3d> moved;
  moved.reserve(observations.xyz.size());
  for (const Eigen::Vector3d& x : observations.xyz) {
    moved.push_back(to_camera(pose, x));
  }
  auto error = chi2(camera, moved, observations.xy, observations.right, observations.variance);
  if (!error) {
    return std::unexpected(std::move(error.error()));
  }
  std::vector<bool> out(moved.size());
  for (std::size_t j = 0; j < out.size(); ++j) {
    out[j] = (*error)[j] < threshold(observations.right[j], options);
  }
  return out;
}

/// @brief Refine a pose (`slam_it.edge.tracking._optimize_pose`), unchecked.
/// @param m The map.
/// @param camera The camera.
/// @param frame The frame.
/// @param tracked The starting pose and the matches.
/// @param options The settings.
/// @return The refined frame, or the refinement's error.
Result<Tracked> refine(const Map& m, const Camera& camera, const Frame& frame,
                       const Tracked& tracked, const Options& options) {
  const Observations observations = gather(m, frame, tracked.points);
  const double scale = std::sqrt(options.chi2_mono) / focal(camera);
  std::vector<bool> use(observations.keypoints.size(), true);
  Eigen::Matrix4d current = tracked.pose;
  for (std::int64_t round = 0; round < options.pose_rounds; ++round) {
    std::vector<std::size_t> chosen;
    for (std::size_t j = 0; j < use.size(); ++j) {
      if (use[j] && to_camera(current, observations.xyz[j]).z() > 0.0) {
        chosen.push_back(j);
      }
    }
    if (chosen.size() < foundations::p3p::kSampleSize) {  // refine_pose's minimum
      std::ranges::fill(use, false);
      for (const std::size_t j : chosen) {
        use[j] = true;
      }
      break;
    }
    auto pose = refine_round(current, observations, chosen, frame, scale);
    if (!pose) {
      return std::unexpected(std::move(pose.error()));
    }
    current = *pose;
    auto kept = classify(camera, observations, current, options);
    if (!kept) {
      return std::unexpected(std::move(kept.error()));
    }
    use = std::move(*kept);
  }
  Tracked out{.pose = current, .points = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  for (std::size_t j = 0; j < use.size(); ++j) {
    if (use[j]) {
      const std::size_t i = observations.keypoints[j];
      out.points[i] = tracked.points[i];
    }
  }
  return out;
}

/// @brief Select the points in view (`slam_it.edge.tracking._in_view`), unchecked.
/// @param m The map.
/// @param camera The camera.
/// @param pose The camera's pose.
/// @param rows The candidates.
/// @param options The settings.
/// @return The points in view, or the level prediction's error.
Result<InView> select_in_view(const Map& m, const Camera& camera, const Eigen::Matrix4d& pose,
                              std::span<const std::int64_t> rows, const Options& options) {
  const Eigen::Vector3d centre = camera_centre(pose);
  const double bf = focal(camera) * camera.baseline;
  InView out;
  std::vector<double> distances;
  for (const std::int64_t row : rows) {
    const auto p = static_cast<std::size_t>(row);
    const Eigen::Vector3d at = to_camera(pose, m.pt_xyz[p]);
    if (!(at.z() > 0.0)) {
      continue;
    }
    const Eigen::Vector2d pixel = project(camera, at);
    const Eigen::Vector3d offset = m.pt_xyz[p] - centre;
    const double distance = offset.norm();
    if (!inside(camera, pixel) || !in_distance_range(m, p, distance, options)) {
      continue;
    }
    const double view_cos = viewing_cos(m, p, offset, distance);
    if (!(view_cos >= options.view_cos_limit)) {
      continue;
    }
    out.rows.push_back(row);
    out.xy.push_back({pixel.x(), pixel.y()});
    out.right.push_back(camera.baseline > 0.0 ? pixel.x() - (bf / at.z()) : kNan);
    out.view_cos.push_back(view_cos);
    distances.push_back(distance);
  }
  auto levels = predict_level(m, out.rows, distances);
  if (!levels) {
    return std::unexpected(std::move(levels.error()));
  }
  out.level = std::move(*levels);
  return out;
}

/// @brief The level range a last-frame point is searched on (`TrackWithMotionModel`): its
///        level and the next ones, or with depth, its level and the coarser ones moving
///        forward, the finer ones moving back.
/// @param m The map.
/// @param camera The camera.
/// @param last_pose The last frame's pose.
/// @param predicted The predicted pose.
/// @param mode The mode.
/// @param level The last keypoint's level.
/// @return The lowest and highest level.
std::pair<std::int64_t, std::int64_t> level_range(const Map& m, const Camera& camera,
                                                  const Eigen::Matrix4d& last_pose,
                                                  const Eigen::Matrix4d& predicted, Mode mode,
                                                  std::int64_t level) {
  if (mode != Mode::kMonocular) {
    // The current camera's centre in the last camera's frame (ORB-SLAM's tlc).
    const double ahead =
        last_pose.block<1, 3>(2, 0).dot(camera_centre(predicted)) + last_pose(2, 3);
    if (ahead > camera.baseline) {
      return {level, m.levels - 1};
    }
    if (-ahead > camera.baseline) {
      return {0, level};
    }
  }
  return {level - 1, level + 1};
}

}  // namespace

Eigen::Matrix4d predict(const Eigen::Matrix4d& last_pose,
                        const std::optional<Eigen::Matrix4d>& velocity) {
  if (!velocity) {
    return last_pose;
  }
  Eigen::Matrix4d out = *velocity * last_pose;
  out.topLeftCorner<3, 3>() = foundations::so3::project_to_so3(out.topLeftCorner<3, 3>());
  return out;
}

Result<Tracked> optimize_pose(const Map& m, const Camera& camera, const Frame& frame,
                              const Tracked& tracked, const Options& options) {
  if (auto ok = check_tracking(camera, options, tracked.points, frame, "tracked.points"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return refine(m, camera, frame, tracked, options);
}

Result<InView> in_view(const Map& m, const Camera& camera, const Eigen::Matrix4d& pose,
                       std::span<const std::int64_t> rows, const Options& options) {
  if (auto ok = check_setup(camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (std::ranges::any_of(rows, [&m](std::int64_t r) {
        return r < 0 || std::cmp_greater_equal(r, point_count(m));
      })) {
    return fail(ErrorKind::kInvalidInput,
                "rows: rows must lie in [0, " + std::to_string(point_count(m)) + ")");
  }
  return select_in_view(m, camera, pose, rows, options);
}

LocalMapSummary summary(const LocalTracking& local) {
  return {.in_view = static_cast<std::int64_t>(local.visible.size()),
          .matches = count_matched(local.matched),
          .inliers = inliers(local.tracked)};
}

Result<Motion> track_last_frame(const Map& m, const Camera& camera, const Frame& frame,
                                const Frame& last, const Tracked& last_tracked,
                                const Eigen::Matrix4d& predicted, Mode mode,
                                const Options& options) {
  if (auto ok = check_tracking(camera, options, last_tracked.points, last, "last_tracked.points");
      !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const double bf = focal(camera) * camera.baseline;
  const double window =
      mode == Mode::kStereo ? options.stereo_motion_window : options.motion_window;
  std::vector<std::int64_t> rows;
  std::vector<std::int64_t> levels;
  std::vector<matching::Projection> projections;
  for (std::size_t i = 0; i < keypoint_count(last); ++i) {
    const std::int64_t p = point_row(m, last_tracked.points[i]);
    if (p < 0) {
      continue;
    }
    const Eigen::Vector3d at = to_camera(predicted, m.pt_xyz[static_cast<std::size_t>(p)]);
    if (!(at.z() > 0.0)) {
      continue;
    }
    const Eigen::Vector2d pixel = project(camera, at);
    if (!inside(camera, pixel)) {
      continue;
    }
    const foundations::orb::Keypoint& k = last.features.keypoints[i];
    const auto [low, high] = level_range(m, camera, last_tracked.pose, predicted, mode, k.level);
    rows.push_back(p);
    levels.push_back(k.level);
    projections.push_back({.xy = {pixel.x(), pixel.y()},
                           .radius = 0.0,
                           .min_level = low,
                           .max_level = high,
                           .descriptor = m.pt_descriptor[static_cast<std::size_t>(p)],
                           .angle = k.angle,
                           .right = mode == Mode::kMonocular ? kNan : pixel.x() - (bf / at.z())});
  }
  std::vector<std::int64_t> points(keypoint_count(frame), -1);
  bool widened = false;
  for (const double factor : {1.0, 2.0}) {
    widened = factor > 1.0;
    for (std::size_t j = 0; j < projections.size(); ++j) {
      projections[j].radius =
          factor * window * std::pow(m.scale_factor, static_cast<double>(levels[j]));
    }
    auto found = search(frame, projections,
                        {.max_distance = options.th_high, .ratio = 1.0, .check_orientation = true});
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    points = matched_ids(m, frame, *found, rows);
    if (count_matched(points) >= options.min_motion_matches) {
      break;
    }
  }
  const std::int64_t matches = count_matched(points);
  if (matches < options.min_motion_matches) {
    return Motion{.tracked = std::nullopt,
                  .summary = {.matches = matches, .widened = widened, .inliers = std::nullopt},
                  .matched = std::move(points)};
  }
  auto tracked = refine(m, camera, frame, {.pose = predicted, .points = points}, options);
  if (!tracked) {
    return std::unexpected(std::move(tracked.error()));
  }
  const std::int64_t kept = inliers(*tracked);
  Motion out{.tracked = std::nullopt,
             .summary = {.matches = matches, .widened = widened, .inliers = kept},
             .matched = std::move(points)};
  if (kept >= options.min_motion_inliers) {
    out.tracked = std::move(*tracked);
  }
  return out;
}

Result<LocalTracking> track_local_map(const Map& m, const Camera& camera, const Frame& frame,
                                      const Tracked& tracked, Mode mode, const Options& options,
                                      bool recovering) {
  if (auto ok = check_tracking(camera, options, tracked.points, frame, "tracked.points"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::vector<std::int64_t> local_points = observed_by(m, window(m, options.window));
  std::vector<std::int64_t> already;
  for (const std::int64_t id : tracked.points) {
    if (const std::int64_t p = point_row(m, id); p >= 0) {
      already.push_back(p);
    }
  }
  std::vector<std::int64_t> sorted_already = already;
  std::ranges::sort(sorted_already);
  std::vector<std::int64_t> candidates;
  std::ranges::set_difference(local_points, sorted_already, std::back_inserter(candidates));
  auto view = select_in_view(m, camera, tracked.pose, candidates, options);
  if (!view) {
    return std::unexpected(std::move(view.error()));
  }
  double th = mode == Mode::kRgbd ? options.rgbd_local_window : options.local_window;
  if (recovering) {
    th = options.recovery_local_window;
  }
  std::vector<matching::Projection> projections;
  for (std::size_t j = 0; j < view->rows.size(); ++j) {
    const double narrow = view->view_cos[j] > options.aligned_view_cos
                              ? options.aligned_view_radius
                              : options.oblique_view_radius;
    const std::int64_t level = view->level[j];
    projections.push_back(
        {.xy = view->xy[j],
         .radius = th * narrow * std::pow(m.scale_factor, static_cast<double>(level)),
         .min_level = level - 1,
         .max_level = level,
         .descriptor = m.pt_descriptor[static_cast<std::size_t>(view->rows[j])],
         .angle = 0,
         .right = view->right[j]});
  }
  std::vector<std::uint8_t> taken(keypoint_count(frame), 0);
  for (std::size_t i = 0; i < taken.size(); ++i) {
    taken[i] = tracked.points[i] >= 0 ? 1 : 0;
  }
  auto found = search(frame, projections,
                      {.max_distance = options.th_high,
                       .ratio = options.local_map_ratio,
                       .check_orientation = false},
                      taken);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  std::vector<std::int64_t> points = matched_ids(m, frame, *found, view->rows);
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (tracked.points[i] >= 0) {
      points[i] = tracked.points[i];
    }
  }
  auto refined = refine(m, camera, frame, {.pose = tracked.pose, .points = points}, options);
  if (!refined) {
    return std::unexpected(std::move(refined.error()));
  }
  LocalTracking out{
      .tracked = std::move(*refined), .visible = {}, .found = {}, .matched = std::move(points)};
  for (const std::int64_t p : already) {
    out.visible.push_back(m.pt_id[static_cast<std::size_t>(p)]);
  }
  for (const std::int64_t p : view->rows) {
    out.visible.push_back(m.pt_id[static_cast<std::size_t>(p)]);
  }
  std::ranges::copy_if(out.tracked.points, std::back_inserter(out.found),
                       [](std::int64_t id) { return id >= 0; });
  return out;
}

Result<PnpRun> run_pnp(const Eigen::Ref<const foundations::correspondences::Rows3>& world,
                       const Eigen::Ref<const foundations::correspondences::Rows3>& rays,
                       const foundations::ransac::Options& settings) {
  auto solved = foundations::pnp::pnp_ransac(world, rays, settings);
  if (!solved) {
    if (solved.error().kind == ErrorKind::kDegenerate) {
      return PnpRun{};
    }
    return std::unexpected(std::move(solved.error()));
  }
  const auto inliers = static_cast<std::int64_t>(solved->state.inliers.count());
  return PnpRun{.consensus = std::move(solved->state),
                .iterations = solved->summary.run.iterations,
                .termination = solved->summary.run.termination,
                .inliers = inliers};
}

Result<Relocalization> relocalize_locally(const Map& m, const Camera& camera, const Frame& frame,
                                          const Options& options) {
  if (auto ok = check_setup(camera, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::vector<std::int64_t> local_points = observed_by(m, window(m, options.window));
  Relocalization out{.tracked = std::nullopt,
                     .summary = {.candidates = static_cast<std::int64_t>(local_points.size()),
                                 .matches = std::nullopt,
                                 .iterations = std::nullopt,
                                 .termination = std::nullopt,
                                 .degenerate = false,
                                 .inliers = std::nullopt,
                                 .refined = std::nullopt},
                     .matched = std::vector<std::int64_t>(keypoint_count(frame), -1)};
  if (std::cmp_less(local_points.size(), options.pnp_min_inliers)) {
    return out;
  }
  std::vector<foundations::orb::Descriptor> descriptors;
  descriptors.reserve(local_points.size());
  for (const std::int64_t p : local_points) {
    descriptors.push_back(m.pt_descriptor[static_cast<std::size_t>(p)]);
  }
  std::vector<std::int64_t> angles;
  angles.reserve(keypoint_count(frame));
  for (const foundations::orb::Keypoint& k : frame.features.keypoints) {
    angles.push_back(k.angle);
  }
  const std::vector<std::int64_t> zeros(local_points.size(), 0);
  auto found = matching::match(frame.features.descriptors, descriptors, angles, zeros,
                               {.max_distance = options.th_high,
                                .ratio = options.relocalize_ratio,
                                .check_orientation = false});
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  std::vector<matching::Match> pairs;
  std::ranges::copy_if(*found, std::back_inserter(pairs), [&frame](const matching::Match& match) {
    return frame.bearings[static_cast<std::size_t>(match.first)].z() > 0.0;
  });
  for (const matching::Match& pair : pairs) {
    out.matched[static_cast<std::size_t>(pair.first)] =
        m.pt_id[static_cast<std::size_t>(local_points[static_cast<std::size_t>(pair.second)])];
  }
  out.summary.matches = static_cast<std::int64_t>(pairs.size());
  if (std::cmp_less(pairs.size(), options.pnp_min_inliers)) {
    return out;
  }
  Rows3 world(static_cast<Eigen::Index>(pairs.size()), 3);
  Rows3 rays(static_cast<Eigen::Index>(pairs.size()), 3);
  for (std::size_t j = 0; j < pairs.size(); ++j) {
    world.row(static_cast<Eigen::Index>(j)) =
        m.pt_xyz[static_cast<std::size_t>(local_points[static_cast<std::size_t>(pairs[j].second)])]
            .transpose();
    rays.row(static_cast<Eigen::Index>(j)) =
        frame.bearings[static_cast<std::size_t>(pairs[j].first)].transpose();
  }
  auto placed = pnp_consensus(world, rays,
                              {.threshold = options.pnp_threshold / focal(camera),
                               .seed = static_cast<std::uint64_t>(options.seed)},
                              options.pnp_min_inliers, out.summary);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  if (!*placed) {
    return out;
  }
  const auto& consensus = **placed;
  std::vector<std::int64_t> points(keypoint_count(frame), -1);
  for (std::size_t j = 0; j < pairs.size(); ++j) {
    if (consensus.inliers(static_cast<Eigen::Index>(j))) {
      points[static_cast<std::size_t>(pairs[j].first)] = m.pt_id[static_cast<std::size_t>(
          local_points[static_cast<std::size_t>(pairs[j].second)])];
    }
  }
  auto tracked = refine(m, camera, frame, {.pose = consensus.model, .points = points}, options);
  if (!tracked) {
    return std::unexpected(std::move(tracked.error()));
  }
  out.summary.refined = inliers(*tracked);
  if (*out.summary.refined >= options.pnp_min_inliers) {
    out.tracked = std::move(*tracked);
  }
  return out;
}

}  // namespace slam_it::edge
