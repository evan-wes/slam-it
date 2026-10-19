/// @file
/// @brief Implementation of the initialisation declared in slam_it/edge/initialization.hpp.
#include "slam_it/edge/initialization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "slam_it/edge/adjust.hpp"
#include "slam_it/edge/covisibility.hpp"
#include "slam_it/foundations/triangulation.hpp"
#include "slam_it/foundations/twoview.hpp"

namespace slam_it::edge {

namespace {

namespace matching = foundations::matching;
namespace twoview = foundations::twoview;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

/// @brief Check the settings and a frame's pyramid against them.
/// @param frame The frame.
/// @param options The settings.
/// @param name The frame's name, for the message.
/// @return Nothing, or `InvalidInput`.
Result<void> check_frame(const Frame& frame, const Options& options, const char* name) {
  if (auto ok = check_options(options); !ok) {
    return ok;
  }
  if (frame.features.levels != options.features.levels ||
      frame.features.scale_factor != options.features.scale_factor) {
    return fail(ErrorKind::kInvalidInput,
                std::string(name) + ": its pyramid must be the settings'");
  }
  return {};
}

/// @brief Check a monocular pair of frames against the settings.
/// @param reference The reference frame.
/// @param frame The later frame.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the frame whose pyramid is not the settings'.
Result<void> check_frames(const Frame& reference, const Frame& frame, const Options& options) {
  if (auto ok = check_frame(reference, options, "reference"); !ok) {
    return ok;
  }
  return check_frame(frame, options, "frame");
}

/// @brief Add a keyframe that observes no point yet.
/// @param m The map.
/// @param frame The frame.
/// @param pose Its `T_cam_world`.
/// @return Nothing, or the map's error.
Result<void> insert_unobserving(Map& m, const Frame& frame, const Eigen::Matrix4d& pose) {
  const std::vector<std::int64_t> none(keypoint_count(frame), -1);
  return insert_keyframe(m, frame, pose, none);
}

/// @brief Scale the map's translations and points, and refresh the points (a monocular map's
///        gauge, `slam_it.edge.initialization._scaled`).
/// @param m The map.
/// @param factor The scale.
/// @return Nothing, or the map's error.
Result<void> scale(Map& m, double factor) {
  for (Eigen::Matrix4d& pose : m.kf_pose) {
    pose.topRightCorner<3, 1>() *= factor;
  }
  for (Eigen::Vector3d& x : m.pt_xyz) {
    x *= factor;
  }
  std::vector<std::int64_t> rows(point_count(m));
  for (std::size_t p = 0; p < rows.size(); ++p) {
    rows[p] = static_cast<std::int64_t>(p);
  }
  return refresh_points(m, rows);
}

/// @brief The median of some values: the mean of the middle two for an even count.
/// @param values The values (reordered).
/// @return The median.
double median(std::vector<double>& values) {
  const std::size_t half = values.size() / 2;
  std::ranges::nth_element(values, values.begin() + static_cast<std::ptrdiff_t>(half));
  const double upper = values[half];
  if (values.size() % 2 == 1) {
    return upper;
  }
  const double lower =
      *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(half));
  return (lower + upper) / 2.0;
}

/// @brief The matches a monocular start keeps, and their points
///        (`slam_it.edge.initialization._kept_points`, ORB-SLAM's `CheckRT`): the chosen
///        motion's inliers in front of both views with rays whose cosine is below
///        `initialization.parallax_cos`.
/// @param result The two-view initialisation.
/// @param bearings1 The matches' bearings in the reference (N x 3).
/// @param bearings2 In the later frame.
/// @param options The parallax.
/// @return The kept matches' indices and their points; the triangulation's error.
Result<std::pair<std::vector<Eigen::Index>, std::vector<Eigen::Vector3d>>> kept_points(
    const twoview::Initialization& result, const foundations::correspondences::Rows3& bearings1,
    const foundations::correspondences::Rows3& bearings2, const Options& options) {
  std::vector<Eigen::Index> inliers;
  for (Eigen::Index i = 0; i < result.inliers.size(); ++i) {
    if (result.inliers(i)) {
      inliers.push_back(i);
    }
  }
  const auto count = static_cast<Eigen::Index>(inliers.size());
  foundations::correspondences::Rows3 rays1(count, 3);
  foundations::correspondences::Rows3 rays2(count, 3);
  for (Eigen::Index j = 0; j < count; ++j) {
    rays1.row(j) = bearings1.row(inliers[static_cast<std::size_t>(j)]);
    rays2.row(j) = bearings2.row(inliers[static_cast<std::size_t>(j)]);
  }
  auto seen = foundations::triangulation::two_view(result.relative_pose, rays1, rays2);
  if (!seen) {
    return std::unexpected(std::move(seen.error()));
  }
  std::pair<std::vector<Eigen::Index>, std::vector<Eigen::Vector3d>> out;
  for (Eigen::Index j = 0; j < count; ++j) {
    if (seen->in_front(j) && std::cos(seen->parallax(j)) < options.initialization.parallax_cos) {
      out.first.push_back(inliers[static_cast<std::size_t>(j)]);
      out.second.emplace_back(seen->points.row(j).transpose());
    }
  }
  return out;
}

/// @brief An attempt that did not start the map.
/// @param outcome Why.
/// @param matches The pairs given, if any.
/// @param solves The adjustment's solves, if it ran.
/// @return The attempt.
Initialization failed(Outcome outcome, std::optional<std::int64_t> matches = std::nullopt,
                      std::vector<Solve> solves = {}) {
  return {.started = std::nullopt,
          .summary = {.outcome = outcome, .matches = matches, .solves = std::move(solves)}};
}

/// @brief The outcome a two-view status that did not initialise stands for.
/// @param status The status.
/// @return The outcome of the same name.
Outcome outcome_of(twoview::Status status) {
  switch (status) {
    case twoview::Status::kPureRotation:
      return Outcome::kPureRotation;
    case twoview::Status::kAmbiguous:
      return Outcome::kAmbiguous;
    case twoview::Status::kLowParallax:
      return Outcome::kLowParallax;
    case twoview::Status::kInitialized:
    case twoview::Status::kCount:
      break;
  }
  return Outcome::kInitialized;
}

}  // namespace

const char* name(Outcome outcome) {
  switch (outcome) {
    case Outcome::kInitialized:
      return "initialized";
    case Outcome::kTooFewFeatures:
      return "too_few_features";
    case Outcome::kReferenceSet:
      return "reference_set";
    case Outcome::kReferenceReplaced:
      return "reference_replaced";
    case Outcome::kTooFewMatches:
      return "too_few_matches";
    case Outcome::kDegenerate:
      return "degenerate";
    case Outcome::kPureRotation:
      return "pure_rotation";
    case Outcome::kAmbiguous:
      return "ambiguous";
    case Outcome::kLowParallax:
      return "low_parallax";
    case Outcome::kTooFewPoints:
      return "too_few_points";
    case Outcome::kBadScale:
      return "bad_scale";
    case Outcome::kCount:
      break;
  }
  return "unknown";
}

Result<Initialization> initialize_from_depth(const Frame& frame, const Options& options) {
  if (auto ok = check_frame(frame, options, "frame"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (std::cmp_less_equal(keypoint_count(frame), options.initialization.min_features_depth)) {
    return failed(Outcome::kTooFewFeatures);
  }
  std::vector<std::int64_t> has_depth;
  for (std::size_t i = 0; i < keypoint_count(frame); ++i) {
    if (!std::isnan(frame.depth[i]) && !std::isnan(frame.bearings[i].x())) {
      has_depth.push_back(static_cast<std::int64_t>(i));
    }
  }
  Started out{.map = empty_map(options.features.levels, options.features.scale_factor),
              .tracked = {}};
  if (auto ok = insert_unobserving(out.map, frame, Eigen::Matrix4d::Identity()); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::vector<Eigen::Vector3d> xyz;
  std::vector<std::vector<std::int64_t>> observers;
  for (const std::int64_t i : has_depth) {
    const Eigen::Vector3d& b = frame.bearings[static_cast<std::size_t>(i)];
    xyz.emplace_back(b * (frame.depth[static_cast<std::size_t>(i)] / b.z()));
    observers.push_back({i});
  }
  if (auto ok = add_points(out.map, xyz, observers); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  out.tracked.points.assign(keypoint_count(frame), -1);
  for (std::size_t j = 0; j < has_depth.size(); ++j) {
    out.tracked.points[static_cast<std::size_t>(has_depth[j])] = out.map.pt_id[j];
  }
  return Initialization{
      .started = std::move(out),
      .summary = {.outcome = Outcome::kInitialized, .matches = std::nullopt, .solves = {}}};
}

Result<std::vector<matching::Match>> match_for_initialization(
    const Frame& reference, const Frame& frame, std::span<const std::array<double, 2>> previous,
    const Options& options) {
  if (auto ok = check_frames(reference, frame, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (previous.size() != keypoint_count(reference) ||
      std::ranges::any_of(previous, [](const std::array<double, 2>& xy) {
        return !std::isfinite(xy[0]) || !std::isfinite(xy[1]);
      })) {
    return fail(ErrorKind::kInvalidInput,
                "previous: expected one finite pixel per reference keypoint");
  }
  std::vector<std::int64_t> rows;
  std::vector<matching::Projection> projections;
  for (std::size_t i = 0; i < keypoint_count(reference); ++i) {
    const foundations::orb::Keypoint& k = reference.features.keypoints[i];
    if (k.level == 0) {
      rows.push_back(static_cast<std::int64_t>(i));
      projections.push_back({.xy = previous[i],
                             .radius = options.initialization.window,
                             .min_level = 0,
                             .max_level = 0,
                             .descriptor = reference.features.descriptors[i],
                             .angle = k.angle,
                             .right = std::numeric_limits<double>::quiet_NaN()});
    }
  }
  auto found = search(frame, projections,
                      {.max_distance = options.matching.th_low,
                       .ratio = options.initialization.ratio,
                       .check_orientation = true});
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  for (matching::Match& match : *found) {
    match.first = rows[static_cast<std::size_t>(match.first)];
  }
  return found;
}

Result<Initialization> initialize_monocular(const Camera& camera, const Frame& reference,
                                            const Frame& frame,
                                            std::span<const std::array<std::int64_t, 2>> pairs,
                                            const Options& options) {
  if (auto ok = check_camera(camera); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_frames(reference, frame, options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (std::ranges::any_of(pairs, [&](const std::array<std::int64_t, 2>& pair) {
        return pair[0] < 0 || pair[1] < 0 ||
               std::cmp_greater_equal(pair[0], keypoint_count(reference)) ||
               std::cmp_greater_equal(pair[1], keypoint_count(frame));
      })) {
    return fail(ErrorKind::kInvalidInput,
                "matches.pairs: each must be a keypoint of the reference and of the frame");
  }
  const auto given = static_cast<std::int64_t>(pairs.size());
  std::vector<std::array<std::int64_t, 2>> usable;
  for (const std::array<std::int64_t, 2>& pair : pairs) {
    if (!std::isnan(reference.bearings[static_cast<std::size_t>(pair[0])].x()) &&
        !std::isnan(frame.bearings[static_cast<std::size_t>(pair[1])].x())) {
      usable.push_back(pair);
    }
  }
  if (std::cmp_less(usable.size(), options.initialization.min_matches)) {
    return failed(Outcome::kTooFewMatches, given);
  }
  const auto n = static_cast<Eigen::Index>(usable.size());
  foundations::correspondences::Rows3 bearings1(n, 3);
  foundations::correspondences::Rows3 bearings2(n, 3);
  for (Eigen::Index i = 0; i < n; ++i) {
    const std::array<std::int64_t, 2>& pair = usable[static_cast<std::size_t>(i)];
    bearings1.row(i) = reference.bearings[static_cast<std::size_t>(pair[0])].transpose();
    bearings2.row(i) = frame.bearings[static_cast<std::size_t>(pair[1])].transpose();
  }
  auto result = twoview::initialize(
      bearings1, bearings2,
      {.focal = focal(camera),
       .max_iterations = static_cast<int>(options.initialization.ransac_iterations),
       .seed = static_cast<std::uint64_t>(options.seed)});
  if (!result) {
    if (result.error().kind == ErrorKind::kDegenerate) {
      return failed(Outcome::kDegenerate, given);
    }
    return std::unexpected(std::move(result.error()));
  }
  if (result->status != twoview::Status::kInitialized) {
    return failed(outcome_of(result->status), given);
  }
  Started out{.map = empty_map(options.features.levels, options.features.scale_factor),
              .tracked = {}};
  Map& m = out.map;
  if (auto ok = insert_unobserving(m, reference, Eigen::Matrix4d::Identity()); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = insert_unobserving(m, frame, result->relative_pose); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  auto kept = kept_points(*result, bearings1, bearings2, options);
  if (!kept) {
    return std::unexpected(std::move(kept.error()));
  }
  const std::int64_t second = m.kf_start[1];
  std::vector<std::vector<std::int64_t>> observers;
  for (const Eigen::Index i : kept->first) {
    const std::array<std::int64_t, 2>& pair = usable[static_cast<std::size_t>(i)];
    observers.push_back({pair[0], second + pair[1]});
  }
  const std::vector<Eigen::Vector3d>& xyz = kept->second;
  if (auto ok = add_points(m, xyz, observers); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = adopt(m, 1); !ok) {  // the second keyframe's parent
    return std::unexpected(std::move(ok.error()));
  }
  Options adjusted = options;
  adjusted.adjustment.iterations = options.initialization.ba_iterations;
  const std::vector<std::int64_t> both{0, 1};
  const std::vector<std::uint8_t> fixed{1, 0};
  auto adjustment = bundle_adjust(m, camera, both, fixed, adjusted);
  if (!adjustment) {
    return std::unexpected(std::move(adjustment.error()));
  }
  if (std::cmp_less(point_count(m), options.initialization.min_matches)) {
    // ORB-SLAM resets with fewer than 100 tracked points
    return failed(Outcome::kTooFewPoints, given, std::move(adjustment->solves));
  }
  const Eigen::Matrix4d& first = m.kf_pose[0];
  std::vector<double> depths;
  depths.reserve(point_count(m));
  for (const Eigen::Vector3d& x : m.pt_xyz) {
    depths.push_back(first.block<1, 3>(2, 0).dot(x.transpose()) + first(2, 3));
  }
  const double middle = median(depths);
  if (!(middle > 0.0)) {
    return failed(Outcome::kBadScale, given, std::move(adjustment->solves));
  }
  if (auto ok = scale(m, 1.0 / middle); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  out.tracked.pose = m.kf_pose[1];
  out.tracked.points.assign(m.kp_point.begin() + m.kf_start[1], m.kp_point.begin() + m.kf_start[2]);
  return Initialization{.started = std::move(out),
                        .summary = {.outcome = Outcome::kInitialized,
                                    .matches = given,
                                    .solves = std::move(adjustment->solves)}};
}

}  // namespace slam_it::edge
