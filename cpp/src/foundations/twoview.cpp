/// @file
/// @brief Implementation of the two-view initialisation declared in
///        slam_it_all/foundations/twoview.hpp.
#include "slam_it_all/foundations/twoview.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <utility>

#include "slam_it_all/foundations/homography.hpp"
#include "slam_it_all/foundations/triangulation.hpp"

namespace slam_it_all::foundations::twoview {

namespace {

/// @brief One candidate motion, triangulated.
struct Tried {
  /// @brief The candidate, unit translation.
  Eigen::Matrix4d pose;
  /// @brief Its inliers in front of both cameras.
  int in_front = 0;
  /// @brief The inliers in front with enough parallax.
  Eigen::Array<bool, Eigen::Dynamic, 1> good;
  /// @brief Their points; NaN elsewhere.
  correspondences::Rows3 points;
};

/// @brief One model's RANSAC options in the normalised plane, on its own stream.
/// @param options The initialiser's settings.
/// @param model E (stream 0, `threshold_px`) or H (stream 1, scaled by
///        `kHomographyThresholdScale`).
/// @return The options.
ransac::Options ransac_options(const Options& options, Model model) {
  const bool planar = model == Model::kHomography;
  const double scale = planar ? kHomographyThresholdScale : 1.0;
  return ransac::Options{.threshold = scale * options.threshold_px / options.focal,
                         .confidence = options.confidence,
                         .max_iterations = options.max_iterations,
                         .seed = options.seed,
                         .stream = planar ? 1U : 0U};
}

/// @brief Run one RANSAC, keeping "no model" as an empty result.
/// @tparam S The run's solution type.
/// @param run The RANSAC's result.
/// @return The solution, nothing if no sample gave a model, or the error otherwise.
template <typename S>
Result<std::optional<S>> found(Result<S> run) {
  if (run) {
    return std::optional<S>(std::move(*run));
  }
  if (run.error().kind == ErrorKind::kDegenerate) {
    return std::optional<S>();
  }
  return std::unexpected(run.error());
}

/// @brief Triangulate a candidate's inliers; count those in front, keep those with parallax.
/// @param pose The candidate.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @param inliers The chosen model's inliers.
/// @param min_parallax The parallax a point needs, in radians.
/// @return The tried candidate, or a triangulation error.
Result<Tried> try_candidate(const Eigen::Matrix4d& pose,
                            const Eigen::Ref<const correspondences::Rows3>& bearings1,
                            const Eigen::Ref<const correspondences::Rows3>& bearings2,
                            const Eigen::Array<bool, Eigen::Dynamic, 1>& inliers,
                            double min_parallax) {
  Tried out{.pose = pose,
            .good = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(inliers.size(), false),
            .points = correspondences::Rows3::Constant(inliers.size(), 3,
                                                       std::numeric_limits<double>::quiet_NaN())};
  out.pose.topRightCorner<3, 1>().normalize();
  ransac::Indices index;
  for (Eigen::Index i = 0; i < inliers.size(); ++i) {
    if (inliers(i)) {
      index.push_back(static_cast<std::uint32_t>(i));
    }
  }
  const auto result = triangulation::two_view(
      out.pose, correspondences::gather(correspondences::Rows3(bearings1), index),
      correspondences::gather(correspondences::Rows3(bearings2), index));
  if (!result) {
    return std::unexpected(result.error());
  }
  for (std::size_t j = 0; j < index.size(); ++j) {
    const auto row = static_cast<Eigen::Index>(j);
    const auto i = static_cast<Eigen::Index>(index[j]);
    if (result->in_front(row)) {
      ++out.in_front;
      if (result->parallax(row) >= min_parallax) {
        out.good(i) = true;
        out.points.row(i) = result->points.row(row);
      }
    }
  }
  return out;
}

/// @brief The chosen model's candidate motions.
/// @param model The model.
/// @param matrix Its matrix.
/// @return The candidates `T_c2_c1`.
Result<std::vector<Eigen::Matrix4d>> candidates(Model model, const Eigen::Matrix3d& matrix) {
  if (model == Model::kEssential) {
    return epipolar::decompose_essential(matrix);
  }
  auto motions = homography::decompose_homography(matrix);
  if (!motions) {
    return std::unexpected(motions.error());
  }
  return std::move(motions->poses);
}

/// @brief The matches in the normalised image plane.
struct Planes {
  /// @brief Camera 1's points `(x, y, 1)`.
  correspondences::Rows3 x1;
  /// @brief Camera 2's points `(x, y, 1)`.
  correspondences::Rows3 x2;
};

/// @brief Both RANSAC runs; at least one found a model.
struct Runs {
  /// @brief E's run, if it found a model.
  std::optional<ransac::RefinedSolution<Eigen::Matrix3d>> essential;
  /// @brief H's run, if it found a model.
  std::optional<ransac::Solution<Eigen::Matrix3d>> planar;
};

/// @brief Run E and H RANSAC on independent streams.
/// @param bearings1 Bearings in camera 1.
/// @param bearings2 Bearings in camera 2.
/// @param planes The matches in the normalised plane.
/// @param options The settings.
/// @return The runs; `Degenerate` if neither found a model.
Result<Runs> run_both(const Eigen::Ref<const correspondences::Rows3>& bearings1,
                      const Eigen::Ref<const correspondences::Rows3>& bearings2,
                      const Planes& planes, const Options& options) {
  auto essential = found(epipolar::essential_ransac(bearings1, bearings2,
                                                    ransac_options(options, Model::kEssential),
                                                    options.essential_solver, options.refine));
  if (!essential) {
    return std::unexpected(essential.error());
  }
  auto planar =
      found(homography::homography_ransac(planes.x1.leftCols<2>(), planes.x2.leftCols<2>(),
                                          ransac_options(options, Model::kHomography)));
  if (!planar) {
    return std::unexpected(planar.error());
  }
  if (!*essential && !*planar) {
    return fail(ErrorKind::kDegenerate, "initialize: neither RANSAC found a model");
  }
  return Runs{.essential = std::move(*essential), .planar = std::move(*planar)};
}

/// @brief Score both models by GRIC over the matches at least one explains, and choose.
/// @param runs Both runs.
/// @param planes The matches in the normalised plane.
/// @param sigma The noise in the normalised plane.
/// @return A result with the scores, summaries and chosen model filled in.
Initialization scored(const Runs& runs, const Planes& planes, double sigma) {
  const Eigen::Index n = planes.x1.rows();
  Eigen::Array<bool, Eigen::Dynamic, 1> support =
      Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(n, false);
  if (runs.essential) {
    support = support || runs.essential->state.inliers;
  }
  if (runs.planar) {
    support = support || runs.planar->state.inliers;
  }
  ransac::Indices explained;
  for (Eigen::Index i = 0; i < n; ++i) {
    if (support(i)) {
      explained.push_back(static_cast<std::uint32_t>(i));
    }
  }
  const correspondences::Rows3 x1 = correspondences::gather(planes.x1, explained);
  const correspondences::Rows3 x2 = correspondences::gather(planes.x2, explained);
  Initialization out;
  out.gric_essential = std::numeric_limits<double>::infinity();
  out.gric_homography = std::numeric_limits<double>::infinity();
  if (runs.essential) {
    out.gric_essential = gric(epipolar::sampson(runs.essential->state.model, x1, x2), sigma,
                              kEssentialDimension, kEssentialParameters);
    out.essential = runs.essential->summary;
  }
  if (runs.planar) {
    out.gric_homography =
        gric(homography::sampson(runs.planar->state.model, x1.leftCols<2>(), x2.leftCols<2>()),
             sigma, kHomographyDimension, kHomographyParameters);
    out.homography = runs.planar->summary;
  }
  // The lower score is finite, so the chosen run exists.
  out.model = out.gric_homography < out.gric_essential ? Model::kHomography : Model::kEssential;
  out.points = correspondences::Rows3::Constant(n, 3, std::numeric_limits<double>::quiet_NaN());
  out.triangulated = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(n, false);
  return out;
}

/// @brief A model's verdict: its status, best candidate and per-candidate counts.
struct Decision {
  /// @brief The status this model reaches.
  Status status = Status::kLowParallax;
  /// @brief The best candidate (the first, for a pure rotation).
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  /// @brief The best candidate, triangulated (none for a pure rotation).
  std::optional<Tried> best;
  /// @brief Each candidate's inliers in front, best first.
  std::vector<int> in_front;
};

/// @brief Try a model's candidate motions on its inliers and judge the best.
/// @param model The model.
/// @param run Its RANSAC consensus.
/// @param rays Both cameras' bearings.
/// @param options The settings.
/// @return The verdict, or a decomposition or triangulation error.
Result<Decision> decide(Model model, const ransac::Consensus<Eigen::Matrix3d>& run,
                        const std::pair<Eigen::Ref<const correspondences::Rows3>,
                                        Eigen::Ref<const correspondences::Rows3>>& rays,
                        const Options& options) {
  const auto poses = candidates(model, run.model);
  if (!poses) {
    return std::unexpected(poses.error());
  }
  Decision out{.pose = poses->front(), .best = std::nullopt, .in_front = {}};
  if (!(out.pose.topRightCorner<3, 1>().norm() > 0.0)) {
    out.status = Status::kPureRotation;
    return out;
  }
  const double min_parallax = options.min_parallax_deg * std::numbers::pi / 180.0;
  std::vector<Tried> tried;
  for (const Eigen::Matrix4d& pose : *poses) {
    auto attempt = try_candidate(pose, rays.first, rays.second, run.inliers, min_parallax);
    if (!attempt) {
      return std::unexpected(attempt.error());
    }
    tried.push_back(std::move(*attempt));
  }
  std::ranges::stable_sort(tried,
                           [](const Tried& a, const Tried& b) { return a.in_front > b.in_front; });
  for (const Tried& t : tried) {
    out.in_front.push_back(t.in_front);
  }
  const Tried& best = tried.front();
  if (best.good.count() < options.min_points) {
    out.status = Status::kLowParallax;
  } else if (tried.size() > 1 && tried[1].in_front > options.ambiguity_ratio * best.in_front) {
    out.status = Status::kAmbiguous;
  } else {
    out.status = Status::kInitialized;
  }
  out.pose = best.pose;
  out.best = best;
  return out;
}

}  // namespace

const char* name(Model model) {
  switch (model) {
    case Model::kEssential:
      return "essential";
    case Model::kHomography:
      return "homography";
    case Model::kCount:
      break;
  }
  return "unknown";
}

const char* name(Status status) {
  switch (status) {
    case Status::kInitialized:
      return "initialized";
    case Status::kPureRotation:
      return "pure_rotation";
    case Status::kAmbiguous:
      return "ambiguous";
    case Status::kLowParallax:
      return "low_parallax";
    case Status::kCount:
      break;
  }
  return "unknown";
}

Result<void> check_options(const Options& options) {
  for (const auto& [name, value] :
       {std::pair{"focal", options.focal}, std::pair{"threshold_px", options.threshold_px},
        std::pair{"sigma_px", options.sigma_px}}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return fail_setting(name, "must be positive and finite", value);
    }
  }
  if (!std::isfinite(options.min_parallax_deg) || options.min_parallax_deg < 0.0) {
    return fail_setting("min_parallax_deg", "must be non-negative", options.min_parallax_deg);
  }
  if (options.min_points < 1) {
    return fail_setting("min_points", "must be at least 1", options.min_points);
  }
  if (std::isnan(options.ambiguity_ratio) || options.ambiguity_ratio <= 0.0 ||
      options.ambiguity_ratio > 1.0) {
    return fail_setting("ambiguity_ratio", "must be in (0, 1]", options.ambiguity_ratio);
  }
  if (options.essential_solver >= epipolar::MinimalSolver::kCount) {
    return fail(ErrorKind::kInvalidInput, "essential_solver: unknown solver");
  }
  return {};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (d, k), as in Torr's formula
double gric(const Eigen::VectorXd& squared_errors, double sigma, int dimension, int parameters) {
  const auto n = static_cast<double>(squared_errors.size());
  const double cap = kGricLambda3 * (kDataDimension - dimension);
  const double fit = (squared_errors / (sigma * sigma)).cwiseMin(cap).sum();
  return fit + (std::log(static_cast<double>(kDataDimension)) * dimension * n) +
         (std::log(kDataDimension * n) * parameters);
}

Result<Initialization> initialize(const Eigen::Ref<const correspondences::Rows3>& bearings1,
                                  const Eigen::Ref<const correspondences::Rows3>& bearings2,
                                  const Options& options) {
  if (auto ok = check_options(options); !ok) {
    return std::unexpected(ok.error());
  }
  if (auto ok = correspondences::check_bearing_pairs(bearings1, bearings2, epipolar::kSampleSize);
      !ok) {
    return std::unexpected(ok.error());
  }
  const Planes planes{.x1 = correspondences::normalised_plane(bearings1),
                      .x2 = correspondences::normalised_plane(bearings2)};
  auto runs = run_both(bearings1, bearings2, planes, options);
  if (!runs) {
    return std::unexpected(runs.error());
  }
  Initialization out = scored(*runs, planes, options.sigma_px / options.focal);
  const ransac::Consensus<Eigen::Matrix3d>* run = nullptr;
  if (out.model == Model::kHomography && runs->planar) {
    run = &runs->planar->state;
  } else if (out.model == Model::kEssential && runs->essential) {
    run = &runs->essential->state;
  }
  if (run == nullptr) {
    // Unreachable: the chosen model's score is finite only when its run found a model.
    return fail(ErrorKind::kDegenerate, "initialize: the chosen model has no run");
  }
  const std::pair<Eigen::Ref<const correspondences::Rows3>,
                  Eigen::Ref<const correspondences::Rows3>>
      rays{bearings1, bearings2};
  auto decision = decide(out.model, *run, rays, options);
  if (!decision) {
    return std::unexpected(decision.error());
  }
  out.inliers = run->inliers;
  if (decision->status == Status::kAmbiguous && out.model == Model::kHomography &&
      runs->essential.has_value()) {
    auto alternative = decide(Model::kEssential, runs->essential->state, rays, options);
    if (!alternative) {
      return std::unexpected(alternative.error());
    }
    if (alternative->status == Status::kInitialized) {
      out.model = Model::kEssential;
      out.fallback = true;
      out.inliers = runs->essential->state.inliers;
      decision = std::move(alternative);
    }
  }
  out.status = decision->status;
  out.relative_pose = decision->pose;
  out.in_front = decision->in_front;
  if (decision->best) {
    out.points = decision->best->points;
    out.triangulated = decision->best->good;
  }
  return out;
}

}  // namespace slam_it_all::foundations::twoview
