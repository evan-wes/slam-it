/// @file
/// @brief Two-view initialisation: the C++ core's E and H RANSAC, GRIC model selection, and
///        the choice of motion by triangulation.
///
/// Mirrors `slam_it.foundations.twoview`, whose docstring states the method: E (normalised
/// 8-point) and H (normalised DLT) by LO-MSAC on independent random streams, Torr's GRIC over
/// the matches either model explains, the winner's decomposition, each candidate tried by
/// triangulating the inliers, and E deciding when H's candidates are ambiguous. The outcome is a
/// status, not an error.
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "slam_it/foundations/correspondences.hpp"
#include "slam_it/foundations/epipolar.hpp"
#include "slam_it/foundations/error.hpp"
#include "slam_it/foundations/ransac.hpp"

/// @brief Two-view initialisation.
namespace slam_it::foundations::twoview {

/// @brief Torr's lambda_3: a residual's contribution is capped at lambda_3 (r - d).
inline constexpr double kGricLambda3 = 2.0;
/// @brief r: a correspondence is two image points.
inline constexpr int kDataDimension = 4;
/// @brief E's variety dimension in the data space.
inline constexpr int kEssentialDimension = 3;
/// @brief E's degrees of freedom.
inline constexpr int kEssentialParameters = 5;
/// @brief H's variety dimension in the data space.
inline constexpr int kHomographyDimension = 2;
/// @brief H's degrees of freedom.
inline constexpr int kHomographyParameters = 8;
/// @brief H's RANSAC threshold over E's: sqrt(5.991 / 3.841), the ratio of the 95% quantiles of
///        chi-squared with two degrees of freedom and with one (H's residual has two).
inline const double kHomographyThresholdScale = std::sqrt(5.991 / 3.841);

/// @brief Which model GRIC chose, with the codes it crosses the bindings as.
enum class Model : std::uint8_t {
  /// @brief The essential matrix: a general scene.
  kEssential = 0,
  /// @brief A homography: a plane, or a rotation.
  kHomography = 1,
  /// @brief The number of models (a sentinel).
  kCount = 2,
};

/// @brief Whether the two views initialise, and if not, why.
enum class Status : std::uint8_t {
  /// @brief One motion is clearly best, with enough well-triangulated points.
  kInitialized = 0,
  /// @brief The camera only rotated: no translation, nothing to triangulate.
  kPureRotation = 1,
  /// @brief Two candidate motions put similar numbers of points in front.
  kAmbiguous = 2,
  /// @brief Too few points have enough parallax for a reliable depth.
  kLowParallax = 3,
  /// @brief The number of statuses (a sentinel).
  kCount = 4,
};

/// @brief The model's name, as `slam_it.foundations.twoview.Model` spells it.
/// @param model The model.
/// @return A static string.
[[nodiscard]] const char* name(Model model);

/// @brief The status's name, as `slam_it.foundations.twoview.Status` spells it.
/// @param status The status.
/// @return A static string.
[[nodiscard]] const char* name(Status status);

/// @brief The initialiser's settings (see `slam_it.foundations.twoview.Options`).
struct Options {
  /// @brief The focal length in pixels.
  double focal = 1.0;
  /// @brief E-RANSAC's inlier threshold (a Sampson distance), in pixels; H's is scaled by
  ///        `kHomographyThresholdScale`.
  double threshold_px = 2.0;
  /// @brief The image noise's standard deviation for GRIC, in pixels.
  double sigma_px = 1.0;
  /// @brief RANSAC's target confidence.
  double confidence = 0.999;
  /// @brief RANSAC's sample limit, per model.
  int max_iterations = 10000;
  /// @brief The random seed; E uses stream 0 and H stream 1.
  std::uint64_t seed = 0;
  /// @brief The parallax a triangulated point needs to count, in degrees.
  double min_parallax_deg = 1.0;
  /// @brief The well-triangulated points initialisation needs.
  int min_points = 50;
  /// @brief The runner-up must have fewer than this fraction of the best's points in front.
  double ambiguity_ratio = 0.75;
  /// @brief E-RANSAC's minimal solver.
  epipolar::MinimalSolver essential_solver = epipolar::MinimalSolver::kStewenius;
  /// @brief Whether E-RANSAC's model is refined on its inliers' Sampson distances.
  bool refine = true;
};

/// @brief Check the settings.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` naming the setting out of range.
[[nodiscard]] Result<void> check_options(const Options& options);

/// @brief Torr's GRIC score of a model's residuals.
/// @param squared_errors Each datum's squared (Sampson) distance to the model.
/// @param sigma The noise's standard deviation, in the same units.
/// @param dimension The model's dimension in the 4D data space.
/// @param parameters The model's degrees of freedom.
/// @return `sum min(e^2 / sigma^2, lambda_3 (r - d)) + ln(r) d n + ln(r n) k`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (d, k), as in Torr's formula
[[nodiscard]] double gric(const Eigen::VectorXd& squared_errors, double sigma, int dimension,
                          int parameters);

/// @brief The initialiser's result (see `slam_it.foundations.twoview.Initialization`).
struct Initialization {
  /// @brief Whether it initialised, and if not, why.
  Status status = Status::kLowParallax;
  /// @brief The model that decided: GRIC's choice, or E when it fell back.
  Model model = Model::kEssential;
  /// @brief Whether H won GRIC but its candidates were ambiguous, and E decided instead.
  bool fallback = false;
  /// @brief The best candidate `T_c2_c1`, unit translation (zero for a pure rotation).
  Eigen::Matrix4d relative_pose = Eigen::Matrix4d::Identity();
  /// @brief The chosen model's RANSAC inliers.
  Eigen::Array<bool, Eigen::Dynamic, 1> inliers;
  /// @brief The well-triangulated points in camera 1's frame; NaN elsewhere (N x 3).
  correspondences::Rows3 points;
  /// @brief Which matches have a point.
  Eigen::Array<bool, Eigen::Dynamic, 1> triangulated;
  /// @brief E's GRIC score (infinity if its RANSAC found no model).
  double gric_essential = 0.0;
  /// @brief H's GRIC score (infinity if its RANSAC found no model).
  double gric_homography = 0.0;
  /// @brief Each candidate's inliers in front of both cameras, best first.
  std::vector<int> in_front;
  /// @brief E-RANSAC's summary (with the refinement's), if it found a model.
  std::optional<ransac::RefinedSummary> essential;
  /// @brief H's RANSAC summary, if it found a model.
  std::optional<ransac::Summary> homography;
};

/// @brief Find the relative pose and first points of two calibrated views, or say why not.
/// @param bearings1 Unit bearings in camera 1 (N x 3), positive z, N >= 8.
/// @param bearings2 Unit bearings in camera 2 (N x 3), positive z.
/// @param options The settings.
/// @return The result; `InvalidInput` for bad bearings or settings; `Degenerate` if neither
///         RANSAC found a model.
[[nodiscard]] Result<Initialization> initialize(
    const Eigen::Ref<const correspondences::Rows3>& bearings1,
    const Eigen::Ref<const correspondences::Rows3>& bearings2, const Options& options);

}  // namespace slam_it::foundations::twoview
