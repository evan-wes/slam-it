/// @file
/// @brief Bindings for `slamlab.foundations.twoview` (`slamlab._cpp.foundations.twoview`).
#include "slamlab/foundations/twoview.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "common.hpp"
#include "foundations/foundations.hpp"
#include "foundations/ransac_crossing.hpp"

namespace nb = nanobind;

namespace slamlab::bindings {

namespace {

namespace twoview = foundations::twoview;

/// @brief `twoview.Options`' fields, as the dict's keys.
constexpr std::array<const char*, 11> kOptionKeys = {
    "focal", "threshold_px",     "sigma_px",   "confidence",      "max_iterations",
    "seed",  "min_parallax_deg", "min_points", "ambiguity_ratio", "essential_solver",
    "refine"};

/// @brief Look a minimal solver up by the name `epipolar.MINIMAL_SOLVERS` spells it.
/// @param name The name.
/// @return The solver.
/// @throws std::invalid_argument (Python `ValueError`) for an unknown name.
foundations::epipolar::MinimalSolver essential_solver(const std::string& name) {
  using foundations::epipolar::MinimalSolver;
  for (int k = 0; k < static_cast<int>(MinimalSolver::kCount); ++k) {
    if (name == foundations::epipolar::name(static_cast<MinimalSolver>(k))) {
      return static_cast<MinimalSolver>(k);
    }
  }
  throw std::invalid_argument("essential_solver: unknown solver " + name);
}

/// @brief Read the initialiser's options from a dict of scalars.
/// @param options The dict, with exactly `kOptionKeys`.
/// @return The options (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key.
twoview::Options read_options(const nb::dict& options) {
  check_keys(options, kOptionKeys, "twoview.Options");
  return twoview::Options{
      .focal = nb::cast<double>(options["focal"]),
      .threshold_px = nb::cast<double>(options["threshold_px"]),
      .sigma_px = nb::cast<double>(options["sigma_px"]),
      .confidence = nb::cast<double>(options["confidence"]),
      .max_iterations = nb::cast<int>(options["max_iterations"]),
      .seed = nb::cast<std::uint64_t>(options["seed"]),
      .min_parallax_deg = nb::cast<double>(options["min_parallax_deg"]),
      .min_points = nb::cast<int>(options["min_points"]),
      .ambiguity_ratio = nb::cast<double>(options["ambiguity_ratio"]),
      .essential_solver = essential_solver(nb::cast<std::string>(options["essential_solver"])),
      .refine = nb::cast<bool>(options["refine"])};
}

/// @brief A RANSAC summary, or None if that run found no model.
/// @param summary The summary.
/// @return A dict (see `ransac_summary`) or None.
nb::object maybe_summary(const std::optional<foundations::ransac::Summary>& summary) {
  return summary ? nb::object(ransac_summary(*summary)) : nb::none();
}

/// @brief Initialise two views.
/// @param bearings1 (N, 3) bearings in camera 1.
/// @param bearings2 (N, 3) bearings in camera 2.
/// @param options `twoview.Options`' fields.
/// @return The result as a dict.
/// @throws CoreError for invalid input, or if neither RANSAC found a model.
nb::dict initialize(const BatchIn<3>& bearings1, const BatchIn<3>& bearings2,
                    const nb::dict& options) {
  using Rows3 = foundations::epipolar::Rows3;
  const twoview::Initialization result = unwrap(twoview::initialize(
      copy_matrix<Rows3>(bearings1.data(), bearings1.shape(0), 3),
      copy_matrix<Rows3>(bearings2.data(), bearings2.shape(0), 3), read_options(options)));
  nb::dict out;
  out["status"] = static_cast<int>(result.status);
  out["model"] = static_cast<int>(result.model);
  out["fallback"] = result.fallback;
  auto [pose, pose_data] = new_array({4, 4});
  Eigen::Map<Item<4, 4>> pose_view(pose_data);
  pose_view = result.relative_pose;
  out["relative_pose"] = pose;
  out["inliers"] = mask_array(result.inliers);
  out["points"] = matrix_array(result.points);
  out["triangulated"] = mask_array(result.triangulated);
  out["gric_essential"] = result.gric_essential;
  out["gric_homography"] = result.gric_homography;
  auto [in_front, counts] = new_array<std::int64_t>({result.in_front.size()});
  for (std::size_t i = 0; i < result.in_front.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    counts[i] = result.in_front[i];
  }
  out["in_front"] = in_front;
  out["essential"] = maybe_summary(result.essential);
  out["homography"] = maybe_summary(result.homography);
  return out;
}

/// @brief The enum tables: status, model and RANSAC termination names in code order.
/// @return The tables.
nb::dict codes() {
  nb::dict out = ransac_codes();
  nb::list statuses;
  for (int k = 0; k < static_cast<int>(twoview::Status::kCount); ++k) {
    statuses.append(twoview::name(static_cast<twoview::Status>(k)));
  }
  nb::list models;
  for (int k = 0; k < static_cast<int>(twoview::Model::kCount); ++k) {
    models.append(twoview::name(static_cast<twoview::Model>(k)));
  }
  out["status"] = statuses;
  out["model"] = models;
  return out;
}

}  // namespace

void bind_twoview(nb::module_& m) {
  m.def("codes", &codes, "Each enum's names in code order.");
  m.def("initialize", &initialize, nb::arg("bearings1").noconvert(),
        nb::arg("bearings2").noconvert(), nb::arg("options"),
        "Initialise two views from (N, 3) bearing pairs.");
}

}  // namespace slamlab::bindings
