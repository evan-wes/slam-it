/// @file
/// @brief Bindings for `slamlab.foundations.twoview` (`slamlab._cpp.foundations.twoview`).
#include "slamlab/foundations/twoview.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

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
constexpr std::array<const char*, 9> kOptionKeys = {
    "focal", "threshold_px",     "sigma_px",   "confidence",     "max_iterations",
    "seed",  "min_parallax_deg", "min_points", "ambiguity_ratio"};

/// @brief Read the initialiser's options from a dict of scalars.
/// @param options The dict, with exactly `kOptionKeys`.
/// @return The options (checked by the core).
/// @throws std::invalid_argument for a missing or unknown key.
twoview::Options read_options(const nb::dict& options) {
  if (options.size() != kOptionKeys.size()) {
    throw std::invalid_argument("options: expected exactly twoview.Options' fields");
  }
  for (const char* key : kOptionKeys) {
    if (!options.contains(key)) {
      throw std::invalid_argument(std::string("options: missing ") + key);
    }
  }
  return twoview::Options{.focal = nb::cast<double>(options["focal"]),
                          .threshold_px = nb::cast<double>(options["threshold_px"]),
                          .sigma_px = nb::cast<double>(options["sigma_px"]),
                          .confidence = nb::cast<double>(options["confidence"]),
                          .max_iterations = nb::cast<int>(options["max_iterations"]),
                          .seed = nb::cast<std::uint64_t>(options["seed"]),
                          .min_parallax_deg = nb::cast<double>(options["min_parallax_deg"]),
                          .min_points = nb::cast<int>(options["min_points"]),
                          .ambiguity_ratio = nb::cast<double>(options["ambiguity_ratio"])};
}

/// @brief Copy a boolean array out.
/// @param mask The mask.
/// @return A new (N,) bool array.
nb::ndarray<nb::numpy, bool> mask_array(const Eigen::Array<bool, Eigen::Dynamic, 1>& mask) {
  auto [out, data] = new_array<bool>({static_cast<std::size_t>(mask.size())});
  for (Eigen::Index i = 0; i < mask.size(); ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): array fill
    data[i] = mask(i);
  }
  return out;
}

/// @brief Flatten a RANSAC summary into a dict with a prefix.
/// @param out The dict.
/// @param prefix The prefix.
/// @param summary The summary, if the run found a model.
void put_summary(nb::dict& out, const std::string& prefix,
                 const std::optional<foundations::ransac::Summary>& summary) {
  out[(prefix + "found").c_str()] = summary.has_value();
  const foundations::ransac::Summary s = summary.value_or(foundations::ransac::Summary{});
  out[(prefix + "iterations").c_str()] = s.iterations;
  out[(prefix + "termination").c_str()] = static_cast<int>(s.termination);
  out[(prefix + "degenerate_samples").c_str()] = s.degenerate_samples;
  out[(prefix + "local_optimizations").c_str()] = s.local_optimizations;
  out[(prefix + "inlier_count").c_str()] = s.inlier_count;
  out[(prefix + "score").c_str()] = s.score;
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
  nb::list in_front;
  for (const int count : result.in_front) {
    in_front.append(count);
  }
  out["in_front"] = in_front;
  put_summary(out, "essential_", result.essential);
  put_summary(out, "homography_", result.homography);
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
