/// @file
/// @brief Native tests for stereo rectification (slamlab/foundations/stereo.hpp), against the
///        parity fixture `tests/fixtures/foundations/stereo.json` that the Python and Rust tests
///        share, and the core's own input checks.
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "fixture_support.hpp"
#include "image_support.hpp"
#include "slamlab/foundations/camera.hpp"
#include "slamlab/foundations/orb.hpp"
#include "slamlab/foundations/stereo.hpp"
#include "test_support.hpp"

namespace {

namespace camera = slamlab::foundations::camera;
namespace stereo = slamlab::foundations::stereo;
using slamlab::foundations::ErrorKind;
using slamlab::testing::expect;

/// @brief A model from its name in the fixture.
/// @param name The name.
/// @return The model.
camera::Model model(const std::string& name) {
  if (name == "radtan") {
    return camera::Model::kRadTan;
  }
  return name == "radial" ? camera::Model::kRadial : camera::Model::kPinhole;
}

/// @brief Read a row-major matrix from a list of rows.
/// @param rows The rows.
/// @return The matrix.
Eigen::MatrixXd read_matrix(const nlohmann::json& rows) {
  Eigen::MatrixXd out(rows.size(), rows[0].size());
  for (Eigen::Index r = 0; r < out.rows(); ++r) {
    for (Eigen::Index c = 0; c < out.cols(); ++c) {
      out(r, c) = rows[r][c].get<double>();
    }
  }
  return out;
}

/// @brief Whether two matrices agree within a tolerance, and are finite.
/// @param a A matrix.
/// @param b Another.
/// @param tolerance The largest absolute difference.
/// @return True if they agree.
bool close(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double tolerance) {
  return a.rows() == b.rows() && a.cols() == b.cols() && a.allFinite() &&
         (a - b).cwiseAbs().maxCoeff() <= tolerance;
}

/// @brief Check every fixture case.
/// @param failures Running failure count.
void check_fixture(int& failures) {
  const nlohmann::json functions =
      slamlab::testing::load_fixture("foundations/stereo.json")["functions"];
  const double rectify_tolerance = functions["rectify"]["tolerance"]["atol"].get<double>();
  for (const auto& c : functions["rectify"]["cases"]) {
    const auto& in = c["input"];
    const auto left_k = in[1].get<std::vector<double>>();
    const auto right_k = in[3].get<std::vector<double>>();
    const Eigen::Matrix4d transform = read_matrix(in[4]);
    const auto got = stereo::rectify(
        model(in[0].get<std::string>()),
        Eigen::Map<const Eigen::VectorXd>(left_k.data(), static_cast<Eigen::Index>(left_k.size())),
        model(in[2].get<std::string>()),
        Eigen::Map<const Eigen::VectorXd>(right_k.data(),
                                          static_cast<Eigen::Index>(right_k.size())),
        transform, in[5][0].get<std::int64_t>(), in[5][1].get<std::int64_t>());
    const auto& e = c["expected"];
    const auto intrinsics = e["intrinsics"].get<std::vector<double>>();
    expect(got.has_value() &&
               close(got->rotation_left, read_matrix(e["rotation_left"]), rectify_tolerance) &&
               close(got->rotation_right, read_matrix(e["rotation_right"]), rectify_tolerance) &&
               close(got->intrinsics, Eigen::Map<const Eigen::Vector4d>(intrinsics.data()),
                     rectify_tolerance) &&
               std::abs(got->baseline - e["baseline"].get<double>()) <= rectify_tolerance,
           "rectify matches the fixture", failures);
  }
  const double map_tolerance = functions["rectification_map"]["tolerance"]["atol"].get<double>();
  for (const auto& c : functions["rectification_map"]["cases"]) {
    const auto& fields = c["input"][2];
    const auto rows = fields["shape"][0].get<std::int64_t>();
    const auto cols = fields["shape"][1].get<std::int64_t>();
    const auto pinhole = fields["intrinsics"].get<std::vector<double>>();
    for (const int side : {0, 1}) {
      const auto& camera_json = c["input"][side];
      const auto k = camera_json[1].get<std::vector<double>>();
      const auto got = stereo::rectification_map(
          model(camera_json[0].get<std::string>()),
          Eigen::Map<const Eigen::VectorXd>(k.data(), static_cast<Eigen::Index>(k.size())),
          read_matrix(fields[side == 0 ? "rotation_left" : "rotation_right"]),
          Eigen::Map<const Eigen::Vector4d>(pinhole.data()), rows, cols);
      const auto expected =
          slamlab::testing::flatten_rows<std::vector<double>>(c["expected"][side]);
      bool ok = got.has_value() && std::cmp_equal(got->rows(), expected.size());
      for (Eigen::Index i = 0; ok && i < got->rows(); ++i) {
        const auto& want = expected[static_cast<std::size_t>(i)];
        ok = std::abs((*got)(i, 0) - want[0]) <= map_tolerance &&
             std::abs((*got)(i, 1) - want[1]) <= map_tolerance;
      }
      expect(ok, "rectification_map matches the fixture", failures);
    }
  }
}

/// @brief Read a match case's features (responses and angles, unused, are zero).
/// @param fields The dict of `xy`, `level`, `level_xy` and `descriptors`.
/// @return The features.
slamlab::foundations::orb::Features read_features(const nlohmann::json& fields) {
  namespace orb = slamlab::foundations::orb;
  orb::Features out;
  for (std::size_t i = 0; i < fields["level"].size(); ++i) {
    out.keypoints.push_back({.xy = fields["xy"][i].get<std::array<double, 2>>(),
                             .level = fields["level"][i].get<std::int64_t>(),
                             .level_xy = fields["level_xy"][i].get<std::array<std::int64_t, 2>>(),
                             .response = 0,
                             .angle = 0});
    out.descriptors.push_back(fields["descriptors"][i].get<orb::Descriptor>());
  }
  return out;
}

/// @brief Check every match case, exactly.
/// @param failures Running failure count.
void check_matches(int& failures) {
  const nlohmann::json cases =
      slamlab::testing::load_fixture("foundations/stereo.json")["functions"]["match"]["cases"];
  for (const auto& c : cases) {
    const auto& in = c["input"];
    const auto& o = in[6];
    const auto& s = in[7];
    const slamlab::foundations::orb::Options extraction{
        .n_features = o["n_features"].get<std::int64_t>(),
        .levels = o["levels"].get<std::int64_t>(),
        .scale_factor = o["scale_factor"].get<double>(),
        .fast_threshold = o["fast_threshold"].get<std::int64_t>(),
        .min_fast_threshold = o["min_fast_threshold"].get<std::int64_t>()};
    const stereo::MatchOptions settings{.max_distance = s["max_distance"].get<std::int64_t>(),
                                        .band = s["band"].get<double>(),
                                        .window = s["window"].get<std::int64_t>(),
                                        .search = s["search"].get<std::int64_t>(),
                                        .outlier_factor = s["outlier_factor"].get<double>()};
    const auto got =
        stereo::match(slamlab::testing::read_image(in[0]), slamlab::testing::read_image(in[1]),
                      read_features(in[2]), read_features(in[3]), in[4].get<double>(),
                      in[5].get<double>(), extraction, settings);
    const auto& e = c["expected"];
    bool ok = got.has_value() && got->size() == e["left"].size();
    for (std::size_t k = 0; ok && k < got->size(); ++k) {
      const stereo::StereoMatch& m = (*got)[k];
      ok = m.left == e["left"][k].get<std::int64_t>() &&
           m.right == e["right"][k].get<std::int64_t>() &&
           m.right_x == e["right_x"][k].get<double>() &&
           m.disparity == e["disparity"][k].get<double>() && m.depth == e["depth"][k].get<double>();
    }
    expect(ok, "match matches the fixture", failures);
  }
}

/// @brief Check that vertical, coincident and malformed pairs are refused.
/// @param failures Running failure count.
void check_rejections(int& failures) {
  const Eigen::Vector4d k(500.0, 500.0, 320.0, 240.0);
  Eigen::Matrix4d vertical = Eigen::Matrix4d::Identity();
  vertical(1, 3) = 0.1;
  const auto up =
      stereo::rectify(camera::Model::kPinhole, k, camera::Model::kPinhole, k, vertical, 480, 640);
  expect(!up && up.error().kind == ErrorKind::kInvalidInput, "rectify rejects a vertical pair",
         failures);
  const auto same = stereo::rectify(camera::Model::kPinhole, k, camera::Model::kPinhole, k,
                                    Eigen::Matrix4d::Identity(), 480, 640);
  expect(!same && same.error().kind == ErrorKind::kDegenerate, "rectify rejects coincident centres",
         failures);
  Eigen::Matrix4d side = Eigen::Matrix4d::Identity();
  side(0, 3) = -0.1;
  const auto flat =
      stereo::rectify(camera::Model::kPinhole, k, camera::Model::kPinhole, k, side, 0, 640);
  expect(!flat && flat.error().kind == ErrorKind::kInvalidInput, "rectify rejects an empty shape",
         failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_fixture(failures);
  check_matches(failures);
  check_rejections(failures);
  return slamlab::testing::finish("foundations/stereo", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("foundations/stereo", run); }
