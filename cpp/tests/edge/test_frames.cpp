/// @file
/// @brief Native tests for building frames (slamlab/edge/frames.hpp): one frame per mode on a
///        textured image, and the builders' refusals. Parity with the Python reference is
///        checked through the whole-run tests (tests/orchestration).
#include <cmath>
#include <cstdint>
#include <vector>

#include "../foundations/test_support.hpp"
#include "slamlab/edge/frames.hpp"

namespace {

namespace edge = slamlab::edge;
namespace foundations = slamlab::foundations;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;

/// @brief The image's width.
constexpr std::int64_t kCols = 320;

/// @brief Its height.
constexpr std::int64_t kRows = 240;

/// @brief The stereo pair's disparity, in pixels.
constexpr std::int64_t kDisparity = 5;

/// @brief A textured image: 4-pixel blocks of hashed intensities, shifted left by `shift`.
/// @param shift Columns to shift by.
/// @return The image.
foundations::image::Image textured(std::int64_t shift) {
  foundations::image::Image image{
      .rows = kRows, .cols = kCols, .pixels = std::vector<std::uint8_t>(kRows * kCols)};
  for (std::int64_t r = 0; r < kRows; ++r) {
    for (std::int64_t c = 0; c < kCols; ++c) {
      const auto x = static_cast<std::uint32_t>((c + shift) / 4);
      const auto y = static_cast<std::uint32_t>(r / 4);
      const std::uint32_t hash = ((x * 73856093U) ^ (y * 19349663U)) * 2654435761U;
      image.pixels[static_cast<std::size_t>((r * kCols) + c)] =
          static_cast<std::uint8_t>(hash >> 24U);
    }
  }
  return image;
}

/// @brief A pinhole camera for the image.
/// @param baseline Its (virtual) baseline.
/// @return The camera.
edge::Camera camera(double baseline) {
  return {.model = foundations::camera::Model::kPinhole,
          .intrinsics = Eigen::Vector4d(250.0, 250.0, 160.0, 120.0),
          .width = kCols,
          .height = kRows,
          .baseline = baseline};
}

/// @brief Check one frame per mode.
/// @param failures Running failure count.
void check_builds(int& failures) {
  const foundations::image::Image image = textured(0);
  const foundations::orb::Options orb{};
  const auto mono = edge::monocular(image, 1.5, camera(0.0), orb);
  expect(mono.has_value() && mono->timestamp == 1.5 && edge::keypoint_count(*mono) > 100 &&
             std::isnan(mono->depth[0]) && std::isnan(mono->right[0]) &&
             std::abs(mono->bearings[0].norm() - 1.0) < 1e-12,
         "a monocular frame has features and bearings, no depth", failures);
  const edge::DepthMap depth{
      .rows = kRows, .cols = kCols, .values = std::vector<double>(kRows * kCols, 2.0)};
  const auto rgbd = edge::rgbd(image, depth, 0.0, camera(0.1), orb);
  bool every = rgbd.has_value() && edge::keypoint_count(*rgbd) > 100;
  for (std::size_t i = 0; every && i < edge::keypoint_count(*rgbd); ++i) {
    const double u = rgbd->features.keypoints[i].xy[0];
    every = rgbd->depth[i] == 2.0 && std::abs(rgbd->right[i] - (u - (250.0 * 0.1 / 2.0))) < 1e-12;
  }
  expect(every, "an RGB-D frame reads each keypoint's depth and its virtual right column",
         failures);
  const auto stereo = edge::stereo_pair(image, textured(kDisparity), 0.0, camera(0.1), orb, {});
  std::size_t matched = 0;
  std::size_t right = 0;
  for (std::size_t i = 0; stereo && i < edge::keypoint_count(*stereo); ++i) {
    if (!std::isnan(stereo->depth[i])) {
      ++matched;
      right += std::abs(stereo->depth[i] - (250.0 * 0.1 / kDisparity)) < 0.25 ? 1 : 0;
    }
  }
  expect(stereo.has_value() && matched > 100 && right * 2 > matched,
         "a stereo frame matches along rows, most within 5% of the pair's depth", failures);
}

/// @brief Check the refusals.
/// @param failures Running failure count.
void check_refusals(int& failures) {
  const foundations::image::Image image = textured(0);
  const foundations::orb::Options orb{};
  const edge::DepthMap small{
      .rows = kRows / 2, .cols = kCols / 2, .values = std::vector<double>(kRows * kCols / 4, 2.0)};
  expect(is_invalid_input(edge::rgbd(image, small, 0.0, camera(0.1), orb)),
         "rgbd refuses a depth map of another shape", failures);
  const edge::DepthMap depth{
      .rows = kRows, .cols = kCols, .values = std::vector<double>(kRows * kCols, 2.0)};
  expect(is_invalid_input(edge::rgbd(image, depth, 0.0, camera(0.0), orb)),
         "rgbd refuses a camera without a virtual baseline", failures);
  expect(is_invalid_input(edge::stereo_pair(image, image, 0.0, camera(0.0), orb, {})),
         "stereo_pair refuses a camera without a baseline", failures);
  edge::Camera distorted = camera(0.1);
  distorted.model = foundations::camera::Model::kRadTan;
  distorted.intrinsics = Eigen::VectorXd::Zero(9);
  distorted.intrinsics.head<4>() << 250.0, 250.0, 160.0, 120.0;
  expect(is_invalid_input(edge::stereo_pair(image, image, 0.0, distorted, orb, {})),
         "stereo_pair refuses a camera that is not pinhole", failures);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  check_builds(failures);
  check_refusals(failures);
  return slamlab::testing::finish("edge/frames", failures);
}

}  // namespace

int main() { return run(); }
