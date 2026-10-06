/// @file
/// @brief Native tests for the sequence readers (slamlab/eval/{tum,euroc}.hpp) against the
///        shared fixtures `tests/fixtures/eval/sequences/` and the Python readers' frames,
///        `sequences.json`; and their refusals.
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slamlab/eval/euroc.hpp"
#include "slamlab/eval/tum.hpp"

namespace {

namespace euroc = slamlab::eval::euroc;
namespace tum = slamlab::eval::tum;
namespace fs = std::filesystem;
using slamlab::testing::expect;
using slamlab::testing::is_invalid_input;

/// @brief A fixture sequence's folder.
/// @param name Its name.
/// @return The path.
std::string folder(const std::string& name) {
  return std::string(SLAMLAB_FIXTURES_DIR) + "/eval/sequences/" + name;
}

/// @brief A path relative to a folder, as the fixtures list them.
/// @param path The path.
/// @param root The folder.
/// @return The relative path.
std::string relative(const std::string& path, const std::string& root) {
  return fs::path(path).lexically_relative(root).generic_string();
}

/// @brief Whether numbers equal a JSON list exactly.
/// @param got The numbers.
/// @param expected The list.
/// @return Whether they do.
template <typename Values>
bool same(const Values& got, const nlohmann::json& expected) {
  if (static_cast<std::size_t>(got.size()) != expected.size()) {
    return false;
  }
  for (std::size_t k = 0; k < expected.size(); ++k) {
    if (got[static_cast<Eigen::Index>(k)] != expected[k].get<double>()) {
      return false;
    }
  }
  return true;
}

/// @brief Whether a frame's depth map is the expected one.
/// @param got The frame's depth map, if any.
/// @param expected Its expected path relative to the folder, or null.
/// @param root The folder.
/// @return Whether they agree.
bool same_depth(const std::optional<std::string>& got, const nlohmann::json& expected,
                const std::string& root) {
  if (!got) {
    return expected.is_null();
  }
  return !expected.is_null() && relative(*got, root) == expected.get<std::string>();
}

/// @brief Check the TUM folders against the Python reader's frames.
/// @param cases The expected frames, by folder.
/// @param failures Running failure count.
void check_tum(const nlohmann::json& cases, int& failures) {
  for (const auto& [name, expected] : cases.items()) {
    const auto frames = tum::read_frames(folder(name));
    bool ok = frames.has_value() && frames->timestamps.size() == expected["timestamps"].size();
    for (std::size_t k = 0; ok && k < frames->timestamps.size(); ++k) {
      ok = frames->timestamps[k] == expected["timestamps"][k].get<double>() &&
           relative(frames->images[k], frames->root) == expected["images"][k].get<std::string>() &&
           same_depth(frames->depths[k], expected["depths"][k], frames->root);
    }
    const auto& camera = expected["camera"];
    ok = ok &&
         slamlab::foundations::camera::name(frames->camera.model) ==
             camera["model"].get<std::string>() &&
         same(frames->camera.intrinsics, camera["intrinsics"]) &&
         frames->camera.width == camera["width"].get<std::int64_t>() &&
         frames->camera.height == camera["height"].get<std::int64_t>() &&
         frames->camera.depth_scale == camera["depth_scale"].get<double>();
    expect(ok, ("tum folder " + name + " reads as Python's").c_str(), failures);
  }
}

/// @brief Whether a camera equals the Python reader's.
/// @param got The camera.
/// @param expected Its JSON.
/// @return Whether it does.
bool same_camera(const euroc::Camera& got, const nlohmann::json& expected) {
  bool ok = same(got.intrinsics, expected["intrinsics"]) &&
            got.rows == expected["rows"].get<std::int64_t>() &&
            got.cols == expected["cols"].get<std::int64_t>();
  for (Eigen::Index r = 0; r < 4; ++r) {
    for (Eigen::Index c = 0; c < 4; ++c) {
      ok = ok && got.T_body_cam(r, c) == expected["T_body_cam"][r][c].get<double>();
    }
  }
  return ok;
}

/// @brief Check the EuRoC folder against the Python reader's frames.
/// @param expected The expected frames.
/// @param failures Running failure count.
void check_euroc(const nlohmann::json& expected, int& failures) {
  const auto frames = euroc::read_frames(folder("euroc"));
  bool ok = frames.has_value() && frames->timestamps.size() == expected["timestamps"].size();
  for (std::size_t k = 0; ok && k < frames->timestamps.size(); ++k) {
    ok = frames->timestamps[k] == expected["timestamps"][k].get<double>() &&
         relative(frames->left[k], frames->root) == expected["left"][k].get<std::string>() &&
         relative(frames->right[k], frames->root) == expected["right"][k].get<std::string>();
  }
  ok = ok && same_camera(frames->left_camera, expected["left_camera"]) &&
       same_camera(frames->right_camera, expected["right_camera"]);
  expect(ok, "the euroc folder reads as Python's", failures);
}

/// @brief Check the refusals: a malformed camera.txt, a nameless folder, a bad stamp.
/// @param failures Running failure count.
void check_refusals(int& failures) {
  const fs::path scratch = fs::temp_directory_path() / "slamlab_test_sequences";
  fs::remove_all(scratch);
  fs::create_directories(scratch / "recording");
  std::ofstream(scratch / "recording" / "rgb.txt") << "1.0 a.png\nx b.png\n";
  expect(is_invalid_input(tum::read_frames((scratch / "recording").string())),
         "a folder without camera.txt or a sensor's name is refused", failures);
  std::ofstream(scratch / "recording" / "camera.txt")
      << "model pinhole\nintrinsics 1 1 0 0\nsize 4 4\ndepth_scale 1\nbaseline 1\n";
  expect(is_invalid_input(tum::read_camera((scratch / "recording").string())),
         "an unknown camera.txt key is refused", failures);
  std::ofstream(scratch / "recording" / "camera.txt")
      << "model pinhole\nintrinsics 1 1 0 0\nsize 4 4\ndepth_scale 1\n";
  expect(is_invalid_input(tum::read_frames((scratch / "recording").string())),
         "a non-numeric stamp is refused", failures);
  expect(is_invalid_input(euroc::read_frames(scratch.string())), "a folder without mav0 is refused",
         failures);
  fs::remove_all(scratch);
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json cases = slamlab::testing::load_fixture("eval/sequences.json")["cases"];
  check_tum(cases["tum"], failures);
  check_euroc(cases["euroc"]["euroc"], failures);
  check_refusals(failures);
  return slamlab::testing::finish("eval/sequences", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("eval/sequences", run); }
