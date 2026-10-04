/// @file
/// @brief Native test of the C++ benchmark's BAL reader (cpp/bench/foundations/bal_reader.hpp)
///        against `tests/fixtures/eval/bal.json`, which the Python reader generates and the
///        Rust example's reader is tested against too.
#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../../bench/foundations/bal_reader.hpp"
#include "fixture_support.hpp"
#include "test_support.hpp"

namespace {

using slamlab::testing::expect;

/// @brief Flatten a nested JSON array of numbers, row-major (iteratively, depth first).
/// @param value The array.
/// @return The numbers.
std::vector<double> flatten(const nlohmann::json& value) {
  std::vector<double> out;
  std::vector<const nlohmann::json*> pending = {&value};
  while (!pending.empty()) {
    const nlohmann::json* item = pending.back();
    pending.pop_back();
    if (item->is_array()) {
      for (auto it = item->rbegin(); it != item->rend(); ++it) {
        pending.push_back(&*it);
      }
    } else {
      out.push_back(item->get<double>());
    }
  }
  return out;
}

/// @brief Whether values match a fixture's nested array within `atol`.
/// @param got The values, row-major.
/// @param want The fixture's array.
/// @param atol The tolerance.
/// @return True when every value is within `atol`.
bool close(const std::vector<double>& got, const nlohmann::json& want, double atol) {
  const std::vector<double> expected = flatten(want);
  if (got.size() != expected.size()) {
    return false;
  }
  for (std::size_t i = 0; i < got.size(); ++i) {
    if (std::abs(got[i] - expected[i]) > atol) {
      return false;
    }
  }
  return true;
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slamlab::testing::load_fixture("eval/bal.json");
  const nlohmann::json& expected = fixture["expected"];
  const double atol = fixture["tolerance"]["atol"].get<double>();
  const auto problem = slamlab::bench::read_bal(std::string(SLAMLAB_FIXTURES_DIR) + "/eval/" +
                                                fixture["input"].get<std::string>());
  expect(problem.has_value(), "read_bal reads the tiny BAL file", failures);
  if (!problem) {
    return slamlab::testing::finish("foundations/bal_reader", failures);
  }
  std::vector<double> poses;
  for (const Eigen::Matrix4d& pose : problem->poses) {
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        poses.push_back(pose(r, c));
      }
    }
  }
  const auto rows = [](const auto& matrix) {
    std::vector<double> out;
    for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
      for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
        out.push_back(matrix(r, c));
      }
    }
    return out;
  };
  expect(close(poses, expected["poses"], atol), "poses match the Python reader", failures);
  expect(close(rows(problem->intrinsics), expected["intrinsics"], atol),
         "intrinsics match the Python reader", failures);
  expect(close(rows(problem->points), expected["points"], atol), "points match the Python reader",
         failures);
  expect(close(rows(problem->observations), expected["observations"], atol),
         "observations match the Python reader", failures);
  expect(problem->cam_idx == expected["cam_idx"].get<std::vector<std::int64_t>>() &&
             problem->pt_idx == expected["pt_idx"].get<std::vector<std::int64_t>>(),
         "indices match the Python reader", failures);
  std::vector<bool> fixed;
  for (const auto& row : expected["fixed_intrinsics"]) {
    for (const auto& value : row) {
      fixed.push_back(value.get<bool>());
    }
  }
  expect(problem->fixed_intrinsics == fixed, "fixed intrinsics match the Python reader", failures);
  return slamlab::testing::finish("foundations/bal_reader", failures);
}

}  // namespace

int main() { return slamlab::testing::guarded("foundations/bal_reader", run); }
