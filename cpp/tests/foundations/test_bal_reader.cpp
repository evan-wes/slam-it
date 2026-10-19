/// @file
/// @brief Native test of the C++ benchmark's BAL reader (cpp/bench/foundations/bal_reader.hpp)
///        against `tests/fixtures/eval/bal.json`, which the Python reader generates and the
///        Rust example's reader is tested against too: the model and every array field,
///        defaults included.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

#include "../../bench/foundations/bal_reader.hpp"
#include "fixture_support.hpp"
#include "slam_it/foundations/camera.hpp"
#include "test_support.hpp"

namespace {

using slam_it::testing::expect;
namespace ba = slam_it::foundations::ba;

/// @brief Flatten a nested JSON array, row-major (iteratively, depth first): numbers as they
///        are, booleans as 1/0, null as NaN.
/// @param value The array.
/// @return The values.
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
    } else if (item->is_null()) {
      out.push_back(std::numeric_limits<double>::quiet_NaN());
    } else if (item->is_boolean()) {
      out.push_back(item->get<bool>() ? 1.0 : 0.0);
    } else {
      out.push_back(item->get<double>());
    }
  }
  return out;
}

/// @brief Whether values match within `atol`, NaN matching NaN (and nothing else).
/// @param got The values.
/// @param want The expected values.
/// @param atol The tolerance.
/// @return True when every value matches.
bool same(const std::vector<double>& got, const std::vector<double>& want, double atol) {
  if (got.size() != want.size()) {
    return false;
  }
  for (std::size_t i = 0; i < got.size(); ++i) {
    const bool both_nan = std::isnan(got[i]) && std::isnan(want[i]);
    if (!both_nan && !(std::abs(got[i] - want[i]) <= atol)) {
      return false;
    }
  }
  return true;
}

/// @brief A matrix's values, row-major.
/// @tparam M The matrix type.
/// @param matrix The matrix.
/// @return The values.
template <typename M>
std::vector<double> rows(const M& matrix) {
  std::vector<double> out;
  for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
    for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
      out.push_back(static_cast<double>(matrix(r, c)));
    }
  }
  return out;
}

/// @brief A list's values as numbers.
/// @tparam T The element type (indices, flags).
/// @param values The list.
/// @return The values.
template <typename T>
std::vector<double> numbers(const std::vector<T>& values) {
  std::vector<double> out;
  out.reserve(values.size());
  for (const T value : values) {
    out.push_back(static_cast<double>(value));
  }
  return out;
}

/// @brief Poses' values, row-major.
/// @param poses The poses.
/// @return The values.
std::vector<double> pose_values(const ba::Poses& poses) {
  std::vector<double> out;
  for (const Eigen::Matrix4d& pose : poses) {
    const std::vector<double> one = rows(pose);
    out.insert(out.end(), one.begin(), one.end());
  }
  return out;
}

/// @brief The test body.
/// @return The exit code.
int run() {
  int failures = 0;
  const nlohmann::json fixture = slam_it::testing::load_fixture("eval/bal.json");
  const nlohmann::json& expected = fixture["expected"];
  const double atol = fixture["tolerance"]["atol"].get<double>();
  const auto problem = slam_it::bench::read_bal(std::string(SLAM_IT_FIXTURES_DIR) + "/eval/" +
                                                fixture["input"].get<std::string>());
  expect(problem.has_value(), "read_bal reads the tiny BAL file", failures);
  if (!problem) {
    return slam_it::testing::finish("foundations/bal_reader", failures);
  }
  expect(std::string(slam_it::foundations::camera::name(problem->model)) ==
             expected["model"].get<std::string>(),
         "model matches the Python reader", failures);
  const std::vector<std::pair<std::string, std::vector<double>>> fields = {
      {"intrinsics", rows(problem->intrinsics)},
      {"poses", pose_values(problem->poses)},
      {"points", rows(problem->points)},
      {"observations", rows(problem->observations)},
      {"cam_idx", numbers(problem->cam_idx)},
      {"pt_idx", numbers(problem->pt_idx)},
      {"intrinsics_idx", numbers(problem->intrinsics_idx)},
      {"weights", rows(problem->weights)},
      {"fixed_poses", numbers(problem->fixed_poses)},
      {"fixed_intrinsics", numbers(problem->fixed_intrinsics)},
      {"fixed_points", numbers(problem->fixed_points)},
      {"depth_prior", rows(problem->depth_prior)},
      {"depth_prior_weights", rows(problem->depth_prior_weights)},
      {"depth_affine", rows(problem->depth_affine)},
      {"fixed_depth_affine", numbers(problem->fixed_depth_affine)},
      {"pose_prior", pose_values(problem->pose_prior)},
      {"pose_prior_weights", rows(problem->pose_prior_weights)},
      {"point_anchor", numbers(problem->point_anchor)},
  };
  expect(fields.size() == expected["arrays"].size(), "every field is compared", failures);
  for (const auto& [name, got] : fields) {
    expect(same(got, flatten(expected["arrays"][name]), atol),
           (name + " matches the Python reader").c_str(), failures);
  }
  return slam_it::testing::finish("foundations/bal_reader", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("foundations/bal_reader", run); }
