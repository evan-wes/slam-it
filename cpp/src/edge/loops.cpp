/// @file
/// @brief Implementation of the loop closing declared in slam_it/edge/loops.hpp: each function
///        mirrors its namesake in `slam_it.edge.loops`, step for step.
#include "slam_it/edge/loops.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "slam_it/edge/covisibility.hpp"
#include "slam_it/edge/residuals.hpp"
#include "slam_it/foundations/camera.hpp"
#include "slam_it/foundations/matching.hpp"
#include "slam_it/foundations/orb.hpp"
#include "slam_it/foundations/ransac.hpp"
#include "slam_it/foundations/sim3.hpp"

namespace slam_it::edge::loops {

namespace {

namespace graphs = foundations::graphs;
namespace matching = foundations::matching;
namespace ransac = foundations::ransac;
namespace sim3 = foundations::sim3;
using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

using Ids = std::vector<std::int64_t>;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

/// @brief The rigid motion from one pose to another, `T_to_w T_frm_w^-1`.
/// @param to The pose moved to.
/// @param from The pose moved from.
/// @return The motion.
Eigen::Matrix4d relative(const Eigen::Matrix4d& to, const Eigen::Matrix4d& from) {
  return to * sim3::inverse(from);
}

/// @brief Check a keyframe row.
/// @param m The map.
/// @param k The row.
/// @param name The argument's name, for the message.
/// @return The row; `InvalidInput` if it is out of range.
Result<std::size_t> keyframe_row_in(const Map& m, std::int64_t k, const char* name) {
  if (k < 0 || std::cmp_greater_equal(k, keyframe_count(m))) {
    return fail(ErrorKind::kInvalidInput, std::string(name) + ": expected a keyframe row");
  }
  return static_cast<std::size_t>(k);
}

/// @brief Check two keyframe rows, `k1` and `k2`.
/// @param m The map.
/// @param k1 The first.
/// @param k2 The second.
/// @return Both rows; `InvalidInput` naming the first out of range.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k1, k2), as Python's
Result<std::pair<std::size_t, std::size_t>> keyframe_rows_in(const Map& m, std::int64_t k1,
                                                             std::int64_t k2) {
  return keyframe_row_in(m, k1, "k1").and_then([&m, k2](std::size_t first) {
    return keyframe_row_in(m, k2, "k2").transform([first](std::size_t second) {
      return std::pair{first, second};
    });
  });
}

/// @brief Pixels as an (N, 2) array.
/// @param pixels The pixels.
/// @return The array.
graphs::Pairs pairs_of(std::span<const std::array<double, 2>> pixels) {
  graphs::Pairs out(static_cast<Eigen::Index>(pixels.size()), 2);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    out.row(static_cast<Eigen::Index>(i)) << pixels[i][0], pixels[i][1];
  }
  return out;
}

/// @brief Check a stage's mode, camera and settings, as detection and correction do first.
/// @param mode The mode's name.
/// @param camera The camera.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` for an unknown mode, a bad camera or bad settings.
Result<void> check_stage(const std::string& mode, const Camera& camera, const Options& options);

/// @brief A detection's start: the keyframe's row, whether similarities are rigid, and whether
///        the map is too small to search.
struct Stage {
  /// @brief The keyframe's row.
  std::size_t k{0};
  /// @brief Rigid similarities: stereo and RGB-D.
  bool fixed_scale{false};
  /// @brief Fewer keyframes than `loops.min_keyframes`.
  bool small{false};
};

/// @brief A detection's start, the mode, camera and settings checked first.
/// @param m The map.
/// @param keyframe The keyframe's id.
/// @param mode The mode's name.
/// @param camera The camera.
/// @param options The settings.
/// @return The start; `InvalidInput` for an unknown mode, bad camera or settings, or a keyframe
///         the map does not hold.
Result<Stage> stage_of(const Map& m, std::int64_t keyframe, const std::string& mode,
                       const Camera& camera, const Options& options) {
  return check_stage(mode, camera, options)
      .and_then([&m, keyframe] { return checked_keyframe_row(m, keyframe); })
      .transform([&](std::size_t k) {
        return Stage{.k = k,
                     .fixed_scale = mode != "monocular",
                     .small = std::cmp_less(keyframe_count(m), options.loops.min_keyframes)};
      });
}

/// @brief How many keypoints a search matched.
/// @param found Each keypoint's point id, or -1.
/// @return The matched ones.
std::int64_t count_matched(std::span<const std::int64_t> found) {
  return std::ranges::count_if(found, [](std::int64_t id) { return id >= 0; });
}

/// @brief The map a place is looked for in, and its database (`slam_it.edge.loops._Searched`):
///        the new keyframe's own for a loop, whose links a group must not hold; another map's
///        for a merge.
struct Target {
  /// @brief The map searched.
  const Map* map{nullptr};
  /// @brief Its keyframe database.
  const Database* database{nullptr};
  /// @brief Whether it is the new keyframe's own map.
  bool own{true};
};

/// @brief Points that project into a keyframe: their rows, pixels and predicted levels.
struct InView {
  /// @brief The points' rows.
  Ids rows;
  /// @brief Their pixels.
  std::vector<Eigen::Vector2d> pixels;
  /// @brief Their predicted levels.
  Ids levels;
};

/// @brief The points that project into a camera through a similarity, as ORB-SLAM checks them
///        (`slam_it.edge.loops._in_view`): in front, in the image, within the distances they
///        are matchable from (no margins), seen within `tracking.view_cos_limit`.
/// @param m The map.
/// @param camera The camera.
/// @param similarity The camera's `S_cw`.
/// @param rows The points' rows.
/// @param options `tracking.view_cos_limit`.
/// @return Those in view.
Result<InView> in_view(const Map& m, const Camera& camera, const Eigen::Matrix4d& similarity,
                       std::span<const std::int64_t> rows, const Options& options) {
  const Eigen::Matrix4d pose = rigid_of(similarity);
  const Eigen::Vector3d at = camera_centre(pose);
  InView out;
  std::vector<double> distances;
  for (const std::int64_t row : rows) {
    const auto p = static_cast<std::size_t>(row);
    const Eigen::Vector3d local = to_camera(pose, m.pt_xyz[p]);
    if (!(local.z() > 0.0)) {
      continue;
    }
    const Eigen::Vector2d pixel = project(camera, local);
    const Eigen::Vector3d offset = m.pt_xyz[p] - at;
    const double distance = offset.norm();
    if (!inside(camera, pixel) || !(distance >= m.pt_min_distance[p]) ||
        !(distance <= m.pt_max_distance[p]) || !(distance > 0.0) ||
        !(viewing_cos(m, p, offset, distance) >= options.tracking.view_cos_limit)) {
      continue;
    }
    out.rows.push_back(row);
    out.pixels.push_back(pixel);
    distances.push_back(distance);
  }
  return predict_level(m, out.rows, distances).transform([&out](std::vector<std::int64_t> levels) {
    out.levels = std::move(levels);
    return std::move(out);
  });
}

/// @brief A keyframe's keypoints within a window, on a level or the one below
///        (`slam_it.edge.loops._candidates_near`), as offsets into its keypoints.
/// @param grid The keyframe's keypoints' grid.
/// @param pixel The window's centre.
/// @param radius Its half-width.
/// @param level The level.
/// @return The offsets, ascending.
Result<Ids> near(const matching::FeatureGrid& grid, const Eigen::Vector2d& pixel, double radius,
                 std::int64_t level) {
  return grid.in_area({pixel.x(), pixel.y()}, radius, level - 1, level);
}

/// @brief A keyframe's keypoints' grid.
/// @param m The map.
/// @param k The keyframe's row.
/// @return The grid.
Result<matching::FeatureGrid> grid_of(const Map& m, std::size_t k) {
  return matching::FeatureGrid::build(keyframe_keypoints(m, k));
}

/// @brief The search through a similarity (`slam_it.edge.loops._search_by_similarity`),
///        its arguments checked: points of `source` matched to keyframe `k` of `m`.
/// @param m The keyframe's map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param similarity Its `S_cw`, in `source`'s world.
/// @param source The points' map (`m`, for a loop).
/// @param points The points' rows in it.
/// @param window The window and the distance.
/// @param options The settings.
/// @return Each keypoint's match, an id in `source`.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (m, source), as Python's
Result<Ids> search(const Map& m, const Camera& camera, std::size_t k,
                   const Eigen::Matrix4d& similarity, const Map& source,
                   std::span<const std::int64_t> points, Window window, const Options& options) {
  const std::int64_t start = m.kf_start[k];
  Ids out(static_cast<std::size_t>(m.kf_start[k + 1] - start), -1);
  auto seen = in_view(source, camera, similarity, points, options);
  if (!seen) {
    return std::unexpected(std::move(seen.error()));
  }
  auto grid = grid_of(m, k);
  if (!grid) {
    return std::unexpected(std::move(grid.error()));
  }
  for (std::size_t i = 0; i < seen->rows.size(); ++i) {
    const auto p = static_cast<std::size_t>(seen->rows[i]);
    const std::int64_t level = seen->levels[i];
    auto found = near(*grid, seen->pixels[i],
                      window.radius * std::pow(m.scale_factor, static_cast<double>(level)), level);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    std::int64_t best = -1;
    std::int64_t best_distance = std::numeric_limits<std::int64_t>::max();
    for (const std::int64_t j : *found) {
      if (out[static_cast<std::size_t>(j)] >= 0) {
        continue;  // matched already in this search
      }
      const std::int64_t d = foundations::orb::distance(
          source.pt_descriptor[p], m.kp_descriptor[static_cast<std::size_t>(start + j)]);
      if (d < best_distance) {
        best_distance = d;
        best = j;
      }
    }
    if (best >= 0 && best_distance <= window.distance) {
      out[static_cast<std::size_t>(best)] = source.pt_id[p];
    }
  }
  return out;
}

/// @brief The points the keyframes observe, each once, in the keyframes' order
///        (`slam_it.edge.loops._keyframe_points`).
/// @param m The map.
/// @param keyframes The keyframes' rows.
/// @return The points' rows.
Ids keyframe_points(const Map& m, std::span<const std::int64_t> keyframes) {
  Ids out;
  std::set<std::int64_t> taken;
  for (const std::int64_t k : keyframes) {
    for (const std::int64_t row : keypoint_rows(m, static_cast<std::size_t>(k))) {
      const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)];
      if (id >= 0 && taken.insert(id).second) {
        out.push_back(point_row(m, id));
      }
    }
  }
  return out;
}

/// @brief A keyframe's links' rows, most shared first, at most `limit` of them.
/// @param m The map.
/// @param k The keyframe's row.
/// @param options The settings.
/// @param limit The most kept, if any.
/// @return The rows.
Result<Ids> link_rows(const Map& m, std::size_t k, const Options& options,
                      std::optional<std::int64_t> limit) {
  return links(m, static_cast<std::int64_t>(k), options)
      .transform([limit](const std::vector<Link>& all) {
        Ids out;
        for (const Link& link : all) {
          if (limit && std::cmp_greater_equal(out.size(), *limit)) {
            break;
          }
          out.push_back(link.row);
        }
        return out;
      });
}

/// @brief A keyframe's best `loops.neighbours` links' rows (`slam_it.edge.loops._best_links`).
/// @param m The map.
/// @param k The keyframe's row.
/// @param options The settings.
/// @return The rows, most shared first.
Result<Ids> best_links(const Map& m, std::size_t k, const Options& options) {
  return link_rows(m, k, options, options.loops.neighbours);
}

/// @brief A keyframe's links' rows, all of them (`slam_it.edge.loops._linked`).
/// @param m The map.
/// @param k The keyframe's row.
/// @param options The settings.
/// @return The rows.
Result<std::set<std::int64_t>> linked_rows(const Map& m, std::size_t k, const Options& options) {
  return link_rows(m, k, options, std::nullopt).transform([](const Ids& rows) {
    return std::set<std::int64_t>(rows.begin(), rows.end());
  });
}

/// @brief The group's points (`slam_it.edge.loops._group_points`), the row checked.
/// @param m The map.
/// @param matched The matched keyframe's row.
/// @param linked The searching keyframe's links' rows.
/// @param options The settings.
/// @return The points' rows.
Result<Ids> group_of(const Map& m, std::size_t matched, const std::set<std::int64_t>& linked,
                     const Options& options) {
  auto first = best_links(m, matched, options);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  Ids group = *first;
  group.push_back(static_cast<std::int64_t>(matched));
  if (std::cmp_less(first->size(), options.loops.neighbours)) {
    std::set<std::int64_t> inside_group(group.begin(), group.end());
    for (const std::int64_t row : *first) {
      auto theirs = best_links(m, static_cast<std::size_t>(row), options);
      if (!theirs) {
        return std::unexpected(std::move(theirs.error()));
      }
      std::int64_t added = 0;
      for (const std::int64_t other : *theirs) {
        if (added == options.loops.neighbours) {
          break;
        }
        if (!inside_group.contains(other) && !linked.contains(other)) {
          inside_group.insert(other);
          group.push_back(other);
          ++added;
        }
      }
    }
  }
  return keyframe_points(m, group);
}

/// @brief The refinement (`slam_it.edge.loops._refine`), its arguments checked.
/// @param m The first keyframe's map.
/// @param camera The camera.
/// @param k1 The first keyframe's row.
/// @param other The second's map (`m`, for a loop).
/// @param k2 The second's row.
/// @param similarity The starting `S_12`.
/// @param matched For each keypoint of `k1`, its match.
/// @param fixed_scale Refine a rigid motion.
/// @param options The settings.
/// @return The refined similarity and its inliers, or none.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): Python's _refine, step for step
Result<std::optional<Refined>> refined(const Map& m, const Camera& camera, std::size_t k1,
                                       const Map& other, std::size_t k2,
                                       const Eigen::Matrix4d& similarity,
                                       std::span<const std::int64_t> matched, bool fixed_scale,
                                       const Options& options) {
  const LoopOptions& settings = options.loops;
  Ids rows;
  std::vector<std::size_t> partners;
  std::vector<Eigen::Vector3d> points1;
  std::vector<Eigen::Vector3d> points2;
  for (std::size_t i = 0; i < matched.size(); ++i) {
    const std::int64_t row = m.kf_start[k1] + static_cast<std::int64_t>(i);
    const std::int64_t own = m.kp_point[static_cast<std::size_t>(row)];
    if (matched[i] < 0 || own < 0) {
      continue;
    }
    const std::int64_t p_own = point_row(m, own);
    const std::int64_t p_partner = point_row(other, matched[i]);
    if (p_own < 0 || p_partner < 0) {
      continue;
    }
    const Eigen::Vector3d one = to_camera(m.kf_pose[k1], m.pt_xyz[static_cast<std::size_t>(p_own)]);
    const Eigen::Vector3d two =
        to_camera(other.kf_pose[k2], other.pt_xyz[static_cast<std::size_t>(p_partner)]);
    if (!(one.z() > foundations::camera::kMinDepth) ||
        !(two.z() > foundations::camera::kMinDepth)) {
      continue;
    }
    rows.push_back(row);
    partners.push_back(static_cast<std::size_t>(p_partner));
    points1.push_back(one);
    points2.push_back(two);
  }
  if (rows.empty()) {
    return std::nullopt;
  }
  const std::size_t n = rows.size();
  graphs::Points all1(static_cast<Eigen::Index>(n), 3);
  graphs::Points all2(static_cast<Eigen::Index>(n), 3);
  graphs::Pairs pixels1(static_cast<Eigen::Index>(n), 2);
  graphs::Pairs pixels2(static_cast<Eigen::Index>(n), 2);
  graphs::Pairs information(static_cast<Eigen::Index>(n), 2);
  const Ids rows2 = keypoint_rows(other, k2);
  for (std::size_t i = 0; i < n; ++i) {
    const auto e = static_cast<Eigen::Index>(i);
    all1.row(e) = points1[i].transpose();
    all2.row(e) = points2[i].transpose();
    const auto r = static_cast<std::size_t>(rows[i]);
    pixels1.row(e) << m.kp_xy[r][0], m.kp_xy[r][1];
    pixels2.row(e) << kNan, kNan;
    std::int64_t level2 = 0;
    const std::int64_t id = other.pt_id[partners[i]];
    for (const std::int64_t at : rows2) {
      if (other.kp_point[static_cast<std::size_t>(at)] == id) {
        const auto a = static_cast<std::size_t>(at);
        pixels2.row(e) << other.kp_xy[a][0], other.kp_xy[a][1];
        level2 = other.kp_level[a];
        break;
      }
    }
    information.row(e) << 1.0 / sigma2(m, m.kp_level[r]), 1.0 / sigma2(other, level2);
  }
  const auto run = [&](const Eigen::Matrix4d& start, const std::vector<std::size_t>& keep,
                       std::optional<double> huber,
                       std::int64_t iterations) -> Result<graphs::SimilarityRefinement> {
    const auto rows_kept = static_cast<Eigen::Index>(keep.size());
    graphs::Points a(rows_kept, 3);
    graphs::Points b(rows_kept, 3);
    graphs::Pairs x1(rows_kept, 2);
    graphs::Pairs x2(rows_kept, 2);
    graphs::Pairs info(rows_kept, 2);
    for (std::size_t j = 0; j < keep.size(); ++j) {
      const auto to = static_cast<Eigen::Index>(j);
      const auto from = static_cast<Eigen::Index>(keep[j]);
      a.row(to) = all1.row(from);
      b.row(to) = all2.row(from);
      x1.row(to) = pixels1.row(from);
      x2.row(to) = pixels2.row(from);
      info.row(to) = information.row(from);
    }
    return graphs::refine_similarity(start, a, b, x1, x2, info, camera.model, camera.intrinsics,
                                     fixed_scale, huber,
                                     {.max_iterations = static_cast<int>(iterations)});
  };
  const auto within = [&settings](const graphs::SimilarityRefinement& found) {
    std::vector<std::size_t> out;
    for (Eigen::Index i = 0; i < found.chi2.rows(); ++i) {
      if (found.chi2(i, 0) <= settings.huber_chi2 && !(found.chi2(i, 1) > settings.huber_chi2)) {
        out.push_back(static_cast<std::size_t>(i));
      }
    }
    return out;
  };
  std::vector<std::size_t> everything(n);
  std::ranges::iota(everything, std::size_t{0});
  auto first =
      run(similarity, everything, std::sqrt(settings.huber_chi2), settings.refine_iterations);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  const std::vector<std::size_t> good = within(*first);
  if (std::cmp_less(good.size(), settings.min_kept)) {
    return std::nullopt;
  }
  const std::int64_t more = good.size() == n ? settings.refine_iterations : settings.refine_more;
  auto second = run(first->similarity, good, std::nullopt, more);
  if (!second) {
    return std::unexpected(std::move(second.error()));
  }
  return Refined{.similarity = second->similarity,
                 .inliers = static_cast<std::int64_t>(within(*second).size())};
}

/// @brief Search a matched keyframe's group's points in a keyframe through a similarity
///        (`slam_it.edge.loops._confirm_by_projection`, `FindMatchesByProjection`).
/// @param m The keyframe's map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param matched The matched keyframe's row, in the searched map.
/// @param similarity The keyframe's `S_cw`, in the searched map's world.
/// @param target The map searched: in the keyframe's own, its links left out of the group.
/// @param options The settings.
/// @return The group's points' ids, and each of the keyframe's keypoints' match.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k, matched), as Python's
Result<std::pair<Ids, Ids>> confirm_by_projection(const Map& m, const Camera& camera, std::size_t k,
                                                  std::size_t matched,
                                                  const Eigen::Matrix4d& similarity,
                                                  const Target& target, const Options& options) {
  std::set<std::int64_t> linked;
  if (target.own) {
    auto own = linked_rows(m, k, options);
    if (!own) {
      return std::unexpected(std::move(own.error()));
    }
    linked = std::move(*own);
  }
  const Map& other = *target.map;
  auto points = group_of(other, matched, linked, options);
  if (!points) {
    return std::unexpected(std::move(points.error()));
  }
  auto found =
      search(m, camera, k, similarity, other, *points,
             {.radius = options.loops.confirm_window, .distance = options.loops.confirm_distance},
             options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  Ids ids;
  ids.reserve(points->size());
  for (const std::int64_t row : *points) {
    ids.push_back(other.pt_id[static_cast<std::size_t>(row)]);
  }
  return std::pair{std::move(ids), std::move(*found)};
}

/// @brief A loop being confirmed after a keyframe, or none once dropped, and how it went.
using Carried = std::pair<std::optional<Pending>, ConfirmationSummary>;

/// @brief Try to confirm the loop being confirmed on a later keyframe
///        (`slam_it.edge.loops._carry`).
/// @param m The keyframe's map, holding the loop's last keyframe.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param pending The loop.
/// @param target The map holding its matched keyframe.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The loop as it stands after the keyframe, and how it went.
Result<Carried> carry(const Map& m, const Camera& camera, std::size_t k, const Pending& pending,
                      const Target& target, bool fixed_scale, const Options& options) {
  const LoopOptions& settings = options.loops;
  const Map& other = *target.map;
  auto matched = checked_keyframe_row(other, pending.matched);
  if (!matched) {
    return std::unexpected(std::move(matched.error()));
  }
  auto last = checked_keyframe_row(m, pending.last);
  if (!last) {
    return std::unexpected(std::move(last.error()));
  }
  const Eigen::Matrix4d carried = relative(m.kf_pose[k], m.kf_pose[*last]) * pending.similarity;
  auto first = confirm_by_projection(m, camera, k, *matched, carried, target, options);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  ConfirmationSummary summary{.matched = pending.matched,
                              .projected = count_matched(first->second),
                              .refined = std::nullopt,
                              .reprojected = std::nullopt,
                              .confirmed = false};
  std::optional<Pending> confirmed;
  if (summary.projected >= settings.confirm_matches) {
    const Eigen::Matrix4d start = carried * sim3::inverse(other.kf_pose[*matched]);
    auto found = refined(m, camera, k, other, *matched, start, first->second, fixed_scale, options);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    if (found->has_value()) {
      summary.refined = (*found)->inliers;
    }
    if (found->has_value() && (*found)->inliers > settings.temporal_refined) {
      const Eigen::Matrix4d placed = (*found)->similarity * other.kf_pose[*matched];
      auto again = confirm_by_projection(m, camera, k, *matched, placed, target, options);
      if (!again) {
        return std::unexpected(std::move(again.error()));
      }
      summary.reprojected = count_matched(again->second);
      if (*summary.reprojected >= settings.temporal_matches) {
        confirmed = Pending{.matched = pending.matched,
                            .last = m.kf_id[k],
                            .similarity = placed,
                            .confirmations = pending.confirmations + 1,
                            .misses = 0,
                            .points = std::move(again->first),
                            .matched_points = std::move(again->second)};
      }
    }
  }
  if (!confirmed) {
    const std::int64_t misses = pending.misses + 1;
    if (misses >= settings.misses) {
      return Carried{std::nullopt, summary};
    }
    Pending left = pending;
    left.misses = misses;
    return Carried{std::move(left), summary};
  }
  summary.confirmed = true;
  return Carried{std::move(confirmed), summary};
}

/// @brief A candidate that passed every test: where it places the loop and what it matched.
struct Found {
  /// @brief The row of the group's keyframe that matched most.
  std::size_t best{0};
  /// @brief The new keyframe's `S_cw` as the loop places it.
  Eigen::Matrix4d similarity;
  /// @brief The group's points' ids.
  Ids points;
  /// @brief Each of the new keyframe's keypoints' match.
  Ids matched_points;
  /// @brief The second search's matches.
  std::int64_t reprojected{0};
  /// @brief The new keyframe's links that confirmed it.
  std::int64_t confirmations{0};
};

/// @brief Match keyframe `k1` of `m` to `k2` of the searched map by their points' words
///        (`slam_it.edge.loops._match_by_word`), its rows checked.
/// @param m The first keyframe's map.
/// @param db Its database.
/// @param k1 The first keyframe's row.
/// @param target The second's map, with its database (`m`'s own, for a loop).
/// @param k2 The second keyframe's row.
/// @param options `matching.th_low` and `loops.ratio`.
/// @return For each keypoint of `k1`, the id of the point of `k2` it matched, -1 where none;
///         `InvalidInput` for a keyframe a database does not hold.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k1, k2), as Python's
Result<Ids> match_words(const Map& m, const Database& db, std::size_t k1, const Target& target,
                        std::size_t k2, const Options& options) {
  const std::array<const Map*, 2> maps{&m, target.map};
  const std::array<const Database*, 2> databases{&db, target.database};
  const std::array<std::size_t, 2> keyframes{k1, k2};
  std::array<std::vector<foundations::orb::Descriptor>, 2> descriptors;
  std::array<Ids, 2> nodes;
  std::array<Ids, 2> angles;
  std::array<Ids, 2> own;
  for (std::size_t side = 0; side < 2; ++side) {
    const Map& map = *maps[side];
    const std::size_t k = keyframes[side];
    const auto words = databases[side]->keyframes.find(map.kf_id[k]);
    if (words == databases[side]->keyframes.end()) {
      return fail(ErrorKind::kInvalidInput,
                  "keyframe " + std::to_string(map.kf_id[k]) + " is not in the database");
    }
    for (const std::int64_t row : keypoint_rows(map, k)) {
      const auto r = static_cast<std::size_t>(row);
      if (map.kp_point[r] < 0) {
        continue;
      }
      own[side].push_back(row);
      descriptors[side].push_back(map.kp_descriptor[r]);
      nodes[side].push_back(words->second.nodes[static_cast<std::size_t>(row - map.kf_start[k])]);
      angles[side].push_back(map.kp_angle[r]);
    }
  }
  auto found = matching::search_by_bow(descriptors[0], nodes[0], descriptors[1], nodes[1],
                                       angles[0], angles[1],
                                       {.max_distance = options.matching.th_low,
                                        .ratio = options.loops.ratio,
                                        .check_orientation = true});
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  Ids out(static_cast<std::size_t>(m.kf_start[k1 + 1] - m.kf_start[k1]), -1);
  for (const matching::Match& match : *found) {
    const std::int64_t row1 = own[0][static_cast<std::size_t>(match.first)];
    const std::int64_t row2 = own[1][static_cast<std::size_t>(match.second)];
    out[static_cast<std::size_t>(row1 - m.kf_start[k1])] =
        target.map->kp_point[static_cast<std::size_t>(row2)];
  }
  return out;
}

/// @brief A keyframe matched by word to a group: each keypoint's point and the row it was
///        matched in, and the row of the group's keyframe that matched most.
struct ByWord {
  /// @brief Each keypoint's point id, or -1.
  Ids points;
  /// @brief The row of the keyframe it was matched in, or -1.
  Ids source;
  /// @brief The group's keyframe that matched most (ties: the first).
  std::size_t best{0};
};

/// @brief Match a keyframe by word to each of a group's keyframes, and merge the matches
///        (`slam_it.edge.loops._merged_by_word`).
/// @param m The keyframe's map.
/// @param db Its database.
/// @param k The keyframe's row.
/// @param group The group's rows, in the searched map.
/// @param target The map searched, with its database.
/// @param options The settings.
/// @return The merged matches.
Result<ByWord> merged_by_word(const Map& m, const Database& db, std::size_t k,
                              std::span<const std::int64_t> group, const Target& target,
                              const Options& options) {
  const auto size = static_cast<std::size_t>(m.kf_start[k + 1] - m.kf_start[k]);
  ByWord out{.points = Ids(size, -1), .source = Ids(size, -1), .best = 0};
  std::set<std::int64_t> used;
  std::int64_t most = -1;
  out.best = static_cast<std::size_t>(group.front());
  for (const std::int64_t g : group) {
    auto found = match_words(m, db, k, target, static_cast<std::size_t>(g), options);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    if (const std::int64_t count = count_matched(*found); count > most) {
      out.best = static_cast<std::size_t>(g);
      most = count;
    }
    for (std::size_t i = 0; i < found->size(); ++i) {
      const std::int64_t id = (*found)[i];
      if (id >= 0 && out.points[i] < 0 && !used.contains(id)) {
        out.points[i] = id;
        out.source[i] = g;
        used.insert(id);
      }
    }
  }
  return out;
}

/// @brief The loop's RANSAC: the similarity, or none, and its inliers (none if it did not run).
struct Ransacked {
  /// @brief `S_kb`, if it found one with enough inliers.
  std::optional<Eigen::Matrix4d> similarity;
  /// @brief Its inliers, if it ran.
  std::optional<std::int64_t> inliers;
};

/// @brief Find the similarity from the best keyframe's frame to the keyframe's by RANSAC
///        (`slam_it.edge.loops._similarity_by_ransac`).
/// @param m The keyframe's map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param merged Its matches by word.
/// @param other The matched points' map (`m`, for a loop).
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The similarity and its inliers.
Result<Ransacked> similarity_by_ransac(const Map& m, const Camera& camera, std::size_t k,
                                       const ByWord& merged, const Map& other, bool fixed_scale,
                                       const Options& options) {
  const LoopOptions& settings = options.loops;
  std::vector<Eigen::Vector3d> points1;
  std::vector<Eigen::Vector3d> points2;
  std::vector<double> sigma1;
  std::vector<double> sigma2s;
  for (std::size_t i = 0; i < merged.points.size(); ++i) {
    if (merged.points[i] < 0) {
      continue;
    }
    const auto row = static_cast<std::size_t>(m.kf_start[k]) + i;
    const auto own = static_cast<std::size_t>(point_row(m, m.kp_point[row]));
    const auto partner = static_cast<std::size_t>(point_row(other, merged.points[i]));
    const auto g = static_cast<std::size_t>(merged.source[i]);
    std::int64_t level2 = 0;
    for (const std::int64_t at : keypoint_rows(other, g)) {
      if (other.kp_point[static_cast<std::size_t>(at)] == merged.points[i]) {
        level2 = other.kp_level[static_cast<std::size_t>(at)];
        break;
      }
    }
    const Eigen::Vector3d one = to_camera(m.kf_pose[k], m.pt_xyz[own]);
    const Eigen::Vector3d two = to_camera(other.kf_pose[merged.best], other.pt_xyz[partner]);
    if (!(one.z() > foundations::camera::kMinDepth) ||
        !(two.z() > foundations::camera::kMinDepth)) {
      continue;
    }
    points1.push_back(one);
    points2.push_back(two);
    sigma1.push_back(sigma2(m, m.kp_level[row]));
    sigma2s.push_back(sigma2(other, level2));
  }
  const auto count = static_cast<std::int64_t>(points1.size());
  if (count < std::max<std::int64_t>(settings.min_inliers, sim3::kSampleSize)) {
    return Ransacked{};
  }
  sim3::Rows3 one(count, 3);
  sim3::Rows3 two(count, 3);
  for (Eigen::Index i = 0; i < count; ++i) {
    one.row(i) = points1[static_cast<std::size_t>(i)].transpose();
    two.row(i) = points2[static_cast<std::size_t>(i)].transpose();
  }
  const ransac::Options run{.threshold = std::sqrt(settings.chi2),
                            .confidence = settings.confidence,
                            .max_iterations = static_cast<int>(ransac_budget(count, options)),
                            .seed = static_cast<std::uint64_t>(options.seed)};
  auto found = sim3::sim3_ransac(one, two, Eigen::Map<const Eigen::VectorXd>(sigma1.data(), count),
                                 Eigen::Map<const Eigen::VectorXd>(sigma2s.data(), count),
                                 camera.model, camera.intrinsics, fixed_scale, run);
  if (!found) {
    if (found.error().kind == ErrorKind::kDegenerate) {
      return Ransacked{.similarity = std::nullopt, .inliers = 0};
    }
    return std::unexpected(std::move(found.error()));
  }
  const std::int64_t inliers = found->summary.inlier_count;
  if (inliers < settings.min_inliers) {
    return Ransacked{.similarity = std::nullopt, .inliers = inliers};
  }
  return Ransacked{.similarity = found->state.model, .inliers = inliers};
}

/// @brief Count the keyframe's best links that confirm the loop, at most
///        `loops.confirmations` (`slam_it.edge.loops._confirmations`).
/// @param m The keyframe's map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param best The matched keyframe's row, in the searched map.
/// @param similarity The keyframe's `S_cw`.
/// @param target The map searched.
/// @param options The settings.
/// @return The confirmations.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k, best), as Python's
Result<std::int64_t> confirmations(const Map& m, const Camera& camera, std::size_t k,
                                   std::size_t best, const Eigen::Matrix4d& similarity,
                                   const Target& target, const Options& options) {
  auto neighbours = best_links(m, k, options);
  if (!neighbours) {
    return std::unexpected(std::move(neighbours.error()));
  }
  std::int64_t count = 0;
  for (const std::int64_t j : *neighbours) {
    if (count == options.loops.confirmations) {
      break;
    }
    const auto r = static_cast<std::size_t>(j);
    const Eigen::Matrix4d carried = relative(m.kf_pose[r], m.kf_pose[k]) * similarity;
    auto found = confirm_by_projection(m, camera, r, best, carried, target, options);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    if (count_matched(found->second) >= options.loops.confirm_matches) {
      ++count;
    }
  }
  return count;
}

/// @brief Where a candidate places the loop: the keyframe's `S_cw`, the group's points' rows
///        and the second search's matches.
struct Placed {
  /// @brief The keyframe's `S_cw`.
  Eigen::Matrix4d similarity;
  /// @brief The group's points' rows.
  Ids points;
  /// @brief The second search's matches.
  Ids fine;
};

/// @brief Place a candidate's loop: search, refine, search again (`slam_it.edge.loops._place`).
/// @param m The keyframe's map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param best The best keyframe's row, in `other`.
/// @param model `S_kb`.
/// @param summary The candidate's summary, its counts filled in as reached.
/// @param other The searched map (`m`, for a loop).
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The placement, or none where a test failed.
Result<std::optional<Placed>> place(const Map& m, const Camera& camera, std::size_t k,
                                    std::size_t best, const Eigen::Matrix4d& model,
                                    CandidateSummary& summary, const Map& other, bool fixed_scale,
                                    const Options& options) {
  const LoopOptions& settings = options.loops;
  auto group = best_links(other, best, options);
  if (!group) {
    return std::unexpected(std::move(group.error()));
  }
  group->push_back(static_cast<std::int64_t>(best));
  const Ids points = keyframe_points(other, *group);
  auto coarse =
      search(m, camera, k, model * other.kf_pose[best], other, points,
             {.radius = settings.coarse_window, .distance = settings.coarse_distance}, options);
  if (!coarse) {
    return std::unexpected(std::move(coarse.error()));
  }
  summary.projected = count_matched(*coarse);
  if (*summary.projected < settings.coarse_matches) {
    return std::nullopt;
  }
  auto found = refined(m, camera, k, other, best, model, *coarse, fixed_scale, options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  if (!found->has_value()) {
    return std::nullopt;
  }
  summary.refined = (*found)->inliers;
  if ((*found)->inliers < settings.min_refined) {
    return std::nullopt;
  }
  const Eigen::Matrix4d placed = (*found)->similarity * other.kf_pose[best];
  auto fine = search(m, camera, k, placed, other, points,
                     {.radius = settings.fine_window, .distance = settings.fine_distance}, options);
  if (!fine) {
    return std::unexpected(std::move(fine.error()));
  }
  summary.reprojected = count_matched(*fine);
  if (*summary.reprojected < settings.fine_matches) {
    return std::nullopt;
  }
  return Placed{.similarity = placed, .points = points, .fine = std::move(*fine)};
}

/// @brief One candidate tried: how far it got, and what it found if it passed.
using Tried = std::pair<CandidateSummary, std::optional<Found>>;

/// @brief Try one of the database's candidates (`slam_it.edge.loops._try_candidate`).
/// @param m The keyframe's map.
/// @param db Its database.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param candidate The candidate's id, in the searched map, and score.
/// @param target The map searched, with its database.
/// @param linked The keyframe's links' rows in its own map (none for another).
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return How far it got, and what it found.
Result<Tried> try_candidate(const Map& m, const Database& db, const Camera& camera, std::size_t k,
                            std::pair<std::int64_t, double> candidate, const Target& target,
                            const std::set<std::int64_t>& linked, bool fixed_scale,
                            const Options& options) {
  const LoopOptions& settings = options.loops;
  const Map& other = *target.map;
  auto c = checked_keyframe_row(other, candidate.first);
  if (!c) {
    return std::unexpected(std::move(c.error()));
  }
  auto neighbours = best_links(other, *c, options);
  if (!neighbours) {
    return std::unexpected(std::move(neighbours.error()));
  }
  Ids group{static_cast<std::int64_t>(*c)};
  group.insert(group.end(), neighbours->begin(), neighbours->end());
  CandidateSummary summary{.keyframe = candidate.first,
                           .score = candidate.second,
                           .near = false,
                           .matches = std::nullopt,
                           .best = std::nullopt,
                           .inliers = std::nullopt,
                           .projected = std::nullopt,
                           .refined = std::nullopt,
                           .reprojected = std::nullopt,
                           .confirmations = std::nullopt};
  if (std::ranges::any_of(group, [&linked](std::int64_t g) { return linked.contains(g); })) {
    summary.near = true;
    return Tried{summary, std::nullopt};
  }
  auto merged = merged_by_word(m, db, k, group, target, options);
  if (!merged) {
    return std::unexpected(std::move(merged.error()));
  }
  summary.matches = count_matched(merged->points);
  summary.best = other.kf_id[merged->best];
  if (*summary.matches < settings.min_matches) {
    return Tried{summary, std::nullopt};
  }
  auto model = similarity_by_ransac(m, camera, k, *merged, other, fixed_scale, options);
  if (!model) {
    return std::unexpected(std::move(model.error()));
  }
  summary.inliers = model->inliers;
  if (!model->similarity) {
    return Tried{summary, std::nullopt};
  }
  auto placed =
      place(m, camera, k, merged->best, *model->similarity, summary, other, fixed_scale, options);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  if (!placed->has_value()) {
    return Tried{summary, std::nullopt};
  }
  auto confirmed =
      confirmations(m, camera, k, merged->best, (*placed)->similarity, target, options);
  if (!confirmed) {
    return std::unexpected(std::move(confirmed.error()));
  }
  summary.confirmations = *confirmed;
  Ids ids;
  ids.reserve((*placed)->points.size());
  for (const std::int64_t row : (*placed)->points) {
    ids.push_back(other.pt_id[static_cast<std::size_t>(row)]);
  }
  const std::int64_t reprojected = count_matched((*placed)->fine);
  return Tried{summary, Found{.best = merged->best,
                              .similarity = (*placed)->similarity,
                              .points = std::move(ids),
                              .matched_points = std::move((*placed)->fine),
                              .reprojected = reprojected,
                              .confirmations = *confirmed}};
}

/// @brief The summary of a map too small to search: skipped, the confirmations carried.
/// @param confirmations The confirmations of what was being confirmed, if anything.
/// @return The summary.
DetectionSummary skipped_summary(std::int64_t confirmations) {
  return {.outcome = Outcome::kSkipped,
          .confirmation = std::nullopt,
          .sharing = 0,
          .scored = 0,
          .candidates = {},
          .confirmations = confirmations};
}

/// @brief The summary of what was being confirmed, confirmed again on the keyframe.
/// @param confirmation How it was carried to the keyframe.
/// @return The summary.
DetectionSummary confirmed_summary(const ConfirmationSummary& confirmation) {
  return {.outcome = Outcome::kConfirmed,
          .confirmation = confirmation,
          .sharing = 0,
          .scored = 0,
          .candidates = {},
          .confirmations = 0};
}

/// @brief A search's summary before its candidates are tried.
/// @param sharing The keyframes sharing a word.
/// @param scored Those scored.
/// @return The summary.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (sharing, scored), as the summary's
DetectionSummary searched_summary(std::int64_t sharing, std::int64_t scored) {
  return {.outcome = Outcome::kNone,
          .confirmation = std::nullopt,
          .sharing = sharing,
          .scored = scored,
          .candidates = {},
          .confirmations = 0};
}

/// @brief The region a candidate found, started from the keyframe it was found on.
/// @param found What the candidate found.
/// @param matched The id of the keyframe it matched.
/// @param keyframe The new keyframe's id.
/// @return The region, its confirmations the candidate's.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (matched, keyframe), as the region's
Pending started_from(Found& found, std::int64_t matched, std::int64_t keyframe) {
  return {.matched = matched,
          .last = keyframe,
          .similarity = found.similarity,
          .confirmations = found.confirmations,
          .misses = 0,
          .points = std::move(found.points),
          .matched_points = std::move(found.matched_points)};
}

/// @brief A search for a region: the best candidate that passed, with its map's place among
///        the maps searched (a loop's own, 0), if any; how the search went; and each candidate's
///        map number (none for a loop).
struct RegionSearch {
  /// @brief The best candidate's map's place, and what it found.
  std::optional<std::pair<std::size_t, Found>> best;
  /// @brief How the search went.
  DetectionSummary summary;
  /// @brief Each candidate's map number.
  std::vector<std::int64_t> maps;
};

/// @brief Try the database's candidates (`slam_it.edge.loops._search`).
/// @param m The map.
/// @param db Its database.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The best candidate that passed, and how the search went.
Result<RegionSearch> search_candidates(const Map& m, const Database& db, const Camera& camera,
                                       std::size_t k, bool fixed_scale, const Options& options) {
  auto candidates = loop_candidates(db, m, m.kf_id[k], options);
  if (!candidates) {
    return std::unexpected(std::move(candidates.error()));
  }
  auto linked = linked_rows(m, k, options);
  if (!linked) {
    return std::unexpected(std::move(linked.error()));
  }
  DetectionSummary summary = searched_summary(candidates->sharing, candidates->scored);
  std::optional<Found> best;
  for (std::size_t i = 0; i < candidates->keyframes.size(); ++i) {
    auto tried =
        try_candidate(m, db, camera, k, {candidates->keyframes[i], candidates->scores[i]},
                      {.map = &m, .database = &db, .own = true}, *linked, fixed_scale, options);
    if (!tried) {
      return std::unexpected(std::move(tried.error()));
    }
    summary.candidates.push_back(tried->first);
    if (tried->second && (!best || tried->second->reprojected > best->reprojected)) {
      best = std::move(tried->second);
    }
  }
  return RegionSearch{
      .best = best ? std::optional(std::pair{std::size_t{0}, std::move(*best)}) : std::nullopt,
      .summary = std::move(summary),
      .maps = {}};
}

/// @brief Close a loop with enough confirmations, carry it with fewer, keep none with none
///        (`slam_it.edge.loops._settle`).
/// @param state The loop as the keyframe left it.
/// @param summary What happened, if it is carried.
/// @param options The settings.
/// @return The detection.
Detection settle(Pending state, DetectionSummary summary, const Options& options) {
  summary.confirmations = state.confirmations;
  if (state.confirmations >= options.loops.confirmations) {
    summary.outcome = Outcome::kClosed;
    Loop loop{.current = state.last,
              .matched = state.matched,
              .similarity = state.similarity,
              .points = std::move(state.points),
              .matched_points = std::move(state.matched_points)};
    return {.pending = std::nullopt, .loop = std::move(loop), .summary = std::move(summary)};
  }
  if (state.confirmations == 0) {
    summary.outcome = Outcome::kNone;
    summary.confirmations = 0;
    return {.pending = std::nullopt, .loop = std::nullopt, .summary = std::move(summary)};
  }
  return {.pending = std::move(state), .loop = std::nullopt, .summary = std::move(summary)};
}

/// @brief Where detection looks: the region being confirmed and the map holding it, and the
///        maps a new region may be found in.
struct Looking {
  /// @brief The region being confirmed, if any.
  std::optional<Pending> pending;
  /// @brief The map holding its matched keyframe, with its database.
  Target holding;
  /// @brief That map's place among `maps`.
  std::size_t holding_at{0};
  /// @brief The maps a new region may be found in, by place.
  std::vector<const Map*> maps;
};

/// @brief A region detected: as a loop's detection holds it, with the place of the map it
///        matched among the maps searched, and each candidate's map number.
struct Detected {
  /// @brief The detection.
  Detection detection;
  /// @brief The place of the map its region matched.
  std::size_t at{0};
  /// @brief Each candidate's map number.
  std::vector<std::int64_t> maps;
};

/// @brief Detection's flow, a loop's and a merge's alike (`slam_it.edge.loops._detect` and
///        `_detect_merge`): the region being confirmed carried to the keyframe, and, when it is
///        not confirmed there, the search's candidates tried.
/// @param m The keyframe's map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param looking Where to look.
/// @param search The search of the database's candidates.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The region detected.
Result<Detected> detect_region(const Map& m, const Camera& camera, std::size_t k,
                               const Looking& looking,
                               const std::function<Result<RegionSearch>()>& search,
                               bool fixed_scale, const Options& options) {
  std::optional<Pending> state = looking.pending;
  std::optional<ConfirmationSummary> confirmation;
  bool dropped = false;
  if (state) {
    auto carried = carry(m, camera, k, *state, looking.holding, fixed_scale, options);
    if (!carried) {
      return std::unexpected(std::move(carried.error()));
    }
    confirmation = carried->second;
    if (confirmation->confirmed && carried->first) {
      return Detected{.detection = settle(std::move(*carried->first),
                                          confirmed_summary(*confirmation), options),
                      .at = looking.holding_at,
                      .maps = {}};
    }
    dropped = !carried->first;
    state = std::move(carried->first);
  }
  auto found = search();
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  DetectionSummary summary = std::move(found->summary);
  summary.confirmation = confirmation;
  if (!found->best) {
    summary.outcome = dropped ? Outcome::kDropped : Outcome::kNone;
    summary.confirmations = state ? state->confirmations : 0;
    return Detected{.detection = {.pending = std::move(state),
                                  .loop = std::nullopt,
                                  .summary = std::move(summary)},
                    .at = looking.holding_at,
                    .maps = std::move(found->maps)};
  }
  auto& [at, best] = *found->best;
  summary.outcome = Outcome::kStarted;
  const std::int64_t matched = looking.maps[at]->kf_id[best.best];
  return Detected{
      .detection = settle(started_from(best, matched, m.kf_id[k]), std::move(summary), options),
      .at = at,
      .maps = std::move(found->maps)};
}

Result<void> check_stage(const std::string& mode, const Camera& camera, const Options& options) {
  if (mode != "monocular" && mode != "stereo" && mode != "rgbd") {
    return fail(ErrorKind::kInvalidInput, "mode: expected monocular, stereo or rgbd, got " + mode);
  }
  return check_setup(camera, options);
}

/// @brief Whether a keyframe observes a point.
/// @param m The map.
/// @param k The keyframe's row.
/// @param p The point's row.
/// @return Whether one of its keypoints observes it.
bool observes(const Map& m, std::size_t k, std::size_t p) {
  return std::ranges::any_of(m.pt_observations[p],
                             [&m, k](std::int64_t r) { return keyframe_of(m, r) == k; });
}

/// @brief Point a keypoint at a point: replace the point it held by it, or give it the point
///        (`MapPoint::Replace`, or `AddMapPoint`).
/// @param m The map.
/// @param row The keypoint's row.
/// @param id The point's id.
/// @return Nothing, or the map's error.
Result<void> take(Map& m, std::int64_t row, std::int64_t id) {
  const std::int64_t held_id = m.kp_point[static_cast<std::size_t>(row)];
  if (held_id >= 0) {
    return replace_point(m, held_id, id);
  }
  if (auto ok = observe(m, row, id); !ok) {
    return ok;
  }
  const Ids rows{point_row(m, id)};
  return refresh_points(m, rows);
}

/// @brief The keyframes moved, each moved point's keyframe, and the points moved.
using MovedBy = std::map<std::int64_t, std::size_t>;

/// @brief Move keyframes by their corrected similarities, each point with the first that sees
///        it (`slam_it.edge.loops._move_keyframes`).
/// @param m The map.
/// @param moved The keyframes' rows, in order.
/// @param corrected Each one's corrected `S_cw`, by row.
/// @return Each moved point's id to the row of the keyframe that moved it.
Result<MovedBy> move_keyframes(Map& m, std::span<const std::int64_t> moved,
                               const std::map<std::size_t, Eigen::Matrix4d>& corrected) {
  MovedBy moved_by;
  std::vector<Eigen::Vector3d> xyz = m.pt_xyz;
  for (const std::int64_t row : moved) {
    const auto r = static_cast<std::size_t>(row);
    const Eigen::Matrix4d carry = sim3::inverse(corrected.at(r)) * m.kf_pose[r];
    for (const std::int64_t at : keypoint_rows(m, r)) {
      const std::int64_t id = m.kp_point[static_cast<std::size_t>(at)];
      if (id < 0 || moved_by.contains(id)) {
        continue;
      }
      const auto p = static_cast<std::size_t>(point_row(m, id));
      xyz[p] = carry.topLeftCorner<3, 3>() * m.pt_xyz[p] + carry.topRightCorner<3, 1>();
      moved_by[id] = r;
    }
  }
  for (const std::int64_t row : moved) {
    const auto r = static_cast<std::size_t>(row);
    m.kf_pose[r] = rigid_of(corrected.at(r));
  }
  m.pt_xyz = std::move(xyz);
  Ids rows;
  rows.reserve(moved_by.size());
  for (const auto& entry : moved_by) {
    rows.push_back(point_row(m, entry.first));
  }
  if (auto ok = refresh_points(m, rows); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return moved_by;
}

/// @brief Replace the keyframe's points by the loop's it matched, or give its free keypoints
///        them (`slam_it.edge.loops._replace_matched`).
/// @param m The map.
/// @param keyframe The keyframe's id.
/// @param matched_points Each keypoint's match.
/// @return The points replaced or given.
Result<std::int64_t> replace_matched(Map& m, std::int64_t keyframe,
                                     std::span<const std::int64_t> matched_points) {
  std::int64_t count = 0;
  for (std::size_t i = 0; i < matched_points.size(); ++i) {
    const std::int64_t id = matched_points[i];
    auto k = checked_keyframe_row(m, keyframe);
    if (!k) {
      return std::unexpected(std::move(k.error()));
    }
    const std::int64_t p = point_row(m, id);
    if (id < 0 || p < 0 || observes(m, *k, static_cast<std::size_t>(p))) {
      continue;  // the keyframe sees it already
    }
    if (auto ok = take(m, m.kf_start[*k] + static_cast<std::int64_t>(i), id); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    ++count;
  }
  return count;
}

/// @brief The points two keyframes share.
/// @param m The map.
/// @param a One keyframe's row.
/// @param b The other's.
/// @return The count.
std::int64_t weight(const Map& m, std::size_t a, std::size_t b) {
  std::set<std::int64_t> own;
  for (const std::int64_t row : keypoint_rows(m, a)) {
    if (const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)]; id >= 0) {
      own.insert(id);
    }
  }
  std::set<std::int64_t> shared;
  for (const std::int64_t row : keypoint_rows(m, b)) {
    if (const std::int64_t id = m.kp_point[static_cast<std::size_t>(row)];
        id >= 0 && own.contains(id)) {
      shared.insert(id);
    }
  }
  return static_cast<std::int64_t>(shared.size());
}

/// @brief The essential graph's edges, as rows, and each one's measured relative similarity.
struct Graph {
  /// @brief The edges.
  std::vector<std::array<std::int64_t, 2>> edges;
  /// @brief Each one's `S_i S_j^-1`.
  std::vector<Eigen::Matrix4d> relatives;
};

/// @brief Collect the essential graph's edges (`slam_it.edge.loops._essential_edges`).
/// @param m The map.
/// @param start Each keyframe's `S_iw` as the graph starts.
/// @param before Its pose before the correction.
/// @param loop_links Each moved keyframe's links new across the loop.
/// @param ends The current and matched keyframes' rows.
/// @param options The settings.
/// @return The edges.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (start, before), as Python's
Result<Graph> essential_edges(const Map& m, std::span<const Eigen::Matrix4d> start,
                              std::span<const Eigen::Matrix4d> before,
                              const std::map<std::size_t, std::set<std::int64_t>>& loop_links,
                              std::pair<std::size_t, std::size_t> ends, const Options& options) {
  const auto [current, matched] = ends;
  Graph out;
  std::set<std::pair<std::int64_t, std::int64_t>> inserted;
  const auto add = [&out](std::int64_t i, std::int64_t j, std::span<const Eigen::Matrix4d> poses) {
    out.edges.emplace_back(std::array<std::int64_t, 2>{i, j});
    out.relatives.emplace_back(poses[static_cast<std::size_t>(i)] *
                               sim3::inverse(poses[static_cast<std::size_t>(j)]));
  };
  for (const auto& [i, theirs] : loop_links) {
    for (const std::int64_t j : theirs) {
      const bool ends_of_loop = i == current && std::cmp_equal(j, matched);
      if (!ends_of_loop &&
          weight(m, i, static_cast<std::size_t>(j)) < options.loops.essential_points) {
        continue;
      }
      add(static_cast<std::int64_t>(i), j, start);
      inserted.insert({std::min<std::int64_t>(static_cast<std::int64_t>(i), j),
                       std::max<std::int64_t>(static_cast<std::int64_t>(i), j)});
    }
  }
  for (std::size_t i = 0; i < keyframe_count(m); ++i) {
    const auto row = static_cast<std::int64_t>(i);
    const std::int64_t parent = keyframe_row(m, m.kf_parent[i]);
    if (m.kf_parent[i] >= 0 && parent >= 0) {
      add(row, parent, before);
    }
    for (const auto& [older, newer] : m.loop_edges) {
      if (newer == m.kf_id[i]) {
        add(row, keyframe_row(m, older), before);
      }
    }
    auto linked = links(m, row, options);
    if (!linked) {
      return std::unexpected(std::move(linked.error()));
    }
    for (const Link& link : *linked) {
      const auto n = static_cast<std::size_t>(link.row);
      if (link.weight < options.loops.essential_points || link.row == parent ||
          m.kf_parent[n] == m.kf_id[i] || m.kf_id[n] > m.kf_id[i] ||
          inserted.contains({std::min(row, link.row), std::max(row, link.row)})) {
        continue;
      }
      add(row, link.row, before);
    }
  }
  return out;
}

/// @brief The essential graph's solution: each keyframe's `S_iw`, the edges and the solve.
struct Essential {
  /// @brief Each keyframe's optimised `S_iw`, by row.
  std::vector<Eigen::Matrix4d> poses;
  /// @brief The graph's edges.
  std::int64_t edges{0};
  /// @brief The solve.
  graphs::Summary summary;
};

/// @brief Optimise the essential graph from the loop's start (`slam_it.edge.loops
///        .essential_graph`, `OptimizeEssentialGraph`), the `fixed` keyframes held: a loop's
///        matched one, a merge's welding windows.
/// @param m The map.
/// @param start Each keyframe's `S_iw` the graph starts from.
/// @param before Each one's pose before the correction.
/// @param loop_links Each moved keyframe's links new across the loop.
/// @param ends The current and matched keyframes' rows.
/// @param fixed The rows held fixed.
/// @param fixed_scale Optimise SE(3), not Sim(3).
/// @param options The settings.
/// @return The solution.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (start, before), as Python's
Result<Essential> essential_graph(const Map& m, std::span<const Eigen::Matrix4d> start,
                                  std::span<const Eigen::Matrix4d> before,
                                  const std::map<std::size_t, std::set<std::int64_t>>& loop_links,
                                  std::pair<std::size_t, std::size_t> ends,
                                  std::span<const std::int64_t> fixed, bool fixed_scale,
                                  const Options& options) {
  auto graph = essential_edges(m, start, before, loop_links, ends, options);
  if (!graph) {
    return std::unexpected(std::move(graph.error()));
  }
  std::vector<Eigen::Matrix4d> world;
  world.reserve(start.size());
  for (const Eigen::Matrix4d& s : start) {
    world.push_back(sim3::inverse(s));
  }
  graphs::IndexPairs edges(static_cast<Eigen::Index>(graph->edges.size()), 2);
  for (std::size_t e = 0; e < graph->edges.size(); ++e) {
    edges.row(static_cast<Eigen::Index>(e)) << graph->edges[e][0], graph->edges[e][1];
  }
  auto solved = graphs::optimize_pose_graph(
      world, edges, graph->relatives, fixed, !fixed_scale,
      {.max_iterations = static_cast<int>(options.loops.essential_iterations),
       .initial_damping = options.loops.essential_damping});
  if (!solved) {
    return std::unexpected(std::move(solved.error()));
  }
  Essential out{.poses = {},
                .edges = static_cast<std::int64_t>(graph->edges.size()),
                .summary = solved->summary};
  out.poses.reserve(solved->poses.size());
  for (const Eigen::Matrix4d& pose : solved->poses) {
    out.poses.push_back(sim3::inverse(pose));
  }
  return out;
}

/// @brief Set each keyframe to its optimised pose, and move each point with its keyframe
///        (`slam_it.edge.loops._move_by_graph`).
/// @param m The map.
/// @param start Each keyframe's `S_iw` the graph started from.
/// @param optimised Each one's optimised `S_iw`.
/// @param moved_by Each point the loop moved, and the keyframe that moved it.
/// @return Nothing, or the map's error.
Result<void> move_by_graph(Map& m, std::span<const Eigen::Matrix4d> start,
                           std::span<const Eigen::Matrix4d> optimised, const MovedBy& moved_by) {
  std::vector<Eigen::Vector3d> xyz = m.pt_xyz;
  for (std::size_t p = 0; p < point_count(m); ++p) {
    std::int64_t r = -1;
    if (const auto found = moved_by.find(m.pt_id[p]); found != moved_by.end()) {
      r = static_cast<std::int64_t>(found->second);
    } else {
      r = keyframe_row(m, m.pt_reference_kf[p]);
      if (r < 0) {
        continue;
      }
    }
    const auto k = static_cast<std::size_t>(r);
    const Eigen::Matrix4d carry = sim3::inverse(optimised[k]) * start[k];
    xyz[p] = carry.topLeftCorner<3, 3>() * m.pt_xyz[p] + carry.topRightCorner<3, 1>();
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    m.kf_pose[k] = rigid_of(optimised[k]);
  }
  m.pt_xyz = std::move(xyz);
  Ids rows(point_count(m));
  std::ranges::iota(rows, std::int64_t{0});
  return refresh_points(m, rows);
}

// --- merging ----------------------------------------------------------------------------------

/// @brief Bundle-adjust the whole map after a correction, if `loops.global_ba` asks.
/// @param m The map, changed in place.
/// @param camera The camera.
/// @param options The settings.
/// @return The solve's summary, or none with `loops.global_ba` off; global BA's failure.
Result<std::optional<graphs::Summary>> global_ba_after(Map& m, const Camera& camera,
                                                       const Options& options);

/// @brief How a correction joins its two keyframes: the edge it adds, and its ends' ids.
struct Join {
  /// @brief The map's operation that adds the edge (`add_loop_edge`, `add_merge_edge`).
  Result<void> (*add)(Map&, std::int64_t, std::int64_t){nullptr};
  /// @brief One end's id.
  std::int64_t first{0};
  /// @brief The other's.
  std::int64_t second{0};
};

/// @brief A correction's last steps (`CorrectLoop`'s and `MergeLocal`'s): each keyframe to its
///        optimised pose and each point with its keyframe, the two keyframes joined, then global
///        BA if `loops.global_ba` asks.
/// @param m The map, changed in place.
/// @param start Each keyframe's `S_iw` the graph started from.
/// @param essential The essential graph's solution.
/// @param moved_by Each point the correction moved, and the keyframe that moved it.
/// @param join The edge joining the two keyframes.
/// @param camera The camera.
/// @param options The settings.
/// @return Global BA's solve, or none; the map's or global BA's failure.
Result<std::optional<graphs::Summary>> finish_correction(
    Map& m, std::span<const Eigen::Matrix4d> start, const Essential& essential,
    const MovedBy& moved_by, const Join& join, const Camera& camera, const Options& options) {
  if (auto ok = move_by_graph(m, start, essential.poses, moved_by); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = join.add(m, join.first, join.second); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return global_ba_after(m, camera, options);
}

/// @brief Try the other maps' candidates (`slam_it.edge.loops._search_merge`), each against its
///        own map, which holds none of the keyframe's links.
/// @param m The active map.
/// @param db Its database.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param others The other maps.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The best that passed, and how the search went.
Result<RegionSearch> search_merge(const Map& m, const Database& db, const Camera& camera,
                                  std::size_t k, std::span<const MapRef> others, bool fixed_scale,
                                  const Options& options) {
  const auto words = db.keyframes.find(m.kf_id[k]);
  if (words == db.keyframes.end()) {
    return fail(ErrorKind::kInvalidInput,
                "database: keyframe " + std::to_string(m.kf_id[k]) + " is not in it");
  }
  auto candidates = merge_candidates(words->second.vector, others, options);
  if (!candidates) {
    return std::unexpected(std::move(candidates.error()));
  }
  RegionSearch out{.best = std::nullopt,
                   .summary = searched_summary(candidates->sharing, candidates->scored),
                   .maps = candidates->maps};
  const std::set<std::int64_t> none;
  for (std::size_t i = 0; i < candidates->keyframes.size(); ++i) {
    const auto at = static_cast<std::size_t>(
        std::ranges::find_if(
            others, [&](const MapRef& other) { return other.number == candidates->maps[i]; }) -
        others.begin());
    const MapRef& other = others[at];
    auto tried = try_candidate(m, db, camera, k, {candidates->keyframes[i], candidates->scores[i]},
                               {.map = other.map, .database = other.database, .own = false}, none,
                               fixed_scale, options);
    if (!tried) {
      return std::unexpected(std::move(tried.error()));
    }
    out.summary.candidates.push_back(tried->first);
    if (tried->second && (!out.best || tried->second->reprojected > out.best->second.reprojected)) {
      out.best = std::pair{at, std::move(*tried->second)};
    }
  }
  return out;
}

/// @brief A pose scaled into a similarity: `[[s R, s t], [0, 1]]`.
/// @param pose The pose.
/// @param scale `s`.
/// @return The similarity.
Eigen::Matrix4d scaled(const Eigen::Matrix4d& pose, double scale) {
  Eigen::Matrix4d out = pose;
  out.topRows<3>() *= scale;
  return out;
}

/// @brief Carry a map into another world by a similarity `G` on its own
///        (`slam_it.edge.loops._moved_into`): each keyframe's pose `rigid_of(T G^-1)`, each
///        point `G p`, its archive with it, every observed point refreshed.
/// @param m The map, changed in place.
/// @param to_world `G`.
/// @return Nothing, or the map's error.
Result<void> moved_into(Map& m, const Eigen::Matrix4d& to_world) {
  const Eigen::Matrix4d back = sim3::inverse(to_world);
  const double scale = scale_of(back);
  const auto carried = [&to_world](const Eigen::Vector3d& x) -> Eigen::Vector3d {
    return to_world.topLeftCorner<3, 3>() * x + to_world.topRightCorner<3, 1>();
  };
  for (Eigen::Matrix4d& pose : m.kf_pose) {
    pose = rigid_of(pose * back);
  }
  for (Eigen::Matrix4d& pose : m.gone_kf_pose) {
    pose = rigid_of(pose * back);
  }
  for (Eigen::Vector3d& x : m.pt_xyz) {
    x = carried(x);
  }
  for (Eigen::Vector3d& x : m.gone_pt_xyz) {
    x = carried(x);
  }
  for (Eigen::Matrix4d& relative_pose : m.gone_kf_relative) {
    relative_pose.topRightCorner<3, 1>() /= scale;
  }
  Ids seen;
  for (std::size_t p = 0; p < point_count(m); ++p) {
    if (!m.pt_observations[p].empty()) {
      seen.push_back(static_cast<std::int64_t>(p));
    }
  }
  return refresh_points(m, seen);
}

/// @brief Ids moved on by an offset, -1 (none) kept.
/// @param ids The ids.
/// @param offset The offset.
/// @return The ids moved on.
Ids offset_ids(std::span<const std::int64_t> ids, std::int64_t offset) {
  Ids out;
  out.reserve(ids.size());
  for (const std::int64_t id : ids) {
    out.push_back(id >= 0 ? id + offset : id);
  }
  return out;
}

/// @brief Append one column after another.
/// @tparam T The column's item.
/// @param into The column appended to.
/// @param more The column appended.
template <typename T>
void append(std::vector<T>& into, std::span<const T> more) {
  into.insert(into.end(), more.begin(), more.end());
}

/// @brief Take `other`'s keyframes, keypoints, points and archive into `into`, after its own
///        (`slam_it.edge.loops._absorb`): every id of `other`'s moved on by its offset.
/// @param into The map taking them, changed in place.
/// @param other The map taken, already in `into`'s world.
/// @param keyframe_offset The keyframe ids' offset.
/// @param point_offset The point ids' offset.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (keyframe_offset, point_offset)
void absorb(Map& into, const Map& other, std::int64_t keyframe_offset, std::int64_t point_offset) {
  const std::int64_t keypoints = into.kf_start.back();
  append<std::int64_t>(into.kf_id, offset_ids(other.kf_id, keyframe_offset));
  append<double>(into.kf_time, other.kf_time);
  append<Eigen::Matrix4d>(into.kf_pose, other.kf_pose);
  append<std::int64_t>(into.kf_parent, offset_ids(other.kf_parent, keyframe_offset));
  for (std::size_t k = 1; k < other.kf_start.size(); ++k) {
    into.kf_start.push_back(other.kf_start[k] + keypoints);
  }
  append<std::array<double, 2>>(into.kp_xy, other.kp_xy);
  append<std::int64_t>(into.kp_level, other.kp_level);
  append<std::int64_t>(into.kp_angle, other.kp_angle);
  append<foundations::orb::Descriptor>(into.kp_descriptor, other.kp_descriptor);
  append<Eigen::Vector3d>(into.kp_bearing, other.kp_bearing);
  append<double>(into.kp_depth, other.kp_depth);
  append<double>(into.kp_right, other.kp_right);
  append<std::int64_t>(into.kp_point, offset_ids(other.kp_point, point_offset));
  append<std::int64_t>(into.pt_id, offset_ids(other.pt_id, point_offset));
  append<Eigen::Vector3d>(into.pt_xyz, other.pt_xyz);
  append<foundations::orb::Descriptor>(into.pt_descriptor, other.pt_descriptor);
  append<Eigen::Vector3d>(into.pt_normal, other.pt_normal);
  append<double>(into.pt_min_distance, other.pt_min_distance);
  append<double>(into.pt_max_distance, other.pt_max_distance);
  append<std::int64_t>(into.pt_visible, other.pt_visible);
  append<std::int64_t>(into.pt_found, other.pt_found);
  append<std::int64_t>(into.pt_first_kf, offset_ids(other.pt_first_kf, keyframe_offset));
  append<std::int64_t>(into.pt_reference_kf, offset_ids(other.pt_reference_kf, keyframe_offset));
  into.next_kf_id = keyframe_offset + other.next_kf_id;
  into.next_pt_id = point_offset + other.next_pt_id;
  append<std::int64_t>(into.gone_pt_id, offset_ids(other.gone_pt_id, point_offset));
  append<Eigen::Vector3d>(into.gone_pt_xyz, other.gone_pt_xyz);
  append<std::int64_t>(into.gone_pt_how, other.gone_pt_how);
  append<std::int64_t>(into.gone_kf_id, offset_ids(other.gone_kf_id, keyframe_offset));
  append<double>(into.gone_kf_time, other.gone_kf_time);
  append<Eigen::Matrix4d>(into.gone_kf_pose, other.gone_kf_pose);
  append<std::int64_t>(into.gone_kf_parent, offset_ids(other.gone_kf_parent, keyframe_offset));
  append<Eigen::Matrix4d>(into.gone_kf_relative, other.gone_kf_relative);
  for (const auto& [older, newer] : other.loop_edges) {
    into.loop_edges.push_back({older + keyframe_offset, newer + keyframe_offset});
  }
  for (const auto& [older, newer] : other.merge_edges) {
    into.merge_edges.push_back({older + keyframe_offset, newer + keyframe_offset});
  }
  index_observations(into);
}

/// @brief Re-root the active side's spanning tree at its current keyframe, hung under the
///        matched one (`slam_it.edge.loops._rehang`).
/// @param m The merged map, changed in place.
/// @param current The current keyframe's id.
/// @param matched The matched keyframe's id.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (current, matched), as Python's
void rehang(Map& m, std::int64_t current, std::int64_t matched) {
  Ids chain{current};
  for (std::int64_t parent = m.kf_parent[static_cast<std::size_t>(keyframe_row(m, current))];
       parent >= 0; parent = m.kf_parent[static_cast<std::size_t>(keyframe_row(m, parent))]) {
    chain.push_back(parent);
  }
  for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
    m.kf_parent[static_cast<std::size_t>(keyframe_row(m, chain[i + 1]))] = chain[i];
  }
  m.kf_parent[static_cast<std::size_t>(keyframe_row(m, current))] = matched;
}

/// @brief A welding window's keyframe rows (`slam_it.edge.loops._weld_window`): the keyframe and
///        its best `loops.weld_neighbours` links; while it holds fewer, each member's best
///        `loops.weld_widen` links not yet in it, at most `loops.weld_rounds` times.
/// @param m The map.
/// @param k The keyframe's row.
/// @param options The settings.
/// @return The rows, in the order taken in.
Result<Ids> weld_window(const Map& m, std::size_t k, const Options& options) {
  const LoopOptions& settings = options.loops;
  auto first = link_rows(m, k, options, settings.weld_neighbours);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  Ids window{static_cast<std::int64_t>(k)};
  window.insert(window.end(), first->begin(), first->end());
  for (std::int64_t round = 0; round < settings.weld_rounds; ++round) {
    if (std::cmp_greater_equal(window.size(), settings.weld_neighbours)) {
      break;
    }
    std::set<std::int64_t> inside(window.begin(), window.end());
    Ids added;
    for (const std::int64_t r : window) {
      auto theirs = link_rows(m, static_cast<std::size_t>(r), options, settings.weld_widen);
      if (!theirs) {
        return std::unexpected(std::move(theirs.error()));
      }
      for (const std::int64_t other : *theirs) {
        if (inside.insert(other).second) {
          added.push_back(other);
        }
      }
    }
    if (added.empty()) {
      break;
    }
    window.insert(window.end(), added.begin(), added.end());
  }
  return window;
}

/// @brief A welding BA's problem (`slam_it.edge.loops._Bundle`): its cameras and points, and
///        each observation of them.
struct Weld {
  /// @brief The cameras' keyframe rows: the windows', ascending, then those held outside them.
  Ids cameras;
  /// @brief The held cameras, as indices into `cameras`.
  Ids fixed;
  /// @brief The points' rows, ascending.
  Ids points;
  /// @brief Each observation's keypoint row, ascending.
  Ids rows;
  /// @brief Each observation's camera, an index into `cameras`.
  Ids camera;
  /// @brief Each observation's point, an index into `points`.
  Ids point;
  /// @brief Each observation's pixel.
  std::vector<std::array<double, 2>> xy;
  /// @brief Its right column, or NaN.
  std::vector<double> right;
  /// @brief Its level's variance.
  std::vector<double> variance;
  /// @brief Its chi-square threshold.
  std::vector<double> limits;
};

/// @brief Gather a welding BA's problem (`slam_it.edge.loops._weld_problem`): both windows'
///        keyframes free with every point they observe; the matched map's keyframes outside
///        them that observe those points held, or else the matched window.
/// @param m The merged map.
/// @param window The active side's window's rows.
/// @param matched_window The matched side's.
/// @param keyframe_offset The first keyframe id that came from the active map.
/// @param options The settings.
/// @return The problem.
Weld weld_problem(const Map& m, std::span<const std::int64_t> window,
                  std::span<const std::int64_t> matched_window, std::int64_t keyframe_offset,
                  const Options& options) {
  std::set<std::int64_t> free(window.begin(), window.end());
  free.insert(matched_window.begin(), matched_window.end());
  Weld out;
  out.cameras.assign(free.begin(), free.end());
  out.points = observed_by(m, out.cameras);
  Ids observing;
  for (const std::int64_t p : out.points) {
    const auto& seen = m.pt_observations[static_cast<std::size_t>(p)];
    observing.insert(observing.end(), seen.begin(), seen.end());
  }
  std::ranges::sort(observing);
  std::set<std::int64_t> outside;
  for (const std::int64_t row : observing) {
    const auto k = static_cast<std::int64_t>(keyframe_of(m, row));
    if (!free.contains(k)) {
      outside.insert(k);
    }
  }
  Ids held;
  for (const std::int64_t k : outside) {
    if (m.kf_id[static_cast<std::size_t>(k)] < keyframe_offset) {
      held.push_back(k);
    }
  }
  out.cameras.insert(out.cameras.end(), held.begin(), held.end());
  Ids fixed = held;
  if (fixed.empty()) {
    fixed.assign(matched_window.begin(), matched_window.end());
    std::ranges::sort(fixed);
  }
  std::map<std::int64_t, std::int64_t> index;
  for (std::size_t i = 0; i < out.cameras.size(); ++i) {
    index[out.cameras[i]] = static_cast<std::int64_t>(i);
  }
  for (const std::int64_t k : fixed) {
    out.fixed.push_back(index.at(k));
  }
  for (const std::int64_t row : observing) {
    const auto k = static_cast<std::int64_t>(keyframe_of(m, row));
    const auto found = index.find(k);
    if (found == index.end()) {
      continue;
    }
    const auto r = static_cast<std::size_t>(row);
    out.rows.push_back(row);
    out.camera.push_back(found->second);
    out.point.push_back(std::ranges::lower_bound(out.points, point_row(m, m.kp_point[r])) -
                        out.points.begin());
    out.xy.push_back(m.kp_xy[r]);
    out.right.push_back(m.kp_right[r]);
    out.variance.push_back(sigma2(m, m.kp_level[r]));
    out.limits.push_back(threshold(m.kp_right[r], options));
  }
  return out;
}

/// @brief The welding BA's state: each camera's `T_world_cam`, each point.
struct WeldState {
  /// @brief The cameras' poses.
  std::vector<Eigen::Matrix4d> poses;
  /// @brief The points.
  std::vector<Eigen::Vector3d> xyz;
};

/// @brief Solve the welding BA on the marked observations (`slam_it.edge.loops._weld_solve`):
///        only the cameras and points they reach take part, with the held cameras.
/// @param problem The problem.
/// @param camera The camera.
/// @param state The state, updated in place.
/// @param use Which observations take part.
/// @param robust With a Huber loss at each one's threshold's root.
/// @param iterations The solve's iterations.
/// @return The solve's summary; GTSAM's failure.
Result<graphs::Summary> weld_solve(const Weld& problem, const Camera& camera, WeldState& state,
                                   const std::vector<bool>& use, bool robust,
                                   std::int64_t iterations) {
  std::set<std::int64_t> cameras(problem.fixed.begin(), problem.fixed.end());
  std::set<std::int64_t> points;
  for (std::size_t i = 0; i < use.size(); ++i) {
    if (use[i]) {
      cameras.insert(problem.camera[i]);
      points.insert(problem.point[i]);
    }
  }
  const Ids used_cameras(cameras.begin(), cameras.end());
  const Ids used_points(points.begin(), points.end());
  const auto place = [](const Ids& in, std::int64_t value) {
    return static_cast<std::int64_t>(std::ranges::lower_bound(in, value) - in.begin());
  };
  Ids which;
  Ids whose;
  std::vector<double> right;
  std::vector<double> information;
  std::vector<double> huber;
  std::vector<std::array<double, 2>> pixels;
  for (std::size_t i = 0; i < use.size(); ++i) {
    if (!use[i]) {
      continue;
    }
    which.push_back(place(used_cameras, problem.camera[i]));
    whose.push_back(place(used_points, problem.point[i]));
    pixels.push_back(problem.xy[i]);
    right.push_back(problem.right[i]);
    information.push_back(1.0 / problem.variance[i]);
    if (robust) {
      huber.push_back(std::sqrt(problem.limits[i]));
    }
  }
  const graphs::Pairs xy = pairs_of(pixels);
  std::vector<Eigen::Matrix4d> poses;
  for (const std::int64_t c : used_cameras) {
    poses.push_back(state.poses[static_cast<std::size_t>(c)]);
  }
  graphs::Points start(static_cast<Eigen::Index>(used_points.size()), 3);
  for (std::size_t p = 0; p < used_points.size(); ++p) {
    start.row(static_cast<Eigen::Index>(p)) =
        state.xyz[static_cast<std::size_t>(used_points[p])].transpose();
  }
  Ids fixed;
  for (const std::int64_t c : problem.fixed) {
    fixed.push_back(place(used_cameras, c));
  }
  const graphs::Observations observations{.cameras = which,
                                          .point_ids = whose,
                                          .pixels = xy,
                                          .right = right,
                                          .information = information,
                                          .huber = huber};
  auto adjusted = graphs::bundle_adjust(poses, start, observations, camera.model, camera.intrinsics,
                                        focal(camera) * camera.baseline, fixed,
                                        {.max_iterations = static_cast<int>(iterations)});
  if (!adjusted) {
    return std::unexpected(std::move(adjusted.error()));
  }
  for (std::size_t c = 0; c < used_cameras.size(); ++c) {
    state.poses[static_cast<std::size_t>(used_cameras[c])] = adjusted->poses[c];
  }
  for (std::size_t p = 0; p < used_points.size(); ++p) {
    state.xyz[static_cast<std::size_t>(used_points[p])] =
        adjusted->points.row(static_cast<Eigen::Index>(p)).transpose();
  }
  return adjusted->summary;
}

/// @brief Each observation's chi-square error at a state (`slam_it.edge.loops._weld_errors`).
/// @param problem The problem.
/// @param camera The camera.
/// @param state The state.
/// @return The errors; the residuals' error.
Result<std::vector<double>> weld_errors(const Weld& problem, const Camera& camera,
                                        const WeldState& state) {
  std::vector<Eigen::Vector3d> local;
  local.reserve(problem.rows.size());
  for (std::size_t i = 0; i < problem.rows.size(); ++i) {
    const Eigen::Matrix4d to_camera_pose =
        sim3::inverse(state.poses[static_cast<std::size_t>(problem.camera[i])]);
    local.push_back(
        to_camera(to_camera_pose, state.xyz[static_cast<std::size_t>(problem.point[i])]));
  }
  return chi2(camera, local, problem.xy, problem.right, problem.variance);
}

/// @brief Bundle-adjust the welding windows (`slam_it.edge.loops._weld`): the problem solved
///        as ORB-SLAM's local BA solves its own, a Huber loss first, the outliers left out of a
///        second solve with none, those still failing forgotten, the points refreshed.
/// @param m The merged map, changed in place.
/// @param camera The camera.
/// @param window The active side's window's rows.
/// @param matched_window The matched side's.
/// @param keyframe_offset The first keyframe id that came from the active map.
/// @param options The settings.
/// @return How it went; GTSAM's or the map's failure.
Result<WeldSummary> weld(Map& m, const Camera& camera, std::span<const std::int64_t> window,
                         std::span<const std::int64_t> matched_window, std::int64_t keyframe_offset,
                         const Options& options) {
  const Weld problem = weld_problem(m, window, matched_window, keyframe_offset, options);
  WeldState state;
  for (const std::int64_t r : problem.cameras) {
    state.poses.push_back(sim3::inverse(m.kf_pose[static_cast<std::size_t>(r)]));
  }
  for (const std::int64_t p : problem.points) {
    state.xyz.push_back(m.pt_xyz[static_cast<std::size_t>(p)]);
  }
  const std::vector<bool> everything(problem.rows.size(), true);
  auto first = weld_solve(problem, camera, state, everything, true, options.adjustment.iterations);
  if (!first) {
    return std::unexpected(std::move(first.error()));
  }
  auto errors = weld_errors(problem, camera, state);
  if (!errors) {
    return std::unexpected(std::move(errors.error()));
  }
  std::vector<bool> inliers(problem.rows.size());
  std::int64_t outliers = 0;
  for (std::size_t i = 0; i < inliers.size(); ++i) {
    inliers[i] = (*errors)[i] <= problem.limits[i];
    outliers += inliers[i] ? 0 : 1;
  }
  std::optional<graphs::Summary> second;
  if (std::ranges::any_of(inliers, [](bool inlier) { return inlier; })) {
    auto solved =
        weld_solve(problem, camera, state, inliers, false, options.adjustment.refine_iterations);
    if (!solved) {
      return std::unexpected(std::move(solved.error()));
    }
    second = *solved;
  }
  auto last = weld_errors(problem, camera, state);
  if (!last) {
    return std::unexpected(std::move(last.error()));
  }
  Ids failing;
  for (std::size_t i = 0; i < problem.rows.size(); ++i) {
    if (!((*last)[i] <= problem.limits[i])) {
      failing.push_back(problem.rows[i]);
    }
  }
  const std::set<std::int64_t> held(problem.fixed.begin(), problem.fixed.end());
  for (std::size_t i = 0; i < problem.cameras.size(); ++i) {
    if (!held.contains(static_cast<std::int64_t>(i))) {
      m.kf_pose[static_cast<std::size_t>(problem.cameras[i])] = sim3::inverse(state.poses[i]);
    }
  }
  for (std::size_t p = 0; p < problem.points.size(); ++p) {
    m.pt_xyz[static_cast<std::size_t>(problem.points[p])] = state.xyz[p];
  }
  if (auto ok = refresh_points(m, problem.points); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = forget_observations(m, failing); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return WeldSummary{.keyframes = static_cast<std::int64_t>(problem.cameras.size() - held.size()),
                     .held = static_cast<std::int64_t>(held.size()),
                     .points = static_cast<std::int64_t>(problem.points.size()),
                     .observations = static_cast<std::int64_t>(problem.rows.size()),
                     .outliers = outliers,
                     .forgotten = static_cast<std::int64_t>(failing.size()),
                     .first = *first,
                     .second = second};
}

}  // namespace

double scale_of(const Eigen::Matrix4d& similarity) { return similarity.col(0).head<3>().norm(); }

Eigen::Matrix4d rigid_of(const Eigen::Matrix4d& similarity) {
  const double s = scale_of(similarity);
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = similarity.topLeftCorner<3, 3>() / s;
  out.topRightCorner<3, 1>() = similarity.topRightCorner<3, 1>() / s;
  return out;
}

Result<std::vector<std::int64_t>> search_by_similarity(const Map& m, const Camera& camera,
                                                       std::int64_t k,
                                                       const Eigen::Matrix4d& similarity,
                                                       std::span<const std::int64_t> points,
                                                       Window window, const Options& options) {
  auto row = keyframe_row_in(m, k, "k");
  if (!row) {
    return std::unexpected(std::move(row.error()));
  }
  if (std::ranges::any_of(points, [&m](std::int64_t p) {
        return p < 0 || std::cmp_greater_equal(p, point_count(m));
      })) {
    return fail(ErrorKind::kInvalidInput, "points: expected point rows");
  }
  return search(m, camera, *row, similarity, m, points, window, options);
}

Result<std::vector<std::int64_t>> match_by_word(const Map& m, const Database& db, std::int64_t k1,
                                                std::int64_t k2, const Options& options) {
  auto rows = keyframe_rows_in(m, k1, k2);
  if (!rows) {
    return std::unexpected(std::move(rows.error()));
  }
  return match_words(m, db, rows->first, {.map = &m, .database = &db, .own = true}, rows->second,
                     options);
}

Result<std::vector<std::int64_t>> group_points(const Map& m, std::int64_t matched,
                                               const std::set<std::int64_t>& linked,
                                               const Options& options) {
  auto row = keyframe_row_in(m, matched, "matched");
  if (!row) {
    return std::unexpected(std::move(row.error()));
  }
  if (std::ranges::any_of(linked, [&m](std::int64_t r) {
        return r < 0 || std::cmp_greater_equal(r, keyframe_count(m));
      })) {
    return fail(ErrorKind::kInvalidInput, "linked: expected keyframe rows");
  }
  return group_of(m, *row, linked, options);
}

std::int64_t ransac_budget(std::int64_t matches, const Options& options) {
  const LoopOptions& settings = options.loops;
  if (settings.min_inliers >= matches) {
    return 1;
  }
  const double bound = ransac::required_iterations(
      static_cast<double>(settings.min_inliers) / static_cast<double>(matches), sim3::kSampleSize,
      settings.confidence);
  return std::max<std::int64_t>(1,
                                std::min<std::int64_t>(static_cast<std::int64_t>(std::ceil(bound)),
                                                       settings.ransac_iterations));
}

Result<std::optional<Refined>> refine(const Map& m, const Camera& camera, std::int64_t k1,
                                      std::int64_t k2, const Eigen::Matrix4d& similarity,
                                      std::span<const std::int64_t> matched, bool fixed_scale,
                                      const Options& options) {
  return keyframe_rows_in(m, k1, k2).and_then(
      [&](std::pair<std::size_t, std::size_t> rows) -> Result<std::optional<Refined>> {
        const std::int64_t size = m.kf_start[rows.first + 1] - m.kf_start[rows.first];
        if (std::cmp_not_equal(matched.size(), size)) {
          return fail(ErrorKind::kInvalidInput, "matched: expected one id per keypoint of k1");
        }
        return refined(m, camera, rows.first, m, rows.second, similarity, matched, fixed_scale,
                       options);
      });
}

const char* name(Outcome outcome) {
  switch (outcome) {
    case Outcome::kSkipped:
      return "skipped";
    case Outcome::kNone:
      return "none";
    case Outcome::kStarted:
      return "started";
    case Outcome::kConfirmed:
      return "confirmed";
    case Outcome::kClosed:
      return "closed";
    case Outcome::kDropped:
      return "dropped";
    case Outcome::kMerged:
      return "merged";
    case Outcome::kCount:
      break;
  }
  return "unknown";
}

std::vector<std::int64_t> held(const Pending& pending) {
  std::vector<std::int64_t> out{std::min(pending.matched, pending.last),
                                std::max(pending.matched, pending.last)};
  if (out[0] == out[1]) {
    out.pop_back();
  }
  return out;
}

Result<Detection> detect(const Map& m, const Database& db, const Camera& camera,
                         std::int64_t keyframe, const std::optional<Pending>& pending,
                         const std::string& mode, const Options& options) {
  auto stage = stage_of(m, keyframe, mode, camera, options);
  if (!stage) {
    return std::unexpected(std::move(stage.error()));
  }
  if (stage->small) {
    return Detection{.pending = pending,
                     .loop = std::nullopt,
                     .summary = skipped_summary(pending ? pending->confirmations : 0)};
  }
  const Looking looking{.pending = pending,
                        .holding = {.map = &m, .database = &db, .own = true},
                        .holding_at = 0,
                        .maps = {&m}};
  return detect_region(
             m, camera, stage->k, looking,
             [&] {
               return search_candidates(m, db, camera, stage->k, stage->fixed_scale, options);
             },
             stage->fixed_scale, options)
      .transform([](Detected found) { return std::move(found.detection); });
}

Result<std::int64_t> fuse_by_similarity(Map& m, const Camera& camera, std::int64_t k,
                                        const Eigen::Matrix4d& similarity,
                                        std::span<const std::int64_t> point_ids,
                                        const Options& options) {
  auto row = keyframe_row_in(m, k, "k");
  if (!row) {
    return std::unexpected(std::move(row.error()));
  }
  const std::size_t kk = *row;
  auto grid = grid_of(m, kk);
  if (!grid) {
    return std::unexpected(std::move(grid.error()));
  }
  std::int64_t fused = 0;
  for (const std::int64_t id : point_ids) {
    const std::int64_t p = point_row(m, id);
    if (p < 0 || observes(m, kk, static_cast<std::size_t>(p))) {
      continue;
    }
    const std::vector<std::int64_t> one{p};
    auto seen = in_view(m, camera, similarity, one, options);
    if (!seen) {
      return std::unexpected(std::move(seen.error()));
    }
    if (seen->rows.empty()) {
      continue;
    }
    const std::int64_t level = seen->levels[0];
    auto found = near(
        *grid, seen->pixels[0],
        options.loops.fuse_window * std::pow(m.scale_factor, static_cast<double>(level)), level);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    if (found->empty()) {
      continue;
    }
    std::int64_t best = -1;
    std::int64_t best_distance = std::numeric_limits<std::int64_t>::max();
    for (const std::int64_t j : *found) {
      const std::int64_t d =
          foundations::orb::distance(m.pt_descriptor[static_cast<std::size_t>(p)],
                                     m.kp_descriptor[static_cast<std::size_t>(m.kf_start[kk] + j)]);
      if (d < best_distance) {
        best_distance = d;
        best = j;
      }
    }
    if (best_distance > options.matching.th_low) {
      continue;
    }
    if (auto ok = take(m, m.kf_start[kk] + best, id); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
    ++fused;
  }
  return fused;
}

namespace {

Result<std::optional<graphs::Summary>> global_ba_after(Map& m, const Camera& camera,
                                                       const Options& options) {
  if (!options.loops.global_ba) {
    return std::nullopt;
  }
  return global_bundle_adjust(m, camera, options).transform([](const graphs::Summary& summary) {
    return std::optional<graphs::Summary>(summary);
  });
}

}  // namespace

Result<graphs::Summary> global_bundle_adjust(Map& m, const Camera& camera, const Options& options,
                                             const std::atomic<bool>* stop) {
  if (!std::ranges::contains(m.kf_id, std::int64_t{0})) {
    return fail(ErrorKind::kInvalidInput, "m: expected the map's first keyframe (id 0)");
  }
  Ids cameras;
  Ids point_ids;
  std::vector<double> right;
  std::vector<double> information;
  std::vector<std::array<double, 2>> pixels;
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    for (const std::int64_t row : keypoint_rows(m, k)) {
      const auto r = static_cast<std::size_t>(row);
      if (m.kp_point[r] < 0) {
        continue;
      }
      cameras.push_back(static_cast<std::int64_t>(k));
      point_ids.push_back(point_row(m, m.kp_point[r]));
      pixels.push_back(m.kp_xy[r]);
      right.push_back(m.kp_right[r]);
      information.push_back(1.0 / sigma2(m, m.kp_level[r]));
    }
  }
  const graphs::Pairs xy = pairs_of(pixels);
  graphs::Points points(static_cast<Eigen::Index>(point_count(m)), 3);
  for (std::size_t p = 0; p < point_count(m); ++p) {
    points.row(static_cast<Eigen::Index>(p)) = m.pt_xyz[p].transpose();
  }
  std::vector<Eigen::Matrix4d> poses;
  poses.reserve(keyframe_count(m));
  Ids fixed;
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    poses.push_back(sim3::inverse(m.kf_pose[k]));
    if (m.kf_id[k] == 0) {
      fixed.push_back(static_cast<std::int64_t>(k));
    }
  }
  const graphs::Observations observations{.cameras = cameras,
                                          .point_ids = point_ids,
                                          .pixels = xy,
                                          .right = right,
                                          .information = information,
                                          .huber = {}};
  auto adjusted = graphs::bundle_adjust(
      poses, points, observations, camera.model, camera.intrinsics, focal(camera) * camera.baseline,
      fixed, {.max_iterations = static_cast<int>(options.loops.global_iterations), .stop = stop});
  if (!adjusted) {
    return std::unexpected(std::move(adjusted.error()));
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    m.kf_pose[k] = sim3::inverse(adjusted->poses[k]);
  }
  for (std::size_t p = 0; p < point_count(m); ++p) {
    m.pt_xyz[p] = adjusted->points.row(static_cast<Eigen::Index>(p)).transpose();
  }
  Ids rows(point_count(m));
  std::ranges::iota(rows, std::int64_t{0});
  if (auto ok = refresh_points(m, rows); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return adjusted->summary;
}

Result<Propagated> propagate_global_ba(Map& now, const Map& solved) {
  std::vector<Eigen::Matrix4d> poses = now.kf_pose;
  std::map<std::int64_t, std::size_t> rows_of;
  std::map<std::int64_t, std::vector<std::size_t>> children;
  for (std::size_t k = 0; k < keyframe_count(now); ++k) {
    rows_of[now.kf_id[k]] = k;
    children[now.kf_parent[k]].push_back(k);
  }
  Propagated out;
  std::deque<std::size_t> walk;
  if (const auto roots = children.find(-1); roots != children.end()) {
    walk.assign(roots->second.begin(), roots->second.end());
  }
  while (!walk.empty()) {
    const std::size_t k = walk.front();
    walk.pop_front();
    const std::int64_t from = keyframe_row(solved, now.kf_id[k]);
    if (from >= 0) {
      poses[k] = solved.kf_pose[static_cast<std::size_t>(from)];
      ++out.adjusted;
    } else if (const auto parent = rows_of.find(now.kf_parent[k]); parent != rows_of.end()) {
      const std::size_t p = parent->second;
      poses[k] = now.kf_pose[k] * sim3::inverse(now.kf_pose[p]) * poses[p];
      ++out.propagated;
    }
    if (const auto below = children.find(now.kf_id[k]); below != children.end()) {
      walk.insert(walk.end(), below->second.begin(), below->second.end());
    }
  }
  for (std::size_t p = 0; p < point_count(now); ++p) {
    const std::int64_t from = point_row(solved, now.pt_id[p]);
    if (from >= 0) {
      now.pt_xyz[p] = solved.pt_xyz[static_cast<std::size_t>(from)];
      ++out.points;
      continue;
    }
    const auto reference = rows_of.find(now.pt_reference_kf[p]);
    if (reference == rows_of.end()) {
      continue;
    }
    const Eigen::Matrix4d& to_camera = now.kf_pose[reference->second];
    const Eigen::Vector3d local =
        to_camera.topLeftCorner<3, 3>() * now.pt_xyz[p] + to_camera.topRightCorner<3, 1>();
    const Eigen::Matrix4d to_world = sim3::inverse(poses[reference->second]);
    now.pt_xyz[p] = to_world.topLeftCorner<3, 3>() * local + to_world.topRightCorner<3, 1>();
    ++out.carried;
  }
  now.kf_pose = std::move(poses);
  Ids rows(point_count(now));
  std::ranges::iota(rows, std::int64_t{0});
  if (auto ok = refresh_points(now, rows); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return out;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity): Python's _correct, step for step
Result<CorrectionSummary> correct(Map& m, const Camera& camera, const Loop& loop,
                                  const std::string& mode, const Options& options) {
  auto current_row = check_stage(mode, camera, options).and_then([&m, &loop] {
    return checked_keyframe_row(m, loop.current);
  });
  if (!current_row) {
    return std::unexpected(std::move(current_row.error()));
  }
  auto matched_row = checked_keyframe_row(m, loop.matched);
  if (!matched_row) {
    return std::unexpected(std::move(matched_row.error()));
  }
  const std::size_t k = *current_row;
  if (std::cmp_not_equal(loop.matched_points.size(), m.kf_start[k + 1] - m.kf_start[k])) {
    return fail(ErrorKind::kInvalidInput,
                "loop.matched_points: expected one id per keypoint of the current keyframe");
  }
  const bool fixed_scale = mode != "monocular";
  auto linked = links(m, static_cast<std::int64_t>(k), options);
  if (!linked) {
    return std::unexpected(std::move(linked.error()));
  }
  Ids moved_rows{static_cast<std::int64_t>(k)};
  for (const Link& link : *linked) {
    moved_rows.push_back(link.row);
  }
  const std::vector<Eigen::Matrix4d> before = m.kf_pose;
  std::map<std::size_t, Eigen::Matrix4d> corrected{{k, loop.similarity}};
  const Eigen::Matrix4d to_world = sim3::inverse(m.kf_pose[k]);
  for (const Link& link : *linked) {
    const auto r = static_cast<std::size_t>(link.row);
    corrected[r] = m.kf_pose[r] * to_world * loop.similarity;
  }
  std::map<std::size_t, std::set<std::int64_t>> previous;
  for (const std::int64_t r : moved_rows) {
    auto theirs = linked_rows(m, static_cast<std::size_t>(r), options);
    if (!theirs) {
      return std::unexpected(std::move(theirs.error()));
    }
    previous[static_cast<std::size_t>(r)] = std::move(*theirs);
  }
  auto moved_by = move_keyframes(m, moved_rows, corrected);
  if (!moved_by) {
    return std::unexpected(std::move(moved_by.error()));
  }
  auto replaced = replace_matched(m, loop.current, loop.matched_points);
  if (!replaced) {
    return std::unexpected(std::move(replaced.error()));
  }
  // Moving, replacing and fusing change points and the moved keyframes' poses, but never add,
  // remove or reorder a keyframe: every keyframe keeps its row from here to the graph, so the
  // rows found above still name them.
  std::int64_t fused = 0;
  for (const std::int64_t r : moved_rows) {
    auto count = fuse_by_similarity(m, camera, r, corrected.at(static_cast<std::size_t>(r)),
                                    loop.points, options);
    if (!count) {
      return std::unexpected(std::move(count.error()));
    }
    fused += *count;
  }
  const std::set<std::int64_t> moving(moved_rows.begin(), moved_rows.end());
  std::map<std::size_t, std::set<std::int64_t>> loop_links;
  std::int64_t new_links = 0;
  for (const std::int64_t row : moved_rows) {
    const auto r = static_cast<std::size_t>(row);
    auto now = linked_rows(m, r, options);
    if (!now) {
      return std::unexpected(std::move(now.error()));
    }
    std::set<std::int64_t> fresh;
    for (const std::int64_t n : *now) {
      if (!previous[r].contains(n) && !moving.contains(n)) {
        fresh.insert(n);
      }
    }
    new_links += static_cast<std::int64_t>(fresh.size());
    loop_links[r] = std::move(fresh);
  }
  std::vector<Eigen::Matrix4d> start = m.kf_pose;
  for (const auto& [r, similarity] : corrected) {
    start[r] = similarity;
  }
  auto essential = essential_graph(
      m, start, before, loop_links, {k, *matched_row},
      std::vector<std::int64_t>{static_cast<std::int64_t>(*matched_row)}, fixed_scale, options);
  if (!essential) {
    return std::unexpected(std::move(essential.error()));
  }
  auto ba = finish_correction(
      m, start, *essential, *moved_by,
      {.add = &add_loop_edge, .first = loop.matched, .second = loop.current}, camera, options);
  if (!ba) {
    return std::unexpected(std::move(ba.error()));
  }
  return CorrectionSummary{.current = loop.current,
                           .matched = loop.matched,
                           .similarity = loop.similarity,
                           .corrected = static_cast<std::int64_t>(moved_rows.size()),
                           .moved = static_cast<std::int64_t>(moved_by->size()),
                           .replaced = *replaced,
                           .fused = fused,
                           .links = new_links,
                           .edges = essential->edges,
                           .essential = essential->summary,
                           .global_ba = *ba};
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity): Python's _detect_merge, step for step
Result<MergeDetection> detect_merge(const Map& m, const Database& db, const Camera& camera,
                                    std::int64_t keyframe, std::span<const MapRef> others,
                                    const std::optional<Merging>& pending, const std::string& mode,
                                    const Options& options) {
  const auto place = [&others](std::int64_t number) {
    return static_cast<std::size_t>(
        std::ranges::find_if(others, [number](const MapRef& o) { return o.number == number; }) -
        others.begin());
  };
  // The mode, camera, settings and keyframe first, then the other maps, as Python checks them.
  auto stage = stage_of(m, keyframe, mode, camera, options).and_then([&](Stage start) {
    return check_numbers(others, "others").and_then([&] -> Result<Stage> {
      if (pending && place(pending->map) == others.size()) {
        return fail(ErrorKind::kInvalidInput, "pending.map: map " + std::to_string(pending->map) +
                                                  " is not among the others");
      }
      return start;
    });
  });
  if (!stage) {
    return std::unexpected(std::move(stage.error()));
  }
  if (stage->small) {
    return MergeDetection{.pending = pending,
                          .merge = std::nullopt,
                          .summary = skipped_summary(pending ? pending->pending.confirmations : 0),
                          .maps = {}};
  }
  Looking looking{.pending = std::nullopt,
                  .holding = {.map = &m, .database = &db, .own = false},
                  .holding_at = 0,
                  .maps = {}};
  for (const MapRef& other : others) {
    looking.maps.push_back(other.map);
  }
  if (pending) {
    const std::size_t at = place(pending->map);
    looking.pending = pending->pending;
    looking.holding = {.map = others[at].map, .database = others[at].database, .own = false};
    looking.holding_at = at;
  }
  auto found = detect_region(
      m, camera, stage->k, looking,
      [&] { return search_merge(m, db, camera, stage->k, others, stage->fixed_scale, options); },
      stage->fixed_scale, options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  Detection& detected = found->detection;
  const std::int64_t number = others.empty() ? 0 : others[found->at].number;
  MergeDetection out{.pending = std::nullopt,
                     .merge = std::nullopt,
                     .summary = std::move(detected.summary),
                     .maps = std::move(found->maps)};
  if (detected.pending) {
    out.pending = Merging{.map = number, .pending = std::move(*detected.pending)};
  }
  if (detected.loop) {
    out.merge = Merge{.map = number, .loop = std::move(*detected.loop)};
  }
  return out;
}

Detection merge_first(Detection loop, const MergeDetection& merging) {
  if (!merging.merge || !loop.loop) {
    return loop;
  }
  loop.summary.outcome = Outcome::kMerged;
  return Detection{
      .pending = std::nullopt, .loop = std::nullopt, .summary = std::move(loop.summary)};
}

Result<Eigen::Matrix4d> merge_transform(const Map& active, const Merge& found) {
  return checked_keyframe_row(active, found.loop.current).transform([&](std::size_t row) {
    return Eigen::Matrix4d(sim3::inverse(found.loop.similarity) * active.kf_pose[row]);
  });
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity): Python's _merge, step for step
Result<Merged> merge(const Map& active, const Map& matched, const Camera& camera,
                     const Merge& found, const std::string& mode, const Options& options) {
  const Loop& region = found.loop;
  auto placed_row = check_stage(mode, camera, options).and_then([&active, &region] {
    return checked_keyframe_row(active, region.current);
  });
  if (!placed_row) {
    return std::unexpected(std::move(placed_row.error()));
  }
  if (auto row = checked_keyframe_row(matched, region.matched); !row) {
    return std::unexpected(std::move(row.error()));
  }
  if (active.levels != matched.levels || active.scale_factor != matched.scale_factor) {
    return fail(ErrorKind::kInvalidInput, "merge: the two maps' pyramids differ");
  }
  const bool fixed_scale = mode != "monocular";
  const double scale = fixed_scale ? 1.0 : scale_of(region.similarity);
  const Eigen::Matrix4d to_matched =
      sim3::inverse(region.similarity) * active.kf_pose[*placed_row];  // merge_transform's G
  const std::int64_t keyframe_offset = matched.next_kf_id;
  const std::int64_t point_offset = matched.next_pt_id;
  Map moved = active;
  if (auto ok = moved_into(moved, to_matched); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  Merged out{.map = matched,
             .keyframe_offset = keyframe_offset,
             .point_offset = point_offset,
             .summary = {}};
  Map& m = out.map;
  absorb(m, moved, keyframe_offset, point_offset);
  const std::int64_t current = region.current + keyframe_offset;
  rehang(m, current, region.matched);
  const auto k = static_cast<std::size_t>(keyframe_row(m, current));
  const auto km = static_cast<std::size_t>(keyframe_row(m, region.matched));
  auto window = weld_window(m, k, options);
  if (!window) {
    return std::unexpected(std::move(window.error()));
  }
  auto matched_window = weld_window(m, km, options);
  if (!matched_window) {
    return std::unexpected(std::move(matched_window.error()));
  }
  Ids points;
  for (const std::int64_t row : keyframe_points(m, *matched_window)) {
    points.push_back(m.pt_id[static_cast<std::size_t>(row)]);
  }
  std::int64_t fused = 0;
  for (const std::int64_t r : *window) {  // fusion erases points, never keyframes: the rows hold
    auto count =
        fuse_by_similarity(m, camera, r, m.kf_pose[static_cast<std::size_t>(r)], points, options);
    if (!count) {
      return std::unexpected(std::move(count.error()));
    }
    fused += *count;
  }
  const std::vector<Eigen::Matrix4d> before = m.kf_pose;
  auto welded = weld(m, camera, *window, *matched_window, keyframe_offset, options);
  if (!welded) {
    return std::unexpected(std::move(welded.error()));
  }
  // The active side's nodes carry its own units, as its edges measured before were.
  std::vector<Eigen::Matrix4d> start;
  std::vector<Eigen::Matrix4d> prior;
  for (std::size_t r = 0; r < keyframe_count(m); ++r) {
    const bool absorbed = m.kf_id[r] >= keyframe_offset;
    start.push_back(absorbed ? scaled(m.kf_pose[r], scale) : m.kf_pose[r]);
    prior.push_back(absorbed ? scaled(before[r], scale) : before[r]);
  }
  std::set<std::int64_t> held(window->begin(), window->end());
  held.insert(matched_window->begin(), matched_window->end());
  const Ids fixed(held.begin(), held.end());
  auto essential = essential_graph(m, start, prior, {}, {k, km}, fixed, fixed_scale, options);
  if (!essential) {
    return std::unexpected(std::move(essential.error()));
  }
  auto ba = finish_correction(m, start, *essential, {},
                              {.add = &add_merge_edge, .first = region.matched, .second = current},
                              camera, options);
  if (!ba) {
    return std::unexpected(std::move(ba.error()));
  }
  out.summary = MergeSummary{.map = found.map,
                             .current = current,
                             .matched = region.matched,
                             .similarity = region.similarity,
                             .window = static_cast<std::int64_t>(window->size()),
                             .matched_window = static_cast<std::int64_t>(matched_window->size()),
                             .fused = fused,
                             .weld = *welded,
                             .edges = essential->edges,
                             .essential = essential->summary,
                             .global_ba = *ba};
  return out;
}

}  // namespace slam_it::edge::loops
