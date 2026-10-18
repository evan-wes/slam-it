/// @file
/// @brief Implementation of the loop closing declared in slam_it/edge/loops.hpp: each function
///        mirrors its namesake in `slam_it.edge.loops`, step for step.
#include "slam_it/edge/loops.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

/// @brief Check a stage's mode, camera and settings, as detection and correction do first.
/// @param mode The mode's name.
/// @param camera The camera.
/// @param options The settings.
/// @return Nothing, or `InvalidInput` for an unknown mode, a bad camera or bad settings.
Result<void> check_stage(const std::string& mode, const Camera& camera, const Options& options);

/// @brief How many keypoints a search matched.
/// @param found Each keypoint's point id, or -1.
/// @return The matched ones.
std::int64_t count_matched(std::span<const std::int64_t> found) {
  return std::ranges::count_if(found, [](std::int64_t id) { return id >= 0; });
}

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
///        its arguments checked.
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param similarity Its `S_cw`.
/// @param points The points' rows.
/// @param window The window and the distance.
/// @param options The settings.
/// @return Each keypoint's match.
Result<Ids> search(const Map& m, const Camera& camera, std::size_t k,
                   const Eigen::Matrix4d& similarity, std::span<const std::int64_t> points,
                   Window window, const Options& options) {
  const std::int64_t start = m.kf_start[k];
  Ids out(static_cast<std::size_t>(m.kf_start[k + 1] - start), -1);
  auto seen = in_view(m, camera, similarity, points, options);
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
          m.pt_descriptor[p], m.kp_descriptor[static_cast<std::size_t>(start + j)]);
      if (d < best_distance) {
        best_distance = d;
        best = j;
      }
    }
    if (best >= 0 && best_distance <= window.distance) {
      out[static_cast<std::size_t>(best)] = m.pt_id[p];
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
/// @param m The map.
/// @param camera The camera.
/// @param k1 The first keyframe's row.
/// @param k2 The second's.
/// @param similarity The starting `S_12`.
/// @param matched For each keypoint of `k1`, its match.
/// @param fixed_scale Refine a rigid motion.
/// @param options The settings.
/// @return The refined similarity and its inliers, or none.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): Python's _refine, step for step
Result<std::optional<Refined>> refined(const Map& m, const Camera& camera, std::size_t k1,
                                       std::size_t k2, const Eigen::Matrix4d& similarity,
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
    const std::int64_t p_partner = point_row(m, matched[i]);
    if (p_own < 0 || p_partner < 0) {
      continue;
    }
    const Eigen::Vector3d one = to_camera(m.kf_pose[k1], m.pt_xyz[static_cast<std::size_t>(p_own)]);
    const Eigen::Vector3d two =
        to_camera(m.kf_pose[k2], m.pt_xyz[static_cast<std::size_t>(p_partner)]);
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
  const Ids rows2 = keypoint_rows(m, k2);
  for (std::size_t i = 0; i < n; ++i) {
    const auto e = static_cast<Eigen::Index>(i);
    all1.row(e) = points1[i].transpose();
    all2.row(e) = points2[i].transpose();
    const auto r = static_cast<std::size_t>(rows[i]);
    pixels1.row(e) << m.kp_xy[r][0], m.kp_xy[r][1];
    pixels2.row(e) << kNan, kNan;
    std::int64_t level2 = 0;
    const std::int64_t id = m.pt_id[partners[i]];
    for (const std::int64_t at : rows2) {
      if (m.kp_point[static_cast<std::size_t>(at)] == id) {
        const auto a = static_cast<std::size_t>(at);
        pixels2.row(e) << m.kp_xy[a][0], m.kp_xy[a][1];
        level2 = m.kp_level[a];
        break;
      }
    }
    information.row(e) << 1.0 / sigma2(m, m.kp_level[r]), 1.0 / sigma2(m, level2);
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
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param matched The matched keyframe's row.
/// @param similarity The keyframe's `S_cw`.
/// @param options The settings.
/// @return The group's points' ids, and each of the keyframe's keypoints' match.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k, matched), as Python's
Result<std::pair<Ids, Ids>> confirm_by_projection(const Map& m, const Camera& camera, std::size_t k,
                                                  std::size_t matched,
                                                  const Eigen::Matrix4d& similarity,
                                                  const Options& options) {
  auto linked = linked_rows(m, k, options);
  if (!linked) {
    return std::unexpected(std::move(linked.error()));
  }
  auto points = group_of(m, matched, *linked, options);
  if (!points) {
    return std::unexpected(std::move(points.error()));
  }
  auto found =
      search(m, camera, k, similarity, *points,
             {.radius = options.loops.confirm_window, .distance = options.loops.confirm_distance},
             options);
  if (!found) {
    return std::unexpected(std::move(found.error()));
  }
  Ids ids;
  ids.reserve(points->size());
  for (const std::int64_t row : *points) {
    ids.push_back(m.pt_id[static_cast<std::size_t>(row)]);
  }
  return std::pair{std::move(ids), std::move(*found)};
}

/// @brief A loop being confirmed after a keyframe, or none once dropped, and how it went.
using Carried = std::pair<std::optional<Pending>, ConfirmationSummary>;

/// @brief Try to confirm the loop being confirmed on a later keyframe
///        (`slam_it.edge.loops._carry`).
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param pending The loop.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The loop as it stands after the keyframe, and how it went.
Result<Carried> carry(const Map& m, const Camera& camera, std::size_t k, const Pending& pending,
                      bool fixed_scale, const Options& options) {
  const LoopOptions& settings = options.loops;
  auto matched = checked_keyframe_row(m, pending.matched);
  if (!matched) {
    return std::unexpected(std::move(matched.error()));
  }
  auto last = checked_keyframe_row(m, pending.last);
  if (!last) {
    return std::unexpected(std::move(last.error()));
  }
  const Eigen::Matrix4d carried = relative(m.kf_pose[k], m.kf_pose[*last]) * pending.similarity;
  auto first = confirm_by_projection(m, camera, k, *matched, carried, options);
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
    const Eigen::Matrix4d start = carried * sim3::inverse(m.kf_pose[*matched]);
    auto found = refined(m, camera, k, *matched, start, first->second, fixed_scale, options);
    if (!found) {
      return std::unexpected(std::move(found.error()));
    }
    if (found->has_value()) {
      summary.refined = (*found)->inliers;
    }
    if (found->has_value() && (*found)->inliers > settings.temporal_refined) {
      const Eigen::Matrix4d placed = (*found)->similarity * m.kf_pose[*matched];
      auto again = confirm_by_projection(m, camera, k, *matched, placed, options);
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

/// @brief A keyframe matched by word to a group: each keypoint's point and the row it was
///        matched in, and the row of the group's keyframe that matched most.
struct Merged {
  /// @brief Each keypoint's point id, or -1.
  Ids points;
  /// @brief The row of the keyframe it was matched in, or -1.
  Ids source;
  /// @brief The group's keyframe that matched most (ties: the first).
  std::size_t best{0};
};

/// @brief Match a keyframe by word to each of a group's keyframes, and merge the matches
///        (`slam_it.edge.loops._merged_by_word`).
/// @param m The map.
/// @param db Its database.
/// @param k The keyframe's row.
/// @param group The group's rows.
/// @param options The settings.
/// @return The merged matches.
Result<Merged> merged_by_word(const Map& m, const Database& db, std::size_t k,
                              std::span<const std::int64_t> group, const Options& options) {
  const auto size = static_cast<std::size_t>(m.kf_start[k + 1] - m.kf_start[k]);
  Merged out{.points = Ids(size, -1), .source = Ids(size, -1), .best = 0};
  std::set<std::int64_t> used;
  std::int64_t most = -1;
  out.best = static_cast<std::size_t>(group.front());
  for (const std::int64_t g : group) {
    auto found = match_by_word(m, db, static_cast<std::int64_t>(k), g, options);
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
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param merged Its matches by word.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The similarity and its inliers.
Result<Ransacked> similarity_by_ransac(const Map& m, const Camera& camera, std::size_t k,
                                       const Merged& merged, bool fixed_scale,
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
    const auto partner = static_cast<std::size_t>(point_row(m, merged.points[i]));
    const auto g = static_cast<std::size_t>(merged.source[i]);
    std::int64_t level2 = 0;
    for (const std::int64_t at : keypoint_rows(m, g)) {
      if (m.kp_point[static_cast<std::size_t>(at)] == merged.points[i]) {
        level2 = m.kp_level[static_cast<std::size_t>(at)];
        break;
      }
    }
    const Eigen::Vector3d one = to_camera(m.kf_pose[k], m.pt_xyz[own]);
    const Eigen::Vector3d two = to_camera(m.kf_pose[merged.best], m.pt_xyz[partner]);
    if (!(one.z() > foundations::camera::kMinDepth) ||
        !(two.z() > foundations::camera::kMinDepth)) {
      continue;
    }
    points1.push_back(one);
    points2.push_back(two);
    sigma1.push_back(sigma2(m, m.kp_level[row]));
    sigma2s.push_back(sigma2(m, level2));
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
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param best The matched keyframe's row.
/// @param similarity The keyframe's `S_cw`.
/// @param options The settings.
/// @return The confirmations.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (k, best), as Python's
Result<std::int64_t> confirmations(const Map& m, const Camera& camera, std::size_t k,
                                   std::size_t best, const Eigen::Matrix4d& similarity,
                                   const Options& options) {
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
    auto found = confirm_by_projection(m, camera, r, best, carried, options);
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
/// @param m The map.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param best The best keyframe's row.
/// @param model `S_kb`.
/// @param summary The candidate's summary, its counts filled in as reached.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The placement, or none where a test failed.
Result<std::optional<Placed>> place(const Map& m, const Camera& camera, std::size_t k,
                                    std::size_t best, const Eigen::Matrix4d& model,
                                    CandidateSummary& summary, bool fixed_scale,
                                    const Options& options) {
  const LoopOptions& settings = options.loops;
  auto group = best_links(m, best, options);
  if (!group) {
    return std::unexpected(std::move(group.error()));
  }
  group->push_back(static_cast<std::int64_t>(best));
  const Ids points = keyframe_points(m, *group);
  auto coarse =
      search(m, camera, k, model * m.kf_pose[best], points,
             {.radius = settings.coarse_window, .distance = settings.coarse_distance}, options);
  if (!coarse) {
    return std::unexpected(std::move(coarse.error()));
  }
  summary.projected = count_matched(*coarse);
  if (*summary.projected < settings.coarse_matches) {
    return std::nullopt;
  }
  auto found = refined(m, camera, k, best, model, *coarse, fixed_scale, options);
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
  const Eigen::Matrix4d placed = (*found)->similarity * m.kf_pose[best];
  auto fine = search(m, camera, k, placed, points,
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
/// @param m The map.
/// @param db Its database.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param candidate The candidate's id and score.
/// @param linked The keyframe's links' rows.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return How far it got, and what it found.
Result<Tried> try_candidate(const Map& m, const Database& db, const Camera& camera, std::size_t k,
                            std::pair<std::int64_t, double> candidate,
                            const std::set<std::int64_t>& linked, bool fixed_scale,
                            const Options& options) {
  const LoopOptions& settings = options.loops;
  auto c = checked_keyframe_row(m, candidate.first);
  if (!c) {
    return std::unexpected(std::move(c.error()));
  }
  auto neighbours = best_links(m, *c, options);
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
  auto merged = merged_by_word(m, db, k, group, options);
  if (!merged) {
    return std::unexpected(std::move(merged.error()));
  }
  summary.matches = count_matched(merged->points);
  summary.best = m.kf_id[merged->best];
  if (*summary.matches < settings.min_matches) {
    return Tried{summary, std::nullopt};
  }
  auto model = similarity_by_ransac(m, camera, k, *merged, fixed_scale, options);
  if (!model) {
    return std::unexpected(std::move(model.error()));
  }
  summary.inliers = model->inliers;
  if (!model->similarity) {
    return Tried{summary, std::nullopt};
  }
  auto placed =
      place(m, camera, k, merged->best, *model->similarity, summary, fixed_scale, options);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  if (!placed->has_value()) {
    return Tried{summary, std::nullopt};
  }
  auto confirmed = confirmations(m, camera, k, merged->best, (*placed)->similarity, options);
  if (!confirmed) {
    return std::unexpected(std::move(confirmed.error()));
  }
  summary.confirmations = *confirmed;
  Ids ids;
  ids.reserve((*placed)->points.size());
  for (const std::int64_t row : (*placed)->points) {
    ids.push_back(m.pt_id[static_cast<std::size_t>(row)]);
  }
  const std::int64_t reprojected = count_matched((*placed)->fine);
  return Tried{summary, Found{.best = merged->best,
                              .similarity = (*placed)->similarity,
                              .points = std::move(ids),
                              .matched_points = std::move((*placed)->fine),
                              .reprojected = reprojected,
                              .confirmations = *confirmed}};
}

/// @brief The best candidate that passed, if any, and how the search went.
using Searched = std::pair<std::optional<Found>, DetectionSummary>;

/// @brief Try the database's candidates (`slam_it.edge.loops._search`).
/// @param m The map.
/// @param db Its database.
/// @param camera The camera.
/// @param k The keyframe's row.
/// @param fixed_scale Rigid similarities.
/// @param options The settings.
/// @return The best candidate that passed, and how the search went.
Result<Searched> search_candidates(const Map& m, const Database& db, const Camera& camera,
                                   std::size_t k, bool fixed_scale, const Options& options) {
  auto candidates = loop_candidates(db, m, m.kf_id[k], options);
  if (!candidates) {
    return std::unexpected(std::move(candidates.error()));
  }
  auto linked = linked_rows(m, k, options);
  if (!linked) {
    return std::unexpected(std::move(linked.error()));
  }
  DetectionSummary summary{.outcome = Outcome::kNone,
                           .confirmation = std::nullopt,
                           .sharing = candidates->sharing,
                           .scored = candidates->scored,
                           .candidates = {},
                           .confirmations = 0};
  std::optional<Found> best;
  for (std::size_t i = 0; i < candidates->keyframes.size(); ++i) {
    auto tried = try_candidate(m, db, camera, k, {candidates->keyframes[i], candidates->scores[i]},
                               *linked, fixed_scale, options);
    if (!tried) {
      return std::unexpected(std::move(tried.error()));
    }
    summary.candidates.push_back(tried->first);
    if (tried->second && (!best || tried->second->reprojected > best->reprojected)) {
      best = std::move(tried->second);
    }
  }
  return Searched{std::move(best), std::move(summary)};
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
///        .essential_graph`, `OptimizeEssentialGraph`), the matched keyframe held fixed.
/// @param m The map.
/// @param start Each keyframe's `S_iw` the graph starts from.
/// @param before Each one's pose before the correction.
/// @param loop_links Each moved keyframe's links new across the loop.
/// @param ends The current and matched keyframes' rows.
/// @param fixed_scale Optimise SE(3), not Sim(3).
/// @param options The settings.
/// @return The solution.
Result<Essential> essential_graph(const Map& m, std::span<const Eigen::Matrix4d> start,
                                  std::span<const Eigen::Matrix4d> before,
                                  const std::map<std::size_t, std::set<std::int64_t>>& loop_links,
                                  std::pair<std::size_t, std::size_t> ends, bool fixed_scale,
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
  const std::vector<std::int64_t> fixed{static_cast<std::int64_t>(ends.second)};
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
  return search(m, camera, *row, similarity, points, window, options);
}

Result<std::vector<std::int64_t>> match_by_word(const Map& m, const Database& db, std::int64_t k1,
                                                std::int64_t k2, const Options& options) {
  auto rows = keyframe_rows_in(m, k1, k2);
  if (!rows) {
    return std::unexpected(std::move(rows.error()));
  }
  const auto [first, second] = *rows;
  std::array<std::vector<foundations::orb::Descriptor>, 2> descriptors;
  std::array<Ids, 2> nodes;
  std::array<Ids, 2> angles;
  std::array<Ids, 2> own;
  const std::array<std::size_t, 2> keyframes{first, second};
  for (std::size_t side = 0; side < 2; ++side) {
    const std::size_t k = keyframes[side];
    const auto words = db.keyframes.find(m.kf_id[k]);
    if (words == db.keyframes.end()) {
      return fail(ErrorKind::kInvalidInput,
                  "keyframe " + std::to_string(m.kf_id[k]) + " is not in the database");
    }
    for (const std::int64_t row : keypoint_rows(m, k)) {
      const auto r = static_cast<std::size_t>(row);
      if (m.kp_point[r] < 0) {
        continue;
      }
      own[side].push_back(row);
      descriptors[side].push_back(m.kp_descriptor[r]);
      nodes[side].push_back(words->second.nodes[static_cast<std::size_t>(row - m.kf_start[k])]);
      angles[side].push_back(m.kp_angle[r]);
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
  Ids out(static_cast<std::size_t>(m.kf_start[first + 1] - m.kf_start[first]), -1);
  for (const matching::Match& match : *found) {
    const std::int64_t row1 = own[0][static_cast<std::size_t>(match.first)];
    const std::int64_t row2 = own[1][static_cast<std::size_t>(match.second)];
    out[static_cast<std::size_t>(row1 - m.kf_start[first])] =
        m.kp_point[static_cast<std::size_t>(row2)];
  }
  return out;
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
        return refined(m, camera, rows.first, rows.second, similarity, matched, fixed_scale,
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
  auto row = check_stage(mode, camera, options).and_then([&m, keyframe] {
    return checked_keyframe_row(m, keyframe);
  });
  if (!row) {
    return std::unexpected(std::move(row.error()));
  }
  const std::size_t k = *row;
  const bool fixed_scale = mode != "monocular";
  const LoopOptions& settings = options.loops;
  if (std::cmp_less(keyframe_count(m), settings.min_keyframes)) {
    DetectionSummary skipped{.outcome = Outcome::kSkipped,
                             .confirmation = std::nullopt,
                             .sharing = 0,
                             .scored = 0,
                             .candidates = {},
                             .confirmations = pending ? pending->confirmations : 0};
    return Detection{.pending = pending, .loop = std::nullopt, .summary = std::move(skipped)};
  }
  std::optional<Pending> state = pending;
  std::optional<ConfirmationSummary> confirmation;
  bool dropped = false;
  if (pending) {
    auto carried = carry(m, camera, k, *pending, fixed_scale, options);
    if (!carried) {
      return std::unexpected(std::move(carried.error()));
    }
    confirmation = carried->second;
    if (confirmation->confirmed && carried->first) {
      DetectionSummary confirmed{.outcome = Outcome::kConfirmed,
                                 .confirmation = confirmation,
                                 .sharing = 0,
                                 .scored = 0,
                                 .candidates = {},
                                 .confirmations = 0};
      return settle(std::move(*carried->first), std::move(confirmed), options);
    }
    dropped = !carried->first;
    state = std::move(carried->first);
  }
  auto searched = search_candidates(m, db, camera, k, fixed_scale, options);
  if (!searched) {
    return std::unexpected(std::move(searched.error()));
  }
  DetectionSummary summary = std::move(searched->second);
  summary.confirmation = confirmation;
  if (!searched->first) {
    summary.outcome = dropped ? Outcome::kDropped : Outcome::kNone;
    summary.confirmations = state ? state->confirmations : 0;
    return Detection{
        .pending = std::move(state), .loop = std::nullopt, .summary = std::move(summary)};
  }
  Found& found = *searched->first;
  summary.outcome = Outcome::kStarted;
  Pending started{.matched = m.kf_id[found.best],
                  .last = keyframe,
                  .similarity = found.similarity,
                  .confirmations = found.confirmations,
                  .misses = 0,
                  .points = std::move(found.points),
                  .matched_points = std::move(found.matched_points)};
  return settle(std::move(started), std::move(summary), options);
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

Result<graphs::Summary> global_bundle_adjust(Map& m, const Camera& camera, const Options& options) {
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
  graphs::Pairs xy(static_cast<Eigen::Index>(pixels.size()), 2);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    xy.row(static_cast<Eigen::Index>(i)) << pixels[i][0], pixels[i][1];
  }
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
                                          .information = information};
  auto adjusted = graphs::bundle_adjust(
      poses, points, observations, camera.model, camera.intrinsics, focal(camera) * camera.baseline,
      fixed, {.max_iterations = static_cast<int>(options.loops.global_iterations)});
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
  auto essential =
      essential_graph(m, start, before, loop_links, {k, *matched_row}, fixed_scale, options);
  if (!essential) {
    return std::unexpected(std::move(essential.error()));
  }
  if (auto ok = move_by_graph(m, start, essential->poses, *moved_by); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = add_loop_edge(m, loop.matched, loop.current); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  std::optional<graphs::Summary> ba;
  if (options.loops.global_ba) {
    auto adjusted = global_bundle_adjust(m, camera, options);
    if (!adjusted) {
      return std::unexpected(std::move(adjusted.error()));
    }
    ba = *adjusted;
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
                           .global_ba = ba};
}

}  // namespace slam_it::edge::loops
