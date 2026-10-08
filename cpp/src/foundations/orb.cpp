/// @file
/// @brief Implementation of the ORB detector declared in slamlab/foundations/orb.hpp.
#include "slamlab/foundations/orb.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace slamlab::foundations::orb {

namespace {

/// @brief A vector's index from a signed count.
/// @param index The index, non-negative.
/// @return It as a `std::size_t`.
std::size_t at(std::int64_t index) { return static_cast<std::size_t>(index); }

/// @brief The circle's pixels as offsets into a level's row-major pixels.
using CircleOffsets = std::array<std::int64_t, kFastCircle.size()>;

/// @brief The circle's offsets on a level.
/// @param cols The level's columns.
/// @return Each circle pixel's offset from the centre's index.
CircleOffsets circle_offsets(std::int64_t cols) {
  CircleOffsets out{};
  for (std::size_t k = 0; k < out.size(); ++k) {
    out[k] = (kFastCircle[k][1] * cols) + kFastCircle[k][0];
  }
  return out;
}

/// @brief A pixel's FAST-9 score: the largest threshold at which some arc of nine is all
///        brighter, or all darker, than the centre by more.
///
/// Every arc's smallest and largest difference, all sixteen arcs at once, in branch-free
/// 16-bit loops a compiler vectorizes (`pminsw`, `pmaxsw`); the arcs' starts are the lanes.
/// @param pixels The level's pixels, row-major.
/// @param index The pixel's index, at least 3 rows and columns from the border.
/// @param circle The circle's offsets on the level.
/// @return The score: one below the best arc's smallest margin (-1 when no arc has one).
std::int64_t arc_score(std::span<const std::uint8_t> pixels, std::int64_t index,
                       const CircleOffsets& circle) {
  constexpr std::size_t kCircle = kFastCircle.size();
  const std::int16_t centre = pixels[at(index)];
  // Each circle pixel minus the centre, twice round, so every arc is contiguous.
  std::array<std::int16_t, 2 * kCircle> d{};
  for (std::size_t k = 0; k < kCircle; ++k) {
    d[k] = static_cast<std::int16_t>(pixels[at(index + circle[k])] - centre);
    d[k + kCircle] = d[k];
  }
  std::array<std::int16_t, kCircle> low{};
  std::array<std::int16_t, kCircle> high{};
  for (std::size_t start = 0; start < kCircle; ++start) {
    low[start] = d[start];
    high[start] = d[start];
  }
  for (std::size_t j = 1; j < kFastArc; ++j) {
    for (std::size_t start = 0; start < kCircle; ++start) {
      low[start] = std::min(low[start], d[start + j]);
      high[start] = std::max(high[start], d[start + j]);
    }
  }
  std::int16_t best = std::numeric_limits<std::int16_t>::min();
  for (std::size_t start = 0; start < kCircle; ++start) {
    best = std::max({best, low[start], static_cast<std::int16_t>(-high[start])});
  }
  // A corner needs every margin strictly above the threshold: the largest it passes is one
  // below the best arc's smallest margin.
  return std::int64_t{best} - 1;
}

/// @brief Per-pixel runs along a row of pixels, reused across rows.
struct RowRuns {
  /// @brief The current run of circle pixels brighter than the centre by more than the floor.
  std::vector<std::uint8_t> brighter;
  /// @brief The current run of circle pixels darker than the centre by more than the floor.
  std::vector<std::uint8_t> darker;
  /// @brief The longest run of either.
  std::vector<std::uint8_t> longest;
};

/// @brief FAST-9 scores of a run of pixels along a row, where they reach `floor`.
///
/// FAST's segment test first, for the whole run at once, as OpenCV's SSE2 FAST makes it: the
/// circle walked once round plus an arc (so every run that wraps past its start is counted
/// whole), a pixel at a time for every centre, counting each centre's
/// current run of brighter and of darker circle pixels and keeping its longest, in byte-wide
/// branch-free loops a compiler vectorizes. A pixel scores at least `floor` exactly when its
/// longest run reaches `kFastArc`; only those are scored (`arc_score`).
/// @param pixels The level's pixels, row-major.
/// @param index The run's first pixel's index, at least 3 rows and columns from the border.
/// @param circle The circle's offsets on the level.
/// @param floor The smallest score of interest, from 0 to `kMaxFastThreshold`.
/// @param scores Each pixel's score if it reaches `floor`, else -1 (the run's length).
/// @param runs Scratch space.
void score_run(std::span<const std::uint8_t> pixels, std::int64_t index,
               const CircleOffsets& circle, std::int64_t floor, std::span<std::int16_t> scores,
               RowRuns& runs) {
  const std::size_t count = scores.size();
  runs.brighter.assign(count, 0);
  runs.darker.assign(count, 0);
  runs.longest.assign(count, 0);
  const std::span<const std::uint8_t> centres = pixels.subspan(at(index), count);
  // Local views: a byte store may alias anything, a vector's own pointers included, so the
  // loop reads them through these, which no store can change.
  const std::span<std::uint8_t> brighter_run(runs.brighter);
  const std::span<std::uint8_t> darker_run(runs.darker);
  const std::span<std::uint8_t> longest(runs.longest);
  const auto margin = static_cast<std::uint8_t>(floor);
  for (std::size_t k = 0; k < circle.size() + kFastArc; ++k) {
    const std::span<const std::uint8_t> ring =
        pixels.subspan(at(index + circle[k % circle.size()]), count);
    for (std::size_t c = 0; c < count; ++c) {
      // Saturating differences and all-ones masks: no branch, so the loop vectorizes.
      const std::uint8_t low = std::min(ring[c], centres[c]);
      const auto up = static_cast<std::uint8_t>(ring[c] - low);
      const auto down = static_cast<std::uint8_t>(centres[c] - low);
      const auto keep_up = static_cast<std::uint8_t>(0U - static_cast<unsigned>(up > margin));
      const auto keep_down = static_cast<std::uint8_t>(0U - static_cast<unsigned>(down > margin));
      const auto brighter = static_cast<std::uint8_t>((brighter_run[c] + 1U) & keep_up);
      const auto darker = static_cast<std::uint8_t>((darker_run[c] + 1U) & keep_down);
      brighter_run[c] = brighter;
      darker_run[c] = darker;
      longest[c] = std::max({longest[c], brighter, darker});
    }
  }
  for (std::size_t c = 0; c < count; ++c) {
    scores[c] = longest[c] >= kFastArc ? static_cast<std::int16_t>(arc_score(
                                             pixels, index + static_cast<std::int64_t>(c), circle))
                                       : std::int16_t{-1};
  }
}

/// @brief A quadtree node: a box `[x0, x1) x [y0, y1)` and the corners inside it.
struct Node {
  /// @brief The box's first column.
  std::int64_t x0;
  /// @brief One past the box's last column.
  std::int64_t x1;
  /// @brief The box's first row.
  std::int64_t y0;
  /// @brief One past the box's last row.
  std::int64_t y1;
  /// @brief The corners inside, ascending.
  std::vector<std::size_t> members;
};

/// @brief Quarter a node's box at its rounded-up midpoint.
/// @param node The node.
/// @param xy The corners' pixels.
/// @return The non-empty quarters (at least one): upper left, upper right, lower left, lower
///         right.
std::vector<Node> split(const Node& node, std::span<const std::array<std::int64_t, 2>> xy) {
  const std::int64_t mx = node.x0 + ((node.x1 - node.x0 + 1) / 2);
  const std::int64_t my = node.y0 + ((node.y1 - node.y0 + 1) / 2);
  std::array<Node, 4> quarters{{{.x0 = node.x0, .x1 = mx, .y0 = node.y0, .y1 = my, .members = {}},
                                {.x0 = mx, .x1 = node.x1, .y0 = node.y0, .y1 = my, .members = {}},
                                {.x0 = node.x0, .x1 = mx, .y0 = my, .y1 = node.y1, .members = {}},
                                {.x0 = mx, .x1 = node.x1, .y0 = my, .y1 = node.y1, .members = {}}}};
  for (const std::size_t i : node.members) {
    const std::size_t quarter = (xy[i][0] >= mx ? 1U : 0U) + (xy[i][1] >= my ? 2U : 0U);
    quarters.at(quarter).members.push_back(i);
  }
  std::vector<Node> out;
  for (auto& quarter : quarters) {
    if (!quarter.members.empty()) {
      out.push_back(std::move(quarter));
    }
  }
  return out;
}

/// @brief The quadtree's first nodes: `round(width / height)` columns over the box, empty ones
///        dropped.
/// @param xy The corners' pixels.
/// @param bounds Their box.
/// @return The non-empty columns, left to right.
std::vector<Node> column_nodes(std::span<const std::array<std::int64_t, 2>> xy,
                               const Bounds& bounds) {
  const std::int64_t width = bounds.x_max - bounds.x_min;
  const std::int64_t height = bounds.y_max - bounds.y_min;
  const std::int64_t roots = std::max<std::int64_t>(1, ((2 * width) + height) / (2 * height));
  std::vector<std::int64_t> edges;
  edges.reserve(at(roots + 1));
  for (std::int64_t k = 0; k <= roots; ++k) {
    edges.push_back(bounds.x_min + ((k * width) / roots));
  }
  std::vector<Node> nodes;
  nodes.reserve(at(roots));
  for (std::int64_t k = 0; k < roots; ++k) {
    nodes.push_back({.x0 = edges[at(k)],
                     .x1 = edges[at(k + 1)],
                     .y0 = bounds.y_min,
                     .y1 = bounds.y_max,
                     .members = {}});
  }
  for (std::size_t i = 0; i < xy.size(); ++i) {
    const auto root = std::distance(edges.begin(), std::ranges::upper_bound(edges, xy[i][0])) - 1;
    nodes[at(root)].members.push_back(i);
  }
  std::erase_if(nodes, [](const Node& node) { return node.members.empty(); });
  return nodes;
}

/// @brief The nodes to quarter next, in order: all of them in node order while a full round
///        fits the target, otherwise the most crowded first (ties in node order).
/// @param nodes The nodes.
/// @param target The number of nodes wanted.
/// @return Indices of the nodes holding several corners in a box larger than a pixel (a
///         one-pixel box cannot be split: its corners coincide).
std::vector<std::size_t> splitting_order(const std::vector<Node>& nodes, std::int64_t target) {
  std::vector<std::size_t> crowded;
  for (std::size_t k = 0; k < nodes.size(); ++k) {
    const Node& node = nodes[k];
    if (node.members.size() > 1 && (node.x1 - node.x0) * (node.y1 - node.y0) > 1) {
      crowded.push_back(k);
    }
  }
  if (std::cmp_greater(nodes.size() + (3 * crowded.size()), target)) {
    std::ranges::stable_sort(crowded, [&nodes](std::size_t a, std::size_t b) {
      return nodes[a].members.size() > nodes[b].members.size();
    });
  }
  return crowded;
}

/// @brief Quarter nodes in the given order until there are `target` of them.
/// @param nodes The nodes.
/// @param order The nodes to quarter, in order.
/// @param target The number of nodes wanted.
/// @param xy The corners' pixels.
/// @return The new nodes, each quartered node replaced by its quarters in place.
std::vector<Node> quarter_round(std::vector<Node> nodes, std::span<const std::size_t> order,
                                std::int64_t target,
                                std::span<const std::array<std::int64_t, 2>> xy) {
  // A quartered node's quarters; empty for a node left whole (a split is never empty).
  std::vector<std::vector<Node>> quartered(nodes.size());
  auto count = static_cast<std::int64_t>(nodes.size());
  for (const std::size_t k : order) {
    if (count >= target) {
      break;
    }
    quartered[k] = split(nodes[k], xy);
    count += static_cast<std::int64_t>(quartered[k].size()) - 1;
  }
  std::vector<Node> next;
  next.reserve(at(count));
  for (std::size_t k = 0; k < nodes.size(); ++k) {
    if (quartered[k].empty()) {
      next.push_back(std::move(nodes[k]));
    } else {
      std::ranges::move(quartered[k], std::back_inserter(next));
    }
  }
  return next;
}

/// @brief Each node's strongest corner: the highest response, ties to the earliest.
/// @param nodes The nodes.
/// @param response The corners' strengths.
/// @return The kept indices, ascending.
std::vector<std::int64_t> strongest(const std::vector<Node>& nodes,
                                    std::span<const std::int64_t> response) {
  std::vector<std::int64_t> kept;
  kept.reserve(nodes.size());
  for (const Node& node : nodes) {
    std::size_t best = node.members.front();
    for (const std::size_t i : node.members) {
      if (response[i] > response[best]) {
        best = i;
      }
    }
    kept.push_back(static_cast<std::int64_t>(best));
  }
  std::ranges::sort(kept);
  return kept;
}

/// @brief Spread checked corners with the quadtree (see `distribute`).
/// @param xy The corners' pixels.
/// @param response Their strengths.
/// @param bounds Their box.
/// @param target How many to keep.
/// @return The kept indices, ascending.
std::vector<std::int64_t> distribute_checked(std::span<const std::array<std::int64_t, 2>> xy,
                                             std::span<const std::int64_t> response,
                                             const Bounds& bounds, std::int64_t target) {
  if (xy.empty() || target == 0) {
    return {};
  }
  std::vector<Node> nodes = column_nodes(xy, bounds);
  while (std::cmp_less(nodes.size(), target)) {
    const std::vector<std::size_t> order = splitting_order(nodes, target);
    if (order.empty()) {
      break;
    }
    nodes = quarter_round(std::move(nodes), order, target, xy);
  }
  return strongest(nodes, response);
}

/// @brief A keypoint's orientation bin, from its disc's intensity centroid.
/// @param image The level.
/// @param x The keypoint's column, at least `kHalfPatchSize` inside.
/// @param y The keypoint's row, at least `kHalfPatchSize` inside.
/// @return The bin whose direction has the largest dot product with `(m10, m01)` (ties: the
///         lowest).
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (x, y), as a pixel
std::int64_t orientation(const image::Image& image, std::int64_t x, std::int64_t y) {
  std::int64_t m10 = 0;
  std::int64_t m01 = 0;
  for (std::int64_t v = -kHalfPatchSize; v <= kHalfPatchSize; ++v) {
    const std::int64_t half_width = kUmax.at(at(std::abs(v)));
    for (std::int64_t u = -half_width; u <= half_width; ++u) {
      const std::int64_t value = image::pixel(image, y + v, x + u);
      m10 += u * value;
      m01 += v * value;
    }
  }
  std::int64_t best = 0;
  std::int64_t best_dot = std::numeric_limits<std::int64_t>::min();
  for (std::int64_t k = 0; k < kAngleBins; ++k) {
    const auto& [c, s] = kRotations.at(at(k));
    const std::int64_t dot = (m10 * c) + (m01 * s);
    if (dot > best_dot) {
      best = k;
      best_dot = dot;
    }
  }
  return best;
}

/// @brief FAST scores over a detection region and a one-pixel ring around it (its maxima's
///        neighbours).
///
/// A score below the lower threshold reads as -1, which leaves every comparison that decides
/// a maximum at or above that threshold unchanged.
class ScoreWindow {
 public:
  /// @brief Score every pixel of the region and its ring.
  /// @param image The level.
  /// @param region The detection region, at least 4 pixels inside the level.
  /// @param floor The lower threshold.
  ScoreWindow(const image::Image& image, const Bounds& region, std::int64_t floor)
      : top_(region.y_min - 1),
        left_(region.x_min - 1),
        cols_(region.x_max - region.x_min + 2),
        scores_(at((region.y_max - region.y_min + 2) * cols_)) {
    const auto rows = static_cast<std::int64_t>(scores_.size()) / cols_;
    const CircleOffsets circle = circle_offsets(image.cols);
    RowRuns runs;
    for (std::int64_t r = 0; r < rows; ++r) {
      score_run(image.pixels, ((top_ + r) * image.cols) + left_, circle, floor,
                std::span(scores_).subspan(at(r * cols_), at(cols_)), runs);
    }
  }

  /// @brief A pixel's score.
  /// @param y Its row, in the region or its ring.
  /// @param x Its column, in the region or its ring.
  /// @return The score, or -1 below the lower threshold.
  [[nodiscard]] std::int64_t score(std::int64_t y, std::int64_t x) const {
    return scores_[at(((y - top_) * cols_) + (x - left_))];
  }

  /// @brief Whether a region pixel's score is strictly above all eight neighbours'.
  /// @param y Its row, in the region.
  /// @param x Its column, in the region.
  /// @return True for a 3x3 maximum.
  [[nodiscard]] bool is_maximum(std::int64_t y, std::int64_t x) const {
    const std::int64_t s = score(y, x);
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        if ((dx != 0 || dy != 0) && s <= score(y + dy, x + dx)) {
          return false;
        }
      }
    }
    return true;
  }

  /// @brief Whether a pixel is a corner at a threshold: a 3x3 maximum scoring at least it.
  /// @param y Its row, in the region.
  /// @param x Its column, in the region.
  /// @param threshold The threshold.
  /// @return True for a corner.
  [[nodiscard]] bool is_corner(std::int64_t y, std::int64_t x, std::int64_t threshold) const {
    return score(y, x) >= threshold && is_maximum(y, x);
  }

 private:
  /// @brief The window's first row (the ring above the region).
  std::int64_t top_;
  /// @brief The window's first column.
  std::int64_t left_;
  /// @brief The window's width.
  std::int64_t cols_;
  /// @brief The scores, row-major (from -1 to 254: 16 bits keep the window small).
  std::vector<std::int16_t> scores_;
};

/// @brief The threshold each region pixel's cell uses: `fast_threshold` where the cell has a
///        corner at it, else `min_fast_threshold`.
/// @param window The scores.
/// @param region The detection region.
/// @param options The settings.
/// @return One threshold per region pixel, row-major.
std::vector<std::int64_t> cell_thresholds(const ScoreWindow& window, const Bounds& region,
                                          const Options& options) {
  const std::int64_t width = region.x_max - region.x_min;
  const std::int64_t height = region.y_max - region.y_min;
  const std::int64_t n_cols = std::max<std::int64_t>(width / kCellSize, 1);
  const std::int64_t n_rows = std::max<std::int64_t>(height / kCellSize, 1);
  const std::int64_t cell_w = (width + n_cols - 1) / n_cols;
  const std::int64_t cell_h = (height + n_rows - 1) / n_rows;
  std::vector<std::int64_t> out(at(width * height));
  for (std::int64_t y0 = region.y_min; y0 < region.y_max; y0 += cell_h) {
    for (std::int64_t x0 = region.x_min; x0 < region.x_max; x0 += cell_w) {
      const Bounds cell{.x_min = x0,
                        .x_max = std::min(x0 + cell_w, region.x_max),
                        .y_min = y0,
                        .y_max = std::min(y0 + cell_h, region.y_max)};
      bool strong = false;
      for (std::int64_t y = cell.y_min; y < cell.y_max && !strong; ++y) {
        for (std::int64_t x = cell.x_min; x < cell.x_max && !strong; ++x) {
          strong = window.is_corner(y, x, options.fast_threshold);
        }
      }
      const std::int64_t threshold = strong ? options.fast_threshold : options.min_fast_threshold;
      for (std::int64_t y = cell.y_min; y < cell.y_max; ++y) {
        const auto row = std::next(out.begin(), ((y - region.y_min) * width) + (x0 - region.x_min));
        std::fill(row, std::next(row, cell.x_max - cell.x_min), threshold);
      }
    }
  }
  return out;
}

/// @brief One level's corners before distribution: their pixels and scores.
struct LevelCorners {
  /// @brief The corners' pixels, in raster order.
  std::vector<std::array<std::int64_t, 2>> xy;
  /// @brief Their FAST scores.
  std::vector<std::int64_t> response;
};

/// @brief Find one level's corners: 3x3 maxima of the FAST score, at the threshold each cell
///        uses.
/// @param image The level.
/// @param region The box keypoints may lie in.
/// @param options The settings.
/// @return The corners, in raster order.
LevelCorners level_corners(const image::Image& image, const Bounds& region,
                           const Options& options) {
  const ScoreWindow window(image, region, options.min_fast_threshold);
  const std::vector<std::int64_t> thresholds = cell_thresholds(window, region, options);
  const std::int64_t width = region.x_max - region.x_min;
  LevelCorners out;
  for (std::int64_t y = region.y_min; y < region.y_max; ++y) {
    for (std::int64_t x = region.x_min; x < region.x_max; ++x) {
      const std::int64_t t = thresholds[at(((y - region.y_min) * width) + (x - region.x_min))];
      if (window.is_corner(y, x, t)) {
        out.xy.push_back({x, y});
        out.response.push_back(window.score(y, x));
      }
    }
  }
  return out;
}

/// @brief Check the settings and build the pyramid.
/// @param image The image.
/// @param options The settings.
/// @return The levels, or `InvalidInput`.
Result<std::vector<image::Image>> checked_pyramid(const image::Image& image,
                                                  const Options& options) {
  if (auto ok = check_options(options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return image::pyramid(image, options.levels, options.scale_factor);
}

/// @brief Detect oriented keypoints on every level of a pyramid.
/// @param levels The pyramid.
/// @param options The settings, checked.
/// @return The keypoints by level, then row, then column.
std::vector<Keypoint> detect_levels(std::span<const image::Image> levels, const Options& options) {
  const std::vector<std::int64_t> budgets =
      level_budgets(options.n_features, options.levels, options.scale_factor);
  std::vector<Keypoint> out;
  const image::Image& full = levels.front();
  for (std::size_t level = 0; level < levels.size(); ++level) {
    const image::Image& pixels = levels[level];
    const Bounds region{.x_min = kEdgeThreshold,
                        .x_max = pixels.cols - kEdgeThreshold,
                        .y_min = kEdgeThreshold,
                        .y_max = pixels.rows - kEdgeThreshold};
    if (region.x_max <= region.x_min || region.y_max <= region.y_min || budgets[level] == 0) {
      continue;
    }
    const LevelCorners corners = level_corners(pixels, region, options);
    for (const std::int64_t i :
         distribute_checked(corners.xy, corners.response, region, budgets[level])) {
      const auto [x, y] = corners.xy[at(i)];
      out.push_back(
          {.xy = {image::full_image_coordinate(static_cast<double>(x), pixels.cols, full.cols),
                  image::full_image_coordinate(static_cast<double>(y), pixels.rows, full.rows)},
           .level = static_cast<std::int64_t>(level),
           .level_xy = {x, y},
           .response = corners.response[at(i)],
           .angle = orientation(pixels, x, y)});
    }
  }
  return out;
}

/// @brief `kPattern` rotated to every orientation bin.
using RotatedPatterns = std::array<std::array<std::array<std::int64_t, 4>, 256>, kAngleBins>;

/// @brief Rotate `kPattern` to every bin: `(x, y)` turns to `(x c - y s, x s + y c) / 2^14`,
///        rounded half up, for each bin's fixed-point `(c, s)`.
/// @return The rotated pairs, built once.
const RotatedPatterns& rotated_patterns() {
  static const RotatedPatterns kRotated = [] {
    constexpr std::int64_t kScale = image::pow2(kRotationBits);
    const auto turn = [](std::int64_t value) {
      return image::floor_div(value + (kScale / 2), kScale);
    };
    RotatedPatterns out{};
    for (std::size_t k = 0; k < out.size(); ++k) {
      const auto& [c, s] = kRotations.at(k);
      for (std::size_t i = 0; i < kPattern.size(); ++i) {
        const auto& [x1, y1, x2, y2] = kPattern.at(i);
        out.at(k).at(i) = {turn((x1 * c) - (y1 * s)), turn((x1 * s) + (y1 * c)),
                           turn((x2 * c) - (y2 * s)), turn((x2 * s) + (y2 * c))};
      }
    }
    return out;
  }();
  return kRotated;
}

/// @brief Every keypoint's rBRIEF descriptor, on its level blurred by `kBlurKernel`.
/// @param levels The pyramid.
/// @param keypoints The keypoints, each at least `kEdgeThreshold` inside its level.
/// @return One descriptor per keypoint, or the blur's error (none for a checked level).
Result<std::vector<Descriptor>> describe(std::span<const image::Image> levels,
                                         const std::vector<Keypoint>& keypoints) {
  const RotatedPatterns& patterns = rotated_patterns();
  std::vector<Descriptor> out(keypoints.size());
  for (std::size_t level = 0; level < levels.size(); ++level) {
    const auto smooth = image::blur(levels[level], kBlurKernel);
    if (!smooth) {
      return std::unexpected(smooth.error());
    }
    for (std::size_t n = 0; n < keypoints.size(); ++n) {
      const Keypoint& k = keypoints[n];
      if (std::cmp_not_equal(k.level, level)) {
        continue;
      }
      const auto [x, y] = k.level_xy;
      const auto& pairs = patterns.at(at(k.angle));
      for (std::size_t byte = 0; byte < kDescriptorBytes; ++byte) {
        unsigned bits = 0;
        for (std::size_t bit = 0; bit < 8; ++bit) {
          const auto& [x1, y1, x2, y2] = pairs.at((8 * byte) + bit);
          const bool darker =
              image::pixel(*smooth, y + y1, x + x1) < image::pixel(*smooth, y + y2, x + x2);
          bits |= (darker ? 1U : 0U) << bit;
        }
        out[n].at(byte) = static_cast<std::uint8_t>(bits);
      }
    }
  }
  return out;
}

/// @brief Detect keypoints on a checked pyramid and describe them.
/// @param levels The pyramid.
/// @param options The settings, checked.
/// @return The features, or the blur's error (none for a checked level).
Result<Features> extract_checked(std::span<const image::Image> levels, const Options& options) {
  std::vector<Keypoint> keypoints = detect_levels(levels, options);
  auto descriptors = describe(levels, keypoints);
  if (!descriptors) {
    return std::unexpected(descriptors.error());
  }
  return Features{.keypoints = std::move(keypoints),
                  .descriptors = std::move(*descriptors),
                  .scale_factor = options.scale_factor,
                  .levels = options.levels};
}

}  // namespace

Result<void> check_options(const Options& options) {
  if (auto ok = check_range("n_features", options.n_features, 1, kMaxFeatures); !ok) {
    return ok;
  }
  if (auto ok = check_range("levels", options.levels, 1, kMaxLevels); !ok) {
    return ok;
  }
  if (auto ok = image::check_scale_factor(options.scale_factor, "scale_factor"); !ok) {
    return ok;
  }
  if (auto ok = check_range("fast_threshold", options.fast_threshold, 1, kMaxFastThreshold); !ok) {
    return ok;
  }
  return check_range("min_fast_threshold", options.min_fast_threshold, 1, options.fast_threshold);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (n_features, levels), as in Python
std::vector<std::int64_t> level_budgets(std::int64_t n_features, std::int64_t levels,
                                        double scale_factor) {
  const double factor = 1.0 / scale_factor;
  double power = 1.0;
  for (std::int64_t level = 0; level < levels; ++level) {
    power *= factor;
  }
  double desired = static_cast<double>(n_features) * (1.0 - factor) / (1.0 - power);
  std::vector<std::int64_t> budgets;
  std::int64_t total = 0;
  for (std::int64_t level = 0; level + 1 < levels; ++level) {
    budgets.push_back(static_cast<std::int64_t>(std::floor(desired + 0.5)));
    total += budgets.back();
    desired *= factor;
  }
  budgets.push_back(std::max<std::int64_t>(n_features - total, 0));
  return budgets;
}

Result<std::vector<std::int64_t>> fast_scores(const image::Image& image) {
  if (auto ok = image::check_image(image, "image"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  constexpr std::int64_t kRadius = 3;
  std::vector<std::int64_t> out(image.pixels.size(), -1);
  if (image.rows <= 2 * kRadius || image.cols <= 2 * kRadius) {
    return out;
  }
  const CircleOffsets circle = circle_offsets(image.cols);
  const std::int64_t width = image.cols - (2 * kRadius);
  std::vector<std::int16_t> row(at(width));
  RowRuns runs;
  for (std::int64_t r = kRadius; r < image.rows - kRadius; ++r) {
    const std::int64_t start = (r * image.cols) + kRadius;
    score_run(image.pixels, start, circle, 0, row, runs);
    std::ranges::copy(row, std::next(out.begin(), start));
  }
  return out;
}

Result<std::vector<std::int64_t>> distribute(std::span<const std::array<std::int64_t, 2>> xy,
                                             std::span<const std::int64_t> response,
                                             const Bounds& bounds, std::int64_t target) {
  if (response.size() != xy.size()) {
    return fail(ErrorKind::kInvalidInput, "response: expected " + std::to_string(xy.size()) +
                                              " integers, got " + std::to_string(response.size()));
  }
  const auto in_range = [](std::int64_t low, std::int64_t high) {
    return low >= 0 && low < high && high <= image::kMaxSide;
  };
  if (!in_range(bounds.x_min, bounds.x_max) || !in_range(bounds.y_min, bounds.y_max)) {
    return fail(ErrorKind::kInvalidInput, "bounds: expected a non-empty box inside [0, " +
                                              std::to_string(image::kMaxSide) + "]^2");
  }
  for (const auto& [x, y] : xy) {
    if (x < bounds.x_min || x >= bounds.x_max || y < bounds.y_min || y >= bounds.y_max) {
      return fail(ErrorKind::kInvalidInput, "xy: every corner must lie inside bounds");
    }
  }
  if (auto ok = check_range("target", target, 0, kMaxFeatures); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return distribute_checked(xy, response, bounds, target);
}

Result<std::vector<Keypoint>> detect(const image::Image& image, const Options& options) {
  const auto levels = checked_pyramid(image, options);
  if (!levels) {
    return std::unexpected(levels.error());
  }
  return detect_levels(*levels, options);
}

Result<Features> extract(const image::Image& image, const Options& options) {
  const auto levels = checked_pyramid(image, options);
  if (!levels) {
    return std::unexpected(levels.error());
  }
  return extract_checked(*levels, options);
}

Result<Features> extract_levels(std::span<const image::Image> levels, const Options& options) {
  if (auto ok = check_options(options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = image::check_pyramid(levels, options.levels, options.scale_factor, "levels"); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return extract_checked(levels, options);
}

}  // namespace slamlab::foundations::orb
