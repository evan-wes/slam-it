/// @file
/// @brief Implementation of the atlas declared in slam_it/edge/atlas.hpp.
#include "slam_it/edge/atlas.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "slam_it/edge/covisibility.hpp"
#include "slam_it/foundations/sim3.hpp"

namespace slam_it::edge {

namespace {

/// @brief An id of a map as the run's, by its table of run ids; -1 kept.
/// @param runs The map's ids' run ids.
/// @param id The id, or -1.
/// @return The run's id, or -1.
std::int64_t run_of(const std::vector<std::int64_t>& runs, std::int64_t id) {
  return id >= 0 ? runs[static_cast<std::size_t>(id)] : -1;
}

/// @brief Two keyframes' run ids, the lower first.
/// @param runs The map's keyframe ids' run ids.
/// @param first One keyframe's id in the map.
/// @param second The other's.
/// @return The run ids, ascending.
std::array<std::int64_t, 2> pair_of(const std::vector<std::int64_t>& runs, std::int64_t first,
                                    std::int64_t second) {
  const std::int64_t a = run_of(runs, first);
  const std::int64_t b = run_of(runs, second);
  return {std::min(a, b), std::max(a, b)};
}

/// @brief A map's strong links, by the run's ids, each the lower first.
/// @param m The map.
/// @param runs Its keyframe ids' run ids.
/// @param options The settings.
/// @param out Where they are appended.
void append_links(const Map& m, const std::vector<std::int64_t>& runs, const Options& options,
                  std::vector<std::array<std::int64_t, 3>>& out) {
  for (const auto& link : strong_links(m, options)) {
    const auto [a, b] = pair_of(runs, link[0], link[1]);
    out.push_back({a, b, link[2]});
  }
}

}  // namespace

std::optional<std::int64_t> Atlas::map_id() const {
  const std::scoped_lock lock(mutex_);
  return map_id_;
}

std::int64_t Atlas::frames() const {
  const std::scoped_lock lock(mutex_);
  return frames_;
}

std::int64_t Atlas::frame() const {
  const std::scoped_lock lock(mutex_);
  return frame_held();
}

std::int64_t Atlas::frame_held() const { return ended_ ? frames_ - 1 : frames_; }

void Atlas::end() {
  const std::scoped_lock lock(mutex_);
  ended_ = true;
}

void Atlas::start() {
  const std::scoped_lock lock(mutex_);
  map_id_ = maps_++;
  runs_ = {};
}

void Atlas::extend(const Map& m) {
  const std::scoped_lock lock(mutex_);
  extend_held(m);
}

void Atlas::extend_held(const Map& m) {
  const std::int64_t frame = frame_held();
  const std::int64_t number = map_id_.value_or(0);
  while (std::cmp_less(runs_.points.size(), m.next_pt_id)) {
    runs_.points.push_back(static_cast<std::int64_t>(point_lives_.size()));
    point_lives_.push_back(Life{frame, -1, -1});
    point_maps_.push_back(number);
    past_points_.emplace_back(Eigen::Vector3d::Zero());
  }
  while (std::cmp_less(runs_.keyframes.size(), m.next_kf_id)) {
    runs_.keyframes.push_back(static_cast<std::int64_t>(keyframe_lives_.size()));
    keyframe_lives_.push_back(Life{frame, -1, -1});
    keyframe_maps_.push_back(number);
    past_times_.push_back(0.0);
    past_poses_.emplace_back(Eigen::Matrix4d::Identity());
    past_parents_.push_back(-1);
  }
}

void Atlas::date(const Map* m) {
  const std::scoped_lock lock(mutex_);
  record_held(m);
  ++frames_;
}

void Atlas::tick() {
  const std::scoped_lock lock(mutex_);
  ++frames_;
}

void Atlas::record(const Map* m) {
  const std::scoped_lock lock(mutex_);
  record_held(m);
}

void Atlas::record_held(const Map* m) {
  const std::int64_t frame = frame_held();
  if (m != nullptr && map_id_) {
    extend_held(*m);
    for (std::size_t g = 0; g < m->gone_pt_id.size(); ++g) {
      const auto id = static_cast<std::size_t>(run_of(runs_.points, m->gone_pt_id[g]));
      point_lives_[id][1] = frame;
      point_lives_[id][2] = m->gone_pt_how[g];
      past_points_[id] = m->gone_pt_xyz[g];
    }
    for (std::size_t g = 0; g < m->gone_kf_id.size(); ++g) {
      const auto id = static_cast<std::size_t>(run_of(runs_.keyframes, m->gone_kf_id[g]));
      keyframe_lives_[id][1] = frame;
      keyframe_lives_[id][2] = static_cast<std::int64_t>(Removal::kCulled);
      past_times_[id] = m->gone_kf_time[g];
      past_poses_[id] = m->gone_kf_pose[g];
      past_parents_[id] = run_of(runs_.keyframes, m->gone_kf_parent[g]);
      if (m->gone_kf_parent[g] >= 0) {
        culled_[static_cast<std::int64_t>(id)] = {run_of(runs_.keyframes, m->gone_kf_parent[g]),
                                                  m->gone_kf_relative[g]};
      }
    }
  }
}

void Atlas::close(Map m, std::optional<Database> database, bool reset, bool dated) {
  const std::scoped_lock lock(mutex_);
  const std::int64_t number = map_id_.value_or(0);
  map_id_.reset();
  pending_.clear();
  if (!reset) {
    const auto at = static_cast<std::size_t>(
        std::ranges::lower_bound(kept_, number, {}, &Indexed::number) - kept_.begin());
    kept_.insert(kept_.begin() + static_cast<std::ptrdiff_t>(at),
                 Indexed{.number = number,
                         .map = std::move(m),
                         .database = database ? std::move(*database) : Database{}});
    kept_runs_.insert(kept_runs_.begin() + static_cast<std::ptrdiff_t>(at), std::move(runs_));
    runs_ = {};
    return;
  }
  const std::int64_t frame = dated ? frames_ - 1 : frames_;
  const auto code = static_cast<std::int64_t>(Removal::kReset);
  for (std::size_t p = 0; p < point_count(m); ++p) {
    const auto id = static_cast<std::size_t>(run_of(runs_.points, m.pt_id[p]));
    past_points_[id] = m.pt_xyz[p];
    point_lives_[id][1] = frame;
    point_lives_[id][2] = code;
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    const auto id = static_cast<std::size_t>(run_of(runs_.keyframes, m.kf_id[k]));
    past_times_[id] = m.kf_time[k];
    past_poses_[id] = m.kf_pose[k];
    past_parents_[id] = run_of(runs_.keyframes, m.kf_parent[k]);
    keyframe_lives_[id][1] = frame;
    keyframe_lives_[id][2] = code;
  }
  append_links(m, runs_.keyframes, options_, past_links_);
  for (const auto& [older, newer] : m.loop_edges) {
    past_loops_.push_back(pair_of(runs_.keyframes, older, newer));
  }
  std::erase_if(placements_, [number](const FramePlace& p) { return p.map == number; });
  runs_ = {};
}

std::size_t Atlas::kept_at(std::int64_t number) const {
  return static_cast<std::size_t>(std::ranges::find(kept_, number, &Indexed::number) -
                                  kept_.begin());
}

Indexed Atlas::resume(std::int64_t number) {
  const std::scoped_lock lock(mutex_);
  const std::size_t at = kept_at(number);
  Indexed out = std::move(kept_[at]);
  map_id_ = number;
  runs_ = std::move(kept_runs_[at]);
  kept_.erase(kept_.begin() + static_cast<std::ptrdiff_t>(at));
  kept_runs_.erase(kept_runs_.begin() + static_cast<std::ptrdiff_t>(at));
  return out;
}

Database Atlas::merge(std::int64_t matched, const Eigen::Matrix4d& to_world) {
  const std::scoped_lock lock(mutex_);
  const std::int64_t current = map_id_.value_or(0);
  const std::size_t at = kept_at(matched);
  Database out = std::move(kept_[at].database);
  Runs runs = std::move(kept_runs_[at]);
  kept_.erase(kept_.begin() + static_cast<std::ptrdiff_t>(at));
  kept_runs_.erase(kept_runs_.begin() + static_cast<std::ptrdiff_t>(at));
  const Eigen::Matrix4d back = foundations::sim3::inverse(to_world);
  const double scale = loops::scale_of(back);
  for (const std::int64_t id : runs_.points) {
    const auto p = static_cast<std::size_t>(id);
    if (point_lives_[p][1] >= 0) {  // removed: where it last was, carried
      past_points_[p] =
          to_world.topLeftCorner<3, 3>() * past_points_[p] + to_world.topRightCorner<3, 1>();
    }
  }
  for (const std::int64_t id : runs_.keyframes) {
    const auto k = static_cast<std::size_t>(id);
    if (keyframe_lives_[k][1] >= 0) {
      past_poses_[k] = loops::rigid_of(past_poses_[k] * back);
    }
    if (const auto gone = culled_.find(id); gone != culled_.end()) {
      gone->second.second.topRightCorner<3, 1>() /= scale;
    }
  }
  runs.keyframes.insert(runs.keyframes.end(), runs_.keyframes.begin(), runs_.keyframes.end());
  runs.points.insert(runs.points.end(), runs_.points.begin(), runs_.points.end());
  runs_ = std::move(runs);
  merged_into_[current] = matched;
  map_id_ = matched;
  return out;
}

void Atlas::place_frame(const Map& m, double timestamp, std::int64_t reference,
                        const Eigen::Matrix4d& pose) {
  const std::scoped_lock lock(mutex_);
  extend_held(m);
  const auto k = static_cast<std::size_t>(keyframe_row(m, reference));
  placements_.push_back({.timestamp = timestamp,
                         .map = map_id_.value_or(0),
                         .reference = run_of(runs_.keyframes, reference),
                         .relative = pose * foundations::sim3::inverse(m.kf_pose[k]),
                         .tracked = pose});
}

void Atlas::place_keyframe(const Map& m, double timestamp, const Eigen::Matrix4d& pose) {
  const std::scoped_lock lock(mutex_);
  extend_held(m);
  placements_.push_back({.timestamp = timestamp,
                         .map = map_id_.value_or(0),
                         .reference = run_of(runs_.keyframes, m.kf_id.back()),
                         .relative = Eigen::Matrix4d::Identity(),
                         .tracked = pose});
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): (frame, timestamp), Python's order
void Atlas::place_pending(std::int64_t frame, double timestamp, const Eigen::Matrix4d& pose) {
  const std::scoped_lock lock(mutex_);
  pending_[frame] = placements_.size();
  placements_.push_back({.timestamp = timestamp,
                         .map = map_id_.value_or(0),
                         .reference = -1,
                         .relative = Eigen::Matrix4d::Identity(),
                         .tracked = pose});
}

void Atlas::resolve(std::int64_t frame, const Map& m) {
  const std::scoped_lock lock(mutex_);
  const auto at = pending_.find(frame);
  if (at == pending_.end()) {
    return;
  }
  extend_held(m);
  placements_[at->second].reference = run_of(runs_.keyframes, m.kf_id.back());
  pending_.erase(at);
}

void Atlas::record_whole(const Map& m, const Runs& runs, Lifetimes& out) const {
  for (std::size_t p = 0; p < point_count(m); ++p) {
    out.points[static_cast<std::size_t>(run_of(runs.points, m.pt_id[p]))] = m.pt_xyz[p];
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    const auto id = static_cast<std::size_t>(run_of(runs.keyframes, m.kf_id[k]));
    out.keyframe_times[id] = m.kf_time[k];
    out.keyframe_poses[id] = m.kf_pose[k];
    out.keyframe_parents[id] = run_of(runs.keyframes, m.kf_parent[k]);
  }
  append_links(m, runs.keyframes, options_, out.keyframe_links);
  for (const auto& [older, newer] : m.loop_edges) {
    out.keyframe_loops.push_back(pair_of(runs.keyframes, older, newer));
  }
  for (const auto& [older, newer] : m.merge_edges) {
    out.keyframe_merges.push_back(pair_of(runs.keyframes, older, newer));
  }
}

Lifetimes Atlas::lifetimes(const Map* m) const {
  const std::scoped_lock lock(mutex_);
  Lifetimes out{.points = past_points_,
                .point_lives = point_lives_,
                .keyframe_times = past_times_,
                .keyframe_poses = past_poses_,
                .keyframe_lives = keyframe_lives_,
                .keyframe_parents = past_parents_,
                .keyframe_links = past_links_,
                .keyframe_loops = past_loops_,
                .keyframe_merges = {},
                .point_maps = {},
                .keyframe_maps = {}};
  for (std::size_t i = 0; i < kept_.size(); ++i) {
    record_whole(kept_[i].map, kept_runs_[i], out);
  }
  if (m != nullptr && map_id_) {
    record_whole(*m, runs_, out);
  }
  std::ranges::sort(out.keyframe_links);
  for (const std::int64_t made_in : point_maps_) {
    out.point_maps.push_back(ends_in(made_in));
  }
  for (const std::int64_t made_in : keyframe_maps_) {
    out.keyframe_maps.push_back(ends_in(made_in));
  }
  return out;
}

Trajectory Atlas::trajectory(const Map* m) const {
  const std::scoped_lock lock(mutex_);
  std::map<std::int64_t, Eigen::Matrix4d> last;
  for (std::size_t id = 0; id < past_poses_.size(); ++id) {
    if (keyframe_lives_[id][1] >= 0) {
      last[static_cast<std::int64_t>(id)] = past_poses_[id];  // removed: where it last was
    }
  }
  const auto anchor_all = [&last](const Map& whole, const Runs& runs) {
    for (std::size_t k = 0; k < keyframe_count(whole); ++k) {
      last[run_of(runs.keyframes, whole.kf_id[k])] = whole.kf_pose[k];
    }
  };
  for (std::size_t i = 0; i < kept_.size(); ++i) {
    anchor_all(kept_[i].map, kept_runs_[i]);
  }
  if (m != nullptr && map_id_) {
    anchor_all(*m, runs_);
  }
  Trajectory out;
  for (const FramePlace& placed : placements_) {
    const auto [keyframe, pose] = through_culled(placed.reference, placed.relative);
    const auto anchor = last.find(keyframe);
    out.timestamps.push_back(placed.timestamp);
    out.poses.push_back(anchor == last.end() ? placed.tracked
                                             : Eigen::Matrix4d(pose * anchor->second));
    out.tracked.push_back(placed.tracked);
    out.maps.push_back(ends_in(placed.map));
    out.tracked_maps.push_back(placed.map);
  }
  return out;
}

std::optional<Eigen::Matrix4d> Atlas::pose_by(const Map& m, std::int64_t keyframe,
                                              const Eigen::Matrix4d& relative) {
  const std::scoped_lock lock(mutex_);
  extend_held(m);
  const auto [found, pose] = through_culled(run_of(runs_.keyframes, keyframe), relative);
  for (std::size_t k = 0; k < m.kf_id.size(); ++k) {
    if (run_of(runs_.keyframes, m.kf_id[k]) == found) {
      return Eigen::Matrix4d(pose * m.kf_pose[k]);
    }
  }
  return std::nullopt;
}

std::pair<std::int64_t, Eigen::Matrix4d> Atlas::through_culled(std::int64_t keyframe,
                                                               Eigen::Matrix4d relative) const {
  for (auto gone = culled_.find(keyframe); gone != culled_.end(); gone = culled_.find(keyframe)) {
    relative = relative * gone->second.second;
    keyframe = gone->second.first;
  }
  return {keyframe, relative};
}

std::int64_t Atlas::ends_in(std::int64_t number) const {
  for (auto into = merged_into_.find(number); into != merged_into_.end();
       into = merged_into_.find(number)) {
    number = into->second;
  }
  return number;
}

std::int64_t Atlas::run_id(std::int64_t id) const {
  const std::scoped_lock lock(mutex_);
  return run_of(runs_.keyframes, id);
}

std::int64_t Atlas::point_run_id(std::int64_t id) const {
  const std::scoped_lock lock(mutex_);
  return run_of(runs_.points, id);
}

}  // namespace slam_it::edge
