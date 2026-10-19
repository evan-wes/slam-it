/// @file
/// @brief Implementation of the atlas declared in slam_it/edge/atlas.hpp.
#include "slam_it/edge/atlas.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "slam_it/edge/covisibility.hpp"
#include "slam_it/foundations/sim3.hpp"

namespace slam_it::edge {

std::optional<std::int64_t> Atlas::map_id() const {
  const std::scoped_lock lock(mutex_);
  return map_id_;
}

std::int64_t Atlas::keyframe_base() const {
  const std::scoped_lock lock(mutex_);
  return kf_base_;
}

std::int64_t Atlas::point_base() const {
  const std::scoped_lock lock(mutex_);
  return pt_base_;
}

std::int64_t Atlas::frames() const {
  const std::scoped_lock lock(mutex_);
  return frames_;
}

std::int64_t Atlas::frame() const {
  const std::scoped_lock lock(mutex_);
  return ended_ ? frames_ - 1 : frames_;
}

void Atlas::end() {
  const std::scoped_lock lock(mutex_);
  ended_ = true;
}

void Atlas::start() {
  const std::scoped_lock lock(mutex_);
  map_id_ = maps_++;
  kf_base_ = static_cast<std::int64_t>(keyframe_lives_.size());
  pt_base_ = static_cast<std::int64_t>(point_lives_.size());
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
  const std::int64_t frame = ended_ ? frames_ - 1 : frames_;
  if (m == nullptr || !map_id_) {
    return;
  }
  const auto points = static_cast<std::size_t>(pt_base_ + m->next_pt_id);
  point_lives_.resize(points, Life{frame, -1, -1});
  point_maps_.resize(points, *map_id_);
  past_points_.resize(points, Eigen::Vector3d::Zero());
  const auto keyframes = static_cast<std::size_t>(kf_base_ + m->next_kf_id);
  keyframe_lives_.resize(keyframes, Life{frame, -1, -1});
  keyframe_maps_.resize(keyframes, *map_id_);
  past_times_.resize(keyframes, 0.0);
  past_poses_.resize(keyframes, Eigen::Matrix4d::Identity());
  past_parents_.resize(keyframes, -1);
  for (std::size_t g = 0; g < m->gone_pt_id.size(); ++g) {
    const auto id = static_cast<std::size_t>(pt_base_ + m->gone_pt_id[g]);
    point_lives_[id][1] = frame;
    point_lives_[id][2] = m->gone_pt_how[g];
    past_points_[id] = m->gone_pt_xyz[g];
  }
  for (std::size_t g = 0; g < m->gone_kf_id.size(); ++g) {
    const auto id = static_cast<std::size_t>(kf_base_ + m->gone_kf_id[g]);
    keyframe_lives_[id][1] = frame;
    keyframe_lives_[id][2] = static_cast<std::int64_t>(Removal::kCulled);
    past_times_[id] = m->gone_kf_time[g];
    past_poses_[id] = m->gone_kf_pose[g];
    past_parents_[id] = run_id(m->gone_kf_parent[g]);
    if (m->gone_kf_parent[g] >= 0) {
      culled_[static_cast<std::int64_t>(id)] = {run_id(m->gone_kf_parent[g]),
                                                m->gone_kf_relative[g]};
    }
  }
}

void Atlas::close(const Map& m, bool reset) {
  const std::scoped_lock lock(mutex_);
  const std::int64_t frame = frames_ - 1;  // dated already
  const auto code = static_cast<std::int64_t>(Removal::kReset);
  for (std::size_t p = 0; p < point_count(m); ++p) {
    const auto id = static_cast<std::size_t>(pt_base_ + m.pt_id[p]);
    past_points_[id] = m.pt_xyz[p];
    if (reset) {
      point_lives_[id][1] = frame;
      point_lives_[id][2] = code;
    }
  }
  for (std::size_t k = 0; k < keyframe_count(m); ++k) {
    const auto id = static_cast<std::size_t>(kf_base_ + m.kf_id[k]);
    past_times_[id] = m.kf_time[k];
    past_poses_[id] = m.kf_pose[k];
    past_parents_[id] = run_id(m.kf_parent[k]);
    if (reset) {
      keyframe_lives_[id][1] = frame;
      keyframe_lives_[id][2] = code;
    }
  }
  for (const auto& link : strong_links(m, options_)) {
    past_links_.push_back({kf_base_ + link[0], kf_base_ + link[1], link[2]});
  }
  for (const auto& [older, newer] : m.loop_edges) {
    past_loops_.push_back({kf_base_ + older, kf_base_ + newer});
  }
  if (reset && map_id_) {  // its frames' poses go with it
    std::erase_if(placements_, [this](const FramePlace& p) { return p.map == *map_id_; });
  }
  map_id_.reset();
  pending_.clear();
}

void Atlas::place_frame(const Map& m, double timestamp, std::int64_t reference,
                        const Eigen::Matrix4d& pose) {
  const std::scoped_lock lock(mutex_);
  const auto k = static_cast<std::size_t>(keyframe_row(m, reference));
  placements_.push_back({.timestamp = timestamp,
                         .map = map_id_.value_or(0),
                         .reference = kf_base_ + reference,
                         .relative = pose * foundations::sim3::inverse(m.kf_pose[k]),
                         .tracked = pose});
}

void Atlas::place_keyframe(const Map& m, double timestamp, const Eigen::Matrix4d& pose) {
  const std::scoped_lock lock(mutex_);
  placements_.push_back({.timestamp = timestamp,
                         .map = map_id_.value_or(0),
                         .reference = kf_base_ + m.kf_id.back(),
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
  placements_[at->second].reference = kf_base_ + m.kf_id.back();
  pending_.erase(at);
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
                .point_maps = point_maps_,
                .keyframe_maps = keyframe_maps_};
  if (m != nullptr) {
    for (std::size_t p = 0; p < point_count(*m); ++p) {
      out.points[static_cast<std::size_t>(pt_base_ + m->pt_id[p])] = m->pt_xyz[p];
    }
    for (std::size_t k = 0; k < keyframe_count(*m); ++k) {
      const auto id = static_cast<std::size_t>(kf_base_ + m->kf_id[k]);
      out.keyframe_times[id] = m->kf_time[k];
      out.keyframe_poses[id] = m->kf_pose[k];
      out.keyframe_parents[id] = run_id(m->kf_parent[k]);
    }
    for (const auto& link : strong_links(*m, options_)) {
      out.keyframe_links.push_back({kf_base_ + link[0], kf_base_ + link[1], link[2]});
    }
    for (const auto& [older, newer] : m->loop_edges) {
      out.keyframe_loops.push_back({kf_base_ + older, kf_base_ + newer});
    }
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
  for (std::size_t id = 0; id < keyframe_maps_.size(); ++id) {
    const auto at = static_cast<std::int64_t>(id);
    if (keyframe_lives_[id][1] < 0 && (!map_id_ || keyframe_maps_[id] != *map_id_)) {
      last[at] = past_poses_[id];  // a closed map's, kept
    }
  }
  if (m != nullptr) {
    for (std::size_t k = 0; k < keyframe_count(*m); ++k) {
      last[kf_base_ + m->kf_id[k]] = m->kf_pose[k];
    }
  }
  Trajectory out;
  for (const FramePlace& placed : placements_) {
    const auto [keyframe, pose] = through_culled(placed.reference, placed.relative);
    const auto anchor = last.find(keyframe);
    out.timestamps.push_back(placed.timestamp);
    out.poses.push_back(anchor == last.end() ? placed.tracked
                                             : Eigen::Matrix4d(pose * anchor->second));
    out.tracked.push_back(placed.tracked);
    out.maps.push_back(placed.map);
  }
  return out;
}

std::optional<Eigen::Matrix4d> Atlas::pose_by(const Map& m, std::int64_t keyframe,
                                              const Eigen::Matrix4d& relative) const {
  const auto [found, pose] = [&] {
    const std::scoped_lock lock(mutex_);
    auto out = through_culled(kf_base_ + keyframe, relative);
    out.first -= kf_base_;  // by the map's id
    return out;
  }();
  const std::int64_t row = keyframe_row(m, found);
  if (row < 0) {
    return std::nullopt;
  }
  return Eigen::Matrix4d(pose * m.kf_pose[static_cast<std::size_t>(row)]);
}

std::pair<std::int64_t, Eigen::Matrix4d> Atlas::through_culled(std::int64_t keyframe,
                                                               Eigen::Matrix4d relative) const {
  for (auto gone = culled_.find(keyframe); gone != culled_.end(); gone = culled_.find(keyframe)) {
    relative = relative * gone->second.second;
    keyframe = gone->second.first;
  }
  return {keyframe, relative};
}

std::int64_t Atlas::run_id(std::int64_t id) const { return id >= 0 ? kf_base_ + id : -1; }

}  // namespace slam_it::edge
