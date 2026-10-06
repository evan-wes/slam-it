/// @file
/// @brief Implementation of the pipeline declared in slamlab/edge/pipeline.hpp.
#include "slamlab/edge/pipeline.hpp"

#include <chrono>
#include <vector>

#include "slamlab/edge/mapping.hpp"
#include "slamlab/edge/tracking.hpp"
#include "slamlab/foundations/se3.hpp"

namespace slamlab::edge {

using foundations::ErrorKind;
using foundations::fail;
using foundations::Result;

const char* name(State state) {
  switch (state) {
    case State::kNotInitialized:
      return "not_initialized";
    case State::kTracking:
      return "tracking";
    case State::kLost:
      return "lost";
    case State::kCount:
      break;
  }
  return "unknown";
}

Result<Pipeline> Pipeline::create(Mode mode, Camera camera, Options options) {
  if (auto ok = check_camera(camera); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (auto ok = check_options(options); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  if (mode != Mode::kMonocular && !(camera.baseline > 0.0)) {
    return fail(ErrorKind::kInvalidInput,
                std::string("camera.baseline: ") + name(mode) + " needs a positive baseline");
  }
  return Pipeline(mode, std::move(camera), options);
}

foundations::orb::Options Pipeline::extraction() const {
  return edge::extraction(options_, mode_ == Mode::kMonocular && state_ == State::kNotInitialized);
}

Result<FrameResult> Pipeline::track(const Frame& frame) {
  const auto start = std::chrono::steady_clock::now();
  std::optional<Tracked> tracked;
  bool keyframe = false;
  if (state_ == State::kNotInitialized) {
    auto started = initialize(frame);
    if (!started) {
      return std::unexpected(std::move(started.error()));
    }
    tracked = std::move(*started);
    keyframe = tracked.has_value();
  } else if (state_ == State::kTracking) {
    auto placed = track_frame(frame);
    if (!placed) {
      return std::unexpected(std::move(placed.error()));
    }
    tracked = std::move(placed->first);
    keyframe = placed->second;
  }
  const std::chrono::duration<double> took = std::chrono::steady_clock::now() - start;
  return FrameResult{.timestamp = frame.timestamp,
                     .state = state_,
                     .pose = tracked ? std::optional<Eigen::Matrix4d>(tracked->pose) : std::nullopt,
                     .tracked = tracked ? inliers(*tracked) : 0,
                     .keyframe = keyframe,
                     .seconds = took.count()};
}

Result<std::optional<Tracked>> Pipeline::initialize(const Frame& frame) {
  auto started = mode_ == Mode::kMonocular ? initialize_monocular(frame)
                                           : initialize_from_depth(frame, options_);
  if (!started) {
    return std::unexpected(std::move(started.error()));
  }
  if (!*started) {
    return std::nullopt;
  }
  map_ = std::move((*started)->map);
  state_ = State::kTracking;
  Tracked tracked = (*started)->tracked;
  if (auto ok = accept(frame, tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return tracked;
}

Result<std::optional<Started>> Pipeline::initialize_monocular(const Frame& frame) {
  if (std::cmp_less(keypoint_count(frame), options_.min_init_features)) {
    reference_.reset();  // too few features: ORB-SLAM drops its initializer
    return std::nullopt;
  }
  if (!reference_) {
    reference_ = frame;
    return std::nullopt;
  }
  auto matched = match_for_initialization(*reference_, frame, options_);
  if (!matched) {
    return std::unexpected(std::move(matched.error()));
  }
  if (std::cmp_less(matched->size(), options_.init_min_matches)) {
    reference_ = frame;  // the views no longer overlap enough: start again
    return std::nullopt;
  }
  // Nothing keeps the reference: a later frame may have the parallax.
  return edge::initialize_monocular(camera_, *reference_, frame, options_);
}

Result<void> Pipeline::accept(const Frame& frame, const Tracked& tracked) {
  velocity_.reset();
  if (last_tracked_ && failures_ == 0) {
    auto inverse = foundations::se3::inverse(last_tracked_->pose);
    if (!inverse) {
      return std::unexpected(std::move(inverse.error()));
    }
    velocity_ = Eigen::Matrix4d(tracked.pose * *inverse);
  }
  last_frame_ = frame;
  last_tracked_ = tracked;
  failures_ = 0;
  return {};
}

void Pipeline::record_failure() {
  ++failures_;
  if (failures_ >= options_.lost_frames) {
    state_ = State::kLost;
  }
}

Result<std::pair<std::optional<Tracked>, bool>> Pipeline::place(const Map& m,
                                                                const Frame& frame) const {
  if (!last_frame_ || !last_tracked_) {
    return fail(ErrorKind::kInvalidInput, "track: no last frame to track from");
  }
  // No velocity (just initialised, or after a failure): predict no motion. ORB-SLAM tracks its
  // reference keyframe by bag of words here; the lite pipeline has none.
  const Eigen::Matrix4d predicted = predict(last_tracked_->pose, velocity_);
  auto tracked =
      track_last_frame(m, camera_, frame, *last_frame_, *last_tracked_, predicted, mode_, options_);
  if (!tracked) {
    return std::unexpected(std::move(tracked.error()));
  }
  if (*tracked) {
    return std::pair{std::move(*tracked), false};
  }
  auto recovered = relocalize_locally(m, camera_, frame, options_);
  if (!recovered) {
    return std::unexpected(std::move(recovered.error()));
  }
  return std::pair{std::move(*recovered), true};
}

Result<std::optional<Tracked>> Pipeline::refine(Map& m, const Frame& frame, const Tracked& placed,
                                                bool wide, bool recent) const {
  auto local =
      track_local_map(m, camera_, frame, placed, mode_, options_, wide ? kRecoveryWiden : 1.0);
  if (!local) {
    return std::unexpected(std::move(local.error()));
  }
  const std::vector<std::int64_t> seen(local->visible.size(), 1);
  const std::vector<std::int64_t> unseen(local->visible.size(), 0);
  if (auto ok = count(m, local->visible, seen, unseen); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::vector<std::int64_t> found(local->found.size(), 1);
  const std::vector<std::int64_t> unfound(local->found.size(), 0);
  if (auto ok = count(m, local->found, unfound, found); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  const std::int64_t needed = recent ? kRecoveryTracked : options_.min_tracked;
  if (inliers(local->tracked) < needed) {
    return std::nullopt;
  }
  return std::move(local->tracked);
}

Result<std::pair<std::optional<Tracked>, bool>> Pipeline::track_frame(const Frame& frame) {
  if (!map_) {
    return fail(ErrorKind::kInvalidInput, "track: no map to track against");
  }
  Map& m = *map_;
  auto placed = place(m, frame);
  if (!placed) {
    return std::unexpected(std::move(placed.error()));
  }
  auto& [where, recovered] = *placed;
  if (recovered && where) {
    since_recovery_ = 0;
  } else if (since_recovery_) {
    ++*since_recovery_;
  }
  const bool recent = since_recovery_ && *since_recovery_ < options_.max_frames;
  const bool wide = since_recovery_ && *since_recovery_ < kRecoveryWideFrames;
  std::optional<Tracked> tracked;
  if (where) {
    auto refined = refine(m, frame, *where, wide, recent);
    if (!refined) {
      return std::unexpected(std::move(refined.error()));
    }
    tracked = std::move(*refined);
  }
  if (!tracked) {
    record_failure();
    return std::pair{std::optional<Tracked>{}, false};
  }
  const bool held = recent && std::cmp_greater(keyframe_count(m), options_.max_frames);
  auto need = held ? Result<bool>(false)
                   : need_keyframe(m, camera_, frame.depth, *tracked, mode_, options_);
  if (!need) {
    return std::unexpected(std::move(need.error()));
  }
  if (*need) {
    if (auto ok = process(m, camera_, frame, *tracked, mode_, options_); !ok) {
      return std::unexpected(std::move(ok.error()));
    }
  }
  if (auto ok = accept(frame, *tracked); !ok) {
    return std::unexpected(std::move(ok.error()));
  }
  return std::pair{std::move(tracked), *need};
}

}  // namespace slamlab::edge
