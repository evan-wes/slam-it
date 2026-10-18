/// @file
/// @brief Implementation of the writer declared in slam_it/edge/threads.hpp.
#include "slam_it/edge/threads.hpp"

#include <mutex>
#include <thread>

namespace slam_it::edge::threads {

Writer::Mapping::Mapping(Writer& writer) : writer_(writer) {
  std::unique_lock lock(writer_.mutex_);
  writer_.take(lock, true);
}

Writer::Mapping::~Mapping() { writer_.release(); }

Writer::Held::Held(Writer& writer) : writer_(writer) {
  std::unique_lock lock(writer_.mutex_);
  writer_.take(lock, false);
}

Writer::Held::~Held() { writer_.release(); }

Writer::Stopped::Stopped(Writer& writer) : writer_(writer), holding_(writer.request_stop()) {}

Writer::Stopped::~Stopped() {
  {
    const std::scoped_lock lock(writer_.mutex_);
    --writer_.stops_;
    if (!holding_) {
      writer_.writing_ = false;
      writer_.holder_ = std::thread::id();
    }
  }
  writer_.changed_.notify_all();
}

bool Writer::request_stop() {
  std::unique_lock lock(mutex_);
  ++stops_;
  abort_.store(true);
  const bool holding = writing_ && holder_ == std::this_thread::get_id();
  if (!holding) {
    take(lock, false);
  }
  return holding;
}

bool Writer::stopping() const {
  const std::scoped_lock lock(mutex_);
  return stops_ > 0;
}

void Writer::take(std::unique_lock<std::mutex>& lock, bool stops_too) {
  changed_.wait(lock, [this, stops_too] { return !writing_ && (!stops_too || stops_ == 0); });
  writing_ = true;
  holder_ = std::this_thread::get_id();
}

void Writer::release() {
  {
    const std::scoped_lock lock(mutex_);
    writing_ = false;
    holder_ = std::thread::id();
  }
  changed_.notify_all();
}

}  // namespace slam_it::edge::threads
