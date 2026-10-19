/// @file
/// @brief The threads' handshakes, what ORB-SLAM's tracking, mapping and loop closing say to each
///        other: the C++ core's mirror of `slam_it.edge.threads`, whose docstring states them.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "slam_it/foundations/error.hpp"

namespace slam_it::edge::threads {

/// @brief One writer of the map at a time, and the stop handshake (`slam_it.edge.threads.Writer`).
///        Local mapping writes in turn (`Mapping`); loop closing and a global BA's results stop
///        it first (`Stopped`, ORB-SLAM's `RequestStop`, waiting for it, then `Release`),
///        setting the abort flag to end its local BA early (`mbAbortBA`); a hold (`Held`) keeps
///        mapping off without aborting it, and a stop inside a hold does not wait for itself.
class Writer {
 public:
  /// @brief Holds the map for local mapping while it lives: waits while another holds it or a
  ///        stop is requested.
  class Mapping {
   public:
    /// @brief Take the map for local mapping.
    /// @param writer The writer.
    explicit Mapping(Writer& writer);
    /// @brief Give it up.
    ~Mapping();
    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;
    Mapping(Mapping&&) = delete;
    Mapping& operator=(Mapping&&) = delete;

   private:
    /// @brief The writer.
    Writer& writer_;
  };

  /// @brief Holds the map whole while it lives, local mapping held off but not stopped.
  class Held {
   public:
    /// @brief Take the map.
    /// @param writer The writer.
    explicit Held(Writer& writer);
    /// @brief Give it up.
    ~Held();
    Held(const Held&) = delete;
    Held& operator=(const Held&) = delete;
    Held(Held&&) = delete;
    Held& operator=(Held&&) = delete;

   private:
    /// @brief The writer.
    Writer& writer_;
  };

  /// @brief Holds the map with local mapping stopped while it lives: its local BA aborted, then
  ///        released; a thread already holding the map stops mapping where it is.
  class Stopped {
   public:
    /// @brief Request the stop and take the map once mapping lets it go.
    /// @param writer The writer.
    explicit Stopped(Writer& writer);
    /// @brief Release mapping.
    ~Stopped();
    Stopped(const Stopped&) = delete;
    Stopped& operator=(const Stopped&) = delete;
    Stopped(Stopped&&) = delete;
    Stopped& operator=(Stopped&&) = delete;

   private:
    /// @brief The writer.
    Writer& writer_;
    /// @brief Whether this thread held the map already.
    bool holding_{false};
  };

  /// @brief Whether a stop is requested or held: no keyframe is asked for then.
  /// @return Whether local mapping is stopped or being stopped.
  [[nodiscard]] bool stopping() const;

  /// @brief The flag that ends local mapping's local BA early (`mbAbortBA`), which the BA reads
  ///        between its iterations.
  /// @return The flag.
  [[nodiscard]] const std::atomic<bool>& abort() const { return abort_; }

  /// @brief End local mapping's local BA early: every keyframe queued (`InsertKeyFrame`), and
  ///        tracking interrupting it (`InterruptBA`); a stop requested does too.
  void interrupt() { abort_.store(true); }

  /// @brief Let local mapping's next local BA run: mapping clears the flag as it starts a
  ///        keyframe.
  void resume() { abort_.store(false); }

 private:
  /// @brief Request a stop, and take the map unless the calling thread holds it already.
  /// @return Whether it held the map already.
  bool request_stop();

  /// @brief Wait until the map is free, then hold it for the calling thread.
  /// @param lock The writer's lock, held.
  /// @param stops_too Whether to wait also while a stop is requested (local mapping's turn).
  void take(std::unique_lock<std::mutex>& lock, bool stops_too);

  /// @brief Give the map up, waking whoever waits for it.
  void release();

  /// @brief Guards the fields below.
  mutable std::mutex mutex_;
  /// @brief Signalled whenever the map is given up or a stop released.
  std::condition_variable changed_;
  /// @brief Whether a thread holds the map.
  bool writing_{false};
  /// @brief The thread holding it.
  std::thread::id holder_;
  /// @brief Stops requested and not yet released.
  std::int64_t stops_{0};
  /// @brief The flag that ends local mapping's local BA early.
  std::atomic<bool> abort_{false};
};

/// @brief The items waiting between two threads, in order (`slam_it.edge.threads.Queue`).
/// @tparam T The item.
template <typename T>
class Queue {
 public:
  /// @brief Queue an item.
  /// @param item The item.
  void put(T item) {
    {
      const std::scoped_lock lock(mutex_);
      items_.push_back(std::move(item));
      ++unfinished_;
    }
    changed_.notify_all();
  }

  /// @brief Take the next item, waiting for one; `done` says when it is handled.
  /// @return The item.
  [[nodiscard]] T get() {
    std::unique_lock lock(mutex_);
    changed_.wait(lock, [this] { return !items_.empty(); });
    T item = std::move(items_.front());
    items_.pop_front();
    return item;
  }

  /// @brief Say the item last taken is handled.
  void done() {
    {
      const std::scoped_lock lock(mutex_);
      --unfinished_;
    }
    changed_.notify_all();
  }

  /// @brief The items waiting, not counting one being handled (`KeyframesInQueue`).
  /// @return The count.
  [[nodiscard]] std::size_t waiting() const {
    const std::scoped_lock lock(mutex_);
    return items_.size();
  }

  /// @brief The items queued and not yet handled, the one being handled included.
  /// @return The count.
  [[nodiscard]] std::size_t unfinished() const {
    const std::scoped_lock lock(mutex_);
    return unfinished_;
  }

  /// @brief Wait until every item queued is handled.
  void drain() {
    std::unique_lock lock(mutex_);
    changed_.wait(lock, [this] { return unfinished_ == 0; });
  }

 private:
  /// @brief Guards the fields below.
  mutable std::mutex mutex_;
  /// @brief Signalled when an item is queued or handled.
  std::condition_variable changed_;
  /// @brief The items waiting.
  std::deque<T> items_;
  /// @brief The items queued and not yet handled.
  std::size_t unfinished_{0};
};

/// @brief The count each global BA started takes (`slam_it.edge.threads.Generation`): a run
///        whose count has moved on was superseded (ORB-SLAM's `mnFullBAIdx`). A solve stops on a
///        flag, so each generation also has one, set when the next starts (`stop`), where
///        Python's solve asks the count itself.
class Generation {
 public:
  /// @brief Start a new generation, superseding any run of the last: its stop flag is set.
  /// @return The new generation.
  std::int64_t next() {
    const std::scoped_lock lock(mutex_);
    stop_->store(true);
    stop_ = std::make_shared<std::atomic<bool>>(false);
    return ++value_;
  }

  /// @brief Whether a run of `value` is still the latest.
  /// @param value The run's generation.
  /// @return Whether no generation was started since.
  [[nodiscard]] bool current(std::int64_t value) const {
    const std::scoped_lock lock(mutex_);
    return value_ == value;
  }

  /// @brief The latest generation's stop flag, set once a newer one starts: a solve of it stops
  ///        at its next iteration (`mbStopGBA`).
  /// @return The flag, kept alive while the run holds it.
  [[nodiscard]] std::shared_ptr<const std::atomic<bool>> stop() const {
    const std::scoped_lock lock(mutex_);
    return stop_;
  }

 private:
  /// @brief Guards the fields below.
  mutable std::mutex mutex_;
  /// @brief The latest generation.
  std::int64_t value_{0};
  /// @brief Its stop flag.
  std::shared_ptr<std::atomic<bool>> stop_{std::make_shared<std::atomic<bool>>(false)};
};

/// @brief A thread running a step on each item of its queue, in order
///        (`slam_it.edge.threads.Worker`): an empty item ends it; a step that fails keeps its
///        error and drops the items after it, so that draining never hangs.
/// @tparam T The item.
template <typename T>
class Worker {
 public:
  /// @brief The step: what the thread does with each item.
  using Step = std::function<foundations::Result<void>(T&)>;

  /// @brief Start the thread.
  /// @param step The step.
  explicit Worker(Step step) : step_(std::move(step)), thread_([this] { run(); }) {}

  /// @brief Stop the thread, after the items queued.
  ~Worker() { stop(); }

  Worker(const Worker&) = delete;
  Worker& operator=(const Worker&) = delete;
  Worker(Worker&&) = delete;
  Worker& operator=(Worker&&) = delete;

  /// @brief Queue an item.
  /// @param item The item.
  void put(T item) { queue_.put(std::optional<T>(std::move(item))); }

  /// @brief The items waiting.
  /// @return The count.
  [[nodiscard]] std::size_t waiting() const { return queue_.waiting(); }

  /// @brief The items queued and not yet handled, the one being handled included.
  /// @return The count.
  [[nodiscard]] std::size_t unfinished() const { return queue_.unfinished(); }

  /// @brief Wait until every item queued is handled.
  void drain() { queue_.drain(); }

  /// @brief The error the step failed with, if it did.
  /// @return Nothing, or the error.
  [[nodiscard]] foundations::Result<void> check() const {
    const std::scoped_lock lock(error_mutex_);
    if (error_) {
      return std::unexpected(*error_);
    }
    return {};
  }

  /// @brief End the thread once the items queued are handled, and wait for it.
  void stop() {
    if (thread_.joinable()) {
      queue_.put(std::nullopt);
      thread_.join();
    }
  }

 private:
  /// @brief Run the step on each item until the end.
  void run() {
    while (true) {
      std::optional<T> item = queue_.get();
      if (!item) {
        queue_.done();
        return;
      }
      bool failed = false;
      {
        const std::scoped_lock lock(error_mutex_);
        failed = error_.has_value();
      }
      if (!failed) {
        if (auto ok = step_(*item); !ok) {
          const std::scoped_lock lock(error_mutex_);
          error_ = std::move(ok.error());
        }
      }
      queue_.done();
    }
  }

  /// @brief The step.
  Step step_;
  /// @brief Its queue; an empty item ends the thread.
  Queue<std::optional<T>> queue_;
  /// @brief Guards `error_`.
  mutable std::mutex error_mutex_;
  /// @brief The step's error, if it failed.
  std::optional<foundations::Error> error_;
  /// @brief The thread, started last.
  std::thread thread_;
};

}  // namespace slam_it::edge::threads
