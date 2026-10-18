/// @file
/// @brief Native tests for the threads' handshakes (slam_it/edge/threads.hpp): one writer, the
///        stop, the queue, the generation, the worker. Each test drives two threads through a
///        handshake and holds each step's order with flags, so none depends on how the threads
///        happen to be scheduled; tests/edge/test_threads.py checks Python's alike.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../foundations/fixture_support.hpp"
#include "../foundations/test_support.hpp"
#include "slam_it/edge/threads.hpp"
#include "slam_it/foundations/error.hpp"

namespace {

namespace threads = slam_it::edge::threads;
using slam_it::testing::expect;

/// @brief The longest a test waits for another thread: a hang fails, not hangs the suite.
constexpr std::chrono::seconds kWait{10};

/// @brief How long a test watches for something that must not happen.
constexpr std::chrono::milliseconds kQuiet{200};

/// @brief Wait until a flag is set, or the time runs out.
/// @param flag The flag.
/// @param limit The longest to wait.
/// @return Whether it was set.
bool wait_for(const std::atomic<bool>& flag, std::chrono::milliseconds limit) {
  const auto until = std::chrono::steady_clock::now() + limit;
  while (!flag.load()) {
    if (std::chrono::steady_clock::now() > until) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

/// @brief A stop requested aborts the local BA, then holds the map once mapping lets it go.
/// @param failures The running failure count.
void check_stop_waits_and_aborts(int& failures) {
  threads::Writer writer;
  std::atomic<bool> mapping{false};
  std::atomic<bool> stopped{false};
  std::atomic<bool> aborted{false};
  std::atomic<bool> waited{false};
  std::atomic<bool> stopping{false};
  std::thread map_one([&] {
    const threads::Writer::Mapping turn(writer);
    mapping.store(true);
    aborted.store(wait_for(writer.abort(), kWait));  // the stop aborted the BA mapping was running
    waited.store(!stopped.load());                   // and waited for mapping to let the map go
  });
  std::thread stop([&] {
    if (!wait_for(mapping, kWait)) {
      return;
    }
    const threads::Writer::Stopped held(writer);
    stopping.store(writer.stopping());
    stopped.store(true);
  });
  map_one.join();
  stop.join();
  expect(aborted.load(), "a stop aborts mapping's local BA", failures);
  expect(waited.load(), "a stop waits for mapping to let the map go", failures);
  expect(stopping.load() && stopped.load(), "the map is held with mapping stopped", failures);
  expect(!writer.stopping(), "the stop is released", failures);
}

/// @brief Released only after the stop: mapping starts no keyframe while the map is stopped.
/// @param failures The running failure count.
void check_mapping_waits_for_a_stop(int& failures) {
  threads::Writer writer;
  std::atomic<bool> held{false};
  std::atomic<bool> released{false};
  std::atomic<bool> mapped{false};
  std::mutex guard;
  std::vector<std::string> order;
  std::thread stop([&] {
    const threads::Writer::Stopped stopped(writer);
    held.store(true);
    wait_for(released, kWait);
    const std::scoped_lock lock(guard);
    order.emplace_back("stop released");
  });
  std::thread map_one([&] {
    wait_for(held, kWait);
    const threads::Writer::Mapping turn(writer);
    const std::scoped_lock lock(guard);
    order.emplace_back("mapped");
    mapped.store(true);
  });
  expect(!wait_for(mapped, kQuiet), "mapping waits while the stop is held", failures);
  released.store(true);
  stop.join();
  map_one.join();
  expect(order == std::vector<std::string>{"stop released", "mapped"},
         "mapping goes on after the stop's release", failures);
}

/// @brief Tracking's hold waits for mapping and keeps it off, without the stop's abort or flag.
/// @param failures The running failure count.
void check_a_hold_aborts_nothing(int& failures) {
  threads::Writer writer;
  std::atomic<bool> started{false};
  std::thread map_one;
  {
    const threads::Writer::Held held(writer);
    expect(!writer.abort().load() && !writer.stopping(), "a hold neither aborts nor stops",
           failures);
    map_one = std::thread([&] {
      const threads::Writer::Mapping turn(writer);
      started.store(true);
    });
    expect(!wait_for(started, kQuiet), "a hold keeps mapping off", failures);
  }
  map_one.join();
  expect(started.load(), "mapping goes on after the hold", failures);
}

/// @brief Loop closing holds the map to search it, then stops mapping there to correct it.
/// @param failures The running failure count.
void check_a_stop_inside_a_hold(int& failures) {
  threads::Writer writer;
  {
    const threads::Writer::Held held(writer);
    {
      const threads::Writer::Stopped stopped(writer);
      expect(writer.stopping() && writer.abort().load(), "a stop inside a hold is held at once",
             failures);
    }
    expect(!writer.stopping(), "and released inside it", failures);
  }
  std::atomic<bool> started{false};
  std::thread map_one([&] {
    const threads::Writer::Mapping turn(writer);
    started.store(true);
  });
  map_one.join();
  expect(started.load(), "the hold, ended, lets mapping go on", failures);
}

/// @brief The count excludes the item being handled; drain waits until each is done.
/// @param failures The running failure count.
void check_the_queue(int& failures) {
  threads::Queue<int> items;
  items.put(1);
  items.put(2);
  expect(items.waiting() == 2, "two waiting", failures);
  expect(items.get() == 1, "the first first", failures);
  expect(items.waiting() == 1 && items.unfinished() == 2, "one handled now, one waiting", failures);
  items.done();
  expect(items.get() == 2, "then the second", failures);
  std::atomic<bool> drained{false};
  std::thread drain([&] {
    items.drain();
    drained.store(true);
  });
  expect(!wait_for(drained, kQuiet), "drain waits for the second to be done", failures);
  items.done();
  drain.join();
  expect(drained.load(), "and returns once it is", failures);
}

/// @brief A global BA's result applies only if no other started since.
/// @param failures The running failure count.
void check_the_generation(int& failures) {
  threads::Generation generation;
  const std::int64_t first = generation.next();
  expect(generation.current(first), "the latest is current", failures);
  const std::int64_t second = generation.next();
  expect(!generation.current(first) && generation.current(second),
         "a new generation supersedes the last", failures);
  const auto stop = generation.stop();
  expect(!stop->load(), "the latest's stop flag is clear", failures);
  generation.next();
  expect(stop->load() && !generation.stop()->load(), "and set once a newer one starts", failures);
}

/// @brief Each item handled once, in order; stop waits for the ones queued; a step that fails
///        keeps its error and drops the rest, so that draining does not hang.
/// @param failures The running failure count.
void check_the_worker(int& failures) {
  std::vector<int> seen;
  {
    threads::Worker<int> worker([&](const int& item) -> slam_it::foundations::Result<void> {
      seen.push_back(item);
      return {};
    });
    for (int item = 0; item < 5; ++item) {
      worker.put(item);
    }
    worker.stop();
    expect(seen == std::vector<int>{0, 1, 2, 3, 4}, "a worker runs its items in order", failures);
    expect(worker.check().has_value(), "and keeps no error", failures);
  }
  seen.clear();
  threads::Worker<int> failing([&](const int& item) -> slam_it::foundations::Result<void> {
    if (item == 1) {
      return slam_it::foundations::fail(slam_it::foundations::ErrorKind::kInvalidInput, "bad item");
    }
    seen.push_back(item);
    return {};
  });
  for (int item = 0; item < 4; ++item) {
    failing.put(item);
  }
  failing.drain();
  expect(seen == std::vector<int>{0}, "a failed step drops the items after it", failures);
  const auto kept = failing.check();
  expect(!kept && kept.error().message == "bad item", "and keeps its error", failures);
  failing.stop();
}

int run() {
  int failures = 0;
  check_stop_waits_and_aborts(failures);
  check_mapping_waits_for_a_stop(failures);
  check_a_hold_aborts_nothing(failures);
  check_a_stop_inside_a_hold(failures);
  check_the_queue(failures);
  check_the_generation(failures);
  check_the_worker(failures);
  return slam_it::testing::finish("edge/threads", failures);
}

}  // namespace

int main() { return slam_it::testing::guarded("edge/threads", run); }
