"""The threads' handshakes (slam_it.edge.threads): one writer, the stop, the queue, the generation.

Each test drives two threads through a handshake and holds each step's order with events, so
none depends on how the threads happen to be scheduled. A thread's failed assertion is kept and
raised where the test joins it (:class:`_Thread`): pytest does not fail a test on an exception in
another thread (B3b's review panel, H2).
"""

from __future__ import annotations

import threading
from collections.abc import Callable

import pytest

from slam_it.edge.threads import Generation, Queue, Worker, Writer

WAIT = 10.0
"""The longest a test waits for another thread, in seconds: a hang fails, not hangs the suite."""


class _Thread:
    """A daemon thread whose failure, a failed assertion's among them, :meth:`join` raises."""

    def __init__(self, target: Callable[[], None]) -> None:
        """Start a thread running `target`."""
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, args=(target,), daemon=True)
        self._thread.start()

    def _run(self, target: Callable[[], None]) -> None:
        """Run `target`, keeping what it fails with."""
        try:
            target()
        except BaseException as error:  # kept, and raised by join() in the test's thread
            self._error = error

    def join(self) -> None:
        """Wait for the thread, failing if it outlives :data:`WAIT`, and raise its failure."""
        self._thread.join(WAIT)
        assert not self._thread.is_alive(), "a thread outlived its wait"
        if self._error is not None:
            raise self._error


def test_a_stop_waits_for_mapping_to_finish_and_aborts_its_ba() -> None:
    """A stop requested aborts the local BA, then holds the map once mapping lets it go."""
    writer = Writer()
    mapping, asked, stopped = threading.Event(), threading.Event(), threading.Event()

    def map_one() -> None:
        with writer.mapping():
            mapping.set()
            assert writer.abort.wait(WAIT)  # the stop aborted the BA mapping was running
            assert not stopped.wait(0.2)  # and waits for mapping to let the map go

    def stop() -> None:
        assert mapping.wait(WAIT)
        asked.set()
        with writer.stopped():
            assert writer.stopping()
            stopped.set()

    threads = [_Thread(map_one), _Thread(stop)]
    for thread in threads:
        thread.join()
    assert stopped.is_set()
    assert not writer.stopping()  # released


def test_mapping_waits_while_a_stop_is_held() -> None:
    """Released only after the stop: mapping starts no keyframe while the map is stopped."""
    writer = Writer()
    held, released, mapped = threading.Event(), threading.Event(), threading.Event()
    order: list[str] = []

    def stop() -> None:
        with writer.stopped():
            held.set()
            assert released.wait(WAIT)
            order.append("stop released")

    def map_one() -> None:
        assert held.wait(WAIT)
        with writer.mapping():
            order.append("mapped")
            mapped.set()

    threads = [_Thread(stop), _Thread(map_one)]
    assert not mapped.wait(0.2)  # still stopped
    released.set()
    for thread in threads:
        thread.join()
    assert order == ["stop released", "mapped"]


def test_holding_the_map_keeps_mapping_off_but_aborts_nothing() -> None:
    """Tracking's hold waits for mapping and keeps it off, without the stop's abort or flag."""
    writer = Writer()
    with writer.held():
        assert not writer.abort.is_set()
        assert not writer.stopping()
        started = threading.Event()

        def map_one() -> None:
            with writer.mapping():
                started.set()

        thread = _Thread(map_one)
        assert not started.wait(0.2)  # held off
    thread.join()
    assert started.is_set()


def test_a_stop_inside_a_hold_does_not_wait_for_itself() -> None:
    """Loop closing holds the map to search it, then stops mapping there to correct it."""
    writer = Writer()
    with writer.held():
        with writer.stopped():
            assert writer.stopping()
            assert writer.abort.is_set()
        assert not writer.stopping()


def test_the_queue_counts_what_waits_and_drains() -> None:
    """The count excludes the item being handled; drain waits until each is done."""
    items: Queue[int] = Queue()
    items.put(1)
    items.put(2)
    assert items.waiting() == 2
    assert items.get() == 1
    assert items.waiting() == 1  # one handled now, one waiting
    items.done()
    assert items.get() == 2
    drained = threading.Event()

    def drain() -> None:
        items.drain()
        drained.set()

    thread = _Thread(drain)
    assert not drained.wait(0.2)  # the second is not done yet
    items.done()
    thread.join()
    assert drained.is_set()


def test_a_new_generation_supersedes_the_last() -> None:
    """A global BA's result applies only if no other started since."""
    generation = Generation()
    first = generation.next()
    assert generation.current(first)
    second = generation.next()
    assert not generation.current(first)
    assert generation.current(second)


def test_a_worker_runs_its_items_in_order() -> None:
    """Each item handled once, in order; stop waits for the ones queued."""
    seen: list[int] = []
    worker: Worker[int] = Worker("test", seen.append)
    for item in range(5):
        worker.queue.put(item)
    worker.stop()
    assert seen == [0, 1, 2, 3, 4]
    worker.check()


def test_a_failed_worker_keeps_its_error_and_drops_the_rest() -> None:
    """The error reaches the thread that gave the work; draining does not hang on what follows."""
    seen: list[int] = []

    def step(item: int) -> None:
        if item == 1:
            raise ValueError("bad item")
        seen.append(item)

    worker: Worker[int] = Worker("test", step)
    for item in range(4):
        worker.queue.put(item)
    worker.queue.drain()
    assert seen == [0]
    with pytest.raises(ValueError, match="bad item"):
        worker.check()
    worker.stop()
