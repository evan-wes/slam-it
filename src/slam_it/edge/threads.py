"""The threads' handshakes: what ORB-SLAM's tracking, mapping and loop closing say to each other.

Milestone 2B's B3b runs the edge pipeline's steps in threads of their own, as ORB-SLAM does
(``docs/design/2026-10-14-online-slam.md``, section 12); :mod:`slam_it.edge.pipeline` composes
them from these parts:

- :class:`Writer`: one writer of the map at a time. Local mapping writes in turn
  (:meth:`Writer.mapping`); loop closing and global BA's results stop it first
  (:meth:`Writer.stopped`, ORB-SLAM's ``RequestStop``, waiting for ``isStopped``, then
  ``Release``), which also aborts its local BA (:attr:`Writer.abort`, ``mbAbortBA``); tracking
  and loop closing read the map and its database with mapping held off (:meth:`Writer.held`),
  aborting nothing, and loop closing stops mapping there once it finds a loop to correct.
- :class:`Queue`: the keyframes waiting between two threads, their count readable
  (``mlNewKeyFrames``, ``KeyframesInQueue``), and a wait for the work given to be done.
- :class:`Generation`: the count each global BA started takes; a run whose count has moved on
  was superseded, and its result is dropped (ORB-SLAM's ``mnFullBAIdx``).
- :class:`Worker`: a thread running a step on each item of its queue, its failure kept for the
  thread that gave it work.

Python runs the threads under its interpreter's lock, so they interleave rather than run at
once; the handshakes are the same as the compiled languages' will be.
"""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Generic, TypeVar

T = TypeVar("T")


class Writer:
    """One writer of the map at a time, and the stop handshake (see the module docstring).

    Attributes
    ----------
    abort : threading.Event
        Set to stop local mapping's local BA at its next check: by every keyframe queued
        (``InsertKeyFrame``), by tracking interrupting it (``InterruptBA``), and by a stop
        requested (``RequestStop``). Local mapping clears it as it starts a keyframe.
    """

    def __init__(self) -> None:
        """Start with no writer and no stop requested."""
        self._condition = threading.Condition()
        self._writing = False
        self._holder: int | None = None
        self._stops = 0
        self.abort = threading.Event()

    @contextmanager
    def mapping(self) -> Iterator[None]:
        """Hold the map for local mapping: wait while another holds it or a stop is requested.

        Yields
        ------
        None
            While local mapping holds the map.
        """
        with self._condition:
            self._condition.wait_for(lambda: not self._writing and self._stops == 0)
            self._take()
        try:
            yield
        finally:
            self._release()

    @contextmanager
    def stopped(self) -> Iterator[None]:
        """Hold the map with local mapping stopped: its local BA aborted, then released after.

        A stop requested keeps local mapping from starting another keyframe, and the one it
        is mapping ends early; the map is held once it has. A thread already holding the map
        (:meth:`held`) stops mapping where it is, without waiting for itself.

        Yields
        ------
        None
            While the map is held and local mapping stopped.
        """
        with self._condition:
            self._stops += 1
            self.abort.set()
            holding = self._holder == threading.get_ident()
            if not holding:
                self._condition.wait_for(lambda: not self._writing)
                self._take()
        try:
            yield
        finally:
            with self._condition:
                self._stops -= 1
                if not holding:
                    self._writing, self._holder = False, None
                self._condition.notify_all()

    @contextmanager
    def held(self) -> Iterator[None]:
        """Hold the map to read it whole, local mapping held off but not stopped or aborted.

        Yields
        ------
        None
            While the map is held.
        """
        with self._condition:
            self._condition.wait_for(lambda: not self._writing)
            self._take()
        try:
            yield
        finally:
            self._release()

    def stopping(self) -> bool:
        """Return whether a stop is requested or held: no keyframe is asked for then.

        Returns
        -------
        bool
            Whether local mapping is stopped or being stopped (``isStopped``,
            ``stopRequested``).
        """
        with self._condition:
            return self._stops > 0

    def _take(self) -> None:
        """Hold the map for the calling thread (the condition's lock held)."""
        self._writing, self._holder = True, threading.get_ident()

    def _release(self) -> None:
        """Give the map up, waking whoever waits for it."""
        with self._condition:
            self._writing, self._holder = False, None
            self._condition.notify_all()


class Queue(Generic[T]):
    """The items waiting between two threads, in order (see the module docstring)."""

    def __init__(self) -> None:
        """Start empty."""
        self._items: queue.Queue[T] = queue.Queue()

    def put(self, item: T) -> None:
        """Queue an item.

        Parameters
        ----------
        item : T
            The item.
        """
        self._items.put(item)

    def get(self) -> T:
        """Take the next item, waiting for one.

        Returns
        -------
        T
            The item; :meth:`done` says when it is handled.
        """
        return self._items.get()

    def done(self) -> None:
        """Say the item last taken is handled."""
        self._items.task_done()

    def waiting(self) -> int:
        """Return the items waiting, not counting one being handled (``KeyframesInQueue``).

        Returns
        -------
        int
            The count.
        """
        return self._items.qsize()

    def drain(self) -> None:
        """Wait until every item queued is handled."""
        self._items.join()


class Generation:
    """The count each global BA started takes (see the module docstring)."""

    def __init__(self) -> None:
        """Start at 0: no global BA yet."""
        self._lock = threading.Lock()
        self._value = 0

    def next(self) -> int:
        """Start a new generation, superseding any run of the last.

        Returns
        -------
        int
            The new generation.
        """
        with self._lock:
            self._value += 1
            return self._value

    def current(self, value: int) -> bool:
        """Return whether a run of `value` is still the latest.

        Parameters
        ----------
        value : int
            The run's generation.

        Returns
        -------
        bool
            Whether no generation was started since.
        """
        with self._lock:
            return value == self._value


class Worker(Generic[T]):
    """A thread running a step on each item of its queue (see the module docstring).

    The step runs on the items in order; an item of None ends the thread. A step that raises
    ends it too, the error kept for :meth:`check`, and every item queued after is dropped, so
    that whoever drains the queue is not left waiting.
    """

    def __init__(self, name: str, step: Callable[[T], None]) -> None:
        """Start the thread.

        Parameters
        ----------
        name : str
            The thread's name.
        step : callable
            What it does with each item.
        """
        self.queue: Queue[T | None] = Queue()
        self._step = step
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def check(self) -> None:
        """Raise the error the step failed with, if it did.

        Raises
        ------
        BaseException
            The step's error.
        """
        if self._error is not None:
            raise self._error

    def stop(self) -> None:
        """End the thread once the items queued are handled, and wait for it."""
        self.queue.put(None)
        self._thread.join()

    def _run(self) -> None:
        """Run the step on each item until the end or a failure."""
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                if self._error is None:
                    self._step(item)
            except BaseException as error:  # kept, and raised in the giver's thread
                self._error = error
            finally:
                self.queue.done()
