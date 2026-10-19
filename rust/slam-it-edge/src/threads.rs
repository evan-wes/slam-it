//! The threads' handshakes, what ORB-SLAM's tracking, mapping and loop closing say to each other
//! (`slam_it.edge.threads`, whose docstring states them; the C++ core's
//! `slam_it/edge/threads.hpp`).
//!
//! - [`Writer`]: one writer of the map at a time. Local mapping writes in turn
//!   ([`Writer::mapping`]); loop closing and global BA's results stop it first
//!   ([`Writer::stopped`], ORB-SLAM's `RequestStop`, waiting for it, then `Release`), which also
//!   aborts its local BA ([`Writer::abort`], `mbAbortBA`); tracking and loop closing read the map
//!   and its database with mapping held off ([`Writer::held`]), and loop closing stops mapping
//!   there once it finds a loop to correct.
//! - [`Queue`]: the items waiting between two threads, their count readable (`KeyframesInQueue`),
//!   and a wait for the work given to be done.
//! - [`Generation`]: the count each global BA started takes, a run whose count has moved on
//!   superseded (`mnFullBAIdx`); each generation's stop flag is set when the next starts.
//! - [`Worker`]: a thread running a step on each item of its queue, its failure kept for the
//!   thread that gave it work.
//!
//! A poisoned lock (a thread that panicked holding it) is taken as it is: every value guarded
//! here is whole between statements.

use std::any::Any;
use std::collections::VecDeque;
use std::panic::{self, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle, ThreadId};

use slam_it_foundations::error::{Error, ErrorKind, Result};
use slam_it_foundations::nls::Stop;

/// Takes a lock, poisoned or not: every value the edge rung's threads share is whole between
/// statements.
///
/// # Arguments
///
/// * `mutex` - The lock.
///
/// # Returns
///
/// Its guard.
pub(crate) fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Waits on a condition until a test holds, poisoned or not.
///
/// # Arguments
///
/// * `changed` - The condition.
/// * `guard` - The lock's guard.
/// * `until` - The test.
///
/// # Returns
///
/// The guard, the test holding.
fn wait<'a, T>(
    changed: &Condvar,
    guard: MutexGuard<'a, T>,
    mut until: impl FnMut(&mut T) -> bool,
) -> MutexGuard<'a, T> {
    changed
        .wait_while(guard, |state| !until(state))
        .unwrap_or_else(PoisonError::into_inner)
}

/// The error a panic in one of the edge rung's threads stands for, as the thread keeps it for
/// whoever gave it work: the shared kinds have none for a bug, so it is `InvalidInput`, an
/// invariant the step relied on broken, its message naming the thread and the panic's.
///
/// # Arguments
///
/// * `thread` - The thread, by what it does.
/// * `payload` - The panic's payload.
///
/// # Returns
///
/// The error.
#[must_use]
pub fn panicked(thread: &str, payload: &(dyn Any + Send)) -> Error {
    let message = payload
        .downcast_ref::<&str>()
        .map(|text| (*text).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "a panic with no message".to_owned());
    Error::new(
        ErrorKind::InvalidInput,
        format!("{thread}: panicked: {message}"),
    )
}

/// Who holds the map, and the stops requested ([`Writer`]'s state).
#[derive(Debug, Default)]
struct Hand {
    /// The thread holding the map, if one does.
    holder: Option<ThreadId>,
    /// Stops requested and not yet released.
    stops: usize,
}

/// One writer of the map at a time, and the stop handshake (`slam_it.edge.threads.Writer`).
#[derive(Debug, Default)]
pub struct Writer {
    /// Who holds the map.
    hand: Mutex<Hand>,
    /// Signalled whenever the map is given up or a stop released.
    changed: Condvar,
    /// The flag that ends local mapping's local BA early.
    abort: Arc<AtomicBool>,
}

/// The map held by a thread while it lives ([`Writer::mapping`], [`Writer::held`]).
#[derive(Debug)]
pub struct Turn<'a> {
    /// The writer.
    writer: &'a Writer,
}

impl Drop for Turn<'_> {
    fn drop(&mut self) {
        lock(&self.writer.hand).holder = None;
        self.writer.changed.notify_all();
    }
}

/// The map held with local mapping stopped while it lives ([`Writer::stopped`]).
#[derive(Debug)]
pub struct Stopped<'a> {
    /// The writer.
    writer: &'a Writer,
    /// Whether the thread held the map already, and keeps it after.
    holding: bool,
}

impl Drop for Stopped<'_> {
    fn drop(&mut self) {
        let mut hand = lock(&self.writer.hand);
        hand.stops -= 1;
        if !self.holding {
            hand.holder = None;
        }
        drop(hand);
        self.writer.changed.notify_all();
    }
}

impl Writer {
    /// Holds the map for local mapping: waits while another holds it or a stop is requested.
    ///
    /// # Returns
    ///
    /// The turn, the map given up when it drops.
    pub fn mapping(&self) -> Turn<'_> {
        self.take(true)
    }

    /// Holds the map to read it whole, local mapping held off but neither stopped nor aborted.
    ///
    /// # Returns
    ///
    /// The hold, the map given up when it drops.
    pub fn held(&self) -> Turn<'_> {
        self.take(false)
    }

    /// Holds the map with local mapping stopped: its local BA aborted, then released when the
    /// hold drops. A thread already holding the map ([`Writer::held`]) stops mapping where it
    /// is, without waiting for itself.
    ///
    /// # Returns
    ///
    /// The stop.
    pub fn stopped(&self) -> Stopped<'_> {
        let mut hand = lock(&self.hand);
        hand.stops += 1;
        self.abort.store(true, Ordering::SeqCst);
        let me = thread::current().id();
        let holding = hand.holder == Some(me);
        if !holding {
            hand = wait(&self.changed, hand, |h| h.holder.is_none());
            hand.holder = Some(me);
        }
        drop(hand);
        Stopped {
            writer: self,
            holding,
        }
    }

    /// Whether a stop is requested or held: no keyframe is asked for then.
    ///
    /// # Returns
    ///
    /// Whether local mapping is stopped or being stopped (`isStopped`, `stopRequested`).
    pub fn stopping(&self) -> bool {
        lock(&self.hand).stops > 0
    }

    /// The flag that ends local mapping's local BA early (`mbAbortBA`), as a solve's stop.
    ///
    /// # Returns
    ///
    /// The stop, the writer's flag.
    #[must_use]
    pub fn abort(&self) -> Stop {
        Stop::on(Arc::clone(&self.abort))
    }

    /// Ends local mapping's local BA early: every keyframe queued (`InsertKeyFrame`), and
    /// tracking interrupting it (`InterruptBA`); a stop requested does too.
    pub fn interrupt(&self) {
        self.abort.store(true, Ordering::SeqCst);
    }

    /// Lets local mapping's next local BA run: mapping clears the flag as it starts a keyframe.
    pub fn resume(&self) {
        self.abort.store(false, Ordering::SeqCst);
    }

    /// Waits until the map is free, then holds it for the calling thread.
    ///
    /// # Arguments
    ///
    /// * `stops_too` - Whether to wait also while a stop is requested (local mapping's turn).
    ///
    /// # Returns
    ///
    /// The turn.
    fn take(&self, stops_too: bool) -> Turn<'_> {
        let hand = lock(&self.hand);
        let mut hand = wait(&self.changed, hand, |h| {
            h.holder.is_none() && (!stops_too || h.stops == 0)
        });
        hand.holder = Some(thread::current().id());
        Turn { writer: self }
    }
}

/// A queue's items and the count not yet handled ([`Queue`]'s state).
#[derive(Debug)]
struct Waiting<T> {
    /// The items waiting.
    items: VecDeque<T>,
    /// The items queued and not yet handled, the one being handled included.
    unfinished: usize,
}

/// The items waiting between two threads, in order (`slam_it.edge.threads.Queue`).
#[derive(Debug)]
pub struct Queue<T> {
    /// The items and the count not yet handled.
    waiting: Mutex<Waiting<T>>,
    /// Signalled when an item is queued or handled.
    changed: Condvar,
}

impl<T> Default for Queue<T> {
    fn default() -> Self {
        Self {
            waiting: Mutex::new(Waiting {
                items: VecDeque::new(),
                unfinished: 0,
            }),
            changed: Condvar::new(),
        }
    }
}

impl<T> Queue<T> {
    /// Queues an item.
    ///
    /// # Arguments
    ///
    /// * `item` - The item.
    pub fn put(&self, item: T) {
        let mut waiting = lock(&self.waiting);
        waiting.items.push_back(item);
        waiting.unfinished += 1;
        drop(waiting);
        self.changed.notify_all();
    }

    /// Takes the next item, waiting for one; [`Queue::done`] says when it is handled.
    ///
    /// # Returns
    ///
    /// The item.
    pub fn get(&self) -> T {
        let mut waiting = lock(&self.waiting);
        loop {
            if let Some(item) = waiting.items.pop_front() {
                return item;
            }
            waiting = self
                .changed
                .wait(waiting)
                .unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// Says the item last taken is handled.
    pub fn done(&self) {
        lock(&self.waiting).unfinished -= 1;
        self.changed.notify_all();
    }

    /// The items waiting, not counting one being handled (`KeyframesInQueue`).
    ///
    /// # Returns
    ///
    /// The count.
    pub fn waiting(&self) -> usize {
        lock(&self.waiting).items.len()
    }

    /// The items queued and not yet handled, the one being handled included.
    ///
    /// # Returns
    ///
    /// The count.
    pub fn unfinished(&self) -> usize {
        lock(&self.waiting).unfinished
    }

    /// Waits until every item queued is handled.
    pub fn drain(&self) {
        let waiting = lock(&self.waiting);
        drop(wait(&self.changed, waiting, |w| w.unfinished == 0));
    }
}

/// The latest generation and its stop flag ([`Generation`]'s state).
#[derive(Debug, Default)]
struct Latest {
    /// The latest generation.
    value: u64,
    /// Its stop flag.
    stop: Arc<AtomicBool>,
}

/// The count each global BA started takes (`slam_it.edge.threads.Generation`): a run whose
/// count has moved on was superseded (ORB-SLAM's `mnFullBAIdx`). A solve stops on a flag, so
/// each generation also has one, set when the next starts ([`Generation::stop`]), where
/// Python's solve asks the count itself.
#[derive(Debug, Default)]
pub struct Generation {
    /// The latest generation and its flag.
    latest: Mutex<Latest>,
}

impl Generation {
    /// Starts a new generation, superseding any run of the last: its stop flag is set.
    ///
    /// # Returns
    ///
    /// The new generation.
    pub fn next(&self) -> u64 {
        let mut latest = lock(&self.latest);
        latest.stop.store(true, Ordering::SeqCst);
        latest.stop = Arc::new(AtomicBool::new(false));
        latest.value += 1;
        latest.value
    }

    /// Whether a run of `value` is still the latest.
    ///
    /// # Arguments
    ///
    /// * `value` - The run's generation.
    ///
    /// # Returns
    ///
    /// Whether no generation was started since.
    pub fn current(&self, value: u64) -> bool {
        lock(&self.latest).value == value
    }

    /// The latest generation's stop flag, set once a newer one starts: a solve of it stops at
    /// its next iteration (`mbStopGBA`).
    ///
    /// # Returns
    ///
    /// The stop.
    pub fn stop(&self) -> Stop {
        Stop::on(Arc::clone(&lock(&self.latest).stop))
    }
}

/// A worker's queue and its step's error, shared with its thread.
struct Shared<T> {
    /// The items; `None` ends the thread.
    queue: Queue<Option<T>>,
    /// The step's error, if it failed.
    error: Mutex<Option<Error>>,
}

/// A worker's queue, for another thread to give it items or count them, or its own step to
/// count what waits behind the item it handles.
pub struct Sender<T>(Arc<Shared<T>>);

impl<T> Clone for Sender<T> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<T> Sender<T> {
    /// Queues an item.
    ///
    /// # Arguments
    ///
    /// * `item` - The item.
    pub fn put(&self, item: T) {
        self.0.queue.put(Some(item));
    }

    /// The items waiting, not counting one being handled.
    ///
    /// # Returns
    ///
    /// The count.
    #[must_use]
    pub fn waiting(&self) -> usize {
        self.0.queue.waiting()
    }
}

/// A thread running a step on each item of its queue, in order (`slam_it.edge.threads.Worker`):
/// a step that fails, or panics, keeps its error and drops the items after it, its item done all
/// the same, so that draining never hangs.
pub struct Worker<T: Send + 'static> {
    /// The queue and the error.
    shared: Arc<Shared<T>>,
    /// The thread, until it is joined.
    thread: Option<JoinHandle<()>>,
}

impl<T: Send + 'static> Worker<T> {
    /// Starts the thread.
    ///
    /// # Arguments
    ///
    /// * `name` - The thread's name.
    /// * `step` - What it does with each item, given the worker's own queue.
    ///
    /// # Returns
    ///
    /// The worker.
    ///
    /// # Panics
    ///
    /// If the system cannot start a thread.
    pub fn new(
        name: &str,
        mut step: impl FnMut(T, &Sender<T>) -> Result<()> + Send + 'static,
    ) -> Self {
        let shared = Arc::new(Shared {
            queue: Queue::default(),
            error: Mutex::new(None),
        });
        let theirs = Arc::clone(&shared);
        let own = Sender(Arc::clone(&shared));
        let label = name.to_owned();
        let thread = thread::Builder::new()
            .name(name.to_owned())
            .spawn(move || {
                while let Some(item) = theirs.queue.get() {
                    if lock(&theirs.error).is_none() {
                        // A step that panics fails as one that errs (B3b's review panel, H6).
                        let failed =
                            match panic::catch_unwind(AssertUnwindSafe(|| step(item, &own))) {
                                Ok(done) => done.err(),
                                Err(payload) => Some(panicked(&label, payload.as_ref())),
                            };
                        if let Some(error) = failed {
                            *lock(&theirs.error) = Some(error);
                        }
                    }
                    theirs.queue.done();
                }
                theirs.queue.done();
            })
            .expect("the system starts a thread");
        Self {
            shared,
            thread: Some(thread),
        }
    }

    /// Queues an item.
    ///
    /// # Arguments
    ///
    /// * `item` - The item.
    pub fn put(&self, item: T) {
        self.shared.queue.put(Some(item));
    }

    /// A handle on its queue, for another thread to give it items.
    ///
    /// # Returns
    ///
    /// The handle.
    #[must_use]
    pub fn sender(&self) -> Sender<T> {
        Sender(Arc::clone(&self.shared))
    }

    /// The items waiting.
    ///
    /// # Returns
    ///
    /// The count.
    #[must_use]
    pub fn waiting(&self) -> usize {
        self.shared.queue.waiting()
    }

    /// The items queued and not yet handled, the one being handled included.
    ///
    /// # Returns
    ///
    /// The count.
    #[must_use]
    pub fn unfinished(&self) -> usize {
        self.shared.queue.unfinished()
    }

    /// Waits until every item queued is handled.
    pub fn drain(&self) {
        self.shared.queue.drain();
    }

    /// The error the step failed with, if it did.
    ///
    /// # Errors
    ///
    /// The step's error.
    pub fn check(&self) -> Result<()> {
        lock(&self.shared.error).clone().map_or(Ok(()), Err)
    }

    /// Ends the thread once the items queued are handled, and waits for it.
    pub fn stop(&mut self) {
        if let Some(thread) = self.thread.take() {
            self.shared.queue.put(None);
            // The step's panics are caught as its error (`check`); the loop itself cannot panic.
            let _ = thread.join();
        }
    }
}

impl<T: Send + 'static> Drop for Worker<T> {
    fn drop(&mut self) {
        self.stop();
    }
}
