//! The threads' handshakes (`slam_it_edge::threads`): one writer, the stop, the queue, the
//! generation, the worker. Each test drives two threads through a handshake and holds each
//! step's order with flags, so none depends on how the threads happen to be scheduled;
//! `tests/edge/test_threads.py` checks Python's alike.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use slam_it_edge::threads::{Generation, Queue, Worker, Writer};
use slam_it_foundations::error::{ErrorKind, fail};

/// The longest a test waits for another thread: a hang fails, not hangs the suite.
const WAIT: Duration = Duration::from_secs(10);

/// How long a test watches for something that must not happen.
const QUIET: Duration = Duration::from_millis(200);

/// Waits until a test holds, or the time runs out.
///
/// # Arguments
///
/// * `limit` - The longest to wait.
/// * `test` - The test.
///
/// # Returns
///
/// Whether it held.
fn wait_for(limit: Duration, test: impl Fn() -> bool) -> bool {
    let until = Instant::now() + limit;
    while !test() {
        if Instant::now() > until {
            return false;
        }
        thread::sleep(Duration::from_millis(1));
    }
    true
}

#[test]
fn a_stop_waits_for_mapping_to_finish_and_aborts_its_ba() {
    let writer = Writer::default();
    let mapping = AtomicBool::new(false);
    let stopped = AtomicBool::new(false);
    thread::scope(|s| {
        let mapper = s.spawn(|| {
            let _turn = writer.mapping();
            mapping.store(true, Ordering::SeqCst);
            let abort = writer.abort();
            let aborted = wait_for(WAIT, || abort.is_set());
            (aborted, !stopped.load(Ordering::SeqCst))
        });
        s.spawn(|| {
            assert!(wait_for(WAIT, || mapping.load(Ordering::SeqCst)));
            let _stop = writer.stopped();
            assert!(writer.stopping());
            stopped.store(true, Ordering::SeqCst);
        });
        let (aborted, waited) = mapper.join().unwrap();
        assert!(aborted, "the stop aborted the BA mapping was running");
        assert!(waited, "and waited for mapping to let the map go");
    });
    assert!(stopped.load(Ordering::SeqCst));
    assert!(!writer.stopping(), "released");
}

#[test]
fn mapping_waits_while_a_stop_is_held() {
    let writer = Writer::default();
    let held = AtomicBool::new(false);
    let released = AtomicBool::new(false);
    let mapped = AtomicBool::new(false);
    let order = Mutex::new(Vec::new());
    thread::scope(|s| {
        s.spawn(|| {
            let _stop = writer.stopped();
            held.store(true, Ordering::SeqCst);
            assert!(wait_for(WAIT, || released.load(Ordering::SeqCst)));
            order.lock().unwrap().push("stop released");
        });
        s.spawn(|| {
            assert!(wait_for(WAIT, || held.load(Ordering::SeqCst)));
            let _turn = writer.mapping();
            order.lock().unwrap().push("mapped");
            mapped.store(true, Ordering::SeqCst);
        });
        assert!(
            !wait_for(QUIET, || mapped.load(Ordering::SeqCst)),
            "still stopped"
        );
        released.store(true, Ordering::SeqCst);
    });
    assert_eq!(*order.lock().unwrap(), ["stop released", "mapped"]);
}

#[test]
fn holding_the_map_keeps_mapping_off_but_aborts_nothing() {
    let writer = Writer::default();
    let started = AtomicBool::new(false);
    thread::scope(|s| {
        let hold = writer.held();
        assert!(!writer.abort().is_set());
        assert!(!writer.stopping());
        s.spawn(|| {
            let _turn = writer.mapping();
            started.store(true, Ordering::SeqCst);
        });
        assert!(
            !wait_for(QUIET, || started.load(Ordering::SeqCst)),
            "held off"
        );
        drop(hold);
    });
    assert!(started.load(Ordering::SeqCst));
}

#[test]
fn a_stop_inside_a_hold_does_not_wait_for_itself() {
    let writer = Writer::default();
    {
        let _hold = writer.held();
        {
            let _stop = writer.stopped();
            assert!(writer.stopping());
            assert!(writer.abort().is_set());
        }
        assert!(!writer.stopping());
    }
    let _turn = writer.mapping(); // the hold, ended, lets mapping go on
}

#[test]
fn the_queue_counts_what_waits_and_drains() {
    let items = Queue::default();
    items.put(1);
    items.put(2);
    assert_eq!(items.waiting(), 2);
    assert_eq!(items.get(), 1);
    assert_eq!((items.waiting(), items.unfinished()), (1, 2)); // one handled now, one waiting
    items.done();
    assert_eq!(items.get(), 2);
    let drained = AtomicBool::new(false);
    thread::scope(|s| {
        s.spawn(|| {
            items.drain();
            drained.store(true, Ordering::SeqCst);
        });
        assert!(
            !wait_for(QUIET, || drained.load(Ordering::SeqCst)),
            "the second is not done yet"
        );
        items.done();
    });
    assert!(drained.load(Ordering::SeqCst));
}

#[test]
fn a_new_generation_supersedes_the_last() {
    let generation = Generation::default();
    let first = generation.next();
    assert!(generation.current(first));
    let stop = generation.stop();
    assert!(!stop.is_set());
    let second = generation.next();
    assert!(!generation.current(first));
    assert!(generation.current(second));
    assert!(stop.is_set(), "the earlier's stop flag is set");
    assert!(!generation.stop().is_set());
}

#[test]
fn a_worker_runs_its_items_in_order_and_keeps_a_failure() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let theirs = Arc::clone(&seen);
    let mut worker = Worker::new("test", move |item: i32, _: &_| {
        theirs.lock().unwrap().push(item);
        Ok(())
    });
    for item in 0..5 {
        worker.put(item);
    }
    worker.stop();
    assert_eq!(*seen.lock().unwrap(), [0, 1, 2, 3, 4]);
    assert!(worker.check().is_ok());

    let seen = Arc::new(Mutex::new(Vec::new()));
    let theirs = Arc::clone(&seen);
    let failing = Worker::new("test", move |item: i32, _: &_| {
        if item == 1 {
            return fail(ErrorKind::InvalidInput, "bad item");
        }
        theirs.lock().unwrap().push(item);
        Ok(())
    });
    for item in 0..4 {
        failing.put(item);
    }
    failing.drain(); // does not hang on what follows the failure
    assert_eq!(*seen.lock().unwrap(), [0]);
    assert_eq!(failing.check().unwrap_err().message, "bad item");
}
