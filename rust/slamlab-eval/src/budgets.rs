//! Compute budgets: the Rust mirror of `slamlab.eval.budgets` where a native program needs it,
//! its own peak memory, which it records in its run (`crate::runs`).

use std::path::Path;

use crate::text;

/// Where Linux states the calling process's memory (`slamlab.eval.budgets.STATUS`).
pub const STATUS: &str = "/proc/self/status";

/// The calling process's largest resident set so far (`slamlab.eval.budgets.peak_memory`):
/// Linux's `VmHWM`, which starts again at `exec`, so it does not count the process that
/// spawned this one.
///
/// # Arguments
///
/// * `status` - The status file to read ([`STATUS`] for the calling process).
///
/// # Returns
///
/// The bytes, or `None` where the system does not say: no such file, or no `VmHWM` line in
/// kilobytes.
#[must_use]
pub fn peak_memory(status: &Path) -> Option<u64> {
    peak_in(&std::fs::read_to_string(status).ok()?)
}

/// The peak resident set a process's status text states, its `VmHWM` line.
///
/// # Arguments
///
/// * `status` - The text of a `/proc/<pid>/status` file.
///
/// # Returns
///
/// The bytes, or `None` if no `VmHWM` line states them in kilobytes.
fn peak_in(status: &str) -> Option<u64> {
    status.lines().find_map(|line| {
        let words: Vec<&str> = line.split_whitespace().collect();
        match words.as_slice() {
            ["VmHWM:", value, "kB"] => text::integer(value)
                .and_then(|kilobytes| u64::try_from(kilobytes).ok())
                .map(|kilobytes| kilobytes * 1024),
            _ => None,
        }
    })
}

#[cfg(test)]
mod tests {
    //! Native tests of reading a process's peak memory, as Python's
    //! `tests/eval/test_budgets.py` reads it.

    use super::*;

    #[test]
    fn reads_the_high_water_mark() {
        let status =
            "Name:\tslamlab_vo\nVmPeak:\t  912340 kB\nVmHWM:\t   47248 kB\nVmRSS:\t   41000 kB\n";
        assert_eq!(peak_in(status), Some(47248 * 1024));
        for text in ["Name:\tx\nVmRSS:\t 41000 kB\n", "VmHWM:\t 47248 MB\n", ""] {
            assert_eq!(peak_in(text), None, "{text:?}");
        }
        assert_eq!(peak_memory(Path::new("/no/such/status")), None);
        if cfg!(target_os = "linux") {
            assert!(peak_memory(Path::new(STATUS)).is_some_and(|bytes| bytes > 0));
        }
    }
}
