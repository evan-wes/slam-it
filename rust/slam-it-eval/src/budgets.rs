//! Compute budgets: the Rust mirror of `slam_it.eval.budgets` where a native program needs it:
//! what it records in its run of its own process (`crate::runs`), its peak memory, the CPUs it
//! may run on and its memory cap.

use std::collections::BTreeSet;
use std::path::Path;

use crate::text;

/// Where Linux states the calling process's memory and the CPUs it may run on
/// (`slam_it.eval.budgets.STATUS`).
pub const STATUS: &str = "/proc/self/status";

/// Where Linux names the calling process's control group (`slam_it.eval.budgets.CGROUP`).
pub const CGROUP: &str = "/proc/self/cgroup";

/// Where the control groups (version 2) are mounted (`slam_it.eval.budgets.CGROUP_ROOT`).
pub const CGROUP_ROOT: &str = "/sys/fs/cgroup";

/// The calling process's largest resident set so far (`slam_it.eval.budgets.peak_memory`):
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
    match status_words(status, "VmHWM:")?.as_slice() {
        [value, "kB"] => text::integer(value)
            .and_then(|kilobytes| u64::try_from(kilobytes).ok())
            .map(|kilobytes| kilobytes * 1024),
        _ => None,
    }
}

/// The words after a key on a status text's line, as `slam_it.eval.budgets` splits it.
///
/// # Arguments
///
/// * `status` - The text of a `/proc/<pid>/status` file.
/// * `key` - The line's first word, e.g. `VmHWM:`.
///
/// # Returns
///
/// The line's other words, or `None` if no line starts with the key.
fn status_words<'a>(status: &'a str, key: &str) -> Option<Vec<&'a str>> {
    status.lines().find_map(|line| {
        let mut words = line.split_whitespace();
        (words.next() == Some(key)).then(|| words.collect())
    })
}

/// The CPUs the calling process may run on (`slam_it.eval.budgets.allowed_cpus`): Linux's
/// `Cpus_allowed_list`, which `taskset` sets.
///
/// # Arguments
///
/// * `status` - The status file to read ([`STATUS`] for the calling process).
///
/// # Returns
///
/// The CPUs, ascending, or `None` where the system does not say: no such file, or no
/// `Cpus_allowed_list` line that reads as a CPU list.
#[must_use]
pub fn allowed_cpus(status: &Path) -> Option<Vec<usize>> {
    cpus_in(&std::fs::read_to_string(status).ok()?)
}

/// The CPUs a process's status text allows, its `Cpus_allowed_list` line.
///
/// # Arguments
///
/// * `status` - The text of a `/proc/<pid>/status` file.
///
/// # Returns
///
/// The CPUs, ascending, or `None` if no such line reads as a CPU list.
fn cpus_in(status: &str) -> Option<Vec<usize>> {
    let words = status_words(status, "Cpus_allowed_list:")?;
    let [line] = words.as_slice() else {
        return None;
    };
    let mut out = BTreeSet::new();
    for part in line.split(',') {
        let (first, last) = part.split_once('-').unwrap_or((part, part));
        let first = usize::try_from(text::integer(first)?).ok()?;
        let last = usize::try_from(text::integer(last)?).ok()?;
        if last < first {
            return None;
        }
        out.extend(first..=last);
    }
    Some(out.into_iter().collect())
}

/// The calling process's memory cap (`slam_it.eval.budgets.memory_cap`): its control group's
/// `memory.max`, which a budget's `systemd-run --scope -p MemoryMax` sets. A cap on an
/// enclosing group only is not reported.
///
/// # Arguments
///
/// * `cgroup` - The file naming the process's control group ([`CGROUP`]).
/// * `root` - Where the control groups are mounted ([`CGROUP_ROOT`]).
///
/// # Returns
///
/// The bytes, or `None` where there is no cap (`max`) or the system does not say: no such
/// file, no version-2 group (a `0::` line), or no `memory.max` in it.
#[must_use]
pub fn memory_cap(cgroup: &Path, root: &Path) -> Option<u64> {
    let groups = std::fs::read_to_string(cgroup).ok()?;
    let group = groups.lines().find_map(|line| line.strip_prefix("0::"))?;
    let path = root.join(group.trim_start_matches('/')).join("memory.max");
    let bytes = u64::try_from(text::integer(&std::fs::read_to_string(path).ok()?)?).ok()?;
    (bytes > 0).then_some(bytes)
}

#[cfg(test)]
mod tests {
    //! Native tests of what a program reads of its own process, as Python's
    //! `tests/eval/test_budgets.py` reads it.

    use super::*;

    #[test]
    fn reads_the_high_water_mark() {
        let status =
            "Name:\tslam_it_vo\nVmPeak:\t  912340 kB\nVmHWM:\t   47248 kB\nVmRSS:\t   41000 kB\n";
        assert_eq!(peak_in(status), Some(47248 * 1024));
        for text in ["Name:\tx\nVmRSS:\t 41000 kB\n", "VmHWM:\t 47248 MB\n", ""] {
            assert_eq!(peak_in(text), None, "{text:?}");
        }
        assert_eq!(peak_memory(Path::new("/no/such/status")), None);
        if cfg!(target_os = "linux") {
            assert!(peak_memory(Path::new(STATUS)).is_some_and(|bytes| bytes > 0));
        }
    }

    #[test]
    fn reads_the_cpus_allowed() {
        let status = "Name:\tslam_it_vo\nCpus_allowed:\tf0\nCpus_allowed_list:\t4-7\n";
        assert_eq!(cpus_in(status), Some(vec![4, 5, 6, 7]));
        assert_eq!(
            cpus_in("Cpus_allowed_list:\t0-3,8,10-11\n"),
            Some(vec![0, 1, 2, 3, 8, 10, 11])
        );
        for text in [
            "Name:\tx\n",
            "Cpus_allowed_list:\t7-4\n",
            "Cpus_allowed_list:\tf0\n",
            "Cpus_allowed_list:\t1-\n",
            "Cpus_allowed_list:\t0-3,7-4\n",
            "Cpus_allowed_list:\t1_0\n",
        ] {
            assert_eq!(cpus_in(text), None, "{text:?}");
        }
        if cfg!(target_os = "linux") {
            assert!(allowed_cpus(Path::new(STATUS)).is_some_and(|cpus| !cpus.is_empty()));
        }
    }

    #[test]
    fn reads_the_groups_memory_cap() {
        let root = std::env::temp_dir().join(format!(
            "slam_it_rust_test_memory_cap_{}",
            std::process::id()
        ));
        let group = root.join("app.slice").join("run-1.scope");
        std::fs::create_dir_all(&group).unwrap();
        let cgroup = root.join("cgroup");
        std::fs::write(&cgroup, "0::/app.slice/run-1.scope\n").unwrap();
        std::fs::write(group.join("memory.max"), "8589934592\n").unwrap();
        let cap = memory_cap(&cgroup, &root);
        std::fs::write(group.join("memory.max"), "max\n").unwrap();
        let uncapped = memory_cap(&cgroup, &root);
        std::fs::write(&cgroup, "1:name=systemd:/app.slice\n").unwrap();
        let unversioned = memory_cap(&cgroup, &root);
        let _ = std::fs::remove_dir_all(&root);
        assert_eq!(
            (cap, uncapped, unversioned),
            (Some(8_589_934_592), None, None)
        );
    }
}
