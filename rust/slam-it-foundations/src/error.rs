//! The error contract shared by every rung's Rust core.
//!
//! Fallible functions return [`Result<T>`]. The kinds mirror the C++ core's
//! `slam_it::foundations::ErrorKind` and the Python exceptions in
//! `slam_it.foundations.errors`; the bindings translate an [`Error`] into the Python exception
//! of the same kind, so all three languages fail the same way.

use std::fmt;

/// What went wrong, independent of where.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorKind {
    /// The input violates the function's contract (shape, range, finiteness).
    InvalidInput,
    /// The input is valid but geometrically degenerate (pure rotation, collinear points, too
    /// few correspondences), so the quantity asked for is not determined.
    Degenerate,
    /// An iterative method failed to converge where that is an error: strict mode was
    /// requested and the convergence criteria were not met, or the iteration diverged with no
    /// usable estimate. Reaching the iteration cap is otherwise not an error: iterative solvers
    /// return their best estimate with a solve summary.
    NotConverged,
}

/// An error from a SLAM It! core function: its kind and a message naming the cause.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    /// What went wrong.
    pub kind: ErrorKind,
    /// A human-readable description naming the offending value or condition.
    pub message: String,
}

impl Error {
    /// Creates an error of the given kind.
    ///
    /// # Arguments
    ///
    /// * `kind` - What went wrong.
    /// * `message` - A description naming the offending value or condition.
    ///
    /// # Returns
    ///
    /// The error.
    #[must_use]
    pub fn new(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
        }
    }
}

impl fmt::Display for Error {
    /// Formats the error as `<kind>: <message>`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}: {}", self.kind, self.message)
    }
}

impl std::error::Error for Error {}

/// The result type of every fallible SLAM It! core function.
pub type Result<T> = std::result::Result<T, Error>;

/// Builds the failed result of a fallible function.
///
/// # Arguments
///
/// * `kind` - What went wrong.
/// * `message` - A description naming the offending value or condition.
///
/// # Returns
///
/// `Err` holding the error, as any `Result<T>`.
///
/// # Errors
///
/// Always: it is the error.
pub fn fail<T>(kind: ErrorKind, message: impl Into<String>) -> Result<T> {
    Err(Error::new(kind, message))
}

/// Builds the `InvalidInput` failure for a setting out of range, naming its value.
///
/// # Arguments
///
/// * `name` - The setting's name.
/// * `rule` - What it must satisfy, e.g. "must be at least 1".
/// * `value` - Its value.
///
/// # Returns
///
/// `"<name>: <rule>, got <value>"` as an `InvalidInput` error.
///
/// # Errors
///
/// Always: it is the error.
pub fn fail_setting<T>(name: &str, rule: &str, value: impl fmt::Display) -> Result<T> {
    fail(
        ErrorKind::InvalidInput,
        format!("{name}: {rule}, got {value}"),
    )
}

/// Checks that an integer setting lies in `[low, high]`.
///
/// # Arguments
///
/// * `name` - The setting's name.
/// * `value` - Its value.
/// * `low` - The smallest value allowed.
/// * `high` - The largest.
///
/// # Errors
///
/// `InvalidInput`: `"<name>: must be from <low> to <high>, got <value>"`, the message of
/// Python's `slam_it.foundations.arrays.as_count`.
pub fn check_range<T: PartialOrd + fmt::Display + Copy>(
    name: &str,
    value: T,
    low: T,
    high: T,
) -> Result<()> {
    if low <= value && value <= high {
        Ok(())
    } else {
        fail_setting(name, &format!("must be from {low} to {high}"), value)
    }
}

#[cfg(test)]
mod tests {
    //! Unit tests for the error type.

    use super::*;

    /// An error displays its kind and message.
    #[test]
    fn error_displays_kind_and_message() {
        let err = Error::new(ErrorKind::Degenerate, "pure rotation: zero baseline");
        assert_eq!(err.to_string(), "Degenerate: pure rotation: zero baseline");
        assert_eq!(err.kind, ErrorKind::Degenerate);
    }
}
