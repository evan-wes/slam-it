//! The error contract shared by every rung's Rust core.
//!
//! Fallible functions return [`Result<T>`]. The kinds mirror the C++ core's
//! `slamlab::foundations::ErrorKind` and the Python exceptions in
//! `slamlab.foundations.errors`; the bindings translate an [`Error`] into the Python exception
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
    /// An iterative method hit its iteration cap without meeting its convergence criteria.
    NotConverged,
}

/// An error from a slamlab core function: its kind and a message naming the cause.
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

/// The result type of every fallible slamlab core function.
pub type Result<T> = std::result::Result<T, Error>;

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
