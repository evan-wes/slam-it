//! Reading the datasets' text files: lines, whitespace fields, and numbers parsed exactly,
//! shared by the readers.

use std::path::Path;

use slam_it_foundations::error::{ErrorKind, Result, fail};

/// Reads a text file's lines, a trailing `\r` dropped from each.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The lines.
///
/// # Errors
///
/// `InvalidInput` naming the file if it cannot be read.
pub fn read_lines(path: &Path) -> Result<Vec<String>> {
    match std::fs::read_to_string(path) {
        Ok(text) => Ok(text
            .lines()
            .map(|line| line.strip_suffix('\r').unwrap_or(line).to_owned())
            .collect()),
        Err(e) => fail(
            ErrorKind::InvalidInput,
            format!("{}: no such file, or not readable ({e})", path.display()),
        ),
    }
}

/// Parses a whole token as a number, as Python's `float` does.
///
/// # Arguments
///
/// * `token` - The token.
///
/// # Returns
///
/// The number, or `None` if the token is not exactly one.
#[must_use]
pub fn number(token: &str) -> Option<f64> {
    token.trim().parse().ok()
}

/// Parses a whole token as an integer.
///
/// # Arguments
///
/// * `token` - The token.
///
/// # Returns
///
/// The integer, or `None` if the token is not exactly one.
#[must_use]
pub fn integer(token: &str) -> Option<i64> {
    token.trim().parse().ok()
}

/// Whether a TUM-style line carries data: not blank, not a `#` comment.
///
/// # Arguments
///
/// * `line` - The line.
///
/// # Returns
///
/// Whether it does.
#[must_use]
pub fn is_data(line: &str) -> bool {
    let stripped = line.trim();
    !stripped.is_empty() && !stripped.starts_with('#')
}

/// The message prefix naming a file and a line.
///
/// # Arguments
///
/// * `path` - The file.
/// * `number` - The 1-based line number.
///
/// # Returns
///
/// `path:number: `.
#[must_use]
pub fn location(path: &Path, number: usize) -> String {
    format!("{}:{number}: ", path.display())
}
