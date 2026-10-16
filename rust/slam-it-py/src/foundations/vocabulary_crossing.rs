//! Shared by every binding that takes a vocabulary: it crosses as a dict of
//! `slam_it.foundations.vocabulary.Vocabulary`'s fields (`k`, `levels`, and its five arrays:
//! `descriptors` (N, 32) uint8, `parents` (N,) int32, `children` (N, 2) int32, `words` (N,)
//! int32, `weights` (W,) float64), exactly those keys, copied in and checked.

use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_foundations::vocabulary as core;

use crate::common::{contiguous, descriptors_in, unwrap};

/// `slam_it.foundations.vocabulary.fields`' keys.
const VOCABULARY_KEYS: [&str; 7] = [
    "k",
    "levels",
    "descriptors",
    "parents",
    "children",
    "words",
    "weights",
];

/// Reads one field of a vocabulary's dict.
///
/// # Arguments
///
/// * `fields` - The dict.
/// * `key` - The field.
///
/// # Errors
///
/// `ValueError` if it is missing.
fn field<'py>(fields: &Bound<'py, PyDict>, key: &str) -> PyResult<Bound<'py, PyAny>> {
    fields
        .get_item(key)?
        .ok_or_else(|| PyValueError::new_err(format!("vocabulary: missing {key}")))
}

/// Copies a vocabulary's dict in, checked.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `fields` - The dict of `Vocabulary`'s fields.
///
/// # Errors
///
/// `ValueError` for a dict without exactly [`VOCABULARY_KEYS`], or a malformed field or tree.
pub fn vocabulary_in(py: Python<'_>, fields: &Bound<'_, PyDict>) -> PyResult<core::Vocabulary> {
    if fields.len() != VOCABULARY_KEYS.len()
        || VOCABULARY_KEYS
            .iter()
            .any(|key| !fields.contains(key).unwrap_or(false))
    {
        return Err(PyValueError::new_err(format!(
            "vocabulary: expected exactly the keys {VOCABULARY_KEYS:?}"
        )));
    }
    let children: PyReadonlyArray2<'_, i32> = field(fields, "children")?.extract()?;
    let parents: PyReadonlyArray1<'_, i32> = field(fields, "parents")?.extract()?;
    let words: PyReadonlyArray1<'_, i32> = field(fields, "words")?.extract()?;
    let weights: PyReadonlyArray1<'_, f64> = field(fields, "weights")?.extract()?;
    let descriptors: PyReadonlyArray2<'_, u8> = field(fields, "descriptors")?.extract()?;
    let out = core::Vocabulary {
        k: field(fields, "k")?.extract()?,
        levels: field(fields, "levels")?.extract()?,
        descriptors: descriptors_in(&descriptors)?,
        parents: contiguous(&parents)?.to_vec(),
        children: contiguous(&children)?.as_chunks::<2>().0.to_vec(),
        words: contiguous(&words)?.to_vec(),
        weights: contiguous(&weights)?.to_vec(),
    };
    unwrap(py, core::check(&out))?;
    Ok(out)
}
