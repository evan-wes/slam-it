//! Bindings for `slam_it.foundations.vocabulary` (`slam_it_rs.foundations.vocabulary`).
//!
//! A vocabulary crosses as a dict of `Vocabulary`'s fields (`k`, `levels`, and its five arrays:
//! `descriptors` (N, 32) uint8, `parents` (N,) int32, `children` (N, 2) int32, `words` (N,)
//! int32, `weights` (W,) float64), copied in per call; descriptors as a C-contiguous (n, 32)
//! uint8 array, words as int64. `read` returns the dict the core read from a folder.

use std::path::PathBuf;

use numpy::{
    IntoPyArray, PyArray1, PyReadonlyArray1, PyReadonlyArray2,
    ndarray::{Array1, Array2},
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_foundations::orb::DESCRIPTOR_BYTES;
use slam_it_foundations::vocabulary as core;

use crate::common::{add_functions, contiguous, descriptors_in, int64_array, unwrap};

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
/// `ValueError` for a malformed field or tree.
fn vocabulary_in(py: Python<'_>, fields: &Bound<'_, PyDict>) -> PyResult<core::Vocabulary> {
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

/// Reads a vocabulary's folder.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `folder` - The folder.
///
/// # Errors
///
/// The core's error, as its Python exception.
#[pyfunction]
fn read(py: Python<'_>, folder: PathBuf) -> PyResult<Bound<'_, PyDict>> {
    let v = unwrap(py, core::read(&folder))?;
    let n = v.parents.len();
    let out = PyDict::new(py);
    out.set_item("k", v.k)?;
    out.set_item("levels", v.levels)?;
    let flat: Vec<u8> = v.descriptors.concat();
    let descriptors = Array2::from_shape_vec((n, DESCRIPTOR_BYTES), flat)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    out.set_item("descriptors", descriptors.into_pyarray(py))?;
    out.set_item("parents", Array1::from_vec(v.parents).into_pyarray(py))?;
    let children = Array2::from_shape_vec((n, 2), v.children.concat())
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    out.set_item("children", children.into_pyarray(py))?;
    out.set_item("words", Array1::from_vec(v.words).into_pyarray(py))?;
    out.set_item("weights", Array1::from_vec(v.weights).into_pyarray(py))?;
    Ok(out)
}

/// Quantises descriptors.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `vocabulary` - The vocabulary's dict.
/// * `descriptors` - The descriptors.
///
/// # Errors
///
/// `ValueError` for a malformed vocabulary or descriptors.
#[pyfunction]
fn quantize<'py>(
    py: Python<'py>,
    vocabulary: &Bound<'py, PyDict>,
    descriptors: PyReadonlyArray2<'py, u8>,
) -> PyResult<Bound<'py, PyDict>> {
    let v = vocabulary_in(py, vocabulary)?;
    let q = core::quantize(&v, &descriptors_in(&descriptors)?);
    let out = PyDict::new(py);
    out.set_item("words", int64_array(py, q.words))?;
    out.set_item("nodes", int64_array(py, q.nodes))?;
    Ok(out)
}

/// A frame's word vector.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `vocabulary` - The vocabulary's dict.
/// * `words` - The frame's words.
///
/// # Errors
///
/// The core's error, as its Python exception.
#[pyfunction]
fn bow<'py>(
    py: Python<'py>,
    vocabulary: &Bound<'py, PyDict>,
    words: PyReadonlyArray1<'py, i64>,
) -> PyResult<Bound<'py, PyDict>> {
    let v = vocabulary_in(py, vocabulary)?;
    let vector = unwrap(py, core::bow(&v, contiguous(&words)?))?;
    let out = PyDict::new(py);
    out.set_item("ids", int64_array(py, vector.ids))?;
    let values: Bound<'py, PyArray1<f64>> = Array1::from_vec(vector.values).into_pyarray(py);
    out.set_item("values", values)?;
    Ok(out)
}

/// Two word vectors' L1 score.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `ids1` - The first's words.
/// * `values1` - Its values.
/// * `ids2` - The second's words.
/// * `values2` - Its values.
///
/// # Errors
///
/// The core's error, as its Python exception.
#[pyfunction]
fn score(
    py: Python<'_>,
    ids1: PyReadonlyArray1<'_, i64>,
    values1: PyReadonlyArray1<'_, f64>,
    ids2: PyReadonlyArray1<'_, i64>,
    values2: PyReadonlyArray1<'_, f64>,
) -> PyResult<f64> {
    let first = core::BowVector {
        ids: contiguous(&ids1)?.to_vec(),
        values: contiguous(&values1)?.to_vec(),
    };
    let second = core::BowVector {
        ids: contiguous(&ids2)?.to_vec(),
        values: contiguous(&values2)?.to_vec(),
    };
    unwrap(py, core::score(&first, &second))
}

/// The `slam_it_rs.foundations.vocabulary` submodule.
///
/// # Errors
///
/// A registration failure.
#[pymodule]
pub fn vocabulary(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; read, quantize, bow, score);
    Ok(())
}
