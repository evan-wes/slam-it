//! Bindings for `slam_it.edge.relocalization` (`slam_it_rs.edge.relocalization`).
//!
//! The map, its database (`edge_crossing::database_in`), the vocabulary
//! (`vocabulary_crossing::vocabulary_in`), the camera, the frame and the settings cross as
//! dicts; in every map, the maps as a list of `Indexed.as_dict()`'s dicts
//! (`edge_crossing::indexed_in`). The result is a dict of `tracked` (or None), `matched` and
//! `summary`: `words`, `sharing`, `scored`, `placed` and `candidates`, a list of dicts of each
//! candidate's summary, RANSAC's termination as its code in `codes()` (or None); in every map,
//! that dict as `relocalized`, with `map` and `maps`.

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use slam_it_edge::relocalization::{self as core, CandidateSummary, Relocalized};
use slam_it_foundations::ransac;

use super::edge_crossing::{
    camera_in, count_or_none, database_in, frame_in, indexed_in, map_in, maybe_tracked, options_in,
    termination_or_none, vector_array,
};
use crate::common::{add_functions, enum_codes, to_i64, unwrap};
use crate::foundations::vocabulary_crossing::vocabulary_in;

/// A candidate's summary as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `candidate` - The summary.
///
/// # Returns
///
/// The dict of its fields.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn candidate_dict<'py>(
    py: Python<'py>,
    candidate: &CandidateSummary,
) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("keyframe", candidate.keyframe)?;
    out.set_item("score", candidate.score)?;
    out.set_item("matches", to_i64(candidate.matches))?;
    out.set_item("iterations", count_or_none(py, candidate.iterations)?)?;
    out.set_item(
        "termination",
        termination_or_none(py, candidate.termination)?,
    )?;
    out.set_item("degenerate", candidate.degenerate)?;
    out.set_item("inliers", count_or_none(py, candidate.inliers)?)?;
    out.set_item("refined", count_or_none(py, candidate.refined)?)?;
    out.set_item("found", count_or_none(py, candidate.found)?)?;
    Ok(out)
}

/// Places a frame in the map through the keyframes that look like it.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `database` - Its database, as a dict.
/// * `vocabulary` - The vocabulary, as a dict of its fields.
/// * `camera` - The camera, as a dict.
/// * `frame` - The frame, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `tracked`, `matched` and `summary`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, database, vocabulary, camera, frame, options))]
fn relocalize<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    database: &Bound<'py, PyDict>,
    vocabulary: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = unwrap(
        py,
        core::relocalize(
            &map_in(py, m)?,
            &database_in(py, database)?,
            &vocabulary_in(py, vocabulary)?,
            &camera_in(camera)?,
            &frame_in(frame)?,
            &options_in(options)?,
        ),
    )?;
    relocalized_dict(py, found)
}

/// A relocalisation's result as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `found` - The result.
///
/// # Returns
///
/// `tracked`, `matched` and `summary`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn relocalized_dict(py: Python<'_>, found: Relocalized) -> PyResult<Bound<'_, PyDict>> {
    let tried = PyList::empty(py);
    for candidate in &found.summary.candidates {
        tried.append(candidate_dict(py, candidate)?)?;
    }
    let summary = PyDict::new(py);
    summary.set_item("words", to_i64(found.summary.words))?;
    summary.set_item("sharing", to_i64(found.summary.sharing))?;
    summary.set_item("scored", to_i64(found.summary.scored))?;
    summary.set_item("candidates", tried)?;
    summary.set_item("placed", found.summary.placed)?;
    let out = PyDict::new(py);
    out.set_item("tracked", maybe_tracked(py, found.tracked)?)?;
    out.set_item("summary", summary)?;
    out.set_item("matched", vector_array(py, &found.matched))?;
    Ok(out)
}

/// Places a frame in whichever map looks like it.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `searched` - The maps, each a dict of `number`, `m` and `database`.
/// * `vocabulary` - The vocabulary, as a dict of its fields.
/// * `camera` - The camera, as a dict.
/// * `frame` - The frame, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `map` (or None), `relocalized` (as `relocalize` returns it) and `maps`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (searched, vocabulary, camera, frame, options))]
fn relocalize_across<'py>(
    py: Python<'py>,
    searched: &Bound<'py, PyList>,
    vocabulary: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    frame: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let found = unwrap(
        py,
        core::relocalize_across(
            &indexed_in(py, searched)?,
            &vocabulary_in(py, vocabulary)?,
            &camera_in(camera)?,
            &frame_in(frame)?,
            &options_in(options)?,
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("map", found.map)?;
    out.set_item("relocalized", relocalized_dict(py, found.relocalized)?)?;
    out.set_item("maps", vector_array(py, &found.maps))?;
    Ok(out)
}

enum_codes!("ransac_termination" => ransac::Termination);

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn relocalization(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; codes, relocalize, relocalize_across);
    Ok(())
}
