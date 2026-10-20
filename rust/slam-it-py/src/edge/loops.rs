//! Bindings for `slam_it.edge.loops` (`slam_it_rs.edge.loops`).
//!
//! The map, its database (`edge_crossing::database_in`), the camera and the settings cross as
//! dicts; a similarity as its (4, 4) array; a loop being confirmed and a closed loop as the
//! dicts `slam_it.edge.loops` builds of them; the mode by name. Results cross back as dicts: a
//! detection's `pending` and `loop` (each a dict or None) and `summary`, its outcome a code in
//! `codes()["loop_outcome"]`; a correction's `map` and `summary`, each solve's termination a
//! code in `codes()["graphs_termination"]`. Merging's: the other maps as a list of
//! `Indexed.as_dict()`'s dicts; a merge being confirmed and a merge found as dicts of `map` with
//! `pending` or `loop`; a detection given to `merge_first` as Python's, its summary's outcome by
//! name; a merge detection back as a detection, with `maps`; a merge back as `map`,
//! `keyframe_offset`, `point_offset` and `summary`.

use std::collections::BTreeSet;

use nalgebra::Matrix4;
use numpy::{PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use slam_it_edge::database::MapRef;
use slam_it_edge::loops::{
    self as core, CandidateSummary, ConfirmationSummary, CorrectionSummary, Detection,
    DetectionSummary, Loop, Merge, MergeDetection, Merging, Outcome, Pending, Search, WeldSummary,
};
use slam_it_foundations::graphs::{self, Termination};
use slam_it_foundations::nls::Stop;

use super::edge_crossing::{
    camera_in, count_or_none, database_in, indexed_in, indices, indices_array, map_dict, map_in,
    mode_in, options_in, pose_in, vector, vector_array,
};
use crate::common::{
    add_functions, by_name, contiguous, encode, enum_codes, item_array, to_i64, unwrap,
};

/// Reads one item of a dict.
///
/// # Arguments
///
/// * `values` - The dict.
/// * `key` - The item's key.
///
/// # Returns
///
/// The item.
///
/// # Errors
///
/// `ValueError` for a missing key.
fn field<'py>(values: &Bound<'py, PyDict>, key: &str) -> PyResult<Bound<'py, PyAny>> {
    values
        .get_item(key)?
        .ok_or_else(|| PyValueError::new_err(format!("missing key {key:?}")))
}

/// Reads a similarity item: a (4, 4) float64 array.
///
/// # Arguments
///
/// * `value` - The item.
///
/// # Returns
///
/// The similarity.
///
/// # Errors
///
/// `ValueError` for another shape or dtype.
fn similarity_in(value: &Bound<'_, PyAny>) -> PyResult<Matrix4<f64>> {
    pose_in(&value.extract()?)
}

/// Reads a loop being confirmed from its dict (`slam_it.edge.loops._pending_as_dict`).
///
/// # Arguments
///
/// * `values` - The dict.
///
/// # Returns
///
/// The loop.
///
/// # Errors
///
/// `ValueError` for a missing key or a malformed item.
fn pending_in(values: &Bound<'_, PyDict>) -> PyResult<Pending> {
    Ok(Pending {
        matched: field(values, "matched")?.extract()?,
        last: field(values, "last")?.extract()?,
        similarity: similarity_in(&field(values, "similarity")?)?,
        confirmations: field(values, "confirmations")?.extract()?,
        misses: field(values, "misses")?.extract()?,
        points: vector(&field(values, "points")?)?,
        matched_points: vector(&field(values, "matched_points")?)?,
    })
}

/// Reads a closed loop from its dict (`slam_it.edge.loops._loop_as_dict`).
///
/// # Arguments
///
/// * `values` - The dict.
///
/// # Returns
///
/// The loop.
///
/// # Errors
///
/// `ValueError` for a missing key or a malformed item.
fn loop_in(values: &Bound<'_, PyDict>) -> PyResult<Loop> {
    Ok(Loop {
        current: field(values, "current")?.extract()?,
        matched: field(values, "matched")?.extract()?,
        similarity: similarity_in(&field(values, "similarity")?)?,
        points: vector(&field(values, "points")?)?,
        matched_points: vector(&field(values, "matched_points")?)?,
    })
}

/// A loop being confirmed's dict, or None.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `pending` - The loop, if any.
///
/// # Returns
///
/// The dict `pending_in` reads, or None.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn pending_dict<'py>(py: Python<'py>, pending: Option<&Pending>) -> PyResult<Bound<'py, PyAny>> {
    let Some(pending) = pending else {
        return Ok(py.None().into_bound(py));
    };
    let out = PyDict::new(py);
    out.set_item("matched", pending.matched)?;
    out.set_item("last", pending.last)?;
    out.set_item("similarity", item_array(py, &pending.similarity)?)?;
    out.set_item("confirmations", to_i64(pending.confirmations))?;
    out.set_item("misses", to_i64(pending.misses))?;
    out.set_item("points", vector_array(py, &pending.points))?;
    out.set_item("matched_points", vector_array(py, &pending.matched_points))?;
    Ok(out.into_any())
}

/// A closed loop's dict, or None.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `closed` - The loop, if any.
///
/// # Returns
///
/// The dict `loop_in` reads, or None.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn loop_dict<'py>(py: Python<'py>, closed: Option<&Loop>) -> PyResult<Bound<'py, PyAny>> {
    let Some(closed) = closed else {
        return Ok(py.None().into_bound(py));
    };
    let out = PyDict::new(py);
    out.set_item("current", closed.current)?;
    out.set_item("matched", closed.matched)?;
    out.set_item("similarity", item_array(py, &closed.similarity)?)?;
    out.set_item("points", vector_array(py, &closed.points))?;
    out.set_item("matched_points", vector_array(py, &closed.matched_points))?;
    Ok(out.into_any())
}

/// A loop detection's result as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `found` - The detection.
///
/// # Returns
///
/// `pending`, `loop` and `summary`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn detection_out<'py>(py: Python<'py>, found: &Detection) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("pending", pending_dict(py, found.pending.as_ref())?)?;
    out.set_item("loop", loop_dict(py, found.closed.as_ref())?)?;
    out.set_item("summary", detection_dict(py, &found.summary)?)?;
    Ok(out)
}

/// A dict item as a dict.
///
/// # Arguments
///
/// * `values` - The dict.
/// * `key` - The item's key.
///
/// # Returns
///
/// The item, or `None` where it is None.
///
/// # Errors
///
/// `ValueError` for a missing key; a Python error for an item that is not a dict.
fn dict_or_none<'py>(
    values: &Bound<'py, PyDict>,
    key: &str,
) -> PyResult<Option<Bound<'py, PyDict>>> {
    let item = field(values, key)?;
    if item.is_none() {
        return Ok(None);
    }
    Ok(Some(item.extract()?))
}

/// A detection's summary, read from Python's `DetectionSummary.as_dict`.
///
/// # Arguments
///
/// * `values` - Its fields, the outcome by name.
///
/// # Returns
///
/// The summary.
///
/// # Errors
///
/// `ValueError` for a missing key, an unknown outcome or a malformed item.
fn detection_summary_in(values: &Bound<'_, PyDict>) -> PyResult<DetectionSummary> {
    let outcome: String = field(values, "outcome")?.extract()?;
    let confirmation = dict_or_none(values, "confirmation")?
        .map(|c| -> PyResult<ConfirmationSummary> {
            Ok(ConfirmationSummary {
                matched: field(&c, "matched")?.extract()?,
                projected: field(&c, "projected")?.extract()?,
                refined: field(&c, "refined")?.extract()?,
                reprojected: field(&c, "reprojected")?.extract()?,
                confirmed: field(&c, "confirmed")?.extract()?,
            })
        })
        .transpose()?;
    let mut candidates = Vec::new();
    for item in field(values, "candidates")?.extract::<Vec<Bound<'_, PyDict>>>()? {
        candidates.push(CandidateSummary {
            keyframe: field(&item, "keyframe")?.extract()?,
            score: field(&item, "score")?.extract()?,
            near: field(&item, "near")?.extract()?,
            matches: field(&item, "matches")?.extract()?,
            best: field(&item, "best")?.extract()?,
            inliers: field(&item, "inliers")?.extract()?,
            projected: field(&item, "projected")?.extract()?,
            refined: field(&item, "refined")?.extract()?,
            reprojected: field(&item, "reprojected")?.extract()?,
            confirmations: field(&item, "confirmations")?.extract()?,
        });
    }
    Ok(DetectionSummary {
        outcome: by_name(&Outcome::ALL, Outcome::name, &outcome, "outcome")?,
        confirmation,
        sharing: field(values, "sharing")?.extract()?,
        scored: field(values, "scored")?.extract()?,
        candidates,
        confirmations: field(values, "confirmations")?.extract()?,
    })
}

/// A merge being confirmed, read from its dict.
///
/// # Arguments
///
/// * `values` - `map` and `pending`.
///
/// # Returns
///
/// The merge.
///
/// # Errors
///
/// `ValueError` for a missing key or a malformed item.
fn merging_in(values: &Bound<'_, PyDict>) -> PyResult<Merging> {
    Ok(Merging {
        map: field(values, "map")?.extract()?,
        pending: pending_in(&field(values, "pending")?.extract()?)?,
    })
}

/// A merge found, read from its dict.
///
/// # Arguments
///
/// * `values` - `map` and `loop`.
///
/// # Returns
///
/// The merge.
///
/// # Errors
///
/// `ValueError` for a missing key or a malformed item.
fn merge_in(values: &Bound<'_, PyDict>) -> PyResult<Merge> {
    Ok(Merge {
        map: field(values, "map")?.extract()?,
        found: loop_in(&field(values, "loop")?.extract()?)?,
    })
}

/// A merge detection's result as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `found` - The merge detection.
///
/// # Returns
///
/// `pending` and `merge` (each a dict of `map` with `pending` or `loop`, or None), `summary`
/// and `maps`.
///
/// # Errors
///
/// A Python error if the dict cannot be built.
fn merge_detection_out<'py>(
    py: Python<'py>,
    found: &MergeDetection,
) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    match &found.pending {
        Some(merging) => {
            let item = PyDict::new(py);
            item.set_item("map", merging.map)?;
            item.set_item("pending", pending_dict(py, Some(&merging.pending))?)?;
            out.set_item("pending", item)?;
        }
        None => out.set_item("pending", py.None())?,
    }
    match &found.merge {
        Some(merge) => {
            let item = PyDict::new(py);
            item.set_item("map", merge.map)?;
            item.set_item("loop", loop_dict(py, Some(&merge.found))?)?;
            out.set_item("merge", item)?;
        }
        None => out.set_item("merge", py.None())?,
    }
    out.set_item("summary", detection_dict(py, &found.summary)?)?;
    out.set_item("maps", vector_array(py, &found.maps))?;
    Ok(out)
}

/// A candidate's summary as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `c` - The summary.
///
/// # Returns
///
/// The dict of its fields.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn candidate_dict<'py>(py: Python<'py>, c: &CandidateSummary) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("keyframe", c.keyframe)?;
    out.set_item("score", c.score)?;
    out.set_item("near", c.near)?;
    out.set_item("matches", count_or_none(py, c.matches)?)?;
    out.set_item("best", c.best)?;
    out.set_item("inliers", count_or_none(py, c.inliers)?)?;
    out.set_item("projected", count_or_none(py, c.projected)?)?;
    out.set_item("refined", count_or_none(py, c.refined)?)?;
    out.set_item("reprojected", count_or_none(py, c.reprojected)?)?;
    out.set_item("confirmations", count_or_none(py, c.confirmations)?)?;
    Ok(out)
}

/// A confirmation's summary as it crosses, or None.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `c` - The summary, if any.
///
/// # Returns
///
/// The dict of its fields, or None.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn confirmation_dict<'py>(
    py: Python<'py>,
    c: Option<&ConfirmationSummary>,
) -> PyResult<Bound<'py, PyAny>> {
    let Some(c) = c else {
        return Ok(py.None().into_bound(py));
    };
    let out = PyDict::new(py);
    out.set_item("matched", c.matched)?;
    out.set_item("projected", to_i64(c.projected))?;
    out.set_item("refined", count_or_none(py, c.refined)?)?;
    out.set_item("reprojected", count_or_none(py, c.reprojected)?)?;
    out.set_item("confirmed", c.confirmed)?;
    Ok(out.into_any())
}

/// A detection's summary as it crosses, its outcome as a code.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `d` - The summary.
///
/// # Returns
///
/// The dict of its fields.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn detection_dict<'py>(py: Python<'py>, d: &DetectionSummary) -> PyResult<Bound<'py, PyDict>> {
    let tried = PyList::empty(py);
    for candidate in &d.candidates {
        tried.append(candidate_dict(py, candidate)?)?;
    }
    let out = PyDict::new(py);
    out.set_item("outcome", encode(&Outcome::ALL, d.outcome)?)?;
    out.set_item(
        "confirmation",
        confirmation_dict(py, d.confirmation.as_ref())?,
    )?;
    out.set_item("sharing", to_i64(d.sharing))?;
    out.set_item("scored", to_i64(d.scored))?;
    out.set_item("candidates", tried)?;
    out.set_item("confirmations", to_i64(d.confirmations))?;
    Ok(out)
}

/// A solve's summary as it crosses (`slam_it.foundations.graphs.summary_from_backend`).
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `summary` - The summary.
///
/// # Returns
///
/// Its errors, iterations and termination code.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn solve_dict<'py>(py: Python<'py>, summary: &graphs::Summary) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("initial_error", summary.initial_error)?;
    out.set_item("final_error", summary.final_error)?;
    out.set_item("iterations", to_i64(summary.iterations))?;
    out.set_item(
        "termination",
        encode(&Termination::ALL, summary.termination)?,
    )?;
    Ok(out)
}

/// A correction's summary as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `c` - The summary.
///
/// # Returns
///
/// The dict of its fields, each solve's as `solve_dict` gives it.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn correction_dict<'py>(py: Python<'py>, c: &CorrectionSummary) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("current", c.current)?;
    out.set_item("matched", c.matched)?;
    out.set_item("similarity", item_array(py, &c.similarity)?)?;
    out.set_item("corrected", to_i64(c.corrected))?;
    out.set_item("moved", to_i64(c.moved))?;
    out.set_item("replaced", to_i64(c.replaced))?;
    out.set_item("fused", to_i64(c.fused))?;
    out.set_item("links", to_i64(c.links))?;
    write_graph(py, &out, c.edges, &c.essential, c.global_ba.as_ref())?;
    Ok(out)
}

/// A solve's summary as it crosses, or None.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `summary` - The summary, if the solve ran.
///
/// # Returns
///
/// Its dict, or None.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn solve_or_none<'py>(
    py: Python<'py>,
    summary: Option<&graphs::Summary>,
) -> PyResult<Bound<'py, PyAny>> {
    match summary {
        Some(summary) => Ok(solve_dict(py, summary)?.into_any()),
        None => Ok(py.None().into_bound(py)),
    }
}

/// Writes a correction's graph and global BA into its summary's dict.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `out` - The summary's dict.
/// * `edges` - The essential graph's edges.
/// * `essential` - Its solve.
/// * `global_ba` - Global BA's, if it ran.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn write_graph(
    py: Python<'_>,
    out: &Bound<'_, PyDict>,
    edges: usize,
    essential: &graphs::Summary,
    global_ba: Option<&graphs::Summary>,
) -> PyResult<()> {
    out.set_item("edges", to_i64(edges))?;
    out.set_item("essential", solve_dict(py, essential)?)?;
    out.set_item("global_ba", solve_or_none(py, global_ba)?)
}

/// A merge's welding BA as it crosses.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `w` - The summary.
///
/// # Returns
///
/// The dict of its fields.
///
/// # Errors
///
/// A Python error if an item cannot be set.
fn weld_dict<'py>(py: Python<'py>, w: &WeldSummary) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("keyframes", to_i64(w.keyframes))?;
    out.set_item("held", to_i64(w.held))?;
    out.set_item("points", to_i64(w.points))?;
    out.set_item("observations", to_i64(w.observations))?;
    out.set_item("outliers", to_i64(w.outliers))?;
    out.set_item("forgotten", to_i64(w.forgotten))?;
    out.set_item("first", solve_dict(py, &w.first)?)?;
    out.set_item("second", solve_or_none(py, w.second.as_ref())?)?;
    Ok(out)
}

/// Matches points to a keyframe's keypoints through a similarity.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `k` - The keyframe's row.
/// * `similarity` - Its (4, 4) `S_cw`.
/// * `points` - The points' rows, int64.
/// * `window` - The window's half-width.
/// * `distance` - The largest Hamming distance.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// Each keypoint's matched point id, -1 for none.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, k, similarity, points, window, distance, options))]
#[allow(
    clippy::too_many_arguments,
    reason = "the map, its camera, the keyframe, the search, as the reference takes them"
)]
fn search_by_similarity<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    k: usize,
    similarity: PyReadonlyArray2<'py, f64>,
    points: &Bound<'py, PyAny>,
    window: f64,
    distance: i64,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let found = unwrap(
        py,
        core::search_by_similarity(
            &map_in(py, m)?,
            &camera_in(camera)?,
            k,
            &pose_in(&similarity)?,
            &indices(points)?,
            Search { window, distance },
            &options_in(options)?,
        ),
    )?;
    Ok(vector_array(py, &found))
}

/// Matches two keyframes' points by word.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `database` - Its database, as a dict.
/// * `k1`, `k2` - The keyframes' rows.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// For each keypoint of `k1`, the matched point id of `k2`, -1 for none.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, database, k1, k2, options))]
fn match_by_word<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    database: &Bound<'py, PyDict>,
    k1: usize,
    k2: usize,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let found = unwrap(
        py,
        core::match_by_word(
            &map_in(py, m)?,
            &database_in(py, database)?,
            k1,
            k2,
            &options_in(options)?,
        ),
    )?;
    Ok(vector_array(py, &found))
}

/// The points a matched keyframe's group sees.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `matched` - The matched keyframe's row.
/// * `linked` - The searching keyframe's links' rows, int64.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// The points' rows.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, matched, linked, options))]
fn group_points<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    matched: usize,
    linked: &Bound<'py, PyAny>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let linked: BTreeSet<usize> = indices(linked)?.into_iter().collect();
    let found = unwrap(
        py,
        core::group_points(&map_in(py, m)?, matched, &linked, &options_in(options)?),
    )?;
    Ok(indices_array(py, &found))
}

/// Refines the similarity between two keyframes against their matches.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `k1`, `k2` - The keyframes' rows.
/// * `similarity` - The starting (4, 4) `S_12`.
/// * `matched` - Each keypoint of `k1`'s matched point id, int64.
/// * `fixed_scale` - Refine a rigid motion.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `similarity` and `inliers`, or None.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, k1, k2, similarity, matched, fixed_scale, options))]
#[allow(
    clippy::too_many_arguments,
    reason = "the map, the two keyframes, the start, the matches, as the reference takes them"
)]
fn refine<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    k1: usize,
    k2: usize,
    similarity: PyReadonlyArray2<'py, f64>,
    matched: PyReadonlyArray1<'py, i64>,
    fixed_scale: bool,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyAny>> {
    let found = unwrap(
        py,
        core::refine(
            &map_in(py, m)?,
            &camera_in(camera)?,
            k1,
            k2,
            &pose_in(&similarity)?,
            contiguous(&matched)?,
            fixed_scale,
            &options_in(options)?,
        ),
    )?;
    let Some(found) = found else {
        return Ok(py.None().into_bound(py));
    };
    let out = PyDict::new(py);
    out.set_item("similarity", item_array(py, &found.similarity)?)?;
    out.set_item("inliers", to_i64(found.inliers))?;
    Ok(out.into_any())
}

/// Fuses points into a keyframe through a similarity.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `k` - The keyframe's row.
/// * `similarity` - Its (4, 4) `S_cw`.
/// * `point_ids` - The points' ids, int64.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `map` and `fused`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, k, similarity, point_ids, options))]
fn fuse_by_similarity<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    k: usize,
    similarity: PyReadonlyArray2<'py, f64>,
    point_ids: PyReadonlyArray1<'py, i64>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut fused_map = map_in(py, m)?;
    let fused = unwrap(
        py,
        core::fuse_by_similarity(
            &mut fused_map,
            &camera_in(camera)?,
            k,
            &pose_in(&similarity)?,
            contiguous(&point_ids)?,
            &options_in(options)?,
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &fused_map)?)?;
    out.set_item("fused", to_i64(fused))?;
    Ok(out)
}

/// Bundle-adjusts every keyframe and point.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `map` and `summary`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, options))]
fn global_bundle_adjust<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut adjusted = map_in(py, m)?;
    let (camera, options) = (camera_in(camera)?, options_in(options)?);
    let never = Stop::never(); // a compiled core's stop is its own threads'
    let summary = unwrap(
        py,
        core::global_bundle_adjust(&mut adjusted, &camera, &options, &never),
    )?;
    let out = PyDict::new(py);
    out.set_item("summary", solve_dict(py, &summary)?)?;
    out.set_item("map", map_dict(py, &adjusted)?)?;
    Ok(out)
}

/// Carries a global BA's result onto the map as it has become.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `now` - The map now, as a dict.
/// * `solved` - The copy the global BA solved, as a dict.
///
/// # Returns
///
/// `map`, and the keyframes and points carried each way.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (now, solved))]
fn propagate_global_ba<'py>(
    py: Python<'py>,
    now: &Bound<'py, PyDict>,
    solved: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let mut moved = map_in(py, now)?;
    let carried = unwrap(
        py,
        core::propagate_global_ba(&mut moved, &map_in(py, solved)?),
    )?;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &moved)?)?;
    out.set_item("adjusted", to_i64(carried.adjusted))?;
    out.set_item("propagated", to_i64(carried.propagated))?;
    out.set_item("points", to_i64(carried.points))?;
    out.set_item("carried", to_i64(carried.carried))?;
    Ok(out)
}

/// Looks for a loop at a newly mapped keyframe.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `database` - Its database, as a dict.
/// * `camera` - The camera, as a dict.
/// * `keyframe` - The keyframe's id.
/// * `pending` - The loop being confirmed, as a dict, or None.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `pending`, `loop` and `summary`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, database, camera, keyframe, pending, mode, options))]
#[allow(
    clippy::too_many_arguments,
    reason = "the map, its database and camera, the keyframe, the state, as the reference takes them"
)]
fn detect<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    database: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    keyframe: i64,
    pending: Option<&Bound<'py, PyDict>>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let carried = pending.map(pending_in).transpose()?;
    let found = unwrap(
        py,
        core::detect(
            &map_in(py, m)?,
            &database_in(py, database)?,
            &camera_in(camera)?,
            keyframe,
            carried.as_ref(),
            mode_in(mode)?,
            &options_in(options)?,
        ),
    )?;
    detection_out(py, &found)
}

/// Corrects a loop closed at a keyframe.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `loop` - The loop, as a dict.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `map` and `summary`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, camera, r#loop, mode, options))]
fn correct<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    r#loop: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let closed = loop_in(r#loop)?;
    let mut corrected = map_in(py, m)?;
    let summary = unwrap(
        py,
        core::correct(
            &mut corrected,
            &camera_in(camera)?,
            &closed,
            mode_in(mode)?,
            &options_in(options)?,
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &corrected)?)?;
    out.set_item("summary", correction_dict(py, &summary)?)?;
    Ok(out)
}

/// Looks for another map at a newly mapped keyframe.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `m` - The active map, as a dict.
/// * `database` - Its database, as a dict.
/// * `camera` - The camera, as a dict.
/// * `keyframe` - The keyframe's id.
/// * `others` - The other maps, each a dict of `number`, `m` and `database`.
/// * `pending` - The merge being confirmed, as a dict, or None.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `pending`, `merge`, `summary` and `maps`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (m, database, camera, keyframe, others, pending, mode, options))]
#[allow(
    clippy::too_many_arguments,
    reason = "the map, its database and camera, the keyframe, the others, the state, as Python's"
)]
fn detect_merge<'py>(
    py: Python<'py>,
    m: &Bound<'py, PyDict>,
    database: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    keyframe: i64,
    others: &Bound<'py, PyList>,
    pending: Option<&Bound<'py, PyDict>>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let carried = pending.map(merging_in).transpose()?;
    let others = indexed_in(py, others)?;
    let others: Vec<MapRef<'_>> = others.iter().map(MapRef::from).collect();
    let found = unwrap(
        py,
        core::detect_merge(
            &map_in(py, m)?,
            &database_in(py, database)?,
            &camera_in(camera)?,
            keyframe,
            &others,
            carried.as_ref(),
            mode_in(mode)?,
            &options_in(options)?,
        ),
    )?;
    merge_detection_out(py, &found)
}

/// Gives a merge found on a keyframe priority over a loop closed on it.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `loop` - Loop detection's result, as Python's crosses it.
/// * `merging` - Merge detection's, as Python's crosses it.
///
/// # Returns
///
/// The loop's detection: `pending`, `loop` and `summary`.
///
/// # Errors
///
/// `ValueError` for a missing key, an unknown outcome or a malformed item.
#[pyfunction]
#[pyo3(signature = (r#loop, merging))]
fn merge_first<'py>(
    py: Python<'py>,
    r#loop: &Bound<'py, PyDict>,
    merging: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let given = Detection {
        pending: dict_or_none(r#loop, "pending")?
            .as_ref()
            .map(pending_in)
            .transpose()?,
        closed: dict_or_none(r#loop, "loop")?
            .as_ref()
            .map(loop_in)
            .transpose()?,
        summary: detection_summary_in(&field(r#loop, "summary")?.extract()?)?,
    };
    let found = MergeDetection {
        pending: dict_or_none(merging, "pending")?
            .as_ref()
            .map(merging_in)
            .transpose()?,
        merge: dict_or_none(merging, "merge")?
            .as_ref()
            .map(merge_in)
            .transpose()?,
        summary: detection_summary_in(&field(merging, "summary")?.extract()?)?,
        maps: vector(&field(merging, "maps")?)?,
    };
    detection_out(py, &core::merge_first(given, &found))
}

/// Merges the active map into the one it matched.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `active` - The active map, as a dict.
/// * `matched` - The matched map, as a dict.
/// * `camera` - The camera, as a dict.
/// * `found` - The merge, as a dict of `map` and `loop`.
/// * `mode` - The mode's name.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `map`, `keyframe_offset`, `point_offset` and `summary`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (active, matched, camera, found, mode, options))]
fn merge<'py>(
    py: Python<'py>,
    active: &Bound<'py, PyDict>,
    matched: &Bound<'py, PyDict>,
    camera: &Bound<'py, PyDict>,
    found: &Bound<'py, PyDict>,
    mode: &str,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let merged = unwrap(
        py,
        core::merge(
            &map_in(py, active)?,
            &map_in(py, matched)?,
            &camera_in(camera)?,
            &merge_in(found)?,
            mode_in(mode)?,
            &options_in(options)?,
        ),
    )?;
    let s = &merged.summary;
    let summary = PyDict::new(py);
    summary.set_item("map", s.map)?;
    summary.set_item("current", s.current)?;
    summary.set_item("matched", s.matched)?;
    summary.set_item("similarity", item_array(py, &s.similarity)?)?;
    summary.set_item("window", to_i64(s.window))?;
    summary.set_item("matched_window", to_i64(s.matched_window))?;
    summary.set_item("fused", to_i64(s.fused))?;
    summary.set_item("weld", weld_dict(py, &s.weld)?)?;
    write_graph(py, &summary, s.edges, &s.essential, s.global_ba.as_ref())?;
    let out = PyDict::new(py);
    out.set_item("map", map_dict(py, &merged.map)?)?;
    out.set_item("keyframe_offset", merged.keyframe_offset)?;
    out.set_item("point_offset", merged.point_offset)?;
    out.set_item("summary", summary)?;
    Ok(out)
}

enum_codes!("loop_outcome" => Outcome, "graphs_termination" => Termination);

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn loops(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(
        m;
        codes,
        search_by_similarity,
        match_by_word,
        group_points,
        refine,
        fuse_by_similarity,
        global_bundle_adjust,
        propagate_global_ba,
        detect,
        correct,
        detect_merge,
        merge_first,
        merge
    );
    Ok(())
}
