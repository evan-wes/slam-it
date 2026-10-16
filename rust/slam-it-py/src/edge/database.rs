//! Bindings for `slam_it.edge.database` (`slam_it_rs.edge.database`).
//!
//! A database crosses as `Database.as_dict()`'s dict, from which the core builds its own index
//! (`edge_crossing::database_in`); a frame's word vector as its `ids` (int64) and `values`
//! (float64). Candidates return as a dict of `keyframes`, `scores`, `sharing` and `scored`.

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use slam_it_edge::database as core;
use slam_it_foundations::vocabulary::BowVector;

use super::edge_crossing::{database_in, map_in, options_in, vector_array};
use crate::common::{add_functions, contiguous, to_i64, unwrap};

/// Finds the keyframes a frame may be relocalised against.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `database` - The database, as a dict.
/// * `m` - The map, as a dict.
/// * `ids` - The frame's words, ascending.
/// * `values` - Their values.
/// * `options` - The settings, as a dict.
///
/// # Returns
///
/// `keyframes` (int64), `scores` (float64), `sharing` and `scored`.
///
/// # Errors
///
/// `ValueError` for a malformed argument; `InvalidInputError` from the core.
#[pyfunction]
#[pyo3(signature = (database, m, ids, values, options))]
fn candidates<'py>(
    py: Python<'py>,
    database: &Bound<'py, PyDict>,
    m: &Bound<'py, PyDict>,
    ids: PyReadonlyArray1<'py, i64>,
    values: PyReadonlyArray1<'py, f64>,
    options: &Bound<'py, PyDict>,
) -> PyResult<Bound<'py, PyDict>> {
    let frame = BowVector {
        ids: contiguous(&ids)?.to_vec(),
        values: contiguous(&values)?.to_vec(),
    };
    let found = unwrap(
        py,
        core::candidates(
            &database_in(py, database)?,
            &map_in(py, m)?,
            &frame,
            &options_in(options)?,
        ),
    )?;
    let out = PyDict::new(py);
    out.set_item("keyframes", vector_array(py, &found.keyframes))?;
    out.set_item("scores", vector_array(py, &found.scores))?;
    out.set_item("sharing", to_i64(found.sharing))?;
    out.set_item("scored", to_i64(found.scored))?;
    Ok(out)
}

/// Registers the module's functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn database(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; candidates);
    Ok(())
}
