//! Bindings for `slamlab.foundations.image` (`slamlab_rs.foundations.image`).
//!
//! Images cross as C-contiguous uint8 arrays: (H, W) gray, (H, W, 3) RGB; the coordinate map
//! as an (H, W, 2) float64 array.

use nalgebra::Vector2;
use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2, PyReadonlyArray3, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use slamlab_foundations::image::{self as core, Image};

use crate::common::{add_functions, contiguous, unwrap};

/// A uint8 numpy image returned to Python.
type ImageOut<'py> = Bound<'py, PyArray2<u8>>;

/// Copies an (H, W) uint8 array in.
///
/// # Arguments
///
/// * `image` - The array.
///
/// # Returns
///
/// The core's image.
///
/// # Errors
///
/// `ValueError` if the array is not C-contiguous.
fn image_in(image: &PyReadonlyArray2<'_, u8>) -> PyResult<Image> {
    let shape = image.shape();
    Ok(Image {
        rows: shape[0],
        cols: shape[1],
        pixels: contiguous(image)?.to_vec(),
    })
}

/// Moves an image out as a new (rows, cols) uint8 array.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `image` - The core's image.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the pixel count does not match the shape (a core bug).
fn image_array(py: Python<'_>, image: Image) -> PyResult<ImageOut<'_>> {
    let array = Array2::from_shape_vec((image.rows, image.cols), image.pixels)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}

/// Converts an (H, W, 3) RGB image to (H, W) gray.
///
/// # Arguments
///
/// * `rgb` - The image.
///
/// # Returns
///
/// A new (H, W) uint8 array.
///
/// # Errors
///
/// `ValueError` for a non-contiguous array; `InvalidInputError` from the core.
#[pyfunction]
fn to_gray<'py>(py: Python<'py>, rgb: PyReadonlyArray3<'py, u8>) -> PyResult<ImageOut<'py>> {
    let shape = rgb.shape();
    if shape[2] != 3 {
        return Err(PyValueError::new_err(format!(
            "expected shape (H, W, 3), got {shape:?}"
        )));
    }
    let image = unwrap(py, core::to_gray(shape[0], shape[1], contiguous(&rgb)?))?;
    image_array(py, image)
}

/// Filters with a separable integer kernel summing to a power of two.
///
/// # Arguments
///
/// * `image` - The image.
/// * `kernel` - The weights.
///
/// # Returns
///
/// A new (H, W) uint8 array.
///
/// # Errors
///
/// `ValueError` for a non-contiguous array; `InvalidInputError` from the core.
#[pyfunction]
fn blur<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
    kernel: Vec<u32>,
) -> PyResult<ImageOut<'py>> {
    let out = unwrap(py, core::blur(&image_in(&image)?, &kernel))?;
    image_array(py, out)
}

/// Resizes bilinearly to (rows, cols).
///
/// # Arguments
///
/// * `image` - The image.
/// * `rows` - The output's rows.
/// * `cols` - The output's columns.
///
/// # Returns
///
/// A new (rows, cols) uint8 array.
///
/// # Errors
///
/// `ValueError` for a non-contiguous array; `InvalidInputError` from the core.
#[pyfunction]
fn resize<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
    rows: usize,
    cols: usize,
) -> PyResult<ImageOut<'py>> {
    let out = unwrap(py, core::resize(&image_in(&image)?, rows, cols))?;
    image_array(py, out)
}

/// Returns the image and its successively downscaled levels.
///
/// # Arguments
///
/// * `image` - Level 0.
/// * `levels` - The number of levels.
/// * `scale_factor` - Each level's downscale.
///
/// # Returns
///
/// A list of new uint8 arrays, one per level.
///
/// # Errors
///
/// `ValueError` for a non-contiguous array; `InvalidInputError` from the core.
#[pyfunction]
fn pyramid<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
    levels: usize,
    scale_factor: f64,
) -> PyResult<Vec<ImageOut<'py>>> {
    unwrap(py, core::pyramid(&image_in(&image)?, levels, scale_factor))?
        .into_iter()
        .map(|level| image_array(py, level))
        .collect()
}

/// Samples the image at an (H', W', 2) map of (x, y) coordinates.
///
/// # Arguments
///
/// * `image` - The image.
/// * `coordinates` - The map.
///
/// # Returns
///
/// A new (H', W') uint8 array.
///
/// # Errors
///
/// `ValueError` for a malformed array; `InvalidInputError` from the core.
#[pyfunction]
fn remap<'py>(
    py: Python<'py>,
    image: PyReadonlyArray2<'py, u8>,
    coordinates: PyReadonlyArray3<'py, f64>,
) -> PyResult<ImageOut<'py>> {
    let shape = coordinates.shape();
    if shape[2] != 2 {
        return Err(PyValueError::new_err(format!(
            "expected shape (H, W, 2), got {shape:?}"
        )));
    }
    let (rows, cols) = (shape[0], shape[1]);
    let points: Vec<Vector2<f64>> = contiguous(&coordinates)?
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&[x, y]| Vector2::new(x, y))
        .collect();
    let out = unwrap(py, core::remap(&image_in(&image)?, &points, rows, cols))?;
    image_array(py, out)
}

/// Registers the `image` functions.
///
/// # Errors
///
/// Returns a Python error if a function cannot be added.
#[pymodule]
pub fn image(m: &Bound<'_, PyModule>) -> PyResult<()> {
    add_functions!(m; to_gray, blur, resize, pyramid, remap);
    Ok(())
}
