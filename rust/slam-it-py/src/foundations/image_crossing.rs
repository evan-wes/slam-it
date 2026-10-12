//! Shared by every binding that takes or returns images: a C-contiguous (H, W) uint8 array
//! crossing as `image::Image`, and back as a new array; a list of them as a pyramid.

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use slam_it_foundations::image::Image;

use crate::common::contiguous;

/// A uint8 numpy image returned to Python.
pub type ImageOut<'py> = Bound<'py, PyArray2<u8>>;

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
pub fn image_in(image: &PyReadonlyArray2<'_, u8>) -> PyResult<Image> {
    let shape = image.shape();
    Ok(Image {
        rows: shape[0],
        cols: shape[1],
        pixels: contiguous(image)?.to_vec(),
    })
}

/// Copies a pyramid in: a list of (H, W) uint8 arrays.
///
/// # Arguments
///
/// * `levels` - The levels' arrays.
///
/// # Returns
///
/// The core's images, in order.
///
/// # Errors
///
/// `ValueError` if an array is not C-contiguous.
pub fn levels_in(levels: &[PyReadonlyArray2<'_, u8>]) -> PyResult<Vec<Image>> {
    levels.iter().map(image_in).collect()
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
pub fn image_array(py: Python<'_>, image: Image) -> PyResult<ImageOut<'_>> {
    let array = Array2::from_shape_vec((image.rows, image.cols), image.pixels)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}
