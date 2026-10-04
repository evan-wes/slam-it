//! Shared helpers for every binding: array conversion, batch mapping, error translation.
//!
//! Bindings take float64 C-contiguous numpy arrays with a leading batch dimension and return
//! new arrays (design doc, "Conventions"). Anything else (another dtype, a strided view) is
//! refused rather than silently copied, the same as the C++ binding. A core [`Error`] crosses
//! as the Python exception of its kind, from `slamlab.foundations.errors`.

use nalgebra::SMatrix;
use numpy::ndarray::{ArrayD, IxDyn};
use numpy::{
    Element, IntoPyArray, PyArrayDyn, PyReadonlyArray1, PyReadonlyArray2, PyReadonlyArrayDyn,
    PyUntypedArrayMethods,
};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyType;
use slamlab_foundations::error::{Error, ErrorKind};

/// A float64 numpy array returned to Python.
pub type ArrayOut<'py> = Bound<'py, PyArrayDyn<f64>>;

/// A read-only float64 numpy array passed in from Python.
pub type ArrayIn<'py> = PyReadonlyArrayDyn<'py, f64>;

/// Raises a core error as the Python exception of its kind.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `error` - The core error.
///
/// # Returns
///
/// The Python exception: `InvalidInputError`, `DegenerateError` or `NotConvergedError` from
/// `slamlab.foundations.errors`, carrying the core's message. If that module cannot be
/// imported, the import error itself.
pub fn core_error(py: Python<'_>, error: Error) -> PyErr {
    let name = match error.kind {
        ErrorKind::InvalidInput => "InvalidInputError",
        ErrorKind::Degenerate => "DegenerateError",
        ErrorKind::NotConverged => "NotConvergedError",
    };
    let class = py
        .import("slamlab.foundations.errors")
        .and_then(|module| module.getattr(name))
        .and_then(|class| Ok(class.cast_into::<PyType>()?));
    match class {
        Ok(class) => PyErr::from_type(class, error.message),
        Err(err) => err,
    }
}

/// Returns a core result's value, or its error as the matching Python exception.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `result` - A core function's result.
///
/// # Returns
///
/// The value on success.
///
/// # Errors
///
/// The core error, translated by [`core_error`].
pub fn unwrap<T>(py: Python<'_>, result: slamlab_foundations::error::Result<T>) -> PyResult<T> {
    result.map_err(|error| core_error(py, error))
}

/// A fixed-size item as stored in a row-major numpy batch.
pub trait Item: Sized {
    /// The number of values in one item.
    const LEN: usize;

    /// The item's numpy shape: `[R]` for a vector, `[R, C]` for a matrix.
    ///
    /// # Returns
    ///
    /// The shape, without the batch dimension.
    fn shape() -> Vec<usize>;

    /// Reads one item from its row-major values.
    ///
    /// # Arguments
    ///
    /// * `values` - Exactly [`Item::LEN`] values.
    ///
    /// # Returns
    ///
    /// The item.
    fn read(values: &[f64]) -> Self;

    /// Appends the item's row-major values.
    ///
    /// # Arguments
    ///
    /// * `out` - The buffer to append to.
    fn write(&self, out: &mut Vec<f64>);
}

impl<const R: usize, const C: usize> Item for SMatrix<f64, R, C> {
    const LEN: usize = R * C;

    fn shape() -> Vec<usize> {
        if C == 1 { vec![R] } else { vec![R, C] }
    }

    fn read(values: &[f64]) -> Self {
        Self::from_row_slice(values)
    }

    fn write(&self, out: &mut Vec<f64>) {
        for i in 0..R {
            for j in 0..C {
                out.push(self[(i, j)]);
            }
        }
    }
}

/// Returns an array's values as one contiguous row-major slice, checking its item shape.
///
/// # Arguments
///
/// * `array` - The array, of shape `(N, *item)`.
/// * `item` - The expected shape of one item.
///
/// # Returns
///
/// The values and the batch length N.
///
/// # Errors
///
/// `ValueError` if the shape differs or the array is not C-contiguous.
pub fn batch<'a>(array: &'a ArrayIn<'_>, item: &[usize]) -> PyResult<(&'a [f64], usize)> {
    let shape = array.shape();
    if shape.len() != item.len() + 1 || shape[1..] != *item {
        return Err(PyValueError::new_err(format!(
            "expected shape (N, {}), got {shape:?}",
            item.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        )));
    }
    let values = array
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok((values, shape[0]))
}

/// Reads a batch as a vector of items.
///
/// # Arguments
///
/// * `array` - A C-contiguous float64 array of shape `(N, *A::shape())`.
///
/// # Returns
///
/// The N items.
///
/// # Errors
///
/// `ValueError` for a wrong shape or a non-contiguous array.
pub fn rows<A: Item>(array: &ArrayIn<'_>) -> PyResult<Vec<A>> {
    let (values, _) = batch(array, &A::shape())?;
    Ok(items(values, A::LEN).map(A::read).collect())
}

/// Wraps row-major values as a new numpy array of the given shape.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `shape` - The array's shape.
/// * `values` - Its values, row-major; their count is the product of `shape`.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// `ValueError` if the count does not match the shape (a binding bug).
pub fn new_array<'py>(
    py: Python<'py>,
    shape: &[usize],
    values: Vec<f64>,
) -> PyResult<ArrayOut<'py>> {
    let array = ArrayD::from_shape_vec(IxDyn(shape), values)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(array.into_pyarray(py))
}

/// Decodes an enum crossing as its integer code.
///
/// # Arguments
///
/// * `all` - The enum's values in code order (its `ALL`).
/// * `code` - The code.
/// * `what` - The argument's name, for the message.
///
/// # Returns
///
/// The enum value.
///
/// # Errors
///
/// `ValueError` for an unknown code.
pub fn decode<E: Copy>(all: &[E], code: i64, what: &str) -> PyResult<E> {
    usize::try_from(code)
        .ok()
        .and_then(|index| all.get(index).copied())
        .ok_or_else(|| PyValueError::new_err(format!("{what}: unknown code {code}")))
}

/// Returns a one-dimensional array's values as a contiguous slice.
///
/// # Arguments
///
/// * `array` - The array.
///
/// # Returns
///
/// The values.
///
/// # Errors
///
/// `ValueError` if the array is not C-contiguous.
pub fn slice<'a, T: Element>(array: &'a PyReadonlyArray1<'_, T>) -> PyResult<&'a [T]> {
    array
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Reads one item, an array of exactly the item's shape (no batch dimension).
///
/// # Arguments
///
/// * `array` - A C-contiguous float64 array of shape `A::shape()`.
///
/// # Returns
///
/// The item.
///
/// # Errors
///
/// `ValueError` for a wrong shape or a non-contiguous array.
pub fn one<A: Item>(array: &ArrayIn<'_>) -> PyResult<A> {
    if array.shape() != A::shape().as_slice() {
        return Err(PyValueError::new_err(format!(
            "expected shape {:?}, got {:?}",
            A::shape(),
            array.shape()
        )));
    }
    let values = array
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(A::read(values))
}

/// Returns a two-dimensional array's values as a contiguous row-major slice.
///
/// # Arguments
///
/// * `array` - The array.
///
/// # Returns
///
/// The values.
///
/// # Errors
///
/// `ValueError` if the array is not C-contiguous.
pub fn slice2<'a, T: Element>(array: &'a PyReadonlyArray2<'_, T>) -> PyResult<&'a [T]> {
    array
        .as_slice()
        .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Copies items into a new array of shape `(N, *A::shape())`.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `items` - The items.
///
/// # Returns
///
/// The array.
///
/// # Errors
///
/// Never in practice (the shape always matches); see [`new_array`].
pub fn items_array<'py, A: Item>(py: Python<'py>, items: &[A]) -> PyResult<ArrayOut<'py>> {
    let mut out = Vec::with_capacity(items.len() * A::LEN);
    for item in items {
        item.write(&mut out);
    }
    new_array(py, &[&[items.len()][..], &A::shape()].concat(), out)
}

/// Packs one fixed-size item as a new array of its own shape.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `item` - The item.
///
/// # Returns
///
/// The array, of shape `A::shape()`.
///
/// # Errors
///
/// Never in practice (the shape always matches); see [`new_array`].
pub fn item_array<'py, A: Item>(py: Python<'py>, item: &A) -> PyResult<ArrayOut<'py>> {
    let mut out = Vec::with_capacity(A::LEN);
    item.write(&mut out);
    new_array(py, &A::shape(), out)
}

/// An enum's names in code order, for a `codes()` table.
///
/// # Arguments
///
/// * `all` - The enum's values in code order.
/// * `name` - The core's name function.
///
/// # Returns
///
/// The names.
pub fn enum_names<E: Copy>(all: &[E], name: fn(E) -> &'static str) -> Vec<&'static str> {
    all.iter().map(|&e| name(e)).collect()
}

/// Encodes an enum as its code: its position in `ALL`, the same list [`decode`] and
/// [`enum_names`] read, so the three cannot disagree.
///
/// # Arguments
///
/// * `all` - The enum's values in code order.
/// * `value` - The value.
///
/// # Returns
///
/// The code.
///
/// # Errors
///
/// `RuntimeError` for a value missing from `all` (a variant added to the enum but not to
/// its `ALL`), rather than a code that would read as another name.
pub fn encode<E: Copy + PartialEq + std::fmt::Debug>(all: &[E], value: E) -> PyResult<i64> {
    all.iter()
        .position(|&e| e == value)
        .and_then(|i| i64::try_from(i).ok())
        .ok_or_else(|| PyRuntimeError::new_err(format!("{value:?} is missing from its ALL")))
}

/// Looks an enum up by its name, as the core's name function spells it.
///
/// # Arguments
///
/// * `all` - The enum's values in code order.
/// * `name` - The core's name function.
/// * `value` - The name to find.
/// * `what` - The field's name, for the message.
///
/// # Returns
///
/// The enum value.
///
/// # Errors
///
/// `ValueError` for an unknown name.
pub fn by_name<E: Copy>(
    all: &[E],
    name: fn(E) -> &'static str,
    value: &str,
    what: &str,
) -> PyResult<E> {
    all.iter()
        .copied()
        .find(|&e| name(e) == value)
        .ok_or_else(|| PyValueError::new_err(format!("{what}: unknown name {value}")))
}

/// Applies a per-item core function to every item of a batch.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `input` - A C-contiguous float64 array of shape `(N, *A::shape())`.
/// * `f` - The core function, item to item; infallible ones return `Ok`.
///
/// # Returns
///
/// A new array of shape `(N, *B::shape())`.
///
/// # Errors
///
/// `ValueError` for a wrong shape or a non-contiguous array; the first item's core error,
/// translated, if `f` fails.
pub fn map_batch<'py, A: Item, B: Item>(
    py: Python<'py>,
    input: &ArrayIn<'py>,
    f: impl Fn(&A) -> slamlab_foundations::error::Result<B>,
) -> PyResult<ArrayOut<'py>> {
    let (values, n) = batch(input, &A::shape())?;
    let mut out = Vec::with_capacity(n * B::LEN);
    for item in items(values, A::LEN) {
        unwrap(py, f(&A::read(item)))?.write(&mut out);
    }
    new_array(py, &[&[n][..], &B::shape()].concat(), out)
}

/// Applies a per-item core function to every pair of items of two equal-length batches.
///
/// # Arguments
///
/// * `py` - The Python token.
/// * `first` - A C-contiguous float64 array of shape `(N, *A::shape())`.
/// * `second` - A C-contiguous float64 array of shape `(N, *B::shape())`.
/// * `f` - The core function, a pair of items to an item.
///
/// # Returns
///
/// A new array of shape `(N, *C::shape())`.
///
/// # Errors
///
/// `ValueError` for a wrong shape, a non-contiguous array or batches of different lengths;
/// the first pair's core error, translated, if `f` fails.
pub fn map_batch2<'py, A: Item, B: Item, C: Item>(
    py: Python<'py>,
    first: &ArrayIn<'py>,
    second: &ArrayIn<'py>,
    f: impl Fn(&A, &B) -> slamlab_foundations::error::Result<C>,
) -> PyResult<ArrayOut<'py>> {
    let (first_values, count) = batch(first, &A::shape())?;
    let (second_values, second_count) = batch(second, &B::shape())?;
    if count != second_count {
        return Err(PyValueError::new_err("the two batches differ in length"));
    }
    let mut out = Vec::with_capacity(count * C::LEN);
    for (x, y) in items(first_values, A::LEN).zip(items(second_values, B::LEN)) {
        unwrap(py, f(&A::read(x), &B::read(y)))?.write(&mut out);
    }
    new_array(py, &[&[count][..], &C::shape()].concat(), out)
}

/// Splits a batch's values into its items.
///
/// # Arguments
///
/// * `values` - The batch, row-major.
/// * `len` - The values in one item.
///
/// # Returns
///
/// An iterator over the items.
#[allow(
    clippy::chunks_exact_to_as_chunks,
    reason = "the item length is an associated constant of a generic type, which as_chunks \
              cannot take on stable Rust"
)]
fn items(values: &[f64], len: usize) -> std::slice::ChunksExact<'_, f64> {
    values.chunks_exact(len)
}

/// Defines a batched binding, mapped item by item with [`map_batch`] (one argument) or
/// [`map_batch2`] (two).
///
/// `batched!(/// doc  name(arg: In) -> Out = core_fn)` defines `fn name(arg)` for a core
/// function `core_fn(&In) -> Result<Out>`; with two arguments, `core_fn(&A, &B)`. The
/// argument names are the Python Protocol's, which the Protocol test checks.
macro_rules! batched {
    ($(#[$meta:meta])* $name:ident($($arg:ident: $ty:ty),+) -> $out:ty = $f:expr) => {
        $(#[$meta])*
        #[pyfunction]
        #[allow(
            clippy::needless_pass_by_value,
            reason = "PyO3 extracts arguments by value; the read-only borrow guard is the argument"
        )]
        fn $name<'py>(
            py: Python<'py>,
            $($arg: $crate::common::ArrayIn<'py>),+
        ) -> PyResult<$crate::common::ArrayOut<'py>> {
            $crate::common::batched!(@map py, $f, $out; $($arg: $ty),+)
        }
    };
    (@map $py:ident, $f:expr, $out:ty; $a:ident: $a_ty:ty) => {
        $crate::common::map_batch::<$a_ty, $out>($py, &$a, $f)
    };
    (@map $py:ident, $f:expr, $out:ty; $a:ident: $a_ty:ty, $b:ident: $b_ty:ty) => {
        $crate::common::map_batch2::<$a_ty, $b_ty, $out>($py, &$a, &$b, $f)
    };
}

/// Adds functions to a module: `add_functions!(m; hat, vee)`.
macro_rules! add_functions {
    ($m:ident; $($name:ident),+ $(,)?) => {
        $($m.add_function(wrap_pyfunction!($name, $m)?)?;)+
    };
}

pub(crate) use {add_functions, batched};
