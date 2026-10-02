//! The `slamlab_rs` Python module: thin `PyO3` wrappers over `slamlab-core`.
//!
//! Submodules mirror the Python package tree (`slamlab_rs.foundations.so3` implements
//! `slamlab.foundations.so3`), so names never collide across rungs. Wrappers only convert
//! types; the algorithms live in `slamlab-core`, which builds without Python, and the Python
//! entry points validate input before calling.

use pyo3::prelude::*;

/// The `slamlab_rs` module.
#[pymodule]
mod slamlab_rs {
    use pyo3::prelude::*;

    /// Rung 0, foundations.
    #[pymodule]
    mod foundations {
        use pyo3::prelude::*;

        /// SO(3), the group of 3D rotations.
        #[pymodule]
        mod so3 {
            use pyo3::prelude::*;

            /// Maps a 3-vector to its skew-symmetric matrix, returned as nested lists.
            ///
            /// # Arguments
            ///
            /// * `v` - The vector to map. `PyO3` raises `ValueError` for a sequence of the
            ///   wrong length and `TypeError` for non-numeric elements; the Python entry
            ///   point validates first, so these only reach direct callers.
            ///
            /// # Returns
            ///
            /// The skew-symmetric matrix `[v]x`, row-major.
            #[pyfunction]
            fn hat(v: [f64; 3]) -> [[f64; 3]; 3] {
                slamlab_core::foundations::so3::hat(v)
            }
        }
    }
}
