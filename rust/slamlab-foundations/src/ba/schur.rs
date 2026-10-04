//! Bundle adjustment internals: the reduced camera-side coordinates and the
//! Schur-complement normal equations.

use nalgebra::{DMatrix, DVector, Dyn, Matrix3, OMatrix, U3, Vector3};

use super::ReducedSystemSolver;
use crate::nls::LinearSystem;

/// A camera-side coupling block: one row per camera-side column, three point columns.
pub type Coupling = OMatrix<f64, Dyn, U3>;

/// Maps between the full camera-side vector (6 per pose, P per group, 2 per affine) and the
/// free coordinates: the free full entries, in order, then each confined pose's basis
/// coordinates.
#[derive(Debug, Clone)]
pub struct Reduction {
    /// The free full entries.
    free: Vec<usize>,
    /// The confined poses' six full entries each.
    rows: Vec<usize>,
    /// Their block-diagonal basis (6 per confined pose by its free directions).
    basis: DMatrix<f64>,
    /// The full vector's length.
    full_size: usize,
}

impl Reduction {
    /// Builds the reduction.
    ///
    /// # Arguments
    ///
    /// * `free` - The free full entries.
    /// * `rows` - The confined poses' six full entries each (or none).
    /// * `basis` - Their block-diagonal basis (or 0 x 0).
    /// * `full_size` - The full vector's length.
    ///
    /// # Returns
    ///
    /// The reduction.
    pub fn new(free: Vec<usize>, rows: Vec<usize>, basis: DMatrix<f64>, full_size: usize) -> Self {
        Self {
            free,
            rows,
            basis,
            full_size,
        }
    }

    /// The number of free coordinates.
    ///
    /// # Returns
    ///
    /// The size.
    pub fn size(&self) -> usize {
        self.free.len() + self.basis.ncols()
    }

    /// Projects a full vector onto the free coordinates.
    ///
    /// # Arguments
    ///
    /// * `vector` - The full vector.
    ///
    /// # Returns
    ///
    /// The reduced vector.
    pub fn reduce_vector(&self, vector: &DVector<f64>) -> DVector<f64> {
        let mut out = DVector::zeros(self.size());
        for (i, &f) in self.free.iter().enumerate() {
            out[i] = vector[f];
        }
        if self.basis.ncols() > 0 {
            let confined = vector.select_rows(&self.rows);
            out.rows_mut(self.free.len(), self.basis.ncols())
                .copy_from(&self.basis.tr_mul(&confined));
        }
        out
    }

    /// Projects a full symmetric matrix onto the free coordinates, `Bᵀ M B`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The full matrix.
    ///
    /// # Returns
    ///
    /// The reduced matrix.
    pub fn reduce_matrix(&self, matrix: &DMatrix<f64>) -> DMatrix<f64> {
        let n_free = self.free.len();
        let k = self.basis.ncols();
        let mut out = DMatrix::zeros(self.size(), self.size());
        for (j, &fj) in self.free.iter().enumerate() {
            for (i, &fi) in self.free.iter().enumerate() {
                out[(i, j)] = matrix[(fi, fj)];
            }
        }
        if k > 0 {
            let side = matrix.select_rows(&self.free).select_columns(&self.rows) * &self.basis;
            out.view_mut((0, n_free), (n_free, k)).copy_from(&side);
            out.view_mut((n_free, 0), (k, n_free))
                .copy_from(&side.transpose());
            let corner = matrix.select_rows(&self.rows).select_columns(&self.rows);
            out.view_mut((n_free, n_free), (k, k))
                .copy_from(&(self.basis.transpose() * corner * &self.basis));
        }
        out
    }

    /// The diagonal of `reduce_matrix(matrix)`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The full matrix.
    ///
    /// # Returns
    ///
    /// The reduced diagonal.
    pub fn reduce_diagonal(&self, matrix: &DMatrix<f64>) -> DVector<f64> {
        let mut out = DVector::zeros(self.size());
        for (i, &f) in self.free.iter().enumerate() {
            out[i] = matrix[(f, f)];
        }
        if self.basis.ncols() > 0 {
            let corner = matrix.select_rows(&self.rows).select_columns(&self.rows);
            let reduced = self.basis.transpose() * corner * &self.basis;
            out.rows_mut(self.free.len(), self.basis.ncols())
                .copy_from(&reduced.diagonal());
        }
        out
    }

    /// Maps free coordinates (a step) to the full vector.
    ///
    /// # Arguments
    ///
    /// * `reduced` - The free coordinates.
    ///
    /// # Returns
    ///
    /// The full vector.
    pub fn expand(&self, reduced: &DVector<f64>) -> DVector<f64> {
        let mut full = DVector::zeros(self.full_size);
        for (i, &f) in self.free.iter().enumerate() {
            full[f] = reduced[i];
        }
        if self.basis.ncols() > 0 {
            let confined = &self.basis * reduced.rows(self.free.len(), self.basis.ncols());
            for (i, &r) in self.rows.iter().enumerate() {
                full[r] += confined[i];
            }
        }
        full
    }
}

/// Scatter-adds a block into a matrix, accumulating repeated indices (a block's columns
/// repeat when an anchored point's host is the observing camera).
///
/// # Arguments
///
/// * `target` - The matrix.
/// * `rows` - The block's rows in `target`.
/// * `cols` - The block's columns in `target`.
/// * `block` - The block.
pub fn scatter_add(
    target: &mut DMatrix<f64>,
    rows: &[usize],
    cols: &[usize],
    block: &DMatrix<f64>,
) {
    // Column-outer: the matrices are column-major.
    for (j, &c) in cols.iter().enumerate() {
        for (i, &r) in rows.iter().enumerate() {
            target[(r, c)] += block[(i, j)];
        }
    }
}

/// Scatter-adds a vector, accumulating repeated indices.
///
/// # Arguments
///
/// * `target` - The vector.
/// * `rows` - The entries' positions in `target`.
/// * `values` - The values.
pub fn scatter_add_vector(target: &mut DVector<f64>, rows: &[usize], values: &DVector<f64>) {
    for (i, &r) in rows.iter().enumerate() {
        target[r] += values[i];
    }
}

/// The blocks of the normal equations, formed once per linearisation.
#[derive(Debug, Clone)]
pub struct Blocks {
    /// The full camera-side Hessian.
    pub hessian_cc: DMatrix<f64>,
    /// The full camera-side gradient.
    pub gradient_c: DVector<f64>,
    /// Each free point's 3x3 Hessian block (inert identity on unused dimensions).
    pub hessian_pp: Vec<Matrix3<f64>>,
    /// Each free point's gradient.
    pub gradient_p: Vec<Vector3<f64>>,
    /// Each point-coupled block's coupling `J_cᵀ W J_p` (w x 3).
    pub coupling: Vec<Coupling>,
    /// Those blocks' camera-side columns.
    pub columns: Vec<Vec<usize>>,
    /// Those blocks' free-point slots.
    pub slots: Vec<usize>,
    /// Each free point's number of variables: 3 (xyz), or 1 (an inverse depth).
    pub point_dims: Vec<usize>,
}

/// The reduced system after eliminating the points.
struct Eliminated {
    /// The reduced camera matrix, damped.
    reduced: DMatrix<f64>,
    /// The reduced right-hand side (the reduced gradient).
    rhs: DVector<f64>,
    /// The damped point blocks' inverses.
    inverse_pp: Vec<Matrix3<f64>>,
}

/// The normal equations with the points eliminated: an [`LinearSystem`].
#[derive(Debug, Clone)]
pub struct SchurSystem<S: ReducedSystemSolver> {
    /// The free camera-side coordinates.
    reduction: Reduction,
    /// The normal equations' blocks (couplings merged into `point_coupling`).
    blocks: Blocks,
    /// Factors the reduced camera system.
    solver: S,
    /// Each free point's distinct camera-side columns, ascending.
    point_columns: Vec<Vec<usize>>,
    /// Each free point's merged coupling over those columns.
    point_coupling: Vec<Coupling>,
    /// Each free point's offset in the point part of the free coordinates.
    offsets: Vec<usize>,
    /// The gradient in the free coordinates, formed once.
    gradient: DVector<f64>,
}

impl<S: ReducedSystemSolver> SchurSystem<S> {
    /// Holds the blocks, merging each point's couplings.
    ///
    /// # Arguments
    ///
    /// * `reduction` - The free camera-side coordinates.
    /// * `blocks` - The normal equations' blocks.
    /// * `solver` - Factors the reduced camera system.
    ///
    /// # Returns
    ///
    /// The system.
    pub fn new(reduction: Reduction, blocks: Blocks, solver: S) -> Self {
        let mut offsets = vec![0];
        for &dims in &blocks.point_dims {
            offsets.push(offsets.last().copied().unwrap_or(0) + dims);
        }
        let mut system = Self {
            reduction,
            blocks,
            solver,
            point_columns: Vec::new(),
            point_coupling: Vec::new(),
            offsets,
            gradient: DVector::zeros(0),
        };
        system.merge_couplings();
        system.gradient = system.free_gradient();
        system
    }

    /// The total number of point variables.
    ///
    /// # Returns
    ///
    /// The count.
    fn point_size(&self) -> usize {
        self.offsets.last().copied().unwrap_or(0)
    }

    /// The gradient in the free coordinates: cameras, then points.
    ///
    /// # Returns
    ///
    /// The gradient.
    fn free_gradient(&self) -> DVector<f64> {
        let n_cam = self.reduction.size();
        let mut out = DVector::zeros(n_cam + self.point_size());
        out.rows_mut(0, n_cam)
            .copy_from(&self.reduction.reduce_vector(&self.blocks.gradient_c));
        for (p, g) in self.blocks.gradient_p.iter().enumerate() {
            let dims = self.blocks.point_dims[p];
            out.rows_mut(n_cam + self.offsets[p], dims)
                .copy_from(&g.rows(0, dims));
        }
        out
    }

    /// Marginal variances of camera-side free coordinates, `diag(S⁻¹)`.
    ///
    /// # Arguments
    ///
    /// * `positions` - The free coordinates wanted.
    ///
    /// # Returns
    ///
    /// Their variances, or infinity for all if the reduced system is singular.
    pub fn camera_variances(&self, positions: &[usize]) -> Vec<f64> {
        let damping = DVector::zeros(self.reduction.size() + self.point_size());
        let factor = self
            .eliminate(&damping)
            .and_then(|undamped| undamped.reduced.cholesky());
        let Some(factor) = factor else {
            return vec![f64::INFINITY; positions.len()];
        };
        let inverse = factor.inverse();
        positions.iter().map(|&i| inverse[(i, i)]).collect()
    }

    /// Eliminates the points from the damped system.
    ///
    /// # Arguments
    ///
    /// * `damping` - Added to the Hessian's diagonal, in the free coordinates.
    ///
    /// # Returns
    ///
    /// The reduced system, or `None` if a point block is indefinite.
    fn eliminate(&self, damping: &DVector<f64>) -> Option<Eliminated> {
        let n_cam = self.reduction.size();
        let mut inverse_pp = Vec::with_capacity(self.blocks.hessian_pp.len());
        for (p, block) in self.blocks.hessian_pp.iter().enumerate() {
            let mut damped = *block;
            for d in 0..self.blocks.point_dims[p] {
                damped[(d, d)] += damping[n_cam + self.offsets[p] + d];
            }
            inverse_pp.push(damped.cholesky()?.inverse());
        }
        let mut schur = self.blocks.hessian_cc.clone();
        let mut rhs = self.blocks.gradient_c.clone();
        for (p, inverse) in inverse_pp.iter().enumerate() {
            // One product per point: S -= E_p H_p⁻¹ E_pᵀ, b -= E_p H_p⁻¹ g_p.
            let coupling = &self.point_coupling[p];
            let weighted = coupling * inverse;
            let columns = &self.point_columns[p];
            subtract_upper(&mut schur, columns, &(&weighted * coupling.transpose()));
            scatter_add_vector(&mut rhs, columns, &-(&weighted * self.blocks.gradient_p[p]));
        }
        schur.fill_lower_triangle_with_upper_triangle();
        let mut reduced = self.reduction.reduce_matrix(&schur);
        for i in 0..n_cam {
            reduced[(i, i)] += damping[i];
        }
        Some(Eliminated {
            reduced,
            rhs: self.reduction.reduce_vector(&rhs),
            inverse_pp,
        })
    }

    /// Merges each point's blocks into one coupling matrix over its distinct camera-side
    /// columns (a point's blocks share, for one, their camera's intrinsics columns), so
    /// elimination does one product per point rather than one per pair of its blocks.
    fn merge_couplings(&mut self) {
        let n_points = self.blocks.hessian_pp.len();
        let mut by_point: Vec<Vec<usize>> = vec![Vec::new(); n_points];
        for (m, &slot) in self.blocks.slots.iter().enumerate() {
            by_point[slot].push(m);
        }
        let mut position: Vec<Option<usize>> = vec![None; self.blocks.hessian_cc.nrows()];
        self.point_columns = Vec::with_capacity(n_points);
        self.point_coupling = Vec::with_capacity(n_points);
        for blocks_of_point in &by_point {
            let mut columns: Vec<usize> = Vec::new();
            for &m in blocks_of_point {
                for &c in &self.blocks.columns[m] {
                    if position[c].is_none() {
                        position[c] = Some(columns.len());
                        columns.push(c);
                    }
                }
            }
            let mut coupling = Coupling::zeros(columns.len());
            for &m in blocks_of_point {
                for (i, &c) in self.blocks.columns[m].iter().enumerate() {
                    if let Some(row) = position[c] {
                        let mut target = coupling.row_mut(row);
                        target += self.blocks.coupling[m].row(i);
                    }
                }
            }
            for &c in &columns {
                position[c] = None;
            }
            // Ascending columns: the elimination's scatter then walks memory in order.
            let mut order: Vec<usize> = (0..columns.len()).collect();
            order.sort_by_key(|&k| columns[k]);
            let sorted_columns: Vec<usize> = order.iter().map(|&k| columns[k]).collect();
            let sorted_coupling = coupling.select_rows(&order);
            self.point_columns.push(sorted_columns);
            self.point_coupling.push(sorted_coupling);
        }
        // Merged: the per-block copies are no longer needed.
        self.blocks.coupling = Vec::new();
        self.blocks.columns = Vec::new();
    }
}

/// Subtracts a symmetric block from the upper triangle of a matrix.
///
/// # Arguments
///
/// * `target` - The matrix (only its upper triangle is updated).
/// * `columns` - The block's rows and columns in `target`, ascending and distinct.
/// * `block` - The symmetric block.
fn subtract_upper(target: &mut DMatrix<f64>, columns: &[usize], block: &DMatrix<f64>) {
    for (j, &cj) in columns.iter().enumerate() {
        for (i, &ci) in columns.iter().enumerate().take(j + 1) {
            target[(ci, cj)] -= block[(i, j)];
        }
    }
}

impl<S: ReducedSystemSolver> LinearSystem for SchurSystem<S> {
    fn gradient(&self) -> &DVector<f64> {
        &self.gradient
    }

    fn hessian_diagonal(&self) -> DVector<f64> {
        let n_cam = self.reduction.size();
        let mut out = DVector::zeros(n_cam + self.point_size());
        out.rows_mut(0, n_cam)
            .copy_from(&self.reduction.reduce_diagonal(&self.blocks.hessian_cc));
        for (p, block) in self.blocks.hessian_pp.iter().enumerate() {
            for d in 0..self.blocks.point_dims[p] {
                out[n_cam + self.offsets[p] + d] = block[(d, d)];
            }
        }
        out
    }

    fn solve(&self, damping: &DVector<f64>) -> Option<DVector<f64>> {
        let eliminated = self.eliminate(damping)?;
        let step_c = self.solver.solve(eliminated.reduced, &-eliminated.rhs)?;
        let full_c = self.reduction.expand(&step_c);
        let n_cam = self.reduction.size();
        let mut step = DVector::zeros(n_cam + self.point_size());
        step.rows_mut(0, n_cam).copy_from(&step_c);
        for (p, inverse) in eliminated.inverse_pp.iter().enumerate() {
            let back = self.blocks.gradient_p[p]
                + self.point_coupling[p].transpose() * full_c.select_rows(&self.point_columns[p]);
            let point_step = -(inverse * back);
            let dims = self.blocks.point_dims[p];
            step.rows_mut(n_cam + self.offsets[p], dims)
                .copy_from(&point_step.rows(0, dims));
        }
        Some(step)
    }
}

#[cfg(test)]
mod tests {
    //! Native tests of the reduced coordinates, runnable without Python.
    #![allow(
        clippy::cast_precision_loss,
        reason = "small indices fill test matrices"
    )]

    use super::*;

    /// A reduction over a 14-entry vector: entries 0, 2 and 3 free, and entries 6..12 (one
    /// confined pose) on a five-column orthonormal basis.
    fn reduction() -> Reduction {
        let raw = DMatrix::from_fn(6, 5, |i, j| ((i * 5 + j) as f64 * 0.37).sin());
        let basis = raw.qr().q();
        Reduction::new(vec![0, 2, 3], (6..12).collect(), basis, 14)
    }

    /// Expanding free coordinates and reducing them again is the identity, as the basis is
    /// orthonormal and its rows apart from the free entries.
    #[test]
    fn expand_then_reduce_is_the_identity() {
        let reduction = reduction();
        let reduced = DVector::from_fn(reduction.size(), |i, _| 1.0 + i as f64);
        let back = reduction.reduce_vector(&reduction.expand(&reduced));
        assert!((back - reduced).amax() < 1e-14);
    }

    /// The reduced diagonal is the diagonal of the reduced matrix, `Bᵀ M B`.
    #[test]
    fn reduced_diagonal_matches_the_reduced_matrix() {
        let reduction = reduction();
        let m = DMatrix::from_fn(14, 14, |i, j| ((i + 2 * j) as f64).cos());
        let symmetric = &m + m.transpose();
        let reduced = reduction.reduce_matrix(&symmetric);
        let diagonal = reduction.reduce_diagonal(&symmetric);
        assert!((reduced.diagonal() - diagonal).amax() < 1e-13);
        assert!((reduced.transpose() - &reduced).amax() < 1e-13);
    }
}
