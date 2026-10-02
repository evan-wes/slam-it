"""The reduced camera-side coordinates and the Schur-complement normal equations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from slamlab.foundations import nls
from slamlab.foundations.ba.problem import Array, IndexArray

PAIR_CHUNK = 4096
"""Observation pairs processed at once when forming the Schur complement (bounds memory)."""


class ReducedSystemSolver(Protocol):
    """Solves the reduced camera system ``S x = b``, S symmetric positive definite."""

    def solve(self, matrix: Array, rhs: Array) -> Array | None:
        """Return ``x``, or ``None`` if ``matrix`` is not positive definite."""
        ...


class DenseCholesky:
    """The dense reduced solver: Cholesky on the full matrix."""

    def solve(self, matrix: Array, rhs: Array) -> Array | None:
        """Solve ``matrix x = rhs`` by Cholesky.

        Parameters
        ----------
        matrix : ndarray of float64, shape (n, n)
            Symmetric positive definite.
        rhs : ndarray of float64, shape (n,)
            The right-hand side.

        Returns
        -------
        ndarray of float64, shape (n,), or None
            The solution, or ``None`` if ``matrix`` is not positive definite.
        """
        return nls.cholesky_solve(matrix, rhs)


# --- the reduced camera-side coordinates ----------------------------------------------------


def householder_complement(normal: Array) -> Array:
    """Return an orthonormal (n, n-1) basis of the complement of ``normal``, deterministically.

    The Householder reflection mapping ``normal`` onto its largest axis keeps every other axis
    orthogonal to it; those columns are the basis.
    """
    unit = normal / np.linalg.norm(normal)
    k = int(np.argmax(np.abs(unit)))
    v = unit.copy()
    v[k] += 1.0 if unit[k] >= 0.0 else -1.0
    reflection = np.eye(unit.shape[0]) - 2.0 * np.outer(v, v) / (v @ v)
    return np.delete(reflection, k, axis=1)


@dataclass(frozen=True)
class Reduction:
    """Maps between the full camera-side vector and the free one.

    The full vector holds 6 entries per pose, P per intrinsics group and 2 per depth affine;
    the free coordinates are the free full entries, in order, then the scale pose's five.
    """

    free: IndexArray
    rows: IndexArray
    basis: Array
    full_size: int

    @property
    def size(self) -> int:
        """The number of free camera-side coordinates."""
        return int(self.free.shape[0] + self.basis.shape[1])

    def reduce_vector(self, vector: Array) -> Array:
        """Project a full vector (a gradient) onto the free coordinates."""
        return np.concatenate([vector[self.free], self.basis.T @ vector[self.rows]])

    def reduce_matrix(self, matrix: Array) -> Array:
        """Project a full symmetric matrix onto the free coordinates: ``Bᵀ M B``."""
        top = matrix[np.ix_(self.free, self.free)]
        if self.basis.shape[1] == 0:
            return top
        side = matrix[np.ix_(self.free, self.rows)] @ self.basis
        corner = self.basis.T @ matrix[np.ix_(self.rows, self.rows)] @ self.basis
        return np.block([[top, side], [side.T, corner]])

    def reduce_diagonal(self, matrix: Array) -> Array:
        """Return the diagonal of ``reduce_matrix(matrix)`` without forming it."""
        corner = self.basis.T @ matrix[np.ix_(self.rows, self.rows)] @ self.basis
        return np.concatenate([np.diagonal(matrix)[self.free], np.diagonal(corner)])

    def expand(self, reduced: Array) -> Array:
        """Map free coordinates (a step) to the full vector."""
        full = np.zeros(self.full_size)
        n_free = self.free.shape[0]
        full[self.free] = reduced[:n_free]
        full[self.rows] += self.basis @ reduced[n_free:]
        return full


# --- the normal equations with the points eliminated ----------------------------------------


@dataclass(frozen=True)
class Blocks:
    """The blocks of the normal equations, formed once per linearisation.

    Attributes
    ----------
    hessian_cc, gradient_c : ndarray of float64
        The full camera-side Hessian (n, n) and gradient (n,).
    hessian_pp, gradient_p : ndarray of float64
        Each free point's 3x3 Hessian block (Nf, 3, 3) and gradient (Nf, 3).
    coupling : ndarray of float64, shape (Mf, w, 3)
        Each point-coupled block's camera-point block ``J_cᵀ W J_p``, zero-padded to the
        widest kind of block.
    columns : ndarray of int64, shape (Mf, w)
        Those blocks' camera-side columns (padding points at column 0, with zero coupling).
    slots : ndarray of int64, shape (Mf,)
        Those blocks' free-point slots.
    real : ndarray of bool, shape (Nf * 3,)
        Which point dimensions are variables. Every point block is 3x3; an inverse-depth
        point uses one dimension, and its other two are inert (identity Hessian, no
        coupling, no gradient), so their step is exactly zero and they are hidden from the
        solver.
    """

    hessian_cc: Array
    gradient_c: Array
    hessian_pp: Array
    gradient_p: Array
    coupling: Array
    columns: IndexArray
    slots: IndexArray
    real: NDArray[np.bool_]


class SchurSystem:
    """The normal equations with the points eliminated: an NLS ``LinearSystem``."""

    def __init__(self, reduction: Reduction, blocks: Blocks, solver: ReducedSystemSolver) -> None:
        """Hold the blocks; see :meth:`solve` for how they combine.

        Parameters
        ----------
        reduction : Reduction
            The free camera-side coordinates.
        blocks : Blocks
            The normal equations' blocks.
        solver : ReducedSystemSolver
            Factors the reduced camera system.
        """
        self._reduction = reduction
        self._blocks = blocks
        self._solver = solver
        slots = blocks.slots
        order = np.argsort(slots, kind="stable")
        counts = np.bincount(slots[order], minlength=blocks.hessian_pp.shape[0])
        per_block = counts[slots[order]]
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])[slots[order]]
        offsets = np.arange(int(per_block.sum())) - np.repeat(
            np.cumsum(per_block) - per_block, per_block
        )
        self._pairs = (
            np.repeat(order, per_block),
            order[np.repeat(starts, per_block) + offsets],
        )

    @property
    def gradient(self) -> Array:
        """The gradient in the free coordinates: cameras, then points."""
        return np.concatenate(
            [
                self._reduction.reduce_vector(self._blocks.gradient_c),
                self._blocks.gradient_p.reshape(-1)[self._blocks.real],
            ]
        )

    @property
    def hessian_diagonal(self) -> Array:
        """The Hessian's diagonal in the free coordinates."""
        return np.concatenate(
            [
                self._reduction.reduce_diagonal(self._blocks.hessian_cc),
                np.diagonal(self._blocks.hessian_pp, axis1=1, axis2=2).reshape(-1)[
                    self._blocks.real
                ],
            ]
        )

    def _eliminate(self, damping: Array) -> tuple[Array, Array, Array] | None:
        """Eliminate the points from the damped system.

        Returns the reduced matrix, the reduced right-hand side and the damped point blocks'
        inverses; or ``None`` if a point block is indefinite.
        """
        b = self._blocks
        n_cam = self._reduction.size
        point_damping = np.zeros(b.real.shape[0])
        point_damping[b.real] = damping[n_cam:]
        damped_pp = b.hessian_pp + point_damping.reshape(-1, 3)[:, :, None] * np.eye(3)
        try:
            lower = np.linalg.cholesky(damped_pp)
        except np.linalg.LinAlgError:
            return None
        lower_inv = np.linalg.inv(lower)
        inverse_pp = np.einsum("pki,pkj->pij", lower_inv, lower_inv)
        weighted = np.einsum("mij,mjk->mik", b.coupling, inverse_pp[b.slots])
        schur = b.hessian_cc.copy()
        first, second = self._pairs
        for start in range(0, first.shape[0], PAIR_CHUNK):
            i, j = first[start : start + PAIR_CHUNK], second[start : start + PAIR_CHUNK]
            pair_blocks = np.einsum("pik,pjk->pij", weighted[i], b.coupling[j])
            np.subtract.at(schur, (b.columns[i][:, :, None], b.columns[j][:, None, :]), pair_blocks)
        rhs = b.gradient_c.copy()
        np.subtract.at(rhs, b.columns, np.einsum("mik,mk->mi", weighted, b.gradient_p[b.slots]))
        reduced = self._reduction.reduce_matrix(schur) + np.diag(damping[:n_cam])
        return reduced, self._reduction.reduce_vector(rhs), inverse_pp

    def solve(self, damping: Array) -> Array | None:
        """Solve the damped normal equations by eliminating the points.

        Parameters
        ----------
        damping : ndarray of float64
            Added to the Hessian's diagonal, in the free coordinates.

        Returns
        -------
        ndarray of float64, or None
            The step, or ``None`` if a point block or the reduced system is indefinite.
        """
        eliminated = self._eliminate(damping)
        if eliminated is None:
            return None
        reduced, rhs, inverse_pp = eliminated
        step_c = self._solver.solve(reduced, -rhs)
        if step_c is None:
            return None
        b = self._blocks
        full_c = self._reduction.expand(step_c)
        back = b.gradient_p.copy()
        np.add.at(back, b.slots, np.einsum("mij,mi->mj", b.coupling, full_c[b.columns]))
        step_p = -np.einsum("pij,pj->pi", inverse_pp, back)
        return np.concatenate([step_c, step_p.reshape(-1)[b.real]])

    def camera_variances(self, positions: IndexArray) -> Array:
        """Return marginal variances of camera-side free coordinates: ``diag(S⁻¹)``.

        Parameters
        ----------
        positions : ndarray of int64
            The free coordinates wanted.

        Returns
        -------
        ndarray of float64
            Their variances, or infinity for all if the reduced system is singular.
        """
        size = self._reduction.size + int(np.count_nonzero(self._blocks.real))
        undamped = self._eliminate(np.zeros(size))
        if undamped is None:
            return np.full(positions.shape, np.inf)
        reduced = undamped[0]
        try:
            lower = np.linalg.cholesky(reduced)
        except np.linalg.LinAlgError:
            return np.full(positions.shape, np.inf)
        lower_inv = np.linalg.inv(lower)
        return np.asarray(np.sum(lower_inv[:, positions] ** 2, axis=0), dtype=np.float64)
