//! What every iterative solver returns: its best estimate, and a summary of how it went.
//!
//! Mirrors `slam_it_all.foundations.solve` and the C++ core's `solve.hpp`. Each solver
//! (non-linear least squares, RANSAC) owns its summary, because what is worth reporting
//! differs; the [`Summary`] trait names what they all report, and [`Solution`] pairs the
//! estimate with its summary. A solve that stops short of its convergence test returns a
//! `Solution`, not an error.

/// What every solver's summary reports.
pub trait Summary {
    /// The solver's reason for stopping.
    type Termination: Copy;

    /// The iterations run.
    ///
    /// # Returns
    ///
    /// The count.
    fn iterations(&self) -> usize;

    /// Whether the solver's convergence test passed.
    ///
    /// # Returns
    ///
    /// True when converged.
    fn converged(&self) -> bool;

    /// Why the solver stopped.
    ///
    /// # Returns
    ///
    /// The termination reason.
    fn termination(&self) -> Self::Termination;
}

/// A solver's best estimate, and how the solve went.
#[derive(Debug, Clone)]
pub struct Solution<State, S: Summary> {
    /// The estimate.
    pub state: State,
    /// The solver's summary.
    pub summary: S,
}
