"""What every iterative solver returns: its best estimate, and a summary of how it went.

Rung 0's iterative solvers (non-linear least squares, bundle adjustment, RANSAC) share one
contract (design doc, "The error contract"): a solve that stops short of its convergence
test is a normal result, not an error. It returns the best estimate it found together with a
summary, and the caller reads ``summary.converged`` and ``summary.termination`` to decide
what to do. Errors are kept for input that is invalid or degenerate, for a caller's strict
mode, and for a solve that ends with no usable estimate at all.

Each solver owns its summary, because what is worth reporting differs: least squares
reports costs and a trace of steps; RANSAC reports samples drawn, inliers and whether the
target confidence was reached. What they share is :class:`Summary`, the three fields every
summary has, and :class:`Solution`, the pair of estimate and summary.

The C++ core mirrors this in ``slamlab/foundations/solve.hpp``: a ``Summary`` concept and a
``Solution<State, Summary>`` template.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar


class Summary(Protocol):
    """The fields every solver's summary has.

    Attributes
    ----------
    iterations : int
        The solver's iterations: steps tried, or hypotheses sampled.
    termination : enum.StrEnum
        Why the solve stopped, from the solver's own termination enum.
    converged : bool
        Whether ``termination`` is one of the solver's converged terminations.
    """

    @property
    def iterations(self) -> int:
        """The solver's iterations."""
        ...

    @property
    def termination(self) -> enum.StrEnum:
        """Why the solve stopped."""
        ...

    @property
    def converged(self) -> bool:
        """Whether the solve converged."""
        ...


StateT = TypeVar("StateT")
SummaryT = TypeVar("SummaryT", bound=Summary)


@dataclass(frozen=True)
class Solution(Generic[StateT, SummaryT]):
    """A solver's best estimate, and how the solve went.

    Attributes
    ----------
    state : StateT
        The estimate.
    summary : SummaryT
        The solver's summary.
    """

    state: StateT
    summary: SummaryT
