"""Non-linear least squares on manifolds: Gauss-Newton and Levenberg-Marquardt.

The solver minimises a robust sum over residual **blocks**,

    cost(x) = ½ Σ_i ρ(|r_i(x)|²),

where each block ``r_i`` (a reprojection error, a prior, ...) has a few rows and ``ρ`` is a
robust loss. The state ``x`` lives on a manifold (poses, rotations, points): the solver
never adds to it directly. It asks the problem for a step ``δ`` in the tangent space and
applies it with the problem's own ``retract(x, δ)`` (for a pose, ``T ⊕ δ = T exp(δ)``).

**The problem** (:class:`Problem`) supplies three things: the blocks' squared norms, a
linearisation (a :class:`LinearSystem`) given per-block weights, and ``retract``. **The
linear system** owns the structure: :class:`DenseSystem` forms and factors the full normal
equations; bundle adjustment supplies a Schur-complement system instead. The loop below is
the same for both.

**Robust losses by iteratively reweighted least squares.** At each linearisation, block i
gets the weight ``w_i = ρ'(s_i)`` and the step solves the weighted Gauss-Newton system
``(Jᵀ W J + D) δ = -Jᵀ W r``. Its gradient ``Jᵀ W r`` is exactly the robust cost's gradient,
so the solver converges to the robust optimum. The second-order term ``ρ''`` that Triggs et
al.'s correction (used by Ceres) adds to the Hessian is dropped: this keeps the system
positive semi-definite without further clamping, at the price of slower convergence where
``ρ''`` is large. The losses act on the squared norm ``s`` with a scale ``c``:

- ``"trivial"``: ``ρ(s) = s``;
- ``"huber"``: ``ρ(s) = s`` for ``s ≤ c²``, else ``2c√s - c²``;
- ``"cauchy"``: ``ρ(s) = c² log(1 + s/c²)``.

These match Ceres' ``HuberLoss``/``CauchyLoss`` and SciPy's ``least_squares`` losses with
``f_scale = c``.

**Levenberg-Marquardt** damps with ``D = μ · clamp(diag(Jᵀ W J))`` (Marquardt's scaling,
clamped to ``[min_diagonal, max_diagonal]`` as Ceres does). A step is accepted when the
ratio of actual to predicted cost decrease exceeds ``min_relative_decrease``; ``μ`` is then
updated by Nielsen's rule, ``μ ← μ · max(1/3, 1 - (2ρ - 1)³)``. A rejected step, or a system
the Cholesky factorisation finds indefinite, multiplies ``μ`` by a growing factor ``ν``.
**Gauss-Newton** takes undamped steps and stops at the first that fails to decrease the cost.

**Termination** (Ceres' tests): the gradient's max-norm falls below ``gradient_tolerance``;
an accepted step decreases the cost by less than ``function_tolerance`` relative to it; the
step's norm falls below ``step_tolerance`` (absolute, in the tangent space, which has no
natural scale here); or ``max_iterations`` steps have been tried. **Every step is accepted
only if it decreases the cost,** so the result is never worse than the start. Reaching the
iteration cap is not an error: :func:`solve` returns its best estimate with a
:class:`Summary` (the shared contract is in :mod:`slamlab.foundations.solve`), and
raises :class:`~slamlab.foundations.errors.NotConvergedError` only when ``strict`` asks it to.

The C++ core (``slamlab/foundations/nls.hpp``) is the same loop as a template over the same
problem concept. Following the "generic in the core, concrete in the bindings" rule, it is
not bound itself: the backends meet at the concrete solvers built on it (bundle adjustment).
"""

from __future__ import annotations

import enum
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, Protocol, TypeVar

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab.foundations.arrays import as_real
from slamlab.foundations.errors import InvalidInputError, NotConvergedError
from slamlab.foundations.solve import Solution

StateT = TypeVar("StateT")

LOSSES = ("trivial", "huber", "cauchy")
"""The robust losses :class:`Loss` accepts."""

METHODS = ("levenberg_marquardt", "gauss_newton")
"""The solver methods :class:`Options` accepts."""


@dataclass(frozen=True)
class Loss:
    """A robust loss ``ρ(s)`` on a block's squared norm ``s``.

    Attributes
    ----------
    kind : {"trivial", "huber", "cauchy"}
        The loss (default: plain least squares).
    scale : float
        The residual norm ``c`` at which the loss starts to discount a block (positive).
    """

    kind: str = "trivial"
    scale: float = 1.0

    def __post_init__(self) -> None:
        """Reject an unknown loss or a non-positive scale."""
        if self.kind not in LOSSES:
            raise InvalidInputError(f"loss: expected one of {LOSSES}, got {self.kind!r}")
        if not (np.isfinite(self.scale) and self.scale > 0.0):
            raise InvalidInputError(f"loss: scale must be positive and finite, got {self.scale}")

    def evaluate(
        self, squared_norms: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Evaluate the loss and its derivative, the IRLS weight.

        Parameters
        ----------
        squared_norms : ndarray of float64, shape (M,)
            The blocks' squared norms ``s_i >= 0``.

        Returns
        -------
        rho : ndarray of float64, shape (M,)
            ``ρ(s_i)``.
        weights : ndarray of float64, shape (M,)
            ``ρ'(s_i)``, each in ``(0, 1]``.

        Examples
        --------
        >>> rho, weight = Loss("huber", 1.0).evaluate(np.array([0.25, 4.0]))
        >>> rho.tolist(), weight.tolist()
        ([0.25, 3.0], [1.0, 0.5])
        """
        s = squared_norms
        c2 = self.scale * self.scale
        if self.kind == "huber":
            norm = np.sqrt(s)
            inlier = s <= c2
            rho = np.where(inlier, s, 2.0 * self.scale * norm - c2)
            weights = np.where(inlier, 1.0, self.scale / np.where(inlier, 1.0, norm))
            return rho, weights
        if self.kind == "cauchy":
            return c2 * np.log1p(s / c2), 1.0 / (1.0 + s / c2)
        return s.copy(), np.ones_like(s)


@dataclass(frozen=True)
class BlockLoss:
    """A loss per block: block i uses ``losses[index[i]]``.

    For problems whose blocks are of different kinds (reprojection errors and depth priors,
    say), each with its own robust loss. Pass it to :func:`solve`, where it replaces
    ``Options.loss``.

    Attributes
    ----------
    losses : tuple of Loss
        The losses.
    index : ndarray of int64, shape (M,)
        Each block's loss, an index into ``losses``.
    """

    losses: tuple[Loss, ...]
    index: NDArray[np.int64]

    def __post_init__(self) -> None:
        """Reject an empty loss list or an index outside it."""
        if not self.losses:
            raise InvalidInputError("block loss: expected at least one loss")
        if self.index.ndim != 1 or (
            self.index.size and (self.index.min() < 0 or self.index.max() >= len(self.losses))
        ):
            raise InvalidInputError("block loss: index must select one of the losses per block")

    def evaluate(
        self, squared_norms: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Evaluate each block's own loss and weight (see :meth:`Loss.evaluate`).

        Parameters
        ----------
        squared_norms : ndarray of float64, shape (M,)
            The blocks' squared norms.

        Returns
        -------
        rho, weights : ndarray of float64, shape (M,)
            ``ρ_i(s_i)`` and ``ρ_i'(s_i)``.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If the number of blocks differs from the index's length.
        """
        if squared_norms.shape != self.index.shape:
            raise InvalidInputError(
                f"block loss: {self.index.shape[0]} blocks indexed, {squared_norms.shape} given"
            )
        rho = np.empty_like(squared_norms)
        weights = np.empty_like(squared_norms)
        for k, loss in enumerate(self.losses):
            selected = self.index == k
            rho[selected], weights[selected] = loss.evaluate(squared_norms[selected])
        return rho, weights


@dataclass(frozen=True)
class Options:
    """Settings for :func:`solve`; the defaults follow Ceres where it has an equivalent.

    Attributes
    ----------
    method : {"levenberg_marquardt", "gauss_newton"}
        The step rule.
    loss : Loss
        The robust loss applied to every block.
    block_loss : BlockLoss or None
        A loss per block, replacing ``loss`` when given.
    max_iterations : int
        Steps tried, accepted or not, before stopping.
    function_tolerance : float
        Stop when an accepted step lowers the cost by less than this fraction of it.
    gradient_tolerance : float
        Stop when the gradient's largest component is below this.
    step_tolerance : float
        Stop when the step's norm (in the tangent space) is below this.
    initial_damping : float
        The first ``μ`` (Ceres' default trust region of 1e4, inverted).
    max_damping : float
        Stop, unconverged, when ``μ`` exceeds this: no step decreases the cost.
    min_diagonal, max_diagonal : float
        The clamp on ``diag(Jᵀ W J)`` before it scales the damping.
    min_relative_decrease : float
        The smallest ratio of actual to predicted decrease for a step to be accepted.
    strict : bool
        Raise :class:`~slamlab.foundations.errors.NotConvergedError` instead of returning
        an unconverged estimate.
    """

    method: str = "levenberg_marquardt"
    loss: Loss = Loss()
    block_loss: BlockLoss | None = None
    max_iterations: int = 50
    function_tolerance: float = 1e-6
    gradient_tolerance: float = 1e-10
    step_tolerance: float = 1e-10
    initial_damping: float = 1e-4
    max_damping: float = 1e32
    min_diagonal: float = 1e-6
    max_diagonal: float = 1e32
    min_relative_decrease: float = 1e-3
    strict: bool = False

    def __post_init__(self) -> None:
        """Reject an unknown method or a setting outside its range."""
        if self.method not in METHODS:
            raise InvalidInputError(f"method: expected one of {METHODS}, got {self.method!r}")
        if self.max_iterations < 1:
            raise InvalidInputError("max_iterations: must be at least 1")
        positive = {
            "initial_damping": self.initial_damping,
            "max_damping": self.max_damping,
            "min_diagonal": self.min_diagonal,
            "max_diagonal": self.max_diagonal,
        }
        non_negative = {
            "function_tolerance": self.function_tolerance,
            "gradient_tolerance": self.gradient_tolerance,
            "step_tolerance": self.step_tolerance,
            "min_relative_decrease": self.min_relative_decrease,
        }
        for name, value in positive.items():
            if not (np.isfinite(value) and value > 0.0):
                raise InvalidInputError(f"{name}: must be positive and finite, got {value}")
        for name, value in non_negative.items():
            if not (np.isfinite(value) and value >= 0.0):
                raise InvalidInputError(f"{name}: must be non-negative and finite, got {value}")
        if self.min_diagonal > self.max_diagonal:
            raise InvalidInputError("min_diagonal: must not exceed max_diagonal")


class Termination(enum.StrEnum):
    """Why :func:`solve` stopped."""

    GRADIENT = "gradient"
    """Converged: the gradient is below ``gradient_tolerance``."""
    FUNCTION = "function"
    """Converged: the last accepted step barely lowered the cost."""
    STEP = "step"
    """Converged: the step is below ``step_tolerance``."""
    MAX_ITERATIONS = "max_iterations"
    """Not converged: ``max_iterations`` steps were tried."""
    DAMPING_LIMIT = "damping_limit"
    """Not converged: Levenberg-Marquardt's damping exceeded ``max_damping``."""
    NO_DECREASE = "no_decrease"
    """Not converged: a Gauss-Newton step failed to lower the cost."""
    SINGULAR = "singular"
    """Not converged: the undamped Gauss-Newton system is not positive definite."""


CONVERGED = frozenset({Termination.GRADIENT, Termination.FUNCTION, Termination.STEP})
"""The terminations that count as converged."""


@dataclass(frozen=True)
class Iteration:
    """One step tried by :func:`solve`.

    Attributes
    ----------
    iteration : int
        The step's index, from 1.
    cost : float
        The cost before the step.
    gradient_norm : float
        The gradient's largest component, before the step.
    step_norm : float
        The step's norm (NaN when the system could not be factored).
    damping : float
        ``μ`` for this step (0 for Gauss-Newton).
    relative_decrease : float
        Actual over predicted cost decrease (NaN when not computed).
    accepted : bool
        Whether the step was taken.
    """

    iteration: int
    cost: float
    gradient_norm: float
    step_norm: float
    damping: float
    relative_decrease: float
    accepted: bool


@dataclass(frozen=True)
class Summary:
    """How a solve went: costs, iteration count, why it stopped, and its trace.

    Satisfies :class:`slamlab.foundations.solve.Summary`.

    Attributes
    ----------
    initial_cost, final_cost : float
        The cost ``½ Σ ρ(s_i)`` at the start and at the returned state.
    iterations : int
        Steps tried, accepted or not.
    termination : Termination
        Why the solve stopped.
    converged : bool
        Whether ``termination`` is one of :data:`CONVERGED`.
    trace : tuple of Iteration
        Every step tried, in order.
    """

    initial_cost: float
    final_cost: float
    iterations: int
    termination: Termination
    converged: bool
    trace: tuple[Iteration, ...]


class LinearSystem(Protocol):
    """A linearisation's normal equations ``(H + diag(d)) δ = -g``; the solver never forms H."""

    @property
    def gradient(self) -> NDArray[np.float64]:
        """The weighted gradient ``g = Jᵀ W r``, shape (n,)."""
        ...

    @property
    def hessian_diagonal(self) -> NDArray[np.float64]:
        """The diagonal of ``H = Jᵀ W J``, shape (n,), which scales the damping."""
        ...

    def solve(self, damping: NDArray[np.float64]) -> NDArray[np.float64] | None:
        """Solve for the step with ``damping`` added to H's diagonal; ``None`` if indefinite."""
        ...


class Problem(Protocol[StateT]):
    """A least-squares problem the solver can drive, on whatever state type it chooses."""

    def squared_norms(self, state: StateT) -> NDArray[np.float64]:
        """Return each residual block's squared norm, shape (M,)."""
        ...

    def linearize(self, state: StateT, weights: NDArray[np.float64]) -> LinearSystem:
        """Linearise at ``state`` with per-block weights (shape (M,)) into normal equations."""
        ...

    def retract(self, state: StateT, delta: NDArray[np.float64]) -> StateT:
        """Apply a tangent-space step to ``state``."""
        ...


def cholesky_solve(
    matrix: NDArray[np.float64], rhs: NDArray[np.float64]
) -> NDArray[np.float64] | None:
    """Solve a symmetric positive-definite system by Cholesky.

    Parameters
    ----------
    matrix : ndarray of float64, shape (n, n)
        The matrix.
    rhs : ndarray of float64, shape (n,)
        The right-hand side.

    Returns
    -------
    ndarray of float64, shape (n,), or None
        The solution, or ``None`` if the factorisation fails (the matrix is not positive
        definite), which a damped solver treats as a rejected step, not an error.

    Examples
    --------
    >>> cholesky_solve(np.array([[4.0, 2.0], [2.0, 3.0]]), np.array([2.0, 1.0])).tolist()
    [0.5, 0.0]
    """
    try:
        lower = np.linalg.cholesky(matrix)
    except np.linalg.LinAlgError:
        return None
    # NumPy has no triangular solver; these solves use the factor's two triangles.
    return np.asarray(np.linalg.solve(lower.T, np.linalg.solve(lower, rhs)))


@dataclass(frozen=True)
class DenseSystem:
    """Normal equations formed densely from a full Jacobian, factored by Cholesky.

    Attributes
    ----------
    hessian : ndarray of float64, shape (n, n)
        ``H = Jᵀ W J``.
    gradient : ndarray of float64, shape (n,)
        ``g = Jᵀ W r``.
    """

    hessian: NDArray[np.float64]
    gradient: NDArray[np.float64]

    @classmethod
    def build(cls, jacobian: ArrayLike, residuals: ArrayLike, weights: ArrayLike) -> DenseSystem:
        """Form the weighted normal equations of M blocks of d rows over n parameters.

        Parameters
        ----------
        jacobian : array_like, shape (M, d, n)
            Each block's Jacobian.
        residuals : array_like, shape (M, d)
            Each block's residual.
        weights : array_like, shape (M,)
            Each block's weight.

        Returns
        -------
        DenseSystem
            The normal equations.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If the shapes disagree or a value is not finite.
        """
        j = as_real(jacobian, "jacobian")
        r = as_real(residuals, "residuals")
        w = as_real(weights, "weights")
        if j.ndim != 3 or r.shape != j.shape[:2] or w.shape != j.shape[:1]:
            raise InvalidInputError(
                "jacobian, residuals, weights: expected shapes (M, d, n), (M, d), (M,); "
                f"got {j.shape}, {r.shape}, {w.shape}"
            )
        rows = j.reshape(-1, j.shape[2])
        row_weights = np.repeat(w, j.shape[1])
        weighted = rows * row_weights[:, None]
        return cls(hessian=weighted.T @ rows, gradient=weighted.T @ r.reshape(-1))

    @property
    def hessian_diagonal(self) -> NDArray[np.float64]:
        """The diagonal of ``H``, shape (n,)."""
        return np.diagonal(self.hessian).copy()

    def solve(self, damping: NDArray[np.float64]) -> NDArray[np.float64] | None:
        """Solve ``(H + diag(damping)) δ = -g`` by Cholesky.

        Parameters
        ----------
        damping : ndarray of float64, shape (n,)
            Added to H's diagonal.

        Returns
        -------
        ndarray of float64, shape (n,), or None
            The step, or ``None`` if the damped system is not positive definite.
        """
        return cholesky_solve(self.hessian + np.diag(damping), -self.gradient)


_DAMPING_GROWTH = 2.0
"""Nielsen's ν: the factor μ first grows by on a rejected step, itself doubled per rejection."""
_DAMPING_MAX_SHRINK = 3.0
"""The most an accepted step divides μ by (Nielsen 1999; Madsen, Nielsen & Tingleff 2004)."""


@dataclass
class _Damping:
    """The Levenberg-Marquardt damping μ and Nielsen's growth factor ν."""

    mu: float
    growth: float = _DAMPING_GROWTH

    def accept(self, ratio: float) -> None:
        """Shrink μ by the gain ratio of an accepted step; reset ν."""
        self.mu *= max(1.0 / _DAMPING_MAX_SHRINK, 1.0 - (2.0 * ratio - 1.0) ** 3)
        self.growth = _DAMPING_GROWTH

    def reject(self) -> None:
        """Grow μ by ν after a rejected step, and double ν."""
        self.mu *= self.growth
        self.growth *= _DAMPING_GROWTH


def _loss(options: Options) -> Loss | BlockLoss:
    """Return the loss in force: the per-block loss if given, else the uniform one."""
    return options.block_loss if options.block_loss is not None else options.loss


@dataclass(frozen=True)
class _Evaluation:
    """The blocks' squared norms at a state, and its cost ``½ Σ ρ(s_i)`` (inf if not finite)."""

    squared_norms: NDArray[np.float64]
    cost: float

    @classmethod
    def of(cls, loss: Loss | BlockLoss, squared_norms: ArrayLike) -> _Evaluation:
        """Evaluate the cost of a set of squared norms."""
        s = np.asarray(squared_norms, dtype=np.float64)
        if not np.all(np.isfinite(s)):
            return cls(s, float("inf"))
        return cls(s, 0.5 * float(np.sum(loss.evaluate(s)[0])))


@dataclass(frozen=True)
class _Point(Generic[StateT]):
    """Where the solver stands: the state, its evaluation, and its linearisation."""

    state: StateT
    evaluation: _Evaluation
    system: LinearSystem

    @classmethod
    def at(
        cls,
        problem: Problem[StateT],
        state: StateT,
        evaluation: _Evaluation,
        loss: Loss | BlockLoss,
    ) -> _Point[StateT]:
        """Linearise ``problem`` at ``state`` with the IRLS weights of its evaluation."""
        weights = loss.evaluate(evaluation.squared_norms)[1]
        return cls(state, evaluation, problem.linearize(state, weights))


@dataclass(frozen=True)
class _Trial(Generic[StateT]):
    """A step tried: its record, and the candidate state with its evaluation (if it had one)."""

    record: Iteration
    candidate: StateT | None = None
    evaluation: _Evaluation | None = None


def _try_step(
    problem: Problem[StateT], point: _Point[StateT], damping: float, number: int, opts: Options
) -> _Trial[StateT] | Termination:
    """Compute, apply and judge one step; or return the convergence test it stops on."""
    system, current = point.system, point.evaluation
    gradient = system.gradient
    gradient_norm = float(np.max(np.abs(gradient), initial=0.0))
    if gradient_norm <= opts.gradient_tolerance:
        return Termination.GRADIENT
    diagonal = damping * np.clip(system.hessian_diagonal, opts.min_diagonal, opts.max_diagonal)
    delta = system.solve(diagonal)
    if delta is None:
        nan = float("nan")
        return _Trial(Iteration(number, current.cost, gradient_norm, nan, damping, nan, False))
    step_norm = float(np.linalg.norm(delta))
    if step_norm <= opts.step_tolerance:
        return Termination.STEP
    candidate = problem.retract(point.state, delta)
    evaluation = _Evaluation.of(_loss(opts), problem.squared_norms(candidate))
    predicted = 0.5 * float(delta @ (diagonal * delta) - gradient @ delta)
    ratio = (current.cost - evaluation.cost) / predicted if predicted > 0.0 else float("nan")
    accepted = bool(
        np.isfinite(evaluation.cost)
        and evaluation.cost < current.cost
        and (opts.method == "gauss_newton" or ratio > opts.min_relative_decrease)
    )
    record = Iteration(number, current.cost, gradient_norm, step_norm, damping, ratio, accepted)
    return _Trial(record, candidate, evaluation)


def solve(
    problem: Problem[StateT], initial: StateT, options: Options | None = None
) -> Solution[StateT, Summary]:
    """Minimise ``½ Σ ρ(|r_i|²)`` from ``initial`` by Levenberg-Marquardt or Gauss-Newton.

    Parameters
    ----------
    problem : Problem
        The residual blocks, their linearisation and the state's retraction.
    initial : StateT
        The starting state.
    options : Options, optional
        Method, loss, tolerances and limits (default: ``Options()``).

    Returns
    -------
    slamlab.foundations.solve.Solution
        The lowest-cost state visited, and the solve's :class:`Summary`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the initial cost is not finite.
    slamlab.foundations.errors.NotConvergedError
        If ``options.strict`` and the solve stopped without converging.
    """
    opts = options if options is not None else Options()
    levenberg = opts.method == "levenberg_marquardt"
    evaluation = _Evaluation.of(_loss(opts), problem.squared_norms(initial))
    if not np.isfinite(evaluation.cost):
        raise InvalidInputError("solve: the initial cost is not finite")
    point = _Point.at(problem, initial, evaluation, _loss(opts))
    damping = _Damping(opts.initial_damping if levenberg else 0.0)
    trace: list[Iteration] = []
    termination = Termination.MAX_ITERATIONS
    while len(trace) < opts.max_iterations:
        trial = _try_step(problem, point, damping.mu, len(trace) + 1, opts)
        if isinstance(trial, Termination):
            termination = trial
            break
        trace.append(trial.record)
        if trial.record.accepted and trial.candidate is not None and trial.evaluation is not None:
            previous = point.evaluation.cost
            if levenberg:
                damping.accept(trial.record.relative_decrease)
            if previous - trial.evaluation.cost <= opts.function_tolerance * previous:
                # The loop ends here, so the new state is not worth linearising.
                point = _Point(trial.candidate, trial.evaluation, point.system)
                termination = Termination.FUNCTION
                break
            point = _Point.at(problem, trial.candidate, trial.evaluation, _loss(opts))
            continue
        if not levenberg:
            singular = trial.candidate is None
            termination = Termination.SINGULAR if singular else Termination.NO_DECREASE
            break
        damping.reject()
        if damping.mu > opts.max_damping:
            termination = Termination.DAMPING_LIMIT
            break
    converged = termination in CONVERGED
    if opts.strict and not converged:
        raise NotConvergedError(f"solve: stopped without converging ({termination})")
    summary = Summary(
        initial_cost=evaluation.cost,
        final_cost=point.evaluation.cost,
        iterations=len(trace),
        termination=termination,
        converged=converged,
        trace=tuple(trace),
    )
    return Solution(state=point.state, summary=summary)


def numeric_jacobian(
    residuals: Callable[[StateT], NDArray[np.float64]],
    state: StateT,
    retract: Callable[[StateT, NDArray[np.float64]], StateT],
    dimension: int,
    step: float = 1e-6,
) -> NDArray[np.float64]:
    """Differentiate residuals with respect to a tangent-space step, by central differences.

    The checker for hand-derived Jacobians: compare it with the analytic one at a few
    states. Its error is about ``step²`` (truncation) plus ``eps / step`` (rounding).

    Parameters
    ----------
    residuals : callable
        Maps a state to a flat residual vector, shape (m,).
    state : StateT
        Where to differentiate.
    retract : callable
        Applies a tangent step to a state, as :meth:`Problem.retract` does.
    dimension : int
        The tangent space's dimension n.
    step : float, optional
        The finite-difference step (default 1e-6).

    Returns
    -------
    ndarray of float64, shape (m, n)
        ``d residuals / d δ`` at ``δ = 0``.

    Examples
    --------
    >>> jac = numeric_jacobian(lambda x: x**2, np.array([3.0]), lambda x, d: x + d, 1)
    >>> bool(abs(jac[0, 0] - 6.0) < 1e-8)
    True
    """
    columns = []
    for k in range(dimension):
        delta = np.zeros(dimension)
        delta[k] = step
        forward = np.asarray(residuals(retract(state, delta)), dtype=np.float64)
        backward = np.asarray(residuals(retract(state, -delta)), dtype=np.float64)
        columns.append((forward - backward) / (2.0 * step))
    return np.stack(columns, axis=-1)
