"""RANSAC: robust model fitting by random sampling, as LO-MSAC.

Given N data (correspondences) of which an unknown fraction are outliers, RANSAC repeatedly
fits a model to a random **minimal sample** and keeps the model that best explains the data.
This module is the generic loop. The models (the essential matrix, a homography, ...) are
:class:`Estimator` implementations that know how to fit a sample and measure each datum's
error; the loop knows nothing else about them.

**Scoring: MSAC** (Torr and Zisserman, "MLESAC", CVIU 2000). A model's cost is
``Σ_i min(r_i², θ²)`` over all data, with ``r_i`` the datum's residual and ``θ`` the inlier
threshold. Plain RANSAC counts inliers, so every inlier scores the same however well it fits;
MSAC's truncated quadratic prefers the model that fits its inliers tightly, at no extra cost.

**Adaptive stopping.** If a fraction ``w`` of the data are inliers, a random minimal sample of
``s`` is all inliers with probability ``wˢ``. After ``k`` samples, the chance that none was is
``(1 - wˢ)ᵏ``. With ``w`` taken from the best model so far, sampling stops once that falls
below ``1 - confidence``: after ``k = log(1 - p) / log(1 - wˢ)`` samples (Fischler and Bolles,
1981). Reaching ``confidence`` is the solver's convergence; reaching ``max_iterations`` first
is a normal, unconverged result (the shared solver contract, :mod:`slamlab.foundations.solve`).

**Local optimisation: LO-RANSAC** (Chum, Matas and Kittler, DAGM 2003; Lebeda, Matas and
Chum, "Fixing the locally optimized RANSAC", BMVC 2012). A model fitted to an all-inlier
minimal sample is still noisy, because a minimal sample is the least data that determines it.
So whenever a new best model is found, it is refined:

1. **Iterative least squares.** Refit the model to its inliers with the estimator's
   non-minimal (least-squares) solver, ``lo_steps`` times, the inlier threshold shrinking from
   ``lo_threshold_multiplier · θ`` to ``θ``. The wide first threshold lets the fit pull in
   inliers the noisy minimal model just missed.
2. **Inner RANSAC.** ``lo_inner_iterations`` times, fit the non-minimal solver to a random
   subset of the current inliers (``lo_sample_factor · s`` of them, at most half), and run
   step 1 from it. Sampling non-minimal subsets of inliers escapes a poor local optimum that
   step 1 alone would stay in.

Local optimisation runs only a few times in a whole run (a new best is found rarely), so its
cost is small, and it raises the inlier count enough that the adaptive bound stops sampling
sooner. Its candidates are kept only when they lower the MSAC cost.

**Degenerate samples.** An estimator returns no model for a sample it cannot fit (collinear
points, a singular system); that sample is counted, not fatal. If no sample in the whole run
gives a model, there is no estimate to return, and :func:`estimate` raises
:class:`~slamlab.foundations.errors.DegenerateError`.

**Random numbers.** Samples come from :class:`slamlab.foundations.rng.Pcg32`, which every
backend implements identically, so a seed draws the same samples in Python and C++.

The C++ core (``slamlab/foundations/ransac.hpp``) is the same loop as a template over the same
estimator concept. Following "generic in the core, concrete in the bindings", it is not bound
itself: the backends meet at the concrete estimators built on it (two-view geometry).
"""

from __future__ import annotations

import enum
import math
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, replace
from typing import Any, Generic, Protocol, TypeVar

import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.foundations import nls, solve
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from slamlab.foundations.rng import Pcg32
from slamlab.foundations.solve import Solution

ModelT = TypeVar("ModelT")
SummaryT = TypeVar("SummaryT", bound=solve.Summary)


class Estimator(Protocol[ModelT]):
    """A model RANSAC can fit: its minimal and least-squares solvers and its residuals."""

    @property
    def sample_size(self) -> int:
        """The minimal sample size ``s``."""
        ...

    @property
    def size(self) -> int:
        """The number of data N."""
        ...

    def fit_minimal(self, sample: NDArray[np.int64]) -> list[ModelT]:
        """Fit a minimal sample; return every model it gives (none if degenerate)."""
        ...

    def fit_nonminimal(self, subset: NDArray[np.int64]) -> list[ModelT]:
        """Fit at least ``s`` data by least squares; return the models (none if degenerate)."""
        ...

    def residuals(self, model: ModelT) -> NDArray[np.float64]:
        """Return every datum's squared residual under `model`, shape (N,)."""
        ...


class Termination(enum.StrEnum):
    """Why :func:`estimate` stopped."""

    CONFIDENCE = "confidence"
    """Converged: enough samples were drawn to reach the target confidence."""
    MAX_ITERATIONS = "max_iterations"
    """Not converged: ``max_iterations`` samples were drawn first."""


CONVERGED = frozenset({Termination.CONFIDENCE})
"""The terminations that count as converged."""


@dataclass(frozen=True)
class Options:
    """RANSAC's settings.

    Attributes
    ----------
    threshold : float
        The inlier threshold ``θ``, in the estimator's residual units (not squared).
    confidence : float
        The target probability of having drawn one all-inlier sample, in (0, 1).
    max_iterations : int
        The most samples to draw.
    seed : int
        The random generator's seed.
    stream : int
        The random generator's stream, so two runs from one seed can be independent.
    local_optimization : bool
        Refine each new best model (LO-RANSAC).
    lo_steps : int
        Iterative least-squares refits per refinement.
    lo_threshold_multiplier : float
        The first refit's threshold, as a multiple of ``θ`` (at least 1).
    lo_inner_iterations : int
        Non-minimal inner samples per refinement.
    lo_sample_factor : int
        The inner sample size, as a multiple of ``s`` (capped at half the inliers).
    """

    threshold: float
    confidence: float = 0.999
    max_iterations: int = 10000
    seed: int = 0
    stream: int = 0
    local_optimization: bool = True
    lo_steps: int = 4
    lo_threshold_multiplier: float = 3.0
    lo_inner_iterations: int = 10
    lo_sample_factor: int = 3

    def __post_init__(self) -> None:
        """Check the settings.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting is out of range.
        """
        if not (math.isfinite(self.threshold) and self.threshold > 0.0):
            raise InvalidInputError(f"threshold: must be positive and finite, got {self.threshold}")
        if not 0.0 < self.confidence < 1.0:
            raise InvalidInputError(f"confidence: must be in (0, 1), got {self.confidence}")
        for name in ("max_iterations", "lo_steps", "lo_sample_factor"):
            if getattr(self, name) < 1:
                raise InvalidInputError(f"{name}: must be at least 1, got {getattr(self, name)}")
        if self.lo_inner_iterations < 0:
            raise InvalidInputError(
                f"lo_inner_iterations: must be non-negative, got {self.lo_inner_iterations}"
            )
        multiplier = self.lo_threshold_multiplier
        if not (math.isfinite(multiplier) and multiplier >= 1.0):
            raise InvalidInputError(
                f"lo_threshold_multiplier: must be at least 1, got {multiplier}"
            )


@dataclass(frozen=True)
class Consensus(Generic[ModelT]):
    """The best model and the data it explains.

    Attributes
    ----------
    model : ModelT
        The model with the lowest MSAC cost.
    inliers : ndarray of bool, shape (N,)
        The data whose residual is below the threshold.
    """

    model: ModelT
    inliers: NDArray[np.bool_]


@dataclass(frozen=True)
class Summary:
    """How a RANSAC run went. Satisfies :class:`slamlab.foundations.solve.Summary`.

    Attributes
    ----------
    iterations : int
        Minimal samples drawn.
    termination : Termination
        Why sampling stopped.
    converged : bool
        Whether ``termination`` is one of :data:`CONVERGED`.
    degenerate_samples : int
        Samples that gave no model.
    local_optimizations : int
        Refinements run (one per new best model).
    inlier_count : int
        The best model's inliers.
    score : float
        The best model's MSAC cost, ``Σ min(r², θ²)``.
    """

    iterations: int
    termination: Termination
    converged: bool
    degenerate_samples: int
    local_optimizations: int
    inlier_count: int
    score: float


def required_iterations(inlier_ratio: float, sample_size: int, confidence: float) -> float:
    """Return the samples needed to draw one all-inlier sample with `confidence`.

    ``log(1 - p) / log(1 - wˢ)``, with ``log1p`` for accuracy when ``wˢ`` is small. Every
    backend evaluates this exact expression, so they stop on the same iteration.

    Parameters
    ----------
    inlier_ratio : float
        The inlier fraction ``w``, in [0, 1].
    sample_size : int
        The minimal sample size ``s``.
    confidence : float
        The target probability ``p``, in (0, 1).

    Returns
    -------
    float
        The bound: 0 when every datum is an inlier, infinity when none is.
    """
    all_inliers = inlier_ratio**sample_size
    if all_inliers >= 1.0:
        return 0.0
    if all_inliers <= 0.0:
        return math.inf
    return math.log1p(-confidence) / math.log1p(-all_inliers)


@dataclass
class _Best(Generic[ModelT]):
    """A scored model: its MSAC cost and its squared residuals."""

    model: ModelT
    score: float
    residuals: NDArray[np.float64] = field(repr=False)


def _score(residuals: NDArray[np.float64], threshold2: float) -> float:
    """Return the MSAC cost ``Σ min(r², θ²)``."""
    return float(np.sum(np.minimum(residuals, threshold2)))


def _best_of(
    estimator: Estimator[ModelT], models: list[ModelT], threshold2: float
) -> _Best[ModelT] | None:
    """Score each model; return the lowest-cost one (the first on a tie), or None."""
    best: _Best[ModelT] | None = None
    for model in models:
        residuals = estimator.residuals(model)
        candidate = _Best(model, _score(residuals, threshold2), residuals)
        if best is None or candidate.score < best.score:
            best = candidate
    return best


def _least_squares(
    estimator: Estimator[ModelT], start: _Best[ModelT], options: Options
) -> _Best[ModelT]:
    """Refit to the inliers ``lo_steps`` times, the threshold shrinking to ``θ``."""
    threshold2 = options.threshold**2
    best = current = start
    for k in range(options.lo_steps):
        span = options.lo_threshold_multiplier - 1.0
        multiplier = options.lo_threshold_multiplier - (
            span * k / (options.lo_steps - 1) if options.lo_steps > 1 else span
        )
        subset = np.flatnonzero(current.residuals < (multiplier * options.threshold) ** 2)
        if subset.size < estimator.sample_size:
            break
        fitted = _best_of(estimator, estimator.fit_nonminimal(subset), threshold2)
        if fitted is None:
            break
        current = fitted
        if current.score < best.score:
            best = current
    return best


def _local_optimization(
    estimator: Estimator[ModelT], start: _Best[ModelT], rng: Pcg32, options: Options
) -> _Best[ModelT]:
    """Refine a new best model: iterative least squares, then an inner RANSAC."""
    threshold2 = options.threshold**2
    best = _least_squares(estimator, start, options)
    for _ in range(options.lo_inner_iterations):
        inliers = np.flatnonzero(best.residuals < threshold2)
        size = min(options.lo_sample_factor * estimator.sample_size, inliers.size // 2)
        if size < estimator.sample_size:
            break
        subset = inliers[rng.sample(int(inliers.size), size)]
        fitted = _best_of(estimator, estimator.fit_nonminimal(subset), threshold2)
        if fitted is None:
            continue
        refined = _least_squares(estimator, fitted, options)
        if refined.score < best.score:
            best = refined
    return best


def estimate(
    estimator: Estimator[ModelT], options: Options
) -> Solution[Consensus[ModelT], Summary]:
    """Fit a model robustly by LO-MSAC.

    Parameters
    ----------
    estimator : Estimator
        The model's solvers and residuals, over N data.
    options : Options
        Threshold, confidence, limits, seed and local-optimisation settings.

    Returns
    -------
    slamlab.foundations.solve.Solution
        The best model and its inliers, and the run's :class:`Summary`.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If there are fewer data than a minimal sample.
    slamlab.foundations.errors.DegenerateError
        If no sample gave a model.
    """
    n, s = estimator.size, estimator.sample_size
    if n < s:
        raise InvalidInputError(f"ransac: {n} data, fewer than a minimal sample of {s}")
    threshold2 = options.threshold**2
    rng = Pcg32(options.seed, options.stream)
    best: _Best[ModelT] | None = None
    bound = math.inf
    iterations = degenerate = local = 0
    while iterations < options.max_iterations and iterations < bound:
        iterations += 1
        sample = np.asarray(rng.sample(n, s), dtype=np.int64)
        fitted = _best_of(estimator, estimator.fit_minimal(sample), threshold2)
        if fitted is None:
            degenerate += 1
            continue
        if best is not None and not fitted.score < best.score:
            continue
        if options.local_optimization:
            fitted = _local_optimization(estimator, fitted, rng, options)
            local += 1
        best = fitted
        ratio = int(np.count_nonzero(best.residuals < threshold2)) / n
        bound = required_iterations(ratio, s, options.confidence)
    if best is None:
        raise DegenerateError(f"ransac: none of {iterations} samples gave a model")
    inliers = best.residuals < threshold2
    termination = Termination.CONFIDENCE if iterations >= bound else Termination.MAX_ITERATIONS
    summary = Summary(
        iterations=iterations,
        termination=termination,
        converged=termination in CONVERGED,
        degenerate_samples=degenerate,
        local_optimizations=local,
        inlier_count=int(np.count_nonzero(inliers)),
        score=best.score,
    )
    return Solution(state=Consensus(model=best.model, inliers=inliers), summary=summary)


def summary_from_backend(raw: dict[str, Any], codes: backends.Codes) -> Summary:
    """Build a summary from the scalars a compiled backend returned.

    Parameters
    ----------
    raw : dict
        The :class:`Summary` fields (``converged`` aside) as scalars, with ``termination`` as
        an index into ``codes["ransac_termination"]``.
    codes : dict of str to list of str
        The backend's enum names in code order (its ``codes()``).

    Returns
    -------
    Summary
        The summary.
    """
    termination = Termination(codes["ransac_termination"][int(raw["termination"])])
    return Summary(
        iterations=int(raw["iterations"]),
        termination=termination,
        converged=termination in CONVERGED,
        degenerate_samples=int(raw["degenerate_samples"]),
        local_optimizations=int(raw["local_optimizations"]),
        inlier_count=int(raw["inlier_count"]),
        score=float(raw["score"]),
    )


@dataclass(frozen=True)
class RefinedSummary:
    """How a RANSAC run and the least-squares refinement of its model went.

    Nests both solvers' summaries, as BA's summary nests its least-squares solve, and
    satisfies :class:`slamlab.foundations.solve.Summary` through the RANSAC run's fields (the
    design doc's rule for a summary that nests others).

    Attributes
    ----------
    run : Summary
        The RANSAC run; its ``inlier_count`` and ``score`` describe the returned model.
    refinement : nls.Summary or None
        The refinement's solve, if it ran (refinement on, and enough inliers).
    refined : bool
        Whether the refined model was kept: it lowered the MSAC cost.
    """

    run: Summary
    refinement: nls.Summary | None
    refined: bool

    @classmethod
    def unrefined(cls, run: Summary) -> RefinedSummary:
        """Return the summary of a run whose refinement did not run.

        Parameters
        ----------
        run : Summary
            The RANSAC run.

        Returns
        -------
        RefinedSummary
            `run`, no refinement, not refined.
        """
        return cls(run, refinement=None, refined=False)

    @property
    def iterations(self) -> int:
        """Minimal samples drawn."""
        return self.run.iterations

    @property
    def termination(self) -> Termination:
        """Why sampling stopped."""
        return self.run.termination

    @property
    def converged(self) -> bool:
        """Whether sampling reached its target confidence."""
        return self.run.converged


def refine_consensus(
    estimator: Estimator[ModelT],
    found: Solution[Consensus[ModelT], Summary],
    refine: Callable[[ModelT, NDArray[np.bool_]], Solution[ModelT, nls.Summary]],
    *,
    threshold: float,
    minimum: int,
) -> Solution[Consensus[ModelT], RefinedSummary]:
    """Refine RANSAC's model on its inliers, keeping it only if the MSAC cost drops.

    The refinement minimises its own (robust, geometric) cost, not MSAC's truncated one, so a
    refined model can score worse; it is then discarded, and the summary says so.

    Parameters
    ----------
    estimator : Estimator
        The estimator RANSAC ran with; its residuals rescore the refined model.
    found : slamlab.foundations.solve.Solution
        RANSAC's result.
    refine : callable
        Refines a model on the data an inlier mask selects.
    threshold : float
        RANSAC's inlier threshold ``θ``.
    minimum : int
        The fewest inliers the refinement needs; with fewer it does not run.

    Returns
    -------
    slamlab.foundations.solve.Solution
        The kept model, its inliers and the :class:`RefinedSummary`, whose run's
        ``inlier_count`` and ``score`` describe the kept model.
    """
    unrefined = RefinedSummary.unrefined(found.summary)
    inliers = found.state.inliers
    if np.count_nonzero(inliers) < minimum:
        return Solution(state=found.state, summary=unrefined)
    solved = refine(found.state.model, inliers)
    residuals = estimator.residuals(solved.state)
    score = _score(residuals, threshold**2)
    if not score < found.summary.score:
        return Solution(state=found.state, summary=replace(unrefined, refinement=solved.summary))
    kept = residuals < threshold**2
    run = replace(found.summary, inlier_count=int(np.count_nonzero(kept)), score=score)
    return Solution(
        state=Consensus(model=solved.state, inliers=kept),
        summary=RefinedSummary(run, refinement=solved.summary, refined=True),
    )


def refined_summary_from_backend(raw: dict[str, Any], codes: backends.Codes) -> RefinedSummary:
    """Build a refined run's summary from the dict a compiled backend returned.

    Parameters
    ----------
    raw : dict
        ``run`` (see :func:`summary_from_backend`), ``refinement`` (see
        :func:`slamlab.foundations.nls.summary_from_backend`, or None) and ``refined``.
    codes : dict of str to list of str
        The backend's enum names: ``ransac_termination`` and ``nls_termination``.

    Returns
    -------
    RefinedSummary
        The summary.
    """
    refinement = raw["refinement"]
    return RefinedSummary(
        run=summary_from_backend(raw["run"], codes),
        refinement=None if refinement is None else nls.summary_from_backend(refinement, codes),
        refined=bool(raw["refined"]),
    )


def from_backend(
    raw: dict[str, Any],
    codes: backends.Codes,
    summary: Callable[[dict[str, Any], backends.Codes], SummaryT],
) -> Solution[Consensus[NDArray[np.float64]], SummaryT]:
    """Build a solution from a compiled backend's RANSAC result.

    Parameters
    ----------
    raw : dict
        ``model`` and ``inliers`` arrays, and ``summary``, the dict `summary` reads.
    codes : dict of str to list of str
        The backend's enum names in code order (its ``codes()``).
    summary : callable
        Decodes the summary dict: :func:`summary_from_backend`, or
        :func:`refined_summary_from_backend` for a refined run.

    Returns
    -------
    slamlab.foundations.solve.Solution
        The model, its inliers and the summary.
    """
    consensus = Consensus(
        model=np.asarray(raw["model"], dtype=np.float64),
        inliers=np.asarray(raw["inliers"], dtype=np.bool_),
    )
    return Solution(state=consensus, summary=summary(raw["summary"], codes))


def decoder(
    summary: Callable[[dict[str, Any], backends.Codes], SummaryT],
) -> Callable[[Any, backends.Codes], Solution[Consensus[NDArray[np.float64]], SummaryT]]:
    """Return a :func:`slamlab.backends.call` decoder for an array-model RANSAC result.

    Parameters
    ----------
    summary : callable
        Decodes the summary dict (see :func:`from_backend`).

    Returns
    -------
    callable
        ``(raw, codes) -> Solution``.
    """
    return lambda raw, codes: from_backend(raw, codes, summary)


def run(
    estimator: Estimator[NDArray[np.float64]],
    arrays: dict[str, NDArray[np.float64]],
    options: Options,
    *,
    backend: str,
    target: tuple[str, str],
) -> Solution[Consensus[NDArray[np.float64]], Summary]:
    """Run an array-valued RANSAC with the reference estimator or a compiled backend.

    Parameters
    ----------
    estimator : Estimator
        The reference estimator, used when `backend` is ``"python"``.
    arrays : dict of str to ndarray
        The validated data, passed to the compiled function by keyword.
    options : Options
        The settings, passed to the compiled function as a dict of its fields.
    backend : {"python", "cpp", "rust"}
        Which implementation runs.
    target : tuple of str
        The calling module's ``__name__`` and the compiled function's name.

    Returns
    -------
    slamlab.foundations.solve.Solution
        The model, its inliers and the run's summary.
    """
    module_name, function = target
    return backends.call(
        backend,
        module_name,
        function,
        reference=lambda: estimate(estimator, options),
        arguments={**arrays, "options": asdict(options)},
        decode=decoder(summary_from_backend),
    )
