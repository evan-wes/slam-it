"""Tests for slam_it.foundations.nls: the Python reference loop, against SciPy and on SO(3)."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np
import pytest
from numpy.typing import NDArray
from scipy.optimize import least_squares

from slam_it.foundations import nls, so3, solve
from slam_it.foundations.errors import InvalidInputError, NotConvergedError

TIGHT = nls.Options(
    max_iterations=200, function_tolerance=0.0, gradient_tolerance=1e-12, step_tolerance=1e-14
)
"""Tolerances tight enough that a solve stops on the gradient or the step, not the cost."""


Array = NDArray[np.float64]


class _Blocks:
    """A test problem from its residual blocks and their Jacobians; dense, Euclidean steps."""

    def residuals(self, state: Array) -> Array:
        """Return the residual blocks, shape (M, d)."""
        raise NotImplementedError

    def jacobian(self, state: Array) -> Array:
        """Return the blocks' Jacobians, shape (M, d, n)."""
        raise NotImplementedError

    def squared_norms(self, state: Array) -> Array:
        """Return the blocks' squared norms."""
        return np.asarray(np.sum(self.residuals(state) ** 2, axis=1), dtype=np.float64)

    def linearize(self, state: Array, weights: Array) -> nls.DenseSystem:
        """Return the dense normal equations."""
        return nls.DenseSystem.build(self.jacobian(state), self.residuals(state), weights)

    def retract(self, state: Array, delta: Array) -> Array:
        """Euclidean retraction."""
        return state + delta


@dataclass(frozen=True)
class Rosenbrock(_Blocks):
    """Rosenbrock's function as two residual blocks, plus ``unused`` ignored parameters."""

    unused: int = 0

    def residuals(self, state: Array) -> Array:
        """Return ``(10 (y - x²), 1 - x)`` as two one-row blocks."""
        return np.array([[10.0 * (state[1] - state[0] ** 2)], [1.0 - state[0]]])

    def jacobian(self, state: Array) -> Array:
        """Return the blocks' Jacobians; the unused parameters' columns are zero."""
        jacobian = np.zeros((2, 1, 2 + self.unused))
        jacobian[0, 0, :2] = [-20.0 * state[0], 10.0]
        jacobian[1, 0, 0] = -1.0
        return jacobian


@dataclass(frozen=True)
class NanJacobian(Rosenbrock):
    """Rosenbrock with a NaN in the Jacobian column of a parameter the residuals ignore."""

    unused: int = 1

    def jacobian(self, state: Array) -> Array:
        """Return Rosenbrock's Jacobians with a NaN in the unused column."""
        jacobian = super().jacobian(state)
        jacobian[0, 0, 2] = np.nan
        return jacobian


@dataclass(frozen=True)
class LineFit(_Blocks):
    """Fit ``y = a x + b``, one block per point."""

    xs: Array
    ys: Array

    def residuals(self, state: Array) -> Array:
        """Return ``a x_i + b - y_i`` as one-row blocks."""
        return np.asarray((state[0] * self.xs + state[1] - self.ys)[:, None], dtype=np.float64)

    def jacobian(self, state: Array) -> Array:
        """Return ``[x_i, 1]`` per block (the fit is linear)."""
        del state
        return np.stack([self.xs, np.ones_like(self.xs)], axis=-1)[:, None, :]


@dataclass(frozen=True)
class RotationAverage(_Blocks):
    """Average rotations: blocks ``log(Rᵀ R_i)``, with ``R ⊕ δ = R exp(δ)``."""

    measured: Array = field(repr=False)

    def residuals(self, state: Array) -> Array:
        """Return ``log(Rᵀ R_i)`` for every measurement, shape (M, 3)."""
        return so3.log(state.T @ self.measured)

    def jacobian(self, state: Array) -> Array:
        """Return each block's Jacobian, ``-Jl⁻¹(r_i)``."""
        return -so3.left_jacobian_inverse(self.residuals(state))

    def retract(self, state: Array, delta: Array) -> Array:
        """Right retraction."""
        return state @ so3.exp(delta)


def _line_with_outliers() -> LineFit:
    """Return ``y = 2x + 1`` with small deterministic noise and three gross outliers."""
    i = np.arange(30)
    xs = np.asarray(0.1 * i, dtype=np.float64)
    ys = 2.0 * xs + 1.0 + 0.01 * np.sin(1.7 * i)
    ys[[4, 17, 25]] += [8.0, -6.0, 9.0]
    return LineFit(xs, ys)


def _rotations_with_outliers() -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return a rotation and 12 noisy measurements of it, plus two outliers."""
    rotation = so3.exp(np.array([0.3, -0.2, 0.5]))
    i = np.arange(12.0)
    noise = 0.01 * np.stack([np.sin(i), np.cos(2.0 * i), np.sin(3.0 * i)], axis=-1)
    outliers = np.array([[0.9, 0.0, 0.0], [0.0, -1.1, 0.4]])
    return rotation, rotation @ so3.exp(np.concatenate([noise, outliers]))


def test_losses() -> None:
    """Each loss and its weight, on both sides of the scale."""
    s = np.array([0.0, 0.25, 4.0])
    assert [v.tolist() for v in nls.Loss("huber", 1.0).evaluate(s)] == [
        [0.0, 0.25, 3.0],
        [1.0, 1.0, 0.5],
    ]
    rho, weight = nls.Loss("cauchy", 2.0).evaluate(s)
    np.testing.assert_allclose(rho, 4.0 * np.log1p(s / 4.0), rtol=1e-15)
    np.testing.assert_allclose(weight, [1.0, 1.0 / 1.0625, 0.5], rtol=1e-15)
    assert [v.tolist() for v in nls.Loss().evaluate(s)] == [s.tolist(), [1.0, 1.0, 1.0]]


@pytest.mark.parametrize("kind", nls.LOSSES)
def test_loss_weight_is_its_derivative(kind: str) -> None:
    """The IRLS weight is ρ'(s), by central differences."""
    loss = nls.Loss(kind, 0.7)
    s = np.array([0.1, 0.3, 0.9, 2.5])
    h = 1e-7
    numeric = (loss.evaluate(s + h)[0] - loss.evaluate(s - h)[0]) / (2.0 * h)
    np.testing.assert_allclose(loss.evaluate(s)[1], numeric, rtol=1e-7)


def test_levenberg_marquardt_reaches_rosenbrock_minimum() -> None:
    """From the classic start (-1.2, 1), LM reaches (1, 1), never raising the cost."""
    solution = nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), TIGHT)
    summary = solution.summary
    assert summary.converged
    np.testing.assert_allclose(solution.state, [1.0, 1.0], rtol=0, atol=1e-10)
    assert summary.final_cost < 1e-20
    accepted = [it.cost for it in summary.trace if it.accepted]
    assert accepted == sorted(accepted, reverse=True)
    assert summary.iterations == len(summary.trace)
    assert summary.trace[0].cost == summary.initial_cost


def test_a_stop_ends_the_solve_between_steps() -> None:
    """Asked before each step, a stop ends the solve there: the steps taken kept, not converged."""
    asked: list[bool] = []

    def stop() -> bool:
        asked.append(True)
        return len(asked) > 2

    solution = nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), TIGHT, stop=stop)
    summary = solution.summary
    assert (summary.iterations, summary.termination) == (2, nls.Termination.STOPPED)
    assert not summary.converged
    assert summary.final_cost <= summary.initial_cost
    never = nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), TIGHT, stop=lambda: False)
    assert never.summary == nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), TIGHT).summary


def test_summary_meets_the_shared_contract() -> None:
    """The NLS summary is a :class:`solve.Summary`; mypy checks the assignment."""
    shared: solve.Summary = nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), TIGHT).summary
    assert shared.converged
    assert shared.termination in nls.CONVERGED
    assert shared.iterations > 0


def test_gauss_newton_stops_on_overshoot_and_converges_nearby() -> None:
    """GN's undamped step from (-1.2, 1) raises the cost (12.1 → 1171), so GN stops there."""
    gn = replace(TIGHT, method="gauss_newton")
    stopped = nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), gn)
    assert stopped.summary.termination == nls.Termination.NO_DECREASE
    assert stopped.summary.iterations == 1
    np.testing.assert_array_equal(stopped.state, [-1.2, 1.0])
    near = nls.solve(Rosenbrock(), np.array([0.5, 0.5]), gn)
    assert near.summary.converged
    np.testing.assert_allclose(near.state, [1.0, 1.0], rtol=0, atol=1e-10)


def test_singular_system() -> None:
    """An ignored parameter makes GN's system singular; LM's damping copes and leaves it alone."""
    start = np.array([-1.2, 1.0, 0.5])
    gn = nls.solve(Rosenbrock(unused=1), start, replace(TIGHT, method="gauss_newton"))
    assert gn.summary.termination == nls.Termination.SINGULAR
    assert not gn.summary.converged
    lm = nls.solve(Rosenbrock(unused=1), start, TIGHT)
    assert lm.summary.converged
    np.testing.assert_allclose(lm.state[:2], [1.0, 1.0], rtol=0, atol=1e-10)
    assert lm.state[2] == 0.5


def test_least_squares_matches_exact_solution() -> None:
    """With the trivial loss the line fit is linear, so ``lstsq`` gives the exact optimum."""
    fit = _line_with_outliers()
    ours = nls.solve(fit, np.zeros(2), TIGHT)
    exact = np.linalg.lstsq(np.stack([fit.xs, np.ones_like(fit.xs)], axis=-1), fit.ys, rcond=None)[
        0
    ]
    assert ours.summary.converged
    np.testing.assert_allclose(ours.state, exact, rtol=0, atol=1e-10)


@pytest.mark.parametrize(("kind", "scale"), [("huber", 0.1), ("cauchy", 0.05)])
def test_robust_optimum_matches_scipy(kind: str, scale: float) -> None:
    """The robust optimum zeroes the robust gradient, and matches SciPy's ``least_squares``.

    SciPy's ``trf`` stops about 1e-8 short of the optimum even at tolerances of 1e-15 (on the
    linear case it is 8e-9 from the exact answer, which this solver reaches to 1e-11), so
    it is a cross-check at 1e-7; the gradient is the sharper test. Its floor is set by the
    cost's resolution: once a step changes the cost by less than about ``eps · cost`` the
    acceptance test cannot see it, which for Cauchy here leaves a gradient near 2e-9 and a
    parameter error near 3e-11.
    """
    fit = _line_with_outliers()
    loss = nls.Loss(kind, scale)
    ours = nls.solve(fit, np.zeros(2), replace(TIGHT, loss=loss))
    residuals = fit.residuals(ours.state)[:, 0]
    weights = loss.evaluate(residuals**2)[1]
    gradient = np.stack([fit.xs, np.ones_like(fit.xs)], axis=-1).T @ (weights * residuals)
    scipy = least_squares(
        lambda p: fit.residuals(p)[:, 0],
        np.zeros(2),
        method="trf",
        loss=kind,
        f_scale=scale,
        xtol=1e-15,
        ftol=1e-15,
        gtol=1e-15,
    )
    assert ours.summary.converged
    np.testing.assert_allclose(gradient, 0.0, rtol=0, atol=1e-8)
    np.testing.assert_allclose(ours.state, scipy.x, rtol=0, atol=1e-7)
    assert ours.summary.final_cost <= scipy.cost * (1.0 + 1e-12)


def test_robust_loss_resists_outliers() -> None:
    """Cauchy recovers the line that outliers drag plain least squares away from."""
    fit = _line_with_outliers()
    plain = nls.solve(fit, np.zeros(2), TIGHT)
    robust = nls.solve(fit, np.zeros(2), replace(TIGHT, loss=nls.Loss("cauchy", 0.05)))
    assert np.linalg.norm(plain.state - [2.0, 1.0]) > 0.3
    assert np.linalg.norm(robust.state - [2.0, 1.0]) < 0.02


def test_rotation_averaging_on_so3() -> None:
    """The robust mean of noisy rotations, solved on the manifold, stays a rotation."""
    rotation, measured = _rotations_with_outliers()
    options = replace(TIGHT, loss=nls.Loss("cauchy", 0.05))
    solution = nls.solve(RotationAverage(measured), np.eye(3), options)
    assert solution.summary.converged
    assert np.linalg.norm(so3.log(rotation.T @ solution.state)) < 0.01
    np.testing.assert_allclose(solution.state.T @ solution.state, np.eye(3), rtol=0, atol=1e-12)


def test_rotation_average_jacobian_matches_numeric() -> None:
    """The hand-derived block Jacobian ``-Jl⁻¹(r)`` matches the numeric checker."""
    _, measured = _rotations_with_outliers()
    problem = RotationAverage(measured)
    state = so3.exp(np.array([0.1, 0.2, -0.3]))
    numeric = nls.numeric_jacobian(
        lambda r: problem.residuals(r).reshape(-1), state, problem.retract, 3
    )
    analytic = problem.jacobian(state).reshape(-1, 3)
    np.testing.assert_allclose(analytic, numeric, rtol=0, atol=1e-8)


def test_strict_mode_and_iteration_cap() -> None:
    """At the cap, the best estimate comes back unconverged; strict mode raises instead."""
    capped = nls.solve(Rosenbrock(), np.array([-1.2, 1.0]), replace(TIGHT, max_iterations=2))
    assert capped.summary.termination == nls.Termination.MAX_ITERATIONS
    assert not capped.summary.converged
    assert capped.summary.final_cost <= capped.summary.initial_cost
    with pytest.raises(NotConvergedError, match="max_iterations"):
        nls.solve(
            Rosenbrock(),
            np.array([-1.2, 1.0]),
            replace(TIGHT, max_iterations=2, strict=True),
        )


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"method": "newton"}, "method"),
        ({"max_iterations": 0}, "max_iterations"),
        ({"initial_damping": -1.0}, "initial_damping"),
        ({"gradient_tolerance": float("nan")}, "gradient_tolerance"),
        ({"min_diagonal": 1.0, "max_diagonal": 0.5}, "min_diagonal"),
    ],
)
def test_invalid_options(settings: dict[str, object], message: str) -> None:
    """Every setting outside its range is rejected when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        nls.Options(**settings)  # type: ignore[arg-type]


def test_invalid_loss_and_system() -> None:
    """Bad losses, mismatched system shapes and a non-finite start are rejected."""
    with pytest.raises(InvalidInputError, match="loss"):
        nls.Loss("tukey")
    with pytest.raises(InvalidInputError, match="scale"):
        nls.Loss("huber", 0.0)
    with pytest.raises(InvalidInputError, match="expected shapes"):
        nls.DenseSystem.build(np.zeros((3, 1, 2)), np.zeros((3, 1)), np.ones(2))
    with pytest.raises(InvalidInputError, match="initial cost"):
        nls.solve(Rosenbrock(), np.array([np.nan, 1.0]))


def test_block_loss() -> None:
    """A per-block loss evaluates each block's own loss.

    Naming one loss everywhere reproduces the uniform solve exactly.
    """
    s = np.array([0.25, 4.0, 4.0])
    block = nls.BlockLoss((nls.Loss(), nls.Loss("huber", 1.0)), np.array([0, 1, 0]))
    rho, weight = block.evaluate(s)
    assert rho.tolist() == [0.25, 3.0, 4.0]
    assert weight.tolist() == [1.0, 0.5, 1.0]
    fit = _line_with_outliers()
    cauchy = nls.Loss("cauchy", 0.05)
    uniform = nls.solve(fit, np.zeros(2), replace(TIGHT, loss=cauchy))
    everywhere = nls.BlockLoss((nls.Loss(), cauchy), np.ones(fit.xs.size, dtype=np.int64))
    per_block = nls.solve(fit, np.zeros(2), replace(TIGHT, block_loss=everywhere))
    np.testing.assert_array_equal(per_block.state, uniform.state)
    with pytest.raises(InvalidInputError, match="index"):
        nls.BlockLoss((nls.Loss(),), np.array([0, 1]))
    with pytest.raises(InvalidInputError, match="blocks"):
        block.evaluate(np.zeros(2))


@pytest.mark.parametrize("method", ["levenberg_marquardt", "gauss_newton"])
def test_nan_gradient_does_not_converge(method: str) -> None:
    """A NaN Jacobian never reads as convergence, even at the minimum (panel L1).

    The reference refuses it outright when the dense system is built; the C++ and Rust cores,
    whose systems can also come from elsewhere (BA's Schur system), never count a NaN
    gradient as converged, which their native tests check.
    """
    with pytest.raises(InvalidInputError, match="jacobian"):
        nls.solve(NanJacobian(), np.array([1.0, 1.0, 0.0]), replace(TIGHT, method=method))
