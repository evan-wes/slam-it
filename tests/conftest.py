"""Shared test helpers: backend selection that cannot silently lose coverage, and fixtures."""

from __future__ import annotations

import json
import os
from collections.abc import Callable
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
import pytest
from numpy.typing import ArrayLike, NDArray

from slam_it import backends
from slam_it.foundations.errors import InvalidInputError

FIXTURES = Path(__file__).parent / "fixtures"

REQUIRED_BACKENDS: frozenset[str] = frozenset(
    name for name in os.environ.get("SLAM_IT_REQUIRE_BACKENDS", "").split(",") if name
)
"""Backends that must be present, from ``SLAM_IT_REQUIRE_BACKENDS`` (comma-separated).

``scripts/check.sh`` (and so CI) sets it to every backend. Then a backend that is not
built, or fails to load, fails the run instead of skipping its parity tests.
"""


def require_backend(name: str) -> str:
    """Return `name` if the backend is usable; otherwise skip, or fail if it is required.

    Every test that needs a particular backend goes through this function, so no test
    can skip a required backend.

    Parameters
    ----------
    name : str
        The backend the test needs.

    Returns
    -------
    str
        `name`, for use as the test's ``backend=`` argument.
    """
    if name not in backends.available():
        message = f"{name} backend not built"
        if name in REQUIRED_BACKENDS:
            pytest.fail(f"{message}, but SLAM_IT_REQUIRE_BACKENDS requires it")
        pytest.skip(message)
    return name


def require_function(name: str, module_name: str, function: str) -> str:
    """Return backend `name` if it implements `function`; otherwise skip or fail.

    A backend that is not built skips, unless it is required (:func:`require_backend`). A
    built one missing the module or the function fails the test: every backend is lock-step.

    Parameters
    ----------
    name : str
        The backend the test needs.
    module_name : str
        The slam_it module, as its ``__name__``.
    function : str
        The function the test calls.

    Returns
    -------
    str
        `name`, for use as the test's ``backend=`` argument.
    """
    require_backend(name)
    if name == "python":
        return name
    try:
        backends.implementation(name, module_name)
    except NotImplementedError:
        pytest.fail(f"{name} does not implement {module_name}, but it is a lock-step backend")
    try:
        backends.function(name, module_name, function)
    except NotImplementedError:
        pytest.fail(f"{name} implements {module_name} without {function}")
    return name


def load_fixture(relative: str) -> dict[str, Any]:
    """Load a JSON parity fixture.

    Parameters
    ----------
    relative : str
        Path under ``tests/fixtures/``, e.g. ``"foundations/so3_hat.json"``.

    Returns
    -------
    dict
        The fixture: ``description``, ``tolerance`` (the policy, see
        :func:`assert_matches_fixture`) and ``cases``.
    """
    fixture: dict[str, Any] = json.loads((FIXTURES / relative).read_text())
    return fixture


def assert_matches_fixture(
    got: ArrayLike | dict[str, Any], expected: ArrayLike | dict[str, Any], tolerance: dict[str, Any]
) -> None:
    """Compare a result with a fixture's expected value under the fixture's policy.

    Parameters
    ----------
    got : array_like or dict
        The value a backend produced: numbers, or (under an exact policy only) a dict of
        named codes and counts, which may hold strings.
    expected : array_like or dict
        The fixture's expected value.
    tolerance : dict
        The fixture's tolerance policy: ``comparison`` is ``"exact"`` (values and the
        sign of every zero must match; ``rtol`` and ``atol`` must be 0) or ``"close"``
        (``rtol`` and ``atol`` are applied as given, never numpy's defaults). With
        ``double_cover`` set, `got` is compared up to sign (``q`` and ``-q`` are the same
        rotation), aligning its sign with `expected` first.

    Raises
    ------
    AssertionError
        If the values differ under the policy, or the policy is malformed.
    """
    if isinstance(expected, dict):
        assert tolerance["comparison"] == "exact", f"named codes need an exact policy: {tolerance}"
        assert got == expected
        return
    got_arr = np.asarray(got, dtype=np.float64)
    expected_arr = np.asarray(expected, dtype=np.float64)
    if tolerance.get("double_cover") and float(np.sum(got_arr * expected_arr)) < 0.0:
        got_arr = -got_arr
    comparison = tolerance["comparison"]
    rtol, atol = float(tolerance["rtol"]), float(tolerance["atol"])
    if comparison == "exact":
        assert rtol == 0.0, f"an exact policy cannot set rtol: {tolerance}"
        assert atol == 0.0, f"an exact policy cannot set atol: {tolerance}"
        np.testing.assert_array_equal(got_arr, expected_arr)
        np.testing.assert_array_equal(
            np.signbit(got_arr), np.signbit(expected_arr), err_msg="signs of zeros differ"
        )
    elif comparison == "close":
        np.testing.assert_allclose(got_arr, expected_arr, rtol=rtol, atol=atol)
    else:
        raise AssertionError(f"unknown comparison {comparison!r} in tolerance policy")


def parity_params(fixture: dict[str, Any]) -> list[Any]:
    """List every ``(function, case)`` pair in a parity fixture as named pytest params.

    Parameters
    ----------
    fixture : dict
        A fixture loaded by :func:`load_fixture`, with a ``functions`` mapping.

    Returns
    -------
    list of pytest.param
        One per case, with id ``"<function>:<case name>"``.
    """
    return [
        pytest.param(name, case, id=f"{name}:{case['name']}")
        for name, spec in fixture["functions"].items()
        for case in spec["cases"]
    ]


def protocol_members(module: ModuleType) -> set[str]:
    """Return the function names of a module's ``Implementation`` Protocol.

    Parameters
    ----------
    module : module
        A SLAM It! algorithm module.

    Returns
    -------
    set of str
        The Protocol's public members.
    """
    return {m for m in vars(module.Implementation) if not m.startswith("_")}


def check_rejects(
    module: ModuleType, function: str, args: tuple[object, ...], message: str, backend: str
) -> None:
    """Check that a function rejects bad input with InvalidInputError matching `message`.

    Parameters
    ----------
    module : module
        The slam_it module under test.
    function : str
        The function's name.
    args : tuple
        The positional arguments, at least one of them invalid.
    message : str
        A regex the error message must match.
    backend : str
        The backend to request (Python validates before any backend runs).
    """
    with pytest.raises(InvalidInputError, match=message):
        getattr(module, function)(*args, backend=backend)


def numeric_right_jacobian(
    x: NDArray[np.float64],
    exp: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    log: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    between: Callable[[NDArray[np.float64], NDArray[np.float64]], NDArray[np.float64]],
    step: float = 1e-6,
) -> NDArray[np.float64]:
    """Estimate a Lie group's right Jacobian at `x` by central differences.

    Uses ``exp(x + δ) ≈ exp(x) exp(Jr(x) δ)``: column k is
    ``(log(exp(x)⁻¹ exp(x + h e_k)) - log(exp(x)⁻¹ exp(x - h e_k))) / 2h``.

    Parameters
    ----------
    x : ndarray, shape (n,)
        The tangent point.
    exp, log : callable
        The group's exponential and logarithm.
    between : callable
        ``between(a, b) = a⁻¹ b`` for two group elements.
    step : float, optional
        The difference step h (truncation and rounding both ~1e-10 at 1e-6).

    Returns
    -------
    ndarray of float64, shape (n, n)
        The estimated right Jacobian.
    """
    base = exp(x)
    n = x.shape[0]
    jacobian = np.empty((n, n))
    for k in range(n):
        delta = np.zeros(n)
        delta[k] = step
        plus = log(between(base, exp(x + delta)))
        minus = log(between(base, exp(x - delta)))
        jacobian[:, k] = (plus - minus) / (2.0 * step)
    return jacobian


def check_parity(
    module: ModuleType, fixture: dict[str, Any], function: str, case: dict[str, Any], backend: str
) -> None:
    """Run one fixture case on one backend and compare under the function's policy.

    The case's ``input`` is the single argument, or -- when the function's spec sets
    ``unpack`` -- the list of positional arguments. A function returning several arrays is
    compared on their concatenation along the last axis.

    Parameters
    ----------
    module : module
        The slam_it module under test.
    fixture : dict
        The loaded fixture.
    function : str
        The function's name in `module`.
    case : dict
        One case of that function.
    backend : str
        The backend to run (skipped or failed via :func:`require_function` if missing).
    """
    spec = fixture["functions"][function]
    chosen = require_function(backend, module.__name__, function)
    args = case["input"] if spec.get("unpack") else [case["input"]]
    got = getattr(module, function)(*args, backend=chosen)
    if isinstance(got, tuple):  # several arrays: compared side by side, as the fixture stores
        got = np.concatenate(got, axis=-1)
    assert got.dtype == np.float64
    assert_matches_fixture(got, case["expected"], spec["tolerance"])


def random_rotation_vectors(
    n: int, seed: int = 0, max_angle: float = 0.999 * np.pi
) -> NDArray[np.float64]:
    """Draw random rotation vectors: uniform axes, angles uniform in [0, `max_angle`).

    Parameters
    ----------
    n : int
        How many vectors.
    seed : int, optional
        The generator seed (tests are reproducible).
    max_angle : float, optional
        The largest angle; just below pi by default, where ``log`` is unambiguous.

    Returns
    -------
    ndarray of float64, shape (n, 3)
        The rotation vectors.
    """
    rng = np.random.default_rng(seed)
    axes = rng.normal(size=(n, 3))
    axes /= np.linalg.norm(axes, axis=-1, keepdims=True)
    return np.asarray(axes * rng.uniform(0.0, max_angle, size=(n, 1)), dtype=np.float64)
