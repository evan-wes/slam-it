"""The error contract shared by every rung, in Python.

Every SLAM It! function fails with one of these exceptions. The C++ core
(``slam_it::foundations::ErrorKind``, returned through ``std::expected``) and the Rust core
(``slam_it_foundations::error::ErrorKind``, returned through ``Result``) use the same three
kinds, and the bindings raise the exception of the matching kind, so a degenerate input
fails the same way in all three languages.

All three are :class:`ValueError` subclasses: each says the input could not produce the
requested result.
"""


class SlamItError(ValueError):
    """Base class of every error a SLAM It! function raises on purpose."""


class InvalidInputError(SlamItError):
    """The input violates the function's contract: shape, dtype, range or finiteness."""


class DegenerateError(SlamItError):
    """The input is valid but geometrically degenerate, so the result is not determined.

    For example pure rotation (zero baseline), collinear points, or too few
    correspondences.
    """


class NotConvergedError(SlamItError):
    """An iterative method failed to converge where that is an error.

    Either the caller requested strict mode and the convergence criteria were not met, or
    the iteration diverged with no usable estimate. Reaching the iteration cap is otherwise
    not an error: iterative solvers return their best estimate with a solve summary.
    """
