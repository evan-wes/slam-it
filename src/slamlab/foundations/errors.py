"""The error contract shared by every rung, in Python.

Every slamlab function fails with one of these exceptions. The C++ core
(``slamlab::foundations::ErrorKind``, returned through ``std::expected``) and the Rust core
(``slamlab_foundations::error::ErrorKind``, returned through ``Result``) use the same three
kinds, and the bindings raise the exception of the matching kind, so a degenerate input
fails the same way in all three languages.

All three are :class:`ValueError` subclasses: each says the input could not produce the
requested result.
"""


class SlamlabError(ValueError):
    """Base class of every error a slamlab function raises on purpose."""


class InvalidInputError(SlamlabError):
    """The input violates the function's contract: shape, dtype, range or finiteness."""


class DegenerateError(SlamlabError):
    """The input is valid but geometrically degenerate, so the result is not determined.

    For example pure rotation (zero baseline), collinear points, or too few
    correspondences.
    """


class NotConvergedError(SlamlabError):
    """An iterative method failed to converge where that is an error.

    Either the caller requested strict mode and the convergence criteria were not met, or
    the iteration diverged with no usable estimate. Reaching the iteration cap is otherwise
    not an error: iterative solvers return their best estimate with a solve summary.
    """
