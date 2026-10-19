"""PCG32, the random generator every backend shares, so a seed draws the same samples everywhere.

RANSAC is random, and numpy's generator and C++'s ``std::mt19937`` do not produce the same
numbers. To make the backends' RANSAC runs comparable sample for sample (same hypotheses, same
iteration count, same inliers), every backend implements this one small generator: O'Neill's
PCG32 (``pcg32_random_r`` in the reference C library, "PCG: A Family of Simple Fast
Space-Efficient Statistically Good Algorithms for Random Number Generation", 2014), with
its unbiased bounded draw (``pcg32_boundedrand_r``) and Floyd's algorithm for a sample of
distinct indices (Bentley and Floyd, "A sample of brilliance", CACM 1987).

The generator is a 64-bit linear congruential state with a permuted 32-bit output:

    state ← state · 6364136223846793005 + inc   (mod 2⁶⁴)
    output = rotr32(((old ≫ 18) ⊕ old) ≫ 27, old ≫ 59)

``seed`` sets the starting state and ``stream`` the increment (``inc = 2·stream + 1``), so two
streams from one seed are independent sequences. The tests hold every backend to the reference
library's published output.

The C++ and Rust cores mirror it (``slam_it/foundations/rng.hpp``,
``slam_it_foundations::rng``). It is not bound to Python: it is reached through the RANSAC
solvers built on it, and every language's tests read the same draws from
``tests/fixtures/foundations/rng.json``.
"""

from __future__ import annotations

from slam_it.foundations.errors import InvalidInputError

_MASK64 = (1 << 64) - 1
_MASK32 = (1 << 32) - 1
_MULTIPLIER = 6364136223846793005


class Pcg32:
    """O'Neill's PCG32 (XSH RR, 64-bit state, 32-bit output).

    Parameters
    ----------
    seed : int
        The initial state, 0 <= seed < 2⁶⁴.
    stream : int, optional
        The stream (sequence) selector, 0 <= stream < 2⁶³ (default 0).

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `seed` or `stream` is out of range.
    """

    def __init__(self, seed: int, stream: int = 0) -> None:
        if not 0 <= seed <= _MASK64:
            raise InvalidInputError(f"seed: expected 0 <= seed < 2**64, got {seed}")
        if not 0 <= stream < (1 << 63):
            raise InvalidInputError(f"stream: expected 0 <= stream < 2**63, got {stream}")
        self._state = 0
        self._increment = ((stream << 1) | 1) & _MASK64
        self.next_u32()
        self._state = (self._state + seed) & _MASK64
        self.next_u32()

    def next_u32(self) -> int:
        """Draw the next 32-bit output.

        Returns
        -------
        int
            A uniform integer in ``[0, 2³²)``.
        """
        old = self._state
        self._state = (old * _MULTIPLIER + self._increment) & _MASK64
        xorshifted = (((old >> 18) ^ old) >> 27) & _MASK32
        rotation = old >> 59
        return ((xorshifted >> rotation) | (xorshifted << ((-rotation) & 31))) & _MASK32

    def bounded(self, bound: int) -> int:
        """Draw a uniform integer below `bound`, without modulo bias.

        Outputs below ``2³² mod bound`` are rejected, so the accepted range is a whole number
        of copies of ``[0, bound)``.

        Parameters
        ----------
        bound : int
            The exclusive upper bound, 1 <= bound < 2³².

        Returns
        -------
        int
            A uniform integer in ``[0, bound)``.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If `bound` is out of range.
        """
        if not 1 <= bound <= _MASK32:
            raise InvalidInputError(f"bound: expected 1 <= bound < 2**32, got {bound}")
        threshold = ((1 << 32) - bound) % bound
        while True:
            value = self.next_u32()
            if value >= threshold:
                return value % bound

    def sample(self, population: int, count: int) -> list[int]:
        """Draw `count` distinct indices from ``range(population)`` (Floyd's algorithm).

        Exactly `count` bounded draws, whatever the outcome, so every backend consumes the
        stream identically.

        Parameters
        ----------
        population : int
            The number of indices to choose from.
        count : int
            How many to draw, 0 <= count <= population.

        Returns
        -------
        list of int
            The indices, in the order Floyd's algorithm adds them.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If `count` is negative or exceeds `population`.
        """
        if not 0 <= count <= population:
            raise InvalidInputError(
                f"sample: expected 0 <= count <= population, got {count} of {population}"
            )
        chosen: list[int] = []
        for j in range(population - count, population):
            pick = self.bounded(j + 1)
            chosen.append(j if pick in chosen else pick)
        return chosen
