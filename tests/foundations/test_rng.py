"""PCG32 tests: the parity fixture (shared with the C++ and Rust tests), and input checks."""

from __future__ import annotations

import pytest

from slamlab.foundations.errors import InvalidInputError
from slamlab.foundations.rng import Pcg32
from tests.conftest import load_fixture

RNG = load_fixture("foundations/rng.json")


def test_matches_the_reference_library() -> None:
    """The output stream is the reference library's, bit for bit."""
    reference = RNG["reference"]
    rng = Pcg32(reference["seed"], reference["stream"])
    assert [rng.next_u32() for _ in reference["next_u32"]] == reference["next_u32"]


def test_samples_match_the_fixture() -> None:
    """Floyd samples from one generator are the fixture's, which the C++ and Rust tests share."""
    samples = RNG["samples"]
    rng = Pcg32(samples["seed"], samples["stream"])
    for draw in samples["draws"]:
        assert rng.sample(draw["population"], draw["count"]) == draw["expected"]


def test_samples_are_distinct_and_uniform() -> None:
    """Every sample is distinct and in range; each index is drawn about equally often."""
    rng = Pcg32(3)
    counts = [0] * 10
    for _ in range(20000):
        drawn = rng.sample(10, 3)
        assert len(set(drawn)) == 3
        for index in drawn:
            counts[index] += 1
    # 6000 expected per index; the standard deviation is about 65.
    assert all(abs(c - 6000) < 400 for c in counts)


def test_streams_are_independent() -> None:
    """Two streams from one seed give different sequences."""
    a, b = Pcg32(9, 0), Pcg32(9, 1)
    assert [a.next_u32() for _ in range(4)] != [b.next_u32() for _ in range(4)]


def test_bounded_is_below_its_bound() -> None:
    """A bounded draw is in range; a bound of 1 always gives 0."""
    rng = Pcg32(1)
    assert all(0 <= rng.bounded(7) < 7 for _ in range(1000))
    assert rng.bounded(1) == 0


@pytest.mark.parametrize(
    ("make", "message"),
    [
        (lambda: Pcg32(-1), "seed"),
        (lambda: Pcg32(1 << 64), "seed"),
        (lambda: Pcg32(0, 1 << 63), "stream"),
        (lambda: Pcg32(0).bounded(0), "bound"),
        (lambda: Pcg32(0).bounded(1 << 32), "bound"),
        (lambda: Pcg32(0).sample(3, 4), "count"),
    ],
)
def test_rejects_invalid_input(make: object, message: str) -> None:
    """Out-of-range seeds, streams, bounds and sample sizes are rejected."""
    assert callable(make)
    with pytest.raises(InvalidInputError, match=message):
        make()
