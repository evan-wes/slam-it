"""The edge pipeline's settings: ORB-SLAM's defaults, checked ranges, the extraction they give."""

from __future__ import annotations

from dataclasses import asdict

import pytest

from slamlab.edge.options import Options
from slamlab.foundations.errors import InvalidInputError


def test_the_defaults_are_orb_slams() -> None:
    """A few defaults, as ORB-SLAM2 sets them for 640x480 frames."""
    options = Options()
    assert (options.n_features, options.levels, options.scale_factor) == (1000, 8, 1.2)
    assert (options.chi2_mono, options.chi2_stereo) == (5.991, 7.815)
    assert (options.window, options.close_depth) == (10, 40.0)


def test_extraction_doubles_the_features_while_initializing() -> None:
    """A monocular frame before the map has init_feature_factor times the features."""
    options = Options(n_features=700)
    assert options.extraction().n_features == 700
    assert options.extraction(initializing=True).n_features == 1400


def test_the_settings_cross_as_scalars() -> None:
    """as_dict gives every field as a plain scalar."""
    assert asdict(Options()) == Options().as_dict()


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"window": 0}, "window: must be from 1"),
        ({"seed": -1}, "seed: must be from 0"),
        ({"chi2_mono": 0.0}, "chi2_mono: must be finite and positive"),
        ({"ref_ratio": 1.5}, r"ref_ratio: must be in \(0, 1\]"),
        ({"view_cos_limit": 0.0}, r"view_cos_limit: must be in \(0, 1\]"),
        ({"levels": 0}, "levels: must be from 1"),
        ({"window": 2.0}, "window: must be an integer"),
    ],
)
def test_settings_out_of_range_are_refused(settings: dict[str, object], message: str) -> None:
    """Every setting is checked when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        Options(**settings)  # type: ignore[arg-type]
