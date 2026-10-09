"""The edge pipeline's settings: ORB-SLAM's defaults, checked ranges, the extraction they give."""

from __future__ import annotations

import math
from dataclasses import asdict, fields

import pytest

from slam_it_all.edge.options import Options
from slam_it_all.foundations.errors import InvalidInputError


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


def test_stereo_matching_carries_the_stereo_settings() -> None:
    """stereo_matching gives the ``stereo_*`` settings as rung 0's row-matching options."""
    matching = Options(stereo_max_distance=60, stereo_band=3.0, stereo_search=4).stereo_matching()
    assert (matching.max_distance, matching.band, matching.window) == (60, 3.0, 5)
    assert (matching.search, matching.outlier_factor) == (4, 2.1)


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
        ({"th_high": 257}, "th_high: must be from 1 to 256"),
        ({"th_low": 0}, "th_low: must be from 1"),
        ({"stereo_outlier_factor": 0.5}, "stereo_outlier_factor: must be at least 1"),
        ({"stereo_window": 50}, "stereo_window: must be from 1"),
        ({"stereo_band": -1.0}, "stereo_band: must be finite and positive"),
    ],
)
def test_settings_out_of_range_are_refused(settings: dict[str, object], message: str) -> None:
    """Every setting is checked when the options are built."""
    with pytest.raises(InvalidInputError, match=message):
        Options(**settings)  # type: ignore[arg-type]


@pytest.mark.parametrize("name", [item.name for item in fields(Options)])
def test_every_setting_refuses_a_bad_value_by_name(name: str) -> None:
    """Each setting refuses an integer above 2**30, or a real that is not a number, by name.

    The native checks are held to the same table (review panel finding H21).
    """
    bad = (1 << 30) + 1 if isinstance(getattr(Options(), name), int) else math.nan
    with pytest.raises(InvalidInputError, match=rf"^{name}: "):
        Options(**{name: bad})  # type: ignore[arg-type]
