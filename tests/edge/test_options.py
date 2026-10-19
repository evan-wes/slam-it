"""The edge pipeline's settings: ORB-SLAM's defaults, checked ranges, the extraction they give."""

from __future__ import annotations

import math
import re
from dataclasses import asdict

import pytest

from slam_it.edge.options import FEATURES_BOUNDS, Options, features_for
from slam_it.foundations.errors import InvalidInputError


def test_the_defaults_are_orb_slams() -> None:
    """A few defaults, as ORB-SLAM2 sets them for 640x480 frames."""
    options = Options()
    assert (
        options.features.n_features,
        options.features.levels,
        options.features.scale_factor,
    ) == (1000, 8, 1.2)
    assert (options.adjustment.chi2_mono, options.adjustment.chi2_stereo) == (5.991, 7.815)
    assert (options.covisibility.local_neighbours, options.stereo.close_depth) == (10, 40.0)
    assert (options.keyframes.busy_ref_ratio, options.mapping.max_queued) == (0.25, 3)


def test_extraction_doubles_the_features_while_initializing() -> None:
    """A monocular frame before the map has ``features.init_factor`` times the features."""
    options = Options().with_settings({"features.n_features": 700})
    assert options.extraction().n_features == 700
    assert options.extraction(initializing=True).n_features == 1400


def test_stereo_matching_carries_the_stereo_settings() -> None:
    """stereo_matching gives the ``stereo`` group's settings as rung 0's row-matching options."""
    matching = (
        Options()
        .with_settings({"stereo.max_distance": 60, "stereo.band": 3.0, "stereo.search": 4})
        .stereo_matching()
    )
    assert (matching.max_distance, matching.band, matching.window) == (60, 3.0, 5)
    assert (matching.search, matching.outlier_factor) == (4, 2.1)


def test_the_settings_cross_as_groups_of_scalars() -> None:
    """as_dict gives each group as a dict of its fields' plain scalars, and the seed."""
    crossed = Options().as_dict()
    assert crossed == asdict(Options())
    assert crossed["relocalization"]["min_inliers"] == 10
    assert crossed["seed"] == 0


@pytest.mark.parametrize(
    ("settings", "message"),
    [
        ({"covisibility.local_neighbours": 0}, "covisibility.local_neighbours: must be from 1"),
        ({"seed": -1}, "seed: must be from 0"),
        ({"adjustment.chi2_mono": 0.0}, "adjustment.chi2_mono: must be finite and positive"),
        ({"keyframes.ref_ratio": 1.5}, r"keyframes.ref_ratio: must be in \(0, 1\]"),
        ({"keyframes.busy_ref_ratio": 0.0}, r"keyframes.busy_ref_ratio: must be in \(0, 1\]"),
        ({"mapping.max_queued": 0}, "mapping.max_queued: must be from 1"),
        ({"tracking.view_cos_limit": 0.0}, r"tracking.view_cos_limit: must be in \(0, 1\]"),
        ({"features.levels": 0}, "features.levels: must be from 1"),
        (
            {"covisibility.local_neighbours": 2.0},
            "covisibility.local_neighbours: must be an integer",
        ),
        ({"matching.th_high": 257}, "matching.th_high: must be from 1 to 256"),
        ({"matching.th_low": 0}, "matching.th_low: must be from 1"),
        ({"stereo.outlier_factor": 0.5}, "stereo.outlier_factor: must be at least 1"),
        ({"stereo.window": 50}, "stereo.window: must be from 1"),
        ({"stereo.band": -1.0}, "stereo.band: must be finite and positive"),
        (
            {"relocalization.confidence": 1.0},
            r"relocalization.confidence: must be in \(0, 1\), got 1.0",
        ),
        (
            {"relocalization.narrow_distance": 257},
            "relocalization.narrow_distance: must be from 1 to 256",
        ),
        ({"loops.confidence": 1.0}, r"loops.confidence: must be in \(0, 1\), got 1.0"),
        ({"loops.fine_distance": 257}, "loops.fine_distance: must be from 1 to 256"),
        ({"loops.essential_damping": 0.0}, "loops.essential_damping: must be finite and positive"),
        ({"features.n_features": "a"}, "features.n_features: must be an integer"),
        ({"features.n_features": None}, "features.n_features: must be an integer"),
    ],
)
def test_settings_out_of_range_are_refused(settings: dict[str, object], message: str) -> None:
    """Every setting is checked when the options are built, and named by its group and field."""
    with pytest.raises(InvalidInputError, match=message):
        Options().with_settings(settings)


@pytest.mark.parametrize("name", list(Options().settings()))
def test_every_setting_refuses_a_bad_value_by_name(name: str) -> None:
    """Each setting refuses an integer above 2**30, or a real that is not a number, by name.

    The native checks are held to the same table (review panel finding H21).
    """
    bad = (1 << 30) + 1 if isinstance(Options().settings()[name], int) else math.nan
    with pytest.raises(InvalidInputError, match=rf"^{re.escape(name)}: "):
        Options().with_settings({name: bad})


def test_an_unknown_setting_or_group_is_refused() -> None:
    """A name no group holds, and a group given as anything but its class."""
    with pytest.raises(InvalidInputError, match="options: no setting 'window'"):
        Options().with_settings({"window": 10})
    with pytest.raises(InvalidInputError, match=r"^tracking: expected TrackingOptions, got dict"):
        Options(tracking={"min_tracked": 30})  # type: ignore[arg-type]


def test_the_settings_are_named_by_group_and_field() -> None:
    """settings() lists all 134, each ``group.field`` but ``seed``, and with_settings reads them."""
    settings = Options().settings()
    assert len(settings) == 134
    assert [name for name in settings if "." not in name] == ["seed"]
    changed = Options().with_settings({"relocalization.min_inliers": 12, "seed": 3})
    assert (changed.relocalization.min_inliers, changed.seed) == (12, 3)
    assert changed.with_settings(Options().settings()) == Options()


@pytest.mark.parametrize(
    ("size", "features"),
    [
        ((640, 480), 1000),  # ORB-SLAM's TUM setting
        ((320, 240), 1000),  # never fewer: a stereo or RGB-D frame must be able to start
        ((854, 480), 1334),  # a 16:9 phone video imported at 480
        ((480, 854), 1334),  # the same on its side
        ((752, 480), 1175),  # ORB-SLAM's EuRoC setting asks 1,200
        ((1241, 376), 1519),  # ORB-SLAM's KITTI setting asks 2,000
        ((1280, 720), 2000),  # never more
    ],
)
def test_features_scale_with_the_frame_area_within_bounds(
    size: tuple[int, int], features: int
) -> None:
    """ORB-SLAM's 1,000 at 640x480, scaled with area, within 1,000 to 2,000."""
    assert features_for(*size) == features
    assert FEATURES_BOUNDS[0] > Options().initialization.min_features_depth
    Options().with_settings(
        {"features.n_features": features_for(*size)}
    )  # a setting the pipeline takes


@pytest.mark.parametrize("size", [(0, 480), (640, -1), (640.5, 480)])
def test_features_for_a_frame_without_a_size_are_refused(size: tuple[float, float]) -> None:
    """A side that is not a positive integer."""
    with pytest.raises(InvalidInputError):
        features_for(*size)  # type: ignore[arg-type]
