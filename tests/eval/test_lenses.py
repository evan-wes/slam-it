"""A phone video's intrinsics: profiles read and matched, fallbacks, and fisheye undistortion.

The profiles are the CC0 fixtures in ``tests/fixtures/eval/lens_profiles/`` (their README names
them); undistortion is checked on the room, rendered through a fisheye lens.
"""

from __future__ import annotations

import io
import json
import math
import tarfile
from pathlib import Path

import numpy as np
import pytest

from slam_it_all.eval import lenses
from slam_it_all.foundations import image
from slam_it_all.foundations.errors import InvalidInputError
from tests.conftest import FIXTURES
from tests.edge import room

PROFILES = FIXTURES / "eval" / "lens_profiles"
"""The fixture profiles."""

IPHONE = {
    lenses.APPLE_MAKE: "Apple",
    lenses.APPLE_MODEL: "iPhone 14 Pro",
    lenses.APPLE_LENS: "iPhone 14 Pro back camera 6.86mm f/1.78",
    lenses.APPLE_FOCAL_35MM: "24",
}
"""An iPhone 14 Pro's main camera, as its video's metadata says."""

PIXEL = {lenses.ANDROID_MAKE: "Google", lenses.ANDROID_MODEL: "Pixel 8"}
"""A Pixel 8, as its video's metadata is assumed to say."""

ROOM_FISHEYE = lenses.Fisheye(
    lenses.Pinhole(170.0, 170.0, 159.5, 119.5, 320, 240), (0.05, -0.02, 0.01, 0.0)
)
"""A wide lens for the room: about 95 degrees across."""


@pytest.fixture(scope="module")
def profiles() -> list[lenses.Profile]:
    """Load the fixture profiles."""
    return lenses.load(PROFILES)


@pytest.mark.parametrize(
    ("text", "equivalent", "kind"),
    [
        ("iPhone 14 Pro back camera 6.86mm f/1.78", False, "main"),
        ("iPhone 14 Pro back ultra wide camera 2.22mm f/2.2", False, "ultrawide"),
        ("iPhone 14 Pro back telephoto camera 9mm f/2.8", False, "telephoto"),
        ("iPhone 14 Pro front camera 2.69mm f/1.9", False, "front"),
        ("24mm", True, "main"),
        ("13mm", True, "ultrawide"),
        ("0,5 Wide", True, "ultrawide"),
        ("x0.6 film mode", True, "ultrawide"),
        ("UW", True, "ultrawide"),
        ("3x", True, "telephoto"),
        ("Telephoto (5x)_115mm", True, "telephoto"),
        ("Wide", True, "main"),
        ("1x zoom", True, "main"),
        ("Reprojection error 0,61_4k_16by9_3840x2160-29.86fps", True, "main"),
    ],
)
def test_a_lens_name_says_its_kind(text: str, equivalent: bool, kind: str) -> None:
    """Apple's lens tags by their words; a profile's free text by its words, zoom and focal."""
    assert lenses.lens_kind(text, equivalent=equivalent) == kind


def test_the_fixture_profiles_read_and_the_rough_one_is_dropped(
    profiles: list[lenses.Profile],
) -> None:
    """Seven of the eight read; the Pixel 8's 1080p one (4 px RMS) is over MAX_RMS."""
    names = {p.name for p in profiles}
    assert len(profiles) == 7
    assert "Google_Pixel 8___1080p_16by9_1920x1080-30.00fps" not in names
    kinds = {p.name: p.lens for p in profiles}
    assert kinds["Apple_iPhone 14 Pro_3x__4k_16by9_3840x2160-29.98fps"] == "telephoto"
    assert kinds["Apple_iPhone 14 Pro Max__0,5 Wide_1080p_16by9_1920x1080-29.99fps"] == "ultrawide"


@pytest.mark.parametrize(
    "change",
    [
        {"distortion_model": "poly3"},
        {"crop": 0.9},
        {"input_horizontal_stretch": 1.33},
        {"fisheye_params": {"RMS_error": 0.4, "camera_matrix": [[1, 0], [0, 1]]}},
        {"calib_dimension": "1920x1080"},
    ],
)
def test_a_profile_not_usable_is_skipped(change: dict[str, object], tmp_path: Path) -> None:
    """Another model, a crop, a stretch, a malformed matrix or size: not usable."""
    source = PROFILES / "Apple_iPhone 14 Pro_24mm__1080p_16by9_1920x1080-25.00fps.json"
    data = json.loads(source.read_text())
    data.update(change)
    (tmp_path / "changed.json").write_text(json.dumps(data))
    assert lenses.read_profile(tmp_path / "changed.json") is None
    assert lenses.read_profile(source) is not None


@pytest.mark.parametrize(
    ("metadata", "size", "name"),
    [
        (IPHONE, (1920, 1080), "Apple_iPhone 14 Pro_24mm__1080p_16by9_1920x1080-25.00fps"),
        (IPHONE, (3840, 2160), "apple_iphone 14 Pro_24mm__4k_16by9_3840x2160-59.92fps"),
        (
            {**IPHONE, lenses.APPLE_LENS: "iPhone 14 Pro back telephoto camera 9mm f/2.8"},
            (1920, 1080),
            "Apple_iPhone 14 Pro_3x__4k_16by9_3840x2160-29.98fps",
        ),
        (
            {
                **IPHONE,
                lenses.APPLE_MODEL: "iPhone 14 Pro Max",
                lenses.APPLE_LENS: "back ultra wide",
            },
            (1920, 1080),
            "Apple_iPhone 14 Pro Max__0,5 Wide_1080p_16by9_1920x1080-29.99fps",
        ),
        (PIXEL, (3840, 2160), "Google_Pixel 8_Main__4k_16by9_3840x2176-29.99fps"),
        (PIXEL, (1920, 1080), "Google_Pixel 8_Main__4k_16by9_3840x2176-29.99fps"),
    ],
)
def test_a_video_matches_its_phone_and_lens(
    metadata: dict[str, str], size: tuple[int, int], name: str, profiles: list[lenses.Profile]
) -> None:
    """Brand, model and lens whole; the nearest resolution; the profile scaled to the video."""
    found = lenses.match(profiles, lenses.Video(metadata, *size, 30.0))
    assert found is not None
    profile, fisheye = found
    assert profile.name == name
    assert (fisheye.camera.width, fisheye.camera.height) == size
    assert fisheye.k == profile.fisheye.k


@pytest.mark.parametrize(
    ("metadata", "size"),
    [
        ({**IPHONE, lenses.APPLE_LENS: "iPhone 14 Pro back ultra wide camera"}, (1920, 1080)),
        ({**IPHONE, lenses.APPLE_MODEL: "iPhone 14"}, (1920, 1080)),
        (IPHONE, (1080, 1920)),  # stored portrait: no profile in that orientation
        (IPHONE, (1440, 1080)),  # 4:3: no profile of that aspect
        ({lenses.APPLE_FOCAL_35MM: "24"}, (1920, 1080)),  # no phone named
    ],
)
def test_no_profile_matches_another_phone_lens_orientation_or_aspect(
    metadata: dict[str, str], size: tuple[int, int], profiles: list[lenses.Profile]
) -> None:
    """No 14 Pro ultrawide, no iPhone 14, no portrait or 4:3 profile, no phone: no match."""
    assert lenses.match(profiles, lenses.Video(metadata, *size, 30.0)) is None


def test_the_best_source_that_has_intrinsics_wins(profiles: list[lenses.Profile]) -> None:
    """A profile, else the 35mm focal length, else the nominal field of view; or as asked."""
    video = lenses.Video(IPHONE, 1920, 1080, 25.0)
    chosen = lenses.choose(video, profiles)
    assert (chosen.source, chosen.profile) == (
        "profile",
        "Apple_iPhone 14 Pro_24mm__1080p_16by9_1920x1080-25.00fps",
    )
    assert isinstance(chosen.lens, lenses.Fisheye)
    unmatched = lenses.Video({**IPHONE, lenses.APPLE_MODEL: "iPhone 15"}, 1920, 1080, 30.0)
    assert lenses.choose(unmatched, profiles).source == "35mm"
    assert lenses.choose(lenses.Video({}, 1920, 1080), profiles).source == "nominal"
    assert lenses.choose(video, profiles, "nominal").source == "nominal"
    assert lenses.choose(video, [], "auto").source == "35mm"
    with pytest.raises(InvalidInputError, match="no lens profile matches"):
        lenses.choose(unmatched, profiles, "profile")
    with pytest.raises(InvalidInputError, match="no 35mm-equivalent"):
        lenses.choose(lenses.Video({}, 1920, 1080), profiles, "35mm")
    with pytest.raises(InvalidInputError, match="expected auto or one of"):
        lenses.choose(video, profiles, "guess")


def test_the_fallbacks_give_their_field_of_view() -> None:
    """35mm on the diagonal: 24 mm is 84 degrees across it; the nominal, its own constant."""
    width, height = 1920, 1080
    diagonal = math.hypot(width, height)
    camera = lenses.from_35mm(24.0, width, height)
    across = 2 * math.degrees(math.atan(diagonal / 2 / camera.fx))
    assert across == pytest.approx(2 * math.degrees(math.atan(lenses.FULL_FRAME_DIAGONAL / 48)))
    assert (camera.cx, camera.cy) == (959.5, 539.5)
    nominal = lenses.nominal(width, height)
    assert 2 * math.degrees(math.atan(diagonal / 2 / nominal.fx)) == pytest.approx(
        lenses.NOMINAL_DIAGONAL_FOV
    )


def test_the_median_field_of_view_is_over_main_cameras(profiles: list[lenses.Profile]) -> None:
    """Over the fixtures' four main-camera profiles, not their telephoto or ultrawide."""
    fov, count = lenses.nominal_fov(profiles)
    assert count == 4
    assert 60.0 < fov < 90.0


def test_the_fisheye_model_inverts() -> None:
    """A ray's angle, bent and unbent; a pinhole ray, to its pixel and back."""
    theta = np.linspace(0.0, 1.2, 50)
    np.testing.assert_allclose(ROOM_FISHEYE.unbend(ROOM_FISHEYE.bend(theta)), theta, atol=1e-12)
    rng = np.random.default_rng(0)
    a, b = rng.uniform(-1.0, 1.0, (2, 100))
    pixels = ROOM_FISHEYE.source(a, b)
    np.testing.assert_allclose(
        ROOM_FISHEYE.pinhole(pixels[:, 0], pixels[:, 1]), np.stack([a, b], -1), atol=1e-9
    )


def test_a_point_lands_where_the_fisheye_formula_puts_it() -> None:
    """The map's source pixel of a point's pinhole pixel is OpenCV's fisheye projection of it."""
    rng = np.random.default_rng(1)
    points = rng.uniform([-1.0, -0.8, 0.8], [1.0, 0.8, 2.0], (200, 3))
    c, (k1, k2, k3, k4) = ROOM_FISHEYE.camera, ROOM_FISHEYE.k
    r = np.hypot(points[:, 0], points[:, 1])
    theta = np.arctan2(r, points[:, 2])
    theta_d = theta * (1 + k1 * theta**2 + k2 * theta**4 + k3 * theta**6 + k4 * theta**8)
    expected = np.stack(
        [c.fx * theta_d * points[:, 0] / r + c.cx, c.fy * theta_d * points[:, 1] / r + c.cy], -1
    )
    got = ROOM_FISHEYE.source(points[:, 0] / points[:, 2], points[:, 1] / points[:, 2])
    np.testing.assert_allclose(got, expected, atol=1e-9)


def test_undistortion_leaves_no_empty_border_and_keeps_the_focal() -> None:
    """Every output pixel samples between the source's outermost pixel centres, all the way."""
    out, mapping = lenses.undistortion(ROOM_FISHEYE)
    assert (out.fx, out.fy) == (ROOM_FISHEYE.camera.fx, ROOM_FISHEYE.camera.fy)
    assert mapping.shape == (out.height, out.width, 2)
    c = ROOM_FISHEYE.camera
    assert mapping[..., 0].min() >= 0.0
    assert mapping[..., 0].max() <= c.width - 1
    assert mapping[..., 1].min() >= 0.0
    assert mapping[..., 1].max() <= c.height - 1
    # largest: one more pixel would reach past an outermost pixel centre somewhere
    assert mapping[..., 0].min() < 1.0 or mapping[..., 0].max() > c.width - 2.0
    assert mapping[..., 1].min() < 1.0 or mapping[..., 1].max() > c.height - 2.0


def _render_fisheye(scene: room.Room, lens: lenses.Fisheye, pose: np.ndarray) -> np.ndarray:
    """Render the room through a fisheye lens, as room.Room renders a pinhole one (2x2 rays)."""
    c = lens.camera
    v, u = np.mgrid[0 : c.height, 0 : c.width].astype(np.float64)
    total = np.zeros((c.height, c.width))
    for du, dv in [(-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25)]:
        ab = lens.pinhole((u + du).ravel(), (v + dv).ravel())
        rays = np.column_stack([ab, np.ones(len(ab))])
        _, shade = scene.cast(pose[:3, 3], rays @ pose[:3, :3].T)
        total += shade.reshape(c.height, c.width)
    return np.clip(np.floor(total / 4.0 + 0.5), 0, 255).astype(np.uint8)


def _box(picture: np.ndarray, side: int = 5) -> np.ndarray:
    """Return a picture's means over side-by-side windows (valid ones only)."""
    c = np.cumsum(np.cumsum(np.pad(picture.astype(float), ((1, 0), (1, 0))), 0), 1)
    out: np.ndarray = c[side:, side:] - c[:-side, side:] - c[side:, :-side] + c[:-side, :-side]
    return out / (side * side)


def test_the_room_through_a_fisheye_undistorts_to_its_pinhole_view() -> None:
    """Undistorted, the fisheye view lines up with the output camera's pinhole view.

    Bilinear resampling cannot keep the room's noise-like texture (a mean error of 5 gray
    levels), so the views are compared as they line up: at their centre, best with no shift;
    over the whole frame, within 2 gray levels on average once both are averaged over 5x5
    windows (1.25 measured).
    """
    scene, pose = room.Room(0), room.path(1)[0]
    distorted = _render_fisheye(scene, ROOM_FISHEYE, pose)
    out, mapping = lenses.undistortion(ROOM_FISHEYE)
    undistorted = image.remap(distorted, mapping).astype(float)
    camera = room.Camera(out.fx, out.fy, out.cx, out.cy, out.width, out.height)
    truth = scene.render(camera, pose)[0].astype(float)
    assert np.abs(_box(undistorted) - _box(truth)).mean() < 2.0
    cy, cx = out.height // 2, out.width // 2
    centre = undistorted[cy - 60 : cy + 60, cx - 80 : cx + 80]

    def error(dx: int, dy: int) -> float:
        shifted = truth[cy - 60 + dy : cy + 60 + dy, cx - 80 + dx : cx + 80 + dx]
        return float(np.abs(centre - shifted).mean())

    shifts = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]
    assert error(0, 0) < min(error(dx, dy) for dx, dy in shifts)


def _archive(path: Path, members: dict[str, bytes], links: tuple[str, ...] = ()) -> str:
    """Write a gzipped tar of the given files (and symlinks); return its file URL."""
    with tarfile.open(path, "w:gz") as archive:
        for name, body in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(body)
            archive.addfile(info, io.BytesIO(body))
        for name in links:
            info = tarfile.TarInfo(name)
            info.type, info.linkname = tarfile.SYMTYPE, "/etc/passwd"
            archive.addfile(info)
    return path.as_uri()


def test_fetch_keeps_only_the_phone_profiles_under_its_own_names(tmp_path: Path) -> None:
    """Phone profiles kept, flattened; others, climbers, links, giants and non-JSON dropped."""
    top = "lens_profiles-0123/"
    url = _archive(
        tmp_path / "db.tar.gz",
        {
            f"{top}Mobile phones/Apple/a.json": b"{}",
            f"{top}Mobile phones/Google/b.json": b"{}",
            f"{top}GoPro/c.json": b"{}",
            f"{top}Mobile phones/../evil.json": b"{}",
            f"{top}Mobile phones/Apple/big.json": b" " * (lenses.MAX_PROFILE_BYTES + 1),
            f"{top}Mobile phones/Apple/notes.txt": b"x",
        },
        links=(f"{top}Mobile phones/Apple/link.json",),
    )
    folder = lenses.fetch(tmp_path / "cache" / "profiles", url=url)
    assert sorted(p.name for p in folder.iterdir()) == ["Apple__a.json", "Google__b.json"]
    assert lenses.fetch(folder, url="file:///nowhere") == folder  # there already: no download
    assert sorted(p.name for p in folder.parent.iterdir()) == ["profiles"]


def test_fetch_refuses_what_is_not_a_phone_archive_and_leaves_nothing(tmp_path: Path) -> None:
    """Not a gzipped tar, or one without phone profiles: refused, nothing left."""
    (tmp_path / "plain").write_bytes(b"plain")
    with pytest.raises(InvalidInputError, match="not a gzipped tar"):
        lenses.fetch(tmp_path / "cache" / "a", url=(tmp_path / "plain").as_uri())
    url = _archive(tmp_path / "none.tar.gz", {"top/GoPro/c.json": b"{}"})
    with pytest.raises(InvalidInputError, match="no phone profile"):
        lenses.fetch(tmp_path / "cache" / "b", url=url)
    assert list((tmp_path / "cache").iterdir()) == []


def test_main_says_the_nominal_field_of_view(capsys: pytest.CaptureFixture[str]) -> None:
    """The program's nominal command, over the fixtures."""
    assert lenses.main(["nominal", "--profiles", str(PROFILES)]) == 0
    assert capsys.readouterr().out.startswith("median diagonal field of view: ")
