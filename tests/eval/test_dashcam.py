"""A dashcam's recordings: formats, drives, mask, GPS records, import, config, command line.

Synthetic segments only. The overlay's reading is in ``test_dashcam_overlay.py`` and the
calibration in ``test_dashcam_calibrate.py``.
"""

from __future__ import annotations

import datetime as dt
import json
import struct
from pathlib import Path

import numpy as np
import pytest

from slam_it.eval import images, lenses, tum, video
from slam_it.eval.dashcam import config, d016, drives, ingest, mask, stream
from slam_it.eval.dashcam.__main__ import main
from slam_it.foundations.errors import InvalidInputError
from tests.eval.dashcam_recordings import (
    HOOD,
    hooded,
    record,
    scene,
    segment_name,
    write_segment,
)

START = dt.datetime(2026, 1, 2, 3, 4, 5)
"""The first segment's start."""


def _segments(
    root: Path, camera: str, starts: list[dt.datetime], *, frames: int = 10, rate: int = 5
) -> list[Path]:
    """Write a camera's segments under a card's root, a moving scene under a hood."""
    rng = np.random.default_rng(len(starts) + ord(camera))
    paths = []
    for start in starts:
        pictures = [hooded(scene(rng, 90, 160)) for _ in range(frames)]
        path = root / "video" / camera / segment_name(start, camera)
        paths.append(write_segment(path, pictures, rate=rate))
    return paths


# -- The D016's formats ------------------------------------------------------------------------


def test_a_segment_name_gives_its_camera_and_start() -> None:
    """A segment's name gives its camera and start; another name is refused."""
    assert d016.segment_start("20260102_030405R.ts") == ("R", START)
    with pytest.raises(InvalidInputError, match="not a D016"):
        d016.segment_start("20260102_030405X.ts")
    with pytest.raises(InvalidInputError, match="no such date"):
        d016.segment_start("20261302_030405F.ts")


def test_the_clock_is_seconds_on_the_cameras_own_clock() -> None:
    """Clock seconds subtract as times on the camera's clock; a zoned time is refused."""
    later = START + dt.timedelta(seconds=90, microseconds=500_000)
    assert d016.clock_seconds(later) - d016.clock_seconds(START) == pytest.approx(90.5)
    with pytest.raises(InvalidInputError, match="naive"):
        d016.clock_seconds(START.replace(tzinfo=dt.UTC))


def test_the_overlay_box_is_known_for_each_recorded_size() -> None:
    """Each recorded frame size has its overlay box; another size has none."""
    assert d016.overlay_box(3840, 2160) == (40, 1840, 1040, 150)
    assert d016.overlay_box(2560, 1440) == (30, 1190, 960, 150)
    with pytest.raises(InvalidInputError, match="no overlay box"):
        d016.overlay_box(1920, 1080)


def test_an_overlay_parses_and_a_field_out_of_range_counts_as_unread() -> None:
    """An overlay's lines parse; a field out of range counts as not read."""
    read = d016.parse_overlay("N:12.34567 W:123.45678 12mph", "2026/01/02 03:04:05")
    assert (read.latitude, read.longitude, read.mph, read.clock) == (
        12.34567,
        -123.45678,
        12,
        START,
    )
    south = d016.parse_overlay("S:12.34567E:023.45678 7mph", "")
    assert (south.latitude, south.longitude, south.clock) == (-12.34567, 23.45678, None)
    assert d016.parse_overlay("N:92.34567W:123.45678 12mph", "").latitude is None
    assert d016.parse_overlay("N:12.34567W:123.45678 180mph", "").mph is None
    assert d016.parse_overlay("", "2026/13/02 03:04:05").clock is None


def test_a_gps_record_decodes_its_time_flag_speed_and_course() -> None:
    """A GPS record gives its UTC time, fix flag, speed in m/s and course."""
    utc = dt.datetime(2026, 1, 2, 11, 4, 5)
    decoded = d016.decode_record(record(utc, knots=20.0, course=270.5))
    assert decoded.utc == utc
    assert decoded.valid
    assert decoded.speed == pytest.approx(20.0 * d016.KNOT)
    assert decoded.course == pytest.approx(270.5)
    assert not d016.decode_record(record(utc, valid=False)).valid
    undated = bytearray(record(utc))
    struct.pack_into("<i", undated, 16, 13)  # a 13th month
    assert d016.decode_record(bytes(undated)).utc is None
    with pytest.raises(InvalidInputError, match="160 bytes"):
        d016.decode_record(b"\0" * 10)


def test_a_segments_gps_records_are_read_with_their_video_times(tmp_path: Path) -> None:
    """A segment's records are read in order, each with its video time."""
    rng = np.random.default_rng(0)
    pictures = [hooded(scene(rng, 90, 160)) for _ in range(30)]
    utc = dt.datetime(2026, 1, 2, 11, 4, 5)
    records = {k: record(utc + dt.timedelta(seconds=k // 10)) for k in (0, 10, 20)}
    records[25] = b"\1" * 40  # a packet of another size: not a record
    path = write_segment(tmp_path / segment_name(START, "F"), pictures, rate=10, records=records)
    found = stream.records(path)
    assert [r.record.utc for r in found] == [utc + dt.timedelta(seconds=s) for s in (0, 1, 2)]
    times = [r.video_s for r in found]
    assert times == sorted(times)
    assert times[0] == pytest.approx(0.0, abs=0.3)
    assert times[2] == pytest.approx(2.0, abs=0.3)


# -- Drives ------------------------------------------------------------------------------------


def _touch(root: Path, camera: str, start: dt.datetime) -> None:
    path = root / "video" / camera / segment_name(start, camera)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"")


def test_segments_group_into_drives_with_their_rear(tmp_path: Path) -> None:
    """Segments group into drives by their gaps, each with its rear segments."""
    minute = dt.timedelta(minutes=1)
    for k in range(3):
        _touch(tmp_path, "F", START + k * minute)
        _touch(tmp_path, "R", START + k * minute + dt.timedelta(seconds=2))
    later = START + dt.timedelta(hours=2)
    _touch(tmp_path, "F", later)
    (tmp_path / "video" / "F" / "notes.txt").write_text("not a segment")
    found = drives.scan(tmp_path)
    assert len(found) == 7
    first, second = drives.find_drives(found)
    assert [s.start for s in first.front] == [START + k * minute for k in range(3)]
    assert len(first.rear) == 3
    assert second.rear == ()
    assert second.front[0].start == later
    assert (first.start, first.end) == (START, START + 2 * minute + dt.timedelta(seconds=2))
    middle = drives.window(first.front, START + minute, START + minute)
    assert [s.start for s in middle] == [START + minute]
    with pytest.raises(InvalidInputError, match="F or R"):
        first.segments("X")
    with pytest.raises(FileNotFoundError):
        drives.scan(tmp_path / "nowhere")


# -- The static mask ---------------------------------------------------------------------------


def test_the_hood_is_measured_static_and_the_moving_scene_is_not(tmp_path: Path) -> None:
    """The hood's edges never move, so it is masked; the moving scene is not."""
    paths = _segments(tmp_path, "F", [START], frames=30, rate=5)
    static = mask.measure(paths, every=0.2, width=160, min_size=20, close=2, grow=1)
    top = round(HOOD * static.shape[0])
    assert static[top + 2 :, :].mean() > 0.9
    assert static[: top - 3, :].mean() < 0.05
    out = mask.finish(static, 320, 180, bottom=0.95, lines=[mask.Line(0.0, 0.0, 1.0, 0.0, 0.02)])
    assert out.shape == (180, 320)
    assert out.dtype == np.uint8
    assert set(np.unique(out)) <= {0, 255}
    assert (out[171:, :] == 0).all()  # the band below 95%
    assert (out[0, :] == 0).all()  # the drawn line along the top
    assert (out[60, :] == 255).mean() > 0.9


def test_a_mask_scaled_down_keeps_a_thin_line() -> None:
    """A one-pixel line survives the mask being scaled down."""
    thin = np.full((100, 300), 255, dtype=np.uint8)
    thin[:, 151] = 0
    small = mask.scaled(thin, 100, 33)
    assert small.shape == (33, 100)
    assert (small[:, 50] == 0).all()
    assert (small == 0).sum() <= 2 * 33


def test_mask_settings_out_of_range_are_refused() -> None:
    """No segments, or a share or band out of range, is refused."""
    with pytest.raises(InvalidInputError, match="none"):
        mask.measure([])
    with pytest.raises(InvalidInputError, match="share"):
        mask.measure(["x.ts"], share=1.5)
    with pytest.raises(InvalidInputError, match="bottom"):
        mask.finish(np.zeros((4, 4), dtype=bool), 8, 8, bottom=1.5)


# -- Importing a drive -------------------------------------------------------------------------


def test_a_drives_segments_import_as_one_sequence_on_the_overlay_clock(tmp_path: Path) -> None:
    """Two segments import as one sequence timed on the overlay clock, gap kept."""
    later = START + dt.timedelta(minutes=1)
    _segments(tmp_path, "F", [START, later], frames=10, rate=5)
    front = drives.find_drives(drives.scan(tmp_path))[0].front
    camera = lenses.Pinhole(150.0, 150.0, 79.5, 44.5, 160, 90)
    keep = np.full((90, 160), 255, dtype=np.uint8)
    keep[70:, :] = 0
    ingest.import_drive(front, tmp_path / "out", camera, lag=1.25, mask=keep)
    frames = tum.read_frames(tmp_path / "out")
    first, second = d016.clock_seconds(START), d016.clock_seconds(later)
    expected = [first + 1.25 + k / 5 for k in range(10)] + [
        second + 1.25 + k / 5 for k in range(10)
    ]
    assert frames.timestamps == pytest.approx(expected, abs=1e-5)
    assert frames.mask is not None
    assert (frames.mask[70:, :] == 0).all()
    source = json.loads((tmp_path / "out" / "source.json").read_text())
    assert (source["kind"], source["camera"], source["lag"]) == ("dashcam", "F", 1.25)
    assert source["segments"] == [s.path.name for s in front]
    assert source["intrinsics"]["source"] == "given"


def test_a_drive_import_is_refused_past_its_limits_or_across_cameras(tmp_path: Path) -> None:
    """An import past its frame limit leaves nothing; two cameras' segments are refused."""
    _segments(tmp_path, "F", [START], frames=10, rate=5)
    _segments(tmp_path, "R", [START], frames=10, rate=5)
    drive = drives.find_drives(drives.scan(tmp_path))[0]
    camera = lenses.Pinhole(150.0, 150.0, 79.5, 44.5, 160, 90)
    few = video.Limits(max_seconds=60.0, max_frames=5, short_side=480)
    with pytest.raises(InvalidInputError, match="more than 5 frames"):
        ingest.import_drive(drive.front, tmp_path / "a", camera, lag=0.0, limits=few)
    assert not (tmp_path / "a").exists()
    with pytest.raises(InvalidInputError, match="one camera"):
        ingest.import_drive(drive.front + drive.rear, tmp_path / "b", camera, lag=0.0)


# -- The config and the command line ------------------------------------------------------------

TEMPLATE = Path(__file__).parents[2] / "docs" / "dashcam.template.toml"


def _config(tmp_path: Path, lens: str = "") -> Path:
    path = tmp_path / "d016.toml"
    path.write_text(
        f"""
[card]
root = "card"
[work]
folder = "work"
[rig]
baseline = 1.83
[front]
mask = "masks/front.png"
bottom = 0.95
lines = [[0.0, 0.0, 1.0, 0.0, 0.02]]
{lens}
[rear]
mask = "masks/rear.png"
[import]
max_frames = 500
""",
        encoding="utf-8",
    )
    return path


FRONT_LENS = """
[front.lens]
model = "pinhole"
size = [160, 90]
fx = 150.0
fy = 150.0
cx = 79.5
cy = 44.5
"""


def test_the_config_reads_paths_from_its_folder_and_refuses_unknown_keys(tmp_path: Path) -> None:
    """Relative paths are the config's folder's; a misspelt key is refused."""
    settings = config.read(_config(tmp_path, FRONT_LENS))
    assert settings.root == tmp_path.resolve() / "card"
    assert settings.front.mask == tmp_path.resolve() / "masks" / "front.png"
    assert settings.front.lines == (mask.Line(0.0, 0.0, 1.0, 0.0, 0.02),)
    assert settings.front.lens == lenses.Pinhole(150.0, 150.0, 79.5, 44.5, 160, 90)
    assert settings.rear.lens is None
    assert settings.rear.bottom is None
    assert settings.limits.max_frames == 500
    assert settings.limits.max_seconds == ingest.LIMITS.max_seconds
    typo = tmp_path / "typo.toml"
    typo.write_text(_config(tmp_path).read_text().replace("baseline", "baseln"))
    with pytest.raises(InvalidInputError, match="unknown keys"):
        config.read(typo)


def test_the_template_in_the_repo_reads(tmp_path: Path) -> None:
    """The repo's template is a valid config."""
    settings = config.read(TEMPLATE)
    assert settings.baseline > 0.0
    assert settings.front.lens is None


def test_the_steps_list_mask_and_import_a_drive(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The command line lists the drives, masks one, and imports it."""
    card = tmp_path / "card"
    _segments(card, "F", [START, START + dt.timedelta(minutes=1)], frames=10, rate=5)
    _segments(card, "R", [START + dt.timedelta(seconds=1)], frames=10, rate=5)
    path = _config(tmp_path, FRONT_LENS)
    assert main(["drives", str(path)]) == 0
    listed = capsys.readouterr().out
    drive = START.strftime("%Y%m%d_%H%M%S")
    assert listed.startswith(drive)
    assert "front 2, rear 1" in listed
    assert main(["mask", str(path), "--drive", drive, "--camera", "F", "--every", "0.2"]) == 0
    written = images.read_gray(tmp_path / "masks" / "front.png")
    assert written.shape == (90, 160)
    assert (written[86:, :] == 0).all()
    assert (tmp_path / "work" / drive / "mask_F_preview.png").is_file()
    with pytest.raises(InvalidInputError, match="exists"):
        main(["mask", str(path), "--drive", drive, "--camera", "F"])
    with pytest.raises(InvalidInputError, match="overlay step"):
        main(["import", str(path), "--drive", drive, "--camera", "F", str(tmp_path / "seq")])
    overlay = {"camera": "F", "lag": 0.5, "reads": 0, "kept": 0, "segments": [], "fixes": []}
    (tmp_path / "work" / drive / "overlay_F.json").write_text(json.dumps(overlay))
    out = tmp_path / "seq"
    assert main(["import", str(path), "--drive", drive, "--camera", "F", str(out)]) == 0
    assert "20 frames from 2 segments, masked" in capsys.readouterr().out
    assert tum.read_frames(out).timestamps[0] == pytest.approx(d016.clock_seconds(START) + 0.5)
    with pytest.raises(InvalidInputError, match="no lens"):
        main(["import", str(path), "--drive", drive, "--camera", "R", str(tmp_path / "r")])
    with pytest.raises(InvalidInputError, match="not on the card"):
        main(["mask", str(path), "--drive", "20000101_000000", "--camera", "R"])
