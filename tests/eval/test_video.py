"""Importing phone recordings: the room as synthetic videos and Stray Scanner zips, and refusals.

The videos are encoded here with PyAV, losslessly (x264 at qp 0), so an imported frame is the
rendered one up to the colour conversion's rounding.
"""

from __future__ import annotations

import io
import json
import zipfile
import zlib
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from typing import Any

import av
import numpy as np
import pytest

from slam_it.eval import images, lenses, lidar, runs, tum, video
from slam_it.foundations.errors import InvalidInputError
from slam_it.orchestration import vo
from tests.conftest import FIXTURES
from tests.edge import room
from tests.edge.support import ATE_BOUND, FRAMES, OPTIONS
from tests.eval.recordings import APPLE, ROOM_CAMERA, encode, png, rgb, stray_maps, stray_zip

PROFILES = FIXTURES / "eval" / "lens_profiles"
"""The fixture lens profiles (their README names them)."""


@pytest.fixture(scope="module")
def rendered() -> list[room.Frame]:
    """Render the room at 30 frames a second."""
    return room.sequence(FRAMES)


def _imported(folder: Path) -> tuple[tum.Frames, dict[str, Any]]:
    """Read an imported folder as a run does, and its source.json."""
    return tum.read_frames(folder), json.loads((folder / "source.json").read_text())


def test_a_video_on_its_side_imports_upright_with_its_camera(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A video stored on its side, its rotation declared, imports upright, its camera turned."""
    stored = [np.rot90(rgb(f.gray), -1) for f in rendered[:6]]  # turned clockwise
    path = encode(tmp_path / "side.mov", stored, rotation=90)  # shown turned back
    video.import_video(path, tmp_path / "out", lenses.Chosen(ROOM_CAMERA.turned(-1), "given"))
    frames, source = _imported(tmp_path / "out")
    assert (frames.camera.model, frames.camera.width, frames.camera.height) == ("pinhole", 320, 240)
    np.testing.assert_allclose(
        frames.camera.intrinsics, [ROOM_CAMERA.fx, ROOM_CAMERA.fy, ROOM_CAMERA.cx, ROOM_CAMERA.cy]
    )
    np.testing.assert_allclose(frames.timestamps, np.arange(6) / 30, atol=1e-6)
    for file, frame in zip(frames.images, rendered, strict=False):
        gray = images.read_gray(file).astype(int)
        assert np.abs(gray - frame.gray).max() <= 2  # the colour conversion's rounding
    assert source["rotation"] == 90
    assert source["intrinsics"] == {
        "source": "given", "profile": None, "fx": ROOM_CAMERA.fx, "fy": ROOM_CAMERA.fy,
        "cx": ROOM_CAMERA.cx, "cy": ROOM_CAMERA.cy, "width": 320, "height": 240,
        "distortion": None,
    }  # fmt: skip


def test_sixty_frames_a_second_keeps_every_other(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A 60 fps video keeps the frames 1/30 s apart."""
    path = encode(tmp_path / "sixty.mov", [rgb(f.gray) for f in rendered[:12]], rate=60)
    video.import_video(path, tmp_path / "out", lenses.Chosen(ROOM_CAMERA, "given"))
    frames, source = _imported(tmp_path / "out")
    np.testing.assert_allclose(frames.timestamps, np.arange(6) / 30, atol=1e-6)
    assert source["frames"] == {"decoded": 12, "kept": 6}


def test_a_large_video_is_scaled_down_with_its_camera(tmp_path: Path) -> None:
    """A 1280x720 video becomes 854x480, its intrinsics scaled, pixel centres kept."""
    rng = np.random.default_rng(0)
    pictures = [rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8) for _ in range(2)]
    camera = lenses.Pinhole(1000.0, 1000.0, 639.5, 359.5, 1280, 720)
    video.import_video(
        encode(tmp_path / "big.mov", pictures), tmp_path / "out", lenses.Chosen(camera, "given")
    )
    frames, _ = _imported(tmp_path / "out")
    assert (frames.camera.width, frames.camera.height) == (854, 480)
    assert images.read_rgb(frames.images[0]).shape == (480, 854, 3)
    scaled = camera.scaled(854, 480)
    np.testing.assert_allclose(
        frames.camera.intrinsics, [scaled.fx, scaled.fy, scaled.cx, scaled.cy]
    )
    np.testing.assert_allclose([scaled.cx, scaled.cy], [426.5, 239.5])
    assert video.output_size(320, 240) == (320, 240)  # never up
    assert video.output_size(1080, 1920) == (480, 854)


def test_a_phone_video_is_matched_to_its_profile_and_undistorted(tmp_path: Path) -> None:
    """An iPhone 14 Pro's 1080p video takes its main camera's profile, undistorted to pinhole."""
    rng = np.random.default_rng(1)
    pictures = [rng.integers(0, 256, (1080, 1920, 3), dtype=np.uint8) for _ in range(2)]
    path = encode(tmp_path / "phone.mov", pictures, metadata=APPLE)
    chosen = lenses.choose(video.probe(path), lenses.load(PROFILES))
    assert (chosen.source, chosen.profile) == (
        "profile",
        "Apple_iPhone 14 Pro_24mm__1080p_16by9_1920x1080-25.00fps",
    )
    assert isinstance(chosen.lens, lenses.Fisheye)
    video.import_video(path, tmp_path / "out", chosen)
    frames, source = _imported(tmp_path / "out")
    pinhole, _ = lenses.undistortion(chosen.lens.scaled(854, 480))
    assert (frames.camera.width, frames.camera.height) == (pinhole.width, pinhole.height)
    assert pinhole.width < 854  # the lens's border cropped
    assert pinhole.height < 480
    np.testing.assert_allclose(
        frames.camera.intrinsics, [pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy]
    )
    assert images.read_rgb(frames.images[0]).shape == (pinhole.height, pinhole.width, 3)
    assert source["intrinsics"]["profile"] == chosen.profile
    assert video.main([str(path), str(tmp_path / "cli"), "--profiles", str(PROFILES)]) == 0
    assert video.main([str(path), str(tmp_path / "nominal"), "--source", "nominal"]) == 0
    empty = tmp_path / "no-profiles"
    empty.mkdir()
    with pytest.raises(InvalidInputError, match="no usable lens profile"):
        video.main([str(path), str(tmp_path / "auto"), "--profiles", str(empty)])
    assert (
        json.loads((tmp_path / "nominal" / "source.json").read_text())["intrinsics"]["source"]
        == "nominal"
    )


def test_only_the_named_metadata_is_kept(rendered: list[room.Frame], tmp_path: Path) -> None:
    """source.json records the make, model and lens, never the location or anything else."""
    path = encode(tmp_path / "tagged.mov", [rgb(rendered[0].gray)] * 2, metadata=APPLE)
    video.import_video(path, tmp_path / "out", lenses.Chosen(ROOM_CAMERA, "given"))
    _, source = _imported(tmp_path / "out")
    assert source["metadata"] == {key: APPLE[key] for key in video.METADATA_KEYS if key in APPLE}
    assert "ISO6709" not in (tmp_path / "out" / "source.json").read_text()


def test_an_imported_video_tracks_the_room(rendered: list[room.Frame], tmp_path: Path) -> None:
    """The room, imported from a video, tracks as close to the truth as the room's own run."""
    path = encode(tmp_path / "room.mov", [rgb(f.gray) for f in rendered])
    video.import_video(path, tmp_path / "room_video", lenses.Chosen(ROOM_CAMERA, "given"))
    run = vo.run("tum", tmp_path / "room_video", "monocular", tmp_path / "run", options=OPTIONS)
    truth = [f.timestamp for f in rendered], np.array([f.T_world_cam for f in rendered])
    assert runs.evaluate(run, *truth).absolute.rmse < ATE_BOUND["monocular"]


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ("long", "longer than 60 s"),
        ("big", "over the cap"),
        ("size", "given for 240x320"),
        ("turn", "multiple of 90"),
        ("used", "new or empty folder"),
        ("text", "not a readable video"),
        ("matroska", "not a readable video"),
        ("wide", "more than 3 to 1"),
        ("huge", "over 200 pixels long"),
        ("many", "more than 5 frames"),
    ],
)
def test_a_video_is_refused_and_leaves_nothing(
    change: str, message: str, rendered: list[room.Frame], tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:  # fmt: skip
    """Over a cap, the wrong size, an odd rotation, a used folder, not an MP4: refused.

    A Matroska video is a video, but not of the one container family an upload is opened as.
    """
    pictures = [rgb(rendered[0].gray)] * 2
    path, camera, out = tmp_path / "in.mov", ROOM_CAMERA, tmp_path / "outs" / "out"
    if change == "long":
        encode(path, [np.zeros((32, 32, 3), np.uint8)] * 62, rate=1)
        camera = lenses.Pinhole(30.0, 30.0, 15.5, 15.5, 32, 32)
    elif change == "turn":
        encode(path, pictures, rotation=45)
    elif change == "text":
        path.write_text("not a video")
    elif change == "matroska":
        _encode_matroska(path, pictures)
    elif change == "wide":
        encode(path, [np.zeros((32, 128, 3), np.uint8)] * 2)
        camera = lenses.Pinhole(60.0, 60.0, 63.5, 15.5, 128, 32)
    elif change == "many":
        encode(path, pictures * 3)
    else:
        encode(path, pictures)
    if change == "huge":
        monkeypatch.setattr(video, "MAX_LONG_SIDE", 200)
    limits = replace(video.UPLOAD, max_frames=5) if change == "many" else video.UPLOAD
    if change == "big":
        monkeypatch.setattr(video, "MAX_BYTES", 100)
    if change == "size":
        camera = ROOM_CAMERA.turned(1)
    if change == "used":
        out.mkdir(parents=True)
        (out / "kept.txt").write_text("not ours")
    with pytest.raises(InvalidInputError, match=message):
        video.import_video(path, out, lenses.Chosen(camera, "given"), limits=limits)
    left = sorted(p.name for p in (tmp_path / "outs").iterdir()) if out.parent.exists() else []
    assert left == (["out"] if change == "used" else [])


def _encode_matroska(path: Path, pictures: list[np.ndarray]) -> None:
    """Encode pictures as a lossless H.264 video in a Matroska container."""
    with av.open(str(path), "w", format="matroska") as out:
        stream = out.add_stream("libx264", rate=30, options={"qp": "0"})
        stream.height, stream.width = pictures[0].shape[:2]
        stream.pix_fmt = "yuv420p"
        for picture in pictures:
            for packet in stream.encode(av.VideoFrame.from_ndarray(picture, format="rgb24")):
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)


def _annex_b(size: tuple[int, int], count: int) -> bytes:
    """Return a raw H.264 stream of gray frames at one size, its headers before each keyframe."""
    width, height = size
    buffer = io.BytesIO()
    with av.open(buffer, "w", format="h264") as out:
        options = {"qp": "0", "bf": "0", "x264-params": "repeat-headers=1"}
        stream = out.add_stream("libx264", rate=30, options=options)
        stream.width, stream.height, stream.pix_fmt = width, height, "yuv420p"
        for _ in range(count):
            picture = np.full((height, width, 3), 128, np.uint8)
            for packet in stream.encode(av.VideoFrame.from_ndarray(picture, format="rgb24")):
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)
    return buffer.getvalue()


def _encode_two_sizes(path: Path, first: tuple[int, int], then: tuple[int, int]) -> None:
    """Write an MP4 whose H.264 stream has two frames at one size, then two at another."""
    raw = path.with_suffix(".h264")
    raw.write_bytes(_annex_b(first, 2) + _annex_b(then, 2))
    with av.open(str(raw), format="h264") as source, av.open(str(path), "w", format="mp4") as out:
        stream = out.add_stream_from_template(source.streams.video[0])
        packets = (p for p in source.demux(source.streams.video[0]) if p.size)
        for index, packet in enumerate(packets):
            packet.stream, packet.pts, packet.dts = stream, index, index
            packet.time_base = Fraction(1, 30)
            out.mux(packet)


def test_a_video_whose_frames_change_size_is_refused(tmp_path: Path) -> None:
    """Refused at the first frame of another size, which the header and the caps never see."""
    path, out = tmp_path / "in.mp4", tmp_path / "outs" / "out"
    _encode_two_sizes(path, (320, 240), (640, 240))
    assert (video.probe(path).width, video.probe(path).height) == (320, 240)
    with pytest.raises(InvalidInputError, match="frame 2 is 640x240, the first 320x240"):
        video.import_video(path, out, lenses.Chosen(ROOM_CAMERA, "given"))
    assert not out.parent.exists() or not any(out.parent.iterdir())


DISTORTED = lenses.RadTan(ROOM_CAMERA, (0.05, 0.03, -0.001, 0.002))
"""The room's camera with a radial-tangential lens, as a dataset's calibration gives one."""


def test_a_dataset_video_keeps_its_lens_and_takes_its_clock(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Frames written as recorded, ``camera.txt`` radtan; each frame timed by the dataset's clock.

    At 60 fps every other frame is kept, stamped with the clock's time; with a smaller short
    side the lens is scaled, its distortion kept.
    """
    path = encode(tmp_path / "room.mov", [rgb(f.gray) for f in rendered[:6]], rate=60)
    clock = [100.0 + i / 60 for i in range(6)]
    chosen = lenses.Chosen(DISTORTED, "given")
    video.import_video(path, tmp_path / "out", chosen, clock=clock)
    frames, source = _imported(tmp_path / "out")
    np.testing.assert_allclose(frames.timestamps, clock[::2], atol=1e-6)
    assert (frames.camera.model, frames.camera.width, frames.camera.height) == ("radtan", 320, 240)
    np.testing.assert_allclose(frames.camera.intrinsics, DISTORTED.intrinsics())
    for file, frame in zip(frames.images, rendered[::2], strict=False):
        gray = images.read_gray(file).astype(int)
        assert np.abs(gray - frame.gray).max() <= 2  # as recorded: not undistorted
    assert source["intrinsics"]["distortion"] == list(DISTORTED.distortion)
    small = video.Limits(max_seconds=600.0, max_frames=10, short_side=120)
    video.import_video(path, tmp_path / "small", chosen, limits=small, clock=clock)
    camera = tum.read_camera(tmp_path / "small")
    assert (camera.model, camera.width, camera.height) == ("radtan", 160, 120)
    np.testing.assert_allclose(camera.intrinsics, DISTORTED.scaled(160, 120).intrinsics())


@pytest.mark.parametrize(
    ("count", "message"), [(5, "more frames than the 5 timed"), (7, "6 frames, 7 timed")]
)
def test_a_clock_that_does_not_time_each_frame_once_is_refused(
    count: int, message: str, rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Fewer times than frames, or more."""
    path = encode(tmp_path / "room.mov", [rgb(f.gray) for f in rendered[:6]])
    clock = [float(i) for i in range(count)]
    with pytest.raises(InvalidInputError, match=message):
        video.import_video(path, tmp_path / "out", lenses.Chosen(DISTORTED, "given"), clock=clock)


@pytest.mark.parametrize(
    "changes", [{"max_seconds": 0.0}, {"max_frames": 0}, {"short_side": 0}, {"max_seconds": -1.0}]
)
def test_limits_that_would_take_nothing_are_refused(changes: dict[str, Any]) -> None:
    """A video of no length, no frames, or frames of no size."""
    with pytest.raises(InvalidInputError, match="expected each positive"):
        replace(video.UPLOAD, **changes)


@pytest.mark.parametrize(("divider", "kept"), [(1, [0, 2, 4, 6]), (2, list(range(8)))])
def test_a_stray_scanner_recording_imports_with_its_truth(
    divider: int, kept: list[int], rendered: list[room.Frame], tmp_path: Path
) -> None:
    """The camera matrix's intrinsics, and the kept frames' poses, as written, as the truth.

    The zip is written as Stray Scanner's encoders write one: HEVC with B-frames, its rotations
    already in our camera's axes, and its video's first frame behind an edit list, which the
    import decodes. At its default 60 fps the import keeps every other frame, and each keeps
    its own pose; at its 30 fps setting, every frame.
    """
    recording = stray_zip(tmp_path / "room.zip", rendered[:8], divider=divider)
    video.import_stray(recording, tmp_path / "out")
    frames, source = _imported(tmp_path / "out")
    assert source["frames"] == {"decoded": 8, "kept": len(kept)}
    np.testing.assert_allclose(
        frames.camera.intrinsics, [ROOM_CAMERA.fx, ROOM_CAMERA.fy, ROOM_CAMERA.cx, ROOM_CAMERA.cy]
    )
    times, poses = runs.read_trajectory(tmp_path / "out" / "groundtruth.txt")
    np.testing.assert_array_equal(times, frames.timestamps)
    np.testing.assert_allclose(times, np.array(kept) * divider / 60, atol=1e-6)
    np.testing.assert_allclose(poses, [rendered[i].T_world_cam for i in kept], atol=1e-9)
    assert (source["kind"], source["intrinsics"]["source"]) == ("stray", "stray-scanner")
    assert sorted(p.name for p in (tmp_path / "out").iterdir()) == [
        "camera.txt", "groundtruth.txt", "rgb", "rgb.txt", "source.json",
    ]  # fmt: skip


def _refused(recording: Path, out: Path, message: str) -> None:
    """Import a zip that must be refused, and check the refusal left nothing beside `out`."""
    with pytest.raises(InvalidInputError, match=message):
        video.import_stray(recording, out)
    assert list(out.parent.iterdir()) == []


def _matrix_with(cell: str) -> str:
    """Return a camera matrix with its top-left cell replaced."""
    return f"{cell},0.0,159.5\n0.0,262.5,119.5\n0.0,0.0,1.0\n"


@pytest.mark.parametrize(
    ("files", "names", "message"),
    [
        ({}, ["../rgb.mp4"], "leaves its folder"),
        ({}, ["/tmp/rgb.mp4"], "leaves its folder"),
        ({"camera_matrix.csv": "1," * 3000}, [], "over its cap"),
        ({"camera_matrix.csv": _matrix_with("fx")}, [], "expected numbers"),
        ({"camera_matrix.csv": "262.5,1.0,159.5\n0.0,262.5,119.5\n0.0,0.0,1.0\n"}, [], "fx 0 cx"),
        ({"odometry.csv": "timestamp, frame, x\n0.0, 0, 1.0\n"}, [], "no column y"),
        ({"odometry.csv": "timestamp, frame, x, y, z, qx, qy, qz, qw\n"}, [], "rows for the"),
        ({"odometry.csv": '"' + "1" * 200_000 + '"\n'}, [], r"odometry\.csv: not a CSV"),
    ],
)
def test_a_hostile_or_malformed_zip_is_refused_and_leaves_nothing(
    files: dict[str, str], names: list[str], message: str, rendered: list[room.Frame],
    tmp_path: Path,
) -> None:  # fmt: skip
    """Names leaving the folder, oversized entries, malformed CSVs, rows not per frame."""
    recording = stray_zip(tmp_path / "bad.zip", rendered[:4], files=files, names=names)
    _refused(recording, tmp_path / "outs" / "out", message)


def test_a_zip_that_cannot_give_an_entry_back_whole_is_refused(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A byte changed inside a stored entry fails its CRC: refused with the import's message."""
    recording = stray_zip(tmp_path / "room.zip", rendered[:3])
    with zipfile.ZipFile(recording) as archive:
        info = next(i for i in archive.infolist() if i.filename.endswith("camera_matrix.csv"))
        start = info.header_offset + 30 + len(info.filename.encode()) + len(info.extra)
    data = bytearray(recording.read_bytes())
    data[start] ^= 0x01  # stored uncompressed: the entry's first byte
    recording.write_bytes(bytes(data))
    with pytest.raises(InvalidInputError, match=r"camera_matrix\.csv cannot be read"):
        video.import_stray(recording, tmp_path / "out")


def test_a_zip_missing_a_file_or_holding_one_twice_is_refused(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Every one of the three files, exactly once; and a zip at all."""
    complete = stray_zip(tmp_path / "complete.zip", rendered[:3])
    with zipfile.ZipFile(complete) as archive:
        kept = {i.filename: archive.read(i) for i in archive.infolist()}
    missing = tmp_path / "missing.zip"
    with zipfile.ZipFile(missing, "w") as archive:
        for name, body in kept.items():
            if not name.endswith("odometry.csv"):
                archive.writestr(name, body)
    with pytest.raises(InvalidInputError, match=r"no odometry\.csv"):
        video.import_stray(missing, tmp_path / "a")
    twice = tmp_path / "twice.zip"
    matrix = next(name for name in kept if name.endswith("camera_matrix.csv"))
    with zipfile.ZipFile(twice, "w") as archive:
        for name, body in kept.items():
            archive.writestr(name, body)
        with pytest.warns(UserWarning, match="Duplicate name"):
            archive.writestr(matrix, b"1")
    with pytest.raises(InvalidInputError, match="appears twice"):
        video.import_stray(twice, tmp_path / "b")
    not_zip = tmp_path / "not.zip"
    not_zip.write_bytes(b"plain")
    with pytest.raises(InvalidInputError, match="not a zip"):
        video.import_stray(not_zip, tmp_path / "c")


MEDIUM_ROWS, MEDIUM_COLUMNS = 60, 80
"""The frame's pixels whose LiDAR pixel is in :data:`tests.eval.recordings.MEDIUM_BLOCK`: its
48x64 block of a 256x192 map, at 320x240."""


def _depth_error(imported: np.ndarray, metres: np.ndarray) -> float:
    """Return an imported depth map's median distance from the room's where confidence is high.

    In metres.
    """
    high = np.ones(metres.shape, bool)
    high[:MEDIUM_ROWS, :MEDIUM_COLUMNS] = False
    return float(np.median(np.abs(imported[high] / lidar.DEPTH_SCALE - metres[high])))


@pytest.mark.parametrize("threshold", [lidar.HIGH, lidar.MEDIUM])
def test_a_stray_scanner_recording_with_depth_imports_as_rgbd(
    threshold: int, rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Each kept frame's LiDAR maps, kept by confidence and brought to its size: an RGB-D folder.

    The depth is the room's, in millimetres, to the maps' resolution; the camera takes LiDAR's
    depth scale and ``bf``. Only the kept frames' maps are read, so the others may be missing.
    """
    skipped = {f"{kind}/{i:06d}.png": None for kind in ("depth", "confidence") for i in (1, 3)}
    recording = stray_zip(tmp_path / "room.zip", rendered[:4], depth=True, files=skipped)
    video.import_stray(recording, tmp_path / "out", threshold)
    frames, source = _imported(tmp_path / "out")
    assert source["depth"] == {"min_confidence": threshold}
    assert frames.camera.depth_scale == lidar.DEPTH_SCALE
    assert frames.camera.bf == pytest.approx(lidar.bf(ROOM_CAMERA.fx))
    for path, kept in zip(frames.depths, (0, 2), strict=True):  # 60 fps: 1 in 2
        assert path is not None
        imported = images.read_depth(path)
        expected = lidar.registered_depth(*stray_maps(rendered[kept]), (320, 240), threshold)
        np.testing.assert_array_equal(imported, expected)
        assert _depth_error(imported, rendered[kept].depth) < 0.01
        medium = imported[:MEDIUM_ROWS, :MEDIUM_COLUMNS]
        assert np.all(medium > 0) if threshold == lidar.MEDIUM else np.all(medium == 0)


def test_stray_depth_turns_upright_with_its_frame(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A video declaring a rotation is turned upright, and its LiDAR maps are turned with it.

    Whichever quarter turn brings the room's image to the imported frame brings the room's
    depth, and the maps' block of medium confidence, to the imported depth.
    """
    recording = stray_zip(tmp_path / "room.zip", rendered[:1], depth=True, rotation=90)
    video.import_stray(recording, tmp_path / "out")
    frames, _ = _imported(tmp_path / "out")
    gray = images.read_gray(frames.images[0])
    assert gray.shape == (320, 240)
    turns = min(
        (1, 3), key=lambda k: np.abs(gray - np.rot90(rendered[0].gray, k).astype(float)).mean()
    )
    assert frames.depths[0] is not None
    upright = np.rot90(images.read_depth(frames.depths[0]), -turns)  # back as stored
    assert _depth_error(upright, rendered[0].depth) < 0.01
    assert np.all(upright[:MEDIUM_ROWS, :MEDIUM_COLUMNS] == 0)


def _png(*sizes: tuple[int, int]) -> bytes:
    """Return a 16-bit gray PNG with a header chunk for each size, then a few pixels' data.

    Pillow takes the last header's size, so a map may declare one size first and decode at
    another; the data is never reached, as the size is refused once the headers are parsed.
    """

    def chunk(kind: bytes, body: bytes) -> bytes:
        return (
            len(body).to_bytes(4, "big") + kind + body + zlib.crc32(kind + body).to_bytes(4, "big")
        )

    headers = b"".join(
        chunk(b"IHDR", w.to_bytes(4, "big") + h.to_bytes(4, "big") + bytes([16, 0, 0, 0, 0]))
        for w, h in sizes
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + headers
        + chunk(b"IDAT", zlib.compress(bytes(16)))
        + chunk(b"IEND", b"")
    )


@pytest.mark.parametrize(
    ("files", "names", "message"),
    [
        ({"confidence/000002.png": None}, [], r"no confidence/000002\.png for frame 2"),
        (
            {"depth/000000.png": b"not a png"},
            [],
            r"depth/000000\.png: image: not a readable image \(not a PNG",
        ),
        ({"depth/000000.png": _png((2000, 1500))}, [], "2000x1500, over 1024 pixels"),
        ({"depth/000000.png": _png((256, 192), (2000, 1500))}, [], "2000x1500, over 1024 pixels"),
        (
            {"confidence/000000.png": b"%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: 0 0 256 192\n"},
            [],
            r"confidence/000000\.png: image: not a readable image \(not a PNG\)",
        ),
        ({"depth/000000.png": b"\0" * (2**20 + 1)}, [], "over its cap"),
        ({}, ["depth/000002.png"], r"depth/000002\.png appears twice"),
        (
            {"depth/000000.png": png(np.zeros((192, 256), np.uint8))},
            [],
            r"depth/000000\.png: image: expected a 16-bit",
        ),
        (
            {"confidence/000000.png": png(np.zeros((192, 256), np.uint16))},
            [],
            r"confidence/000000\.png: image: expected an 8-bit",
        ),
        (
            {
                "depth/000002.png": png(np.zeros((100, 100), np.uint16)),
                "confidence/000002.png": png(np.zeros((100, 100), np.uint8)),
            },
            [],
            "frame 2's LiDAR maps: .* aspect",
        ),
    ],
)
def test_lidar_maps_missing_or_malformed_refuse_the_zip_and_leave_nothing(
    files: dict[str, bytes | None], names: list[str], message: str, rendered: list[room.Frame],
    tmp_path: Path,
) -> None:  # fmt: skip
    """Each refusal of a zip's LiDAR maps, with the import leaving nothing.

    A kept frame without a map; a map not a PNG, too large, over its cap, there twice, of the
    wrong bit depth, or not its frame's aspect.
    """
    recording = stray_zip(tmp_path / "bad.zip", rendered[:4], depth=True, files=files, names=names)
    _refused(recording, tmp_path / "outs" / "out", message)


@pytest.mark.parametrize("stem", ["00000\u00b2", "\u0660\u0660\u0660\u0660\u0660\u0661"])
def test_a_map_named_with_other_than_six_ascii_digits_is_not_a_map(
    stem: str, rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A superscript two, or Arabic-Indic digits, never name a frame: the entry is ignored."""
    folder = "2026-10-10--12-00-00/"
    recording = stray_zip(tmp_path / "room.zip", rendered[:3], names=[f"{folder}depth/{stem}.png"])
    video.import_stray(recording, tmp_path / "out")
    _, source = _imported(tmp_path / "out")
    assert source["depth"] is None


def test_an_oversized_map_is_named_as_we_name_it_not_as_the_zip_does(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A map's folder is free text: a newline in it never reaches the refusal."""
    recording = stray_zip(tmp_path / "room.zip", rendered[:3], depth=True)
    with zipfile.ZipFile(recording, "a") as archive:
        archive.writestr("forged\nline/depth/000004.png", b"\0" * (2**20 + 1))
    with pytest.raises(
        InvalidInputError, match=r"zip: depth/000004\.png is over its cap"
    ) as raised:
        video.import_stray(recording, tmp_path / "out")
    assert "\n" not in str(raised.value)


def test_main_imports_and_says_what(
    rendered: list[room.Frame], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The program imports a video with the intrinsics given, or a Stray Scanner zip."""
    path = encode(tmp_path / "room.mov", [rgb(f.gray) for f in rendered[:3]])
    c = room.SMALL
    given = [str(v) for v in (c.fx, c.fy, c.cx, c.cy)]
    assert video.main([str(path), str(tmp_path / "a"), "--intrinsics", *given]) == 0
    recording = stray_zip(tmp_path / "room.zip", rendered[:3])
    assert video.main([str(recording), str(tmp_path / "b"), "--stray"]) == 0
    with_depth = stray_zip(tmp_path / "depth.zip", rendered[:3], depth=True)
    command = [str(with_depth), str(tmp_path / "c"), "--stray", "--min-confidence", "1"]
    assert video.main(command) == 0
    assert capsys.readouterr().out.splitlines() == [
        f"{tmp_path / 'a'}: 3 frames, 320x240, intrinsics from given",
        f"{tmp_path / 'b'}: 2 frames, 320x240, intrinsics from stray-scanner",  # 60 fps: 1 in 2
        f"{tmp_path / 'c'}: 2 frames, 320x240, intrinsics from stray-scanner, "
        "LiDAR depth of confidence 1 up",
    ]
    with pytest.raises(SystemExit):
        video.main([str(path), str(tmp_path / "d"), "--min-confidence", "1"])


RAYS = np.array([(0.0, 0.0), (-0.45, -0.22), (0.4, 0.2), (-0.3, 0.25), (0.47, -0.25), (0.15, -0.1)])
"""Rays through the iPhone 14 Pro profile's view, as pinhole coordinates (x/z, y/z)."""


def _turn_rays(rays: np.ndarray, turns: int) -> np.ndarray:
    """Return rays in the camera of the image turned counter-clockwise by quarter turns.

    Turning the image a quarter counter-clockwise turns the camera with it: a ray at (a, b)
    is then at (b, -a).
    """
    a, b = rays[:, 0], rays[:, 1]
    for _ in range(turns % 4):
        a, b = b, -a
    return np.stack([a, b], axis=-1)


@pytest.mark.parametrize("rotation", [90, -90, 180])
def test_imported_frames_show_each_ray_where_their_camera_projects_it(
    rotation: int, tmp_path: Path
) -> None:
    """Scaled, undistorted and turned, a frame shows a ray where the written camera puts it.

    Blobs are drawn at known rays through a fisheye profile at 1920x1080, stored with a
    declared rotation (an iPhone's portrait clip declares -90), then imported; each blob's
    centre must be where camera.txt projects its ray, turned as the frame was, within 0.1 px.
    """
    profile = lenses.read_profile(
        PROFILES / "Apple_iPhone 14 Pro_24mm__1080p_16by9_1920x1080-25.00fps.json"
    )
    assert profile is not None
    lens = profile.fisheye
    sources = lens.source(RAYS[:, 0], RAYS[:, 1])
    ys, xs = np.mgrid[0 : lens.camera.height, 0 : lens.camera.width].astype(np.float64)
    drawn = np.full(xs.shape, 20.0)
    for x0, y0 in sources:
        drawn += 200.0 * np.exp(-((xs - x0) ** 2 + (ys - y0) ** 2) / (2 * 7.0**2))
    picture = rgb(np.clip(np.round(drawn), 0, 255).astype(np.uint8))
    path = encode(tmp_path / "rays.mov", [picture, picture], rotation=rotation)
    video.import_video(path, tmp_path / "out", lenses.Chosen(lens, "profile", profile.name))
    frames = tum.read_frames(tmp_path / "out")
    fx, fy, cx, cy = frames.camera.intrinsics
    shown = images.read_gray(frames.images[0]).astype(np.float64)
    shown = np.clip(shown - np.median(shown), 0.0, None)
    turned = _turn_rays(RAYS, rotation // 90)
    expected = np.stack([fx * turned[:, 0] + cx, fy * turned[:, 1] + cy], axis=-1)
    ys, xs = np.mgrid[0 : shown.shape[0], 0 : shown.shape[1]].astype(np.float64)
    for x0, y0 in expected:
        weights = shown * ((np.abs(xs - x0) < 12) & (np.abs(ys - y0) < 12))
        centre = np.array([(weights * xs).sum(), (weights * ys).sum()]) / weights.sum()
        assert np.hypot(*(centre - (x0, y0))) < 0.1
