"""Importing phone recordings: the room as synthetic videos and Stray Scanner zips, and refusals.

The videos are encoded here with PyAV, losslessly (x264 at qp 0), so an imported frame is the
rendered one up to the colour conversion's rounding.
"""

from __future__ import annotations

import json
import zipfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import av
import numpy as np
import pytest
from numpy.typing import NDArray

from slam_it_all.eval import images, lenses, runs, tum, video
from slam_it_all.foundations import so3
from slam_it_all.foundations.errors import InvalidInputError
from slam_it_all.orchestration import vo
from tests.conftest import FIXTURES
from tests.edge import room
from tests.edge.support import ATE_BOUND, FRAMES, OPTIONS

PROFILES = FIXTURES / "eval" / "lens_profiles"
"""The fixture lens profiles (their README names them)."""

ROOM_CAMERA = lenses.Pinhole(
    room.SMALL.fx, room.SMALL.fy, room.SMALL.cx, room.SMALL.cy, room.SMALL.width,
    room.SMALL.height,
)  # fmt: skip
"""The room's camera, as a pinhole."""

APPLE = {
    "com.apple.quicktime.make": "Apple",
    "com.apple.quicktime.model": "iPhone 14 Pro",
    "com.apple.quicktime.camera.lens_model": "iPhone 14 Pro back camera 6.86mm f/1.78",
    "com.apple.quicktime.location.ISO6709": "+52.5163+013.3777+034.000/",
    "com.apple.quicktime.software": "17.0",
}
"""Apple-style tags, a location among them."""


@pytest.fixture(scope="module")
def rendered() -> list[room.Frame]:
    """Render the room at 30 frames a second."""
    return room.sequence(FRAMES)


def _rgb(gray: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return a gray image as RGB."""
    return np.ascontiguousarray(np.repeat(gray[:, :, None], 3, axis=2))


def _encode(
    path: Path,
    pictures: list[NDArray[np.uint8]],
    *,
    rate: int = 30,
    rotation: int = 0,
    metadata: dict[str, str] | None = None,
) -> Path:
    """Encode RGB pictures as a lossless H.264 video, with a declared rotation and tags."""
    options = {"movflags": "use_metadata_tags"} if metadata else {}
    with av.open(str(path), "w", format="mov", options=options) as out:
        for key, value in (metadata or {}).items():
            out.metadata[key] = value
        stream = out.add_stream("libx264", rate=rate, options={"qp": "0"})
        stream.height, stream.width = pictures[0].shape[:2]
        stream.pix_fmt = "yuv420p"
        if rotation:
            stream.set_display_rotation(rotation)
        for picture in pictures:
            for packet in stream.encode(av.VideoFrame.from_ndarray(picture, format="rgb24")):
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)
    return path


def _imported(folder: Path) -> tuple[tum.Frames, dict[str, Any]]:
    """Read an imported folder as a run does, and its source.json."""
    return tum.read_frames(folder), json.loads((folder / "source.json").read_text())


def test_a_video_on_its_side_imports_upright_with_its_camera(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A video stored on its side, its rotation declared, imports upright, its camera turned."""
    stored = [np.rot90(_rgb(f.gray), -1) for f in rendered[:6]]  # turned clockwise
    path = _encode(tmp_path / "side.mov", stored, rotation=90)  # shown turned back
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
    }  # fmt: skip


def test_sixty_frames_a_second_keeps_every_other(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """A 60 fps video keeps the frames 1/30 s apart."""
    path = _encode(tmp_path / "sixty.mov", [_rgb(f.gray) for f in rendered[:12]], rate=60)
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
        _encode(tmp_path / "big.mov", pictures), tmp_path / "out", lenses.Chosen(camera, "given")
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
    path = _encode(tmp_path / "phone.mov", pictures, metadata=APPLE)
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
    assert (
        json.loads((tmp_path / "nominal" / "source.json").read_text())["intrinsics"]["source"]
        == "nominal"
    )


def test_only_the_named_metadata_is_kept(rendered: list[room.Frame], tmp_path: Path) -> None:
    """source.json records the make, model and lens, never the location or anything else."""
    path = _encode(tmp_path / "tagged.mov", [_rgb(rendered[0].gray)] * 2, metadata=APPLE)
    video.import_video(path, tmp_path / "out", lenses.Chosen(ROOM_CAMERA, "given"))
    _, source = _imported(tmp_path / "out")
    assert source["metadata"] == {key: APPLE[key] for key in video.METADATA_KEYS if key in APPLE}
    assert "ISO6709" not in (tmp_path / "out" / "source.json").read_text()


def test_an_imported_video_tracks_the_room(rendered: list[room.Frame], tmp_path: Path) -> None:
    """The room, imported from a video, tracks as close to the truth as the room's own run."""
    path = _encode(tmp_path / "room.mov", [_rgb(f.gray) for f in rendered])
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
    ],
)
def test_a_video_is_refused_and_leaves_nothing(
    change: str, message: str, rendered: list[room.Frame], tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:  # fmt: skip
    """Over a cap, the wrong size, an odd rotation, a used folder, not a video: refused."""
    pictures = [_rgb(rendered[0].gray)] * 2
    path, camera, out = tmp_path / "in.mov", ROOM_CAMERA, tmp_path / "outs" / "out"
    if change == "long":
        _encode(path, [np.zeros((32, 32, 3), np.uint8)] * 62, rate=1)
        camera = lenses.Pinhole(30.0, 30.0, 15.5, 15.5, 32, 32)
    elif change == "turn":
        _encode(path, pictures, rotation=45)
    elif change == "text":
        path.write_text("not a video")
    else:
        _encode(path, pictures)
    if change == "big":
        monkeypatch.setattr(video, "MAX_BYTES", 100)
    if change == "size":
        camera = ROOM_CAMERA.turned(1)
    if change == "used":
        out.mkdir(parents=True)
        (out / "kept.txt").write_text("not ours")
    with pytest.raises(InvalidInputError, match=message):
        video.import_video(path, out, lenses.Chosen(camera, "given"))
    left = sorted(p.name for p in (tmp_path / "outs").iterdir()) if out.parent.exists() else []
    assert left == (["out"] if change == "used" else [])


def _stray_zip(
    path: Path,
    rendered: list[room.Frame],
    *,
    folder: str = "2026-10-10--12-00-00/",
    files: Mapping[str, bytes | str] | None = None,
    names: list[str] | None = None,
) -> Path:
    """Write the room as a Stray Scanner zip: the video, the camera matrix, ARKit's poses.

    `files` replaces or adds entries, by name in the folder; `names` adds entries named as
    given, with a small body.
    """
    video_bytes = _encode(path.with_suffix(".mp4"), [_rgb(f.gray) for f in rendered]).read_bytes()
    c = room.SMALL
    matrix = f"{c.fx},0.0,{c.cx}\n0.0,{c.fy},{c.cy}\n0.0,0.0,1.0\n"
    rows = ["timestamp, frame, x, y, z, qx, qy, qz, qw"]
    for i, f in enumerate(rendered):
        arkit = f.T_world_cam @ video.ARKIT_TO_CAMERA  # the diagonal is its own inverse
        values = [100.0 + f.timestamp, *arkit[:3, 3], *so3.to_quaternion(arkit[:3, :3])]
        x, y, z, qx, qy, qz, qw = (repr(float(v)) for v in values[1:])
        rows.append(f"{float(values[0])!r}, {i}, {x}, {y}, {z}, {qx}, {qy}, {qz}, {qw}")
    entries: dict[str, bytes | str] = {
        "rgb.mp4": video_bytes,
        "camera_matrix.csv": matrix,
        "odometry.csv": "\n".join(rows) + "\n",
        "depth/000000.png": b"not read",
    }
    entries.update(files or {})
    with zipfile.ZipFile(path, "w") as archive:
        for name, body in entries.items():
            archive.writestr(folder + name, body)
        for name in names or []:
            archive.writestr(name, b"x")
    return path


def test_a_stray_scanner_recording_imports_with_its_truth(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """The camera matrix's intrinsics, and ARKit's poses in our axes as the ground truth."""
    recording = _stray_zip(tmp_path / "room.zip", rendered[:8])
    video.import_stray(recording, tmp_path / "out")
    frames, source = _imported(tmp_path / "out")
    np.testing.assert_allclose(
        frames.camera.intrinsics, [ROOM_CAMERA.fx, ROOM_CAMERA.fy, ROOM_CAMERA.cx, ROOM_CAMERA.cy]
    )
    times, poses = runs.read_trajectory(tmp_path / "out" / "groundtruth.txt")
    np.testing.assert_array_equal(times, frames.timestamps)
    np.testing.assert_allclose(poses, [f.T_world_cam for f in rendered[:8]], atol=1e-9)
    assert (source["kind"], source["intrinsics"]["source"]) == ("stray", "stray-scanner")
    assert sorted(p.name for p in (tmp_path / "out").iterdir()) == [
        "camera.txt", "groundtruth.txt", "rgb", "rgb.txt", "source.json",
    ]  # fmt: skip


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
    ],
)
def test_a_hostile_or_malformed_zip_is_refused_and_leaves_nothing(
    files: dict[str, str], names: list[str], message: str, rendered: list[room.Frame],
    tmp_path: Path,
) -> None:  # fmt: skip
    """Names leaving the folder, oversized entries, malformed CSVs, rows not per frame."""
    recording = _stray_zip(tmp_path / "bad.zip", rendered[:4], files=files, names=names)
    with pytest.raises(InvalidInputError, match=message):
        video.import_stray(recording, tmp_path / "outs" / "out")
    assert list((tmp_path / "outs").iterdir()) == []


def test_a_zip_missing_a_file_or_holding_one_twice_is_refused(
    rendered: list[room.Frame], tmp_path: Path
) -> None:
    """Every one of the three files, exactly once; and a zip at all."""
    complete = _stray_zip(tmp_path / "complete.zip", rendered[:3])
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


def test_main_imports_and_says_what(
    rendered: list[room.Frame], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The program imports a video with the intrinsics given, or a Stray Scanner zip."""
    path = _encode(tmp_path / "room.mov", [_rgb(f.gray) for f in rendered[:3]])
    c = room.SMALL
    given = [str(v) for v in (c.fx, c.fy, c.cx, c.cy)]
    assert video.main([str(path), str(tmp_path / "a"), "--intrinsics", *given]) == 0
    recording = _stray_zip(tmp_path / "room.zip", rendered[:3])
    assert video.main([str(recording), str(tmp_path / "b"), "--stray"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        f"{tmp_path / 'a'}: 3 frames, 320x240, intrinsics from given",
        f"{tmp_path / 'b'}: 3 frames, 320x240, intrinsics from stray-scanner",
    ]
