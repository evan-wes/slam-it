"""Generate the sequence fixtures in ``sequences/`` and their expected frames, ``sequences.json``.

Run from the repository root after changing the cases (the fixtures are committed)::

    python -m tests.fixtures.eval.generate_sequences

Each folder holds a dataset's text files only (the readers list images, they do not open
them): a TUM folder named for its sensor, one describing its camera in ``camera.txt``, one
without depth maps, and a EuRoC folder whose right camera misses a frame. The expected values
are what the Python readers' run inputs give (:func:`slamlab.eval.tum.read_frames`,
:func:`slamlab.eval.euroc.read_frames`), paths relative to each folder; the C++ and Rust
readers are held to them exactly.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from slamlab.eval import euroc, tum

HERE = Path(__file__).parent
FOLDER = HERE / "sequences"

RGB = """# color images
# file: 'rgbd_dataset_freiburg1_fixture.bag'
# timestamp filename
1305031102.175304 rgb/1305031102.175304.png
1305031102.211214 rgb/1305031102.211214.png
1305031102.243211 rgb/1305031102.243211.png
1305031102.243211 rgb/1305031102.243211b.png
1305031102.311267 rgb/1305031102.311267.png
"""

DEPTH = """# depth maps
# timestamp filename
1305031102.160407 depth/1305031102.160407.png
1305031102.226738 depth/1305031102.226738.png
1305031102.262886 depth/1305031102.262886.png
1305031102.360224 depth/1305031102.360224.png
"""

CAMERA = """# a recording's camera
model pinhole
intrinsics 525.0 525.5 319.5 239.5   # fx fy cx cy
size 640 480
depth_scale 1000
bf 40.0
"""

MONOCULAR_CAMERA = CAMERA.replace("bf 40.0\n", "")
"""A camera.txt with no bf: a folder that runs monocular only."""

SENSOR = """# General sensor definitions.
sensor_type: camera
comment: VI-Sensor {name} (fixture)

# Sensor extrinsics wrt. the body-frame.
T_BS:
  cols: 4
  rows: 4
  data: [{pose}]

# Camera specific definitions.
rate_hz: 20
resolution: [752, 480]
camera_model: pinhole
intrinsics: [{intrinsics}] #fu, fv, cu, cv
distortion_model: radial-tangential
distortion_coefficients: [{distortion}]
"""

POSES = {
    "cam0": "0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,\n"
    "         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,\n"
    "        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,\n"
    "         0.0, 0.0, 0.0, 1.0",
    "cam1": "0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,\n"
    "         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,\n"
    "        -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,\n"
    "         0.0, 0.0, 0.0, 1.0",
}

INTRINSICS = {
    "cam0": (
        "458.654, 457.296, 367.215, 248.375",
        "-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05",
    ),
    "cam1": (
        "457.587, 456.134, 379.999, 255.238",
        "-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05",
    ),
}

STAMPS = [1403715273262142976 + k * 50_000_000 for k in range(5)]
"""The cameras' timestamps, in nanoseconds; cam1 misses the third."""


def _write_tum() -> None:
    """Write the three TUM folders."""
    named = FOLDER / "rgbd_dataset_freiburg1_fixture"
    recorded = FOLDER / "tum_camera"
    flat = FOLDER / "tum_no_depth"
    for folder in (named, recorded, flat):
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "rgb.txt").write_text(RGB)
    for folder in (named, recorded):
        (folder / "depth.txt").write_text(DEPTH)
    (recorded / "camera.txt").write_text(CAMERA)
    (flat / "camera.txt").write_text(MONOCULAR_CAMERA)


def _write_euroc() -> None:
    """Write the EuRoC folder: two cameras, cam1 missing one frame, no ground truth."""
    for name in ("cam0", "cam1"):
        camera = FOLDER / "euroc" / "mav0" / name
        camera.mkdir(parents=True, exist_ok=True)
        pinhole, distortion = INTRINSICS[name]
        (camera / "sensor.yaml").write_text(
            SENSOR.format(name=name, pose=POSES[name], intrinsics=pinhole, distortion=distortion)
        )
        rows = ["#timestamp [ns],filename"]
        rows += [f"{s},{s}.png" for k, s in enumerate(STAMPS) if not (name == "cam1" and k == 2)]
        (camera / "data.csv").write_text("\n".join(rows) + "\n")


def _relative(path: Path | None, root: Path) -> str | None:
    """Return a path relative to its folder, or None."""
    return None if path is None else path.relative_to(root).as_posix()


def _tum_case(name: str) -> dict[str, Any]:
    """Read a TUM folder with the Python reader."""
    frames = tum.read_frames(FOLDER / name)
    camera = frames.camera
    return {
        "timestamps": frames.timestamps.tolist(),
        "images": [_relative(p, frames.root) for p in frames.images],
        "depths": [_relative(p, frames.root) for p in frames.depths],
        "camera": {
            "model": camera.model,
            "intrinsics": camera.intrinsics.tolist(),
            "width": camera.width,
            "height": camera.height,
            "depth_scale": camera.depth_scale,
            "bf": camera.bf,
        },
    }


def _euroc_camera(camera: euroc.Camera) -> dict[str, Any]:
    """Return a EuRoC camera as JSON."""
    return {
        "intrinsics": camera.intrinsics.tolist(),
        "T_body_cam": camera.T_body_cam.tolist(),
        "rows": camera.shape[0],
        "cols": camera.shape[1],
    }


def main() -> None:
    """Write the folders and their expected frames."""
    _write_tum()
    _write_euroc()
    frames = euroc.read_frames(FOLDER / "euroc")
    cases = {
        "tum": {
            name: _tum_case(name)
            for name in ("rgbd_dataset_freiburg1_fixture", "tum_camera", "tum_no_depth")
        },
        "euroc": {
            "euroc": {
                "timestamps": frames.timestamps.tolist(),
                "left": [_relative(p, frames.root) for p in frames.left],
                "right": [_relative(p, frames.root) for p in frames.right],
                "left_camera": _euroc_camera(frames.left_camera),
                "right_camera": _euroc_camera(frames.right_camera),
            }
        },
    }
    (HERE / "sequences.json").write_text(json.dumps({"cases": cases}, indent=1) + "\n")


if __name__ == "__main__":
    main()
