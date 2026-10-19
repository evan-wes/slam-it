"""The dashcam steps from the command line: ``python -m slam_it.eval.dashcam <step> CONFIG``.

Each step reads the config (:mod:`.config`) and writes into its work folder, a folder a drive,
named by the drive's id (its first front segment's start, ``YYYYMMDD_HHMMSS``):

- ``drives``: list the card's drives, their ids, spans and segments.
- ``mask``: measure a camera's static mask from a drive (:mod:`.mask`), add the config's drawn
  parts, and write it to the config's mask path, with a preview in the work folder.
- ``overlay``: read each camera's overlay over a drive (:mod:`.overlay`, Tesseract): the GPS
  fixes and the camera's lag, ``overlay_F.json`` and ``overlay_R.json``.
- ``calibrate``: calibrate the rig over a drive, or a window of it (:mod:`.calibrate`, COLMAP):
  ``calibration.json``, whose lenses the config's ``[front.lens]`` and ``[rear.lens]`` take.
- ``import``: import one camera's share of a drive, or a window of it, as a TUM-format sequence
  (:mod:`.ingest`), with the config's lens and mask and the overlay's lag.

``mask``, ``overlay`` and ``calibrate`` refuse to replace what they wrote before. Each step
checks everything it would write, and the earlier steps' outputs it reads, before it writes
anything, so a refusal leaves nothing half done. Every step skips a damaged segment with a
warning and goes on (:func:`.drives.skip`); ``overlay`` and ``import`` list those skipped in
what they write and print.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from slam_it.eval import gnss, images, video
from slam_it.eval.dashcam import calibrate, config, drives, ingest, mask, overlay
from slam_it.foundations.errors import InvalidInputError

DRIVE_ID = "%Y%m%d_%H%M%S"
"""A drive's id: its first front segment's start."""


def _drive(settings: config.Config, wanted: str) -> tuple[str, drives.Drive]:
    """Return the drive with an id, and the id."""
    found = {
        d.front[0].start.strftime(DRIVE_ID): d
        for d in drives.find_drives(drives.scan(settings.root))
    }
    if wanted not in found:
        raise InvalidInputError(f"drive {wanted!r}: not on the card (see the drives step)")
    return wanted, found[wanted]


def _segments(
    drive: drives.Drive, camera: str, start: dt.datetime | None, end: dt.datetime | None
) -> tuple[drives.Segment, ...]:
    """Return a camera's segments of a drive, within a window."""
    chosen = drives.window(drive.segments(camera), start, end)
    if not chosen:
        raise InvalidInputError(f"camera {camera}: no segments in the window")
    return chosen


def _sized(segments: Sequence[drives.Segment]) -> tuple[drives.Segment, tuple[int, int]]:
    """Return the first segment that opens, and its frame size; the step warns of the rest."""
    for segment in segments:
        try:
            return segment, video.segment_size(segment.path)
        except video.DamagedSegmentError:
            continue
    raise InvalidInputError("segments: every one is damaged")


def _skipped(skipped: Sequence[tuple[str, str]]) -> str:
    """Return a summary's note of the damaged segments skipped, if any."""
    return (
        f", skipped {len(skipped)} damaged ({', '.join(n for n, _ in skipped)})" if skipped else ""
    )


def _fresh(*paths: Path) -> None:
    """Refuse if any of a step's outputs exists, then make the folders they go in.

    A step passes every output it writes, before it writes the first.
    """
    for path in paths:
        if path.exists():
            raise InvalidInputError(f"{path}: exists; move it aside to write it again")
    for path in paths:
        path.parent.mkdir(parents=True, exist_ok=True)


def _read_overlay(folder: Path, camera: str) -> dict[str, Any]:
    """Return a camera's overlay step's output."""
    path = folder / f"overlay_{camera}.json"
    if not path.is_file():
        raise InvalidInputError(f"{path}: missing; run the overlay step first")
    data: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    return data


def _track(fixes: Sequence[dict[str, Any]]) -> gnss.Track:
    """Return the overlay's fixes as a smoothed track, on the camera's clock."""
    lat = [f["latitude"] for f in fixes]
    lon = [f["longitude"] for f in fixes]
    return gnss.smooth([f["clock"] for f in fixes], gnss.local_frame(lat, lon))


def _list(settings: config.Config) -> None:
    """Print the card's drives."""
    for drive in drives.find_drives(drives.scan(settings.root)):
        minutes = (drive.end - drive.start).total_seconds() / 60
        print(
            f"{drive.front[0].start.strftime(DRIVE_ID)}  {drive.start} .. {drive.end} "
            f"({minutes:.0f} min of starts)  front {len(drive.front)}, rear {len(drive.rear)}"
        )


def _mask(settings: config.Config, args: argparse.Namespace) -> None:
    """Measure, finish and write a camera's mask, and its preview."""
    name, drive = _drive(settings, args.drive)
    camera = settings.camera(args.camera)
    segments = _segments(drive, args.camera, args.start, args.end)
    target = camera.mask
    shown = settings.work / name / f"mask_{args.camera}_preview.png"
    _fresh(target, shown)
    static = mask.measure([s.path for s in segments], every=args.every, share=args.share)
    first, (width, height) = _sized(segments)
    finished = mask.finish(static, width, height, bottom=camera.bottom, lines=camera.lines)
    _, frame = next(video.sample_frames(first.path, every=1.0, width=960))
    small = mask.scaled(finished, frame.shape[1], frame.shape[0])
    preview = np.where((small == 0)[..., None], frame // 3, frame).astype(np.uint8)
    images.write_png(target, finished)
    images.write_png(shown, preview)
    share = float((finished == 0).mean())
    print(f"{target}: {width}x{height}, {share:.1%} dropped, from {len(segments)} segments")


def _overlay(settings: config.Config, args: argparse.Namespace) -> None:
    """Read each camera's overlay over a drive, and write its fixes and lag."""
    name, drive = _drive(settings, args.drive)
    chosen = {camera: _segments(drive, camera, args.start, args.end) for camera in args.cameras}
    targets = {camera: settings.work / name / f"overlay_{camera}.json" for camera in chosen}
    _fresh(*targets.values())
    for camera, segments in chosen.items():
        target = targets[camera]
        read = overlay.read(segments, every=args.every, jobs=args.jobs)
        record = {
            "camera": camera,
            "lag": read.lag,
            "reads": read.reads,
            "kept": read.kept,
            "segments": [s.path.name for s in segments],
            "skipped": [{"segment": n, "reason": why} for n, why in read.skipped],
            "fixes": [asdict(f) for f in read.fixes],
        }
        target.write_text(json.dumps(record, indent=1) + "\n", encoding="utf-8")
        print(
            f"{target}: lag {read.lag:+.2f} s, {len(read.fixes)} fixes, "
            f"{read.kept} of {read.reads} reads kept" + _skipped(read.skipped)
        )


def _calibrate(settings: config.Config, args: argparse.Namespace) -> None:
    """Calibrate the rig over a drive, or a window of it."""
    name, drive = _drive(settings, args.drive)
    folder = settings.work / name
    target = folder / "calibration.json"
    work = folder / "calibration"
    chosen = {camera: _segments(drive, camera, args.start, args.end) for camera in ("F", "R")}
    sizes = {camera: _sized(segments)[1] for camera, segments in chosen.items()}
    overlays = {camera: _read_overlay(folder, camera) for camera in chosen}
    track = _track(overlays["F"]["fixes"])
    _fresh(target, work)
    found: dict[str, tuple[dict[float, calibrate.Array], calibrate.Lens]] = {}
    for camera, segments in chosen.items():
        lag = float(overlays[camera]["lag"])
        frames = work / camera / "frames"
        clocks = calibrate.extract_frames(segments, frames, lag=lag, fps=args.fps, width=args.width)
        width, height = images.size(frames / next(iter(clocks)))
        static = settings.camera(camera).mask
        keep = None
        if static.is_file():
            keep = mask.scaled(images.read_gray(static), width, height)
        poses, lens = calibrate.reconstruct(
            frames, work / camera / "colmap", mask=keep, threads=args.threads
        )
        found[camera] = (calibrate.by_time(poses, clocks), lens)
        print(
            f"{camera}: {lens.registered} of {lens.frames} frames placed, "
            f"{lens.error:.2f} px reprojection error, fx/fy {lens.aspect:.4f}"
        )
    report = calibrate.solve(
        found["F"][0],
        found["R"][0],
        (found["F"][1], found["R"][1]),
        track,
        baseline=settings.baseline,
    )
    result = report.as_dict(sizes["F"], sizes["R"])
    target.write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
    print(
        f"{target}: rear clock {result['rear_clock_offset_s']:+.2f} s "
        f"(r {result['offset_correlation']:.2f}), turn residual median "
        f"{result['turn_residual_deg']['median']:.2f} deg, baseline "
        f"{result['baseline_m']:.2f} m"
    )


def _import(settings: config.Config, args: argparse.Namespace) -> None:
    """Import one camera's share of a drive."""
    name, drive = _drive(settings, args.drive)
    camera = settings.camera(args.camera)
    if camera.lens is None:
        raise InvalidInputError(f"camera {args.camera}: no lens in the config; calibrate first")
    lag = float(_read_overlay(settings.work / name, args.camera)["lag"])
    keep = images.read_gray(camera.mask) if camera.mask.is_file() else None
    limits = settings.limits
    if args.max_frames is not None:
        limits = video.Limits(limits.max_seconds, args.max_frames, limits.short_side)
    segments = _segments(drive, args.camera, args.start, args.end)
    skipped = ingest.import_drive(
        segments, args.out, camera.lens, lag=lag, mask=keep, limits=limits
    )
    record = json.loads((args.out / "source.json").read_text(encoding="utf-8"))
    print(
        f"{args.out}: {record['frames']['kept']} frames from {len(segments) - len(skipped)} "
        "segments" + (", masked" if record["mask"] else ", no mask") + _skipped(skipped)
    )


def main(arguments: Sequence[str]) -> int:
    """Run a step from the command line (the module docstring).

    Parameters
    ----------
    arguments : sequence of str
        The arguments after the program's name.

    Returns
    -------
    int
        0, with a summary on stdout.
    """
    parser = argparse.ArgumentParser(prog="python -m slam_it.eval.dashcam")
    steps = parser.add_subparsers(dest="step", required=True)
    listing = steps.add_parser("drives", help="list the card's drives")
    listing.add_argument("config", type=Path, help="the config TOML")

    def step(name: str, text: str) -> argparse.ArgumentParser:
        sub = steps.add_parser(name, help=text)
        sub.add_argument("config", type=Path, help="the config TOML")
        sub.add_argument("--drive", required=True, help="the drive's id (the drives step)")
        sub.add_argument("--start", type=dt.datetime.fromisoformat, help="first segment start")
        sub.add_argument("--end", type=dt.datetime.fromisoformat, help="last segment start")
        return sub

    masking = step("mask", "measure a camera's static mask")
    masking.add_argument("--camera", choices=("F", "R"), required=True)
    masking.add_argument("--every", type=float, default=1.0, help="seconds between samples")
    masking.add_argument("--share", type=float, default=0.4, help="an edge's least share")
    reading = step("overlay", "read the overlays' fixes and clocks")
    reading.add_argument("--cameras", nargs="+", choices=("F", "R"), default=["F", "R"])
    reading.add_argument("--every", type=float, default=overlay.EVERY, help="seconds a read")
    reading.add_argument("--jobs", type=int, default=4, help="Tesseract processes")
    calibrating = step("calibrate", "calibrate the rig")
    calibrating.add_argument("--fps", type=float, default=calibrate.FPS)
    calibrating.add_argument("--width", type=int, default=calibrate.WIDTH)
    calibrating.add_argument("--threads", type=int, default=4, help="COLMAP's threads")
    importing = step("import", "import a camera's share as a sequence")
    importing.add_argument("--camera", choices=("F", "R"), required=True)
    importing.add_argument("out", type=Path, help="the TUM-format folder to write")
    importing.add_argument("--max-frames", type=int, help="widen the config's frame limit")
    args = parser.parse_args(arguments)
    settings = config.read(args.config)
    actions = {
        "mask": _mask,
        "overlay": _overlay,
        "calibrate": _calibrate,
        "import": _import,
    }
    if args.step == "drives":
        _list(settings)
    else:
        actions[args.step](settings, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
