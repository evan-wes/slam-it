"""Two-view initialisation on a real image pair: slamlab against the ground truth and OpenCV.

Takes two colour frames of a TUM RGB-D sequence (default: ``freiburg1_xyz``), matches SIFT
features between them (OpenCV, with Lowe's ratio test), unprojects the matches through the
sensor's calibrated radial-tangential model, and runs :func:`slamlab.foundations.twoview.
initialize` with each backend. It prints the status, the chosen model, the inliers and
triangulated points, and the motion's error against the motion-capture ground truth: the
rotation's angle and the translation direction's angle (two views cannot fix the scale).
OpenCV's ``findEssentialMat`` and ``recoverPose`` (the 5-point solver, RANSAC) run on the same
matches as an end-to-end reference.

With ``--rerun FILE.rrd`` it also saves a rerun recording of both cameras and the
triangulated points (open it with ``rerun FILE.rrd``).

Run from the repository root, in the slamlab environment::

    python scripts/demo_two_view.py
    python scripts/demo_two_view.py --first 100 --gap 20 --rerun /tmp/two_view.rrd

The data stays outside the repository (default ``../datasets/tum``); download the sequence
from https://cvg.cit.tum.de/data/datasets/rgbd-dataset and extract it there.
"""

from __future__ import annotations

import argparse
import collections
import math
import time
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.eval import tum
from slamlab.foundations import camera, epipolar, se3, so3, twoview

DEFAULT_SEQUENCE = Path("../datasets/tum/rgbd_dataset_freiburg1_xyz")
SIFT_FEATURES = 3000
"""SIFT keypoints detected per image."""
RATIO = 0.75
"""Lowe's ratio test: a match is kept if its distance is below this fraction of the
second-best match's."""


def matches(first: Path, second: Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Match SIFT features between two images, with Lowe's ratio test.

    Parameters
    ----------
    first, second : pathlib.Path
        The images.

    Returns
    -------
    tuple of ndarray of float64, shape (M, 2)
        The matched pixels in each image.

    Raises
    ------
    FileNotFoundError
        If an image cannot be read.
    """
    sift = cv2.SIFT.create(nfeatures=SIFT_FEATURES)
    images = []
    for path in (first, second):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"cannot read the image {path}")
        images.append(image)
    (k1, d1), (k2, d2) = (sift.detectAndCompute(image, None) for image in images)
    pairs = cv2.BFMatcher(cv2.NORM_L2).knnMatch(d1, d2, k=2)
    kept = [m for m, n in pairs if m.distance < RATIO * n.distance]
    p1 = np.array([k1[m.queryIdx].pt for m in kept], dtype=np.float64)
    p2 = np.array([k2[m.trainIdx].pt for m in kept], dtype=np.float64)
    return p1, p2


def errors_deg(found: NDArray[np.float64], truth: NDArray[np.float64]) -> tuple[float, float]:
    """Return the rotation error and the translation-direction error, in degrees.

    Parameters
    ----------
    found, truth : ndarray of float64, shape (4, 4)
        Relative poses ``T_c2_c1``.

    Returns
    -------
    tuple of float
        The angle of ``R_foundᵀ R_true``, and the angle between the unit translations (NaN if
        either translation is zero, as for a pure rotation).
    """
    rotation = math.degrees(float(np.linalg.norm(so3.log(found[:3, :3].T @ truth[:3, :3]))))
    lengths = np.linalg.norm(found[:3, 3]), np.linalg.norm(truth[:3, 3])
    if not (lengths[0] > 0.0 and lengths[1] > 0.0):
        return rotation, math.nan
    a, b = found[:3, 3] / lengths[0], truth[:3, 3] / lengths[1]
    direction = math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(a @ b)))
    return rotation, direction


FEW_MATCHES = 50
"""Below this many matches a pair's consistency with the ground truth says more about the
matching than about the motion capture."""


def truth_consistency(
    b1: NDArray[np.float64],
    b2: NDArray[np.float64],
    truth: NDArray[np.float64],
    options: twoview.Options,
) -> str:
    """Describe how well the ground truth's E explains the matches.

    The share of matches within the RANSAC threshold of the true E, and their median Sampson
    distance. Both measure the matching as much as the motion capture: on a pair with few
    matches, mostly outliers, the median is an outlier's distance. Such pairs are flagged.

    Parameters
    ----------
    b1, b2 : ndarray of float64, shape (M, 3)
        The matches' bearings.
    truth : ndarray of float64, shape (4, 4)
        The true ``T_c2_c1``.
    options : twoview.Options
        The focal length and threshold.

    Returns
    -------
    str
        E.g. ``"434 matches, 24% within 2 px of the truth's E (median 3.78 px)"``.
    """
    essential = so3.hat(truth[:3, 3] / np.linalg.norm(truth[:3, 3])) @ truth[:3, :3]
    distances = np.sqrt(epipolar.sampson_essential(essential, b1, b2)) * options.focal
    share = float(np.mean(distances < options.threshold_px))
    text = (
        f"{len(distances)} matches, {100 * share:.0f}% within {options.threshold_px:g} px of "
        f"the truth's E (median {float(np.median(distances)):.2f} px)"
    )
    return text + (
        " -- few matches, not a ground-truth measure" if len(distances) < FEW_MATCHES else ""
    )


def opencv_reference(
    p1: NDArray[np.float64],
    p2: NDArray[np.float64],
    intrinsics: NDArray[np.float64],
    options: twoview.Options,
) -> NDArray[np.float64] | None:
    """Return OpenCV's relative pose: the 5-point E by RANSAC, then ``recoverPose``.

    It uses the same threshold and confidence as slamlab's run, so the two stay comparable.

    Parameters
    ----------
    p1, p2 : ndarray of float64, shape (M, 2)
        The matched pixels.
    intrinsics : ndarray of float64, shape (9,)
        The ``radtan`` intrinsics.
    options : twoview.Options
        slamlab's settings, whose threshold and confidence OpenCV uses.

    Returns
    -------
    ndarray of float64, shape (4, 4), or None
        ``T_c2_c1`` with unit translation; None if OpenCV found no essential matrix.
    """
    k = np.array([[intrinsics[0], 0, intrinsics[2]], [0, intrinsics[1], intrinsics[3]], [0, 0, 1]])
    distortion = intrinsics[4:]
    u1 = cv2.undistortPoints(p1.reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    u2 = cv2.undistortPoints(p2.reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    e, mask = cv2.findEssentialMat(
        u1,
        u2,
        np.eye(3),
        method=cv2.RANSAC,
        prob=options.confidence,
        threshold=options.threshold_px / options.focal,
    )
    if e is None or mask is None or e.shape != (3, 3):
        return None
    _, rotation, t, _ = cv2.recoverPose(e, u1, u2, np.eye(3), mask=mask)
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = t.ravel()
    return pose


def record(path: Path, result: twoview.Initialization, truth: NDArray[np.float64]) -> None:
    """Save a rerun recording of the cameras and the triangulated points.

    Parameters
    ----------
    path : pathlib.Path
        The ``.rrd`` file to write.
    result : twoview.Initialization
        The initialiser's result (camera 1 is the world).
    truth : ndarray of float64, shape (4, 4)
        The true ``T_c2_c1``, its translation scaled to unit length.
    """
    import rerun as rr  # noqa: PLC0415 -- optional, only for --rerun

    rr.init("slamlab two-view initialisation")
    rr.save(str(path))
    rr.log("world/points", rr.Points3D(result.points[result.triangulated], radii=0.005))
    for name, pose in (
        ("camera1", np.eye(4)),
        ("camera2", result.relative_pose),
        ("truth2", truth),
    ):
        world_cam = se3.inverse(pose)
        rr.log(
            f"world/{name}", rr.Transform3D(translation=world_cam[:3, 3], mat3x3=world_cam[:3, :3])
        )
        rr.log(f"world/{name}/frustum", rr.Pinhole(focal_length=0.5, width=1.0, height=0.75))


def pair(
    sequence: tum.Sequence, first: int, gap: int
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Match two frames; return the pixels, the bearings and the true ``T_c2_c1``.

    Parameters
    ----------
    sequence : tum.Sequence
        The sequence.
    first, gap : int
        The first frame's index, and the frames between the two.

    Returns
    -------
    pixels : ndarray of float64, shape (M, 4)
        Each match's pixel in frame 1, then in frame 2.
    bearings1, bearings2 : ndarray of float64, shape (M, 3)
        Each match's unit bearing in each camera.
    truth : ndarray of float64, shape (4, 4)
        The ground truth's ``T_c2_c1``.

    Raises
    ------
    IndexError
        If a frame index is outside the sequence.
    """
    second = first + gap
    if not (0 <= first < second < len(sequence.images)):
        raise IndexError(
            f"frames {first} and {second}: the sequence has {len(sequence.images)} frames"
        )
    truth = se3.inverse(sequence.T_world_cam[second]) @ sequence.T_world_cam[first]
    p1, p2 = matches(sequence.images[first], sequence.images[second])
    intrinsics = sequence.intrinsics
    b1 = camera.unproject("radtan", intrinsics, p1)
    b2 = camera.unproject("radtan", intrinsics, p2)
    return np.hstack([p1, p2]), b1, b2, truth


def survey(sequence: tum.Sequence, options: twoview.Options, backend: str) -> None:
    """Run the initialiser on pairs across the sequence and print the tally and errors.

    The pairs start every 40 frames, with gaps of 15 and 30 frames: the 38 pairs of fr1/xyz
    behind the two-view note's table.

    Parameters
    ----------
    sequence : tum.Sequence
        The sequence.
    options : twoview.Options
        The initialiser's settings.
    backend : str
        The backend to run.
    """
    tally: collections.Counter[str] = collections.Counter()
    ours, theirs = [], []
    for first in range(0, len(sequence.images) - 40, 40):
        for gap in (15, 30):
            pixels, b1, b2, truth = pair(sequence, first, gap)
            result = twoview.initialize(b1, b2, options, backend=backend)
            fallback = " (fallback)" if result.fallback else ""
            tally[f"{result.status}, {result.model}{fallback}"] += 1
            reference = opencv_reference(pixels[:, :2], pixels[:, 2:], sequence.intrinsics, options)
            consistency = truth_consistency(b1, b2, truth, options)
            row = f"{first:4d} +{gap:2d}: {result.status:<13} {result.model:<10}"
            if result.status is twoview.Status.INITIALIZED and reference is not None:
                ours.append(errors_deg(result.relative_pose, truth))
                theirs.append(errors_deg(reference, truth))
                (r1, d1), (r2, d2) = ours[-1], theirs[-1]
                row += f" {r1:5.2f} {d1:6.2f} | opencv {r2:5.2f} {d2:6.2f}"
            print(f"{row} | {consistency}")
    for key, count in sorted(tally.items()):
        print(f"{count:3d}  {key}")
    if ours:
        a, b = np.array(ours), np.array(theirs)
        print(
            f"initialised {len(a)}: median rotation {np.median(a[:, 0]):.2f} deg, direction "
            f"{np.median(a[:, 1]):.2f} deg (OpenCV on the same pairs {np.median(b[:, 0]):.2f}, "
            f"{np.median(b[:, 1]):.2f}); direction off by > 20 deg: {int((a[:, 1] > 20).sum())} "
            f"(OpenCV {int((b[:, 1] > 20).sum())})"
        )


def main() -> None:
    """Run the demo."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--first", type=int, default=0, help="the first frame's index")
    parser.add_argument("--gap", type=int, default=30, help="frames between the two views")
    parser.add_argument("--backend", choices=backends.BACKENDS, help="one backend only")
    parser.add_argument("--rerun", type=Path, help="save a rerun recording to this .rrd file")
    parser.add_argument(
        "--survey", action="store_true", help="run 38 pairs across the sequence instead"
    )
    args = parser.parse_args()

    sequence = tum.read_sequence(args.sequence)
    intrinsics = sequence.intrinsics
    options = twoview.Options(focal=float(intrinsics[0]))
    lockstep = [b for b in backends.available() if b in backends.LOCKSTEP_BACKENDS]
    if args.survey:
        survey(sequence, options, args.backend or ("cpp" if "cpp" in lockstep else "python"))
        return
    pixels, b1, b2, truth = pair(sequence, args.first, args.gap)
    first, second = args.first, args.first + args.gap
    print(
        f"{args.sequence.name}: frames {first} and {second} "
        f"({sequence.timestamps[second] - sequence.timestamps[first]:.2f} s apart, "
        f"baseline {100 * np.linalg.norm(truth[:3, 3]):.1f} cm); "
        f"{truth_consistency(b1, b2, truth, options)}"
    )
    chosen = [args.backend] if args.backend else lockstep
    result = None
    for backend in chosen:
        start = time.perf_counter()
        result = twoview.initialize(b1, b2, options, backend=backend)
        elapsed = 1000 * (time.perf_counter() - start)
        rotation, direction = errors_deg(result.relative_pose, truth)
        print(
            f"{backend:>6}: {result.status}, {result.model}"
            f"{' (after an ambiguous H)' if result.fallback else ''} "
            f"(GRIC E {result.gric_essential:.0f}, H {result.gric_homography:.0f}); "
            f"{np.count_nonzero(result.inliers)} inliers, "
            f"{np.count_nonzero(result.triangulated)} points; "
            f"rotation error {rotation:.2f} deg, translation direction {direction:.2f} deg; "
            f"{elapsed:.0f} ms"
        )
    reference = opencv_reference(pixels[:, :2], pixels[:, 2:], intrinsics, options)
    if reference is None:
        print(f"{'opencv':>6}: no essential matrix")
    else:
        rotation, direction = errors_deg(reference, truth)
        print(
            f"{'opencv':>6}: 5-point RANSAC, rotation error {rotation:.2f} deg, "
            f"translation direction {direction:.2f} deg"
        )
    if args.rerun and result is not None:
        unit = truth.copy()
        unit[:3, 3] /= np.linalg.norm(unit[:3, 3])
        record(args.rerun, result, unit)
        print(f"saved {args.rerun}")


if __name__ == "__main__":
    main()
