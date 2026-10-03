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
import math
import time
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.eval import tum
from slamlab.foundations import camera, se3, so3, twoview

DEFAULT_SEQUENCE = Path("../datasets/tum/rgbd_dataset_freiburg1_xyz")
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
    sift = cv2.SIFT.create(nfeatures=3000)
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
        The angle of ``R_foundᵀ R_true``, and the angle between the unit translations.
    """
    rotation = math.degrees(float(np.linalg.norm(so3.log(found[:3, :3].T @ truth[:3, :3]))))
    a = found[:3, 3] / np.linalg.norm(found[:3, 3])
    b = truth[:3, 3] / np.linalg.norm(truth[:3, 3])
    direction = math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(a @ b)))
    return rotation, direction


def opencv_reference(
    p1: NDArray[np.float64], p2: NDArray[np.float64], intrinsics: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Return OpenCV's relative pose: the 5-point E by RANSAC, then ``recoverPose``.

    Parameters
    ----------
    p1, p2 : ndarray of float64, shape (M, 2)
        The matched pixels.
    intrinsics : ndarray of float64, shape (9,)
        The ``radtan`` intrinsics.

    Returns
    -------
    ndarray of float64, shape (4, 4)
        ``T_c2_c1`` with unit translation.
    """
    k = np.array([[intrinsics[0], 0, intrinsics[2]], [0, intrinsics[1], intrinsics[3]], [0, 0, 1]])
    distortion = intrinsics[4:]
    u1 = cv2.undistortPoints(p1.reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    u2 = cv2.undistortPoints(p2.reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    e, mask = cv2.findEssentialMat(
        u1, u2, np.eye(3), method=cv2.RANSAC, prob=0.999, threshold=2.0 / intrinsics[0]
    )
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


def main() -> None:
    """Run the demo."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--first", type=int, default=0, help="the first frame's index")
    parser.add_argument("--gap", type=int, default=30, help="frames between the two views")
    parser.add_argument("--backend", choices=backends.BACKENDS, help="one backend only")
    parser.add_argument("--rerun", type=Path, help="save a rerun recording to this .rrd file")
    args = parser.parse_args()

    sequence = tum.read_sequence(args.sequence)
    first, second = args.first, args.first + args.gap
    intrinsics = sequence.intrinsics
    t_world = sequence.T_world_cam
    truth = se3.inverse(t_world[second]) @ t_world[first]  # T_c2_c1
    print(
        f"{args.sequence.name}: frames {first} and {second} "
        f"({sequence.timestamps[second] - sequence.timestamps[first]:.2f} s apart, "
        f"baseline {100 * np.linalg.norm(truth[:3, 3]):.1f} cm)"
    )

    p1, p2 = matches(sequence.images[first], sequence.images[second])
    b1 = camera.unproject("radtan", intrinsics, p1)
    b2 = camera.unproject("radtan", intrinsics, p2)
    print(f"{len(p1)} SIFT matches after the ratio test")

    options = twoview.Options(focal=float(intrinsics[0]))
    lockstep = [b for b in backends.available() if b in backends.LOCKSTEP_BACKENDS]
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
    rotation, direction = errors_deg(opencv_reference(p1, p2, intrinsics), truth)
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
