"""Absolute pose on a real sequence: PnP RANSAC against the ground truth and OpenCV.

A reference frame A provides 3D points, in A's camera frame (the world here); a later frame C
is matched to A with slamlab's ORB features (:mod:`slamlab.foundations.orb` and
:mod:`slamlab.foundations.matching`, ORB-SLAM's extractor and filters), and
:func:`slamlab.foundations.pnp.pnp_ransac` places C against the points: ``T_C_A``. The
points come from one of two sources:

- ``depth`` (default): A's keypoints back-projected through its registered depth map, metric
  and independent of slamlab's two-view code;
- ``twoview``: :func:`slamlab.foundations.twoview.initialize` on A and B = A + ``--init-gap``,
  its triangulated points scaled by the true baseline (two views fix no scale), so the errors
  are in metres here too. This is a tracker's first step: initialise, then place the next
  frame.

It prints the inliers, the rotation error (degrees) and the camera-position error
(centimetres) against the motion-capture ground truth, per backend and P3P solver, and
OpenCV's ``solvePnPRansac`` (AP3P) with ``solvePnPRefineLM`` on the inliers, on the same
correspondences. ``--survey`` runs frames across the sequence and prints the medians.

Run from the repository root, in the slamlab environment::

    python scripts/demo_absolute_pose.py
    python scripts/demo_absolute_pose.py --source twoview --first 80 --offset 45
    python scripts/demo_absolute_pose.py --survey --sequence ../datasets/tum/<sequence>

The data stays outside the repository (default ``../datasets/tum``); download a sequence from
https://cvg.cit.tum.de/data/datasets/rgbd-dataset and extract it there.
"""

from __future__ import annotations

import argparse
import collections
import math
from pathlib import Path
from typing import TypeAlias

import cv2
import numpy as np
from numpy.typing import NDArray

from slamlab import backends
from slamlab.eval import images, tum
from slamlab.foundations import camera, matching, orb, p3p, pnp, ransac, se3, so3, twoview
from slamlab.foundations.correspondences import normalised_xy
from slamlab.foundations.errors import DegenerateError
from slamlab.foundations.solve import Solution

DEFAULT_SEQUENCE = Path("../datasets/tum/rgbd_dataset_freiburg1_xyz")
FEATURES = orb.Options(n_features=2000)
"""ORB features per image: twice ORB-SLAM's per-frame budget, as its initialiser uses."""
THRESHOLD_PX = 2.0
"""PnP RANSAC's inlier threshold, in pixels."""
MIN_CORRESPONDENCES = 12
"""Fewer 3D-2D correspondences than this and a frame is skipped."""
MIN_INLIERS = 20
"""A placement with fewer inliers than this is rejected, as a tracker would reject it. On
TUM fr1/xyz's blurred stretch, the ground truth itself explains under 5% of some frames'
matches; RANSAC then finds 3-7 inliers and a wrong pose."""
SURVEY_STRIDE = 40
"""The survey takes a reference frame every this many frames."""
SURVEY_OFFSETS = (15, 30)
"""Each survey reference places the frames this many frames later."""
OPENCV_ITERATIONS = 10000
"""OpenCV's RANSAC sample limit: slamlab's default ``max_iterations``."""
OPENCV_CONFIDENCE = 0.999
"""OpenCV's RANSAC confidence: slamlab's default."""
OPENCV_MIN_INLIERS = 4
"""Fewer OpenCV inliers than this and its pose is not refined (``solvePnPRefineLM`` needs
four)."""
CONFIGURATIONS = (("ding", True), ("lambda_twist", True), ("ding", False))
"""The (P3P solver, refine) configurations the survey compares."""


def _backend() -> str:
    """Return the fastest built backend: every one extracts and matches the same features."""
    return next(b for b in ("cpp", "rust", "python") if b in backends.available())


def features(path: Path) -> orb.Features:
    """Extract slamlab's ORB features from an image.

    Parameters
    ----------
    path : pathlib.Path
        The image.

    Returns
    -------
    slamlab.foundations.orb.Features
        Its keypoints and descriptors.
    """
    return orb.extract(images.read_gray(path), FEATURES, backend=_backend())


def match(a: orb.Features, b: orb.Features) -> NDArray[np.int64]:
    """Match two images' features with ORB-SLAM's filters.

    Parameters
    ----------
    a, b : slamlab.foundations.orb.Features
        The two images' features.

    Returns
    -------
    ndarray of int64, shape (M, 2)
        Each match's keypoint index in `a`, then in `b`.
    """
    found = matching.match(
        a.descriptors, b.descriptors, a.keypoints.angle, b.keypoints.angle, backend=_backend()
    )
    return found.pairs


def depth_points(
    sequence: tum.Sequence, frame: int, keypoints: orb.Features
) -> NDArray[np.float64]:
    """Back-project keypoints through the frame's depth map, in its camera frame.

    Parameters
    ----------
    sequence : tum.Sequence
        The sequence.
    frame : int
        The frame's index.
    keypoints : slamlab.foundations.orb.Features
        Its keypoints.

    Returns
    -------
    ndarray of float64, shape (K, 3)
        Each keypoint's point (NaN where the depth map has no measurement).

    Raises
    ------
    ValueError
        If the frame has no depth map.
    FileNotFoundError
        If the depth map cannot be read.
    """
    path = sequence.depths[frame]
    if path is None:
        raise ValueError(f"frame {frame} has no depth map within the association tolerance")
    depth = tum.read_depth(path)
    pixels = keypoints.keypoints.xy
    pixel = np.rint(pixels).astype(np.int64)
    height, width = depth.shape
    inside = np.all((pixel >= 0) & (pixel < [width, height]), axis=1)
    z = np.full(len(pixel), np.nan)
    z[inside] = depth[pixel[inside, 1], pixel[inside, 0]]
    rays = camera.unproject("radtan", sequence.intrinsics, pixels)
    return np.asarray(rays * (z / rays[:, 2])[:, None], dtype=np.float64)


def twoview_points(
    sequence: tum.Sequence, first: int, gap: int, found: dict[int, orb.Features]
) -> NDArray[np.float64] | None:
    """Triangulate frame `first`'s keypoints with frame ``first + gap``, at the true scale.

    Parameters
    ----------
    sequence : tum.Sequence
        The sequence.
    first, gap : int
        The reference frame, and the frames to the second view.
    found : dict of int to slamlab.foundations.orb.Features
        Keypoints already detected, by frame (filled as needed).

    Returns
    -------
    ndarray of float64, shape (K, 3), or None
        Each keypoint's point in the reference camera (NaN where not triangulated); None if
        the initialiser did not initialise.
    """
    a, b = keypoints(sequence, first, found), keypoints(sequence, first + gap, found)
    pairs = match(a, b)
    intrinsics = sequence.intrinsics
    rays_a = camera.unproject("radtan", intrinsics, a.keypoints.xy[pairs[:, 0]])
    rays_b = camera.unproject("radtan", intrinsics, b.keypoints.xy[pairs[:, 1]])
    try:
        result = twoview.initialize(rays_a, rays_b, twoview.Options(focal=float(intrinsics[0])))
    except DegenerateError:  # neither RANSAC found a model
        return None
    if result.status is not twoview.Status.INITIALIZED:
        return None
    truth = se3.inverse(sequence.T_world_cam[first + gap]) @ sequence.T_world_cam[first]
    scale = float(np.linalg.norm(truth[:3, 3]) / np.linalg.norm(result.relative_pose[:3, 3]))
    points = np.full((len(a.keypoints.xy), 3), np.nan)
    good = pairs[result.triangulated, 0]
    points[good] = scale * result.points[result.triangulated]
    return points


def keypoints(sequence: tum.Sequence, frame: int, found: dict[int, orb.Features]) -> orb.Features:
    """Return a frame's keypoints, detecting them once."""
    if frame not in found:
        found[frame] = features(sequence.images[frame])
    return found[frame]


def correspondences(
    sequence: tum.Sequence, points: NDArray[np.float64], first: int, target: int,
    found: dict[int, orb.Features],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:  # fmt: skip
    """Match the reference frame to the target; keep the matches whose keypoint has a point.

    Returns
    -------
    points : ndarray of float64, shape (M, 3)
        The points, in the reference camera.
    bearings : ndarray of float64, shape (M, 3)
        Their unit bearings in the target camera.
    pixels : ndarray of float64, shape (M, 2)
        Their pixels in the target image.
    """
    a, c = keypoints(sequence, first, found), keypoints(sequence, target, found)
    pairs = match(a, c)
    pairs = pairs[np.all(np.isfinite(points[pairs[:, 0]]), axis=1)]
    pixels = c.keypoints.xy[pairs[:, 1]]
    return points[pairs[:, 0]], camera.unproject("radtan", sequence.intrinsics, pixels), pixels


def errors(found: NDArray[np.float64], truth: NDArray[np.float64]) -> tuple[float, float]:
    """Return the rotation error (degrees) and the camera-position error (centimetres).

    Parameters
    ----------
    found, truth : ndarray of float64, shape (4, 4)
        ``T_C_A``, estimated and true.

    Returns
    -------
    tuple of float
        The angle of ``R_foundᵀ R_true``, and the distance between the camera centres.
    """
    rotation = math.degrees(float(np.linalg.norm(so3.log(found[:3, :3].T @ truth[:3, :3]))))
    centre_found = -found[:3, :3].T @ found[:3, 3]
    centre_true = -truth[:3, :3].T @ truth[:3, 3]
    return rotation, 100.0 * float(np.linalg.norm(centre_found - centre_true))


def opencv_reference(
    points: NDArray[np.float64], bearings: NDArray[np.float64], threshold: float
) -> tuple[NDArray[np.float64], int] | None:
    """Return OpenCV's ``T_C_A``: ``solvePnPRansac`` (AP3P), then ``solvePnPRefineLM``.

    It works on the bearings' normalised image points with an identity camera, and the same
    threshold and confidence as slamlab's run.

    Parameters
    ----------
    points : ndarray of float64, shape (M, 3)
        The points.
    bearings : ndarray of float64, shape (M, 3)
        Their bearings, positive z.
    threshold : float
        The inlier threshold in the normalised plane.

    Returns
    -------
    tuple of (ndarray of float64, shape (4, 4), int), or None
        The pose and its inlier count, or None if OpenCV found none.
    """
    image = normalised_xy(bearings)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        points, image, np.eye(3), None, iterationsCount=OPENCV_ITERATIONS,
        reprojectionError=threshold, confidence=OPENCV_CONFIDENCE, flags=cv2.SOLVEPNP_AP3P,
    )  # fmt: skip
    if not ok or inliers is None or len(inliers) < OPENCV_MIN_INLIERS:
        return None
    kept = inliers.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(points[kept], image[kept], np.eye(3), None, rvec, tvec)
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return pose, len(kept)


Placed: TypeAlias = Solution[ransac.Consensus[NDArray[np.float64]], ransac.RefinedSummary]
"""A placement: the pose and its inliers, and the run's summary."""


def place(
    points: NDArray[np.float64],
    bearings: NDArray[np.float64],
    focal: float,
    configuration: tuple[str, bool],
    backend: str,
) -> Placed | None:
    """Run PnP RANSAC with a (solver, refine) configuration; None if no sample gave a pose."""
    solver, refine = configuration
    options = ransac.Options(threshold=THRESHOLD_PX / focal)
    try:
        return pnp.pnp_ransac(
            points, bearings, options, minimal_solver=solver, refine=refine, backend=backend
        )
    except DegenerateError:
        return None


def reference_points(
    sequence: tum.Sequence, source: str, first: int, init_gap: int, found: dict[int, orb.Features]
) -> NDArray[np.float64] | None:
    """Return the reference frame's points from the chosen source (None: not initialised)."""
    if source == "depth":
        return depth_points(sequence, first, keypoints(sequence, first, found))
    return twoview_points(sequence, first, init_gap, found)


def survey(sequence: tum.Sequence, source: str, init_gap: int, backend: str) -> None:
    """Place frames across the sequence and print each configuration's median errors.

    Reference frames start every :data:`SURVEY_STRIDE` frames; each places the frames
    :data:`SURVEY_OFFSETS` later.

    Parameters
    ----------
    sequence : tum.Sequence
        The sequence.
    source : {"depth", "twoview"}
        Where the reference frame's points come from.
    init_gap : int
        The two-view initialiser's gap (source ``twoview``).
    backend : str
        The backend to run.
    """
    focal = float(sequence.intrinsics[0])
    results: dict[str, list[tuple[float, float]]] = collections.defaultdict(list)
    rejected: collections.Counter[str] = collections.Counter()
    skipped: collections.Counter[str] = collections.Counter()
    last = len(sequence.images) - max(SURVEY_OFFSETS)
    for first in range(0, last, SURVEY_STRIDE):
        if source == "depth" and sequence.depths[first] is None:
            skipped["no depth map"] += len(SURVEY_OFFSETS)
            continue
        found: dict[int, orb.Features] = {}
        points = reference_points(sequence, source, first, init_gap, found)
        if points is None:
            skipped["not initialised"] += len(SURVEY_OFFSETS)
            continue
        for offset in SURVEY_OFFSETS:
            world, rays, _ = correspondences(sequence, points, first, first + offset, found)
            if len(world) < MIN_CORRESPONDENCES:
                skipped["too few correspondences"] += 1
                continue
            truth = se3.inverse(sequence.T_world_cam[first + offset]) @ sequence.T_world_cam[first]
            for configuration in CONFIGURATIONS:
                name = f"{configuration[0]}{'' if configuration[1] else ', unrefined'}"
                run = place(world, rays, focal, configuration, backend)
                if run is None or np.count_nonzero(run.state.inliers) < MIN_INLIERS:
                    rejected[name] += 1
                    continue
                results[name].append(errors(run.state.model, truth))
            reference = opencv_reference(world, rays, THRESHOLD_PX / focal)
            if reference is None or reference[1] < MIN_INLIERS:
                rejected["opencv"] += 1
                continue
            results["opencv"].append(errors(reference[0], truth))
    for name, rows in results.items():
        table = np.array(rows)
        print(
            f"{name:>17}: {len(table)} placed, {rejected[name]} rejected; median rotation "
            f"{np.median(table[:, 0]):.3f} deg, position {np.median(table[:, 1]):.2f} cm (90th "
            f"percentile {np.quantile(table[:, 0], 0.9):.3f} deg, "
            f"{np.quantile(table[:, 1], 0.9):.2f} cm)"
        )
    for reason, count in skipped.items():
        print(f"skipped {count} frames: {reason}")


def report_single(sequence: tum.Sequence, args: argparse.Namespace, chosen: list[str]) -> None:
    """Place one frame against the reference's points and print each backend's result.

    Parameters
    ----------
    sequence : tum.Sequence
        The sequence.
    args : argparse.Namespace
        The parsed arguments (``first``, ``offset``, ``source``, ``init_gap``, ``sequence``).
    chosen : list of str
        The backends to run.
    """
    target = args.first + args.offset
    found: dict[int, orb.Features] = {}
    points = reference_points(sequence, args.source, args.first, args.init_gap, found)
    if points is None:
        print(
            f"two-view initialisation of frames {args.first} and {args.first + args.init_gap} "
            "did not initialise; try another --first or --init-gap"
        )
        return
    world, rays, _ = correspondences(sequence, points, args.first, target, found)
    truth = se3.inverse(sequence.T_world_cam[target]) @ sequence.T_world_cam[args.first]
    focal = float(sequence.intrinsics[0])
    print(
        f"{args.sequence.name}: frame {target} against frame {args.first}'s points "
        f"({args.source}); {len(world)} correspondences; the camera moved "
        f"{100 * np.linalg.norm(truth[:3, 3]):.1f} cm"
    )
    for backend in chosen:
        for solver in p3p.SOLVERS:
            run = place(world, rays, focal, (solver, True), backend)
            if run is None:
                print(f"{backend:>6} {solver:>12}: no pose")
                continue
            rotation, position = errors(run.state.model, truth)
            print(
                f"{backend:>6} {solver:>12}: {np.count_nonzero(run.state.inliers)} inliers, "
                f"{run.summary.iterations} samples, refined {run.summary.refined}; rotation "
                f"error {rotation:.3f} deg, position {position:.2f} cm"
            )
    reference = opencv_reference(world, rays, THRESHOLD_PX / focal)
    if reference is None:
        print(f"{'opencv':>19}: no pose")
    else:
        rotation, position = errors(reference[0], truth)
        print(
            f"{'opencv':>19}: {reference[1]} inliers; rotation error {rotation:.3f} deg, "
            f"position {position:.2f} cm"
        )


def main() -> None:
    """Run the demo."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--first", type=int, default=0, help="the reference frame's index")
    parser.add_argument("--offset", type=int, default=30, help="frames from the reference")
    parser.add_argument("--source", choices=("depth", "twoview"), default="depth")
    parser.add_argument("--init-gap", type=int, default=30, help="two-view initialiser's gap")
    parser.add_argument("--backend", choices=backends.BACKENDS, help="one backend only")
    parser.add_argument("--survey", action="store_true", help="run frames across the sequence")
    args = parser.parse_args()

    sequence = tum.read_sequence(args.sequence)
    lockstep = [b for b in backends.available() if b in backends.LOCKSTEP_BACKENDS]
    if args.survey:
        backend = args.backend or ("cpp" if "cpp" in lockstep else "python")
        survey(sequence, args.source, args.init_gap, backend)
        return
    target = args.first + args.offset
    if not 0 <= args.first < target < len(sequence.images):
        parser.error(f"frames {args.first} and {target}: the sequence has {len(sequence.images)}")
    report_single(sequence, args, [args.backend] if args.backend else lockstep)


if __name__ == "__main__":
    main()
