"""Two-view initialisation: the relative pose and first map points from two calibrated views.

A monocular SLAM system starts from two frames. It has their feature matches as bearing
vectors, and nothing else: no pose, no depth. This module finds the relative pose
``T_c2_c1`` (rotation, and translation up to scale) and triangulates the matches, or says why
it cannot.

**Two models compete** (as in ORB-SLAM, Mur-Artal et al., 2015). A general scene fixes an
essential matrix; a planar scene, or a camera that only rotated, does not (the 8-point system
has a family of solutions) but fixes a homography. So both are estimated, by LO-MSAC on the
same matches with independent random streams:

- E by the normalised 8-point algorithm, on the bearings;
- H by the normalised DLT, on the normalised image points ``f / f_z``.

**GRIC chooses between them** (Torr, "Geometric motion segmentation and model selection",
Phil. Trans. R. Soc. A, 1998). Each model's score adds a robust fit term and a penalty for its
complexity:

    GRIC = Σ_i min(e_i² / σ², λ₃ (r - d)) + λ₁ d n + λ₂ k,

with ``e_i`` each match's Sampson distance (both models' are distances in the joint image
space, so they compare directly), ``σ`` the image noise, ``r = 4`` the data dimension (two
image points), ``d`` the model's dimension (E: 3, H: 2), ``k`` its parameters (E: 5, H: 8),
``n`` the number of matches, and Torr's ``λ₁ = ln r``, ``λ₂ = ln(r n)``, ``λ₃ = 2``. The lower
score wins. H wins on a plane or a rotation, where it fits as well with a smaller dimension; E
wins on a general scene, where H's residuals are large.

Both scores are taken over the matches at least one model explains (the union of the two
RANSAC inlier sets), not over every match. A match neither model explains is an outlier, and
tells nothing about which model is right; but the cap charges it ``λ₃ (r - d)``, which is 2 for
E and 4 for H, so counting outliers favours E in proportion to how many there are. On a pure
rotation with 30% outliers, scoring every match chose E by 8; over the explained ones H wins.

**Choosing the motion.** The winning model's decomposition gives four candidates
(:func:`~slamlab.foundations.epipolar.decompose_essential`,
:func:`~slamlab.foundations.homography.decompose_homography`). Each is tried by triangulating
the model's inliers (:func:`~slamlab.foundations.triangulation.two_view`) and counting the
points in front of both cameras. The best must beat the runner-up clearly, and enough of its
points must have the parallax that makes their depth well determined.

**Falling back to E.** On a scene with little parallax (a desk seen from half a metre),
GRIC often prefers H, rightly: H explains most matches within the noise. But H's
decomposition then leaves two physically valid motions with similar support, and choosing
between them is a guess. In that case, if E on its own is unambiguous and has enough
parallax, E decides (``fallback`` is set). On 38 frame pairs of TUM fr1/xyz (1 to 1.5 s
apart), refusing on an ambiguous H initialised 3 pairs; falling back to E initialised 22,
2 with a translation direction off by more than 20 degrees (OpenCV's 5-point RANSAC: 3 of
the same 22).

**The outcome is a status, not an exception.** Not being able to initialise from two frames is
normal (the camera has not moved enough yet), so :func:`initialize` always returns its best
estimate and a :class:`Status`:

- ``initialized``: one candidate is clearly best, with at least ``min_points`` triangulated
  points of at least ``min_parallax_deg`` parallax;
- ``pure_rotation``: H won and is a rotation; the translation is zero and nothing can be
  triangulated (the rotation is returned);
- ``low_parallax``: too few points have enough parallax. Checked first, because it is the
  cause the caller can act on (wait for more motion), and it usually makes the candidates
  hard to tell apart too. A noisy pure rotation usually ends here: its H is not exactly a
  rotation, so its candidates have a tiny translation and no parallax;
- ``ambiguous``: the runner-up has more than ``ambiguity_ratio`` of the best's points in front.

The translation has unit length: two views cannot fix the scale.

The C++ core (``slamlab/foundations/twoview.hpp``) implements :func:`initialize`.
"""

from __future__ import annotations

import enum
import math
from dataclasses import asdict, dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slamlab import backends
from slamlab.foundations import correspondences, epipolar, homography, ransac, triangulation
from slamlab.foundations.arrays import as_bearing_pairs
from slamlab.foundations.errors import DegenerateError, InvalidInputError
from slamlab.foundations.solve import Solution

GRIC_LAMBDA3 = 2.0
"""Torr's ``λ₃``: a residual's contribution is capped at ``λ₃ (r - d)``."""
DATA_DIMENSION = 4
"""``r``: a correspondence is two image points."""
ESSENTIAL_DIMENSION, ESSENTIAL_PARAMETERS = 3, 5
"""E's variety has dimension 3 in the 4D data space, and E has 5 degrees of freedom."""
HOMOGRAPHY_DIMENSION, HOMOGRAPHY_PARAMETERS = 2, 8
"""H's variety has dimension 2, and H has 8 degrees of freedom."""


class Model(enum.StrEnum):
    """Which model GRIC chose."""

    ESSENTIAL = "essential"
    """The essential matrix: a general scene."""
    HOMOGRAPHY = "homography"
    """A homography: a plane, or a rotation."""


class Status(enum.StrEnum):
    """Whether the two views initialise, and if not, why."""

    INITIALIZED = "initialized"
    """One motion is clearly best, with enough well-triangulated points."""
    PURE_ROTATION = "pure_rotation"
    """The camera only rotated: no translation, nothing to triangulate."""
    AMBIGUOUS = "ambiguous"
    """Two candidate motions put similar numbers of points in front."""
    LOW_PARALLAX = "low_parallax"
    """Too few points have enough parallax for a reliable depth."""


@dataclass(frozen=True)
class Options:
    """The initialiser's settings.

    Attributes
    ----------
    focal : float
        The focal length in pixels, converting the pixel settings to the normalised plane.
    threshold_px : float
        RANSAC's inlier threshold (a Sampson distance), in pixels.
    sigma_px : float
        The image noise's standard deviation for GRIC, in pixels.
    confidence : float
        RANSAC's target confidence.
    max_iterations : int
        RANSAC's sample limit, per model.
    seed : int
        The random seed; E uses stream 0 and H stream 1.
    min_parallax_deg : float
        The parallax a triangulated point needs to count, in degrees.
    min_points : int
        The well-triangulated points initialisation needs.
    ambiguity_ratio : float
        The runner-up candidate must have fewer than this fraction of the best's points in
        front.
    """

    focal: float
    threshold_px: float = 2.0
    sigma_px: float = 1.0
    confidence: float = 0.999
    max_iterations: int = 10000
    seed: int = 0
    min_parallax_deg: float = 1.0
    min_points: int = 50
    ambiguity_ratio: float = 0.75

    def __post_init__(self) -> None:
        """Check the settings.

        Raises
        ------
        slamlab.foundations.errors.InvalidInputError
            If a setting is out of range.
        """
        for name in ("focal", "threshold_px", "sigma_px"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise InvalidInputError(f"{name}: must be positive and finite")
        if not (math.isfinite(self.min_parallax_deg) and self.min_parallax_deg >= 0.0):
            raise InvalidInputError("min_parallax_deg: must be non-negative")
        if self.min_points < 1:
            raise InvalidInputError("min_points: must be at least 1")
        if not 0.0 < self.ambiguity_ratio <= 1.0:
            raise InvalidInputError("ambiguity_ratio: must be in (0, 1]")

    def ransac_options(self, stream: int) -> ransac.Options:
        """Return RANSAC's options in the normalised plane, on one random stream.

        Parameters
        ----------
        stream : int
            The random stream.

        Returns
        -------
        ransac.Options
            The threshold divided by the focal length, and this run's limits and seed.
        """
        return ransac.Options(
            threshold=self.threshold_px / self.focal,
            confidence=self.confidence,
            max_iterations=self.max_iterations,
            seed=self.seed,
            stream=stream,
        )


@dataclass(frozen=True)
class Initialization:
    """The initialiser's result.

    Attributes
    ----------
    status : Status
        Whether it initialised, and if not, why.
    model : Model
        The model that decided: GRIC's choice, or E when it fell back.
    fallback : bool
        Whether H won GRIC but its candidates were ambiguous, and E decided instead.
    relative_pose : ndarray of float64, shape (4, 4)
        The best candidate ``T_c2_c1``, unit translation (zero for a pure rotation).
    inliers : ndarray of bool, shape (N,)
        The chosen model's RANSAC inliers.
    points : ndarray of float64, shape (N, 3)
        The well-triangulated points in camera 1's frame; NaN elsewhere.
    triangulated : ndarray of bool, shape (N,)
        Which matches have a point: inliers, in front of both cameras, enough parallax.
    gric_essential, gric_homography : float
        Each model's GRIC score (infinity if its RANSAC found no model).
    in_front : ndarray of int, shape (K,)
        Each candidate's count of inliers triangulated in front of both cameras, best first.
    essential, homography : ransac.Summary or None
        Each RANSAC run's summary (None if it found no model).
    """

    status: Status
    model: Model
    fallback: bool
    relative_pose: NDArray[np.float64]
    inliers: NDArray[np.bool_]
    points: NDArray[np.float64]
    triangulated: NDArray[np.bool_]
    gric_essential: float
    gric_homography: float
    in_front: NDArray[np.int64]
    essential: ransac.Summary | None
    homography: ransac.Summary | None


class Implementation(Protocol):
    """The functions a compiled backend's ``foundations.twoview`` submodule provides.

    ``initialize`` takes :class:`Options`' fields as one dict and returns a dict of the
    :class:`Initialization` fields: ``status`` and ``model`` as codes into ``codes()``,
    ``in_front`` as an int64 array, and ``essential`` and ``homography`` as summary dicts
    (:func:`slamlab.foundations.ransac.summary_from_backend`), or None when that model's
    RANSAC found nothing.
    """

    def codes(self) -> dict[str, list[str]]:
        """Return each enum's names in code order."""
        ...

    def initialize(
        self,
        bearings1: NDArray[np.float64],
        bearings2: NDArray[np.float64],
        options: dict[str, Any],
    ) -> dict[str, Any]:
        """Initialise from (N, 3) bearing pairs."""
        ...


# --- the reference ------------------------------------------------------------------------


def gric(
    squared_errors: NDArray[np.float64], sigma: float, dimension: int, parameters: int
) -> float:
    """Return Torr's GRIC score of a model's residuals.

    Parameters
    ----------
    squared_errors : ndarray of float64, shape (n,)
        Each datum's squared distance to the model (Sampson's).
    sigma : float
        The noise's standard deviation, in the same units.
    dimension : int
        The model's dimension ``d`` in the data space of dimension 4.
    parameters : int
        The model's degrees of freedom ``k``.

    Returns
    -------
    float
        ``Σ min(e²/σ², λ₃ (r - d)) + ln(r) d n + ln(r n) k``.
    """
    n = int(squared_errors.shape[0])
    cap = GRIC_LAMBDA3 * (DATA_DIMENSION - dimension)
    fit = float(np.sum(np.minimum(squared_errors / sigma**2, cap)))
    penalty = math.log(DATA_DIMENSION) * dimension * n + math.log(DATA_DIMENSION * n) * parameters
    return fit + float(penalty)


def _fit(
    estimator: ransac.Estimator[correspondences.Matrix3], options: ransac.Options
) -> Solution[ransac.Consensus[correspondences.Matrix3], ransac.Summary] | None:
    """Run one RANSAC; None if no sample gave a model."""
    try:
        return ransac.estimate(estimator, options)
    except DegenerateError:
        return None


@dataclass(frozen=True)
class _Tried:
    """One candidate motion, triangulated."""

    pose: NDArray[np.float64]
    in_front: int
    good: NDArray[np.bool_]
    points: NDArray[np.float64]


def _try(
    pose: NDArray[np.float64],
    rays: tuple[NDArray[np.float64], NDArray[np.float64]],
    inliers: NDArray[np.bool_],
    min_parallax: float,
) -> _Tried:
    """Triangulate a candidate's inliers; count those in front, keep those with parallax."""
    unit = pose.copy()
    unit[:3, 3] /= np.linalg.norm(unit[:3, 3])
    index = np.flatnonzero(inliers)
    packed = triangulation.triangulate_pairs(unit, rays[0][index], rays[1][index])
    in_front = packed[:, 3] > 0.5
    good = np.zeros(inliers.shape, dtype=bool)
    good[index] = in_front & (packed[:, 4] >= min_parallax)
    points = np.full((inliers.shape[0], 3), np.nan)
    points[good] = packed[good[index], :3]
    return _Tried(unit, int(np.count_nonzero(in_front)), good, points)


def _candidates(model: Model, matrix: correspondences.Matrix3) -> NDArray[np.float64]:
    """Return the chosen model's candidate motions ``T_c2_c1``, shape (K, 4, 4)."""
    if model is Model.ESSENTIAL:
        return epipolar.decompose_reference(matrix)
    return homography.decompose_reference(matrix)["poses"]


@dataclass(frozen=True)
class _Decision:
    """A model's verdict: its status, best candidate and per-candidate counts."""

    status: Status
    pose: NDArray[np.float64]
    best: _Tried | None
    counts: list[int]


def _decide(
    model: Model,
    run: Solution[ransac.Consensus[correspondences.Matrix3], ransac.Summary],
    rays: tuple[NDArray[np.float64], NDArray[np.float64]],
    options: Options,
) -> _Decision:
    """Try a model's candidate motions on its inliers and judge the best."""
    candidates = _candidates(model, run.state.model)
    if not np.linalg.norm(candidates[0][:3, 3]) > 0.0:
        return _Decision(Status.PURE_ROTATION, candidates[0], None, [])
    min_parallax = math.radians(options.min_parallax_deg)
    tried = sorted(
        (_try(pose, rays, run.state.inliers, min_parallax) for pose in candidates),
        key=lambda t: -t.in_front,
    )
    counts = [t.in_front for t in tried]
    best = tried[0]
    if np.count_nonzero(best.good) < options.min_points:
        status = Status.LOW_PARALLAX
    elif len(tried) > 1 and tried[1].in_front > options.ambiguity_ratio * best.in_front:
        status = Status.AMBIGUOUS
    else:
        status = Status.INITIALIZED
    return _Decision(status, best.pose, best, counts)


def _initialize(
    bearings1: NDArray[np.float64], bearings2: NDArray[np.float64], options: Options
) -> Initialization:
    """Initialise two views (reference)."""
    n = bearings1.shape[0]
    x1, x2 = (
        correspondences.normalised_plane(bearings1),
        correspondences.normalised_plane(bearings2),
    )
    essential = _fit(epipolar.essential_estimator(bearings1, bearings2), options.ransac_options(0))
    planar = _fit(homography.homography_estimator(x1[:, :2], x2[:, :2]), options.ransac_options(1))
    if essential is None and planar is None:
        raise DegenerateError("initialize: neither RANSAC found a model")
    sigma = options.sigma_px / options.focal
    # Score both over the matches at least one model explains (see the module docstring).
    support = np.zeros(n, dtype=bool)
    for run in (essential, planar):
        if run is not None:
            support |= run.state.inliers
    gric_e = math.inf
    if essential is not None:
        e_errors = epipolar.sampson(essential.state.model, x1[support], x2[support])
        gric_e = gric(e_errors, sigma, ESSENTIAL_DIMENSION, ESSENTIAL_PARAMETERS)
    gric_h = math.inf
    if planar is not None:
        h_errors = homography.sampson(planar.state.model, x1[support, :2], x2[support, :2])
        gric_h = gric(h_errors, sigma, HOMOGRAPHY_DIMENSION, HOMOGRAPHY_PARAMETERS)
    model = Model.HOMOGRAPHY if gric_h < gric_e else Model.ESSENTIAL
    chosen = planar if model is Model.HOMOGRAPHY else essential
    assert chosen is not None  # the lower score is finite
    rays = (bearings1, bearings2)
    decision = _decide(model, chosen, rays, options)
    fallback = False
    if decision.status is Status.AMBIGUOUS and model is Model.HOMOGRAPHY and essential is not None:
        alternative = _decide(Model.ESSENTIAL, essential, rays, options)
        if alternative.status is Status.INITIALIZED:
            model, chosen, decision, fallback = Model.ESSENTIAL, essential, alternative, True
    best = decision.best
    return Initialization(
        status=decision.status,
        model=model,
        fallback=fallback,
        relative_pose=decision.pose,
        inliers=chosen.state.inliers,
        points=best.points if best is not None else np.full((n, 3), np.nan),
        triangulated=best.good if best is not None else np.zeros(n, dtype=bool),
        gric_essential=gric_e,
        gric_homography=gric_h,
        in_front=np.asarray(decision.counts, dtype=np.int64),
        essential=essential.summary if essential is not None else None,
        homography=planar.summary if planar is not None else None,
    )


# --- validation and dispatch --------------------------------------------------------------


def _summary(raw: dict[str, Any] | None, codes: dict[str, list[str]]) -> ransac.Summary | None:
    """Rebuild a RANSAC summary a compiled backend returned (None if that run found no model)."""
    return None if raw is None else ransac.summary_from_backend(raw, codes)


def _from_backend(raw: dict[str, Any], codes: dict[str, list[str]]) -> Initialization:
    """Build the result from a compiled backend's dict."""
    return Initialization(
        status=Status(codes["status"][int(raw["status"])]),
        model=Model(codes["model"][int(raw["model"])]),
        fallback=bool(raw["fallback"]),
        relative_pose=np.asarray(raw["relative_pose"], dtype=np.float64),
        inliers=np.asarray(raw["inliers"], dtype=np.bool_),
        points=np.asarray(raw["points"], dtype=np.float64),
        triangulated=np.asarray(raw["triangulated"], dtype=np.bool_),
        gric_essential=float(raw["gric_essential"]),
        gric_homography=float(raw["gric_homography"]),
        in_front=np.asarray(raw["in_front"], dtype=np.int64),
        essential=_summary(raw["essential"], codes),
        homography=_summary(raw["homography"], codes),
    )


def initialize(
    bearings1: ArrayLike, bearings2: ArrayLike, options: Options, *, backend: str = "python"
) -> Initialization:
    """Find the relative pose and first points of two calibrated views, or say why not.

    Parameters
    ----------
    bearings1, bearings2 : array_like, shape (N, 3)
        Unit bearing vectors of each match in camera 1 and camera 2, positive z, N >= 8.
    options : Options
        The focal length, thresholds, RANSAC settings and acceptance criteria.
    backend : {"python", "cpp", "rust"}, optional
        Which implementation runs (default: the Python reference).

    Returns
    -------
    Initialization
        The status, the chosen model and motion, the inliers and triangulated points, and the
        scores and summaries behind the decision.

    Raises
    ------
    slamlab.foundations.errors.InvalidInputError
        If the bearings are not matching (N, 3) unit vectors with positive z, or N < 8.
    slamlab.foundations.errors.DegenerateError
        If neither RANSAC found a model.
    """
    b1, b2 = as_bearing_pairs(bearings1, bearings2, epipolar.SAMPLE_SIZE)
    backends.check_name(backend)
    if backend == "python":
        return _initialize(b1, b2, options)
    implementation = backends.implementation(backend, __name__)
    raw = backends.function(backend, __name__, "initialize")(b1, b2, options=asdict(options))
    return _from_backend(raw, implementation.codes())
