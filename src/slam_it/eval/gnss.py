"""A GNSS track, smoothed, and a run scored against it.

A consumer receiver's fixes (a dashcam's, read from its overlay by
:mod:`slam_it.eval.dashcam`) are a few metres out, worse among tall buildings and absent in a
garage, and a fix read off a video can be misread outright. So they are a reference, not ground
truth:

- :func:`local_frame`: latitude and longitude to metres east and north of an origin, by the
  equirectangular approximation (good to well under a centimetre per kilometre at a city's
  scale).
- :func:`smooth`: a constant-velocity Kalman filter over the fixes, whatever their spacing, each
  fix's innovation tested against the χ² bound :data:`GATE` and refused past it (a misread, or a
  receiver's multipath); then a Rauch-Tung-Striebel smoother. The state is ``(x, y, vx, vy)``;
  the process noise is white acceleration. The filter starts from a robust line through the
  first :data:`START_FIXES` fixes (Theil-Sen: the median of the pairs' velocities), so one
  misread among them cannot start it astray; and :data:`RESTART` fixes refused in a row mean
  the filter has lost the car, not that the fixes are wrong, so the last of them is taken with
  the filter's state let go.
- :func:`score`: a run's maps against the smoothed track. Fixes carry no altitude, so each map
  is scored in its plane of motion: its positions put in their two leading principal axes, then
  a 2D similarity (Umeyama: rotation, translation, scale; a monocular map's scale is unknown)
  aligned to the track at the frames' times. The errors are in metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from slam_it.eval import runs
from slam_it.foundations.errors import InvalidInputError

GATE = 13.82
"""The χ² bound, 2 degrees of freedom at 99.9%, past which a fix's innovation is refused."""

EARTH_RADIUS = 6_371_000.0
"""The mean Earth radius, in metres, for :func:`local_frame`."""

START_FIXES = 5
"""The first fixes the filter's start is fitted to (the module docstring)."""

RESTART = 3
"""The fixes refused in a row after which the filter lets its state go and takes the last."""

LET_GO = (1e6, 1e6, 100.0, 100.0)
"""The variances added to the state when the filter lets it go: positions unknown to a
kilometre, velocities to 10 m/s."""

Array = NDArray[np.float64]


@dataclass(frozen=True)
class Track:
    """A smoothed track.

    Attributes
    ----------
    times : ndarray of float64, shape (N,)
        The fixes' times, in seconds, increasing.
    position : ndarray of float64, shape (N, 2)
        The smoothed position, metres east and north of the origin.
    velocity : ndarray of float64, shape (N, 2)
        The smoothed velocity, in metres per second.
    covariance : ndarray of float64, shape (N, 4, 4)
        The smoothed state's covariance, ``(x, y, vx, vy)``.
    refused : ndarray of bool, shape (N,)
        The fixes the gate refused.
    """

    times: Array
    position: Array
    velocity: Array
    covariance: Array
    refused: NDArray[np.bool_]

    def at(self, times: ArrayLike) -> Array:
        """Return the smoothed position at `times`, interpolated linearly, shape (M, 2).

        Parameters
        ----------
        times : array_like, shape (M,)
            Times within the track's span, in seconds.

        Returns
        -------
        ndarray of float64, shape (M, 2)
            The positions.

        Raises
        ------
        slam_it.foundations.errors.InvalidInputError
            If a time is not finite or lies outside the track.
        """
        t = np.asarray(times, dtype=np.float64)
        if t.ndim != 1 or not np.all(np.isfinite(t)):
            raise InvalidInputError("times: expected finite times, shape (M,)")
        if t.size and (t.min() < self.times[0] or t.max() > self.times[-1]):
            raise InvalidInputError("times: outside the track")
        return np.stack([np.interp(t, self.times, self.position[:, k]) for k in (0, 1)], axis=1)


@dataclass(frozen=True)
class MapScore:
    """One map scored against a track.

    Attributes
    ----------
    map : int
        The map's number in the run.
    frames : int
        Its frames scored: those within the track's span.
    scale : float
        Metres per map unit, from the alignment.
    rmse, median, p90 : float
        The planar error's root mean square, median and 90th percentile, in metres.
    flatness : float
        The map's third principal extent over its first: near 0 for a drive on level ground.
    """

    map: int
    frames: int
    scale: float
    rmse: float
    median: float
    p90: float
    flatness: float


def local_frame(lat: ArrayLike, lon: ArrayLike, origin: tuple[float, float] | None = None) -> Array:
    """Return positions in metres east and north of an origin.

    Parameters
    ----------
    lat, lon : array_like, shape (N,)
        Latitude and longitude, in degrees (north and east positive).
    origin : tuple of float, optional
        The origin's latitude and longitude, in degrees; the first fix if None.

    Returns
    -------
    ndarray of float64, shape (N, 2)
        East and north, in metres.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the inputs are empty, of different lengths, not finite or out of range.
    """
    la = np.asarray(lat, dtype=np.float64)
    lo = np.asarray(lon, dtype=np.float64)
    if la.ndim != 1 or la.shape != lo.shape or la.size == 0:
        raise InvalidInputError("lat, lon: expected two non-empty arrays of one length")
    if not (np.all(np.isfinite(la)) and np.all(np.isfinite(lo))):
        raise InvalidInputError("lat, lon: expected finite degrees")
    if np.any(np.abs(la) > 90.0) or np.any(np.abs(lo) > 180.0):
        raise InvalidInputError("lat, lon: out of range")
    lat0, lon0 = origin if origin is not None else (float(la[0]), float(lo[0]))
    east = EARTH_RADIUS * np.radians(lo - lon0) * math.cos(math.radians(lat0))
    north = EARTH_RADIUS * np.radians(la - lat0)
    return np.stack([east, north], axis=1)


def _start(t: Array, z: Array) -> tuple[Array, float]:
    """Return the filter's first state, fitted to the first fixes, and their span in seconds.

    The velocity is the median of the pairs' (Theil-Sen), the position at the first time the
    median of the fixes taken back to it along that velocity.
    """
    n = min(START_FIXES, t.size)
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    velocity = np.median([(z[j] - z[i]) / (t[j] - t[i]) for i, j in pairs], axis=0)
    position = np.median(z[:n] - np.outer(t[:n] - t[0], velocity), axis=0)
    return np.concatenate([position, velocity]), float(t[n - 1] - t[0])


def smooth(
    times: ArrayLike,
    position: ArrayLike,
    *,
    sigma: float = 4.0,
    accel: float = 1.5,
    gate: float = GATE,
) -> Track:
    """Filter and smooth fixes: a gated constant-velocity Kalman filter, then RTS.

    Parameters
    ----------
    times : array_like, shape (N,)
        The fixes' times, in seconds, increasing.
    position : array_like, shape (N, 2)
        The fixes, in metres (:func:`local_frame`).
    sigma : float, optional
        A fix's standard deviation per axis, in metres.
    accel : float, optional
        The white acceleration's standard deviation, in m/s² (the process noise).
    gate : float, optional
        The χ² bound on a fix's innovation (:data:`GATE`).

    Returns
    -------
    Track
        The smoothed track, and which fixes the gate refused.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If there are fewer than 2 fixes, the times do not increase, a value is not finite, or a
        noise or the gate is not positive.
    """
    t = np.asarray(times, dtype=np.float64)
    z = np.asarray(position, dtype=np.float64)
    if t.ndim != 1 or z.shape != (t.size, 2) or t.size < 2:
        raise InvalidInputError("times, position: expected (N,) and (N, 2), N >= 2")
    if not (np.all(np.isfinite(t)) and np.all(np.isfinite(z))):
        raise InvalidInputError("times, position: expected finite values")
    if np.any(np.diff(t) <= 0.0):
        raise InvalidInputError("times: expected increasing times")
    for name, value in (("sigma", sigma), ("accel", accel), ("gate", gate)):
        if not (math.isfinite(value) and value > 0.0):
            raise InvalidInputError(f"{name}: expected a positive value, got {value}")
    n = t.size
    h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
    r = np.eye(2) * sigma**2
    x, span = _start(t, z)
    spread = max(2.0 * sigma**2 / span**2, 1.0)  # the start's velocity, to 1 m/s at best
    p = np.diag([sigma**2, sigma**2, spread, spread])
    in_a_row = 0
    filtered = np.empty((n, 4))
    covs = np.empty((n, 4, 4))
    predicted = np.empty((n, 4))
    predicted_covs = np.empty((n, 4, 4))
    transitions = np.empty((n, 4, 4))
    refused = np.zeros(n, dtype=bool)
    for k in range(n):
        f = np.eye(4)
        if k:
            d = t[k] - t[k - 1]
            f[0, 2] = f[1, 3] = d
            q = accel**2 * np.array(
                [
                    [d**4 / 4, 0, d**3 / 2, 0],
                    [0, d**4 / 4, 0, d**3 / 2],
                    [d**3 / 2, 0, d**2, 0],
                    [0, d**3 / 2, 0, d**2],
                ]
            )
            x, p = f @ x, f @ p @ f.T + q
        predicted[k], predicted_covs[k], transitions[k] = x, p, f
        s = h @ p @ h.T + r
        nu = z[k] - h @ x
        if float(nu @ np.linalg.solve(s, nu)) > gate:
            in_a_row += 1
            if in_a_row < RESTART:
                refused[k] = True  # predicted only: the fix is not believed
                filtered[k], covs[k] = x, p
                continue
            p = p + np.diag(LET_GO)  # the filter has lost the car: take this fix
            predicted_covs[k] = p  # and the smoother lets go too, not pulling the past along
            s = h @ p @ h.T + r
        in_a_row = 0
        gain = p @ h.T @ np.linalg.inv(s)
        x = x + gain @ nu
        p = (np.eye(4) - gain @ h) @ p
        filtered[k], covs[k] = x, p
    for k in range(n - 2, -1, -1):
        c = covs[k] @ transitions[k + 1].T @ np.linalg.inv(predicted_covs[k + 1])
        filtered[k] = filtered[k] + c @ (filtered[k + 1] - predicted[k + 1])
        covs[k] = covs[k] + c @ (covs[k + 1] - predicted_covs[k + 1]) @ c.T
    return Track(t, filtered[:, :2].copy(), filtered[:, 2:].copy(), covs, refused)


def align_planar(source: ArrayLike, target: ArrayLike) -> tuple[float, Array, Array]:
    """Return the 2D similarity taking `source` onto `target` (Umeyama), least squares.

    Parameters
    ----------
    source, target : array_like, shape (N, 2)
        Corresponding points, N >= 2, the source not all one point.

    Returns
    -------
    tuple
        The scale, the rotation (2, 2) and the translation (2,): ``target ≈ s R source + t``.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If the shapes differ, there are fewer than 2 points, or the source does not spread.
    """
    a = np.asarray(source, dtype=np.float64)
    b = np.asarray(target, dtype=np.float64)
    if a.ndim != 2 or a.shape != b.shape or a.shape[1] != 2 or a.shape[0] < 2:
        raise InvalidInputError("source, target: expected two (N, 2) arrays, N >= 2")
    mu_a, mu_b = a.mean(axis=0), b.mean(axis=0)
    da, db = a - mu_a, b - mu_b
    spread = float((da**2).sum(axis=1).mean())
    if spread <= 0.0:
        raise InvalidInputError("source: the points do not spread")
    u, sv, vt = np.linalg.svd(db.T @ da / len(a))
    d = np.diag([1.0, float(np.sign(np.linalg.det(u @ vt)) or 1.0)])
    rotation = u @ d @ vt
    scale = float(np.trace(np.diag(sv) @ d) / spread)
    return scale, rotation, mu_b - scale * rotation @ mu_a


def score(run: runs.Run, track: Track, clock: ArrayLike, min_frames: int = 30) -> list[MapScore]:
    """Score each of a run's maps against a smoothed track, in its plane of motion.

    Parameters
    ----------
    run : slam_it.eval.runs.Run
        The run: its final trajectory, its frames' maps.
    track : Track
        The smoothed track.
    clock : array_like, shape (F,)
        Each of the run's tracked frames' times on the track's clock, in seconds.
    min_frames : int, optional
        The fewest frames within the track's span for a map to be scored.

    Returns
    -------
    list of MapScore
        One per map scored, by map number.

    Raises
    ------
    slam_it.foundations.errors.InvalidInputError
        If `clock` is not one finite time per tracked frame.
    """
    times = np.asarray(clock, dtype=np.float64)
    if times.shape != (len(run.timestamps),) or not np.all(np.isfinite(times)):
        raise InvalidInputError(f"clock: expected {len(run.timestamps)} finite times")
    maps = np.asarray(run.pose_maps)
    positions = np.asarray(run.poses, dtype=np.float64)[:, :3, 3]
    inside = (times >= track.times[0]) & (times <= track.times[-1])
    out: list[MapScore] = []
    for number in sorted({int(m) for m in maps}):
        chosen = (maps == number) & inside
        if int(chosen.sum()) < min_frames:
            continue
        p = positions[chosen]
        centred = p - p.mean(axis=0)
        _, extents, axes = np.linalg.svd(centred, full_matrices=False)
        if extents[1] <= 0.0:
            continue
        planar = centred @ axes[:2].T
        reference = track.at(times[chosen])
        # the plane's two axes may come out mirrored: try both, keep the better
        fits = []
        for flip in (1.0, -1.0):
            candidate = planar * np.array([1.0, flip])
            s, rotation, shift = align_planar(candidate, reference)
            err = np.linalg.norm((s * (rotation @ candidate.T)).T + shift - reference, axis=1)
            fits.append((float(np.sqrt((err**2).mean())), s, err))
        rmse, scale, err = min(fits, key=lambda fit: fit[0])
        out.append(
            MapScore(
                number,
                int(chosen.sum()),
                scale,
                rmse,
                float(np.median(err)),
                float(np.percentile(err, 90)),
                float(extents[2] / extents[0]) if extents.size > 2 else 0.0,
            )
        )
    return out
