"""A synthetic textured room for the edge rung's tests: images, depth and a known trajectory.

A box-shaped room with a few boxes standing in it, every face covered in a texture of
overlapping rectangles (strong corners, so FAST finds plenty at every scale). A pinhole camera
moves along a smooth path inside; each frame is ray-cast exactly:

- **Geometry:** every pixel's ray is intersected with the room (from inside, its exit face)
  and with each box (from outside, its entry face); the nearest hit wins. Rays leave through
  the pixel's centre and the four quarter-pixel points around it (a 2x2 supersample), which
  are averaged.
- **Texture:** each face has its own texture image, sampled bilinearly at the hit's face
  coordinates; there is no lighting, so a surface looks the same from every view.
- **Depth:** the hit's z in the camera, the centre ray's, in metres.
- **Stereo:** a second camera :data:`BASELINE` metres to the right of the first, the same
  orientation and intrinsics (an already-rectified pair).

Everything is deterministic: the room, the textures and the path are fixed by a seed. The
renderer is test infrastructure, never imported by the package.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import gaussian_filter

from slam_it.foundations import se3, so3

Array = NDArray[np.float64]

ROOM = ((-3.0, 3.0), (-1.5, 1.5), (-4.0, 4.0))
"""The room's extent along x, y (down) and z, in metres."""

BOXES = (
    ((-2.2, -1.2), (0.5, 1.5), (1.0, 2.0)),
    ((0.8, 1.6), (-0.2, 1.5), (2.2, 3.0)),
    ((1.5, 2.5), (0.7, 1.5), (-1.5, -0.6)),
    ((-1.8, -1.0), (1.0, 1.5), (-3.0, -2.2)),
)
"""Boxes standing on the floor (y = 1.5), each ``((x0, x1), (y0, y1), (z0, z1))``."""

TEXELS_PER_METRE = 100.0
"""Each texture's resolution on its face."""

TEXTURE_BLUR = 1.0
"""The textures' Gaussian blur, in texels: softened edges keep corners where they are under
sub-pixel motion (keypoint repeatability 0.6 against 0.57 unblurred, measured three frames
apart)."""

BASELINE = 0.1
"""The stereo pair's baseline, in metres."""


@dataclass(frozen=True)
class Camera:
    """A pinhole camera: ``(fx, fy, cx, cy)`` and the image size."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @property
    def intrinsics(self) -> Array:
        """The ``pinhole`` intrinsics vector."""
        return np.array([self.fx, self.fy, self.cx, self.cy])


SMALL = Camera(fx=262.5, fy=262.5, cx=159.5, cy=119.5, width=320, height=240)
"""A 320x240 camera with TUM's field of view, for fast tests."""


@dataclass(frozen=True)
class Frame:
    """One rendered frame."""

    timestamp: float
    gray: NDArray[np.uint8]
    depth: Array
    right: NDArray[np.uint8] | None
    T_world_cam: Array


def _texture(rng: np.random.Generator, width_m: float, height_m: float) -> Array:
    """Draw a face's texture: a smooth gradient under random triangles, at three scales.

    Triangles at random orientations give corners of every angle, so descriptors differ
    from one corner to the next (rectangles of random gray would all look alike).
    """
    w = max(8, round(width_m * TEXELS_PER_METRE))
    h = max(8, round(height_m * TEXELS_PER_METRE))
    y, x = np.mgrid[0:h, 0:w].astype(np.float64)
    gx, gy = rng.normal(0.0, 0.4, 2)
    image = 128.0 + gx * (x - w / 2) + gy * (y - h / 2)
    for size, count in ((0.9, 3), (0.4, 12), (0.15, 45)):
        span = size * TEXELS_PER_METRE
        for _ in range(round(count * width_m * height_m)):
            corners = rng.uniform([-span, -span], [w + span, h + span]) + rng.normal(
                0.0, span / 2, (3, 2)
            )
            x0, y0 = np.maximum(np.floor(corners.min(axis=0)).astype(int), 0)
            x1, y1 = np.minimum(np.ceil(corners.max(axis=0)).astype(int) + 1, [w, h])
            if x0 >= x1 or y0 >= y1:
                continue
            px, py = x[y0:y1, x0:x1], y[y0:y1, x0:x1]
            (ax, ay), (bx, by), (cx, cy) = corners
            d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by)
            d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy)
            d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay)
            inside = ~(((d1 < 0) | (d2 < 0) | (d3 < 0)) & ((d1 > 0) | (d2 > 0) | (d3 > 0)))
            image[y0:y1, x0:x1][inside] = rng.uniform(0, 255)
    blurred = np.asarray(gaussian_filter(image, TEXTURE_BLUR), dtype=np.float64)
    return np.clip(blurred, 0.0, 255.0)


@dataclass(frozen=True)
class _Face:
    """One textured face: the plane ``axis = value``, its two in-plane axes and texture."""

    axis: int
    value: float
    u_axis: int
    v_axis: int
    u0: float
    v0: float
    texture: Array


def _faces(box: tuple[tuple[float, float], ...], rng: np.random.Generator) -> list[_Face]:
    """Each of a box's six faces, textured."""
    out = []
    for axis in range(3):
        u_axis, v_axis = [a for a in range(3) if a != axis]
        (u0, u1), (v0, v1) = box[u_axis], box[v_axis]
        for value in box[axis]:
            out.append(_Face(axis, value, u_axis, v_axis, u0, v0, _texture(rng, u1 - u0, v1 - v0)))
    return out


def _sample(texture: Array, u: Array, v: Array) -> Array:
    """Sample a texture bilinearly at texel coordinates (clamped to its edges)."""
    h, w = texture.shape
    x = np.clip(u, 0.0, w - 1.0)
    y = np.clip(v, 0.0, h - 1.0)
    x0 = np.minimum(np.floor(x).astype(np.int64), w - 2) if w > 1 else np.zeros_like(x, int)
    y0 = np.minimum(np.floor(y).astype(np.int64), h - 2) if h > 1 else np.zeros_like(y, int)
    fx, fy = x - x0, y - y0
    top = texture[y0, x0] * (1 - fx) + texture[y0, x0 + 1] * fx
    bottom = texture[y0 + 1, x0] * (1 - fx) + texture[y0 + 1, x0 + 1] * fx
    return np.asarray(top * (1 - fy) + bottom * fy, dtype=np.float64)


class Room:
    """The room and its boxes, textured from a seed."""

    def __init__(self, seed: int = 0) -> None:
        """Texture every face.

        Parameters
        ----------
        seed : int, optional
            The textures' seed.
        """
        rng = np.random.default_rng(seed)
        self.boxes = [ROOM, *BOXES]
        self.faces = [_faces(box, rng) for box in self.boxes]

    def cast(self, origin: Array, directions: Array) -> tuple[Array, Array]:
        """Intersect rays with the room and boxes; return each ray's distance and intensity.

        Parameters
        ----------
        origin : ndarray, shape (3,)
            The rays' common origin, inside the room and outside every box.
        directions : ndarray, shape (R, 3)
            The rays' directions (any length; distances are in their units).

        Returns
        -------
        tuple of ndarray, shape (R,)
            The distance along each ray to its hit, and the hit's intensity.
        """
        r = len(directions)
        best = np.full(r, np.inf)
        shade = np.zeros(r)
        with np.errstate(divide="ignore", invalid="ignore"):
            inverse = 1.0 / directions
        for index, (box, faces) in enumerate(zip(self.boxes, self.faces, strict=True)):
            lo = (np.array([b[0] for b in box]) - origin) * inverse
            hi = (np.array([b[1] for b in box]) - origin) * inverse
            near, far = np.minimum(lo, hi), np.maximum(lo, hi)
            if index == 0:  # the room, from inside: the exit
                t = np.nanmin(far, axis=1)
                axis = np.nanargmin(far, axis=1)
            else:  # a box, from outside: the entry, if the ray meets it ahead
                entry = np.nanmax(near, axis=1)
                exit_ = np.nanmin(far, axis=1)
                t = np.where((entry <= exit_) & (entry > 1e-9), entry, np.inf)
                axis = np.nanargmax(near, axis=1)
            closer = t < best
            if not np.any(closer):
                continue
            best = np.where(closer, t, best)
            hits = origin + directions[closer] * t[closer, None]
            for face in faces:
                on = (axis[closer] == face.axis) & np.isclose(hits[:, face.axis], face.value)
                if not np.any(on):
                    continue
                u = (hits[on, face.u_axis] - face.u0) * TEXELS_PER_METRE
                v = (hits[on, face.v_axis] - face.v0) * TEXELS_PER_METRE
                where = np.flatnonzero(closer)[on]
                shade[where] = _sample(face.texture, u, v)
        return best, shade

    def render(
        self,
        camera: Camera,
        T_world_cam: Array,  # noqa: N803 -- the frame convention's name
        *,
        stereo: bool = False,
    ) -> tuple[NDArray[np.uint8], Array, NDArray[np.uint8] | None]:
        """Render one view (and its right view): gray image, depth, right image.

        Parameters
        ----------
        camera : Camera
            The intrinsics and size.
        T_world_cam : ndarray, shape (4, 4)
            The camera's pose.
        stereo : bool, optional
            Also render the right camera, :data:`BASELINE` to the right.

        Returns
        -------
        tuple
            The gray image (H, W) uint8, the depth (H, W) in metres, the right image or None.
        """
        gray, depth = self._view(camera, T_world_cam)
        right = None
        if stereo:
            offset = np.eye(4)
            offset[0, 3] = BASELINE
            right, _ = self._view(camera, T_world_cam @ offset)
        return gray, depth, right

    def _view(
        self,
        camera: Camera,
        T_world_cam: Array,  # noqa: N803 -- the frame convention's name
    ) -> tuple[NDArray[np.uint8], Array]:
        """Render one camera: the 2x2-supersampled image and the centre rays' depth."""
        h, w = camera.height, camera.width
        v, u = np.mgrid[0:h, 0:w].astype(np.float64)
        rotation, origin = T_world_cam[:3, :3], T_world_cam[:3, 3]
        total = np.zeros((h, w))
        depth = np.zeros((h, w))
        offsets = [(-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25), (0.0, 0.0)]
        for k, (du, dv) in enumerate(offsets):
            rays = np.stack(
                [
                    (u + du - camera.cx) / camera.fx,
                    (v + dv - camera.cy) / camera.fy,
                    np.ones_like(u),
                ],
                axis=-1,
            ).reshape(-1, 3)
            distance, shade = self.cast(origin, rays @ rotation.T)
            if k < 4:
                total += shade.reshape(h, w)
            else:
                depth = distance.reshape(h, w)  # z, since the rays have unit z
        gray = np.clip(np.floor(total / 4.0 + 0.5), 0, 255).astype(np.uint8)
        return gray, depth


def path(frames: int, rate: float = 30.0) -> list[Array]:
    """Return a smooth trajectory inside the room: ``T_world_cam`` for each frame.

    The camera sweeps sideways along a slow Lissajous curve, drifting forward, looking
    roughly down the room's z axis while it yaws and pitches a little: about 0.5 m/s and 10
    degrees per second at 30 frames a second. Mostly sideways motion gives a monocular
    initialisation parallax without scaling the scene (which moves keypoints off level 0).

    Parameters
    ----------
    frames : int
        How many poses.
    rate : float, optional
        Frames per second.

    Returns
    -------
    list of ndarray, shape (4, 4)
        The poses.
    """
    out = []
    for i in range(frames):
        t = i / rate
        position = np.array(
            [1.2 * math.sin(0.45 * t), 0.15 * math.sin(1.1 * t) - 0.2, -2.5 + 0.12 * t]
        )
        yaw = 0.3 * math.sin(0.4 * t)
        pitch = 0.08 * math.sin(0.9 * t)
        pose = np.eye(4)
        pose[:3, :3] = so3.exp([0.0, yaw, 0.0]) @ so3.exp([pitch, 0.0, 0.0])
        pose[:3, 3] = position
        out.append(pose)
    return out


LOOP_RADIUS = 0.6
"""The loop path's circle's radius, in metres."""

LOOP_TURN = 2.0 * math.pi * 7.0 / 6.0
"""How far the loop path turns: once round, and a sixth more, back over where it began."""

LOOP_FRAMES = 280
"""The loop path's frames at 30 a second: its turn at about 45 degrees a second."""

LOOP_PITCH = 0.25
"""How far the loop path's camera looks up, in radians (y is down): the ceiling in view above
the walls, so the depths across an image differ and a turn is told from a step sideways
(looking level at a wall 2.4 m off, RGB-D's tracking jumped by up to 3 degrees)."""


def loop_path(frames: int, rate: float = 30.0) -> list[Array]:
    """Return a trajectory that comes back to where it began: ``T_world_cam`` for each frame.

    The camera goes round a circle of :data:`LOOP_RADIUS` about the room's centre, looking
    outward and :data:`LOOP_PITCH` up as it goes, so it moves sideways to its view (a
    monocular start's parallax) and sees each wall in turn; over ``frames`` it turns
    :data:`LOOP_TURN`, so the walls it first saw come back into view after it has looked
    everywhere else: a loop to close. It bobs a few centimetres, so the views are not all at
    one height.

    Parameters
    ----------
    frames : int
        How many poses.
    rate : float, optional
        Frames per second (the bobbing's clock).

    Returns
    -------
    list of ndarray, shape (4, 4)
        The poses.
    """
    out = []
    for i in range(frames):
        angle = LOOP_TURN * i / max(frames - 1, 1)
        t = i / rate
        position = np.array(
            [
                LOOP_RADIUS * math.sin(angle),
                -0.2 + 0.05 * math.sin(1.3 * t),
                LOOP_RADIUS * math.cos(angle) - 0.4,
            ]
        )
        pose = np.eye(4)
        tilt = LOOP_PITCH + 0.06 * math.sin(0.7 * t)
        pose[:3, :3] = so3.exp([0.0, angle, 0.0]) @ so3.exp([tilt, 0.0, 0.0])
        pose[:3, 3] = position
        out.append(pose)
    return out


def sequence(
    frames: int,
    camera: Camera = SMALL,
    *,
    stereo: bool = False,
    seed: int = 0,
    rate: float = 30.0,
    loop: bool = False,
) -> list[Frame]:
    """Render a sequence along :func:`path`, or :func:`loop_path`.

    Parameters
    ----------
    frames : int
        How many frames.
    camera : Camera, optional
        The camera (default: :data:`SMALL`).
    stereo : bool, optional
        Render the right views too.
    seed : int, optional
        The room's texture seed.
    rate : float, optional
        Frames per second (the timestamps' spacing).
    loop : bool, optional
        Follow :func:`loop_path`, which comes back to where it began (default: :func:`path`).

    Returns
    -------
    list of Frame
        The frames.
    """
    room = Room(seed)
    out = []
    for i, pose in enumerate((loop_path if loop else path)(frames, rate)):
        gray, depth, right = room.render(camera, pose, stereo=stereo)
        out.append(Frame(i / rate, gray, depth, right, pose))
    return out


def with_right_views(frames: list[Frame], camera: Camera = SMALL, *, seed: int = 0) -> list[Frame]:
    """Render the right views of already rendered frames.

    Parameters
    ----------
    frames : list of Frame
        Frames rendered with the same camera and seed, without right views.
    camera : Camera, optional
        The camera (default: :data:`SMALL`).
    seed : int, optional
        The room's texture seed.

    Returns
    -------
    list of Frame
        The frames, each with its right view.
    """
    room = Room(seed)
    offset = np.eye(4)
    offset[0, 3] = BASELINE
    out = []
    for f in frames:
        right, _ = room._view(camera, f.T_world_cam @ offset)
        out.append(Frame(f.timestamp, f.gray, f.depth, right, f.T_world_cam))
    return out


def relative(a: Array, b: Array) -> Array:
    """Return ``T_a_b`` from two ``T_world_cam`` poses (``a⁻¹ b``)."""
    return np.asarray(se3.inverse(a) @ b)
