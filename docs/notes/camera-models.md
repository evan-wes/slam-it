# Camera models

Design note for `slamlab.foundations.camera` (Python reference) and
`slamlab::foundations::camera` (C++ core).

## The problem

Every geometric estimate in the ladder is measured in pixels: reprojection error is
`observed pixel − project(camera, point)`. So rung 0 needs:
- the projection;
- its derivatives with respect to the point and to the intrinsics, because BA refines
  focal length (milestone 3: VGGT's focal is itself an estimate);
- the inverse, from pixels to rays. Two-view geometry, PnP and triangulation work on rays.

## The models

| Model | Intrinsics | Used by |
|---|---|---|
| `pinhole` | `[fx, fy, cx, cy]` | rectified images, VGGT cameras |
| `radtan` | `[fx, fy, cx, cy, k1, k2, p1, p2, k3]` | OpenCV calibrations, TUM RGB-D, EuRoC |

**Projection.** Points are in the camera frame (x right, y down, z forward).
1. Normalise: `x = X/Z`, `y = Y/Z`.
2. For radtan, apply Brown-Conrady distortion, with `r² = x² + y²`:
   - radial factor `1 + k1 r² + k2 r⁴ + k3 r⁶`;
   - tangential terms `2 p1 x y + p2 (r² + 2x²)` (and their mirror for y).
3. Map to pixels: `u = fx x_d + cx`, `v = fy y_d + cy`.

Pixels follow OpenCV: the top-left pixel's centre is `(0, 0)`. COLMAP's `(0.5, 0.5)`
convention is converted by its reader.

Points at or behind `Z = 1e-9` are rejected rather than projected through the camera centre.
BA checks cheirality before it ever calls projection.

## Jacobians

`project_jacobians` returns `d(u, v)/d(X, Y, Z)` and `d(u, v)/d(intrinsics)`, by the chain
rule through normalisation, distortion and the focal map. The distortion's own 2x2 Jacobian
is shared with undistortion.

## Unprojection: pixels to bearing vectors

The calibrated solvers (8-point, 5-point, PnP, triangulation) take **unit bearing vectors**,
not normalised image points. A bearing is defined for any ray, including those at or behind
90° from the axis that wide-angle lenses produce. So adding a fisheye model in milestone 6
changes only this module.

Radtan has no closed-form inverse. `unproject` solves `distort(x) = x_d` by Newton's method:
- it starts at `x = x_d`;
- it uses the analytic 2x2 Jacobian;
- it stops at 1e-15 in normalised units, within 30 iterations.

Two failures are reported as `NotConvergedError`, never as a wrong ray:
- The **distortion Jacobian's determinant is not positive.** The map is not locally
  invertible there: the pixel is beyond the radius where the lens model folds back on
  itself.
- **Newton does not converge.**

OpenCV's `undistortPoints` runs a fixed 5 iterations with no check, which is why it is not
used as an oracle here.

## How it's checked

- **Parity fixture** (`camera.json`, 45 cases):
  - cameras: pinhole, a strongly distorting radtan lens (TUM fr1-like), and a mild one
    (EuRoC-like);
  - points: on the optical axis, general, near the image corners, very close (Z = 0.05),
    and far (Z = 400).
- **The oracle (mpmath, 50 digits):**
  - projection by its defining formula;
  - the Jacobians by `mp.diff`, numerical differentiation in high precision, so the analytic
    chain rule under test is not reused;
  - unprojection by `mp.findroot` on the distortion equations.
- **Properties:**
  - `unproject(project(p)) = p/|p|`;
  - the Jacobians against central differences in every point coordinate and every intrinsic;
  - radtan with zero coefficients equals pinhole exactly;
  - the uninvertible pixel raises `NotConvergedError`;
  - behind-camera points, a bad intrinsics length and non-positive focal lengths are
    rejected.
