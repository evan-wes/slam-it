# Lie groups: SO(3)

Design note for `slamlab.foundations.so3` (Python reference), `slamlab::foundations::so3`
(C++ core), and the Rust subset. SE(3) is added to this note with its own implementation.

## The problem

Every rung estimates rotations, either directly or as part of a pose. A rotation has three
degrees of freedom, but its matrix has nine entries tied together by six constraints
(RᵀR = I, det R = +1). An optimiser can't step freely in that matrix space, because each
step leaves the set of rotations.

The Lie-group view resolves this. Rotations form the group SO(3). Near any rotation, the
group looks like a flat 3-vector space, its tangent space or Lie algebra so(3). So:
- a solver works in that flat space, on a small 3-vector δ;
- it maps δ back onto the group with the exponential map: `R ⊕ δ = R · exp(δ)`.

Rung 0 needs five things from this module:
- the maps `exp` and `log`;
- `hat` and `vee`, which convert between 3-vectors and skew-symmetric matrices;
- the Jacobians that link a step in the tangent space to its effect on the group;
- conversions to and from quaternions;
- a way to repair a matrix that has drifted off the group.

## The maths

**hat and vee.** `hat(ω)` is the skew-symmetric matrix with `hat(ω) u = ω × u`. `vee` inverts
it.

**Exponential (Rodrigues' formula).** For `ω = θ n`, with unit axis n and angle θ, and
`W = hat(ω)`:

```
exp(ω) = I + (sin θ / θ) W + ((1 - cos θ) / θ²) W²
```

**Logarithm.** The skew part of R is `v = sin θ · n`, and its trace gives
`cos θ = (tr R - 1)/2`. Then:

```
θ = atan2(|v|, cos θ),   ω = θ · v / |v|
```

**Jacobians.** With right perturbations, `exp(ω + δ) ≈ exp(ω) exp(Jr(ω) δ)`, where:

```
Jr(ω) = I - ((1 - cos θ)/θ²) W + ((θ - sin θ)/θ³) W²
Jl(ω) = Jr(-ω)
Jr⁻¹(ω) = I + ½ W + (1/θ² - cot(θ/2)/(2θ)) W²
```

The inverses are singular at θ = 2π, so larger angles are rejected.

**Quaternions.** Hamilton convention, stored `(x, y, z, w)`. `q` and `-q` are the same
rotation, so `to_quaternion` returns the one with `w >= 0`. It uses Shepperd's method: take
whichever of (trace, R₀₀, R₁₁, R₂₂) is largest from the diagonal, and the rest from
off-diagonal sums.

**Projection.** For `M = U S Vᵀ`, the nearest rotation in the Frobenius norm is
`U diag(1, 1, det(U Vᵀ)) Vᵀ`.

## Numerical choices

Each formula above has a numerical trap. Each was handled the same way in Python and C++,
with shared thresholds:

| Quantity | Trap | Choice |
|---|---|---|
| `(1 - cos θ)/θ²` | Catastrophic cancellation at small θ | Rewritten as `½ (sin(θ/2)/(θ/2))²`: no subtraction at all |
| `sin θ / θ`, `θ / sin θ` | 0/0 at θ = 0 | Series below 1e-4 (truncation below 1e-17) |
| `(θ - sin θ)/θ³`, `1/θ² - cot(θ/2)/(2θ)` | Cancellation of about 6 eps/θ² | Series below 0.05, with four terms (truncation below 1e-18); about 5e-13 for the closed form at the switch |
| The angle in `log` | `arccos((tr R - 1)/2)` returns NaN when rounding pushes its argument past ±1, and loses precision near 0 and π | `atan2(\|v\|, cos θ)`: no domain to leave, accurate everywhere |
| The axis near θ = π | `v = sin θ · n` vanishes, so `v/\|v\|` amplifies rounding | Below `sin θ = 1e-3` with `cos θ < 0`, take n from the symmetric part, `(R + Rᵀ)/2 - cos θ I = (1 - cos θ) n nᵀ`, and its sign from v |
| θ = π exactly | n and -n are the same rotation | Deterministic: the largest-magnitude component positive, in every backend |

**Not a rotation.** `log` and `to_quaternion` reject a matrix that is not a rotation (RᵀR
off by more than 1e-6, or det ≤ 0) with `InvalidInputError`. They never repair it silently.
`project_to_so3` is the explicit repair. The tolerance is loose enough for rotations that
passed through float32, as VGGT's do.

## How it's checked

- **Parity fixture** (`tests/fixtures/foundations/so3.json`, 165 cases): 16 inputs per
  function, covering zero, 1e-12, both sides of both Taylor switches, π − 5e-4, π − 1e-6,
  π − 1e-12, exactly π, above π, near 2π, and values float32 can't represent. Every backend
  is compared against the fixture under a per-function tolerance that states its reason.
- **The oracle is mpmath at 50 digits,** using algorithms independent of ours:
  - Rodrigues for `exp`;
  - a quaternion route for `log`, `2 atan2(|q_v|, q_w)`;
  - matrix inversion for the inverse Jacobians;
  - mpmath's own SVD for the projection.

  It shares no code and no rounding with the implementations.
- **Properties:**
  - round trips: `log(exp(ω))`, `vee(hat(ω))`, and quaternion round trips;
  - orthonormality and det = +1;
  - `Jr` against central differences;
  - `Jl(ω) = Jr(-ω)`, and `Jr⁻¹ Jr = I`;
  - the double cover;
  - batch shapes;
  - every invalid input, rejected the same way in every backend.
- **Cross-checks at moderate angles:** SciPy's `Rotation` for `exp`, manif for `Jr`.

## What was learned

- **manif is not a reliable oracle at small angles.** Measured against 50-digit mpmath:
  - manif's `Jr` is off by 3.6e-11 at θ = 1e-6 and 1.3e-13 at θ = 1e-4;
  - this implementation is within 1e-20 at both.

  The disagreement first showed up as a mismatch between our Jacobians and manif's. An
  independent high-precision referee settled which one was wrong. **A library is a
  cross-check, not ground truth;** the fixtures use mpmath.
- **The oracle can be the bug.**
  - My first mpmath quaternion read every component from the diagonal. For a float rotation
    of 1e-12, the diagonal has rounded to exactly 1, so the angle was lost: high precision
    can't recover information that float64 rounding has already removed. Shepperd's method
    takes the small components from the off-diagonal entries.
  - Building `hat`'s expected values in mpmath dropped the sign of `-0.0`. The exact-match
    check compares signed zeros, and caught it.
- **Conditioning, not just precision.** Projecting a near-reflection onto SO(3) flips its
  weakest singular direction, which amplifies rounding by about 1/(σ₂ − σ₃). That case
  needs a 1e-13 tolerance where the others pass at 1e-15. The fixture records the reason.
