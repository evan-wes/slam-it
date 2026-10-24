# Review panel: `feature/ba-held-cameras-constant`

## Run 1 -- 2026-10-23

| | |
|---|---|
| **Branch** | `feature/ba-held-cameras-constant` |
| **Range** | `dfdc511a...6d0aef35` (base: `origin/develop`) |
| **Reviewers** | algorithm (opus) alone: the PR changes how bundle adjustment and the pose graph hold fixed cameras, the algorithm reviewer's area; hygiene and architecture not run (MICRO change) |
| **How they ran** | registered slam-it-* agents |
| **Layer 1** | at `6d0aef35`: all 29 checks passed (`scripts/check.sh --env slam-it`, exit 0; pytest 5,157 passed, 1 skipped, 19 deselected) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L1 | should-fix | algorithm | `cpp/src/foundations/graphs.cpp:355-386,541-568`, Rust and Python twins, `CHANGELOG.md:29-31` | GTSAM's inner λ search no longer sees the error of edges with both ends held, so a converged pose graph can end at `damping_limit` | fix (a): carry the constant inside GTSAM's graph as a zero-Jacobian factor, dropping `solve()`'s `constant` | fixed (c8d997c3: one factor with a zero Jacobian carries the both-held edges' residuals inside GTSAM's graph; `solve()`'s constant removed; the deterministic case a test in every language) -- decided by evan-wes, 2026-10-23 |
| L2 | should-fix | algorithm | `CHANGELOG.md:47-49` | advio-20's "same result bit for bit" is vacuous: neither version accepts a step | fix (say so) | fixed (bad85b60: neither version accepts a step there) -- decided by evan-wes, 2026-10-23 |
| L3 | note | algorithm | `CHANGELOG.md:58` | The cost-agreement figure's scope is unstated (the global BA's own cost vs the later solves') | fix (state the scope) | fixed (bad85b60: the global BA's own cost to 1.6e-10, the later solves' to 2.2e-7) -- decided by evan-wes, 2026-10-23 |
| L4 | note | algorithm | `CHANGELOG.md:55-59` | The gauge explanation is right but overstates roundoff's role on this branch; damping alone will not hold the scale | fix (reword); a scale test before and after global BA goes to the damping PR | fixed (bad85b60: reworded); the scale test before and after global BA deferred (the damping PR, G) -- decided by evan-wes, 2026-10-23 |
| L5 | note | algorithm | `cpp/src/foundations/graphs.cpp:358-361` and twins | GTSAM's absolute λ bound (1e5) is tiny next to BA's Hessian; advio-20's global BA accepts no step in either version | defer (the damping PR: λ0 scaled to max diag H, no practical upper bound) | deferred (the damping PR, G: λ0 scaled to max diag H, no practical upper bound) -- decided by evan-wes, 2026-10-23 |
| L6 | note | algorithm | `src/slam_it/foundations/graphs.py:637-660` | `_held_end` whitens twice unless the noise is unit | fix (wrap with a unit noise model) | fixed (832564b6: unit noise wraps the whitened edge) -- decided by evan-wes, 2026-10-23 |
| L7 | note | algorithm | `src/slam_it/foundations/graphs.py:1029-1044` | The held stereo factor copies GTSAM's cheirality convention by hand, untested against the unheld factor behind the camera | fix (add that test) | fixed (04ff1893: held vs GenericStereoFactor3D behind the camera, bit for bit) -- decided by evan-wes, 2026-10-23 |
| L8 | note | algorithm | `tests/fixtures/foundations/generate_graphs.py` | The weld's typical stereo case (half the cameras held) is not in the parity fixture | defer (the damping PR, which regenerates the fixture) | deferred (the damping PR, G, which regenerates graphs.json) -- decided by evan-wes, 2026-10-23 |

**After the fixes** (`c8d997c3`, `832564b6`, `04ff1893`, `bad85b60`):

- Layer 1 at `bad85b60` (the tree of the implementer's `f4690d18`, its two earlier commits' messages since reworded): exit 0, all 29 checks ok (pytest 5,161 passed, 1 skipped, 19 deselected).
- The reviewer's `held_edge_termination_probe.py`, run against every backend after L1: the deterministic case ends `function`, converged, after 1 iteration, as develop; all 150 random graphs end as develop's, with the same iteration counts. The rooms' golden records regenerated after the fix are identical bit for bit.
- The reviewer was not run again: L1 is the fix it proposed (its option a), and its own probe checks it.
- The C++ and Rust commits' messages had put zju-B6's essential-graph cost differences (1.1e-12 and 3.1e-11 relative) down to the held edges' constant being summed apart; the L1 fix moved them by 4.4e-16 at most, so they come from the poses the graph starts from. Both messages were reworded on the unpushed branch (owner's decision, 2026-10-23), every commit's tree unchanged.

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Algorithm review

**Scope:** `dfdc511a...6d0aef35` (feature/ba-held-cameras-constant, 0.3.17.1 -> 0.3.17.2), five commits. I read these in full: `cpp/src/foundations/graphs.cpp`, `cpp/include/slam_it/foundations/graphs.hpp`, `rust/slam-it-foundations/src/graphs.rs`, `rust/slam-it-foundations/cxx/graphs_bridge.{cpp,hpp}`, `src/slam_it/foundations/graphs.py`, `cpp/tests/foundations/test_graphs.cpp`, `tests/foundations/test_graphs.py` and `tests/fixtures/foundations/generate_graphs.py`. I also went through the regenerated golden records (`tests/fixtures/edge/room_runs_*.json`), the CHANGELOG and the commit messages. The figures were checked against `../results/held_constants_2026-10-23/` and the implementer's held_constants scripts. References used: the GTSAM 4.3.0 source (`build/gtsam-4.3.0/src`), ORB-SLAM3 `src/Optimizer.cc` (master, github.com/UZ-SLAMLab/ORB_SLAM3) and Triggs et al. 2000. I also wrote my own read-only Python probes against the reference backend; no compiled backend was imported and nothing was built.

**Summary:** The change is correct, and it is the standard way to hold fixed vertices: g2o's `setFixed` and Ceres's `SetParameterBlockConstant` both take the vertex out of the state. Every new factor reproduces the factor it replaces:
- **Python:** verified bit for bit, including a point behind the camera, a Huber loss and the stereo right column.
- **C++ and Rust:** the same by construction (they share the residual code), and they match develop's constrained solves on the new held-heavy fixture cases to 6.2e-15.

Local BA is untouched. One behaviour differs from the CHANGELOG's "as before": GTSAM's inner λ search no longer sees the error of edges whose ends are both held. A converged pose graph can therefore now report `damping_limit` (converged = false). I reproduced this deterministically, and in 2 of 150 random graphs shaped like the essential graph.

The gauge explanation is right in substance. F1 adds no new freedom; it makes the stopping point along the free scale far less sensitive to roundoff. Every figure I checked reproduces. Two wordings in the record mislead, and there is no must-fix.

#### Findings

##### [should-fix] GTSAM's inner λ search no longer sees the held edges' constant, so converged pose graphs can report `damping_limit`
- **Where:**
  - `cpp/src/foundations/graphs.cpp:355-386` and `:541-568`
  - `rust/slam-it-foundations/src/graphs.rs:866-905` and `:1357` (`add_edge`)
  - `src/slam_it/foundations/graphs.py:504-560` (`_solve`) and `:690`
  - `CHANGELOG.md:29-31`: "so the summaries' errors and the stopping rule are the whole graph's, as before"
- **Issue:** SLAM It!'s own rule adds the constant back, but GTSAM's `tryLambda` has a rule of its own. It stops raising λ when `|costChange| < relativeErrorTol * currentState->error` (`build/gtsam-4.3.0/src/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp:223-231`), and that error is now the free part's error alone.
  - Suppose a trial step fails with |Δ| between relTol·E_free and relTol·(E_free + C).
  - **Develop** stopped searching, accepted no step, and SLAM It!'s rule then saw a decrease of 0 and reported `function`.
  - **HEAD** keeps raising λ up to `lambdaUpperBound` (1e5, `LevenbergMarquardtParams.h:74`), and SLAM It!'s rule reports `damping_limit`.
  - The acceptance test (`modelFidelity`) and accepted steps are unaffected, because C cancels in both cost changes. The `linearizedCostChange > eps·oldLinearizedError` guard is slightly looser.
- **Why it matters:** A converged essential-graph solve is reported as not converged. `loops` records the termination (`src/slam_it/edge/loops.py:1339`). In the affected iteration it also costs about 21 extra linear solves, from `essential_damping` 1e-16 up to 1e5. Solutions are unaffected.
- **Evidence:** run with the Python reference against an emulation of develop's form (NonlinearEquality, the same `graphs._solve`).
  - **Deterministic case:** three poses, edge (0, 1) with both ends held and error 0.00625, pose 2 free and in no edge.
    - HEAD: `iterations=1, termination=damping_limit, converged=False`
    - develop: `iterations=1, termination=function, converged=True`
  - **Random graphs:** Sim(3), 30 poses, 15 held, λ0 1e-16, 20 iterations, 150 seeds.
    - 146 are identical.
    - 2 now end `damping_limit` where develop ended `function`, with the same final errors (0.012631318401573789 vs …824, and 0.010630279924983886 vs …863).
    - 2 run one more iteration.
    - The largest pose difference is 4.7e-7.
  - Probe script: `held_edge_termination_probe.py` in the owner's held_constants scripts.
- **Suggested fix:** pick one.
  - **(a)** Carry the constant inside GTSAM's graph: one factor on any free key, with a zero Jacobian, returning the stacked residuals of the both-held edges. GTSAM then sees exactly develop's errors in every internal rule, Cholesky is unaffected, and `solve()`'s `constant` parameter goes away.
  - **(b)** Keep the current code and qualify the CHANGELOG and docstrings: SLAM It!'s rule sees the whole error, GTSAM's inner λ search only the free part's, so a converged solve may end at `damping_limit`.

##### [should-fix] The CHANGELOG's "the same result bit for bit" for advio-20's global BA is vacuous
- **Where:** `CHANGELOG.md:47-49`
- **Issue:** In both versions that solve accepts no step: 1 iteration, `damping_limit`, final cost 889583.8198, equal to the initial cost (`../results/held_constants_2026-10-23/percall_compare_before_final.md`). "Bit for bit" therefore only says that both versions returned their input. The C++ commit message says so ("its one iteration accepts no step"); the CHANGELOG drops it. A reader would take it as evidence that the two solves are equivalent.
- **Why it matters:** The owner's rule is that commit messages and the CHANGELOG are part of the record and must not mislead.
- **Evidence:** in the table, the `advio20_gba` rows for cpp and rust show "1 damping_limit / 1 damping_limit" and a relative difference of 0.0e+00.
- **Suggested fix:** Add "(neither version accepts a step)", or drop the bit-for-bit claim for this problem.

##### [note] Cost-agreement figures: 2.2e-7 and "6e-11 or better" are both supported, but measure different things
- **Where:** `CHANGELOG.md:58`, and the implementer's report.
- **Evidence:**
  - **2.2e-7** is the largest relative change of any cost in the loop room's monocular golden records. It is in C++: `[243]['mapping']['ba'][0]['initial_cost']`, 1081.7703407 -> 1081.7705737, a local BA after the global BA. Python's largest is 4.9e-8 and Rust's 1.1e-8, matching the commit messages.
  - **The global BA itself** (frame 235) moved 1.55e-10 in Python, 6.2e-11 in C++ and 1.8e-12 in Rust in the records.
  - **Per call** on the dumped global BA: 6.2e-11, 5.3e-13 and 4.1e-13.
  - So "6e-11 relative or better" is right for the per-call solves only. It would be wrong for the records' global BA, where Python's change is 1.55e-10.
- **Suggestion:** State the scope, e.g. "the global BA's own cost the same to 1.6e-10, the later solves' to 2.2e-7".

##### [note] Gauge: the explanation is right; F1 adds no freedom and makes the stop along the scale less sensitive to roundoff
- **Where:** `CHANGELOG.md:55-59`
- **Issue:** I agree with the CHANGELOG on three points:
  - A monocular global BA that holds one camera keeps one gauge direction: scale about that camera (Triggs et al., "Bundle Adjustment — A Modern Synthesis", 2000, §9; Strasdat et al., RSS 2010).
  - The cost is invariant along it.
  - Develop's NonlinearEquality fixed the same 6 DoF. In exact arithmetic both forms take the same LM steps (λ·I damping on the same free variables, δ = 0 on the held one).
  - So F1 introduces nothing new. ORB-SLAM3 also fixes only the map's initial keyframe in global BA (`Optimizer.cc:125`, `vSE3->setFixed(pKF->mnId==pMap->GetInitKFid())`), so its monocular scale is free too and is held back only by g2o's damping.

  The refinement: develop's constrained QR pushed the solve along the gauge by roundoff, amplified as λ falls ×10 per accepted step towards `lambdaLowerBound` 0. Cholesky on the constant form follows LM's path to nearly the same point in every language. "The elimination's roundoff decides where it stops" describes develop well; it overstates HEAD, where all three languages agree to 2.2e-8.
- **Evidence:**
  - **Records:** develop's languages differed in scale by a factor of 1.3516 (Python against C++); HEAD's agree to 2.19e-8 and 2.22e-8 in a final position (recomputed).
  - **Synthetic monocular BA**, 8 cameras, camera 0 held (`gauge_sensitivity_probe.py`):

    | Form | Final scale | Spread under 1e-13 input perturbations |
    |---|---|---|
    | Constant cameras (HEAD) | 2.109001 | 5.6e-9 relative |
    | NonlinearEquality (develop) | 2.114094 | 1.6e-5 relative |

    The costs are equal to 3e-15.
  - The scale is not the only change. Aligned by a similarity, the final positions still move 6.5e-3, 1.7e-3 and 1.5e-3 map units (Python, C++, Rust). Later solves run at another scale, and λ·I damping is not scale-invariant. ATE improved to 1.19 cm in every language (recomputed).
- **Suggestion:**
  - Reword the CHANGELOG along the lines of: "develop's constrained QR let roundoff move the solve along the free scale; with the cameras constants it stops at the same point in every language (2.2e-8); the scale is still not held".
  - For the planned damping PR: g2o-style damping (λ0 = τ·max H_ii, τ = 1e-5, Nielsen updates) reduces the slide but does not remove the freedom. Holding the scale needs a seventh constraint: a scale prior, or holding a distance.
  - Zhang, Gallego & Scaramuzza, "On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation", IEEE RA-L 3(3):2710-2717, 2018 (https://www.research-collection.ethz.ch/handle/20.500.11850/334931), find fixed, prior and free gauges similar in accuracy.
  - Whichever is chosen, test the scale before and after global BA against a stated tolerance.

##### [note] Backlog, not caused by F1: GTSAM's absolute λ bound is tiny next to BA's Hessian, and advio-20's global BA accepts no step
- **Where:** `cpp/src/foundations/graphs.cpp:358-361` (and its Rust and Python twins) set only λ0 and the tolerances. GTSAM's defaults then apply: `lambdaUpperBound` 1e5, `diagonalDamping` false (`build/gtsam-4.3.0/src/gtsam/nonlinear/LevenbergMarquardtParams.h:74,78`).
- **Evidence:**
  - In advio-20's dumped global BA, the camera-translation Hessian diagonal (roughly Σ info·(fx/z)²) is 4.05e8 at minimum, 1.33e9 at the median and 2.51e11 at most.
  - So λ ≤ 1e5 can never move LM towards gradient descent, and the solve gives up after one iteration without accepting a step, in both versions.
  - 17 observations have z < 0.05.
  - g2o would start at λ0 ≈ 1e-5 × 2.5e11 = 2.5e6 and grow without bound on failure.
- **Suggestion:** Take this into the planned damping PR: GTSAM's `diagonalDamping` (Marquardt), or a λ scaled to the Hessian.

##### [note] Python `_held_end` whitens twice unless the noise is unit
- **Where:** `src/slam_it/foundations/graphs.py:637-660`
- **Issue:** It returns the whitened (A, b) from `edge.linearize(...)`, then wraps them in `edge.noiseModel()`, which whitens again. This is correct only because the noise is unit, and the docstring says so. A future weighted essential graph would be silently wrong.
- **Suggestion:** Wrap with `gtsam.noiseModel.Unit.Create(dimension)`, which is right for any Gaussian model, or assert that the noise is unit.

##### [note] The Python held stereo factor copies GTSAM's cheirality convention by hand
- **Where:** `src/slam_it/foundations/graphs.py:1029-1044`
- **Issue:** The `<= 0.0` check returning 2fx equals `GenericStereoFactor3D` only while the environment's GTSAM defines `GTSAM_THROW_CHEIRALITY_EXCEPTION`. It does today: the slam-it env's `include/gtsam/config.h:91`, and the GTSAM option defaults to ON (`build/gtsam-4.3.0/src/cmake/HandleGeneralOptions.cmake:49`). `test_bundle_adjustment_charges_a_point_behind_its_camera` (fixed = [0, 1]) covers only the held side.
- **Suggestion:** A test comparing the held and unheld factor at a point behind the camera would pin this down.

##### [note] The weld's typical stereo case is not in the parity fixture
- **Where:** `tests/fixtures/foundations/generate_graphs.py`, module docstring.
- **Issue:** The stereo BA with half its cameras held (seed 11) was left out because develop's C++ already differed from Python's by 2.4e-8, past the 1e-8 policy. The live test still covers it at 1e-7 / 1e-6.
- **Suggestion:** A case that converges tightly (more iterations, or another seed) would put the weld's stereo form into the parity fixture.

#### Checked and fine
- **C++ `HeldProjectionFactor`:** it is `ProjectionFactor`'s point block (the shared `reproject` and `projection_residual`, `cpp/src/foundations/graphs.cpp:150-155` and `:606-643`). This includes the behind-camera case (`kMinDepth`, 2fx, zero Jacobians), the right column `u - bf/z` with its `bf/z²` term, and the same noise object (Precision, then Huber; `:948-962`).
- **Rust `Factor::HeldProjection`:** the same `projection_residual` (`rust/slam-it-foundations/src/graphs.rs:605-611`) and the same information and Huber arguments to `add_rust_factor`; its key and kind are the point only (`:1589-1611`).
- **Python held factors:** I verified `TriangulationFactorCal3_S2`, the held StereoCamera custom factor and the radtan custom factor against GTSAM's own factors with the pose fixed. In front, behind and at z ≈ 0, plain and Huber, mono and stereo, all 12 cases agree bit for bit in the error, in b and in the point block of A. In GTSAM 4.3.0, `TriangulationFactor::linearize` goes through `evaluateError`, which catches the cheirality exception.
- **One-end-held pose-graph factors:** C++ `HeldBetweenFactor` and Rust `OneEndHeld` call `BetweenFactor::evaluateError` in the edge's own key order, with the free end's Jacobian slot. Python `_held_end` matches the full `BetweenFactor` bit for bit, in SE(3) and Sim(3), with the held end first and second.
- **Both-held edge constant:** computed by GTSAM's own `BetweenFactor::error` from the same converted values, and added to the initial error and to every iteration's error. SLAM It!'s outer stopping rule and the summary errors are as in develop.
- **Every pose held:** returns early with 0 iterations, `function`, converged, and the poses as given. **Every camera held in BA:** only the points are solved, and this is tested.
- **Held cameras and poses come back exactly as given**, with no Pose3 or Similarity3 round trip, in all three languages. Index validation runs before `held_flags` in every language.
- **Local BA is unchanged:** it runs rung 0's `ba::solve` (`cpp/src/edge/adjust.cpp:385,408`, `rust/slam-it-edge/src/adjust.rs:14`, `src/slam_it/edge/adjust.py:272`), whose layout already leaves out fixed poses (`cpp/src/foundations/ba/layout.cpp:170`). The diff touches nothing in foundations or edge outside graphs.
- **Faithful to ORB-SLAM3:**
  - Global BA holds keyframe id 0 (`cpp/src/edge/loops.cpp:2293`, `src/slam_it/edge/loops.py:1610`), as `Optimizer.cc:125` holds the map's initial keyframe.
  - The weld's held keyframes match `Optimizer.cc:3540`.
  - The merge's essential graph matches `Optimizer.cc:1830` and `:1864`, with λ0 1e-16 at `:1800` (`essential_damping`).
  - g2o leaves fixed vertices out of H and b and still counts edges between two fixed vertices in `activeChi2`, exactly F1's constant. g2o's LM has no relative-tolerance stop inside its λ search.
- **The motivation is sound:** `EliminatePreferCholesky` falls back to `EliminateQR` when any factor is constrained (`build/gtsam-4.3.0/src/gtsam/linear/HessianFactor.cpp:646-657`). GTSAM's documentation says the same: https://docs.ros.org/en/api/gtsam/html/group__LinearSolving.html
- **Tests:** the held-heavy fixture cases cover both edge orientations (ring edges (3,4) and (0,4) with the held end first, (7,0) with it second), SE(3) and Sim(3), radtan and pinhole, and every stereo camera held. They match develop's constrained results within 6.7e-16 in a pose and 6.2e-15 in a point, with the same termination codes. The existing behind-camera and Huber tests now exercise the held factors, since they use fixed = [0, 1].
- **Figures that reproduce:**
  - **zju-B6 logs:**
    - Merge time: 657.879 -> 1.412 s (C++) and 668.468 -> 1.640 s (Rust).
    - Peak memory: 2509 -> 300 MiB and 2515 -> 308 MiB.
    - Whole run: 710.6 -> 53.1 s and 709.5 -> 42.3 s.
    - Costs: 1.8e-14 and 5.6e-13; essential graph 1.1e-12 and 3.1e-11.
  - **zju-B6 trajectories:** C++ moved 4.3e-3 (a scale of 1.0055); Rust moved 1.6e-4; C++ and Rust now agree to 9.2e-7, where they differed by 4.1e-3.
  - **zju-B6 ATE, recomputed:** 0.7390 -> 0.7235 cm (C++) and 0.7238 -> 0.7235 cm (Rust).
  - **Loop-room records:**
    - Scales 1.259 / 0.931 / 1.058 (Python / C++ / Rust).
    - RGB-D and stereo poses moved at most 9.3e-10; the other 24 cases did not move.
    - Rust's one changed decision is as described.
    - Develop's languages were up to 35% apart in scale.
  - **Loop-room ATE:** 1.32 / 1.34 / 1.14 -> 1.19 cm (Python / C++ / Rust, recomputed).
  - **Per-call:** the times and costs, and the advio weld-like 6.9e-5 / 1.9e-6, 3.0 and 0.29.
  - **Fixture margins:** 3.8e-11 / 1.9e-10 and 6.2e-15.

---
Probe scripts (outside the repository, for the caller): `held_edge_termination_probe.py` reproduces the `damping_limit` finding (the deterministic case and 150 random essential-graph-like graphs; after a fix every tally entry should be identical); `gauge_sensitivity_probe.py` measures how far roundoff moves a monocular BA's free scale, in both forms.

Sources:
- [GTSAM linear solving docs (EliminatePreferCholesky falls back to QR)](https://docs.ros.org/en/api/gtsam/html/group__LinearSolving.html)
- [ORB-SLAM3 Optimizer.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Optimizer.cc)
- [Zhang, Gallego, Scaramuzza, RA-L 2018 (ETH Research Collection)](https://www.research-collection.ethz.ch/handle/20.500.11850/334931)
- [Same paper, EPFL Infoscience record](https://infoscience.epfl.ch/record/255637)
