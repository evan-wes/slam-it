# Review panel: `feature/ba-spine`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/ba-spine` |
| **Range** | `0d9366fcf8a282ea61ea8327f03cff8d0ca6b3b4...e3c60fc5167d6567c3029ba021595dc5d0940609` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents (symlinked into reifyscene's `.claude/`) |
| **Layer 1** | all checks passed (`scripts/check.sh --env slamlab`, 16 checks) |

### Findings and resolutions

Findings two reviewers raised about the same issue are marked "same as": they are decided
once.

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| H1 | must-fix | hygiene | `cpp/src/foundations/ba.cpp` (`check_problem`) | `check_anchors` reads `pt_idx[a]` before `pt_idx`'s length is checked (out-of-bounds read for C++ callers) | fix | pending |
| H2 | should-fix | hygiene | `ba/problem.py` | `BAProblem` validation bypassed by the constructor and `dataclasses.replace` (same as A2) | fix (with A2) | pending |
| H3 | should-fix | hygiene | `ba/solve.py`, `bindings/foundations/ba.cpp` | packed `settings`/`codes`/`counts` held by position and enum order (same as A4) | fix (with A4) | pending |
| H4 | should-fix | hygiene | `readers.py`, `colmap_reference.py`, `demo_slice.py` | COLMAP pixel offset and principal-point slices copied (same as A8) | fix (with A8) | pending |
| H5 | should-fix | hygiene | `ba/readers.py` (`read_colmap_text`) | raw `KeyError`/`IndexError` on malformed COLMAP files; image names with spaces truncated | fix | pending |
| H6 | should-fix | hygiene | `eval/colmap_reference.py` | inline `import time` with a `noqa` | fix | pending |
| H7 | should-fix | hygiene | `scripts/demo_slice.py`, `scripts/demo_bal.py` | cwd-relative default path; interrupted download leaves a corrupt file | fix | pending |
| H8 | should-fix | hygiene | `eval/reprojection.py` | no direct tests; empty mask gives NaN; per-camera loop is quadratic | fix | pending |
| H9 | should-fix | hygiene | `ba/problem.py` (`BAOptions.gauge`), `ba.solve` | docstring points to a module docstring that does not describe the gauge; Raises incomplete | fix | pending |
| H10 | should-fix | hygiene | `ba/bundle.py` | per-group masked loops make the Python reference O(K x M) | fix | pending |
| H11 | should-fix | hygiene | `nls.py` (`solve`), `ba/bundle.py` | unnamed damping constants; long `solve`, `Bundle.__init__`, `linearize` | fix | pending |
| H12 | note | hygiene | various | smaller items: triangular solves, `numeric_jacobian` arguments, BAL negative counts and encoding, Umeyama scale via cbrt, `Bundle` method docstrings | fix the BAL counts/encoding and the `Bundle` docstrings; waive the rest | pending |
| A1 | must-fix | architecture | `tests/fixtures/foundations/` | BA has no parity fixture; the live parity test covers one scene, whose pose priors make `auto` pick `none` (gauges, GN, degeneracy, radial unchecked across backends) | fix | pending |
| A2 | must-fix | architecture | `ba/problem.py`, `ba/solve.py` | reference and C++ fail differently on an unvalidated problem (`IndexError` vs `InvalidInputError`) | fix | pending |
| A3 | should-fix | architecture | `nls.py`, `nls.hpp`, design doc | solve summary lives in `nls`, not the doc's `foundations.solve`; doc's open items and Status line stale | fix the doc now; defer the move to PR 2 (RANSAC) | pending |
| A4 | should-fix | architecture | `ba/solve.py`, `bindings/foundations/ba.cpp`, `nls.hpp` | cross-language options held by position; implicit enum values; hard-coded enum sizes | fix | pending |
| A5 | should-fix | architecture | `eval/colmap_reference.py`, `pyproject.toml` | pycolmap loaded via `importlib` (invisible to import-linter); no optional extra | fix | pending |
| A6 | should-fix | architecture | `ba/readers.py`, `eval/__init__.py` | file-format readers in rung 0's solver package; eval docstring stale ("Stub") | fix (move to `slamlab.eval`) | pending |
| A7 | should-fix | architecture | `scripts/demo_slice.py` | reifyscene's slice list and run layout built into slamlab | fix (generic script; slice list documented, not coded) | pending |
| A8 | should-fix | architecture | readers, reference, demo | model constants copied (same as H4) | fix | pending |
| A9 | should-fix | architecture | `ba/problem.py` | general validators belong in `arrays.py`; `_depth_prior` skips `as_real` | fix | pending |
| A10 | should-fix | architecture | `scripts/check_layers.py` | rung-0 internal-order contract not applied to C++ includes | fix | pending |
| A11 | note | architecture | `cpp/bindings/common.hpp` | batching lives in the bindings; standalone C++/Rust callers have no batched API | defer (design doc open question, before the edge rung) | pending |
| A12 | note | architecture | `ba.hpp` | the sparse reduced solver belongs in rung 0 behind `ReducedSystemSolver` | defer (record in the design doc for milestone 4) | pending |
| A13 | note | architecture | `pyproject.toml` | no contract orders the modules inside `ba` | fix (add the contract) | pending |
| A14 | note | architecture | `tests/foundations/test_ba_readers.py` | `eval` tests live under `tests/foundations/` | fix (move to `tests/eval/`) | pending |
| L1 | must-fix | algorithm | `ba/layout.py`, `ba/layout.cpp` | `auto` gauge ignores fixed points (motion-only BA freezes a wrong pose) and treats any pose prior as a full anchor (scale left free; focal falsely unobservable) | fix | pending |
| L2 | must-fix | algorithm | `ba/bundle.py` (`_rays`) | an anchor pixel leaving the invertible region raises out of the Python solve; C++ rejects the step; at the start the two raise different kinds | fix | pending |
| L3 | should-fix | algorithm | `ba/solve.py`, `schur.py` | focal deviations not scaled by the a-posteriori variance factor; the 1% rule rests on arbitrary weights | fix | pending |
| L4 | should-fix | algorithm | `ba/problem.py`, `ba/layout.py` | "first observation" relies on NumPy's unspecified last-write order | fix | pending |
| L5 | should-fix | algorithm | `eval/colmap_reference.py` | reference silently drops pose priors, weights, fixed poses/points, per-parameter masks | fix | pending |
| L6 | note | algorithm | `nls.py` | absolute step tolerance never fires on BA's mixed scales | defer (backlog: per-block relative test) | pending |
| L7 | note | algorithm | `nls.py` | Triggs correction is the standard upgrade over plain IRLS | defer (backlog, milestone 3) | pending |
| L8 | note | algorithm | reduced solve | state of the art for milestone 4 (sparse Cholesky / PCG, root BA, power BA) | fix (cite in the BA note) | pending |
| L9 | note | algorithm | `bundle-adjustment.md` | the slice's focal result is the classic critical-motion degeneracy (Sturm 1997; Kahl, Triggs & Åström 2000) | fix (cite) | pending |
| L10 | note | algorithm | `ba/problem.py` | a fixed anchored point still moves with its host | fix (reject in validation) | pending |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `git diff 0d9366f...e3c60fc` (89 files). I read in full: `foundations/nls.py`, `ba/{problem,bundle,solve,readers}.py`, `alignment.py`, `eval/{colmap_reference,reprojection}.py`, `backends.py`, both demos, `cpp/bindings/foundations/ba.cpp` and `cpp/src/foundations/ba.cpp`. I skimmed the other C++ files, the tests (by test name, plus greps for edge cases), the CHANGELOG and VERSION. I did not read `so3.py`, `se3.py`, `camera.py`, `layout.py`, `schur.py`, the C++ `bundle.cpp`/`schur.hpp`/`nls.hpp`, the fixtures or the docs notes in full.

**Summary:** The code is well documented and readable. Conventions are stated, validation is centralised, and C++ and Python validation mirror each other. I found one real defect: a C++ validation order that can read out of bounds. The other findings are should-fix items: validation bypass via `dataclasses.replace`, a fragile packed-argument protocol, duplicated constants, and thin edge-case tests for the readers and the COLMAP reference.

### Findings

#### [must-fix] `check_problem` reads `pt_idx` before its length is checked
- **Where:** `cpp/src/foundations/ba.cpp:136` (`check_anchors`, which indexes `problem.pt_idx[a]` at line ~84) versus the length check at line ~140 (`problem.cam_idx.size() != n_obs || problem.pt_idx.size() != n_obs ...`).
- **Issue:** `check_anchors` runs first and does `problem.pt_idx[static_cast<std::size_t>(a)]` with `a < observations.rows()`. It does not check that `pt_idx.size() == n_obs`. That check comes afterwards.
- **Why it matters:** `ba::solve` is a public core entry point, and its checks exist so the core can reject bad input without Python's help. A caller that supplies a short `pt_idx` and a valid anchor index gets an out-of-bounds `std::vector` read (undefined behaviour) instead of `InvalidInput`. The nanobind path is shielded only because Python validates first. `test_core_checks_indices_too` covers index range, not this ordering.
- **Evidence:**
  ```
  if (auto checked = check_anchors(problem); !checked) {   // reads pt_idx[a]
  ...
  if (problem.cam_idx.size() != n_obs || problem.pt_idx.size() != n_obs ||   // checked after
  ```
- **Suggested fix:** Move the size checks for `cam_idx`, `pt_idx` and `intrinsics_idx` (and the index range checks) above `check_anchors`. Add a core test with `pt_idx.size() != n_obs` and an anchor.

#### [should-fix] `BAProblem` validation is bypassed by `dataclasses.replace`, and `ba.solve` does not re-validate
- **Where:** `src/slamlab/foundations/ba/problem.py` (`BAProblem` has no `__post_init__`; `anchor_points` uses `replace`). Callers: `scripts/demo_slice.py:focal_free` and `scripts/demo_bal.py` (`replace(problem, ...)`), `eval/colmap_reference.py`, `eval/reprojection.py`.
- **Issue:** The module says "`create` validates it once, so the solvers can trust it". The frozen dataclass can be built or `replace`d into any shape. The Python reference then fails with an obscure numpy error on bad shapes. The C++ backend re-checks, but the two backends then disagree on the error type.
- **Why it matters:** Production boundary, and the documented contract does not hold for a public type. The arrays are also mutable in place (`problem.poses[...] = ...`).
- **Suggested fix:** Either validate in `__post_init__` by moving the checks out of `create`, or call a `check` at the top of `ba.solve`. At minimum, document that `replace` skips validation and offer `BAProblem.with_(...)`.

#### [should-fix] Packed `settings`/`codes`/`counts` arguments depend on unnamed positions and on enum order
- **Where:** `ba/solve.py:_backend_arguments`, `_result_from_backend` (`list(nls.Termination)[int(counts[0])]`, `GAUGES[int(counts[1])]`, `counts[2:]`); `cpp/bindings/foundations/ba.cpp:options_from` (`codes(1)`, `codes(2)`, `settings(2)`, and the magic range limits `3`, `1`, `2`, and `codes(0) > 2`).
- **Issue:** Python and C++ agree only by index convention. The upper bounds `3`, `1` and `2` are hard-coded copies of enum sizes. Reordering `nls.Termination`, `GAUGES` or `camera.MODELS` silently mis-decodes the result. `codes(0) > 2` assumes exactly three camera models.
- **Why it matters:** This is silent corruption on a refactor, where a loud failure would be better. Parity tests only catch it if the order differs in tested paths.
- **Suggested fix:** Name the slots in Python (module constants such as `CODE_MODEL = 0`, or an `IntEnum`). In C++, derive the bounds from the enums' `kCount` sentinel values. Add a test that pins the Python tuples to the C++ enum order, for example by exporting the enum names from the extension.

#### [should-fix] The same constant and slice are defined three or four times
- **Where:** `COLMAP_PIXEL_OFFSET = 0.5` in `ba/readers.py:41` and `PIXEL_OFFSET = 0.5` in `eval/colmap_reference.py:25`. Principal-point slices live in three places: `readers._cameras` (`slice(1,3) if radial else slice(2,4)`), `colmap_reference._CENTRE`, and `scripts/demo_slice.py:_CENTRE`. `camera.py` presumably knows these indices already.
- **Why it matters:** If a model's parameter layout changes, the copies drift. This is the kind of duplication jscpd cannot see.
- **Suggested fix:** Expose `camera.PRINCIPAL_POINT_INDICES[model]` (it sits next to the existing `camera.FOCAL_INDICES`). Define the 0.5 offset once, in `readers`, and import it into `colmap_reference`.

#### [should-fix] `read_colmap_text` raises raw `KeyError`/`IndexError`/`ValueError` on malformed files
- **Where:** `ba/readers.py:read_colmap_text` (`point_index[int(pid)]`, `group_of[int(h[8])]`, `header[1:8]` unpacking, `h[9]`, `reshape(-1, 3)` on a ragged keypoint line). `_cameras` indexes `row[4:...]` without a length check.
- **Issue:** The docstring promises only `InvalidInputError` for unsupported models. A dangling point id, an unknown camera id or a truncated row crashes with an unnamed error. `read_bal` handles truncation properly, so the two readers are inconsistent. The test suite covers only the mixed-model rejection. Image names containing spaces are also silently truncated (`h[9]`).
- **Suggested fix:** Wrap lookups in a helper that raises `InvalidInputError` naming the file and the offending id. Add tests for a dangling point id, an unknown camera id and a short line. Join `header[9:]` for the image name.

#### [should-fix] Inline `import time` with a `noqa` in `colmap_reference.solve`
- **Where:** `eval/colmap_reference.py:~162` (`import time  # noqa: PLC0415 -- only the reference's timing needs it`).
- **Issue:** `time` is stdlib and cheap. The lazy import and lint suppression are unjustified; the comment even reads like an excuse.
- **Suggested fix:** Import `time` at module top and delete the `noqa`.

#### [should-fix] `demo_slice.py` default path depends on the working directory, and `demo_bal.py` can leave a corrupt download
- **Where:** `scripts/demo_slice.py:main` (`default=Path("$REIFYSCENE_DATA_DIR/runs").resolve()`). `scripts/demo_bal.py:fetch` (`urllib.request.urlretrieve(url, path)` straight to the final path).
- **Issue:** `resolve()` is applied to a cwd-relative path, so the default is only right when run from the slamlab root, and nothing says so. An interrupted download leaves a truncated `.bz2` that later runs treat as present.
- **Suggested fix:** Anchor the default to `Path(__file__).resolve().parents[N]`, or require `--runs`. Download to `path.with_suffix(".part")` and `rename` on success. Also `SLICE` hard-codes reifyscene scene names; keep it, since it is a documented demo.

#### [should-fix] `eval/reprojection.py` has no direct tests, and `rms` is unvalidated and loops per camera
- **Where:** `src/slamlab/eval/reprojection.py:rms` and `in_front`. There is no `tests/eval/`; the functions are only exercised through `test_ba_readers.py`.
- **Issue:** An all-False `used` mask gives `np.mean([])`, which returns NaN with a RuntimeWarning rather than an error. `rms` loops `for c in np.unique(cam)` with `cam == c` masks, which is O(cameras x observations). It is also not grouped by intrinsics group the way `Bundle` does it (per-camera BAL groups make this quadratic on large problems). The docstring says used observations "must be in front" but nothing checks it.
- **Suggested fix:** Raise `InvalidInputError` on an empty selection or a wrong-length mask. Add small value tests: a hand-computed RMS and a point behind the camera. Group by `intrinsics_idx` (a stable sort) rather than by camera.

#### [should-fix] Docstring pointer is broken: "see the module docstring" for the gauge
- **Where:** `ba/problem.py`, `BAOptions` attribute `gauge` ("How the gauge freedom is removed (see the module docstring)"). The module docstring of `problem.py` says nothing about gauge.
- **Suggested fix:** Describe the gauge choices where they are implemented (`layout.py`'s module docstring) and point there, or add a short description to `BAOptions`. `ba.solve`'s Raises section also omits `ValueError` from `backends.check_name`, and `InvalidInputError` for an unknown option.

#### [should-fix] Python reference does a per-group masked loop in the hot path
- **Where:** `ba/bundle.py:_rays`, `_anchors`, `_reprojection`, `_reprojection_kind`: `for g, params in enumerate(state.intrinsics): sel = group == g`.
- **Issue:** With one intrinsics group per camera (BAL, and the slice demos), every residual and Jacobian evaluation is O(K x M) in Python. `np.add.at` in `linearize` is also known to be slow. The Python reference is not the performance path, so I have not made this a must-fix. It is still a needless quadratic, and a sort-by-group split would make it linear.
- **Suggested fix:** Precompute the per-group index lists once in `Bundle.__init__`, or add a batched `project` that takes per-row intrinsics.

#### [should-fix] Magic numbers and long functions in the solver loop
- **Where:** `nls.py:solve`. The `growth = 2.0` initial value, its `growth *= 2.0` update, and the Nielsen constants `1.0/3.0` and `(2ρ-1)**3` are unnamed. The function is about 60 lines across several phases: initial evaluation, the loop, the accept/reject branches and the summary. Also `ba/bundle.py:Bundle.__init__` (~65 lines) and `Bundle.linearize` (~55 lines).
- **Suggested fix:** Name the constants (`_NIELSEN_MIN_SCALE`, `_DAMPING_GROWTH`) and say where they come from. Split `solve`'s loop body into `_accept` and `_reject` helpers. Split `Bundle.__init__` into `_index_reprojection`, `_index_depth`, `_index_priors` and `_index_anchors`.

#### [note] Smaller items
- `nls.cholesky_solve` uses two general `np.linalg.solve` calls on triangular factors, which is O(n³) each. The comment acknowledges NumPy has no triangular solver; `scipy.linalg.cho_solve` would be one line if SciPy is already a test dependency. `DenseSystem` is only used by the reference and the tests, so this is low priority.
- `nls.numeric_jacobian` takes `dimension` and `step` without validation (non-positive step, negative dimension). It is a test helper, so it is fine, but the docstring should say that.
- `read_bal` opens text files with the platform default encoding and does not reject negative counts in the header. A negative `n_obs` could make `expected` coincide with the number count. Check `n_cams, n_points, n_obs >= 0`.
- `alignment.align` recovers `scale` from `cbrt(det(matrix[:3,:3]))` of the backend output instead of having the backend return it. This is correct, but a roundabout way of reading a value the algorithm already has.
- Public `Bundle` methods (`world_points`, `reduction`, `kinds`) have one-line docstrings with no shapes or frames. They are semi-internal, but the frame ("poses are world-to-camera in `State`, camera-to-world in `BAProblem`") deserves a sentence, since the convention flips there.

### Checked and fine
- Docstring usefulness in `nls.py`, `problem.py`, `readers.py` and `colmap_reference.py`: shapes, frames, units and conventions are stated, and module docstrings explain the algorithm and where each piece sits.
- Input validation in `BAProblem.create`, `BAOptions`, `nls.Options`, `Loss` and `BlockLoss`: errors name the argument and the bad value.
- The C++ core `check_problem` covers sizes, finiteness, index ranges and rigid transforms (apart from the ordering bug above). The binding also range-checks its enum codes.
- No debug prints in library code (the prints are only in the scripts, where they are the output). No swallowed or bare exceptions. `backends._extension` correctly distinguishes a missing extension from a failed load. No `unwrap()` on fallible library input (`unwrap` in the bindings is the Result-to-exception bridge). The NOLINTs are all narrowly scoped and justified.
- `nls.solve` returns the best estimate and a summary, never a worse state than the start, and raises only in strict mode. This matches the CLAUDE.md error contract.
- Parity and oracle tests in `test_ba.py` assert on values: chi-squared, gradient, comparison with SciPy and Ceres, the gauge, dropped counts, strict mode, and invalid-problem parametrisation.
- CHANGELOG and VERSION bumped (0.2.0.0) with a faithful entry.

### Architecture review

## Architecture review

**Scope:** `git diff 0d9366f...e3c60fc` on `feature/ba-spine` (89 files). Read in full or in part: `src/slamlab/backends.py`, `src/slamlab/foundations/{arrays,nls,camera,so3}.py`, the whole `src/slamlab/foundations/ba/` package, `src/slamlab/eval/*`, `cpp/include/slamlab/foundations/{nls,ba,camera}.hpp`, the C++ include graph, `cpp/bindings/{common.hpp,common.cpp,foundations/ba.cpp}`, `rust/slamlab-py/src/foundations/so3.rs`, `pyproject.toml`, `CMakeLists.txt`, `scripts/check_layers.py`, `scripts/demo_slice.py`, `tests/foundations/test_ba.py`, `tests/test_backends.py`, and the design-doc and CLAUDE.md diffs.

**Summary:** The slice is well structured. Rung 0 holds the Lie groups, cameras, NLS and BA, as the design doc requires. The Python `ba` package and the C++ core mirror each other module for module. The generic NLS solver is a Protocol in Python and a concept in C++, and only `ba.solve` is bound. The error kinds cross the binding correctly, and nothing breaks dependency direction. Two rules are broken:
- BA, the PR's main compiled algorithm, has no parity fixture.
- The Python reference and the C++ core fail differently on the same input, because `BAProblem` can skip validation.

The rest are places where a convention is held by position, or by copies of the same constant, instead of in one checked place. There are also some divergences from the design doc and some stale docs.

### Findings

#### [must-fix] Bundle adjustment has no parity fixture; C++ is checked only by a live comparison on one scene
- **Where:** `tests/fixtures/foundations/` (fixtures exist for so3, se3, alignment and camera only); `tests/foundations/test_ba.py:397-432`
- **Issue:** CLAUDE.md says "Every C++/Rust implementation has a Python reference, a parity fixture in `tests/fixtures/<rung>/` that states its tolerance policy". The design doc says the solve-summary fields "go into the parity fixtures". BA has a C++ implementation but no fixture. Its parity test runs the reference and C++ side by side, in-process, on one scene. That scene has pose priors, so `gauge="auto"` resolves to `"none"`. As a result, these C++ paths are never checked against the reference:
  - the `first_pose` and `similarity` gauges (the Householder basis is documented as "deterministic in every backend");
  - Gauss-Newton;
  - degenerate-point dropping and its counts;
  - the `radial` model inside BA.

  The Ceres comparison in `test_ba_readers.py:171` is a correctness check, not parity.
- **Why it matters:** A live comparison needs the Python reference at test time. Native C++ tests and a future Rust BA cannot check themselves against expected outputs without Python, and that check is what fixtures are for. A divergence in the C++ gauge code would go undetected today.
- **Evidence:** `git ls-tree -r --name-only HEAD tests/fixtures` lists no BA fixture. `_every_kind` (test_ba.py:147) sets `pose_prior_weights`. The module docstring says `"auto"` picks `"none"` "when any pose has a prior".
- **Suggested fix:** Add `tests/fixtures/foundations/ba.json`. Generate it from the reference, with a stated tolerance policy, covering at least: each gauge, LM and GN, a degenerate-drop case, radial/BAL, and the every-kind scene. Assert the summary fields too. If the owner prefers live parity for iterative solvers, change the CLAUDE.md and design-doc rule to say so, and parametrise the live test over those cases instead. I recommend the fixture.

#### [must-fix] The reference and C++ core fail differently on the same unvalidated problem
- **Where:** `src/slamlab/foundations/ba/problem.py:33` (a public frozen dataclass with no `__post_init__`); `:96` (`create` is the only validation); `ba/solve.py:1033`
- **Issue:** `BAProblem(...)` and `dataclasses.replace(problem, ...)` build a problem without validation. The tests do this, `anchor_points` does it, and so do callers. `ba.solve` trusts the result. On the same bad problem, the C++ core returns `InvalidInputError`, as the design doc requires, but the Python reference crashes with a raw numpy `IndexError`.
- **Why it matters:** The design doc says "every language fails the same way". The design rule "validates its input once" can be bypassed through the type's own constructor.
- **Evidence:** I built a valid problem, then ran `replace(p, pt_idx=np.array([0,0,1,99]))`:
  ```
  python: IndexError index 99 is out of bounds for axis 0 with size 2
  cpp: InvalidInputError pt_idx: indices must lie in [0, 2)
  ```
- **Suggested fix:** Move the checks into `BAProblem.__post_init__`, so construction and `replace` both validate. Keep `create` for filling defaults and converting array-likes. Add the Python backend to `test_core_checks_indices_too` (test_ba.py:477).

#### [should-fix] The solve summary is not where the design doc puts it, and the doc's open items are stale
- **Where:** `src/slamlab/foundations/nls.py:269-363` (`Termination`, `Iteration`, `SolveSummary`, `Solution`, `CONVERGED`); `cpp/include/slamlab/foundations/nls.hpp:114-190`; design doc lines 3, 108, 121-126, 459-470
- **Issue:** The design doc puts the shared summary in `slamlab.foundations.solve` and `cpp/include/slamlab/foundations/solve.hpp`, and has C++ return `std::expected<Solution<T>, Error>`. The code puts it in `nls`, and C++ `ba::solve` returns its own `ba::Solution`. Two open items are now settled by the code but still listed as open:
  - "first fallible function → error translation";
  - "first solver defines the summary; Ceres-style converged mapping; RANSAC summary decision".

  The Status line still says "No SLAM algorithms exist yet".
- **Why it matters:** RANSAC is the next rung-0 solver and is meant to return this summary. With the summary in `nls`, RANSAC would have to import the least-squares module, and `Termination` includes NLS-only reasons (`DAMPING_LIMIT`, `SINGULAR`). The Python name `ba/solve.py` will also clash conceptually with a future `foundations.solve`.
- **Suggested fix:** Before RANSAC, either move the generic types to `foundations/solve.py` and `solve.hpp` and keep the NLS-specific reasons in `nls`, or change the doc to say the summary lives in `nls`. I recommend the move. In both cases, close the two open items and update the Status line.

#### [should-fix] The cross-language contract of `ba.solve` is held by position, which the Protocol test cannot see
- **Where:** `src/slamlab/foundations/ba/solve.py:137` (`list(nls.Termination)[int(counts[0])]`), `:171-229` (packed `settings`/`codes`/`tolerances`); `cpp/bindings/foundations/ba.cpp:89,199` (`in_range(gauge, 3)`, `codes(0) > 2`); `nls.hpp:31,39,114` (`Method`, `LossKind` and `Termination` have implicit values)
- **Issue:** The design doc says options cross "as scalars, with integer codes for enums". Here, 18 options travel inside three arrays, identified only by index. `test_backends.py` checks parameter *names*, so it cannot catch a reordering inside these arrays. The `Termination`, `Method` and `LossKind` codes come from declaration order in both languages, while `Gauge` and `camera::Model` have explicit values. The binding also hard-codes how many values each enum has, so adding a camera model or a loss means a silent edit in three places.
- **Why it matters:** Swapping, say, `function_tolerance` and `step_tolerance` on one side passes every structural check. Whether parity notices depends on the scene.
- **Suggested fix:** Pass options as named scalar arguments, as the design doc says, so the Protocol test covers them. Give every crossing enum explicit values in C++. Add one test that checks the Python code tables against the compiled side, for example by binding `nls::name(Termination)` or exposing each enum's size. Derive the binding's range checks from the enums.

#### [should-fix] pycolmap is imported through `importlib`, which import-linter cannot see, and is not declared as an extra
- **Where:** `src/slamlab/eval/colmap_reference.py:58-65`; `pyproject.toml` (pycolmap is listed only under `dev`)
- **Issue:** A library module depends on a package that is only in the dev dependencies, and loads it with `importlib.import_module("pycolmap")`. grimp does see function-level `import` statements. The dynamic form hides the dependency, so no contract can stop a lower rung from copying the pattern.
- **Suggested fix:** Use a plain local `import pycolmap` inside the function, and declare an optional extra (e.g. `colmap = ["pycolmap"]`). Add a forbidden contract so only `slamlab.eval` may import `pycolmap`.

#### [should-fix] BAL/COLMAP readers sit in rung 0's solver package, while eval's docstring claims "benchmark loaders"
- **Where:** `src/slamlab/foundations/ba/readers.py`; `src/slamlab/eval/__init__.py:1-9`
- **Issue:** File-format I/O (bz2, text parsing, dataset conventions) lives inside `foundations.ba`, yet no rung-0 algorithm needs it. Only demos, tests and eval use it. `eval/__init__.py` says loaders belong in eval and still reads "Stub: no code yet." even though this PR adds two eval modules. Milestone 2 brings TUM and EuRoC loaders, and this choice sets their precedent.
- **Suggested fix:** Pick one home for dataset and format readers. I recommend `slamlab.eval`, which matches its docstring; the alternative is a dedicated `slamlab.foundations.io`. Record the choice in the design doc's package-layout table. Update eval's docstring: drop "Stub" and state its milestone and budget.

#### [should-fix] `demo_slice.py` builds a reifyscene yardstick into slamlab
- **Where:** `scripts/demo_slice.py:1-16, 31-46, 98`
- **Issue:** The script hard-codes reifyscene's slice list, its run layout (`runs/slice_*`, `pose_refine/colmap_seed`) and a default path to the reifyscene checkout. The design doc says "reifyscene-specific yardsticks stay in reifyscene". This is not an import, but it is a data-level dependency on reifyscene's internals, and it breaks when reifyscene's layout changes.
- **Suggested fix:** Make the slamlab script generic ("slamlab vs COLMAP BA over a list of COLMAP text models"). Move the slice list and run-layout knowledge to reifyscene, which calls it.

#### [should-fix] Model and convention constants are copied across modules
- **Where:** principal-point indices in `ba/readers.py:166`, `eval/colmap_reference.py:32` and `scripts/demo_slice.py:49`; the COLMAP pixel offset in `ba/readers.py:42` (`COLMAP_PIXEL_OFFSET`) and `eval/colmap_reference.py:29` (`PIXEL_OFFSET`); the intrinsics sizes `5/4/9` in `readers.py:153-157`, which duplicate `camera.MODELS`
- **Issue:** One camera-model fact and one convention conversion (design doc, "Pixel coordinates") each have several copies. Adding a model means editing four files.
- **Suggested fix:** Add `camera.CENTRE_INDICES` beside `FOCAL_INDICES`. Take sizes from `camera.MODELS`. Keep one COLMAP offset constant and import it.

#### [should-fix] General validation helpers live in `ba/problem.py` instead of `arrays.py`
- **Where:** `src/slamlab/foundations/ba/problem.py:340-445` (`_shaped`, `_indices`, `_weights`, `_mask`, `_depth_prior`)
- **Issue:** CLAUDE.md puts shared checks in `slamlab.foundations.arrays`. Index-array, mask and weight checks are general; RANSAC, PnP and the edge rung will need them. `_depth_prior` also skips `as_real` and uses `np.asarray(value, dtype=np.float64)`, so booleans are silently converted, which the `arrays` contract says must be rejected.
- **Suggested fix:** Promote `as_indices`, `as_mask` and `as_weights` to `arrays.py`, and route `_depth_prior` through `as_real` with NaN allowed.

#### [should-fix] `check_layers.py` does not apply the new rung-0 internal-order contract to C++
- **Where:** `pyproject.toml` contract "Rung 0's internal order"; `scripts/check_layers.py:92` (reads only `containers == ["slamlab"]` exhaustive contracts)
- **Issue:** CLAUDE.md says `check_layers.py` holds C++ "to the same contracts as import-linter". The new module-order contract is enforced only in Python. Today's C++ include graph happens to follow it (I checked every `#include "slamlab/..."`).
- **Suggested fix:** Extend `check_layers.py` to check includes between the files of `cpp/include/slamlab/foundations/*.hpp` and `cpp/src/foundations/*` against that contract. Otherwise, mark the contract as Python-only in its comment.

#### [note] Batching lives in the bindings, not the cores
- **Where:** `cpp/bindings/common.hpp` (`map_batch`), used by `se3.cpp` and the other module bindings; `rust/slamlab-py/src/foundations/so3.rs`
- The cores are per-item (fixed-size Eigen), and the bindings loop over the batch. This fits the design doc ("fixed-size arrays, spans or slices") and adds no algorithm logic. The consequence: a standalone C++ or Rust caller (the edge budgets) has no batched API, and per-item error semantics (first failure aborts the batch) are defined in the binding. Worth deciding before the edge rung.

#### [note] The sparse reduced solver should land in rung 0
- `ReducedSystemSolver` is a Protocol in Python and a concept in `ba.hpp`, but C++ `ba::solve` is not parametrised on it. When milestone 4's sparse solver arrives, it belongs in rung 0 behind this interface, not in `classic`. The edge rung's local BA will want it too.

#### [note] No contract orders the modules inside the `ba` package
- Today the order is clean: problem → layout|schur → bundle → solve, with readers above problem. An import-linter layers contract on `slamlab.foundations.ba` would keep it that way as the package grows.

#### [note] Tests for `eval.colmap_reference` live under `tests/foundations/`
- `tests/foundations/test_ba_readers.py` exercises `slamlab.eval.colmap_reference`, and there is no `tests/eval/`. The mirroring rule calls for `tests/eval/`.

### Checked and fine
- Ladder direction: `eval` imports only `foundations`; nothing in `foundations` imports `eval`, `orchestration` or reifyscene. Demos use only public package exports.
- Rung placement: SO(3), SE(3), Umeyama, cameras, NLS and BA all belong in rung 0 per the design doc's ladder table, with no copies in other rungs.
- Edge-budget dependencies: rung 0 adds only Eigen (header-only, a PUBLIC dependency, pinned in setup). SciPy, mpmath, manifpy, pycolmap and rerun are dev or test only. No torch or GPU stack.
- Generic in the core, concrete in the bindings: NLS is a Protocol/concept with native tests (`cpp/tests/foundations/test_nls.cpp`), and only `ba.solve` is bound.
- `slamlab.backends` stays generic: `function` and `dispatch` locate and call implementations without knowing what they compute. The `LOCKSTEP_BACKENDS` change to `test_backends.py` correctly enforces "C++ lock-step, Rust subset".
- Error translation: `cpp/bindings/common.cpp` maps all three kinds to `slamlab.foundations.errors`. Each kind is exercised through C++ (alignment for Degenerate, strict BA for NotConverged, core index checks for InvalidInput).
- C++ structure: `ba` private headers stay in `cpp/src/foundations/ba/`, and the public `ba.hpp` is small. The binding `ba.cpp` only converts types, apart from the enum range checks noted above.
- The import-linter layer contracts and `slamlab_add_rung` linking are unchanged and still correct. Eigen's addition to every rung library is documented in `CMakeLists.txt`.
- No globals, hidden singletons, import-time side effects or environment-dependent behaviour in `src/`.
- The design doc's new "Conventions" section is reflected in the code (T_cam_world inside the solver, T_world_cam at the boundary, xyzw quaternions, OpenCV pixel centres via the COLMAP reader).

Key paths:
- src/slamlab/foundations/ba/problem.py
- src/slamlab/foundations/ba/solve.py
- src/slamlab/foundations/nls.py
- cpp/bindings/foundations/ba.cpp
- src/slamlab/eval/colmap_reference.py
- src/slamlab/foundations/ba/readers.py
- scripts/demo_slice.py
- scripts/check_layers.py
- tests/foundations/test_ba.py
- docs/design/2026-10-02-slamlab-ladder.md

(One side effect: the `conda run` I used for evidence triggered the editable install's automatic C++ rebuild into `build/slamlab/`. That directory is git-ignored and the rebuild was a no-op ("no work to do"). No tracked files were touched.)

### Algorithm review

## Algorithm review

**Scope:** `0d9366f...e3c60fc` (feature/ba-spine): `src/slamlab/foundations/{so3,se3,alignment,camera,nls}.py`, `src/slamlab/foundations/ba/*`, `src/slamlab/eval/{colmap_reference,reprojection}.py`, and the matching C++ in `cpp/src/foundations/` (`ba/bundle.cpp`, `ba/layout.cpp`, `camera.cpp`), read against `docs/notes/{lie-groups,camera-models,nls,bundle-adjustment}.md`. Checked against Barfoot (SO(3)/SE(3) Jacobians, Q), Solà et al. (right perturbations), Umeyama 1991, Triggs et al. 1999 (Schur, gauge, robust IRLS), Ceres' LM strategy (Nielsen update, Marquardt diagonal clamp, loss definitions) and the BAL/COLMAP model definitions. I ran experiments in a scratch directory against the HEAD working tree with both backends.

**Summary:** The maths is sound and well tested. The Lie-group series and closed forms are correct; I checked the coefficients by hand. The Schur elimination, the anchored-inverse-depth chain (including the implicit-function intrinsics Jacobian), the Householder scale-gauge basis, the pose-prior `Jr⁻¹` and the readers' frame flips are all correct. There are two must-fix defects in the BA layer, each shown on a small synthetic scene:
- **The `auto` gauge over-constrains or under-constrains common problems.** On motion-only BA with fixed points it freezes a pose at its wrong starting value. With a single pose prior it leaves scale free, and the focal length is then wrongly reported as unobservable.
- **An anchor pixel that cannot be undistorted crashes the Python solve.** The Python reference raises, while the C++ core rejects the step and converges.

### Findings

#### [must-fix] `auto` gauge ignores fixed points and treats any pose prior as a full anchor
- **Where:** `src/slamlab/foundations/ba/layout.py:59-65` (`_resolve_gauge`), used at `layout.py:139-146`. Mirrored at `cpp/src/foundations/ba/layout.cpp:176-185`.
- **Issue:** `auto` decides using only "any pose prior → none" and "metric depth → first_pose", and otherwise picks `similarity`. Two consequences:
  1. **Fixed points are never considered.** Motion-only BA (all points fixed, poses free, the standard PnP refinement and tracking case) or a problem with fixed control points still gets `similarity`. That freezes the first free pose and confines a second one, even though the fixed structure already defines the gauge.
  2. **Any single pose prior selects `none`.** One SE(3) prior pins 6 DoF but not the monocular scale, and a translation-only prior on one camera pins only 3. So the reduced system stays singular.
- **Why it matters:**
  - Case 1 returns a wrong answer with no error raised: pose 0 stays at its perturbed start and the cost cannot reach zero.
  - Case 2 makes `S` singular at the solution. `intrinsics_std` becomes inf, and `unobservable_focal` flags a focal length that is in fact well observed. The note recommends that milestone 3 decide whether to free the focal length on exactly this flag.
  - The tests do not cover either path. All the pose-prior tests put priors on every camera.
- **Evidence:** Synthetic scene `_scene(cameras=4, points=40, perturb=0.01, seed=1)` from `tests/foundations/test_ba.py`, with `points = truth` and `fixed_points = all True`:
  ```
  auto python similarity gauge_pose 0 scale_pose 3 final cost 3.318e+02 pose err per cam [0.008447 0. 0. 0.011194]
  auto cpp    similarity gauge_pose 0 scale_pose 3 final cost 3.318e+02 pose err per cam [0.008447 0. 0. 0.011194]
  none python none ... final cost 2.345e-20 pose err per cam [0. 0. 0. 0.]
  ```
  Second scene: `_scene(cameras=5, points=60, noise=0.5, seed=3, rotation=0.3)` with focal free. Adding one full pose prior (weights 1e4) on camera 0 only:
  ```
  no prior, auto      gauge=similarity  f_std=[1.63 2.16]  unobservable_focal=0
  one pose prior, auto gauge=none       f_std=[inf inf]    unobservable_focal=1
  ```
  Triggs et al. 1999, §9, covers gauge: the constraints must be minimal (no more and no fewer than the gauge's dimension), or the solution is biased or the covariance singular.
- **Suggested fix:** Have `auto` count what the fixed variables and priors actually remove:
  - **Fixed points:** three or more non-collinear fixed points, or fixed points seen by a fixed pose, leave no gauge, so use `none`.
  - **Scale:** it is pinned by metric depth, by two or more fixed or prior-constrained camera centres at distinct places, or by fixed points with spread. Otherwise add the scale constraint even under pose priors, a "scale-only" gauge using the existing Householder machinery.
  - Alternatively, `auto` could raise `InvalidInput` and ask for an explicit gauge when fixed points are present.

  Add tests for motion-only BA and for a single pose prior. Fix both cores.

#### [must-fix] Python BA crashes (and the backends disagree) when an anchor pixel leaves the distortion's invertible region
- **Where:** `src/slamlab/foundations/ba/bundle.py:200-210` (`_rays` calls `camera.unproject`, which raises for the whole batch), reached from `squared_norms` during `nls.solve`. Compare `cpp/src/foundations/ba/bundle.cpp:409-420` (`anchor_ray` returns `nullopt`, so the cost is inf and the step is rejected).
- **Issue:** With free radtan intrinsics and anchored points, an LM trial step that moves k1 far enough makes an anchor pixel non-invertible.
  - **Mid-solve:** the Python reference lets `NotConvergedError` escape out of `ba.solve`. That breaks the solver contract, under which NotConverged is only for strict mode or divergence. C++ rejects the step and converges.
  - **At the start:** the two backends raise different error kinds (Python `NotConvergedError`, C++ `InvalidInputError` "initial cost not finite"). Neither drops the point, unlike the documented handling of an anchor behind its camera.
- **Why it matters:**
  - The reference and the compiled core diverge on the same input.
  - Free distortion with anchored points is exactly the milestone 3 use (refining VGGT cameras with depth anchors).
  - The parity test does not exercise this path.
- **Evidence:** `_scene(cameras=4, points=30, model="radtan", seed=2, perturb=0.02)`, then `ba.anchor_points`, k1 started at -1.0, f and distortion free, cx/cy fixed:
  ```
  -1.0 python: RAISES NotConvergedError | cpp: gradient cost 4.3e-25 k1 -0.0800
  -1.1 python: RAISES NotConvergedError | cpp: RAISES InvalidInputError
  cpp trace: [(1, 31658.163, False), (2, 31658.163, True), ...]   # C++ rejected the bad first step
  python: ... camera.py:232 _undistort -> NotConvergedError: undistort: pixel outside the invertible region
  ```
- **Suggested fix:**
  - **Mid-solve:** in `_rays`/`world_points`, undistort per anchor and catch the failure. Set that point's world position to NaN, so its blocks become inf cost and the step is rejected, matching C++.
  - **At the start:** drop such anchored points as degenerate in `_kept_observations`, with the same rule in both cores, and count them. That gives one error behaviour in both backends.
  - Add a parity test that starts from a k1 like the one above.

#### [should-fix] Focal standard deviation assumes the weights are true informations; the default weight of 1 px⁻² rarely is
- **Where:** `src/slamlab/foundations/ba/solve.py:70-86`, `ba/schur.py:255-279`, and the decision rule `_unobservable_focal` (`focal_std_ratio = 1%`).
- **Issue:** `σ_f = sqrt(diag(S⁻¹))` is not scaled by the a-posteriori variance factor `σ̂₀² = 2·cost/(m − n)`. With the default weights of 1 (σ = 1 px) and real residuals of about 0.4–0.9 px (the slice's RMS column), σ_f is overstated by roughly 1/σ̂₀. The 1% threshold then flags or clears groups on an arbitrary scale.
- **Why it matters:** The note's recommendation for milestone 3 ("hold focal fixed unless `unobservable_focal` is 0") hinges on this threshold.
- **Evidence:** The docstring itself says "Those deviations assume the weights are true informations". The standard practice is to scale by the a-posteriori variance factor: Triggs et al. 1999, §2.2–2.3; Förstner & Wrobel, *Photogrammetric Computer Vision* (2016), §4.2.
- **Suggested fix:** Report `σ̂₀²` in the summary, and either scale the deviations by it or make the scaling an option with a stated default. Keep the unscaled value available.

#### [should-fix] Two "first observation" computations rely on undefined NumPy behaviour
- **Where:** `src/slamlab/foundations/ba/problem.py:420` (`anchor[problem.pt_idx[weighted]] = weighted  # the last write per point is its first`) and `src/slamlab/foundations/ba/layout.py:49` (`first[pt[order]] = order`).
- **Issue:** Both depend on "last write wins" for repeated indices in a fancy assignment.
- **Why it matters:** NumPy documents this order as unspecified. If it changes, the anchor choice and the parallax reference ray change silently, and parity with C++ (which loops explicitly) breaks.
- **Evidence:** NumPy indexing docs, "Detailed notes": "For advanced assignments, there is in general no guarantee for the iteration order. This means that if an element is set more than once, it is not possible to predict the final result." (https://numpy.org/doc/stable/user/basics.indexing.html)
- **Suggested fix:** Use `np.minimum.at(first, pt, np.arange(...))` (initialised to a large value), or `np.unique(pt, return_index=True)`.

#### [should-fix] The COLMAP reference silently ignores parts of the problem it cannot represent
- **Where:** `src/slamlab/eval/colmap_reference.py:68-73` (`_check`), and `to_reconstruction` / `solve`.
- **Issue:** `_check` rejects anchors and depth priors but not the following, which are dropped without warning:
  - pose priors (`pose_prior_weights > 0`);
  - non-unit or zero observation `weights`;
  - `fixed_poses` and `fixed_points`;
  - per-parameter `fixed_intrinsics` beyond "principal point fixed".
- **Why it matters:** The "identical problem" comparisons in `bundle-adjustment.md` hold only when none of these are set. A future comparison could pass or fail on a different problem from the one solved by slamlab.
- **Suggested fix:** Reject these in `_check` (or map the fixed sets onto `BundleAdjustmentConfig` constant poses and points). State in the docstring what the comparison covers.

#### [note] Step tolerance is absolute, unlike Ceres
- **Where:** `src/slamlab/foundations/nls.py`, `Options.step_tolerance`, with the termination test in `_try_step`.
- Ceres' parameter tolerance is relative, `|Δx| ≤ (|x| + tol)·tol`. The note documents the absolute choice ("a manifold state has no norm"). In BA, points (scale ~10–1000) and focal lengths (~1000) share the step vector, so 1e-10 absolute effectively never fires. Termination then falls to the function test or the cap, which matches the notes' "five scenes never converge within 300 iterations". A per-block relative test (pose twist absolute, points and intrinsics relative) would be closer to Ceres. This is not a defect.

#### [note] IRLS without the ρ'' term is documented; Triggs' correction is the standard upgrade
- **Where:** `nls.py` module docstring; `nls.md`.
- Dropping `2ρ'' Jᵀ r rᵀ J` is a valid, PSD-safe choice, and the note measures its cost (108 iterations on Huber outliers). For context, Ceres' `Corrector` implements the Triggs et al. 1999 §4.3 rescaling. It is the natural next step if robust BA convergence matters in milestone 3.

#### [note] State of the art for the reduced solve (milestone 4 context)
- The dense Cholesky on `S` is appropriate for this rung, and the `ReducedSystemSolver` seam is in the right place. When milestone 4 plugs in a sparse solver, the current options are:
  - sparse Cholesky, or Schur-preconditioned CG ("inexact step LM"): Agarwal, Snavely, Seitz & Szeliski, "Bundle Adjustment in the Large", ECCV 2010;
  - QR null-space marginalisation of landmarks, numerically more stable and float32-capable: Demmel, Sommer, Cremers & Usenko, "Square Root Bundle Adjustment for Large-Scale Reconstruction", CVPR 2021, [arXiv:2103.01843](https://arxiv.org/abs/2103.01843v2);
  - a power-series inverse of the Schur complement: Weber, Demmel, Chan & Cremers, "Power Bundle Adjustment for Large-Scale 3D Reconstruction", CVPR 2023, [arXiv:2204.12834](https://arxiv.org/abs/2204.12834v4).
- Anchored inverse depth follows Civera, Davison & Montiel, "Inverse Depth Parametrization for Monocular SLAM", T-RO 2008, and DSO's host frames (Engel, Koltun & Cremers, TPAMI 2018).

#### [note] The slice's focal-unobservability result matches known theory
- The finding that focal length is unobservable on 9 of 10 DAVIS clips is the classic critical-motion degeneracy. Pure translation and small rotation do not determine focal length: Sturm, "Critical Motion Sequences for Monocular Self-Calibration and Uncalibrated Euclidean Reconstruction", CVPR 1997; Kahl, Triggs & Åström, "Critical Motions for Auto-Calibration When Some Intrinsic Parameters Can Vary", JMIV 2000.
- Citing it in `bundle-adjustment.md` would ground the recommendation to milestone 3.

#### [note] A fixed anchored point still moves with its host
- **Where:** `problem.py`, `_anchors` (it accepts `point_anchor >= 0` together with `fixed_points[p] = True`).
- Such a point's ρ is held, but its world position follows the host pose and intrinsics. `anchor_points` never produces this combination. A caller-supplied one is accepted without a stated meaning. Either reject it in validation or document it.

### Checked and fine
- **SO(3):**
  - Rodrigues and the half-angle form of `(1−cos θ)/θ²`.
  - The `atan2` angle in `log`.
  - The near-π axis from the symmetric part, with the sign taken from the skew part and a deterministic choice at exactly π.
  - The series for `(θ−sin θ)/θ³` (1/6, −1/120, 1/5040, −1/362880) and for `1/θ² − cot(θ/2)/(2θ)` (1/12, 1/720, 1/30240, 1/1209600), both checked by hand.
  - Shepperd's quaternion with w ≥ 0, and the SVD projection with its det fix.
- **SE(3):**
  - Barfoot's Q, with all four terms and their signs, and its block inverse.
  - The exact series `c_k` for the three Q coefficients, including `(k+1)/(2k+5)!`.
  - The adjoint for `(ρ, θ)` order, and `exp`/`log` through `Jl(θ)`.
  - The oracle (series of `adⁿ/(n+1)!`) is genuinely independent of the implementation.
- **Umeyama:** the `det(U)det(V)` reflection guard, `s = tr(DS)/σ_x²`, and collinear and too-few-points degeneracy raised as `DegenerateError`.
- **Camera:**
  - The radtan distortion and its 2×2 Jacobian (every entry checked by hand), and the intrinsics Jacobian.
  - The radial-through-radtan map E.
  - Newton undistortion with the det ≤ 0 guard. A 2000-point stress test on three lenses, including the note's k3 = −6 lens, never returned a root beyond the fold. The only round-trip mismatches were points projected from beyond the fold, which correctly map back to the invertible branch.
- **NLS:**
  - The predicted decrease `½δᵀ(Dδ − g)` is derived correctly from `(H+D)δ = −g`.
  - The Nielsen update and doubling ν match Ceres.
  - The Marquardt diagonal is clamped to [1e-6, 1e32].
  - The Huber and Cauchy ρ and ρ' match Ceres' definitions.
  - The best state is always returned, with a summary and strict mode.
  - Gauss-Newton stopping on `no_decrease` and `singular` is right.
- **BA Jacobians:**
  - `dp/dρ = R`, `dp/dθ = −R hat(X)` for a right perturbation of `T_cw`.
  - For anchored points, `dX/d(host) = [−I, hat(X)]` and `dX/dρ = −Rᵀm/ρ²`.
  - The IFT intrinsics term uses `∂π/∂(x,y)` at Z = 1, and the depth-prior affine columns are correct.
  - The pose prior `log(T̄_cw⁻¹ T_cw)` with `Jr⁻¹(e)`. Its translation part approximates the world-frame centre difference, which suits a GNSS prior.
- **Schur:** damping applied before the H_pp inverse and in reduced coordinates, so the step equals the damped full solve. Back-substitution is correct, and repeated columns are accumulated in both cores.
- **Scale gauge:** `dc = −ρ + c×θ` and `a = (−b, b×c)` verified by the scalar triple product. The Householder complement is orthonormal and deterministic, and it is recomputed per linearisation consistently with `retract`.
- **Cheirality:** observations behind a camera at the start are dropped; mid-solve, they give infinite cost and the step is rejected. `ρ ≤ 0` likewise rejects the step.
- **Readers:**
  - BAL's `F = diag(1,−1,−1)` flip gives pixels `(x, −y)`, checked by hand.
  - COLMAP's pose is world-to-camera with a Hamilton quaternion, pixels and principal point shift by −0.5, and the OPENCV and RADIAL family slot maps are correct.
- **Tests:** synthetic scenes are seeded (`default_rng(seed)`). Schur-vs-dense and numeric Jacobians are tested with radtan, free intrinsics, Huber and anchors. Fixtures carry tolerance policies, and mpmath oracles stand independent of the implementations.

Scratch scripts used for the evidence are in `/tmp/scratch/` (`gauge_fixed_points.py`, `prior_gauge.py`, `anchor_undistort4.py`, `anchor_undistort5.py`, `undistort_floor.py`, `undistort_root.py`). Run them from the repo root with `PYTHONPATH=. conda run -n slamlab python <script>`. No repository files were modified.

Sources:
- [NumPy indexing docs](https://numpy.org/doc/stable/user/basics.indexing.html)
- [Square Root Bundle Adjustment, arXiv:2103.01843](https://arxiv.org/abs/2103.01843v2)
- [Square Root Bundle Adjustment, CVPR 2021](https://openaccess.thecvf.com/content/CVPR2021/html/Demmel_Square_Root_Bundle_Adjustment_for_Large-Scale_Reconstruction_CVPR_2021_paper.html)
- [Power Bundle Adjustment, arXiv:2204.12834](https://arxiv.org/abs/2204.12834v4)
- [Power Bundle Adjustment, CVPR 2023](https://openaccess.thecvf.com/content/CVPR2023/html/Weber_Power_Bundle_Adjustment_for_Large-Scale_3D_Reconstruction_CVPR_2023_paper.html)
