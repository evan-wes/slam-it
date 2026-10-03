# Review panel: `feature/two-view`

<!-- Written by /slamlab-review-panel. One file per branch; later runs append a "Run N" section. -->

## Run 1 -- 2026-10-02

| | |
|---|---|
| **Branch** | `feature/two-view` |
| **Range** | `478d399800f23ce4d3d4560b03905d2565de5748...f737c9b36359944370b82965451169bbbb52b1f5` (base: `origin/develop`) |
| **Reviewers** | hygiene (sonnet), architecture (opus), algorithm (opus) |
| **How they ran** | registered slamlab-* agents |
| **Layer 1** | all checks passed (`scripts/check.sh`; 1146 Python tests passed, 337 skipped) |

### Findings and resolutions

| ID | Severity | Reviewer | Where | Finding | Recommended | Resolution |
|---|---|---|---|---|---|---|
| L1 | must-fix | algorithm | `homography.py` / `homography.cpp` pure-rotation branch | `pure_rotation` returns a relative pose whose rotation is not in SO(3) | fix | fixed (6effd5c) -- decided by evan-wes, 2026-10-02 |
| A1 | must-fix | architecture | `epipolar.py`, `epipolar.hpp`, `pyproject.toml` | Shared rung-0 helpers live in `epipolar`, making homography → epipolar → triangulation a false layering | fix | fixed (18eb216) -- decided by evan-wes, 2026-10-02 |
| A2 | must-fix | architecture | `pyproject.toml` dev extra | `opencv-python-headless` is in the lock but not the dev extra; the CHANGELOG says otherwise | fix | fixed (aebc2c0) -- decided by evan-wes, 2026-10-02 |
| L2 | should-fix | algorithm | `twoview.py` `Options.ransac_options` | E and H RANSAC share one Sampson threshold although H's residual has two degrees of freedom | fix | fixed (4622ea6; re-measured on TUM, decisions unchanged) -- decided by evan-wes, 2026-10-02, confirmed after the re-measure 2026-10-03 |
| H1 | should-fix | hygiene | `twoview._candidates` → `homography.decompose_reference` | A singular H reaches the unchecked decomposition in Python; C++ errors | fix | fixed (18eb216, test 67987fa) -- decided by evan-wes, 2026-10-02 |
| H2 | should-fix | hygiene | `triangulation.two_view`, `epipolar.bearing_pairs` | Bearing-pair validation written twice | fix | fixed (18eb216) -- decided by evan-wes, 2026-10-02 |
| H3 | should-fix | hygiene | `Options.__post_init__` (twoview, ransac), `check_options` (C++) | Option errors do not name the bad field or value | fix | fixed (6bc1fda) -- decided by evan-wes, 2026-10-02 |
| H4 | should-fix | hygiene | `homography.decompose_reference` | Long, mixes phases, terse names | fix | fixed (6bc1fda) -- decided by evan-wes, 2026-10-02 |
| H5 | should-fix | hygiene | `scripts/demo_two_view.py` | Duplicates RANSAC settings; unguarded edge cases | fix | fixed (0d915a5) -- decided by evan-wes, 2026-10-02 |
| H6 | should-fix | hygiene | `eval/tum.py` | Trusts file contents; O(F·G) association | fix | fixed (09cab62) -- decided by evan-wes, 2026-10-02 |
| H7 | should-fix | hygiene | `epipolar.py`, `homography.py`, `triangulation.py` helpers | Public helpers take unvalidated input without saying so | fix | fixed (18eb216) -- decided by evan-wes, 2026-10-02 |
| A3 | should-fix | architecture | `pyproject.toml` import-linter | Forbid `cv2` in the library by contract | fix | fixed (6bc1fda) -- decided by evan-wes, 2026-10-02 |
| A4 | should-fix | architecture | `epipolar.py` `_or_nan`/`_solved`, `homography._dlt_reference` | The reference signals "degenerate" with a NaN sentinel the backends do not use | fix | fixed (18eb216) -- decided by evan-wes, 2026-10-02 |
| A5 | should-fix | architecture | `twoview.cpp` binding, `ransac.summary_from_backend` | Two `ransac::Summary` encoders, prefixed flat keys, `in_front` as a list | fix | fixed (67987fa) -- decided by evan-wes, 2026-10-02 |
| A6 | should-fix | architecture | `ransac_crossing.hpp`, `twoview.cpp` binding | 3x3 helpers in the RANSAC header; the exact-key check is duplicated | fix | fixed (67987fa) -- decided by evan-wes, 2026-10-02 |
| A7 | should-fix | architecture | design doc "Conventions", `ba.cpp` binding | "Options cross as one dict" is unrecorded; BA does it differently | fix doc; defer BA migration | fixed (13bd639: the rule in Conventions); BA migration deferred (design doc open items) -- decided by evan-wes, 2026-10-02 |
| H8 | note | hygiene | `twoview.py` docstring | The TUM sweep numbers are not reproducible from the repo | fix | fixed (0d915a5: demo --survey; 13bd639) -- decided by evan-wes, 2026-10-02 |
| H9 | note | hygiene | various | Smaller observations (sample O(count²), E recomputes the plane, assert vs error, n_view loop, rust in docstrings) | no action | waived: the reviewer judged each acceptable; no change -- decided by evan-wes, 2026-10-02 |
| A8 | note | architecture | `backends.dispatch` | Typed for arrays only, so four call sites hand-roll dispatch | defer | deferred (design doc open items, "Deferred from PR 2's review panel") -- decided by evan-wes, 2026-10-02 |
| A9 | note | architecture | `test_rng`, `test_ransac` (both languages) | Parity values duplicated as literals, not fixtures | defer | deferred (design doc open items, to PR 5) -- decided by evan-wes, 2026-10-02 |
| A10 | note | architecture | `twoview` | Rung 0 is the right home; keep policy as options | no action | waived: no action, confirms the placement -- decided by evan-wes, 2026-10-02 |
| A11 | note | architecture | tests, design doc, `eval/__init__.py` | Small layout drifts | fix | fixed (4dc0d5d native tests; 13bd639 datasets table and eval docstring) -- decided by evan-wes, 2026-10-02 |
| L3 | note | algorithm | `twoview.py` GRIC | GRIC over the union of inliers departs from Torr (documented) | no action | waived: documented choice, no change -- decided by evan-wes, 2026-10-02 |
| L4 | note | algorithm | `docs/notes/two-view.md` | TUM ground-truth quality varies from 1 to 4 px by pair | fix | fixed (13bd639; the survey prints each pair's ground-truth consistency, 0d915a5) -- decided by evan-wes, 2026-10-02 |
| L5 | note | algorithm | `ransac` | State of the art: SupeRANSAC, MAGSAC++, PROSAC | defer | deferred (design doc open items) -- decided by evan-wes, 2026-10-02 |
| L6 | note | algorithm | `twoview.py` ambiguity test | Counts every in-front point, not only those with parallax | fix | fixed (13bd639) -- decided by evan-wes, 2026-10-02 |

Resolution values: `fixed (<commit>)`, `deferred (<where tracked>)`, `waived: <reason>`,
`disputed: <reason>`, each followed by `-- decided by <user>, <date>`.

### Hygiene review

## Hygiene review

**Scope:** `git diff 478d399...f737c9b` (slamlab PR 2, two-view geometry). Read in full on HEAD: `foundations/{solve,rng,ransac,triangulation,epipolar,homography,twoview}.py`, `eval/tum.py`, `scripts/demo_two_view.py`, `cpp/src/foundations/twoview.cpp`; surveyed test files by test name; grepped the whole diff for TODO/print/bare except/unwrap/NOLINT. Did not re-run check.sh or tests.
**Summary:** Production-quality work: docs state frames, shapes, units and conventions; tolerances are named and justified (`RANK_TOLERANCE`, `PARALLEL_SINE`, `PURE_ROTATION_TOLERANCE`); validation sits at public boundaries; tests assert on values with parity fixtures and edge cases. No must-fix items. The should-fix items are a duplicated validation block, a Python/C++ divergence on a singular H, error messages that omit the bad value, and a few smaller points.

### Findings

#### [should-fix] Singular H reaches the unchecked decomposition in `twoview`; Python and C++ diverge
- **Where:** `src/slamlab/foundations/twoview.py:_candidates` (calls `homography.decompose_reference`); `src/slamlab/foundations/homography.py:decompose_reference` (`h = homography / svd(...)[1]`); `cpp/src/foundations/twoview.cpp:candidates`
- **Issue:** The public `decompose_homography` checks `singular[1] > RANK_TOLERANCE * singular[0]` and raises `DegenerateError`; the *unchecked* `decompose_reference` that `twoview` calls does not. C++ `twoview::candidates` goes through the checked `decompose_homography` and returns `kDegenerate`.
- **Why it matters:** A numerically rank-deficient H from RANSAC makes Python divide by zero (NaN/inf poses, a numpy RuntimeWarning, then NaN propagating through the status logic), while C++ aborts `initialize` with an error on the same input. Backend divergence, and a silent failure path in Python.
- **Evidence:** `h = homography / np.linalg.svd(homography, compute_uv=False)[1]` with no rank guard in `_decide`/`_candidates`.
- **Suggested fix:** Move the rank check into `decompose_reference` (so every caller shares it), or treat a rank-deficient H as "model unusable" in `_decide` in both languages; add a test. (E decomposition is safe: 8-point output has singular values (1,1,0) by construction.)

#### [should-fix] The same bearing-pair validation is written twice
- **Where:** `src/slamlab/foundations/triangulation.py:two_view` (shape-equality / positive-z / ndim checks after `as_bearings`); `src/slamlab/foundations/epipolar.py:bearing_pairs`
- **Issue:** Identical checks and messages, in two forms (short enough that the duplication tool misses them). `epipolar` already imports `as_bearings` from `triangulation`; the shared helper (`bearing_pairs` with `minimum=0`) belongs beside `as_bearings` in `triangulation` (or `arrays`), and `two_view` should call it.
- **Why it matters:** A change to one copy (e.g. relaxing positive-z) drifts from the other.
- **Suggested fix:** Hoist `bearing_pairs` (shape + minimum + positive-z) to the lower module; call it from `two_view`, `epipolar` and `twoview`.

#### [should-fix] Option-validation errors do not name the bad value
- **Where:** `twoview.py:Options.__post_init__`, `ransac.py:Options.__post_init__`, `cpp/src/foundations/twoview.cpp:check_options`
- **Issue:** Messages like `"focal, threshold_px, sigma_px: must be positive and finite"`, `"confidence: must be in (0, 1)"`, `"lo_steps, lo_inner_iterations, lo_sample_factor: out of range"` do not always say which field failed or what value it had. The project rule is that errors name the bad value; the rng `bounded`/`sample` messages do (`got {seed}`).
- **Suggested fix:** Check each field in its own branch and include the value, e.g. `f"{name}: must be positive and finite, got {value}"`; same in C++; update the tests' `match=` strings.

#### [should-fix] `decompose_reference` (homography) is long and mixes phases
- **Where:** `src/slamlab/foundations/homography.py:decompose_reference` (~55 lines); C++ mirrors it
- **Issue:** Pure-rotation shortcut, the minors table, the three-way `k` branch for `na`/`nb`, the Malis-Vargas scalars `v`, `r`, `n_t`, and the pose assembly all in one function. Single-letter names (`m`, `s`, `e`, `k`, `v`, `r`) are not self-explanatory without the paper.
- **Suggested fix:** Split into `_plane_normals(s) -> (na, nb, k)`, `_scalars(s, m)` and the assembly loop; a comment mapping names to the paper's symbols. Keep the C++ structure parallel.

#### [should-fix] The demo duplicates RANSAC settings and has unguarded edge cases
- **Where:** `scripts/demo_two_view.py:opencv_reference` (`threshold=2.0 / intrinsics[0]`, `prob=0.999`), `matches` (`nfeatures=3000`), `errors_deg`, `main`
- **Issue:** The OpenCV reference hard-codes the same 2 px / 0.999 that are the `twoview.Options` defaults; if those change the comparison silently stops being like-for-like. Read them from the `Options` instance; name the SIFT feature count as a constant. `errors_deg` divides by the translation norm (a pure-rotation result gives NaN and a numpy warning). `--first`+`--gap` past the sequence raises a bare IndexError (a message like "frame index N, sequence has F frames" is clearer). `cv2.findEssentialMat` may return `mask=None`, unhandled.
- **Why it matters:** Demo code, so clarity and robustness only.

#### [should-fix] `eval/tum.py` trusts file contents
- **Where:** `src/slamlab/eval/tum.py:_poses`, `read_sequence`
- **Issue:** A zero-norm quaternion normalises to NaN silently; a short or non-numeric row raises a bare IndexError/ValueError with no file or line context; an empty `groundtruth.txt` crashes in `values[:, 4:8]`; `max_difference` is not validated. The per-frame `np.argmin` over all ground-truth samples is O(F·G); `np.searchsorted` on the sorted timestamps is the natural vectorisation.
- **Suggested fix:** Validate row length, finiteness and non-zero quaternion norm with errors naming the file and line; add a malformed-file test (the current test covers only unknown sensors and no association).

#### [should-fix] Public helpers in `epipolar.py` take unvalidated input
- **Where:** `src/slamlab/foundations/epipolar.py:homogeneous`, `normalised_plane`, `hartley_normalisation`, `null_vector`, `sampson`; `homography.sampson`; `triangulation.triangulate_pairs`, `decompose_reference`
- **Issue:** Public names (no underscore), imported by `twoview` and the estimators, that skip validation. The docstrings say "validated", "unchecked" or "reference" in places, but not consistently. `normalised_plane` divides by z; `sampson` is documented for z=1 but does not enforce it.
- **Suggested fix:** Prefix the internal ones with `_`, or group them under a documented "unchecked, internal" note in the module docstring; at least say "unchecked" in each summary line.

#### [note] Empirical claims in `twoview.py` are not reproducible from the repo
- **Where:** `src/slamlab/foundations/twoview.py` module docstring ("On 38 frame pairs of TUM fr1/xyz … initialised 3 … 22 … 2 off by more than 20 degrees; OpenCV 3 of 22")
- **Issue:** Real evidence for an owner-decided design choice, but there is no script or recorded command that regenerates it (`demo_two_view.py` does one pair).
- **Suggested fix:** Record the sweep command/parameters in `docs/notes/two-view.md`, or add a `--sweep` mode to the demo.

#### [note] Smaller observations
- `Pcg32.sample` uses `pick in chosen` on a list: O(count²). Fine for s=4/8 and the LO inner samples (up to 3s); note it if sample sizes grow.
- `epipolar._essential_8pt` recomputes `normalised_plane` per RANSAC sample although `essential_estimator` already holds `x1`/`x2`; a small per-iteration cost that could be passed through.
- `twoview._initialize` uses `assert chosen is not None` for an invariant; acceptable (the lower score is finite), though C++ returns a Degenerate error for the same "unreachable" case, the better pattern.
- `triangulation._n_view` loops over points in Python; fine for a reference (C++ is the performance path); the docstring could say so.
- Docstrings advertise `backend="rust"` though Rust is out of scope; consistent with the dispatch convention. Confirm `backends.dispatch` errors clearly when `rust` is unavailable.

### Checked and fine
- Module docstrings state purpose, rung, references, conventions (`T_c2_c1`, normalised plane, up-to-sign, unit translation) and the C++ counterpart.
- No debug prints, commented-out code, TODOs, bare `except`, swallowed errors or hard-coded paths in library code. The only `except` is `_fit` catching `DegenerateError` to mean "no model" (intended, documented). NOLINTs each carry a reason. `unwrap` in the C++ bindings is the expected→exception converter.
- Magic numbers are named and justified: GRIC λ and dimension constants, `RANK_TOLERANCE`, `PARALLEL_SINE`, `BEARING_TOLERANCE`, `PURE_ROTATION_TOLERANCE` (cites OpenCV).
- Python and C++ `twoview.initialize` follow the same phases, statuses, fallback rule and error semantics (apart from the H-rank case above).
- `foundations.solve` and `Pcg32` are small and well documented; `Pcg32` has range validation and tests against the reference library's published output; RANSAC returns best estimate plus summary and treats the iteration cap as a result.
- Strong test coverage: parity fixtures; value-based tests of degenerate geometry (planar scenes for 8-point, collinear for DLT, points on the baseline, rank-one E, singular H), bad options and shapes, run-for-run backend agreement, each `twoview` status, the ambiguous-H fallback, and the cores' own input checks.
- `Estimator` is a clean Protocol; `PairEstimator` shares one class across E, F and H without duplicating RANSAC glue.
- Vectorisation: Sampson errors, the 8-point system and triangulation are vectorised; Python loops remain only in the reference decomposition and N-view reference, small and intentional.

### Architecture review

## Architecture review

**Scope:** `git diff 478d399...f737c9b` on `feature/two-view` (13 commits, 44f5e5b..f737c9b) in the repository root. Read every new Python, C++ core, binding and test module in full or in the parts that matter; checked against the design doc, `CLAUDE.md`, `pyproject.toml`, `scripts/setup_slamlab.sh` and `src/slamlab/backends.py`. Layer 1 already passes; not re-run.

**Summary:** The large structure is faithful to the design: the solver contract (A3) landed as decided and as the design doc records it; RANSAC is generic in both cores and bound only through concrete estimators; PCG32 gives sample-for-sample parity; C++ and Python are lock-step, with an `Implementation` Protocol and a parity fixture for every new compiled function; nothing heavy reaches rung 0. The main defect is within rung 0: helpers shared by several modules (validation, Hartley normalisation, the pair estimator, RANSAC dispatch) were placed in `epipolar` in both languages, so `homography` sits above `epipolar` in the contracts and `epipolar` imports `triangulation`, with no conceptual reason for either. PR 3 (5-point) and PR 4 (PnP) would build directly on these, so now is the moment to fix it. The other must-fix is an environment drift: OpenCV is in the lock but missing from the dev extra.

### Findings

#### [must-fix] Helpers shared across rung 0 live in `epipolar`, creating false layering (homography → epipolar → triangulation)
- **Where:** `src/slamlab/foundations/epipolar.py`: `homogeneous` :148, `normalised_plane` :164, `hartley_normalisation` :180, `null_vector` :203, `PairEstimator` :298, `bearing_pairs` :395, `point_pairs` :430, `as_matrix3` :463, `run_ransac` :699; `src/slamlab/foundations/homography.py:49-59` imports all of them; `epipolar.py:67` imports `as_bearings` from `triangulation`; `twoview.py:481` uses `epipolar.bearing_pairs`. C++ the same: `cpp/include/slamlab/foundations/epipolar.hpp:47` (`hartley_normalisation`), :96 (`check_point_pairs`), :228 (`gather`), all used by `cpp/src/foundations/homography.cpp:17-34,71,85,109,188,231`. `pyproject.toml:133-135` encodes it: `"(ba) | (twoview)", "(homography)", "(epipolar)"`.
- **Issue:** Homography estimation does not depend on epipolar geometry; it depends on generic two-view machinery that happens to be defined in `epipolar.py`/`.hpp`. `epipolar` imports `triangulation` only for a bearing check. `run_ransac` is generic "run an Estimator with the reference or a compiled backend" infrastructure (it takes `target=(module_name, function)` strings) living in an algorithm module. The rule "each public algorithm validates its input once, using the shared checks in `slamlab.foundations.arrays`" (design doc :246, `CLAUDE.md`) is broken: `bearing_pairs`, `point_pairs`, `as_matrix3` and `as_bearings` are shared checks in `epipolar` and `triangulation`.
- **Why it matters:** The contract now declares a dependency the maths does not have, and it hardens with every rung-0 module that follows. PR 4's PnP RANSAC needs `run_ransac`, bearing validation and `gather`, and would have to import `epipolar` (absolute pose depending on the essential-matrix module) or copy them; PR 3's 5-point needs the same. The C++ include graph that `check_layers.py` checks inherits the false edge.
- **Evidence:** `homography.py:49`: `from slamlab.foundations.epipolar import (RANK_TOLERANCE, Matrix3, PairEstimator, as_matrix3, hartley_normalisation, homogeneous, null_vector, point_pairs, run_ransac)`. `epipolar.py:67`: `from slamlab.foundations.triangulation import as_bearings`.
- **Suggested fix (before PR 3):** Move the validators (`as_bearings`, `bearing_pairs`, `point_pairs`, `as_matrix3`) to `foundations.arrays` (bearing vectors are a row of the design doc's array-encoding table). Move the two-view building blocks (`homogeneous`, `normalised_plane`, `hartley_normalisation`, `null_vector`, `RANK_TOLERANCE`, `PairEstimator`, `Rows2`/`Rows3`, `gather`) to a small new module, e.g. `foundations.correspondences` (`correspondences.hpp`). Move `run_ransac` into `ransac`, beside `from_backend`. Reorder the contract to `"(ba) | (twoview)", "(epipolar) | (homography)", "(camera) | (nls) | (triangulation)", "(correspondences) | (ransac)"`…, making epipolar and homography independent siblings.

#### [must-fix] `opencv-python-headless` is in the lock but not in the `dev` extra, so an unlocked setup fails layer 1
- **Where:** `scripts/locks/slamlab.lock:19` (added); `pyproject.toml:21-39` (`dev` extra, unchanged); `scripts/demo_two_view.py:31` (`import cv2`); `CHANGELOG.md` ("`opencv-python-headless` joins the dev extra").
- **Issue:** The dev extra is documented as "the single list `scripts/setup_slamlab.sh` installs from when not using the lockfile". `setup_slamlab.sh --unlocked` resolves `dependencies + dev + colmap` from `pyproject.toml`, which lacks OpenCV; mypy's `files` includes `scripts` (`pyproject.toml:94`), so mypy then fails on the unresolved `cv2`. The CHANGELOG states something that did not happen.
- **Why it matters:** The two environment sources have drifted; the next `gen_env_lock.sh` after an unlocked rebuild would silently drop OpenCV from the lock. Configuration must be explicit.
- **Evidence:** `git show f737c9b:pyproject.toml | grep -i opencv` returns nothing; the PR's pyproject diff touches only `allowed-confusables` and the rung-0 contract.
- **Suggested fix:** Add `"opencv-python-headless>=5.0"` to `dev` with a comment like the rerun one ("demos only, never imported by the library").

#### [should-fix] Forbid OpenCV in the library by contract
- **Where:** `pyproject.toml` `[tool.importlinter]`.
- **Issue:** OpenCV is now in the environment, and the next PRs (5-point, PnP) have direct OpenCV equivalents (`findEssentialMat`, `solvePnPRansac`). Nothing stops `cv2` entering `foundations` or `edge`; wrapping existing systems is a stated non-goal, and the edge rungs need a light dependency set.
- **Why it matters:** The demo-only role of OpenCV is a convention today, not a rule.
- **Evidence:** The contracts forbid `pycolmap` outside `eval.colmap_reference` and torch & co on CPU rungs, but say nothing about `cv2`.
- **Suggested fix:** A forbidden contract with `source_modules = ["slamlab"]`, `forbidden_modules = ["cv2"]`, mirroring the pycolmap allow-list; a reference wrapper, if ever needed, goes in `eval` with an explicit `ignore_imports`.

#### [should-fix] The Python reference signals "degenerate" with a NaN sentinel the compiled backends do not use
- **Where:** `src/slamlab/foundations/epipolar.py:489-508` (`_solved`, `_or_nan`, `_essential_reference`, `_fundamental_reference`); same pattern in `homography.py` (`_dlt_reference` :213).
- **Issue:** `_or_nan`'s docstring says NaN is "the compiled backends' way"; it is not (the C++ core returns `Degenerate` and the binding raises `DegenerateError`, `cpp/bindings/foundations/epipolar.cpp`, `unwrap(epipolar::essential_8pt(...))`). The sentinel exists only to fit `backends.dispatch`'s `NDArray` return type; `dispatch` passes exceptions through, so the reference can simply raise.
- **Why it matters:** The error contract is meant to be one shape in every language; here the reference uses an internal second channel and its docs misstate the backends. `_solved` also turns any all-NaN array a backend might return into `Degenerate`, which would mask a real core bug.
- **Evidence:** `def _or_nan(...): """Return the reference's "no solution" as a NaN matrix, the compiled backends' way."""`
- **Suggested fix:** Raise `DegenerateError` inside the `_*_reference` functions; delete `_or_nan` and `_solved`.

#### [should-fix] twoview's result crossing: prefixed flat keys, a second `ransac::Summary` encoder, and a list where an array belongs
- **Where:** `cpp/bindings/foundations/twoview.cpp:71-83` (`put_summary`), :108-114; `cpp/bindings/foundations/ransac_crossing.hpp:88` (`ransac_result`); `src/slamlab/foundations/ransac.py:378` (`summary_from_backend(..., prefix)`); `src/slamlab/foundations/twoview.py:427-451`.
- **Issue:** Two separate C++ encoders of `ransac::Summary` (`ransac_result` and `put_summary`); the twoview one flattens into `essential_iterations`, `homography_found`, …, so the generic `ransac` module gained a string `prefix` parameter to decode it; `in_front` crosses as a Python `list` of ints instead of an `int64` array, against the boundary rule.
- **Why it matters:** Adding a field to `ransac.Summary` means editing two C++ encoders that can drift (only the twoview fixture would catch it); string-prefixed keys leak a binding detail into the generic module; PR 4's PnP RANSAC would add a third encoder.
- **Evidence:** `out[(prefix + "iterations").c_str()] = s.iterations;` in `twoview.cpp` alongside `out["iterations"] = summary.iterations;` in `ransac_crossing.hpp`.
- **Suggested fix:** One `ransac_summary(const Summary&) -> nb::dict` in `ransac_crossing.hpp`, used by `ransac_result`; twoview returns `essential` and `homography` as a nested dict or `None` (still plain scalars); drop `prefix` from `summary_from_backend`; return `in_front` via `new_array<std::int64_t>`.

#### [should-fix] Generic binding helpers placed in the RANSAC header; the exact-key check is duplicated
- **Where:** `cpp/bindings/foundations/ransac_crossing.hpp:38-57` (`Matrix3In`, `matrix3`, `matrix3_array`); `ransac_crossing.hpp:65-74` and `cpp/bindings/foundations/twoview.cpp:35-43` (the same size+contains loop).
- **Issue:** The 3×3 helpers have nothing to do with RANSAC (`epipolar.cpp` uses them for `sampson_*` and `decompose_essential`), so any binding needing a 3×3 matrix must include the RANSAC crossing header. The "dict with exactly these keys" check is written twice.
- **Why it matters:** This PR moved the array helpers from `ba.cpp` into `common.hpp` for exactly this reason; the same misplacement has reappeared one level down.
- **Suggested fix:** Move the 3×3 helpers into `common.hpp`; add one `check_keys(const nb::dict&, std::span<const char* const>, const char* what)` there and call it from both option readers.

#### [should-fix] "Options cross as one dict of `Options` fields" is a new boundary rule the design doc does not record, and BA does it differently
- **Where:** design doc "Conventions" → "Structured problems cross as structs of arrays" (:194-202, "options as scalars, with integer codes for enums"); compare `epipolar.run_ransac`/`twoview.initialize` (`options=asdict(options)`) with BA's ~25 keyword scalars (`cpp/bindings/foundations/ba.cpp:221-236`).
- **Issue:** Two option-crossing conventions now exist (keyword scalars for BA; one dict keyed by dataclass fields with an exact-key check for RANSAC and twoview), and two enum-name mechanisms (BA's Python-side `ENUM_TABLES`; `codes()` from the binding), the second already tracked by A27. The dict is treated as decided but the design doc is silent; `tests/test_backends.py` checks parameter names, so the dict's keys are outside its reach, and drift is caught only at runtime by the exact-key check, only when a C++ parity test runs.
- **Why it matters:** PR 3 and PR 4 will copy whichever pattern they see first; Rust (PR 5) has to implement one of them.
- **Suggested fix:** Record the dict-of-fields rule under "Structured problems…", with the exact-key check as its guard; add an open item "BA migrates to the dict" or "BA keeps kwargs because…" (recommended: migrate BA in a later PR, as a backlog item, so there is one rule); fold the enum-table half into A27.

#### [note] `backends.dispatch` is typed for array results only, so four call sites hand-roll dispatch
- **Where:** `src/slamlab/backends.py` (`dispatch(...) -> NDArray[np.float64]`); hand-rolled copies in `epipolar.run_ransac` :699, `homography.decompose_homography` (end of file), `twoview.initialize` :454, and BA's `solve`.
- **Issue:** Each copy repeats `check_name`, the python branch, `implementation`+`function`, and a decoder.
- **Suggested fix (backlog):** A `dispatch` generic over its return type with an optional `decode` callable would remove the copies without teaching `backends` about algorithms; if `run_ransac` moves into `ransac` (above), it can be the first user.

#### [note] Parity values for the unbound `rng` and `ransac` cores are duplicated literals, not fixtures
- **Where:** `tests/foundations/test_rng.py:21-26` ↔ `cpp/tests/foundations/test_rng.cpp`; `tests/foundations/test_ransac.py:69-104` ↔ `cpp/tests/foundations/test_ransac.cpp`.
- **Issue:** Consistent with "generic modules are tested natively in each core", and the concrete RANSAC fixtures cover parity end to end; but Rust's PR 5 will need the PCG32 draws, making a third hand copy, and the `rng` docstring already says "every backend implements this one small generator" while Rust does not yet.
- **Suggested fix:** When Rust gains PCG32, move the draws to `tests/fixtures/foundations/rng.json` and have the native tests read them (or generate a header from it).

#### [note] `twoview` on rung 0
- `twoview.initialize` on rung 0 is supported by the design doc's ladder ("two-view geometry") and status line; the classic rung's incremental SfM will need it too, so rung 0 is the right shared home. Keep its policy knobs (`ambiguity_ratio`, the E fallback, `min_points`) as options, as now, rather than letting edge-VO-specific behaviour accrete there; milestone 2's tracker owns retry policy, as the open item says.

#### [note] Small layout drifts
- `cpp/tests/foundations/test_twoview_solvers.cpp` tests `epipolar.hpp` and `homography.hpp` together; "tests mirror the packages" suggests `test_epipolar.cpp` and `test_homography.cpp`.
- The design doc's datasets table (:417) still says TUM RGB-D "arrives with milestone 2", but `eval.tum` landed in milestone 1.
- `eval/__init__.py`'s "EuRoC joins it at milestone 2" reads as though EuRoC goes into the `tum` module; when it arrives, a per-dataset module (`eval.euroc`) or an `eval.datasets` package is clearer.

### Checked and fine
- **A3 solver contract:** `foundations.solve` (Protocol + generic `Solution`) and `solve.hpp` (concept + template) mirror each other; `nls.Summary`, `ransac.Summary` and their `Solution` aliases satisfy them; BA wraps `nls.Summary`; the design doc matches the code; the old `SolveSummary` was removed outright, no shim.
- **Generic in the core, concrete in the bindings:** `ransac.estimate` is a template over the `Estimator` concept (a Protocol in Python), not bound; only concrete `*_ransac` and `initialize` cross, so no compiled loop calls back into Python.
- **PCG32:** at the bottom of rung 0, no global or hidden state, seeded per run from `Options` (seed and stream); twoview's E and H use independent streams.
- **Error contract:** cores return `Result` with `InvalidInput`/`Degenerate`, bindings `unwrap` to the shared exceptions; RANSAC with no model is `Degenerate` and the iteration cap is a summary, as documented; twoview's "cannot initialise" is a status and only "neither model" is `Degenerate`, in both languages.
- **Validation:** each entry point validates once before dispatch, and the cores re-check (`test_core_checks_its_own_input` tests).
- **Binding boundary:** float64 arrays, `noconvert`, bool masks, termination and status as integer codes plus `codes()` tables; bindings only convert types.
- **Standalone C++:** no nanobind in `cpp/include` or `cpp/src`; the RANSAC crossing helpers are confined to `cpp/bindings`.
- **Language split:** every new C++ function has a Python reference, an `Implementation` Protocol and a parity fixture (`triangulation`, `epipolar`, `homography`, `twoview` JSON); Rust correctly absent and not gating.
- **Flat modules rather than a `twoview/` package:** a reasonable, documented way to avoid a second C++ facade before A24.
- **Budgets and dependencies:** rung 0 imports only numpy; `eval.tum` only numpy and `so3`; OpenCV and rerun confined to `scripts/`; no PyTorch or GPU path.
- **Layer direction:** `eval.tum` (eval) uses `foundations`; the demo script sits above everything; slamlab never imports reifyscene.
- **Binding helper cleanup:** moving the array copy helpers from `ba.cpp` to `common.hpp` is right.
- **Design doc updates:** the status line, the A3 resolution, the `arccos` closure, and the new open items (Sampson refinement for E, low parallax) are consistent with the code.

### Algorithm review

## Algorithm review

**Scope:** `git diff 478d399...f737c9b` (feature/two-view, slamlab PR 2): `src/slamlab/foundations/{triangulation,rng,ransac,epipolar,homography,twoview}.py`, the C++ mirrors in `cpp/src/foundations/` and `cpp/include/slamlab/foundations/` (homography, twoview, ransac), tests in `tests/foundations/`, fixture tolerance policies, `docs/notes/{triangulation,ransac,two-view}.md`, `scripts/demo_two_view.py`, `src/slamlab/eval/tum.py`. Numerical probes from the scratchpad (`.../scratchpad/p/probe{1,2,3}.py`) on both backends and on TUM fr1/xyz.

**Summary:** Careful work with correct formulations: Lindstrom's niter2, the bearing DLT with camera-centre normalisation, PCG32/Floyd, LO-MSAC, the normalised 8-point E/F, both Sampson distances, E's decomposition, Malis-Vargas and GRIC were checked against the references and OpenCV, with no derivation errors. One real defect: the pure-rotation branch of the homography decomposition returns a scaled H that is not a rotation, and `initialize` passes it out as `relative_pose` on a `pure_rotation` status; the noisy-data tests never reach that branch. The other concern is that E and H RANSAC share one Sampson threshold although their residuals have different degrees of freedom.

### Findings

#### [must-fix] `pure_rotation` returns a relative pose whose rotation is not in SO(3)
- **Where:** `src/slamlab/foundations/homography.py:340-344` (`decompose_reference`, the `PURE_ROTATION_TOLERANCE` branch) and `cpp/src/foundations/homography.cpp` (`decompose_homography`, the `kPureRotationTolerance` branch); the value flows out through `twoview.py:355-356` / `cpp/src/foundations/twoview.cpp:229` as `Initialization.relative_pose`.
- **Issue:** The branch accepts H when `max|HᵀH - I| < 1e-3` and returns `±H` (scaled to a middle singular value of 1) as the rotation, unprojected. The same situation the PR fixed for the four-candidate path with `project_to_so3` ("Formulas assume exact inputs"), missed here (OpenCV's `HomographyDecompInria` has the same omission). An estimated H in this branch is up to ~5e-4 from orthonormal.
- **Why it matters:** The initialiser hands the caller a `T_c2_c1` that the project's own `se3.check_transform` rejects; any consumer (a rotation-only tracker, a keyframe pose) fails validation or silently carries a non-rigid pose. The noise-free fixture does not exercise the branch, and the noisy `test_a_rotation_does_not_initialise` (f = 520, 300 points) ends in `low_parallax`, so neither reaches it. A well-estimated H (more matches, a longer focal length) does.
- **Evidence:** `probe1.py` (noisy pure rotation `se3.exp([0,0,0,0.05,-0.1,0.02])`, 1 px noise):
  ```
  520 1000 0 python low_parallax homography orth dev 2.22e-16 det 1.000000
  2000 1000 0 python pure_rotation homography orth dev 3.08e-04 det 1.000221
    check_transform: rotation: not orthonormal within 1e-06; use project_to_so3 to repair it explicitly
  2000 1000 0 cpp    pure_rotation homography orth dev 3.08e-04 det 1.000221   (same for seeds 1, 2: 4.1e-4, 2.8e-4)
  ```
- **Suggested fix:** Project the branch's rotation with `so3.project_to_so3` (C++ `so3::project_to_so3`) in both backends, after the determinant sign fix. Add a noisy pure-rotation test that reaches `pure_rotation` (e.g. focal 2000, 1000 points as above) and asserts `check_transform` passes on both backends; consider a fixture case of `decompose_homography` on a non-orthonormal near-rotation H.

#### [should-fix] E and H RANSAC share one Sampson threshold, but H's residual has two degrees of freedom
- **Where:** `src/slamlab/foundations/twoview.py:186-192` (`Options.ransac_options`, used for both streams at `:379-380`); the same in `cpp/src/foundations/twoview.cpp`.
- **Issue:** Under isotropic image noise σ, the squared Sampson distance to E (codimension 1 in the 4D joint space) is ~σ²χ²₁ and to H (codimension 2) ~σ²χ²₂. One `threshold_px = 2` keeps 95% of true E inliers but only 86% of true H inliers. Standard practice is a χ² quantile per codimension: Hartley and Zisserman, *MVG* 2nd ed., Table 4.1 (t² = 3.84σ² for F/E, 5.99σ² for H); ORB-SLAM's `Initializer::CheckFundamental`/`CheckHomography` use 3.841 and 5.991 (Mur-Artal et al., T-RO 2015). GRIC already uses the matching per-model caps `λ₃(r - d)`.
- **Why it matters:** H's RANSAC rejects ~13.5% of its true inliers, shrinking its LO refit set, the inliers it triangulates and counts per candidate, and skewing the union support used for GRIC towards E's inliers: the counts the ambiguity test and the owner's E-fallback measurement depend on.
- **Evidence:** `probe2.py`, 20 000 points on a plane, 1 px noise, true E and H:
  ```
  true E, frac e<2px: 0.95365  true H frac e<2px: 0.86445
  mean e^2/sigma^2: E 1.002 H 1.987 (expect 1 and 2)
  ```
- **Suggested fix:** Derive each model's threshold from `sigma_px` and the χ² quantile for its codimension (or scale H's by √(5.99/3.84)), document it in `two-view.md`, and re-run the 38-pair TUM table, since the fallback decision was measured under the current threshold.

#### [note] GRIC over the union of inliers is a documented departure from Torr
- **Where:** `src/slamlab/foundations/twoview.py:384-396`; `docs/notes/two-view.md` ("GRIC over the explained matches only").
- Torr (Phil. Trans. R. Soc. A 356, 1998) scores every match and lets the cap `λ₃(r - d)` absorb outliers. Restricting to the union is a reasonable, measured choice, and the note records why. It does make GRIC depend on each RANSAC's threshold, one more reason to fix the shared threshold above.

#### [note] TUM ground-truth quality varies a lot by pair; "good to about a degree" should not be generalised from frames 0/10
- **Where:** `docs/notes/two-view.md` ("On real data", last paragraph).
- **Evidence:** `probe3.py`, median Sampson distance (px) of matches under the ground-truth E, with the PR's nearest-timestamp association:

  | Pair (first, gap) | Median Sampson | Within 2 px |
  |---|---|---|
  | 0, 10 | 3.62 | 130/624 |
  | 0, 30 | 3.78 | 106/434 |
  | 100, 15 | 1.31 | 437/542 |
  | 300, 30 | 0.97 | 143/191 |

  Slerp interpolation of the ground truth moves these only slightly (3.15 px for pair 0/10) and a ±20 ms offset makes them worse, so timestamp association is not the cause; the early part of the sequence is simply less consistent.
- **Suggestion:** Say the ground truth's epipolar consistency varies from about 1 to 4 px across pairs; optionally report each pair's ground-truth Sampson median beside its errors, so degree-level differences (0.77° vs OpenCV's 1.12°) are read against that floor.

#### [note] State of the art for the robust estimator
- The loop is close to current practice (MSAC scoring, Lebeda-style LO (BMVC 2012), adaptive stopping), matching PoseLib's LO-MSAC (Larsson et al., github.com/PoseLib/PoseLib). The deferred Sampson Levenberg-Marquardt refinement and the 5-point minimal solver (PR 3) close most of the gap. Context for later rungs, not a defect:
  - D. Barath, "SupeRANSAC: One RANSAC to Rule Them All", arXiv:2506.04803, 2025 (https://arxiv.org/abs/2506.04803): a component-by-component analysis of what matters for E/F/H estimation; a good checklist for this module.
  - D. Barath, J. Noskova, M. Ivashechkin, J. Matas, "MAGSAC++, a fast, reliable and accurate robust estimator", CVPR 2020: threshold-free scoring (marginalising over σ), which would sidestep the threshold issue above.
  - O. Chum, J. Matas, "Matching with PROSAC – progressive sample consensus", CVPR 2005: ordered sampling by match quality, cheap to add since the demo already has SIFT ratio-test scores.

#### [note] The ambiguity test counts every in-front point, not only those with parallax
- **Where:** `twoview.py:358-367`: candidates are ranked and compared by `in_front`, while `min_points` uses parallax-qualified points.
- More discriminating than ORB-SLAM's `CheckRT`, which counts negative-depth points with very low parallax as good for every candidate; consistent across backends (stable sort in both). Recording the choice in the note would help a reader comparing against ORB-SLAM.

### Checked and fine
- Lindstrom niter2 (`triangulation.py:174-196`) matches the paper's listing with `(x, x') = (x₂, x₁)`; the stable `λ = c/(b + d)` form is used; the depth from the cross product is correct. The 50-digit oracle at atol 1e-9 is a sound known-answer test.
- N-view DLT on bearings: `[f]ₓ[R|t]` rows, Hartley-style normalisation by the observing cameras' centres, undetermined-point detection via σ₃ ≤ 1e-12 σ₁ (a 2-D null space) are correct; back-mapping `mean + scale·X/w` is right.
- Cheirality (`f·(RX + t) > 0` in both views) and parallax (the angle at the point between the two centres, via atan2) are correct.
- PCG32 seeding matches `pcg32_srandom_r`; the bounded draw (rejection below `2³² mod bound`) and Floyd's sampling (exactly `count` draws) are correct.
- LO-MSAC: truncated-quadratic score; a candidate must be strictly better; the `log1p` bound; the termination flag reported and an iteration cap returning a result; an all-degenerate run raising `Degenerate`; iterative least squares with a threshold shrinking from 3θ to θ and inner non-minimal sampling at `min(3s, |I|/2)`, following Lebeda et al. The C++ template mirrors it, including RNG stream consumption.
- Normalised 8-point: per-image Hartley normalisation; for E on `f/f_z` with the `diag(1,1,0)` projection after de-normalisation; for F, rank 2 before de-normalisation (Hartley 1997); the uniqueness test uses `σ₈`, the second-smallest singular value for both 8 and more rows.
- Sampson for E/F matches H&Z 11.4.3. For H, the 2x4 Jacobian `∂(ε₁, ε₂)/∂(x₁, y₁, x₂, y₂)` checked by hand against `ε = (y₂w − v, u − x₂w)`; the closed-form 2x2 inverse is correct; probe2's mean e²/σ² of 1.99 confirms codimension-2 behaviour.
- The H DLT rows equal the first two rows of `x₂ × Hx₁` (H&Z Alg. 4.2), with unit-norm de-normalisation.
- E decomposition: U, V made proper, `R ∈ {UWVᵀ, UWᵀVᵀ}`, `t = ±u₃` (H&Z Result 9.19); candidates compared as an unordered set in the fixture.
- Malis-Vargas matches OpenCV's `HomographyDecompInria` line by line, including `oppositeOfMinor` indexing, the `signd(0) = +1` convention and `na`/`nb` per `argmax|S_ii|`; the `t*`/`R` formulas, determinant sign fix and clamped square roots are correct; the SO(3) projection on the four-candidate path is correct and tested.
- GRIC terms (r = 4; d = 3/2; k = 5/8; λ₁ = ln r, λ₂ = ln(rn), λ₃ = 2) match Torr 1998.
- The motion choice (`low_parallax` before ambiguity; ≥ 50 points with ≥ 1°) is equivalent to ORB-SLAM's 50th-parallax test.
- The demo undistorts (radtan) before unprojecting; the OpenCV comparison uses the same undistorted matches.
- Fixtures state `comparison`/`rtol`/`atol`/`reason` explicitly with `rtol` given; randomised tests are seeded.
