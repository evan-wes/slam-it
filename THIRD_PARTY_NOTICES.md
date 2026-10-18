# Third-party notices

SLAM It! is licensed under the PolyForm Noncommercial License 1.0.0 ([LICENSE](LICENSE)). The
files below contain code or data from other projects and stay under those projects' licences,
whose texts are in [LICENSES/](LICENSES).

## Code in this repository under another licence

### stb_image (Sean Barrett): MIT

- **`third_party/stb/stb_image.h`** is stb_image v2.30 (<https://github.com/nothings/stb>, commit
  `2c980bb59875b0d32144a71867fbdebb2f77cd20`), unmodified. Copyright (c) 2017 Sean Barrett. It is
  offered under the MIT License or as public domain, at the user's choice; SLAM It! takes the MIT
  License ([LICENSES/MIT-stb.txt](LICENSES/MIT-stb.txt)). Only `cpp/src/eval/png.cpp` compiles
  it, with PNG support alone.

### OpenCV: BSD 3-Clause

- **ORB's learned test pattern.** `PATTERN` in `src/slam_it/foundations/orb.py`, `kPattern` in
  `cpp/include/slam_it/foundations/orb.hpp` and `PATTERN` in
  `rust/slam-it-foundations/src/orb.rs` are the 256 test pairs `bit_pattern_31_` from OpenCV's
  `modules/features2d/src/orb.cpp` (<https://github.com/opencv/opencv>; authors Ethan Rublee,
  Vincent Rabaud, Gary Bradski). The same three files restate that file's split of the feature
  budget over the pyramid's levels (`level_budgets`) and its orientation patch's row widths
  (`UMAX`; `kUmax` in C++). Copyright (c) 2009, Willow Garage, Inc. BSD 3-Clause
  ([LICENSES/BSD-3-Clause-opencv-orb.txt](LICENSES/BSD-3-Clause-opencv-orb.txt)); each copy of
  the pattern carries the notice beside it.
- **The homography decomposition** in `src/slam_it/foundations/homography.py`,
  `cpp/src/foundations/homography.cpp` and `rust/slam-it-foundations/src/homography.rs` follows
  OpenCV's `HomographyDecompInria` (`modules/calib3d/src/homography_decomp.cpp`, Samson Yilma's
  implementation of Malis and Vargas, INRIA research report 6303, 2007): its helper functions,
  its table of plane normals and the order of its four motions. Copyright (C) 2014, Samson
  Yilma; Copyright (C) 2018, Intel Corporation. BSD 3-Clause
  ([LICENSES/BSD-3-Clause-opencv-homography.txt](LICENSES/BSD-3-Clause-opencv-homography.txt)).

### PoseLib (Viktor Larsson): BSD 3-Clause

- **The P3P solvers** in `src/slam_it/foundations/p3p.py`, `cpp/src/foundations/p3p.cpp` and
  `rust/slam-it-foundations/src/p3p.rs` follow PoseLib's (<https://github.com/PoseLib/PoseLib>):
  Ding et al.'s solver (`PoseLib/solvers/p3p.cc`) and Lambda Twist
  (`PoseLib/solvers/p3p_lambdatwist.cc`). Copyright (c) 2020, Viktor Larsson. BSD 3-Clause
  ([LICENSES/BSD-3-Clause-poselib.txt](LICENSES/BSD-3-Clause-poselib.txt)). `p3p.py`'s module
  docstring lists where they depart from PoseLib's.

### Eigen: Mozilla Public License 2.0

- **The Rust 3x3 SVD**, `rust/slam-it-foundations/src/linalg/jacobi_svd3.rs` (`Rotation`,
  `jacobi_2x2`, `jacobi_svd3`, `svd3`, `singular_values3`), is translated from Eigen's two-sided
  Jacobi SVD (<https://gitlab.com/libeigen/eigen>, as in Eigen 5.0.1: `JacobiSVD::compute` in
  `Eigen/src/SVD/JacobiSVD.h`, `JacobiRotation::makeJacobi` in `Eigen/src/Jacobi/Jacobi.h` and
  `real_2x2_jacobi_svd` in `Eigen/src/misc/RealSvd2x2.h`), so that it gives Eigen's factors bit
  for bit. Copyright (C) 2009-2010 Benoit Jacob; Copyright (C) 2009, 2013-2016 Gael Guennebaud.
  Mozilla Public License 2.0 ([LICENSES/MPL-2.0.txt](LICENSES/MPL-2.0.txt)), which applies file
  by file: that file is under it, and carries its notice and Eigen's copyright lines.

## Methods implemented from others' work

These are SLAM It!'s own code, under its own licence, and are credited here:

- **ORB-SLAM, ORB-SLAM2 and ORB-SLAM3** (Mur-Artal, Montiel and Tardós, T-RO 2015; Mur-Artal and
  Tardós, T-RO 2017; Campos et al., T-RO 2021; <https://github.com/UZ-SLAMLab/ORB_SLAM3>): the
  edge rung (`src/slam_it/edge/` and its C++ and Rust cores) implements their tracking, local
  mapping, covisibility graph and spanning tree, keyframe database and relocalisation from the
  papers and a reading of their code, and takes many of its parameters from that code. The front
  end follows their ORB extractor's spreading of keypoints and their matcher's thresholds and
  rotation check, and `slam_it.foundations.stereo` follows `Frame::ComputeStereoMatches`. Where
  the code departs from ORB-SLAM's, its docstrings say so.
  No ORB-SLAM code is included. ORB-SLAM's code is under the GNU General Public License v3.
- **DBoW2** (Gálvez-López and Tardós, T-RO 2012; <https://github.com/dorian3d/DBoW2>):
  `slam_it.foundations.vocabulary` implements its vocabulary tree from the paper and a reading of
  its code: k-means++ seeding as DBoW2 does it, binary centres by bitwise majority, and the L1
  score. No DBoW2 code is included.
- **OpenCV's EPnP** (`modules/calib3d/src/epnp.cpp`, Apache License 2.0): `slam_it.foundations.pnp`
  follows its arrangement of Lepetit et al.'s distance system and its three approximations.
- **Ceres Solver** (<http://ceres-solver.org>, BSD 3-Clause): `slam_it.foundations.nls` takes
  Ceres' defaults and termination tests where it has an equivalent.
- **PCG** (M. O'Neill, 2014; <https://www.pcg-random.org>, Apache License 2.0):
  `slam_it.foundations.rng` and its C++ and Rust twins implement PCG32 and reproduce the
  reference C library's output.

## Data in this repository

- **ARKitScenes** (G. Baruch et al., "ARKitScenes: A Diverse Real-World Dataset for 3D Indoor
  Scene Understanding Using Mobile RGB-D Data", NeurIPS 2021 Datasets and Benchmarks;
  <https://github.com/apple/ARKitScenes>). `src/slam_it/eval/vocabularies/default/` is a
  vocabulary trained on 9,969 frames drawn from 20 captures of ARKitScenes' training fold
  (`training.json`; `src/slam_it/eval/vocabularies/local.json` lists the captures): each word is
  a cluster centre of ORB descriptors computed on those frames, with its weight. No ARKitScenes
  frames are included. ARKitScenes is used under Apple's licence
  ([LICENSES/ARKitScenes-LICENSE.txt](LICENSES/ARKitScenes-LICENSE.txt)).
- **TUM RGB-D** (J. Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS
  2012; <https://cvg.cit.tum.de/data/datasets/rgbd-dataset>), licensed under CC BY 4.0
  (<https://creativecommons.org/licenses/by/4.0/>).
  `tests/fixtures/foundations/twoview_rank_one_h.json` holds 88 SIFT matches between frames 160
  and 190 of `fr1/desk`, as bearings through freiburg1's published calibration. The fixtures
  under `tests/fixtures/eval/sequences/` copy the layout and headers of TUM's `rgb.txt` and
  `depth.txt`, and `tests/fixtures/foundations/generate_camera.py` uses freiburg2's published
  intrinsics and distortion. No TUM images, depth maps or ground truth are included.
- **EuRoC MAV** (M. Burri et al., "The EuRoC micro aerial vehicle datasets", IJRR 2016;
  <https://doi.org/10.3929/ethz-b-000690084>; ETH Zurich: in copyright, non-commercial use
  permitted). `tests/fixtures/eval/sequences/euroc/` copies the layout of `mav0/cam0` and
  `mav0/cam1`, their `sensor.yaml` with the published calibration, and a few timestamps; the
  stereo and camera fixtures use cam0's and cam1's published calibration. No EuRoC images or
  ground truth are included.
- **ZJU-SenseTime VISLAM benchmark** (J. Li et al., 2019; no licence stated).
  `tests/eval/test_zju.py` copies sequence B0's camera and ground-truth `sensor.yaml` files as
  published (sensor descriptions; no trajectory) and three of its frame times.
- **ADVIO** (S. Cortés et al., ECCV 2018; CC BY-NC 4.0). `tests/eval/test_advio.py` uses the four
  distortion coefficients of its `iphone-02.yaml`.
- **Gyroflow lens profiles** (<https://github.com/gyroflow/lens_profiles>, CC0 1.0).
  `tests/fixtures/eval/lens_profiles/` holds eleven profiles from its `Mobile phones/` folder,
  copied unchanged at commit `886fc616c9c2c788856f54c21d451b9cb7fc4f44`; each file's
  `calibrated_by` names its calibrator.
- The notes, the design documents and the changelog hold SLAM It!'s own scores on these
  datasets. No frames, depth maps or ground truth from any dataset are included.

## What you install or download

None of the following is included in this repository. The setup script installs the libraries
and tools, and you download the datasets yourself; each comes with its own terms, which apply to
you when you use it. The Docker images (`docker/`) install some of them when built: an image you
build and pass on carries their licences, among them the GPL of the FFmpeg build inside PyAV's
wheels.

| What | Licence | Used for |
|---|---|---|
| Eigen 5.0.1 | MPL 2.0 | the C++ core |
| numpy; Pillow | BSD 3-Clause and others; MIT-CMU | the Python package |
| PyAV 18.1, whose wheels bundle FFmpeg built with x264 and x265 | BSD 3-Clause; the bundled FFmpeg is GPL | importing videos (the `video` extra) |
| pycolmap 4.2 (COLMAP, Ceres) | BSD 3-Clause | the BA reference (the `colmap` extra) |
| Rust crates (nalgebra, faer, png, pyo3, numpy, pythonize, serde and their dependencies) | MIT, Apache 2.0, or either | the Rust core and its bindings |
| FastAPI, pydantic, uvicorn, boto3 and botocore, httpx | MIT, MIT, BSD 3-Clause, Apache 2.0, BSD 3-Clause | the upload server |
| three.js 0.186.0, loaded by the page from jsDelivr | MIT | the run viewer |
| IBM Plex Sans and Mono, loaded from Google Fonts | SIL Open Font License 1.1 | the viewer's pages |
| Gyroflow's lens profiles, fetched by the setup script and the worker's image | CC0 1.0 | a phone video's lens |
| Test oracles and tools: SciPy, mpmath, PoseLib, OpenCV (`opencv-python-headless`), manifpy, rerun-sdk, segno, pytest, ruff, mypy, import-linter, TypeScript, prettier, jscpd, Google Benchmark, criterion | permissive (BSD, MIT, Apache 2.0) | tests, demos and checks |
| TUM RGB-D | CC BY 4.0 | the test set |
| EuRoC MAV | in copyright, non-commercial use permitted | the test set |
| ZJU-SenseTime | no licence stated | the test set |
| ADVIO | CC BY-NC 4.0 | the test set |
| ARKitScenes | Apple's ARKitScenes licence | the test set; the default vocabulary's training frames |
| Rawseeds Bovisa 2008-09-01 | no licence stated | a vocabulary kept outside the repository |
| Bundle Adjustment in the Large | its authors' terms | BA demos and benchmarks |
