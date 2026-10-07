//! Native tests for building frames: one frame per mode on a textured image, and the
//! builders' refusals. Parity with the Python reference is checked through the whole-run tests
//! (tests/orchestration).

use slamlab_edge::frames::{Camera, DepthMap, monocular, rgbd, stereo_pair};
use slamlab_foundations::camera::Model;
use slamlab_foundations::error::ErrorKind;
use slamlab_foundations::image::Image;
use slamlab_foundations::orb;
use slamlab_foundations::stereo::MatchOptions;

/// The image's width.
const COLS: usize = 320;

/// Its height.
const ROWS: usize = 240;

/// The stereo pair's disparity, in pixels.
const DISPARITY: usize = 5;

/// A textured image: 4-pixel blocks of hashed intensities, shifted left by `shift`.
#[allow(
    clippy::cast_possible_truncation,
    reason = "block indices of a 320x240 image, and the hash's top byte"
)]
fn textured(shift: usize) -> Image {
    let mut pixels = vec![0_u8; ROWS * COLS];
    for r in 0..ROWS {
        for c in 0..COLS {
            let (x, y) = (((c + shift) / 4) as u32, (r / 4) as u32);
            let hash = ((x.wrapping_mul(73_856_093)) ^ (y.wrapping_mul(19_349_663)))
                .wrapping_mul(2_654_435_761);
            pixels[r * COLS + c] = (hash >> 24) as u8;
        }
    }
    Image {
        rows: ROWS,
        cols: COLS,
        pixels,
    }
}

/// A pinhole camera for the image, with a (virtual) baseline.
fn camera(baseline: f64) -> Camera {
    Camera {
        model: Model::Pinhole,
        intrinsics: vec![250.0, 250.0, 160.0, 120.0],
        width: COLS,
        height: ROWS,
        baseline,
    }
}

/// A depth map of 2 m everywhere, of a shape.
fn flat(rows: usize, cols: usize) -> DepthMap {
    DepthMap {
        rows,
        cols,
        values: vec![2.0; rows * cols],
    }
}

/// One frame per mode: monocular without depth; RGB-D at the map's depth with its virtual
/// right column; stereo along rows, most within 5% of the pair's depth.
#[test]
#[allow(clippy::cast_precision_loss, reason = "a disparity of 5")]
fn every_mode_builds_a_frame() {
    let image = textured(0);
    let options = orb::Options::default();
    let mono = monocular(&image, 1.5, &camera(0.0), &options).unwrap();
    assert!(mono.size() > 100);
    assert!(mono.depth[0].is_nan() && mono.right[0].is_nan());
    assert!((mono.bearings[0].norm() - 1.0).abs() < 1e-12);
    let depth = rgbd(&image, &flat(ROWS, COLS), 0.0, &camera(0.1), &options).unwrap();
    assert!(depth.size() > 100);
    for (i, k) in depth.features.keypoints.iter().enumerate() {
        assert!((depth.depth[i] - 2.0).abs() < f64::EPSILON);
        assert!((depth.right[i] - (k.xy[0] - 250.0 * 0.1 / 2.0)).abs() < 1e-12);
    }
    let pair = stereo_pair(
        (&image, &textured(DISPARITY)),
        0.0,
        &camera(0.1),
        &options,
        &MatchOptions::default(),
    )
    .unwrap();
    let truth = 250.0 * 0.1 / DISPARITY as f64;
    let matched: Vec<f64> = pair.depth.iter().copied().filter(|d| !d.is_nan()).collect();
    let close = matched.iter().filter(|d| (*d - truth).abs() < 0.25).count();
    assert!(matched.len() > 100 && 2 * close > matched.len());
}

/// A depth map of another shape, no baseline, or a stereo camera that is not pinhole.
#[test]
fn the_builders_refuse_what_they_cannot_build() {
    let image = textured(0);
    let options = orb::Options::default();
    let stereo = MatchOptions::default();
    let wrong = rgbd(
        &image,
        &flat(ROWS / 2, COLS / 2),
        0.0,
        &camera(0.1),
        &options,
    );
    assert_eq!(wrong.unwrap_err().kind, ErrorKind::InvalidInput);
    let virtual_less = rgbd(&image, &flat(ROWS, COLS), 0.0, &camera(0.0), &options);
    assert_eq!(virtual_less.unwrap_err().kind, ErrorKind::InvalidInput);
    let baseline_less = stereo_pair((&image, &image), 0.0, &camera(0.0), &options, &stereo);
    assert_eq!(baseline_less.unwrap_err().kind, ErrorKind::InvalidInput);
    let mut distorted = camera(0.1);
    distorted.model = Model::RadTan;
    distorted.intrinsics = vec![250.0, 250.0, 160.0, 120.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let not_pinhole = stereo_pair((&image, &image), 0.0, &distorted, &options, &stereo);
    assert_eq!(not_pinhole.unwrap_err().kind, ErrorKind::InvalidInput);
}
