//! EuRoC sequences, as a run reads them: the Rust mirror of `slam_it_all.eval.euroc.read_frames`
//! and `read_camera` (the module's docstring states the format). Ground truth stays Python's:
//! a run never reads it.
//!
//! `sensor.yaml` is read as the Python reader's regular expressions read it, by hand: comments
//! (`#` to the line's end) dropped, a setting as the first token after `key:` at a line's
//! start, a list as the comma-separated numbers between `key: [` and the next `]`, and `T_BS`
//! as the `data:` list of the text from `T_BS:` to its first `]`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use nalgebra::Matrix4;
use slam_it_all_foundations::error::{ErrorKind, Result, fail};
use slam_it_all_foundations::image::MAX_SIDE;
use slam_it_all_foundations::se3;

use crate::text;

/// The data files' timestamps per second.
pub const NANOSECONDS: f64 = 1e9;

/// One camera (`slam_it_all.eval.euroc.Camera`).
#[derive(Debug, Clone, PartialEq)]
#[allow(non_snake_case, reason = "T_body_cam: the frame convention's name")]
pub struct Camera {
    /// `radtan` intrinsics `[fx, fy, cx, cy, k1, k2, p1, p2, 0]`.
    pub intrinsics: Vec<f64>,
    /// The camera's pose in the body frame.
    pub T_body_cam: Matrix4<f64>,
    /// Its images' rows.
    pub rows: usize,
    /// Their columns.
    pub cols: usize,
}

/// A sequence's stereo frames, as a run reads them (`slam_it_all.eval.euroc.Frames`).
#[derive(Debug, Clone, PartialEq)]
pub struct Frames {
    /// The sequence folder (the one holding `mav0`).
    pub root: PathBuf,
    /// Each frame's timestamp, in seconds, ascending.
    pub timestamps: Vec<f64>,
    /// Each frame's left (`cam0`) image.
    pub left: Vec<PathBuf>,
    /// Its right (`cam1`) image.
    pub right: Vec<PathBuf>,
    /// The left camera.
    pub left_camera: Camera,
    /// The right camera.
    pub right_camera: Camera,
}

/// Where `key:` starts a line (after its indentation), if anywhere.
///
/// # Arguments
///
/// * `text` - The text.
/// * `key` - The key.
///
/// # Returns
///
/// The offset just past the colon.
fn after_key(text: &str, key: &str) -> Option<usize> {
    let mut start = 0;
    for line in text.split_inclusive('\n') {
        let indent = line.len() - line.trim_start_matches([' ', '\t']).len();
        let rest = &line[indent..];
        if rest.starts_with(key) && rest[key.len()..].starts_with(':') {
            return Some(start + indent + key.len() + 1);
        }
        start += line.len();
    }
    None
}

/// A `key: value` setting's value: the first token after the colon.
///
/// # Arguments
///
/// * `text` - The text.
/// * `key` - The key.
///
/// # Returns
///
/// The value, if the key is there.
fn setting<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    after_key(text, key).and_then(|at| text[at..].split_whitespace().next())
}

/// A `key: [a, b, ...]` list's numbers.
///
/// # Arguments
///
/// * `text` - The text.
/// * `key` - The key.
///
/// # Returns
///
/// The numbers; `None` if the key, the brackets or a number are missing.
fn numbers(text: &str, key: &str) -> Option<Vec<f64>> {
    let rest = &text[after_key(text, key)?..];
    let open = rest.find(|c: char| !matches!(c, ' ' | '\t' | '\n'))?;
    let close = rest.find(']')?;
    if !rest[open..].starts_with('[') || close < open {
        return None;
    }
    rest[open + 1..close]
        .split(',')
        .map(str::trim)
        .filter(|item| !item.is_empty())
        .map(text::number)
        .collect()
}

/// `T_BS`'s 16 numbers: the `data:` list of the text from `T_BS:` to its first `]`.
///
/// # Arguments
///
/// * `text` - The text.
///
/// # Returns
///
/// The numbers, if there.
fn body_pose(text: &str) -> Option<Vec<f64>> {
    let start = text.find("T_BS:")?;
    let close = start + text[start..].find(']')?;
    numbers(&text[start + 5..=close], "data")
}

/// Reads a camera's `sensor.yaml` (`slam_it_all.eval.euroc.read_camera`).
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The camera.
///
/// # Errors
///
/// `InvalidInput` for a missing file, a missing or malformed field, a camera that is not a
/// radial-tangential pinhole, or a `T_BS` that is not rigid.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::float_cmp,
    reason = "the resolution is checked to be whole numbers from 1 to MAX_SIDE, exactly"
)]
pub fn read_camera(path: &Path) -> Result<Camera> {
    let mut text = String::new();
    for line in text::read_lines(path)? {
        text.push_str(line.split('#').next().unwrap_or_default());
        text.push('\n');
    }
    let invalid = |message: &str| {
        fail(
            ErrorKind::InvalidInput,
            format!("{}: {message}", path.display()),
        )
    };
    if setting(&text, "camera_model") != Some("pinhole") {
        return invalid("expected camera_model: pinhole");
    }
    if setting(&text, "distortion_model") != Some("radial-tangential") {
        return invalid("expected distortion_model: radial-tangential");
    }
    let fields = (
        numbers(&text, "intrinsics"),
        numbers(&text, "distortion_coefficients"),
        numbers(&text, "resolution"),
        body_pose(&text),
    );
    let (Some(pinhole), Some(distortion), Some(resolution), Some(pose)) = fields else {
        return invalid(
            "expected 4 intrinsics, 4 distortion coefficients, a 2-value resolution and a \
             16-value T_BS",
        );
    };
    if pinhole.len() != 4 || distortion.len() != 4 || resolution.len() != 2 || pose.len() != 16 {
        return invalid(
            "expected 4 intrinsics, 4 distortion coefficients, a 2-value resolution and a \
             16-value T_BS",
        );
    }
    if !pinhole
        .iter()
        .chain(&distortion)
        .chain(&resolution)
        .chain(&pose)
        .all(|v| v.is_finite())
    {
        return invalid("expected finite numbers");
    }
    if !resolution
        .iter()
        .all(|&v| v == v.floor() && (1.0..=MAX_SIDE as f64).contains(&v))
    {
        return invalid(&format!(
            "expected a resolution of two integers from 1 to {MAX_SIDE}"
        ));
    }
    let transform = Matrix4::from_row_slice(&pose);
    se3::check_transform(&transform)?;
    let mut intrinsics = pinhole;
    intrinsics.extend(&distortion);
    intrinsics.push(0.0);
    Ok(Camera {
        intrinsics,
        T_body_cam: transform,
        rows: resolution[1] as usize,
        cols: resolution[0] as usize,
    })
}

/// A camera folder's frames, by timestamp in nanoseconds (`_frames`).
///
/// # Arguments
///
/// * `folder` - The camera's folder.
///
/// # Returns
///
/// Each stamp's image.
///
/// # Errors
///
/// `InvalidInput` for an unreadable file, a malformed row or a repeated stamp.
fn camera_frames(folder: &Path) -> Result<BTreeMap<i64, PathBuf>> {
    let path = folder.join("data.csv");
    let mut out = BTreeMap::new();
    for (k, line) in text::read_lines(&path)?.iter().enumerate() {
        if line.is_empty() || line.trim().starts_with('#') {
            continue;
        }
        let mut cells = line.split(',');
        let first = cells.next().unwrap_or_default().trim();
        let stamp = if !first.is_empty() && first.bytes().all(|b| b.is_ascii_digit()) {
            text::integer(first)
        } else {
            None
        };
        let (Some(stamp), Some(name)) = (stamp, cells.next()) else {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "{}expected `timestamp,filename`",
                    text::location(&path, k + 1)
                ),
            );
        };
        if out
            .insert(stamp, folder.join("data").join(name.trim()))
            .is_some()
        {
            return fail(
                ErrorKind::InvalidInput,
                format!("{}timestamp {stamp} repeats", text::location(&path, k + 1)),
            );
        }
    }
    Ok(out)
}

/// Reads a sequence's stereo frames (`slam_it_all.eval.euroc.read_frames`): every frame with
/// both images, in time order.
///
/// # Arguments
///
/// * `root` - The sequence folder.
///
/// # Returns
///
/// The frames.
///
/// # Errors
///
/// `InvalidInput` for a missing or malformed file, or no frame with both images.
#[allow(
    clippy::cast_precision_loss,
    reason = "nanosecond stamps to seconds, as Python divides them"
)]
pub fn read_frames(root: &Path) -> Result<Frames> {
    let mav = root.join("mav0");
    let left_camera = read_camera(&mav.join("cam0").join("sensor.yaml"))?;
    let right_camera = read_camera(&mav.join("cam1").join("sensor.yaml"))?;
    let left = camera_frames(&mav.join("cam0"))?;
    let right = camera_frames(&mav.join("cam1"))?;
    let mut out = Frames {
        root: root.to_path_buf(),
        timestamps: Vec::new(),
        left: Vec::new(),
        right: Vec::new(),
        left_camera,
        right_camera,
    };
    for (stamp, image) in &left {
        if let Some(other) = right.get(stamp) {
            out.timestamps.push(*stamp as f64 / NANOSECONDS);
            out.left.push(image.clone());
            out.right.push(other.clone());
        }
    }
    if out.timestamps.is_empty() {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{}: no frame has both a left and a right image",
                root.display()
            ),
        );
    }
    Ok(out)
}

/// The rigid transform from left-camera to right-camera coordinates
/// (`slam_it_all.eval.euroc.right_from_left`).
///
/// # Arguments
///
/// * `left` - The left camera.
/// * `right` - The right camera.
///
/// # Returns
///
/// `T_right_left`.
///
/// # Errors
///
/// The inversion's error.
pub fn right_from_left(left: &Camera, right: &Camera) -> Result<Matrix4<f64>> {
    Ok(se3::inverse(&right.T_body_cam)? * left.T_body_cam)
}

#[cfg(test)]
mod tests {
    //! Native tests against the shared fixture `tests/fixtures/eval/sequences/euroc/` and the
    //! Python reader's frames, `sequences.json`, which the C++ tests read too.

    use super::*;
    use serde_json::Value;

    /// The fixtures' folder.
    fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/eval")
    }

    /// Checks a camera against its JSON.
    fn check_camera(got: &Camera, expected: &Value) {
        let intrinsics: Vec<f64> = expected["intrinsics"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        assert_eq!(got.intrinsics, intrinsics);
        for r in 0..4 {
            for c in 0..4 {
                assert_eq!(
                    got.T_body_cam[(r, c)].to_bits(),
                    expected["T_body_cam"][r][c].as_f64().unwrap().to_bits()
                );
            }
        }
        assert_eq!(
            u64::try_from(got.rows).unwrap(),
            expected["rows"].as_u64().unwrap()
        );
        assert_eq!(
            u64::try_from(got.cols).unwrap(),
            expected["cols"].as_u64().unwrap()
        );
    }

    #[test]
    fn the_folder_reads_as_pythons() {
        let text = std::fs::read_to_string(fixtures().join("sequences.json")).unwrap();
        let cases: Value = serde_json::from_str(&text).unwrap();
        let expected = &cases["cases"]["euroc"]["euroc"];
        let frames = read_frames(&fixtures().join("sequences").join("euroc")).unwrap();
        let times: Vec<f64> = expected["timestamps"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        assert_eq!(frames.timestamps, times);
        for (side, paths) in [("left", &frames.left), ("right", &frames.right)] {
            let got: Vec<String> = paths
                .iter()
                .map(|p| {
                    p.strip_prefix(&frames.root)
                        .unwrap()
                        .to_string_lossy()
                        .replace('\\', "/")
                })
                .collect();
            let wanted: Vec<String> = expected[side]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_str().unwrap().to_owned())
                .collect();
            assert_eq!(got, wanted);
        }
        check_camera(&frames.left_camera, &expected["left_camera"]);
        check_camera(&frames.right_camera, &expected["right_camera"]);
        assert!(right_from_left(&frames.left_camera, &frames.right_camera).is_ok());
    }

    #[test]
    fn a_folder_without_cameras_is_refused() {
        let err =
            read_frames(&std::env::temp_dir().join("slam_it_all_no_such_sequence")).unwrap_err();
        assert_eq!(err.kind, ErrorKind::InvalidInput);
    }
}
