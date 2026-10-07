//! TUM-format sequence folders, as a run reads them: the Rust mirror of
//! `slamlab.eval.tum.read_frames` and `read_camera` (the module's docstring states the format,
//! `camera.txt` included). Ground truth stays Python's: a run never reads it.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use slamlab_foundations::camera::{self, Model};
use slamlab_foundations::error::{Error, ErrorKind, Result, fail};
use slamlab_foundations::image::MAX_SIDE;

use crate::png;
use crate::text;

/// A depth map's value per metre of z, in the benchmark.
pub const DEPTH_SCALE: f64 = 5000.0;

/// The largest frame-to-depth-map time difference accepted, in seconds.
pub const MAX_DIFFERENCE: f64 = 0.02;

/// The benchmark's images' width and height.
pub const SIZE: (usize, usize) = (640, 480);

/// The file a TUM-format folder describes its own camera in.
pub const CAMERA_FILE: &str = "camera.txt";

/// `camera.txt`'s keys.
const CAMERA_KEYS: [&str; 4] = ["model", "intrinsics", "size", "depth_scale"];

/// The keys it may hold too.
const OPTIONAL_CAMERA_KEYS: [&str; 1] = ["bf"];

/// Every freiburg sensor's virtual stereo baseline times focal length, in pixel-metres
/// (`slamlab.eval.tum.SENSOR_BF`: ORB-SLAM2's TUM RGB-D settings' `Camera.bf`).
pub const SENSOR_BF: f64 = 40.0;

/// The benchmark's sensors' published `radtan` intrinsics, by the name a folder holds.
const SENSORS: [(&str, [f64; 9]); 3] = [
    (
        "freiburg1",
        [
            517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633,
        ],
    ),
    (
        "freiburg2",
        [
            520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172,
        ],
    ),
    (
        "freiburg3",
        [535.4, 539.2, 320.1, 247.6, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
];

/// A folder's colour camera (`slamlab.eval.tum.Camera`).
#[derive(Debug, Clone, PartialEq)]
pub struct Camera {
    /// The camera model.
    pub model: Model,
    /// Its intrinsics.
    pub intrinsics: Vec<f64>,
    /// The images' width, in pixels.
    pub width: usize,
    /// Their height.
    pub height: usize,
    /// A depth map's value per metre of z.
    pub depth_scale: f64,
    /// The depth sensor's virtual stereo baseline times the focal length, in pixel-metres
    /// (ORB-SLAM's `Camera.bf`); `None` if `camera.txt` gives none (no RGB-D run).
    pub bf: Option<f64>,
}

/// A folder's colour frames, as a run reads them (`slamlab.eval.tum.Frames`).
#[derive(Debug, Clone, PartialEq)]
pub struct Frames {
    /// The folder.
    pub root: PathBuf,
    /// Each frame's timestamp, in seconds, in `rgb.txt`'s order (non-decreasing).
    pub timestamps: Vec<f64>,
    /// Each frame's colour image.
    pub images: Vec<PathBuf>,
    /// Each frame's nearest depth map within the tolerance, if any.
    pub depths: Vec<Option<PathBuf>>,
    /// The camera.
    pub camera: Camera,
}

/// A depth map in metres (`slamlab.eval.tum.read_depth`).
#[derive(Debug, Clone, PartialEq)]
pub struct Depth {
    /// Its rows.
    pub rows: usize,
    /// Its columns.
    pub cols: usize,
    /// Row-major z in metres; NaN where nothing was measured.
    pub values: Vec<f64>,
}

/// A `timestamp filename` file's rows.
struct Stamped {
    /// The stamps, non-decreasing.
    stamps: Vec<f64>,
    /// The file names.
    names: Vec<String>,
}

/// Reads a `timestamp filename` file.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// Its rows.
///
/// # Errors
///
/// `InvalidInput` for an unreadable file, a short row, a non-numeric or non-finite stamp, or
/// stamps that descend.
fn stamped(path: &Path) -> Result<Stamped> {
    let mut out = Stamped {
        stamps: Vec::new(),
        names: Vec::new(),
    };
    for (k, line) in text::read_lines(path)?.iter().enumerate() {
        if !text::is_data(line) {
            continue;
        }
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 2 {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "{}expected 2 fields, got {}",
                    text::location(path, k + 1),
                    parts.len()
                ),
            );
        }
        let Some(stamp) = text::number(parts[0]) else {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "{}expected a numeric timestamp, got '{}'",
                    text::location(path, k + 1),
                    parts[0]
                ),
            );
        };
        out.stamps.push(stamp);
        out.names.push(parts[1].to_owned());
    }
    if out.stamps.iter().any(|t| !t.is_finite()) || out.stamps.windows(2).any(|w| w[1] < w[0]) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: must ascend (sort them first)", path.display()),
        );
    }
    Ok(out)
}

/// The index of the nearest of ascending times to a stamp (ties: the earlier),
/// `slamlab.eval.timestamps.nearest`.
///
/// # Arguments
///
/// * `times` - The times, ascending, not empty.
/// * `stamp` - The stamp.
///
/// # Returns
///
/// The index.
fn nearest(times: &[f64], stamp: f64) -> usize {
    if times.len() == 1 {
        return 0;
    }
    let after = times
        .partition_point(|&t| t < stamp)
        .clamp(1, times.len() - 1);
    let before = after - 1;
    if (times[before] - stamp).abs() <= (times[after] - stamp).abs() {
        before
    } else {
        after
    }
}

/// Reads a `camera.txt`'s keys, each once.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// Each key's values.
///
/// # Errors
///
/// `InvalidInput` for an unreadable file, or an unknown, repeated or missing key.
fn camera_values(path: &Path) -> Result<BTreeMap<String, Vec<String>>> {
    let mut out = BTreeMap::new();
    for (k, line) in text::read_lines(path)?.iter().enumerate() {
        let data = line.split('#').next().unwrap_or_default();
        let mut parts = data.split_whitespace();
        let Some(key) = parts.next() else {
            continue;
        };
        let known = CAMERA_KEYS.contains(&key) || OPTIONAL_CAMERA_KEYS.contains(&key);
        if !known || out.contains_key(key) {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "{}unknown or repeated key '{key}'",
                    text::location(path, k + 1)
                ),
            );
        }
        out.insert(key.to_owned(), parts.map(str::to_owned).collect());
    }
    if let Some(missing) = CAMERA_KEYS.iter().find(|key| !out.contains_key(**key)) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: missing {missing}", path.display()),
        );
    }
    Ok(out)
}

/// The error for a `camera.txt` value that is not a number, naming it.
///
/// # Arguments
///
/// * `path` - The file.
/// * `key` - Its key.
/// * `value` - The value.
///
/// # Returns
///
/// `InvalidInput`.
fn not_a_number(path: &Path, key: &str, value: &str) -> Error {
    Error {
        kind: ErrorKind::InvalidInput,
        message: format!("{}: expected numbers ({key}: '{value}')", path.display()),
    }
}

/// Reads a `camera.txt` value as a number.
///
/// # Arguments
///
/// * `path` - The file.
/// * `key` - Its key.
/// * `value` - The value.
///
/// # Returns
///
/// The number.
///
/// # Errors
///
/// `InvalidInput` naming the value if it is not one.
fn number_of(path: &Path, key: &str, value: &str) -> Result<f64> {
    text::number(value).ok_or_else(|| not_a_number(path, key, value))
}

/// Reads a `camera.txt` (`slamlab.eval.tum._camera_file`).
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
/// `InvalidInput` for a malformed one.
fn camera_file(path: &Path) -> Result<Camera> {
    let values = camera_values(path)?;
    let (model, size, scale) = (&values["model"], &values["size"], &values["depth_scale"]);
    if model.len() != 1
        || size.len() != 2
        || scale.len() != 1
        || values.get("bf").is_some_and(|bf| bf.len() != 1)
    {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{}: expected one model, two sizes, one depth scale, one bf",
                path.display()
            ),
        );
    }
    let Some(model) = Model::ALL.iter().copied().find(|m| m.name() == model[0]) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: unknown camera model '{}'", path.display(), model[0]),
        );
    };
    let intrinsics = values["intrinsics"]
        .iter()
        .map(|v| number_of(path, "intrinsics", v))
        .collect::<Result<Vec<f64>>>()?;
    let (Some(width), Some(height)) = (text::integer(&size[0]), text::integer(&size[1])) else {
        return Err(not_a_number(path, "size", &size.join(" ")));
    };
    let depth_scale = number_of(path, "depth_scale", &scale[0])?;
    camera::check_intrinsics(model, &intrinsics)?;
    let within = |v: i64| {
        usize::try_from(v)
            .ok()
            .filter(|&s| (1..=MAX_SIDE).contains(&s))
    };
    let (Some(width), Some(height)) = (within(width), within(height)) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: size must lie in [1, {MAX_SIDE}]", path.display()),
        );
    };
    if !(depth_scale.is_finite() && depth_scale > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{}: depth_scale must be finite and positive",
                path.display()
            ),
        );
    }
    let bf = values
        .get("bf")
        .map(|bf| number_of(path, "bf", &bf[0]))
        .transpose()?;
    if bf.is_some_and(|v| !(v.is_finite() && v > 0.0)) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: bf must be finite and positive", path.display()),
        );
    }
    Ok(Camera {
        model,
        intrinsics,
        width,
        height,
        depth_scale,
        bf,
    })
}

/// Reads a folder's camera: its `camera.txt`, or the sensor its name says
/// (`slamlab.eval.tum.read_camera`).
///
/// # Arguments
///
/// * `root` - The folder.
///
/// # Returns
///
/// The camera.
///
/// # Errors
///
/// `InvalidInput` for a malformed `camera.txt`, or none and no sensor in the name.
pub fn read_camera(root: &Path) -> Result<Camera> {
    if root.join(CAMERA_FILE).exists() {
        return camera_file(&root.join(CAMERA_FILE));
    }
    let name = root
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    match SENSORS.iter().find(|(sensor, _)| name.contains(sensor)) {
        Some((_, values)) => Ok(Camera {
            model: Model::RadTan,
            intrinsics: values.to_vec(),
            width: SIZE.0,
            height: SIZE.1,
            depth_scale: DEPTH_SCALE,
            bf: Some(SENSOR_BF),
        }),
        None => fail(
            ErrorKind::InvalidInput,
            format!(
                "{name}: expected a {CAMERA_FILE}, or 'freiburg1', '2' or '3' in the folder's name"
            ),
        ),
    }
}

/// Reads a folder's colour frames, each with its nearest depth map
/// (`slamlab.eval.tum.read_frames`).
///
/// # Arguments
///
/// * `root` - The folder.
/// * `max_difference` - The largest frame-to-depth-map time difference, in seconds.
///
/// # Returns
///
/// The frames.
///
/// # Errors
///
/// `InvalidInput` for a missing `rgb.txt`, a malformed file (named with its line), a negative
/// `max_difference`, or an unreadable camera.
pub fn read_frames(root: &Path, max_difference: f64) -> Result<Frames> {
    if !(max_difference.is_finite() && max_difference >= 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            "max_difference: must be non-negative",
        );
    }
    let camera = read_camera(root)?;
    let colour = stamped(&root.join("rgb.txt"))?;
    let mut depths = vec![None; colour.stamps.len()];
    if root.join("depth.txt").exists() {
        let depth = stamped(&root.join("depth.txt"))?;
        if !depth.stamps.is_empty() {
            for (slot, &t) in depths.iter_mut().zip(&colour.stamps) {
                let d = nearest(&depth.stamps, t);
                if (depth.stamps[d] - t).abs() <= max_difference {
                    *slot = Some(root.join(&depth.names[d]));
                }
            }
        }
    }
    Ok(Frames {
        root: root.to_path_buf(),
        images: colour.names.iter().map(|n| root.join(n)).collect(),
        timestamps: colour.stamps,
        depths,
        camera,
    })
}

/// Reads a depth map in metres (`slamlab.eval.tum.read_depth`).
///
/// # Arguments
///
/// * `path` - A 16-bit depth PNG.
/// * `depth_scale` - Its value per metre.
///
/// # Returns
///
/// The depths, NaN for a stored 0.
///
/// # Errors
///
/// `InvalidInput` for an unreadable file, or a `depth_scale` that is not finite and positive.
pub fn read_depth(path: &Path, depth_scale: f64) -> Result<Depth> {
    if !(depth_scale.is_finite() && depth_scale > 0.0) {
        return fail(
            ErrorKind::InvalidInput,
            "depth_scale: must be finite and positive",
        );
    }
    let raw = png::read_depth(path)?;
    Ok(Depth {
        rows: raw.rows,
        cols: raw.cols,
        values: raw
            .values
            .iter()
            .map(|&v| {
                if v == 0 {
                    f64::NAN
                } else {
                    f64::from(v) / depth_scale
                }
            })
            .collect(),
    })
}

#[cfg(test)]
mod tests {
    //! Native tests against the shared fixtures `tests/fixtures/eval/sequences/` and the Python
    //! readers' frames, `sequences.json`, which the C++ tests read too.

    use super::*;
    use serde_json::Value;

    /// The fixtures' folder.
    fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/eval")
    }

    /// A path relative to a folder, as the fixtures list them.
    fn relative(path: &Path, root: &Path) -> String {
        path.strip_prefix(root)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/")
    }

    /// Numbers from a JSON list.
    fn numbers(value: &Value) -> Vec<f64> {
        value
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect()
    }

    #[test]
    fn folders_read_as_pythons() {
        let text = std::fs::read_to_string(fixtures().join("sequences.json")).unwrap();
        let cases: Value = serde_json::from_str(&text).unwrap();
        for (name, expected) in cases["cases"]["tum"].as_object().unwrap() {
            let frames =
                read_frames(&fixtures().join("sequences").join(name), MAX_DIFFERENCE).unwrap();
            assert_eq!(
                frames.timestamps,
                numbers(&expected["timestamps"]),
                "{name}"
            );
            let images: Vec<String> = frames
                .images
                .iter()
                .map(|p| relative(p, &frames.root))
                .collect();
            assert_eq!(
                images,
                expected["images"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_str().unwrap().to_owned())
                    .collect::<Vec<_>>()
            );
            let depths: Vec<Option<String>> = frames
                .depths
                .iter()
                .map(|d| d.as_ref().map(|p| relative(p, &frames.root)))
                .collect();
            let wanted: Vec<Option<String>> = expected["depths"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_str().map(str::to_owned))
                .collect();
            assert_eq!(depths, wanted, "{name}");
            let camera = &expected["camera"];
            assert_eq!(
                frames.camera.model.name(),
                camera["model"].as_str().unwrap()
            );
            assert_eq!(frames.camera.intrinsics, numbers(&camera["intrinsics"]));
            assert_eq!(
                (frames.camera.width, frames.camera.height),
                (
                    usize::try_from(camera["width"].as_u64().unwrap()).unwrap(),
                    usize::try_from(camera["height"].as_u64().unwrap()).unwrap()
                )
            );
            assert!(
                (frames.camera.depth_scale - camera["depth_scale"].as_f64().unwrap()).abs()
                    < f64::EPSILON
            );
            assert_eq!(frames.camera.bf, camera["bf"].as_f64());
        }
    }

    #[test]
    fn malformed_folders_are_refused() {
        // Named for this process: concurrent test runs never share the folder.
        let scratch =
            std::env::temp_dir().join(format!("slamlab_rust_test_tum_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&scratch);
        let folder = scratch.join("recording");
        std::fs::create_dir_all(&folder).unwrap();
        std::fs::write(folder.join("rgb.txt"), "1.0 a.png\nx b.png\n").unwrap();
        assert_eq!(
            read_frames(&folder, MAX_DIFFERENCE).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        std::fs::write(
            folder.join("camera.txt"),
            "model pinhole\nintrinsics 1 1 0 0\nsize 4 4\ndepth_scale 1\nbaseline 1\n",
        )
        .unwrap();
        assert_eq!(
            read_camera(&folder).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        std::fs::write(
            folder.join("camera.txt"),
            "model pinhole\nintrinsics 1 1 0 0\nsize 4 4\ndepth_scale 1\n",
        )
        .unwrap();
        let stamp = read_frames(&folder, MAX_DIFFERENCE).unwrap_err();
        assert!(stamp.message.contains("got 'x'"), "{}", stamp.message);
        std::fs::write(
            folder.join("camera.txt"),
            "model pinhole\nintrinsics 1 y 0 0\nsize 4 4\ndepth_scale 1\n",
        )
        .unwrap();
        let camera = read_camera(&folder).unwrap_err();
        assert!(
            camera.message.contains("intrinsics: 'y'"),
            "{}",
            camera.message
        );
        std::fs::remove_dir_all(&scratch).unwrap();
    }
}
