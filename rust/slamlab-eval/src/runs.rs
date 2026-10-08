//! The run format's writer: the Rust mirror of `slamlab.eval.runs.write_run` (the module's
//! docstring states the folder's files). JSON and `.npy` are written by hand: the formats
//! written are small and fixed, and nothing native reads them back.

use std::fmt::Write as _;
use std::path::Path;

use nalgebra::{Matrix4, Vector3};
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::{se3, so3};

/// `run.json`'s `format`.
pub const FORMAT: &str = "slamlab-run";

/// `run.json`'s `version` (2 added `peak_memory`).
pub const VERSION: i64 = 2;

/// The modes a run may have.
pub const MODES: [&str; 3] = ["monocular", "stereo", "rgbd"];

/// The backends a run may come from.
pub const BACKENDS: [&str; 3] = ["python", "cpp", "rust"];

/// numpy leaves room for the first dimension to grow to this many digits.
const GROWTH_DIGITS: usize = 21;

/// numpy aligns the data to this many bytes.
const ALIGN: usize = 64;

/// A JSON scalar: an integer, a finite number, a boolean or a string.
#[derive(Debug, Clone, PartialEq)]
pub enum JsonValue {
    /// An integer.
    Integer(i64),
    /// A finite number.
    Number(f64),
    /// A boolean.
    Bool(bool),
    /// A string.
    Text(String),
}

/// A flat JSON object, its keys in order.
pub type JsonObject = Vec<(String, JsonValue)>;

/// A pipeline's run (`slamlab.eval.runs.Run`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Run {
    /// The dataset's name.
    pub dataset: String,
    /// The sequence's name.
    pub sequence: String,
    /// `monocular`, `stereo` or `rgbd`.
    pub mode: String,
    /// `python`, `cpp` or `rust`.
    pub backend: String,
    /// Each tracked frame's timestamp, ascending.
    pub timestamps: Vec<f64>,
    /// Each tracked frame's `T_world_cam`.
    pub poses: Vec<Matrix4<f64>>,
    /// The keyframes' timestamps.
    pub keyframe_timestamps: Vec<f64>,
    /// The keyframes' `T_world_cam`.
    pub keyframe_poses: Vec<Matrix4<f64>>,
    /// The map's points, in the world frame.
    pub points: Vec<Vector3<f64>>,
    /// The pipeline's options.
    pub settings: JsonObject,
    /// One object per frame, each with a `timestamp`.
    pub diagnostics: Vec<JsonObject>,
    /// The largest resident set of the program that ran it, in bytes, if known
    /// (`crate::budgets::peak_memory`); written as null otherwise.
    pub peak_memory: Option<u64>,
}

/// A finite number as JSON, shortest round-trip (`.0` kept on a whole number, as Python
/// writes it).
///
/// # Arguments
///
/// * `value` - The number, finite.
///
/// # Returns
///
/// Its text.
#[must_use]
pub fn json_number(value: f64) -> String {
    format!("{value:?}")
}

/// A string as JSON: quoted, escaped.
///
/// # Arguments
///
/// * `value` - The string.
///
/// # Returns
///
/// Its text.
fn json_string(value: &str) -> String {
    let mut out = String::from('"');
    for c in value.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            c if u32::from(c) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", u32::from(c));
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// A JSON value's text.
///
/// # Arguments
///
/// * `value` - The value.
///
/// # Returns
///
/// Its text.
fn json_value(value: &JsonValue) -> String {
    match value {
        JsonValue::Integer(v) => v.to_string(),
        JsonValue::Number(v) => json_number(*v),
        JsonValue::Bool(v) => v.to_string(),
        JsonValue::Text(v) => json_string(v),
    }
}

/// A flat object as JSON.
///
/// # Arguments
///
/// * `object` - The object.
///
/// # Returns
///
/// Its text.
fn json_object(object: &[(String, JsonValue)]) -> String {
    let items: Vec<String> = object
        .iter()
        .map(|(key, value)| format!("{}: {}", json_string(key), json_value(value)))
        .collect();
    format!("{{{}}}", items.join(", "))
}

/// Whether an object's numbers are all finite.
///
/// # Arguments
///
/// * `object` - The object.
///
/// # Returns
///
/// Whether they are.
fn finite(object: &[(String, JsonValue)]) -> bool {
    object
        .iter()
        .all(|(_, v)| !matches!(v, JsonValue::Number(n) if !n.is_finite()))
}

/// A `.npy` file's bytes (version 1.0, little-endian float64, C order), as numpy writes it.
///
/// # Arguments
///
/// * `shape` - The array's shape.
/// * `values` - Its values, row-major.
///
/// # Returns
///
/// The bytes.
#[must_use]
pub fn npy(shape: &[usize], values: &[f64]) -> Vec<u8> {
    let dims: Vec<String> = shape.iter().map(ToString::to_string).collect();
    let mut dims = dims.join(", ");
    if shape.len() == 1 {
        dims.push(',');
    }
    let mut header = format!("{{'descr': '<f8', 'fortran_order': False, 'shape': ({dims}), }}");
    if let Some(first) = shape.first() {
        header.push_str(&" ".repeat(GROWTH_DIGITS - first.to_string().len()));
    }
    let prefix = 10; // magic, version, header length
    header.push_str(&" ".repeat(ALIGN - (prefix + header.len() + 1) % ALIGN));
    header.push('\n');
    let mut out = b"\x93NUMPY\x01\x00".to_vec();
    out.extend(
        u16::try_from(header.len())
            .unwrap_or(u16::MAX)
            .to_le_bytes(),
    );
    out.extend(header.as_bytes());
    for value in values {
        out.extend(value.to_le_bytes());
    }
    out
}

/// Checks stamped poses: finite, strictly ascending times, one rigid transform each.
///
/// # Arguments
///
/// * `times` - The times.
/// * `poses` - The poses.
/// * `name` - Their name, for messages.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check_stamped(times: &[f64], poses: &[Matrix4<f64>], name: &str) -> Result<()> {
    if times.len() != poses.len() {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected (F,) timestamps and (F, 4, 4) poses"),
        );
    }
    if times.iter().any(|t| !t.is_finite()) || times.windows(2).any(|w| !(w[1] > w[0])) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name} timestamps: must be finite and ascend strictly"),
        );
    }
    poses.iter().try_for_each(se3::check_transform)
}

/// Checks a run before anything is written.
///
/// # Arguments
///
/// * `run` - The run.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check(run: &Run) -> Result<()> {
    if !MODES.contains(&run.mode.as_str()) {
        return fail(
            ErrorKind::InvalidInput,
            format!("mode: unknown '{}'", run.mode),
        );
    }
    if !BACKENDS.contains(&run.backend.as_str()) {
        return fail(
            ErrorKind::InvalidInput,
            format!("backend: unknown '{}'", run.backend),
        );
    }
    check_stamped(&run.timestamps, &run.poses, "trajectory")?;
    check_stamped(&run.keyframe_timestamps, &run.keyframe_poses, "keyframes")?;
    if !run.points.iter().all(|p| p.iter().all(|v| v.is_finite())) {
        return fail(ErrorKind::InvalidInput, "points: expected finite values");
    }
    if !finite(&run.settings) {
        return fail(
            ErrorKind::InvalidInput,
            "settings: not JSON (a number is not finite)",
        );
    }
    for (k, entry) in run.diagnostics.iter().enumerate() {
        if !entry.iter().any(|(key, _)| key == "timestamp") || !finite(entry) {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "diagnostics[{k}]: expected an object with a timestamp, its numbers finite"
                ),
            );
        }
    }
    Ok(())
}

/// Stamped poses as TUM-format lines, each number round-tripping.
///
/// # Arguments
///
/// * `times` - The times.
/// * `poses` - The poses.
///
/// # Returns
///
/// The text.
///
/// # Errors
///
/// The quaternion conversion's error.
fn trajectory_text(times: &[f64], poses: &[Matrix4<f64>]) -> Result<String> {
    let mut out = String::from("# timestamp tx ty tz qx qy qz qw\n");
    for (t, pose) in times.iter().zip(poses) {
        let q = so3::to_quaternion(&pose.fixed_view::<3, 3>(0, 0).into_owned())?;
        let values = [
            *t,
            pose[(0, 3)],
            pose[(1, 3)],
            pose[(2, 3)],
            q[0],
            q[1],
            q[2],
            q[3],
        ];
        let line: Vec<String> = values.iter().map(|v| json_number(*v)).collect();
        out.push_str(&line.join(" "));
        out.push('\n');
    }
    Ok(out)
}

/// Writes a file whole: to a temporary sibling, then renamed over it.
///
/// # Arguments
///
/// * `path` - The file.
/// * `data` - Its bytes.
///
/// # Errors
///
/// `InvalidInput` naming the file.
fn replace(path: &Path, data: &[u8]) -> Result<()> {
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let temporary = path.with_file_name(format!(".{name}.tmp"));
    std::fs::write(&temporary, data)
        .and_then(|()| std::fs::rename(&temporary, path))
        .or_else(|e| {
            fail(
                ErrorKind::InvalidInput,
                format!("{}: could not be written ({e})", path.display()),
            )
        })
}

/// Writes a run's folder, created if needed, each file replaced whole (written to a temporary
/// sibling, then renamed): `slamlab.eval.runs.write_run`.
///
/// # Arguments
///
/// * `folder` - The folder.
/// * `run` - The run.
///
/// # Errors
///
/// `InvalidInput` for an unknown mode or backend, times that are not finite and strictly
/// ascending, poses that are not rigid or not one per time, a point that is not finite, a
/// diagnostic without a `timestamp`, or a number JSON cannot hold (nothing is written then);
/// or a file that cannot be written.
pub fn write_run(folder: &Path, run: &Run) -> Result<()> {
    check(run)?;
    let trajectory = trajectory_text(&run.timestamps, &run.poses)?;
    let info: JsonObject = vec![
        ("format".into(), JsonValue::Text(FORMAT.into())),
        ("version".into(), JsonValue::Integer(VERSION)),
        ("dataset".into(), JsonValue::Text(run.dataset.clone())),
        ("sequence".into(), JsonValue::Text(run.sequence.clone())),
        ("mode".into(), JsonValue::Text(run.mode.clone())),
        ("backend".into(), JsonValue::Text(run.backend.clone())),
    ];
    let mut info_text = json_object(&info);
    info_text.pop();
    info_text.push_str(", \"settings\": ");
    info_text.push_str(&json_object(&run.settings));
    info_text.push_str(", \"peak_memory\": ");
    info_text.push_str(
        &run.peak_memory
            .map_or_else(|| "null".to_owned(), |b| b.to_string()),
    );
    info_text.push_str("}\n");
    let diagnostics: String = run
        .diagnostics
        .iter()
        .map(|entry| json_object(entry) + "\n")
        .collect();
    let poses: Vec<f64> = run
        .keyframe_poses
        .iter()
        .flat_map(|p| (0..4).flat_map(move |r| (0..4).map(move |c| p[(r, c)])))
        .collect();
    let points: Vec<f64> = run.points.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    std::fs::create_dir_all(folder).or_else(|e| {
        fail(
            ErrorKind::InvalidInput,
            format!("{}: {e}", folder.display()),
        )
    })?;
    let files: [(&str, Vec<u8>); 6] = [
        ("run.json", info_text.into_bytes()),
        ("trajectory.txt", trajectory.into_bytes()),
        (
            "keyframes.npy",
            npy(&[run.keyframe_timestamps.len()], &run.keyframe_timestamps),
        ),
        (
            "keyframe_poses.npy",
            npy(&[run.keyframe_poses.len(), 4, 4], &poses),
        ),
        ("points.npy", npy(&[run.points.len(), 3], &points)),
        ("diagnostics.jsonl", diagnostics.into_bytes()),
    ];
    for (name, data) in files {
        replace(&folder.join(name), &data)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Native tests against the golden folder `tests/fixtures/eval/run/`, which Python wrote
    //! from `run_values.json` (the C++ tests read them too): the `.npy` files byte for byte,
    //! the JSON and trajectory value for value.

    use super::*;
    use serde_json::Value;
    use std::path::PathBuf;

    /// A scratch folder, removed when the test ends, failed or not.
    struct Scratch(std::path::PathBuf);

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// The fixtures' folder.
    fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/eval")
    }

    /// Poses from JSON.
    fn poses(values: &Value) -> Vec<Matrix4<f64>> {
        values
            .as_array()
            .unwrap()
            .iter()
            .map(|pose| Matrix4::from_fn(|r, c| pose[r][c].as_f64().unwrap()))
            .collect()
    }

    /// Numbers from JSON.
    fn numbers(values: &Value) -> Vec<f64> {
        values
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect()
    }

    /// A JSON object's scalars, in order.
    fn flat(object: &Value) -> JsonObject {
        object
            .as_object()
            .unwrap()
            .iter()
            .map(|(key, value)| {
                let scalar = match value {
                    Value::Bool(b) => JsonValue::Bool(*b),
                    Value::Number(n) if n.is_i64() => JsonValue::Integer(n.as_i64().unwrap()),
                    Value::Number(n) => JsonValue::Number(n.as_f64().unwrap()),
                    other => JsonValue::Text(other.as_str().unwrap().to_owned()),
                };
                (key.clone(), scalar)
            })
            .collect()
    }

    /// The run Python wrote, as the Rust backend.
    fn golden_run() -> Run {
        let text = std::fs::read_to_string(fixtures().join("run_values.json")).unwrap();
        let values: Value = serde_json::from_str(&text).unwrap();
        Run {
            dataset: values["dataset"].as_str().unwrap().into(),
            sequence: values["sequence"].as_str().unwrap().into(),
            mode: values["mode"].as_str().unwrap().into(),
            backend: "rust".into(),
            timestamps: numbers(&values["timestamps"]),
            poses: poses(&values["poses"]),
            keyframe_timestamps: numbers(&values["keyframe_timestamps"]),
            keyframe_poses: poses(&values["keyframe_poses"]),
            points: values["points"]
                .as_array()
                .unwrap()
                .iter()
                .map(|p| {
                    Vector3::new(
                        p[0].as_f64().unwrap(),
                        p[1].as_f64().unwrap(),
                        p[2].as_f64().unwrap(),
                    )
                })
                .collect(),
            settings: flat(&values["settings"]),
            diagnostics: values["diagnostics"]
                .as_array()
                .unwrap()
                .iter()
                .map(flat)
                .collect(),
            peak_memory: values["peak_memory"].as_u64(),
        }
    }

    /// A trajectory file's numbers.
    fn trajectory(path: &Path) -> Vec<Vec<f64>> {
        std::fs::read_to_string(path)
            .unwrap()
            .lines()
            .filter(|line| !line.starts_with('#') && !line.is_empty())
            .map(|line| {
                line.split_whitespace()
                    .map(|v| v.parse().unwrap())
                    .collect()
            })
            .collect()
    }

    /// A JSON value with every number as f64, so `1` and `1.0` compare equal.
    fn numeric(value: Value) -> Value {
        match value {
            Value::Number(n) => serde_json::json!(n.as_f64().unwrap()),
            Value::Array(items) => Value::Array(items.into_iter().map(numeric).collect()),
            Value::Object(map) => {
                Value::Object(map.into_iter().map(|(k, v)| (k, numeric(v))).collect())
            }
            other => other,
        }
    }

    /// A JSON file's value, or a JSON-lines file's values.
    fn json(path: &Path, lines: bool) -> Vec<Value> {
        let text = std::fs::read_to_string(path).unwrap();
        if lines {
            text.lines()
                .map(|l| numeric(serde_json::from_str(l).unwrap()))
                .collect()
        } else {
            vec![numeric(serde_json::from_str(&text).unwrap())]
        }
    }

    #[test]
    fn the_golden_folder_is_reproduced() {
        let golden = fixtures().join("run");
        // Named for this process: concurrent test runs never share the folder.
        let out =
            std::env::temp_dir().join(format!("slamlab_rust_test_runs_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let _guard = Scratch(out.clone());
        write_run(&out, &golden_run()).unwrap();
        for name in ["keyframes.npy", "keyframe_poses.npy", "points.npy"] {
            assert_eq!(
                std::fs::read(out.join(name)).unwrap(),
                std::fs::read(golden.join(name)).unwrap(),
                "{name}"
            );
        }
        let mut written = json(&out.join("run.json"), false).remove(0);
        let mut expected = json(&golden.join("run.json"), false).remove(0);
        assert_eq!(written["backend"], "rust");
        written.as_object_mut().unwrap().remove("backend");
        expected.as_object_mut().unwrap().remove("backend");
        assert_eq!(written, expected);
        let (got, wanted) = (
            trajectory(&out.join("trajectory.txt")),
            trajectory(&golden.join("trajectory.txt")),
        );
        assert_eq!(got.len(), wanted.len());
        for (a, b) in got.iter().zip(&wanted) {
            assert_eq!(a[..4], b[..4]);
            for i in 4..8 {
                assert!((a[i] - b[i]).abs() < 1e-15);
            }
        }
        assert_eq!(
            json(&out.join("diagnostics.jsonl"), true),
            json(&golden.join("diagnostics.jsonl"), true)
        );
        assert!(!out.join(".run.json.tmp").exists());
        let unknown = Run {
            peak_memory: None,
            ..golden_run()
        };
        write_run(&out, &unknown).unwrap();
        assert!(json(&out.join("run.json"), false)[0]["peak_memory"].is_null());
    }

    #[test]
    fn a_malformed_run_writes_nothing() {
        // Named for this process: concurrent test runs never share the folder.
        let out = std::env::temp_dir().join(format!(
            "slamlab_rust_test_runs_refused_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&out);
        let _guard = Scratch(out.clone());
        let mut run = golden_run();
        run.mode = "lidar".into();
        assert_eq!(
            write_run(&out, &run).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let mut run = golden_run();
        run.timestamps.swap(0, 1);
        assert_eq!(
            write_run(&out, &run).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let mut run = golden_run();
        run.diagnostics[0].retain(|(key, _)| key != "timestamp");
        assert_eq!(
            write_run(&out, &run).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        assert!(!out.exists());
        assert_eq!(
            (json_number(1.0), json_number(0.1)),
            ("1.0".into(), "0.1".into())
        );
    }
}
