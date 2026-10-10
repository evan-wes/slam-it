//! The run format's writer: the Rust mirror of `slam_it_all.eval.runs.write_run` (the module's
//! docstring states the folder's files). JSON and `.npy` are written by hand: the formats
//! written are small and fixed, and nothing native reads them back.

use std::fmt::Write as _;
use std::path::Path;

use nalgebra::{Matrix4, Vector3};
use slam_it_all_foundations::error::{ErrorKind, Result, fail};
use slam_it_all_foundations::{se3, so3};

/// `run.json`'s `format`.
pub const FORMAT: &str = "slam-it-all-run";

/// `run.json`'s `version` (2 added `peak_memory`, `cpus` and `memory_cap`; 3 every point and
/// keyframe the run made, with their lives).
pub const VERSION: i64 = 3;

/// The number of removal codes a life may hold (`slam_it_all.eval.runs.REMOVALS`).
pub const REMOVALS: i64 = 3;

/// A life: the frame that made a point or keyframe, the frame that removed it or -1, and why
/// (its index in `slam_it_all.eval.runs.REMOVALS`) or -1.
pub type Life = [i64; 3];

/// The modes a run may have.
pub const MODES: [&str; 3] = ["monocular", "stereo", "rgbd"];

/// The backends a run may come from.
pub const BACKENDS: [&str; 3] = ["python", "cpp", "rust"];

/// numpy leaves room for the first dimension to grow to this many digits.
const GROWTH_DIGITS: usize = 21;

/// numpy aligns the data to this many bytes.
const ALIGN: usize = 64;

/// A JSON value: null, an integer, a finite number, a boolean, a string, an array or an
/// object.
#[derive(Debug, Clone, PartialEq)]
pub enum JsonValue {
    /// Null.
    Null,
    /// An integer.
    Integer(i64),
    /// A finite number.
    Number(f64),
    /// A boolean.
    Bool(bool),
    /// A string.
    Text(String),
    /// An array.
    Array(Vec<JsonValue>),
    /// An object, its keys in order.
    Object(JsonObject),
}

/// A JSON object, its keys in order.
pub type JsonObject = Vec<(String, JsonValue)>;

/// A pipeline's run (`slam_it_all.eval.runs.Run`).
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
    /// The timestamps of every keyframe the run made, by id.
    pub keyframe_timestamps: Vec<f64>,
    /// Their last `T_world_cam`.
    pub keyframe_poses: Vec<Matrix4<f64>>,
    /// Every point the run made, by id, where it last was, in the world frame.
    pub points: Vec<Vector3<f64>>,
    /// The pipeline's options.
    pub settings: JsonObject,
    /// One object per frame, each with a `timestamp`; the edge pipeline's also time its steps
    /// (`read_seconds`, `build_seconds`, `track_seconds`, `map_seconds`, as
    /// `slam_it_all.eval.runs.STEPS` defines them).
    pub diagnostics: Vec<JsonObject>,
    /// The largest resident set of the program that ran it, in bytes, if known
    /// (`crate::budgets::peak_memory`); written as null otherwise.
    pub peak_memory: Option<u64>,
    /// The CPUs the program that ran it could run on, ascending, if known
    /// (`crate::budgets::allowed_cpus`); written as null otherwise.
    pub cpus: Option<Vec<usize>>,
    /// Its memory cap, in bytes, if it had one (`crate::budgets::memory_cap`); written as null
    /// otherwise.
    pub memory_cap: Option<u64>,
    /// Each point's life, by id.
    pub point_lives: Vec<Life>,
    /// Each keyframe's life, by id.
    pub keyframe_lives: Vec<Life>,
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
        JsonValue::Null => "null".to_owned(),
        JsonValue::Integer(v) => v.to_string(),
        JsonValue::Number(v) => json_number(*v),
        JsonValue::Bool(v) => v.to_string(),
        JsonValue::Text(v) => json_string(v),
        JsonValue::Array(items) => {
            let items: Vec<String> = items.iter().map(json_value).collect();
            format!("[{}]", items.join(", "))
        }
        JsonValue::Object(object) => json_object(object),
    }
}

/// An object as JSON, as Python's `json.dumps` writes it (`", "` and `": "` between items).
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

/// Whether an object's numbers, nested ones included, are all finite.
///
/// # Arguments
///
/// * `object` - The object.
///
/// # Returns
///
/// Whether they are.
fn finite(object: &[(String, JsonValue)]) -> bool {
    object.iter().all(|(_, v)| finite_value(v))
}

/// Whether a value's numbers, nested ones included, are all finite.
///
/// # Arguments
///
/// * `value` - The value.
///
/// # Returns
///
/// Whether they are.
fn finite_value(value: &JsonValue) -> bool {
    match value {
        JsonValue::Number(n) => n.is_finite(),
        JsonValue::Array(items) => items.iter().all(finite_value),
        JsonValue::Object(object) => finite(object),
        JsonValue::Null | JsonValue::Integer(_) | JsonValue::Bool(_) | JsonValue::Text(_) => true,
    }
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
    let mut out = npy_header(shape, "<f8");
    for value in values {
        out.extend(value.to_le_bytes());
    }
    out
}

/// A `.npy` file's bytes (version 1.0, little-endian int64, C order), as numpy writes it.
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
pub fn npy_int64(shape: &[usize], values: &[i64]) -> Vec<u8> {
    let mut out = npy_header(shape, "<i8");
    for value in values {
        out.extend(value.to_le_bytes());
    }
    out
}

/// A `.npy` file's header, as numpy writes it.
///
/// # Arguments
///
/// * `shape` - The array's shape.
/// * `descr` - The dtype's descriptor (`<f8`, `<i8`).
///
/// # Returns
///
/// The magic, version, header length and header.
fn npy_header(shape: &[usize], descr: &str) -> Vec<u8> {
    let dims: Vec<String> = shape.iter().map(ToString::to_string).collect();
    let mut dims = dims.join(", ");
    if shape.len() == 1 {
        dims.push(',');
    }
    let mut header = format!("{{'descr': '{descr}', 'fortran_order': False, 'shape': ({dims}), }}");
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
    out
}

/// Checks lives: one per row; made in one of the run's frames, removed in that frame or a later
/// one or never, and why exactly when removed.
///
/// # Arguments
///
/// * `lives` - The lives.
/// * `rows` - The rows they are for.
/// * `frames` - The run's frames.
/// * `name` - Their name, for messages.
///
/// # Errors
///
/// `InvalidInput` otherwise.
fn check_lives(lives: &[Life], rows: usize, frames: usize, name: &str) -> Result<()> {
    if lives.len() != rows {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected one life per row"),
        );
    }
    let count = i64::try_from(frames).unwrap_or(i64::MAX);
    let ok = lives.iter().all(|&[made, removed, how]| {
        let alive = removed == -1;
        (0..count).contains(&made)
            && (alive || (made..count).contains(&removed))
            && if alive {
                how == -1
            } else {
                (0..REMOVALS).contains(&how)
            }
    });
    if !ok {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected lives of the run's frames, removal codes"),
        );
    }
    Ok(())
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
    let frames = run.diagnostics.len();
    check_lives(&run.point_lives, run.points.len(), frames, "point_lives")?;
    check_lives(
        &run.keyframe_lives,
        run.keyframe_timestamps.len(),
        frames,
        "keyframe_lives",
    )?;
    if !finite(&run.settings) {
        return fail(
            ErrorKind::InvalidInput,
            "settings: not JSON (a number is not finite)",
        );
    }
    if let Some(cpus) = &run.cpus
        && (cpus.is_empty() || cpus.windows(2).any(|pair| pair[0] >= pair[1]))
    {
        return fail(
            ErrorKind::InvalidInput,
            "cpus: expected CPU numbers, ascending",
        );
    }
    if run.memory_cap == Some(0) {
        return fail(
            ErrorKind::InvalidInput,
            "memory_cap: expected a positive number of bytes",
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
/// sibling, then renamed): `slam_it_all.eval.runs.write_run`.
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
/// diagnostic without a `timestamp`, a number JSON cannot hold, CPUs that are empty or not
/// ascending, or a memory cap of 0 (nothing is written then); or a file that cannot be written.
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
    let integer_or_null =
        |value: Option<u64>| value.map_or_else(|| "null".to_owned(), |v| v.to_string());
    info_text.push_str(", \"peak_memory\": ");
    info_text.push_str(&integer_or_null(run.peak_memory));
    info_text.push_str(", \"cpus\": ");
    info_text.push_str(&run.cpus.as_ref().map_or_else(
        || "null".to_owned(),
        |cpus| {
            let listed: Vec<String> = cpus.iter().map(ToString::to_string).collect();
            format!("[{}]", listed.join(", "))
        },
    ));
    info_text.push_str(", \"memory_cap\": ");
    info_text.push_str(&integer_or_null(run.memory_cap));
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
    let lives = |lives: &[Life]| -> Vec<i64> { lives.iter().flatten().copied().collect() };
    let files: [(&str, Vec<u8>); 8] = [
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
        (
            "keyframe_lives.npy",
            npy_int64(&[run.keyframe_lives.len(), 3], &lives(&run.keyframe_lives)),
        ),
        ("points.npy", npy(&[run.points.len(), 3], &points)),
        (
            "point_lives.npy",
            npy_int64(&[run.point_lives.len(), 3], &lives(&run.point_lives)),
        ),
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

    /// A JSON object's members, as the writer takes them.
    fn flat(object: &Value) -> JsonObject {
        object
            .as_object()
            .unwrap()
            .iter()
            .map(|(key, value)| (key.clone(), value_of(value)))
            .collect()
    }

    /// A JSON value as the writer takes it, nested values included.
    fn value_of(value: &Value) -> JsonValue {
        match value {
            Value::Null => JsonValue::Null,
            Value::Bool(b) => JsonValue::Bool(*b),
            Value::Number(n) if n.is_i64() => JsonValue::Integer(n.as_i64().unwrap()),
            Value::Number(n) => JsonValue::Number(n.as_f64().unwrap()),
            Value::String(s) => JsonValue::Text(s.clone()),
            Value::Array(items) => JsonValue::Array(items.iter().map(value_of).collect()),
            Value::Object(_) => JsonValue::Object(flat(value)),
        }
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
            cpus: values["cpus"].as_array().map(|cpus| {
                cpus.iter()
                    .map(|cpu| usize::try_from(cpu.as_u64().unwrap()).unwrap())
                    .collect()
            }),
            memory_cap: values["memory_cap"].as_u64(),
            point_lives: lives(&values["point_lives"]),
            keyframe_lives: lives(&values["keyframe_lives"]),
        }
    }

    /// Lives from JSON rows of three integers.
    fn lives(values: &Value) -> Vec<Life> {
        values
            .as_array()
            .unwrap()
            .iter()
            .map(|life| {
                let at = |k: usize| life[k].as_i64().unwrap();
                [at(0), at(1), at(2)]
            })
            .collect()
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
            std::env::temp_dir().join(format!("slam_it_all_rust_test_runs_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let _guard = Scratch(out.clone());
        write_run(&out, &golden_run()).unwrap();
        for name in [
            "keyframes.npy",
            "keyframe_poses.npy",
            "keyframe_lives.npy",
            "points.npy",
            "point_lives.npy",
        ] {
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
            cpus: None,
            memory_cap: None,
            ..golden_run()
        };
        write_run(&out, &unknown).unwrap();
        let info = &json(&out.join("run.json"), false)[0];
        for key in ["peak_memory", "cpus", "memory_cap"] {
            assert!(info[key].is_null(), "{key}");
        }
    }

    #[test]
    fn a_malformed_run_writes_nothing() {
        // Named for this process: concurrent test runs never share the folder.
        let out = std::env::temp_dir().join(format!(
            "slam_it_all_rust_test_runs_refused_{}",
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
        for cpus in [vec![], vec![5, 4], vec![4, 4]] {
            let run = Run {
                cpus: Some(cpus),
                ..golden_run()
            };
            assert_eq!(
                write_run(&out, &run).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
        let run = Run {
            memory_cap: Some(0),
            ..golden_run()
        };
        assert_eq!(
            write_run(&out, &run).unwrap_err().kind,
            ErrorKind::InvalidInput
        );
        let mut short = golden_run();
        short.point_lives.pop();
        let mut early = golden_run();
        early.point_lives[1] = [3, 0, 1];
        let mut alive = golden_run();
        alive.keyframe_lives[0] = [0, -1, 2];
        for run in [short, early, alive] {
            assert_eq!(
                write_run(&out, &run).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
        assert!(!out.exists());
        assert_eq!(
            (json_number(1.0), json_number(0.1)),
            ("1.0".into(), "0.1".into())
        );
    }
}
